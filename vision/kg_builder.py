#!/usr/bin/env python3
"""
Perception service — caption + knowledge graph.

Runs on a timer (KG_INTERVAL seconds, default 10). Each tick:
  1. Grabs the latest camera frame from Redis.
  2. Calls Qwen3-VL with a combined prompt → caption + structured KG updates.
  3. Writes caption to vlm:latest (TTL 30s) — schema: {text, ts, frame_id}.
  4. Writes new nodes/edges to Kuzu with deduplication:
       - Objects: upsert by label (kg_store handles it)
       - Events:  skip if same description added within DEDUP_WINDOW_S seconds
       - Changes: touch_label() to update last_seen
  5. Publishes kg:snapshot to Redis after any graph change (TTL 300s).
  6. Auto-links named faces from face:latest to Person nodes.

Reset: watches kg:reset Redis key, wipes graph when set.

Usage:
    REDIS_URL=redis://localhost:6380 python -u vision/kg_builder.py
"""
import base64
import io
import json
import os
import re
import shutil
import sys
import time
from pathlib import Path

import redis as redis_lib
import requests
from PIL import Image

sys.path.insert(0, str(Path(__file__).parent))
from kg_store import KGStore

# ── Config ─────────────────────────────────────────────────────────────────────

REDIS_URL   = os.getenv("REDIS_URL",   "redis://localhost:6380")
MLX_VLM_URL = os.getenv("MLX_VLM_URL", "http://localhost:8000")
VLM_MODEL   = os.getenv("VLM_MODEL",   "mlx-community/Qwen3-VL-2B-Instruct-4bit")
VLM_SIZE    = int(os.getenv("VLM_SIZE", "384"))
KG_INTERVAL = float(os.getenv("KG_INTERVAL", "10.0"))

DEDUP_WINDOW_S = 300  # skip event if same description was inserted within 5 min

# ── VLM prompt ─────────────────────────────────────────────────────────────────

_PROMPT_TMPL = """\
You are a robot's perception system. Analyze this image.

Things already in my knowledge graph: [{known}]

Return ONLY valid JSON — no markdown, no prose, no code fences:

{{
  "caption": "One clear sentence describing the current scene",
  "new_objects": [{{"label": "...", "attrs": {{"color": "..."}}}}],
  "new_events":  [{{"description": "...", "involves": ["label1", "label2"]}}],
  "changes":     [{{"label": "...", "change": "moved|appeared|disappeared"}}]
}}

new_objects: objects clearly visible that are NOT in my knowledge graph.
new_events: notable situations or activities (person entering, object being used, etc.).
changes: things I already know about that have visibly changed state.
Use empty arrays if nothing is new. Always include caption. /no_think"""


def _build_prompt(known_labels: set) -> str:
    known = ", ".join(sorted(known_labels)[:40]) if known_labels else "nothing yet"
    return _PROMPT_TMPL.format(known=known)


# ── Image helpers ──────────────────────────────────────────────────────────────

def _resize_b64(b64_jpeg: str, size: int) -> str:
    img = Image.open(io.BytesIO(base64.b64decode(b64_jpeg))).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _call_vlm(b64_jpeg: str, prompt: str):
    """Call Qwen3-VL and return parsed JSON dict, or None on failure."""
    try:
        b64 = _resize_b64(b64_jpeg, VLM_SIZE)
        resp = requests.post(
            f"{MLX_VLM_URL}/v1/chat/completions",
            headers={"Content-Type": "application/json"},
            json={
                "model": VLM_MODEL,
                "messages": [{"role": "user", "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                    {"type": "text", "text": prompt},
                ]}],
                "temperature": 0.0,
                "max_tokens": 500,
                "stream": False,
            },
            timeout=30,
        )
        resp.raise_for_status()
        text = (resp.json().get("choices") or [{}])[0].get("message", {}).get("content", "").strip()
        if not text:
            return None
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
        return json.loads(text.strip())
    except Exception as e:
        print(f"[kg] VLM call failed: {e}", flush=True)
        return None


# ── Snapshot ───────────────────────────────────────────────────────────────────

def _publish_snapshot(store: KGStore, r: redis_lib.Redis, ts: float):
    """Serialize full KG to kg:snapshot for bridge-api.py (TTL 300s)."""
    try:
        snap = store.graph_snapshot()
        snap["ts"] = ts
        snap["counts"] = store.count_nodes()
        node_images = {}
        for node in snap["nodes"]:
            imgs = store.get_node_images(node["id"])
            if imgs:
                node_images[node["id"]] = imgs
        snap["node_images"] = node_images
        snap["image_index"] = {
            img["id"]: img["path"]
            for imgs in node_images.values()
            for img in imgs
        }
        r.set("kg:snapshot", json.dumps(snap), ex=300)
    except Exception as e:
        print(f"[kg] snapshot publish failed: {e}", flush=True)


# ── Event dedup ────────────────────────────────────────────────────────────────

_recent_events: dict = {}  # description → ts of last insert


def _is_event_new(desc: str, now: float) -> bool:
    last = _recent_events.get(desc)
    return last is None or (now - last) >= DEDUP_WINDOW_S


def _record_event(desc: str, now: float):
    _recent_events[desc] = now
    cutoff = now - DEDUP_WINDOW_S
    for k in list(_recent_events):
        if _recent_events[k] < cutoff:
            del _recent_events[k]


# ── VLM item coercion (handle str or dict from VLM) ───────────────────────────

_ARTICLES = ("a ", "an ", "the ")
_MAX_LABEL_WORDS = 4  # reject sentence-length strings landing in object slots


def _normalize_label(raw: str) -> str:
    """Strip leading articles and reject multi-word sentences."""
    s = raw.strip().lower()
    for art in _ARTICLES:
        if s.startswith(art):
            s = s[len(art):]
            break
    if len(s.split()) > _MAX_LABEL_WORDS:
        return ""   # too long to be an object label — caller will skip
    return s


def _obj_label(item):
    if isinstance(item, str):
        return _normalize_label(item), {}
    raw = (item.get("label") or "").strip()
    return _normalize_label(raw), (item.get("attrs") or {})


def _evt_desc(item):
    if isinstance(item, str):
        return item.strip().lower(), []
    involves = [l.lower() for l in (item.get("involves") or [])]
    return (item.get("description") or "").strip().lower(), involves


def _chg_label(item):
    if isinstance(item, str):
        return _normalize_label(item), "changed"
    return _normalize_label((item.get("label") or "").strip()), (item.get("change") or "changed")


# ── Main loop ──────────────────────────────────────────────────────────────────

def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    store = KGStore()

    print(f"[kg] Store ready. VLM={VLM_MODEL}  interval={KG_INTERVAL}s", flush=True)

    last_cam_ts = None

    while True:
        t0 = time.time()

        # Handle reset signal from bridge POST /kg/reset
        if r.get("kg:reset"):
            store.reset()
            shutil.rmtree(str(store._images), ignore_errors=True)
            store._images.mkdir(parents=True, exist_ok=True)
            r.delete("kg:reset")
            r.delete("kg:snapshot")
            r.set("kg:last_update", "0", ex=3600)
            _recent_events.clear()
            print("[kg] reset complete", flush=True)
            time.sleep(KG_INTERVAL)
            continue

        # Wait up to KG_INTERVAL for a new camera frame
        deadline = t0 + KG_INTERVAL
        b64 = None
        cam_ts = None
        while time.time() < deadline:
            ts = r.get("camera:ts")
            if ts and ts != last_cam_ts:
                frame = r.get("camera:frame")
                if frame:
                    b64 = frame
                    cam_ts = ts
                    break
            time.sleep(0.1)

        if b64 is None:
            continue

        # Grab freshest frame before the VLM call
        ts2 = r.get("camera:ts")
        if ts2 and ts2 != cam_ts:
            frame2 = r.get("camera:frame")
            if frame2:
                b64 = frame2
                cam_ts = ts2

        last_cam_ts = cam_ts
        now = time.time()

        # ── Call VLM ──────────────────────────────────────────────────────────
        known_labels = store.existing_labels()
        prompt = _build_prompt(known_labels)
        t_vlm = time.monotonic()
        result = _call_vlm(b64, prompt)
        elapsed = time.monotonic() - t_vlm

        if not result:
            continue

        # ── Write caption → vlm:latest ─────────────────────────────────────────
        caption = (result.get("caption") or "").strip()
        if caption:
            r.set("vlm:latest", json.dumps({"text": caption, "ts": now, "frame_id": cam_ts}), ex=30)
            print(f"[kg] {elapsed:.1f}s  {caption[:90]}", flush=True)

        new_objects = result.get("new_objects") or []
        new_events  = result.get("new_events")  or []
        changes     = result.get("changes")     or []

        if not new_objects and not new_events and not changes:
            continue

        print(f"[kg]   objects={len(new_objects)} events={len(new_events)} changes={len(changes)}", flush=True)

        # Image is saved lazily — only when the first node is actually inserted.
        # All new nodes this tick share the same image file.
        _img_id: list = []
        _img_path: list = []

        def _get_image():
            if not _img_id:
                iid, ipath = store.save_image(b64)
                _img_id.append(iid)
                _img_path.append(ipath)
            return _img_id[0], _img_path[0]

        added = 0

        # ── Insert new objects ─────────────────────────────────────────────────
        for item in new_objects:
            label, attrs = _obj_label(item)
            if not label or label in known_labels:
                continue
            img_id, img_path = _get_image()
            store.add_object(label, attrs, img_id, img_path, now)
            print(f"[kg]   + object: {label}", flush=True)
            added += 1
            known_labels.add(label)

        # ── Insert new events ──────────────────────────────────────────────────
        for item in new_events:
            desc, involves = _evt_desc(item)
            if not desc or not _is_event_new(desc, now):
                continue
            img_id, img_path = _get_image()
            store.add_event(desc, involves, img_id, img_path, now)
            _record_event(desc, now)
            print(f"[kg]   + event: {desc}", flush=True)
            added += 1

        # ── Handle changes (update last_seen) ──────────────────────────────────
        for item in changes:
            label, change_type = _chg_label(item)
            if label:
                print(f"[kg]   ~ change: {label} → {change_type}", flush=True)
                store.touch_label(label, now)

        # ── Auto-link named faces ──────────────────────────────────────────────
        changed = added > 0
        try:
            raw_face = r.get("face:latest")
            if raw_face:
                face_data = json.loads(raw_face)
                for face in face_data.get("faces") or []:
                    name = face.get("name", "").strip()
                    if name and name.lower() != "unknown":
                        img_id, img_path = _get_image()
                        store.add_person(name.lower(), img_id, img_path, now)
                        print(f"[kg]   + person: {name}", flush=True)
                        changed = True
        except Exception:
            pass

        # ── Publish snapshot ───────────────────────────────────────────────────
        if added:
            r.publish("kg:updated", json.dumps({"ts": now, "added": added}))
            r.set("kg:last_update", str(now), ex=3600)
        if changed:
            _publish_snapshot(store, r, now)


if __name__ == "__main__":
    main()
