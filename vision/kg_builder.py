#!/usr/bin/env python3
"""
Knowledge-graph builder — native macOS, noir_env.

Subscribes to the vision:events Redis pubsub (published by vision/app.py).
On each event, checks whether YOLOE detected any labels not yet in the KG.
If so (or after a 30-second cooldown), calls Qwen3-VL with a structured-JSON
prompt to extract NEW or CHANGED objects/events in the scene.

Parsed output is written to the Kuzu KG and images are saved to data/kg/images/.
Named faces from face:latest are auto-linked to Person nodes.

Publishes kg:updated to Redis after each successful graph update.

Usage:
    REDIS_URL=redis://localhost:6380 BRIDGE_URL=http://localhost:8012 \\
        python -u vision/kg_builder.py
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

# ── Config ────────────────────────────────────────────────────────────────────

REDIS_URL   = os.getenv("REDIS_URL",   "redis://localhost:6380")
MLX_VLM_URL = os.getenv("MLX_VLM_URL", "http://localhost:8000")
VLM_MODEL   = os.getenv("VLM_MODEL",   "mlx-community/Qwen3-VL-2B-Instruct-4bit")
VLM_SIZE    = int(os.getenv("VLM_SIZE", "384"))

KG_COOLDOWN  = float(os.getenv("KG_COOLDOWN",  "30.0"))  # min seconds between VLM calls
KG_MAX_IMGS  = int(os.getenv("KG_MAX_IMGS",    "5"))     # max images kept per node (not yet enforced)
MIN_CONF     = 0.45

# ── VLM prompt ────────────────────────────────────────────────────────────────

_PROMPT_TMPL = """\
You are observing a scene through a robot's camera.
Things I already know about: [{known}]

Looking at this image: what is NEW or CHANGED compared to what I already know?
Respond with ONLY valid JSON — no markdown, no prose, no code fences.

{{
  "new_objects": [{{"label": "...", "attrs": {{"color": "..."}}}}, ...],
  "new_events":  [{{"description": "...", "involves": ["label1", ...]}}],
  "changes":     [{{"label": "...", "change": "moved|appeared|disappeared"}}]
}}

If nothing is new or changed, return: {{"new_objects":[],"new_events":[],"changes":[]}} /no_think"""


def _build_prompt(known_labels: set[str]) -> str:
    known = ", ".join(sorted(known_labels)[:40]) if known_labels else "nothing yet"
    return _PROMPT_TMPL.format(known=known)


# ── Image helpers ─────────────────────────────────────────────────────────────

def _resize_b64(b64_jpeg: str, size: int) -> str:
    img = Image.open(io.BytesIO(base64.b64decode(b64_jpeg))).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _call_vlm(b64_jpeg: str, prompt: str) -> dict | None:
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
                "max_tokens": 400,
                "stream": False,
            },
            timeout=30,
        )
        resp.raise_for_status()
        text = (resp.json().get("choices") or [{}])[0].get("message", {}).get("content", "").strip()
        if not text:
            return None
        # Strip markdown code fences if present
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
        text = text.strip()
        return json.loads(text)
    except Exception as e:
        print(f"[kg] VLM call failed: {e}", flush=True)
        return None


# ── Snapshot ─────────────────────────────────────────────────────────────────

def _publish_snapshot(store, r: redis_lib.Redis, ts: float):
    """Serialize full KG to kg:snapshot for bridge-api.py (TTL 300s)."""
    try:
        snap = store.graph_snapshot()
        snap["ts"] = ts
        snap["counts"] = store.count_nodes()
        node_images: dict = {}
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


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    store = KGStore()

    print(f"[kg] Store ready. VLM={VLM_MODEL}  cooldown={KG_COOLDOWN}s", flush=True)

    pubsub = r.pubsub()
    pubsub.subscribe("vision:events")

    last_vlm_ts: float = 0.0

    for message in pubsub.listen():
        if message["type"] != "message":
            continue

        now = time.time()

        # Handle reset signal from bridge POST /kg/reset
        if r.get("kg:reset"):
            store.reset()
            shutil.rmtree(str(store._images), ignore_errors=True)
            store._images.mkdir(parents=True, exist_ok=True)
            r.delete("kg:reset")
            r.delete("kg:snapshot")
            r.set("kg:last_update", "0", ex=3600)
            print("[kg] reset complete", flush=True)
            continue

        # Read latest YOLOE detections
        raw_vis = r.get("vision:latest")
        if not raw_vis:
            continue
        try:
            vis = json.loads(raw_vis)
        except Exception:
            continue

        detected_labels = {
            d["label"].lower() for d in vis.get("detections", [])
            if d.get("conf", 0) >= MIN_CONF
        }

        known_labels = store.existing_labels()

        novel_labels = detected_labels - known_labels
        time_since   = now - last_vlm_ts

        # Trigger VLM only on novelty OR cooldown expiry
        should_trigger = bool(novel_labels) or time_since >= KG_COOLDOWN
        if not should_trigger:
            continue

        # Grab current camera frame
        b64 = r.get("camera:frame")
        if not b64:
            continue

        last_vlm_ts = now

        # ── Call VLM ──────────────────────────────────────────────────────────
        prompt   = _build_prompt(known_labels)
        t0       = time.monotonic()
        result   = _call_vlm(b64, prompt)
        elapsed  = time.monotonic() - t0

        if not result:
            continue

        new_objects = result.get("new_objects") or []
        new_events  = result.get("new_events")  or []
        changes     = result.get("changes")     or []

        if not new_objects and not new_events and not changes:
            print(f"[kg] {elapsed:.1f}s — nothing new.", flush=True)
            continue

        print(f"[kg] {elapsed:.1f}s — objects={len(new_objects)} events={len(new_events)} changes={len(changes)}", flush=True)

        # Save image once per tick — all new nodes this tick share this image
        img_id, img_path = store.save_image(b64)
        added = 0

        # ── Insert new objects ────────────────────────────────────────────────
        for obj in new_objects:
            label = (obj.get("label") or "").strip().lower()
            if not label or label in known_labels:
                continue
            attrs = obj.get("attrs") or {}
            store.add_object(label, attrs, img_id, img_path, now)
            # _add_image_node is idempotent — duplicate Image inserts are swallowed
            print(f"[kg]   + object: {label}", flush=True)
            added += 1
            known_labels.add(label)  # prevent duplicate within same tick

        # ── Insert new events ─────────────────────────────────────────────────
        for evt in new_events:
            desc = (evt.get("description") or "").strip().lower()
            if not desc:
                continue
            involves = [l.lower() for l in (evt.get("involves") or [])]
            store.add_event(desc, involves, img_id, img_path, now)
            print(f"[kg]   + event: {desc}", flush=True)
            added += 1

        # ── Handle changes (update last_seen on existing objects) ─────────────
        for change in changes:
            label = (change.get("label") or "").strip().lower()
            change_type = change.get("change", "")
            if label and change_type:
                print(f"[kg]   ~ change: {label} → {change_type}", flush=True)
                store.touch_label(label, now)

        # ── Auto-link named faces ─────────────────────────────────────────────
        changed = added > 0
        try:
            raw_face = r.get("face:latest")
            if raw_face:
                face_data = json.loads(raw_face)
                for face in face_data.get("faces") or []:
                    name = face.get("name", "").strip()
                    if name and name.lower() != "unknown":
                        store.add_person(name.lower(), "", "", now)
                        print(f"[kg]   + person: {name}", flush=True)
                        changed = True
        except Exception:
            pass

        # ── Notify dashboard + publish snapshot ───────────────────────────────
        if added:
            r.publish("kg:updated", json.dumps({"ts": now, "added": added}))
            r.set("kg:last_update", str(now), ex=3600)
        if changed:
            _publish_snapshot(store, r, now)


if __name__ == "__main__":
    main()
