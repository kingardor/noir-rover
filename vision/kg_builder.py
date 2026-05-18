#!/usr/bin/env python3
"""
Perception service — caption + knowledge graph.

Continuously samples one camera frame every KG_SAMPLE_INTERVAL seconds into a
rolling buffer (deque, maxlen=KG_FRAMES). As soon as the buffer is full and a
new frame was just added, calls Qwen3-VL with all KG_FRAMES images at once:

  1. Multi-image stateless VLM call → caption + KG updates.
  2. Writes caption to vlm:latest (TTL 30s) — schema: {text, ts, frame_id}.
  3. Post-hoc fuzzy merge: incoming labels/descriptions are matched against
     existing DB nodes via normalized string similarity before insertion.
  4. All frames in the batch are attached as images to each new/matched node.
  5. Publishes kg:snapshot to Redis after any graph change (TTL 300s).
  6. Auto-links named faces from face:latest to Person nodes.

Reset: watches kg:reset Redis key, wipes graph when set.

Usage:
    REDIS_URL=redis://localhost:6380 python -u vision/kg_builder.py
"""
import base64
import collections
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

# Redirect KG storage to test paths before KGStore reads its env vars at import time.
if os.getenv("TEST_MODE") == "1":
    _root = Path(__file__).parent.parent
    os.environ.setdefault("KG_DB_DIR",     str(_root / "data" / "kg" / "graph_db_test"))
    os.environ.setdefault("KG_IMAGES_DIR", str(_root / "data" / "kg" / "images_test"))

from kg_store import KGStore

# ── Config ─────────────────────────────────────────────────────────────────────

REDIS_URL   = os.getenv("REDIS_URL",   "redis://localhost:6380")
MLX_VLM_URL = os.getenv("MLX_VLM_URL", "http://localhost:8000")
VLM_MODEL   = os.getenv("VLM_MODEL",   "mlx-community/Qwen3-VL-2B-Instruct-4bit")
VLM_SIZE               = int(os.getenv("VLM_SIZE",                 "256"))
KG_SAMPLE_INTERVAL     = float(os.getenv("KG_SAMPLE_INTERVAL",     "3.0"))
KG_FRAMES              = int(os.getenv("KG_FRAMES",                "3"))
KG_FUZZY_OBJ_THRESHOLD = float(os.getenv("KG_FUZZY_OBJ_THRESHOLD", "0.82"))
KG_FUZZY_EVT_THRESHOLD = float(os.getenv("KG_FUZZY_EVT_THRESHOLD", "0.78"))

# ── VLM prompt ─────────────────────────────────────────────────────────────────

_PROMPT = """\
You are a robot's perception system. Frames captured seconds apart from the \
same vantage are below. Treat them as one scene observed over a brief moment.

Return ONLY valid JSON — no markdown, no prose, no code fences:

{
  "caption": "One clear sentence describing the overall scene",
  "objects":  [{"label": "...", "attrs": {"color": "...", "size": "..."}}],
  "events":   [{"description": "...", "involves": ["label1", "label2"]}],
  "relationships": [{"subject": "label1", "predicate": "on", "object": "label2"}]
}

Rules:
- objects: every physically distinct, nameable thing across the frames. \
Short noun labels (≤4 words). Be thorough.
- events: transient actions or situations visible in any frame. Include \
the object labels involved.
- relationships: spatial facts. Subject and object must be object labels. \
Use snake_case predicates. Only emit when confident.
- Use empty arrays when nothing applies. Always include caption. /no_think"""


# ── Image helpers ──────────────────────────────────────────────────────────────

def _resize_b64(b64_jpeg: str, size: int) -> str:
    img = Image.open(io.BytesIO(base64.b64decode(b64_jpeg))).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _call_vlm(b64_list: list, prompt: str):
    """Call Qwen3-VL with one or more images and return parsed JSON dict, or None on failure."""
    try:
        content = []
        for b64 in b64_list:
            b64r = _resize_b64(b64, VLM_SIZE)
            content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64r}"}})
        content.append({"type": "text", "text": prompt})
        resp = requests.post(
            f"{MLX_VLM_URL}/v1/chat/completions",
            headers={"Content-Type": "application/json"},
            json={
                "model": VLM_MODEL,
                "messages": [{"role": "user", "content": content}],
                "temperature": 0.0,
                "max_tokens": 800,
                "stream": False,
            },
            timeout=45,
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


def _normalize_predicate(p: str) -> str:
    s = (p or "").strip().lower()
    s = re.sub(r"[^a-z0-9]+", "_", s).strip("_")
    return s if s and len(s.split("_")) <= 4 else ""


def _rel_triple(item):
    if isinstance(item, str):
        return "", "", ""
    subj = _normalize_label((item.get("subject") or "").strip())
    pred = _normalize_predicate(item.get("predicate") or "")
    obj  = _normalize_label((item.get("object") or "").strip())
    return subj, pred, obj


# ── Main loop ──────────────────────────────────────────────────────────────────

def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    store = KGStore()

    print(f"[kg] Store ready. VLM={VLM_MODEL}  sample={KG_SAMPLE_INTERVAL}s  frames={KG_FRAMES}", flush=True)

    # Rolling buffer of the latest KG_FRAMES camera frames.
    # A new frame is sampled every KG_SAMPLE_INTERVAL seconds; VLM fires when
    # the buffer is full and a fresh frame was just added.
    frame_buffer  = collections.deque(maxlen=KG_FRAMES)
    last_cam_ts   = None
    last_sample_t = 0.0

    while True:
        # Handle reset signal from bridge POST /kg/reset
        if r.get("kg:reset"):
            store.reset()
            shutil.rmtree(str(store._images), ignore_errors=True)
            store._images.mkdir(parents=True, exist_ok=True)
            r.delete("kg:reset")
            r.delete("kg:snapshot")
            r.set("kg:last_update", "0", ex=3600)
            frame_buffer.clear()
            last_cam_ts   = None
            last_sample_t = 0.0
            print("[kg] reset complete", flush=True)
            continue

        # Sample one frame every KG_SAMPLE_INTERVAL seconds.
        now = time.time()
        new_frame = False
        if now - last_sample_t >= KG_SAMPLE_INTERVAL:
            ts = r.get("camera:ts")
            if ts and ts != last_cam_ts:
                frame = r.get("camera:frame")
                if frame:
                    frame_buffer.append(frame)
                    last_cam_ts = ts
                    new_frame = True
            last_sample_t = now

        # Fire VLM only when a fresh frame was just added and the buffer is full.
        if not new_frame or len(frame_buffer) < KG_FRAMES:
            time.sleep(0.05)
            continue

        batch = list(frame_buffer)  # latest KG_FRAMES frames

        # ── Call VLM ──────────────────────────────────────────────────────────
        t_vlm = time.monotonic()
        result = _call_vlm(batch, _PROMPT)
        elapsed = time.monotonic() - t_vlm

        if not result:
            continue

        now = time.time()

        # ── Write caption → vlm:latest ─────────────────────────────────────────
        caption = (result.get("caption") or "").strip()
        if caption:
            r.set("vlm:latest", json.dumps({"text": caption, "ts": now, "frame_id": last_cam_ts}), ex=30)
            print(f"[kg] {elapsed:.1f}s  {caption[:90]}", flush=True)

        objects       = result.get("objects")       or []
        events        = result.get("events")        or []
        relationships = result.get("relationships") or []

        if not objects and not events and not relationships:
            continue

        print(f"[kg]   objects={len(objects)} events={len(events)} rels={len(relationships)}", flush=True)

        # Image is saved lazily — only when the first node is actually inserted.
        # frame 0 is the primary image; extra frames are attached after.
        _img_id: list = []
        _img_path: list = []

        def _get_image():
            if not _img_id:
                iid, ipath = store.save_image(batch[0])
                _img_id.append(iid)
                _img_path.append(ipath)
            return _img_id[0], _img_path[0]

        added = 0

        # ── Insert objects with post-hoc fuzzy merge ───────────────────────────
        for item in objects:
            label, attrs = _obj_label(item)
            if not label:
                continue
            canonical = store.find_similar_object(label, KG_FUZZY_OBJ_THRESHOLD) or label
            if canonical != label:
                print(f"[kg]   ~ merge object: '{label}' → '{canonical}'", flush=True)
            img_id, img_path = _get_image()
            store.add_object(canonical, attrs, img_id, img_path, now)
            for extra_b64 in batch[1:]:
                eid, epath = store.save_image(extra_b64)
                store.attach_image_to_label(canonical, eid, epath, now)
            print(f"[kg]   + object: {canonical}", flush=True)
            added += 1

        # ── Insert events with post-hoc fuzzy merge ────────────────────────────
        for item in events:
            desc, involves = _evt_desc(item)
            if not desc:
                continue
            canonical = store.find_similar_event(desc, KG_FUZZY_EVT_THRESHOLD) or desc
            if canonical != desc:
                print(f"[kg]   ~ merge event: '{desc[:50]}' → '{canonical[:50]}'", flush=True)
            img_id, img_path = _get_image()
            store.add_event(canonical, involves, img_id, img_path, now)
            for extra_b64 in batch[1:]:
                eid, epath = store.save_image(extra_b64)
                store.attach_image_to_event(canonical, eid, epath, now)
            print(f"[kg]   + event: {canonical[:70]}", flush=True)
            added += 1

        # ── Insert relationships ───────────────────────────────────────────────
        seen_triples: set = set()
        for item in relationships:
            subj, pred, obj = _rel_triple(item)
            if not subj or not pred or not obj or subj == obj:
                continue
            key = (subj, pred, obj)
            if key in seen_triples:
                continue
            seen_triples.add(key)
            # Resolve through fuzzy match so rels wire to canonical node labels
            subj_can = store.find_similar_object(subj, KG_FUZZY_OBJ_THRESHOLD) or subj
            obj_can  = store.find_similar_object(obj,  KG_FUZZY_OBJ_THRESHOLD) or obj
            if store.add_relationship(subj_can, pred, obj_can, now):
                print(f"[kg]   ↔ {subj_can} —{pred}→ {obj_can}", flush=True)
                added += 1

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
