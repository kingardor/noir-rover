#!/usr/bin/env python3
"""
VLM head-to-head benchmark — mlx_vlm.server vs SwiftLM.

Sends identical requests to two OpenAI-compatible servers and compares
median end-to-end latency on the exact KG-builder workload.

Usage:
    # Baseline
    python scripts/bench_vlm.py --server-url http://localhost:8000 \
        --model mlx-community/Qwen3-VL-2B-Instruct-4bit --trials 20

    # SwiftLM candidate
    python scripts/bench_vlm.py --server-url http://localhost:8001 \
        --model mlx-community/Qwen3-VL-2B-Instruct-4bit --trials 20
"""

import argparse
import base64
import io
import json
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import requests
from PIL import Image

# ── KG-builder prompt (exact template from vision/kg_builder.py) ───────────────
_KG_PROMPT = """\
You are a robot's perception system. Study this image and report what you observe.

Things already in my knowledge graph: [nothing yet]

Return ONLY valid JSON — no markdown, no prose, no code fences:

{
  "caption": "One clear sentence describing the overall scene",
  "new_objects": [{"label": "...", "attrs": {"color": "...", "size": "..."}}],
  "new_events":  [{"description": "...", "involves": ["label1", "label2"]}],
  "relationships": [{"subject": "label1", "predicate": "on", "object": "label2"}],
  "changes":     [{"label": "...", "change": "moved|appeared|disappeared"}]
}

Rules:
- new_objects: every physically distinct, nameable thing NOT already in my graph. Use short noun labels.
- new_events: transient actions or situations.
- relationships: spatial or functional facts between entities.
- changes: things already in my graph that have visibly moved or changed.
- Use empty arrays only when truly nothing applies. Always include caption. /no_think"""


def _extract_frames(video_path: str, n: int) -> list[bytes]:
    """Extract n evenly-spaced frames from an MP4 as JPEG bytes."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    indices = [int(i * total / n) for i in range(n)]
    frames = []
    for idx in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if not ret:
            continue
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            frames.append(buf.tobytes())
    cap.release()
    return frames


def _load_jpeg_dir(path: str) -> list[bytes]:
    frames = []
    for f in sorted(Path(path).glob("*.jpg")):
        frames.append(f.read_bytes())
    return frames


def _resize_b64(jpeg_bytes: bytes, size: int) -> str:
    img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def run_benchmark(
    server_url: str,
    model: str,
    frames: list[bytes],
    prompt: str,
    max_tokens: int,
    image_size: int,
    trials: int,
    warmup: int,
) -> dict:
    url = server_url.rstrip("/") + "/v1/chat/completions"

    # Pre-encode all frames at target size
    encoded = [_resize_b64(f, image_size) for f in frames]
    n_frames = len(encoded)

    results = []
    sample_outputs = []

    print(f"\n[bench] server={server_url}  model={model.split('/')[-1]}")
    print(f"[bench] {trials} trials ({warmup} warmup), {n_frames} frames cycling, {image_size}px, max_tokens={max_tokens}")
    print(f"[bench] endpoint: {url}")

    # Verify server reachable
    try:
        r = requests.get(server_url.rstrip("/") + "/v1/models", timeout=10)
        r.raise_for_status()
        ids = [m["id"] for m in r.json().get("data", [])]
        if not any(model in mid or mid in model for mid in ids):
            print(f"[bench] WARNING: model '{model}' not found in server model list: {ids}")
    except Exception as e:
        print(f"[bench] ERROR: cannot reach server — {e}")
        sys.exit(1)

    total = trials + warmup
    for i in range(total):
        b64 = encoded[i % n_frames]
        payload = {
            "model": model,
            "messages": [{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                {"type": "text", "text": prompt},
            ]}],
            "temperature": 0.0,
            "max_tokens": max_tokens,
            "stream": False,
        }
        t0 = time.perf_counter()
        try:
            resp = requests.post(url, json=payload, timeout=60)
            resp.raise_for_status()
            data = resp.json()
        except Exception as e:
            print(f"  trial {i+1:3d}: ERROR {e}")
            continue
        latency_ms = (time.perf_counter() - t0) * 1000

        usage = data.get("usage", {})
        ptok = usage.get("prompt_tokens", -1)
        ctok = usage.get("completion_tokens", -1)
        content = (data.get("choices") or [{}])[0].get("message", {}).get("content", "")
        caption = content[:120].replace("\n", " ")

        label = "WARM" if i < warmup else f"  {i - warmup + 1:3d}"
        print(f"  {label}  {latency_ms:7.1f} ms  p={ptok} c={ctok}  {caption[:80]}")

        if i >= warmup:
            results.append({
                "trial": i - warmup + 1,
                "latency_ms": round(latency_ms, 1),
                "prompt_tokens": ptok,
                "completion_tokens": ctok,
            })
            if len(sample_outputs) < 2:
                sample_outputs.append(content[:300])

    if not results:
        print("[bench] No successful trials.")
        return {}

    lats = [r["latency_ms"] for r in results]
    agg = {
        "server_url": server_url,
        "model": model,
        "image_size": image_size,
        "max_tokens": max_tokens,
        "trials": len(results),
        "median_ms": round(statistics.median(lats), 1),
        "mean_ms": round(statistics.mean(lats), 1),
        "stdev_ms": round(statistics.stdev(lats) if len(lats) > 1 else 0, 1),
        "p95_ms": round(sorted(lats)[int(len(lats) * 0.95)], 1),
        "min_ms": round(min(lats), 1),
        "max_ms": round(max(lats), 1),
        "prompt_tokens_median": statistics.median(r["prompt_tokens"] for r in results if r["prompt_tokens"] > 0),
        "completion_tokens_median": statistics.median(r["completion_tokens"] for r in results if r["completion_tokens"] > 0),
        "sample_outputs": sample_outputs,
        "raw": results,
        "ts": datetime.utcnow().isoformat() + "Z",
    }

    print(f"\n[bench] ── Results ─────────────────────────────────")
    print(f"  median  {agg['median_ms']:7.1f} ms")
    print(f"  mean    {agg['mean_ms']:7.1f} ms  ± {agg['stdev_ms']:.1f}")
    print(f"  p95     {agg['p95_ms']:7.1f} ms")
    print(f"  min/max {agg['min_ms']:.1f} / {agg['max_ms']:.1f} ms")
    print(f"  tokens  prompt={agg['prompt_tokens_median']}  completion={agg['completion_tokens_median']}")

    return agg


def main():
    root = Path(__file__).parent.parent
    default_video = str(root / "recordings" / "test_feed.mp4")
    default_reports = str(root / "reports")

    ap = argparse.ArgumentParser(description="VLM latency benchmark")
    ap.add_argument("--server-url",  default="http://localhost:8000")
    ap.add_argument("--model",       default="mlx-community/Qwen3-VL-2B-Instruct-4bit")
    ap.add_argument("--frames",      default=default_video,
                    help="Path to MP4 file or directory of JPEGs")
    ap.add_argument("--trials",      type=int, default=20)
    ap.add_argument("--warmup",      type=int, default=3)
    ap.add_argument("--max-tokens",  type=int, default=300)
    ap.add_argument("--image-size",  type=int, default=384)
    ap.add_argument("--prompt",      default=_KG_PROMPT)
    ap.add_argument("--out-dir",     default=default_reports)
    args = ap.parse_args()

    # Load frames
    p = Path(args.frames)
    if p.is_dir():
        frames = _load_jpeg_dir(str(p))
    elif p.suffix.lower() in (".mp4", ".mov", ".avi"):
        frames = _extract_frames(str(p), n=10)
    else:
        frames = [p.read_bytes()]

    if not frames:
        print(f"[bench] No frames found at: {args.frames}")
        sys.exit(1)
    print(f"[bench] Loaded {len(frames)} source frame(s) from {args.frames}")

    result = run_benchmark(
        server_url=args.server_url,
        model=args.model,
        frames=frames,
        prompt=args.prompt,
        max_tokens=args.max_tokens,
        image_size=args.image_size,
        trials=args.trials,
        warmup=args.warmup,
    )

    if result:
        os.makedirs(args.out_dir, exist_ok=True)
        host = args.server_url.replace("http://", "").replace(":", "_").replace("/", "")
        ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(args.out_dir, f"vlm_bench_{host}_{ts}.json")
        with open(out_path, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\n[bench] Results written to {out_path}")


if __name__ == "__main__":
    main()
