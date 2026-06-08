#!/usr/bin/env python3
"""
Quick multi-trial latency benchmark for InternVL3-1B via mlx_vlm.generate directly.
Bypasses the mlx_vlm.server threading bug that affects InternVL.

Usage:
    python scripts/bench_internvl_direct.py [--trials 10] [--image path/to/image.jpg]
"""
import argparse
import io
import json
import re
import statistics
import sys
import time
from pathlib import Path

from PIL import Image

_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_ROOT / "vision"))

MODEL_PATH = "mlx-community/InternVL3-1B-4bit"
IMAGE_SIZE = 384

_PROMPT = """\
Analyze this image carefully. Return ONLY valid JSON with no extra text:

{
  "caption": "",
  "new_objects": [{"label": "object name", "attrs": {"color": "", "size": ""}}],
  "new_events": [],
  "relationships": [{"subject": "label1", "predicate": "on", "object": "label2"}],
  "changes": []
}

Fill in caption with one sentence describing the overall scene.
Fill in new_objects with every distinct physical object you can see. Use short noun labels (1-3 words).
Fill in relationships with spatial facts. Leave new_events and changes as empty arrays if nothing applies.
Reply with only the completed JSON."""


def resize_to_square(image_path: str, size: int) -> str:
    import tempfile, os
    img = Image.open(image_path).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    tmp = tempfile.mktemp(suffix=".jpg")
    img.save(tmp, format="JPEG", quality=85)
    return tmp


def extract_json(text: str):
    """Extract first complete JSON object from text, ignoring surrounding noise."""
    text = text.strip()
    text = re.sub(r'^```(?:json)?\s*', '', text)
    text = re.sub(r'\s*```.*$', '', text, flags=re.DOTALL)
    # Find the outermost { ... }
    start = text.find('{')
    if start == -1:
        return None, text
    depth = 0
    for i, ch in enumerate(text[start:], start):
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                candidate = text[start:i+1]
                try:
                    return json.loads(candidate), candidate
                except json.JSONDecodeError:
                    return None, candidate
    return None, text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials",  type=int, default=10)
    ap.add_argument("--warmup",  type=int, default=2)
    ap.add_argument("--image",   default=str(_ROOT / "data" / "kg" / "images_test" /
                                             "038151c1-5be3-4c2e-bb76-dc674d6976b0.jpg"))
    ap.add_argument("--max-tokens", type=int, default=512)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    print(f"[bench] Loading {MODEL_PATH}...")
    from mlx_vlm import load, generate
    from mlx_vlm.prompt_utils import apply_chat_template
    from mlx_vlm.utils import load_config

    t0 = time.monotonic()
    model, processor = load(MODEL_PATH)
    config = load_config(MODEL_PATH)
    print(f"[bench] Model loaded in {time.monotonic()-t0:.1f}s")

    formatted = apply_chat_template(processor, config, _PROMPT, num_images=1)

    tmp_image = resize_to_square(args.image, IMAGE_SIZE)
    print(f"[bench] Image: {args.image} → {IMAGE_SIZE}px")
    print(f"[bench] {args.trials} trials ({args.warmup} warmup), max_tokens={args.max_tokens}\n")

    lats = []
    total = args.trials + args.warmup
    for i in range(total):
        t_start = time.monotonic()
        result = generate(model, processor, formatted, image=tmp_image,
                          max_tokens=args.max_tokens, temperature=0.0, verbose=False)
        elapsed = time.monotonic() - t_start

        text = result.text if hasattr(result, "text") else str(result)
        parsed, raw_json = extract_json(text)
        if parsed is not None:
            caption = (parsed.get("caption") or "")[:60]
            objects = parsed.get("new_objects") or []
            ok = "✓"
        else:
            caption = text[:60].replace("\n", " ")
            objects = []
            ok = "✗"

        if args.verbose and i >= args.warmup:
            print(f"  --- raw ({len(text)} chars) ---")
            print(text[:400])
            print(f"  --- parsed json ---")
            print(raw_json[:200] if raw_json else "(none)")

        label = "WARM" if i < args.warmup else f"  {i - args.warmup + 1:3d}"
        ptok = result.prompt_tokens if hasattr(result, "prompt_tokens") else -1
        ctok = result.generation_tokens if hasattr(result, "generation_tokens") else -1
        print(f"  {label}  {elapsed*1000:7.1f} ms  {ok}  p={ptok} c={ctok}  obj={len(objects)}  '{caption}'")

        if i >= args.warmup:
            lats.append(elapsed * 1000)

    import os
    os.unlink(tmp_image)

    if not lats:
        print("[bench] No results.")
        return

    print(f"\n[bench] ── Results ──────────────────────────────────")
    print(f"  median  {statistics.median(lats):7.1f} ms")
    print(f"  mean    {statistics.mean(lats):7.1f} ms  ± {statistics.stdev(lats) if len(lats)>1 else 0:.1f}")
    print(f"  p95     {sorted(lats)[int(len(lats)*0.95)]:7.1f} ms")
    print(f"  min/max {min(lats):.1f} / {max(lats):.1f} ms")
    print(f"\n[bench] Baseline (Qwen3-VL-2B via mlx_vlm.server): median ~2785 ms")
    speedup = 2785 / statistics.median(lats)
    print(f"[bench] Speedup vs baseline: {speedup:.1f}×")


if __name__ == "__main__":
    main()
