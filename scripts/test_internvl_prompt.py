#!/usr/bin/env python3
"""
Test InternVL3-1B prompt formats for KG extraction.
Tries several prompt variations and reports which produces clean caption + objects.
Usage: python scripts/test_internvl_prompt.py [--image path/to/image.jpg]
"""
import json
import sys
import time
from pathlib import Path

# ── bootstrap ──────────────────────────────────────────────────────────────────
_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_ROOT / "vision"))

from mlx_vlm import load
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config
import mlx.core as mx

MODEL_PATH = "mlx-community/InternVL3-1B-4bit"

# ── prompts ────────────────────────────────────────────────────────────────────

PROMPTS = {
    # Final KG candidate — includes known-labels context, no bracket placeholders
    "kg_final": """\
Analyze this image. Return ONLY valid JSON, no extra text.

Things already in my knowledge graph: [laptop, headphones]

{
  "caption": "",
  "new_objects": [{"label": "object name", "attrs": {"color": "color", "size": "small/medium/large"}}],
  "new_events": [],
  "relationships": [{"subject": "label1", "predicate": "on", "object": "label2"}],
  "changes": []
}

Fill caption with one sentence about the overall scene.
Fill new_objects with every distinct physical object NOT already in my graph. Short noun labels (1-3 words).
Fill relationships with spatial facts between objects (on, next_to, held_by, inside, above, etc.).
Reply with only the completed JSON.""",

    # Combined prompt — no context, confirmed working
    "combined": """\
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
Reply with only the completed JSON.""",
}

# ── load model ─────────────────────────────────────────────────────────────────

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=str(_ROOT / "data" / "kg" / "images_test" /
                    "038151c1-5be3-4c2e-bb76-dc674d6976b0.jpg"))
    ap.add_argument("--prompt", default=None, help="Run only this prompt key")
    ap.add_argument("--max-tokens", type=int, default=512)
    args = ap.parse_args()

    image_path = args.image
    print(f"[test] Loading {MODEL_PATH}...")
    t0 = time.monotonic()
    model, processor = load(MODEL_PATH)
    config = load_config(MODEL_PATH)
    print(f"[test] Model loaded in {time.monotonic()-t0:.1f}s\n")
    print(f"[test] Image: {image_path}\n")

    keys = [args.prompt] if args.prompt else list(PROMPTS.keys())

    for key in keys:
        prompt_text = PROMPTS[key]
        print(f"{'─'*60}")
        print(f"PROMPT: {key}")
        print(f"{'─'*60}")

        try:
            formatted = apply_chat_template(processor, config, prompt_text,
                                            num_images=1)
            t_start = time.monotonic()
            output = ""
            from mlx_vlm import generate
            result = generate(model, processor, formatted, image=image_path,
                              max_tokens=args.max_tokens, temperature=0.0, verbose=False)
            elapsed = time.monotonic() - t_start

            output = result.text if hasattr(result, "text") else (result if isinstance(result, str) else str(result))
            print(f"Time: {elapsed:.2f}s")
            print(f"Output ({len(output)} chars):")
            print(output[:600])

            # Try parsing JSON
            import re
            clean = output.strip()
            clean = re.sub(r'^```(?:json)?\s*', '', clean)
            clean = re.sub(r'\s*```$', '', clean)
            try:
                parsed = json.loads(clean)
                caption = parsed.get("caption") or parsed.get("Caption") or ""
                objects = parsed.get("new_objects") or parsed.get("objects") or []
                print(f"\n✓ Valid JSON  |  caption='{caption[:80]}'  |  objects={len(objects)}")
                if objects:
                    labels = [o.get("label", o) if isinstance(o, dict) else o for o in objects[:5]]
                    print(f"  labels: {labels}")
            except json.JSONDecodeError as e:
                print(f"\n✗ JSON parse failed: {e}")
        except Exception as e:
            print(f"ERROR: {e}")
        print()


if __name__ == "__main__":
    main()
