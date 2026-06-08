#!/usr/bin/env python3
"""One-shot: print full raw output from InternVL3-1B at 384px."""
import io, json, re, sys, time, tempfile
from pathlib import Path
from PIL import Image

_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_ROOT / "vision"))

MODEL_PATH = "mlx-community/InternVL3-1B-4bit"
IMAGE_SIZE = 384
IMAGE = str(_ROOT / "data" / "kg" / "images_test" / "038151c1-5be3-4c2e-bb76-dc674d6976b0.jpg")

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

# resize
img = Image.open(IMAGE).convert("RGB").resize((IMAGE_SIZE, IMAGE_SIZE), Image.LANCZOS)
tmp = tempfile.mktemp(suffix=".jpg")
img.save(tmp, format="JPEG", quality=85)

from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

model, processor = load(MODEL_PATH)
config = load_config(MODEL_PATH)
formatted = apply_chat_template(processor, config, _PROMPT, num_images=1)

result = generate(model, processor, formatted, image=tmp,
                  max_tokens=2048, temperature=0.0, verbose=False)
text = result.text if hasattr(result, "text") else str(result)

print(f"=== FULL RAW OUTPUT ({len(text)} chars, c={result.generation_tokens}) ===")
print(repr(text))
print()
print("=== RENDERED ===")
print(text)
print()

# Try to extract JSON
start = text.find('{')
if start != -1:
    depth = 0
    for i, ch in enumerate(text[start:], start):
        if ch == '{': depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                candidate = text[start:i+1]
                print(f"=== JSON CANDIDATE ({len(candidate)} chars) ===")
                print(candidate)
                try:
                    parsed = json.loads(candidate)
                    print("\n✓ Valid JSON")
                    print("Caption:", parsed.get("caption"))
                    print("Objects:", [o.get("label") for o in parsed.get("new_objects", [])])
                except Exception as e:
                    print(f"\n✗ Parse error: {e}")
                break
    else:
        print("=== No complete JSON object found ===")

import os
os.unlink(tmp)
