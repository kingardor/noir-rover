#!/usr/bin/env python3
"""
Test camera feed — noir_env, no robot required.

Writes synthetic JPEG frames to Redis (camera:frame / camera:ts) at a
configurable rate so the full vision pipeline (YOLOE, VLM, face-rec,
kg_builder) can be exercised without the Scout robot being powered on.

Frames are generated with PIL: a 640×384 test card showing colour bars,
a timestamp, the frame counter, and a "TEST MODE" label. The colours
cycle slowly so the VLM actually sees variation between frames.

Yields immediately to real bridge frames: if camera:ts advances without
us writing it, we do nothing for that cycle (bot must have just come on).

Usage:
    REDIS_URL=redis://localhost:6380 python -u vision/test_feed.py
"""

import base64
import io
import math
import os
import time

import redis as redis_lib
from PIL import Image, ImageDraw, ImageFont

# ── Config ─────────────────────────────────────────────────────────────────────
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6380")
FPS       = float(os.getenv("TEST_FPS", "5"))
W, H      = 640, 384


def _make_frame(frame_no: int, ts: float) -> bytes:
    """Generate a synthetic test-card JPEG."""
    img = Image.new("RGB", (W, H), (8, 8, 16))
    draw = ImageDraw.Draw(img)

    # Slowly cycling hue for colour bars (7 bars)
    n_bars = 7
    bar_w  = W // n_bars
    hue_offset = (frame_no * 2) % 360
    for i in range(n_bars):
        hue = (hue_offset + i * (360 // n_bars)) % 360
        r, g, b = _hsv_to_rgb(hue / 360, 0.75, 0.75)
        x0, x1 = i * bar_w, (i + 1) * bar_w
        draw.rectangle([x0, H // 2, x1, H - 1], fill=(r, g, b))

    # Corner registration marks
    _corner(draw, 0, 0, 20, (0, 255, 136))
    _corner(draw, W - 20, 0, 20, (0, 255, 136))
    _corner(draw, 0, H - 20, 20, (0, 255, 136))
    _corner(draw, W - 20, H - 20, 20, (0, 255, 136))

    # Labels — try a proper font, fall back to default
    try:
        font_lg = ImageFont.truetype("/System/Library/Fonts/Supplemental/Courier New.ttf", 28)
        font_sm = ImageFont.truetype("/System/Library/Fonts/Supplemental/Courier New.ttf", 14)
    except Exception:
        font_lg = ImageFont.load_default()
        font_sm = font_lg

    draw.text((W // 2, 28), "TEST MODE", fill=(255, 34, 85), font=font_lg, anchor="mm")
    ts_str   = time.strftime("%H:%M:%S", time.localtime(ts))
    draw.text((W // 2, 66), f"frame {frame_no:06d}  ·  {ts_str}", fill=(0, 212, 255), font=font_sm, anchor="mm")
    draw.text((W // 2, 86), "no robot connected — synthetic feed", fill=(69, 69, 104), font=font_sm, anchor="mm")

    # Centre crosshair
    cx, cy = W // 2, H // 4
    draw.line([(cx - 30, cy), (cx + 30, cy)], fill=(200, 200, 200), width=1)
    draw.line([(cx, cy - 20), (cx, cy + 20)], fill=(200, 200, 200), width=1)
    draw.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], outline=(200, 200, 200), width=1)

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _hsv_to_rgb(h: float, s: float, v: float):
    if s == 0:
        c = int(v * 255)
        return c, c, c
    i = int(h * 6)
    f = h * 6 - i
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    i %= 6
    if i == 0: return int(v*255), int(t*255), int(p*255)
    if i == 1: return int(q*255), int(v*255), int(p*255)
    if i == 2: return int(p*255), int(v*255), int(t*255)
    if i == 3: return int(p*255), int(q*255), int(v*255)
    if i == 4: return int(t*255), int(p*255), int(v*255)
    return     int(v*255), int(p*255), int(q*255)


def _corner(draw, x, y, size, color):
    draw.line([(x, y), (x + size, y)], fill=color, width=2)
    draw.line([(x, y), (x, y + size)], fill=color, width=2)


def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=False)
    r_str = redis_lib.from_url(REDIS_URL, decode_responses=True)

    interval = 1.0 / FPS
    frame_no = 0

    print(f"[test-feed] writing {FPS:.0f} fps synthetic frames → camera:frame  (Redis {REDIS_URL})", flush=True)
    print(f"[test-feed] ctrl-C to stop — stop before powering on robot", flush=True)

    while True:
        t0 = time.time()

        # Yield to real bridge frames: if camera:ts advanced since last cycle
        # by more than 2× our interval, the bridge must be writing real frames.
        real_ts = r_str.get("camera:ts")
        if real_ts:
            try:
                if (t0 - float(real_ts)) < interval * 2:
                    # Real feed is live — sleep and let bridge own the key
                    time.sleep(interval)
                    continue
            except ValueError:
                pass

        # Generate and write frame
        now = time.time()
        jpg = _make_frame(frame_no, now)
        b64 = base64.b64encode(jpg)
        r.set("camera:frame", b64, ex=5)
        r_str.set("camera:ts", str(now), ex=5)
        frame_no += 1

        if frame_no % (int(FPS) * 10) == 0:
            print(f"[test-feed] frame {frame_no:06d}  {time.strftime('%H:%M:%S')}", flush=True)

        elapsed = time.time() - t0
        sleep   = max(0.0, interval - elapsed)
        time.sleep(sleep)


if __name__ == "__main__":
    main()
