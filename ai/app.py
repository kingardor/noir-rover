import os
import time
import cv2
import requests
import numpy as np
from PIL import Image
import supervision as sv
from ultralytics import YOLOE

# ---------- Config ----------
API_BASE = os.getenv("BOT_API_BASE", "http://0.0.0.0:6969")
CAMERA_ENDPOINT = f"{API_BASE}/camera/frame"
HTTP_TIMEOUT = 1.0
CHECKPOINT = "yoloe-11l-seg.pt"
NAMES = ["person", "spectacles", "tattoos"]
DEVICE = "cuda"  # falls back to CPU if unavailable

# ---------- Model ----------
model = YOLOE(CHECKPOINT)
try:
    model.to(DEVICE)
except Exception:
    print("Falling back to CPU")
    model.to("cpu")
model.set_classes(NAMES, model.get_text_pe(NAMES))

# ---------- HTTP session ----------
sess = requests.Session()
sess.headers.update({"Accept": "image/jpeg"})

# ---------- Annotators (initialized after first frame size is known) ----------
box_annotator = None
label_annotator = None
thickness = 2
text_scale = 0.5

win_name = "YOLOE — HTTP camera (q to quit)"
cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

while True:
    # ---- Fetch frame over HTTP ----
    try:
        r = sess.get(CAMERA_ENDPOINT, timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        arr = np.frombuffer(r.content, dtype=np.uint8)
        frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame_bgr is None:
            frame_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    except Exception as e:
        print(f"[HTTP] {e}")
        time.sleep(1)
        continue

    # ---- Lazy annotator init based on resolution ----
    if box_annotator is None:
        h, w = frame_bgr.shape[:2]
        resolution_wh = (w, h)
        thickness = sv.calculate_optimal_line_thickness(resolution_wh=resolution_wh)
        text_scale = sv.calculate_optimal_text_scale(resolution_wh=resolution_wh)
        box_annotator = sv.BoxAnnotator(color_lookup=sv.ColorLookup.INDEX, thickness=thickness)
        label_annotator = sv.LabelAnnotator(
            color_lookup=sv.ColorLookup.INDEX, text_scale=text_scale, smart_position=True
        )

    # ---- Inference ----
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(frame_rgb)

    t0 = time.time()
    results = model.predict(pil_img, verbose=False)
    dt = time.time() - t0
    fps = 1.0 / dt if dt > 0 else 0.0
    print(f"Inference: {dt*1000:.1f} ms  ({fps:.1f} FPS)")

    dets = sv.Detections.from_ultralytics(results[0])

    if "class_name" in dets:
        labels = [f"{cn} {conf:.2f}" for cn, conf in zip(dets["class_name"], dets.confidence)]
    else:
        labels = [f"{(NAMES[cid] if cid < len(NAMES) else cid)} {conf:.2f}"
                  for cid, conf in zip(dets.class_id, dets.confidence)]

    annotated = frame_rgb.copy()
    annotated = box_annotator.annotate(scene=annotated, detections=dets)
    annotated = label_annotator.annotate(scene=annotated, detections=dets, labels=labels)

    out_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
    cv2.imshow(win_name, out_bgr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
