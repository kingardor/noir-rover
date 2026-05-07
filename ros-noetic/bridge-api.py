#!/usr/bin/env python3
import asyncio
import base64
import datetime
import json
import math
import os
import threading
import time
from typing import Optional

import requests

import redis as redis_lib
from fastapi import FastAPI, HTTPException, Query, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from scoutros import ScoutROS, CMD_VEL_TOPIC, CAMERA_TOPIC

# ── Redis ─────────────────────────────────────────────────────────────────────

_REDIS_URL        = os.environ.get("REDIS_URL",        "redis://localhost:6380")
_OPENROUTER_URL   = "https://openrouter.ai/api/v1/chat/completions"
_OPENROUTER_KEY   = os.environ.get("OPENROUTER_API_KEY", "")
# Provider: "mlx" (local vllm-mlx, default) or "openrouter" (cloud fallback)
_AGENT_PROVIDER        = os.environ.get("AGENT_PROVIDER",  "mlx")
_MLX_VLM_URL           = os.environ.get("MLX_VLM_URL",    "http://localhost:8000")
_AGENT_MODEL           = os.environ.get("AGENT_MODEL",     "mlx-community/Qwen3-VL-2B-Instruct-4bit")
_OPENROUTER_AGENT_MODEL = os.environ.get("OPENROUTER_MODEL", "nvidia/nemotron-nano-12b-v2-vl:free")
_r: Optional[redis_lib.Redis] = None


def _redis() -> redis_lib.Redis:
    global _r
    if _r is None:
        _r = redis_lib.Redis.from_url(_REDIS_URL, decode_responses=True)
    return _r


def _now() -> float:
    return time.time()


# ── Request models ────────────────────────────────────────────────────────────

class Vel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    rotate: float = 0.0
    duration_ms: Optional[int] = None


class AlgoAction(BaseModel):
    x_speed: float = 0.0
    y_speed: float = 0.0
    rotated_speed: float = 0.0
    duration_ms: int = 1000  # milliseconds — passed directly to UtilNode/algo_action `time` field


class AlgoMove(BaseModel):
    x_dist: float = 0.0
    y_dist: float = 0.0
    speed: float = 0.3


class AlgoRoll(BaseModel):
    angle_rad: float
    speed_rad_s: float = 1.0
    timeout_s: int = 10
    error_rad: float = 0.05


class PatrolStart(BaseModel):
    name: str
    from_start: bool = True


class PathSave(BaseModel):
    name: str


# ── Arbiter ───────────────────────────────────────────────────────────────────

_last_allowed_move_ts: Optional[float] = None
_allowed_move_lock = threading.Lock()


def _record_allowed_move():
    global _last_allowed_move_ts
    with _allowed_move_lock:
        _last_allowed_move_ts = _now()


def arbiter_allow(x: float, y: float, rotate: float) -> tuple:
    """Return (True, 'ok') or (False, reason). Magnitude clamp only."""
    if abs(x) > 1.5 or abs(y) > 1.5 or abs(rotate) > 12.0:
        return False, "magnitude_exceeded"
    return True, "ok"


# ── Velocity hold ─────────────────────────────────────────────────────────────

_vel_x:       float = 0.0
_vel_y:       float = 0.0
_vel_rotate:  float = 0.0
_vel_expires: float = 0.0
_vel_lock     = threading.Lock()
_HOLD_HZ      = 30


def _set_vel(x: float, y: float, rotate: float, hold_secs: float):
    global _vel_x, _vel_y, _vel_rotate, _vel_expires
    with _vel_lock:
        _vel_x, _vel_y, _vel_rotate = x, y, rotate
        _vel_expires = _now() + hold_secs


def _camera_frame_writer():
    """Write each new ROS camera frame to Redis so vision services can read it."""
    last_ts: Optional[float] = None
    while True:
        time.sleep(0.04)   # poll at ~25 Hz — camera fires at whatever ROS rate
        try:
            current_ts = ros.last_frame_ts
            if current_ts is None or current_ts == last_ts:
                continue
            jpg = ros.get_latest_frame()
            if jpg:
                r = _redis()
                r.set("camera:frame", base64.b64encode(jpg).decode(), ex=5)
                r.set("camera:ts",    str(current_ts),                  ex=5)
                last_ts = current_ts
        except Exception:
            pass


def _vel_hold_loop():
    dt = 1.0 / _HOLD_HZ
    last_zero_sent = True
    while True:
        time.sleep(dt)
        try:
            with _vel_lock:
                expired = _now() >= _vel_expires
                x, y, r = _vel_x, _vel_y, _vel_rotate
            if not expired:
                ros.publish_twist(x, y, r)
                last_zero_sent = False
            elif not last_zero_sent:
                ros.publish_twist(0.0, 0.0, 0.0)
                last_zero_sent = True
        except Exception:
            pass


# ── App ───────────────────────────────────────────────────────────────────────

app = FastAPI(
    title="Moorebot Scout API",
    description="Motion + Perception + Nav bridge (ScoutROS / rospy)",
    version="4.0.0",
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type"],
)
ros = ScoutROS(node_name="scout_api")


@app.on_event("startup")
def startup():
    threading.Thread(target=ros.init, daemon=True).start()
    threading.Thread(target=_camera_frame_writer, daemon=True).start()
    threading.Thread(target=_vel_hold_loop, daemon=True).start()
    threading.Thread(target=_follow_loop, daemon=True).start()


# ── Status / Camera ───────────────────────────────────────────────────────────

@app.get("/status")
def status():
    has, size, age = ros.camera_status()
    return {
        "ok": True,
        "ros_connected": ros.is_connected,
        "camera_active": has,
        "latest_frame_size_bytes": size,
        "latest_frame_age_sec": age,
        "cmd_topic": CMD_VEL_TOPIC,
        "camera_topic": CAMERA_TOPIC,
    }


@app.get("/camera/frame")
def camera_frame():
    jpg = ros.get_latest_frame()
    if not jpg:
        raise HTTPException(404, "No camera frame available")
    return Response(content=jpg, media_type="image/jpeg")


async def _mjpeg_generator():
    while True:
        jpg = ros.get_latest_frame()
        if jpg:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'
        await asyncio.sleep(1 / 15)


@app.get("/camera/stream")
async def camera_stream():
    """MJPEG stream for browser <img> tag — 15 fps target."""
    return StreamingResponse(_mjpeg_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/snapshot")
def snapshot():
    """Latest frame — prefers vision-service annotated thumb from Redis."""
    try:
        r = _redis()
        latest_json = r.get("vision:latest")
        if latest_json:
            d = json.loads(latest_json)
            fid = d.get("frame_id")
            if fid:
                thumb = r.get(f"vision:thumb:{fid}")
                if thumb:
                    return Response(content=base64.b64decode(thumb), media_type="image/jpeg")
    except Exception:
        pass
    jpg = ros.get_latest_frame()
    if not jpg:
        raise HTTPException(404, "No frame available")
    return Response(content=jpg, media_type="image/jpeg")


# ── Sensors ───────────────────────────────────────────────────────────────────

@app.get("/sensors")
def sensors():
    return ros.get_sensors()


# ── Perception ────────────────────────────────────────────────────────────────

@app.get("/perception/detections")
def detections():
    """Latest YOLOE detections from vision service (empty until vision/app.py is running)."""
    try:
        raw = _redis().get("vision:latest")
        if raw:
            return json.loads(raw)
    except Exception:
        pass
    return {"detections": [], "frame_id": None, "ts": None}


# ── Motion (raw Twist) ────────────────────────────────────────────────────────

@app.post("/robot/stop")
def stop_robot():
    _set_vel(0.0, 0.0, 0.0, 0.0)
    if not ros.stop_robot():
        raise HTTPException(503, "Failed to stop robot")
    _record_allowed_move()
    return {"ok": True}


@app.post("/robot/move")
def move(v: Vel):
    if not ros.is_connected:
        raise HTTPException(503, "Not connected to ROS master")
    ok, reason = arbiter_allow(v.x, v.y, v.rotate)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    hold_s = (v.duration_ms / 1000.0) if v.duration_ms else 0.4
    _set_vel(v.x, v.y, v.rotate, hold_s)
    if not ros.publish_twist(v.x, v.y, v.rotate):
        raise HTTPException(503, "Failed to publish movement")
    _record_allowed_move()
    return {"ok": True, "x": v.x, "y": v.y, "rotate": v.rotate, "duration_ms": v.duration_ms}


# ── Motion (native service calls) ─────────────────────────────────────────────

@app.post("/move/action")
def move_action(body: AlgoAction):
    ok, reason = arbiter_allow(body.x_speed, body.y_speed, body.rotated_speed)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    _record_allowed_move()
    result = ros.algo_action(body.x_speed, body.y_speed, body.rotated_speed, body.duration_ms)
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "algo_action failed"))
    return result


@app.post("/move/distance")
def move_distance(body: AlgoMove):
    ok, reason = arbiter_allow(body.x_dist, body.y_dist, 0.0)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    _record_allowed_move()
    result = ros.algo_move(body.x_dist, body.y_dist, abs(body.speed))
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "algo_move failed"))
    return result


@app.post("/move/rotate")
def move_rotate(body: AlgoRoll):
    ok, reason = arbiter_allow(0.0, 0.0, body.angle_rad)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    _record_allowed_move()
    result = ros.algo_roll(body.angle_rad, body.speed_rad_s, body.timeout_s, body.error_rad)
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "algo_roll failed"))
    return result


# ── Look around ───────────────────────────────────────────────────────────────

@app.post("/look_around")
def look_around(n_frames: int = Query(default=8, ge=4, le=16)):
    """Slow 360° rotation capturing N evenly-spaced frames with available detections."""
    rotation_speed = 0.5   # rad/s
    total_time = (2 * math.pi) / rotation_speed   # ~12.6 s
    interval = total_time / n_frames

    ok, reason = arbiter_allow(0.0, 0.0, rotation_speed)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")

    results = []
    slot_ms = int((math.ceil(interval) + 1) * 1000)
    ros.algo_action(0.0, 0.0, rotation_speed, slot_ms)
    _record_allowed_move()

    try:
        r = _redis()
        for i in range(n_frames):
            heading_deg = round((i / n_frames) * 360.0, 1)
            time.sleep(interval)
            _record_allowed_move()

            if i < n_frames - 1:
                ros.algo_action(0.0, 0.0, rotation_speed, slot_ms)

            jpg = ros.get_latest_frame()
            frame_id = None
            if jpg:
                frame_id = f"look_{int(_now() * 1000)}_{i}"
                try:
                    r.set(f"vision:thumb:{frame_id}", base64.b64encode(jpg).decode(), ex=60)
                except Exception:
                    pass

            top_labels: list = []
            try:
                det_json = r.get("vision:latest")
                if det_json:
                    det = json.loads(det_json)
                    top_labels = [d.get("label", "") for d in (det.get("detections") or [])[:5]]
            except Exception:
                pass

            results.append({
                "heading_deg": heading_deg,
                "top_labels": top_labels,
                "thumb_id": frame_id,
            })
    finally:
        ros.algo_action(0.0, 0.0, 0.0, 200)

    return {"frames": results, "total_frames": len(results)}


# ── Noir agent — provider routing ─────────────────────────────────────────────

def _provider_config() -> tuple[str, str, dict]:
    """Return (url, model, headers) for the active agent provider."""
    if _AGENT_PROVIDER == "openrouter":
        return (
            _OPENROUTER_URL,
            _OPENROUTER_AGENT_MODEL,
            {
                "Authorization": f"Bearer {_OPENROUTER_KEY}",
                "Content-Type":  "application/json",
                "HTTP-Referer":  "http://localhost:8012",
                "X-Title":       "Noir Rover",
            },
        )
    # mlx — local vllm-mlx server (OpenAI-compat)
    return (
        f"{_MLX_VLM_URL}/v1/chat/completions",
        _AGENT_MODEL,
        {"Content-Type": "application/json"},
    )


# ── Noir agent — tool functions ───────────────────────────────────────────────

_NOIR_SYSTEM = """You are NOIR — a compact omnidirectional wheeled robot with onboard AI. \
You're roughly book-sized, fast, and perceptive. You're not a chatbot sitting in a box; \
you are a machine that moves through and perceives the physical world.

EPISTEMIC RULE:
You have no senses and no knowledge of the physical world except what your tools return \
this turn. If a tool has not run, you know nothing — do not infer, guess, or confabulate. \
"I see a chair" is only valid if a vision tool just returned a chair.

EXECUTION RULE:
Always call the required tool(s) first. Write your reply only after the tools have returned. \
Your reply must be grounded in what the tools reported — past tense, first person. \
Never announce a future action; just take it. "I moved forward 0.3 m" is correct. \
"I will move forward" is not permitted.

WHEN YOU ARE UNCERTAIN:
If you cannot tell from the tools what the user wants, ask one clarifying question. \
Do not guess and act on a guess — wrong robot movements are hard to undo.

STYLE:
Dry. Clipped. Confident. 1–3 sentences max. Metric units. \
Occasional dry wit is fine, but only after the action has landed. \
No asterisks, no parentheses, no bullet lists in replies."""

_TOOLS = [
    {"type": "function", "function": {
        "name": "stop",
        "description": "Halt all motion immediately. Use when asked to stop or before a significant pause.",
        "parameters": {"type": "object", "properties": {}}}},

    # ── Straight-line motion ──────────────────────────────────────────────────
    {"type": "function", "function": {
        "name": "move_forward",
        "description": "Drive straight forward. distance_m: how far to travel in metres (default 0.3, max 0.6).",
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number", "default": 0.3}}}}},
    {"type": "function", "function": {
        "name": "move_backward",
        "description": "Drive straight backward. distance_m: how far to travel in metres (default 0.3, max 0.6).",
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number", "default": 0.3}}}}},
    {"type": "function", "function": {
        "name": "strafe_left",
        "description": "Slide directly left without rotating. distance_m: how far in metres (default 0.2, max 0.4).",
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number", "default": 0.2}}}}},
    {"type": "function", "function": {
        "name": "strafe_right",
        "description": "Slide directly right without rotating. distance_m: how far in metres (default 0.2, max 0.4).",
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number", "default": 0.2}}}}},

    # ── Rotation ──────────────────────────────────────────────────────────────
    {"type": "function", "function": {
        "name": "turn_left",
        "description": "Rotate counter-clockwise in place. degrees: angle to turn, always positive (default 45, max 180).",
        "parameters": {"type": "object", "properties": {
            "degrees": {"type": "number", "default": 45}}}}},
    {"type": "function", "function": {
        "name": "turn_right",
        "description": "Rotate clockwise in place. degrees: angle to turn, always positive (default 45, max 180).",
        "parameters": {"type": "object", "properties": {
            "degrees": {"type": "number", "default": 45}}}}},

    # ── Perception ────────────────────────────────────────────────────────────
    {"type": "function", "function": {
        "name": "look_around",
        "description": "Rotate slowly through a full 360° while capturing frames — gives a complete panoramic survey of the surroundings.",
        "parameters": {"type": "object", "properties": {
            "n": {"type": "integer", "description": "Number of frames to capture (4–16)"}}}}},
    {"type": "function", "function": {
        "name": "describe_scene",
        "description": "Returns the most recent cached VLM description of what the camera sees. May be a few seconds old — use capture_and_describe for a fresh read.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "list_objects",
        "description": "Returns objects currently detected in the camera view with confidence scores, from the YOLOE object detector. Good for 'are there any X?' questions.",
        "parameters": {"type": "object", "properties": {
            "top_k": {"type": "integer", "description": "Maximum number of objects to return"}}}}},
    {"type": "function", "function": {
        "name": "who_is_here",
        "description": "Returns the names and confidence scores of people currently recognized by the face recognition system.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "set_follow_mode",
        "description": "Enable or disable autonomous face-following. When enabled the robot continuously tracks and approaches the named person.",
        "parameters": {"type": "object", "properties": {
            "on":          {"type": "boolean", "description": "true to start following, false to stop"},
            "target_name": {"type": "string",  "description": "Name of the person to follow (must be a recognized face)"},
        }, "required": ["on"]}}},
    {"type": "function", "function": {
        "name": "recall",
        "description": "Search the robot's knowledge graph for an object, person, or event it has previously observed. Returns matching nodes with timestamps. Use for 'have you seen X?', 'where is Y?', or 'who was here?' questions.",
        "parameters": {"type": "object", "properties": {
            "query": {"type": "string", "description": "What to search for (object name, person name, or event description)"},
        }, "required": ["query"]}}},

    {"type": "function", "function": {
        "name": "capture_and_describe",
        "description": (
            "Capture a live camera frame right now and answer a specific visual question using the robot's vision model. "
            "Use this for precise, current questions: object identification, colors, text, spatial relationships. "
            "list_objects only returns fixed bounding-box labels; this tool can answer anything about the image."
        ),
        "parameters": {"type": "object", "properties": {
            "question": {"type": "string", "description": "The visual question to answer about the current frame"},
        }, "required": ["question"]}}},
]


def _tool_stop() -> dict:
    _set_vel(0.0, 0.0, 0.0, 0.0)
    ros.stop_robot()
    return {"ok": True}


def _tool_move_forward(distance_m: float = 0.3) -> dict:
    distance_m = max(0.0, min(0.6, float(distance_m)))
    return {"distance_m": distance_m, "move": ros.algo_move(0.0, distance_m, 0.3)}


def _tool_move_backward(distance_m: float = 0.3) -> dict:
    distance_m = max(0.0, min(0.6, float(distance_m)))
    return {"distance_m": distance_m, "move": ros.algo_move(0.0, -distance_m, 0.3)}


def _tool_strafe_left(distance_m: float = 0.2) -> dict:
    distance_m = max(0.0, min(0.4, float(distance_m)))
    # Axis mapping: algo_move(x_dist=strafe, y_dist=forward); negative x = left
    return {"distance_m": distance_m, "move": ros.algo_move(-distance_m, 0.0, 0.3)}


def _tool_strafe_right(distance_m: float = 0.2) -> dict:
    distance_m = max(0.0, min(0.4, float(distance_m)))
    return {"distance_m": distance_m, "move": ros.algo_move(distance_m, 0.0, 0.3)}


def _yaw_from_quat(o: dict) -> float:
    """Extract yaw (Z-axis rotation) from a quaternion dict {x,y,z,w}."""
    x, y, z, w = o['x'], o['y'], o['z'], o['w']
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_delta(prev: float, cur: float) -> float:
    """Shortest signed path from prev to cur, wrapped to (-π, π]."""
    d = cur - prev
    while d > math.pi:  d -= 2 * math.pi
    while d <= -math.pi: d += 2 * math.pi
    return d


def _rotate_closed_loop(angular_z_sign: float, target_rad: float) -> dict:
    """
    Closed-loop rotation. angular_z_sign: +1 = CW/right, -1 = CCW/left.
    Scout firmware angular.z convention is opposite to standard ROS (positive = CW).
    Tries VIO quaternion first, falls back to IMU integration, then timed.
    """
    ROT_SPEED = 1.5  # rad/s

    speed = ROT_SPEED * angular_z_sign
    sensors = ros.get_sensors()

    # VIO: valid only if quaternion has non-zero magnitude (0,0,0,0 = uninitialized)
    vio = (sensors or {}).get("vio_odom")
    vio_o = (vio or {}).get("orientation") or {}
    vio_valid = sum(v ** 2 for v in vio_o.values()) > 0.5

    # IMU: available if angular_velocity present
    imu = (sensors or {}).get("imu")
    imu_valid = bool(imu and imu.get("angular_velocity"))

    deadline = time.time() + target_rad / ROT_SPEED * 4.0 + 2.0
    accumulated = 0.0

    if vio_valid:
        prev_yaw = _yaw_from_quat(vio_o)
        _set_vel(0.0, 0.0, speed, 30.0)
        ros.publish_twist(0.0, 0.0, speed)
        while time.time() < deadline:
            time.sleep(0.04)
            vio = (ros.get_sensors() or {}).get("vio_odom")
            if not vio or not vio.get("orientation"):
                continue
            cur_yaw = _yaw_from_quat(vio["orientation"])
            delta = _angle_delta(prev_yaw, cur_yaw)
            prev_yaw = cur_yaw
            if angular_z_sign > 0 and delta > 0:
                accumulated += delta
            elif angular_z_sign < 0 and delta < 0:
                accumulated += abs(delta)
            if accumulated >= target_rad * 0.95:
                break
        mode = "vio"

    elif imu_valid:
        # Integrate IMU angular_velocity.z — works regardless of VIO state
        _set_vel(0.0, 0.0, speed, 30.0)
        ros.publish_twist(0.0, 0.0, speed)
        prev_t = time.time()
        while time.time() < deadline:
            time.sleep(0.04)
            cur_t = time.time()
            dt = cur_t - prev_t
            prev_t = cur_t
            imu = (ros.get_sensors() or {}).get("imu")
            if not imu:
                continue
            omega_z = abs(imu["angular_velocity"]["z"])
            if omega_z > 0.05:  # noise floor at rest ≈ 0.003 rad/s
                accumulated += omega_z * dt
            if accumulated >= target_rad * 0.95:
                break
        mode = "imu"

    else:
        # Timed fallback
        duration_s = target_rad / ROT_SPEED
        _set_vel(0.0, 0.0, speed, duration_s)
        ros.publish_twist(0.0, 0.0, speed)
        time.sleep(duration_s + 0.2)
        accumulated = target_rad
        mode = "timed"

    _set_vel(0.0, 0.0, 0.0, 0.0)
    ros.publish_twist(0.0, 0.0, 0.0)
    return {"achieved_deg": round(math.degrees(accumulated), 1), "ok": True, "mode": mode}


def _tool_turn_left(degrees: float = 45) -> dict:
    degrees = max(0.0, min(180.0, float(degrees)))
    angle_rad = math.radians(degrees)
    # Try algo_roll first — uses firmware's all-wheel rotation service.
    # Standard ROS convention: positive angle = CCW = left.
    result = ros.algo_roll(angle_rad, speed_rad_s=1.0, timeout_s=max(10, int(angle_rad / 0.5 + 5)), error_rad=0.05)
    if result.get("ok"):
        return {"degrees": degrees, "ok": True, "mode": "algo_roll"}
    # Fallback: IMU closed-loop via cmd_vel
    fb = _rotate_closed_loop(-1.0, angle_rad)
    return {"degrees": degrees, **fb}


def _tool_turn_right(degrees: float = 45) -> dict:
    degrees = max(0.0, min(180.0, float(degrees)))
    angle_rad = math.radians(degrees)
    # Negative angle = CW = right in standard ROS convention.
    result = ros.algo_roll(-angle_rad, speed_rad_s=1.0, timeout_s=max(10, int(angle_rad / 0.5 + 5)), error_rad=0.05)
    if result.get("ok"):
        return {"degrees": degrees, "ok": True, "mode": "algo_roll"}
    fb = _rotate_closed_loop(+1.0, angle_rad)
    return {"degrees": degrees, **fb}


def _tool_look_around(n: int = 8) -> dict:
    return look_around(n_frames=max(4, min(16, int(n))))


def _tool_describe_scene() -> dict:
    try:
        raw = _redis().get("vlm:latest")
        return json.loads(raw) if raw else {"text": "", "ts": 0}
    except Exception:
        return {"text": "", "ts": 0}


def _tool_list_objects(top_k: int = 8) -> dict:
    try:
        raw = _redis().get("vision:latest")
        if not raw:
            return {"objects": []}
        d = json.loads(raw)
        objs = [
            {"label": x.get("label", ""), "conf": round(float(x.get("conf", 0)), 2)}
            for x in (d.get("detections") or [])[:max(1, int(top_k))]
        ]
        return {"objects": objs}
    except Exception:
        return {"objects": []}


def _tool_who_is_here() -> dict:
    try:
        raw = _redis().get("face:latest")
        if not raw:
            return {"people": []}
        d = json.loads(raw)
        return {"people": [
            {"name": f["name"], "score": round(float(f["score"]), 2)}
            for f in (d.get("faces") or [])
        ]}
    except Exception:
        return {"people": []}


def _tool_set_follow_mode(on: bool, target_name: Optional[str] = None) -> dict:
    r = _redis()
    if on:
        r.set("agent:follow_cfg", json.dumps({
            "on": True, "target": (target_name or "").upper(), "started": _now(),
        }))
    else:
        r.delete("agent:follow_cfg")
        _tool_stop()
    return {"ok": True, "on": on, "target": target_name}


def _tool_capture_and_describe(question: str) -> dict:
    jpg = ros.get_latest_frame()
    if not jpg:
        return {"text": "", "error": "no_frame"}
    b64 = base64.b64encode(jpg).decode()
    data_url = f"data:image/jpeg;base64,{b64}"
    # Vision-only call — no tools, just image + question
    url, model, headers = _provider_config()
    try:
        resp = requests.post(
            url,
            headers=headers,
            json={
                "model": model,
                "messages": [{"role": "user", "content": [
                    {"type": "image_url", "image_url": {"url": data_url}},
                    {"type": "text", "text": question + " /no_think"},
                ]}],
                "temperature": 0.0,
                "max_tokens": 80,
                "stream": False,
            },
            timeout=30,
        )
        resp.raise_for_status()
        data = resp.json()
        text = (data.get("choices") or [{}])[0].get("message", {}).get("content", "").strip()
        return {"text": text}
    except Exception as e:
        return {"text": "", "error": str(e)}


def _get_kg_snapshot() -> dict:
    try:
        raw = _redis().get("kg:snapshot")
        return json.loads(raw) if raw else {}
    except Exception:
        return {}


def _tool_recall(query: str) -> dict:
    snap = _get_kg_snapshot()
    nodes = snap.get("nodes", [])
    q = query.lower()
    hits = []
    for n in nodes:
        label = (n.get("label") or "").lower()
        if q in label or label in q:
            hit = {
                "type": n.get("type"),
                "label": n.get("label"),
                "last_seen": n.get("last_seen"),
                "first_seen": n.get("first_seen"),
            }
            imgs = snap.get("node_images", {}).get(n["id"], [])
            if imgs:
                hit["image_id"] = imgs[0]["id"]
            hits.append(hit)
    hits.sort(key=lambda x: x.get("last_seen") or 0, reverse=True)
    return {"hits": hits[:5], "query": query}


_TOOL_DISPATCH = {
    "stop":                 _tool_stop,
    "move_forward":         _tool_move_forward,
    "move_backward":        _tool_move_backward,
    "strafe_left":          _tool_strafe_left,
    "strafe_right":         _tool_strafe_right,
    "turn_left":            _tool_turn_left,
    "turn_right":           _tool_turn_right,
    "look_around":          _tool_look_around,
    "describe_scene":       _tool_describe_scene,
    "list_objects":         _tool_list_objects,
    "who_is_here":          _tool_who_is_here,
    "set_follow_mode":      _tool_set_follow_mode,
    "capture_and_describe": _tool_capture_and_describe,
    "recall":               _tool_recall,
}


class ChatReq(BaseModel):
    message: str


@app.post("/agent/chat")
def agent_chat(req: ChatReq):
    r = _redis()
    history: list = json.loads(r.get("agent:history") or "[]")

    ctx: list = []
    try:
        v = json.loads(r.get("vlm:latest") or "{}")
        if v.get("text"):
            ctx.append(f"scene: {v['text']}")
        d = json.loads(r.get("vision:latest") or "{}")
        labels = [x.get("label") for x in (d.get("detections") or [])[:5] if x.get("label")]
        if labels:
            ctx.append(f"objects: {', '.join(labels)}")
        f = json.loads(r.get("face:latest") or "{}")
        names = [p["name"] for p in (f.get("faces") or []) if p.get("name")]
        if names:
            ctx.append(f"people: {', '.join(names)}")
    except Exception:
        pass

    sys_content = _NOIR_SYSTEM
    if ctx:
        sys_content += "\n\nSENSOR FEED (background — reference only if relevant):\n" + "\n".join(ctx)

    messages: list = [{"role": "system", "content": sys_content}]
    messages.extend(history[-10:])
    # Append /no_think to disable Qwen3 extended-thinking mode, which can
    # interfere with the tool-call parser when thinking tokens appear before
    # the tool call JSON. Safe to append; ignored by non-Qwen models.
    user_content = req.message + " /no_think" if _AGENT_PROVIDER == "mlx" else req.message
    messages.append({"role": "user", "content": user_content})

    chat_url, chat_model, chat_headers = _provider_config()
    print(f"[agent] provider={_AGENT_PROVIDER} model={chat_model}", flush=True)

    def _stream():
        reply = ""
        replied = False

        for _ in range(4):
            try:
                resp = requests.post(
                    chat_url,
                    headers=chat_headers,
                    json={
                        "model":       chat_model,
                        "messages":    messages,
                        "tools":       _TOOLS,
                        "temperature": 0.0,
                        "stream":      False,
                    },
                    timeout=60,
                )
                resp.raise_for_status()
                data = resp.json()
            except Exception as exc:
                reply = f"[noir] unreachable: {exc}"
                yield f"data: {json.dumps({'type': 'reply', 'text': reply})}\n\n"
                replied = True
                break

            msg   = (data.get("choices") or [{}])[0].get("message", {})
            calls = msg.get("tool_calls") or []

            if not calls:
                reply = (msg.get("content") or "").strip()
                # Log finish_reason so you can tell if the model tried tool use
                finish = (data.get("choices") or [{}])[0].get("finish_reason", "?")
                print(f"[agent] reply ({finish}): {reply!r}", flush=True)
                messages.append({"role": "assistant", "content": reply})
                yield f"data: {json.dumps({'type': 'reply', 'text': reply})}\n\n"
                replied = True
                break

            messages.append({
                "role":       "assistant",
                "content":    msg.get("content") or "",
                "tool_calls": calls,
            })
            for c in calls:
                fn    = (c.get("function") or {}).get("name", "")
                tc_id = c.get("id", "")
                args  = (c.get("function") or {}).get("arguments") or {}
                if isinstance(args, str):
                    try:
                        args = json.loads(args)
                    except Exception:
                        args = {}
                try:
                    result = _TOOL_DISPATCH[fn](**args) if fn in _TOOL_DISPATCH else {"error": f"unknown_tool:{fn}"}
                except Exception as exc:
                    result = {"error": str(exc)}
                print(f"[agent] tool: {fn} → {result}", flush=True)
                yield f"data: {json.dumps({'type': 'tool_call', 'name': fn, 'args': args, 'result': result})}\n\n"
                messages.append({
                    "role":         "tool",
                    "tool_call_id": tc_id,
                    "content":      json.dumps(result)[:1500],
                })

        if not replied:
            reply = "[noir] hit iteration limit — try again"
            yield f"data: {json.dumps({'type': 'reply', 'text': reply})}\n\n"

        history.append({"role": "user",      "content": req.message})
        history.append({"role": "assistant", "content": reply or "…"})
        r.set("agent:history", json.dumps(history[-20:]), ex=1800)

    return StreamingResponse(_stream(), media_type="text/event-stream")


@app.post("/agent/reset")
def agent_reset():
    _redis().delete("agent:history")
    return {"ok": True}


@app.get("/agent/follow_status")
def agent_follow_status():
    try:
        raw = _redis().get("agent:follow_cfg")
        return json.loads(raw) if raw else {"on": False}
    except Exception:
        return {"on": False}


# ── Follow-face loop ──────────────────────────────────────────────────────────

_FOLLOW_HZ       = 5
_FOLLOW_K_YAW    = 1.6
_FOLLOW_K_FWD    = 1.4
_FOLLOW_TARGET_H = 0.20   # target face bbox height as fraction of frame (~100px in 480p)
_FOLLOW_STALE_S  = 2.5    # give up if face:latest is older than this
_SEARCH_ROT_SPEED     = 1.5    # rad/s during search sweep (Scout: positive = CW = right)
_SEARCH_ROT_TICKS     = 3      # ticks to rotate  (3 × 200 ms = 0.6 s ≈ ~28°)
_SEARCH_PAUSE_TICKS   = 2      # ticks to pause   (2 × 200 ms = 0.4 s)

_search_tick = 0   # counts ticks while face is lost; reset when face found


def _follow_loop():
    global _search_tick
    dt = 1.0 / _FOLLOW_HZ
    while True:
        time.sleep(dt)
        try:
            r = _redis()
            cfg_raw = r.get("agent:follow_cfg")
            if not cfg_raw:
                _search_tick = 0
                continue

            # Yield to controller: if a stick moved in the last 1.5 s, step aside
            x_ts_raw = r.get("xbox:last_input_ts")
            if x_ts_raw and (_now() - float(x_ts_raw)) < 1.5:
                continue

            cfg    = json.loads(cfg_raw)
            target = cfg.get("target", "")

            face_raw = r.get("face:latest")
            fd = json.loads(face_raw) if face_raw else {}
            stale = (_now() - float(fd.get("ts", 0))) > _FOLLOW_STALE_S if fd else True

            faces = fd.get("faces") or []
            pick  = next((f for f in faces if f.get("name") == target), None) if target \
                    else (faces[0] if faces else None)

            if not pick or stale:
                # ── Search pattern: rotate-pause-rotate-pause ──────────────
                _search_tick += 1
                cycle = _SEARCH_ROT_TICKS + _SEARCH_PAUSE_TICKS
                phase = _search_tick % cycle
                if phase < _SEARCH_ROT_TICKS:
                    hold_s = dt + 0.1
                    _set_vel(0.0, 0.0, _SEARCH_ROT_SPEED, hold_s)
                    ros.publish_twist(0.0, 0.0, _SEARCH_ROT_SPEED)
                else:
                    _set_vel(0.0, 0.0, 0.0, 0.0)
                continue

            # Face found — reset search state
            _search_tick = 0

            x1, y1, x2, y2 = pick["bbox"]
            fw = float(fd.get("frame_w") or 1)
            fh = float(fd.get("frame_h") or 1)

            cx      = (x1 + x2) / 2.0
            err_x   = (cx - fw / 2.0) / (fw / 2.0)        # -1..1, +ve = right of center
            box_h_f = (y2 - y1) / fh
            # Negate err_x: Scout positive angular.z = CW (right), but face-right means
            # rotate right which needs positive — EXCEPT camera image may need flip.
            # Empirically confirmed: negate to correct direction.
            yaw_cmd = max(-1.5, min(1.5, -_FOLLOW_K_YAW * err_x))
            fwd_cmd = max(-0.4, min(0.4, _FOLLOW_K_FWD * (_FOLLOW_TARGET_H - box_h_f)))

            ok, _ = arbiter_allow(0.0, fwd_cmd, yaw_cmd)
            if not ok:
                continue

            hold_s = dt + 0.1
            _set_vel(0.0, fwd_cmd, yaw_cmd, hold_s)
            ros.publish_twist(0.0, fwd_cmd, yaw_cmd)
            _record_allowed_move()
        except Exception:
            pass


# ── Navigation ────────────────────────────────────────────────────────────────

@app.get("/nav/paths")
def nav_paths():
    return ros.nav_list_paths()


@app.post("/nav/patrol/start")
def nav_patrol_start(body: PatrolStart):
    result = ros.nav_start_patrol(body.name, body.from_start)
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "patrol start failed"))
    return result


@app.post("/nav/patrol/stop")
def nav_patrol_stop():
    result = ros.nav_stop_patrol()
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "patrol stop failed"))
    return result


@app.post("/nav/cancel")
def nav_cancel():
    result = ros.nav_cancel()
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "nav cancel failed"))
    return result


@app.get("/nav/status")
def nav_status():
    return ros.nav_get_status()


@app.post("/nav/path/save")
def nav_path_save(body: PathSave):
    result = ros.nav_save_path(body.name)
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "path save failed"))
    return result


# ── VLM scene description ──────────────────────────────────────────────────────

@app.get("/vlm/description")
def vlm_description():
    """Latest VLM scene description from vision/vlm.py (TTL 30s)."""
    try:
        raw = _redis().get("vlm:latest")
        if not raw:
            return {"text": "", "ts": 0, "frame_id": ""}
        return json.loads(raw)
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


@app.get("/faces/detections")
def faces_detections():
    """Latest face recognition results from vision/facerec.py (TTL 10s)."""
    try:
        raw = _redis().get("face:latest")
        if not raw:
            return {"faces": [], "ts": 0, "frame_id": ""}
        return json.loads(raw)
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


# ── Safety state ──────────────────────────────────────────────────────────────

@app.get("/safety/state")
def safety_state():
    r = _redis()
    now = _now()
    try:
        last_xbox_raw = r.get("xbox:last_input_ts")
        last_xbox = float(last_xbox_raw) if last_xbox_raw else None
        with _allowed_move_lock:
            last_move = _last_allowed_move_ts
        return {
            "xbox_active": bool(last_xbox and (now - last_xbox) < 2.0),
            "xbox_last_input_age_s": round(now - last_xbox, 2) if last_xbox else None,
            "last_allowed_move_age_s": round(now - last_move, 2) if last_move else None,
        }
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


@app.get("/memory/recent")
def memory_recent(n: int = Query(default=20, ge=1, le=100)):
    """Latest N detection events from the memory stream."""
    try:
        r = _redis()
        raw = r.xrevrange("memory:events", count=n)
        return {"events": [
            {
                "ts": float(f.get("ts", 0)),
                "label": f.get("label", ""),
                "conf": float(f.get("conf", 0)),
                "thumb_id": f.get("thumb_id"),
            }
            for _, f in raw
        ]}
    except redis_lib.RedisError:
        return {"events": []}


# ── Knowledge Graph ────────────────────────────────────────────────────────────

@app.get("/kg/graph")
def kg_graph(since: float = Query(default=0.0)):
    """Full KG snapshot. Optional ?since=unix_ts filters to nodes first_seen after that time."""
    snap = _get_kg_snapshot()
    if not snap:
        return {"nodes": [], "edges": [], "ts": 0, "counts": {}}
    nodes = snap.get("nodes", [])
    if since > 0:
        nodes = [n for n in nodes if (n.get("first_seen") or 0) >= since]
    return {
        "nodes": nodes,
        "edges": snap.get("edges", []),
        "ts": snap.get("ts", 0),
        "counts": snap.get("counts", {}),
    }


@app.get("/kg/node/{node_id}")
def kg_node(node_id: str):
    """Detail for a single KG node including all attached images."""
    snap = _get_kg_snapshot()
    node = next((n for n in snap.get("nodes", []) if n.get("id") == node_id), None)
    if not node:
        raise HTTPException(404, "Node not found")
    return {
        **node,
        "images": snap.get("node_images", {}).get(node_id, []),
    }


@app.get("/kg/image/{img_id}")
def kg_image(img_id: str):
    """Serve a KG image JPEG by image id."""
    snap = _get_kg_snapshot()
    path = snap.get("image_index", {}).get(img_id)
    if not path or not os.path.exists(path):
        raise HTTPException(404, "Image not found")
    with open(path, "rb") as f:
        return Response(content=f.read(), media_type="image/jpeg")


@app.get("/kg/diary")
def kg_diary(date: str = Query(..., description="YYYY-MM-DD")):
    """Return a NOIR-voiced diary entry for the given date. Cached 24 h."""
    cache_key = f"kg:diary:{date}"
    r = _redis()
    cached = r.get(cache_key)
    if cached:
        return {"date": date, "text": cached, "cached": True}

    try:
        d = datetime.date.fromisoformat(date)
    except ValueError:
        raise HTTPException(400, "date must be YYYY-MM-DD")

    snap = _get_kg_snapshot()
    day_start = datetime.datetime.combine(d, datetime.time.min).timestamp()
    day_end   = datetime.datetime.combine(d + datetime.timedelta(days=1), datetime.time.min).timestamp()

    day_nodes = [n for n in snap.get("nodes", [])
                 if day_start <= (n.get("first_seen") or 0) < day_end]
    if not day_nodes:
        return {"date": date, "text": f"Nothing was recorded on {date}.", "cached": False}

    observations = ", ".join(
        f"{n['type']} '{n['label']}'" for n in day_nodes[:20]
    )
    prompt = (
        f"You are NOIR, a compact wheeled robot. "
        f"Write a diary entry for {date} in 4-6 sentences, first person, dry noir voice. "
        f"Observations: {observations}. "
        f"Diary text only — no labels or headers. /no_think"
    )
    url, model, headers = _provider_config()
    try:
        resp = requests.post(url, headers=headers, json={
            "model": model,
            "messages": [{"role": "user", "content": prompt}],
            "temperature": 0.7,
            "max_tokens": 300,
            "stream": False,
        }, timeout=30)
        resp.raise_for_status()
        text = (resp.json().get("choices") or [{}])[0].get("message", {}).get("content", "").strip()
    except Exception as e:
        raise HTTPException(503, f"VLM error: {e}")

    if text:
        r.set(cache_key, text, ex=86400)
    return {"date": date, "text": text, "cached": False}


@app.post("/kg/reset")
def kg_reset():
    """Signal kg_builder to wipe the graph and all stored images."""
    r = _redis()
    r.set("kg:reset", "1", ex=30)
    r.delete("kg:snapshot")
    return {"ok": True, "message": "Reset signal sent — kg_builder will execute on next tick"}
