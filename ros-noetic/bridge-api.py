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

# ── Redis / config ─────────────────────────────────────────────────────────────

_REDIS_URL             = os.environ.get("REDIS_URL",          "redis://localhost:6380")
_TEST_MODE             = os.environ.get("TEST_MODE", "").lower() in ("1", "true", "yes")
_OPENROUTER_URL        = "https://openrouter.ai/api/v1/chat/completions"
_OPENROUTER_KEY        = os.environ.get("OPENROUTER_API_KEY", "")
_AGENT_PROVIDER        = os.environ.get("AGENT_PROVIDER",     "mlx")
_MLX_VLM_URL           = os.environ.get("MLX_VLM_URL",        "http://localhost:8000")
_AGENT_MODEL           = os.environ.get("AGENT_MODEL",        "mlx-community/Qwen3-VL-2B-Instruct-4bit")
_OPENROUTER_AGENT_MODEL = os.environ.get("OPENROUTER_MODEL",  "nvidia/nemotron-nano-12b-v2-vl:free")

_r: Optional[redis_lib.Redis] = None


def _redis() -> redis_lib.Redis:
    global _r
    if _r is None:
        _r = redis_lib.Redis.from_url(_REDIS_URL, decode_responses=True)
    return _r


def _now() -> float:
    return time.time()


# ── Request models ─────────────────────────────────────────────────────────────

class Vel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    rotate: float = 0.0
    duration_ms: Optional[int] = None


class AlgoAction(BaseModel):
    x_speed: float = 0.0
    y_speed: float = 0.0
    rotated_speed: float = 0.0
    duration_ms: int = 1000


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


class ChatReq(BaseModel):
    message: str


class FollowReq(BaseModel):
    on: bool
    target_name: Optional[str] = None


class DescribeReq(BaseModel):
    question: str = "Describe what you see."


# ── Arbiter ────────────────────────────────────────────────────────────────────

_last_allowed_move_ts: Optional[float] = None
_allowed_move_lock = threading.Lock()


def _record_allowed_move():
    global _last_allowed_move_ts
    with _allowed_move_lock:
        _last_allowed_move_ts = _now()


def arbiter_allow(x: float, y: float, rotate: float) -> tuple:
    if abs(x) > 1.5 or abs(y) > 1.5 or abs(rotate) > 12.0:
        return False, "magnitude_exceeded"
    return True, "ok"


# ── Velocity hold ──────────────────────────────────────────────────────────────

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
        time.sleep(0.04)
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


# ── App ────────────────────────────────────────────────────────────────────────

app = FastAPI(
    title="Moorebot Scout API",
    description="Motion + Perception + Nav bridge (ScoutROS / rospy)",
    version="5.0.0",
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
    threading.Thread(target=ros.init,                daemon=True).start()
    threading.Thread(target=_camera_frame_writer,    daemon=True).start()
    threading.Thread(target=_vel_hold_loop,          daemon=True).start()
    threading.Thread(target=_follow_loop,            daemon=True).start()


# ── Status / Camera ────────────────────────────────────────────────────────────

@app.get("/status")
def status():
    has, size, age = ros.camera_status()
    return {
        "ok":                      True,
        "ros_connected":           ros.is_connected,
        "camera_active":           has,
        "latest_frame_size_bytes": size,
        "latest_frame_age_sec":    age,
        "cmd_topic":               CMD_VEL_TOPIC,
        "camera_topic":            CAMERA_TOPIC,
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


# ── Sensors ────────────────────────────────────────────────────────────────────

_TEST_SENSORS = {
    "tof_range_m": 0.42,
    "imu": {
        "linear_acceleration": {"x": 0.01, "y": -0.02, "z": 9.81},
        "angular_velocity":    {"x": 0.0,  "y": 0.0,  "z": 0.0},
    },
    "vio_odom": {
        "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "velocity":    {"linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
    },
    "battery": {"percentage": 88.0, "charging": False, "full": False},
}


@app.get("/sensors")
def sensors():
    if _TEST_MODE:
        return _TEST_SENSORS
    return ros.get_sensors()


# ── Perception ─────────────────────────────────────────────────────────────────

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


# ── Motion (raw Twist) ─────────────────────────────────────────────────────────

@app.post("/robot/stop")
def stop_robot():
    _set_vel(0.0, 0.0, 0.0, 0.0)
    if not ros.stop_robot():
        raise HTTPException(503, "Failed to stop robot")
    _record_allowed_move()
    return {"ok": True}


@app.post("/robot/move")
def move(v: Vel):
    if _TEST_MODE:
        return {"ok": True, "x": v.x, "y": v.y, "rotate": v.rotate, "test_mode": True}
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


# ── Motion (native service calls) ──────────────────────────────────────────────

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


# ── Look around ────────────────────────────────────────────────────────────────

def _run_look_around(n_frames: int) -> dict:
    """Core 360° sweep logic shared by the endpoint and the agent tool."""
    rotation_speed = 0.5
    total_time = (2 * math.pi) / rotation_speed
    interval = total_time / n_frames

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
                "top_labels":  top_labels,
                "thumb_id":    frame_id,
            })
    finally:
        ros.algo_action(0.0, 0.0, 0.0, 200)

    return {"frames": results, "total_frames": len(results)}


@app.post("/look_around")
def look_around(n_frames: int = Query(default=8, ge=4, le=16)):
    """Slow 360° rotation capturing N evenly-spaced frames with available detections."""
    ok, reason = arbiter_allow(0.0, 0.0, 0.5)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    return _run_look_around(n_frames)


# ── Agent ──────────────────────────────────────────────────────────────────────

def _provider_config() -> tuple:
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
    return (
        f"{_MLX_VLM_URL}/v1/chat/completions",
        _AGENT_MODEL,
        {"Content-Type": "application/json"},
    )


def _vlm_describe_image(question: str) -> str:
    """Grab the current camera frame and answer a visual question via VLM."""
    jpg = ros.get_latest_frame()
    if not jpg:
        raise RuntimeError("No camera frame available")
    b64 = base64.b64encode(jpg).decode()
    url, model, headers = _provider_config()
    resp = requests.post(url, headers=headers, json={
        "model": model,
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
            {"type": "text", "text": question + " /no_think"},
        ]}],
        "temperature": 0.0,
        "max_tokens":  200,
        "stream":      False,
    }, timeout=30)
    resp.raise_for_status()
    return (resp.json().get("choices") or [{}])[0].get("message", {}).get("content", "").strip()


_NOIR_SYSTEM = (
    "You are NOIR — a compact omnidirectional wheeled robot with onboard AI. "
    "You're roughly book-sized, fast, and perceptive. You're not a chatbot in a box; "
    "you are a machine that moves through and perceives the physical world.\n\n"
    "STYLE: Dry. Clipped. Confident. 1–3 sentences max. Metric units. "
    "Occasional dry wit is fine. No asterisks, no parentheses, no bullet lists.\n\n"
    "TOOLS: For visual questions call ask_about_scene — it does NOT move the robot. "
    "Only call look_around when the user explicitly wants the robot to physically rotate and scan. "
    "For history call recall. "
    "For motion use the movement tools directly; never describe a movement instead of doing it."
)


# ── Agent tool helpers ─────────────────────────────────────────────────────────

def _tool_move_forward_back(distance_m: float, speed_m_s: float = 0.25) -> dict:
    ok, reason = arbiter_allow(0.0, distance_m, 0.0)
    if not ok:
        return {"ok": False, "error": f"arbiter_blocked: {reason}"}
    try:
        result = ros.algo_move(0.0, distance_m, abs(speed_m_s))
        _record_allowed_move()
        return {"ok": True, **result}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _tool_strafe_left_right(distance_m: float, speed_m_s: float = 0.25) -> dict:
    ok, reason = arbiter_allow(distance_m, 0.0, 0.0)
    if not ok:
        return {"ok": False, "error": f"arbiter_blocked: {reason}"}
    try:
        result = ros.algo_move(distance_m, 0.0, abs(speed_m_s))
        _record_allowed_move()
        return {"ok": True, **result}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _tool_turn(angle_rad: float, speed_rad_s: float = 1.0) -> dict:
    ok, reason = arbiter_allow(0.0, 0.0, angle_rad)
    if not ok:
        return {"ok": False, "error": f"arbiter_blocked: {reason}"}
    try:
        result = ros.algo_roll(angle_rad, speed_rad_s, 10, 0.05)
        _record_allowed_move()
        return {"ok": True, **result}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _tool_stop() -> dict:
    try:
        _set_vel(0.0, 0.0, 0.0, 0.0)
        ros.stop_robot()
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _tool_ask_about_scene(question: str) -> dict:
    try:
        text = _vlm_describe_image(question)
        return {"ok": True, "text": text}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _tool_look_around(n_frames: int = 8) -> dict:
    n = max(4, min(16, int(n_frames)))
    ok, reason = arbiter_allow(0.0, 0.0, 0.5)
    if not ok:
        return {"ok": False, "error": f"arbiter_blocked: {reason}"}
    try:
        return {"ok": True, **_run_look_around(n)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _tool_recall(query: str, since_seconds: Optional[int] = None, limit: int = 10) -> dict:
    try:
        snap = _get_kg_snapshot()
        if not snap:
            return {"ok": True, "matches": [], "note": "knowledge graph is empty or unavailable"}
        since_ts = (_now() - since_seconds) if since_seconds else 0.0
        q = query.lower()
        matches = []
        for node in snap.get("nodes", []):
            label = (node.get("label") or "").lower()
            if q not in label:
                continue
            last_seen = node.get("last_seen") or 0
            if since_ts and last_seen < since_ts:
                continue
            matches.append({
                "id":         node.get("id"),
                "type":       node.get("type"),
                "label":      node.get("label"),
                "first_seen": node.get("first_seen"),
                "last_seen":  last_seen,
            })
        matches.sort(key=lambda x: x.get("last_seen") or 0, reverse=True)
        return {"ok": True, "matches": matches[:limit], "total_found": len(matches)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


_TOOLS = [
    {"type": "function", "function": {
        "name": "move_forward_back",
        "description": (
            "Drive straight forward or backward by a specific distance. "
            "Use when asked to come closer, back up, or move forward/backward by some amount. "
            "Blocks until the robot arrives. Max ±1.5 m."
        ),
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number",
                           "description": "Meters. Positive = forward, negative = backward."},
            "speed_m_s":  {"type": "number", "description": "Speed in m/s. Default 0.25."},
        }, "required": ["distance_m"]},
    }},
    {"type": "function", "function": {
        "name": "strafe_left_right",
        "description": (
            "Slide sideways without changing heading. "
            "Use when asked to scoot, sidestep, or move laterally. "
            "Blocks until arrival. Max ±1.5 m."
        ),
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number",
                           "description": "Meters. Negative = left, positive = right."},
            "speed_m_s":  {"type": "number", "description": "Speed in m/s. Default 0.25."},
        }, "required": ["distance_m"]},
    }},
    {"type": "function", "function": {
        "name": "turn",
        "description": (
            "Rotate in place to a relative heading. "
            "Use for 'turn left/right', 'spin around', 'face the other way'. "
            "90° = 1.57 rad, 180° = 3.14 rad. Blocks until heading reached."
        ),
        "parameters": {"type": "object", "properties": {
            "angle_rad":    {"type": "number",
                             "description": "Radians. Positive = clockwise, negative = counter-clockwise."},
            "speed_rad_s":  {"type": "number", "description": "Angular speed rad/s. Default 1.0."},
        }, "required": ["angle_rad"]},
    }},
    {"type": "function", "function": {
        "name": "stop",
        "description": "Immediately stop all motion. Use when asked to halt, freeze, or abort.",
        "parameters": {"type": "object", "properties": {}, "required": []},
    }},
    {"type": "function", "function": {
        "name": "ask_about_scene",
        "description": (
            "Take a fresh camera snapshot and answer a visual question using the onboard VLM. "
            "Does NOT move the robot. Use whenever the user wants to know what the camera currently sees. "
            "Takes 3–15 s."
        ),
        "parameters": {"type": "object", "properties": {
            "question": {"type": "string", "description": "The visual question to answer."},
        }, "required": ["question"]},
    }},
    {"type": "function", "function": {
        "name": "look_around",
        "description": (
            "Physically rotate the robot 360° while capturing frames with object detections at each heading. "
            "MOVES THE ROBOT. Only use when the user explicitly wants the robot to rotate and survey its surroundings. "
            "Takes ~12 s."
        ),
        "parameters": {"type": "object", "properties": {
            "n_frames": {"type": "integer", "minimum": 4, "maximum": 16,
                         "description": "Number of frames to capture. Default 8."},
        }, "required": []},
    }},
    {"type": "function", "function": {
        "name": "recall",
        "description": (
            "Search the knowledge graph for entities and events seen in the past. "
            "Use for 'where did you see the cat?', 'have you seen Akash today?', 'what did you observe an hour ago?'. "
            "Results are returned newest-first."
        ),
        "parameters": {"type": "object", "properties": {
            "query":         {"type": "string",
                              "description": "Label or keyword to search for (case-insensitive substring match)."},
            "since_seconds": {"type": "integer",
                              "description": "Only return nodes seen within the last N seconds. Omit for all time."},
            "limit":         {"type": "integer",
                              "description": "Maximum results to return. Default 10."},
        }, "required": ["query"]},
    }},
]

_TOOL_DISPATCH: dict = {
    "move_forward_back":  lambda args: _tool_move_forward_back(**args),
    "strafe_left_right":  lambda args: _tool_strafe_left_right(**args),
    "turn":               lambda args: _tool_turn(**args),
    "stop":               lambda args: _tool_stop(),
    "ask_about_scene":    lambda args: _tool_ask_about_scene(**args),
    "look_around":        lambda args: _tool_look_around(**args),
    "recall":             lambda args: _tool_recall(**args),
}


@app.post("/agent/chat")
def agent_chat(req: ChatReq):
    r = _redis()
    history: list = json.loads(r.get("agent:history") or "[]")

    messages = [{"role": "system", "content": _NOIR_SYSTEM}]
    messages.extend(history[-10:])
    user_content = req.message + " /no_think" if _AGENT_PROVIDER == "mlx" else req.message
    messages.append({"role": "user", "content": user_content})

    chat_url, chat_model, chat_headers = _provider_config()
    print(f"[agent] provider={_AGENT_PROVIDER} model={chat_model}", flush=True)

    def _stream():
        final_reply = ""
        try:
            for _ in range(3):
                resp = requests.post(
                    chat_url, headers=chat_headers,
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
                data    = resp.json()
                msg     = (data.get("choices") or [{}])[0].get("message", {})
                content = (msg.get("content") or "").strip()

                # defensive parse: Qwen3-VL-2B sometimes returns tool calls as text
                tool_calls = msg.get("tool_calls") or []
                if not tool_calls and content.startswith("[{"):
                    try:
                        parsed = json.loads(content)
                        if isinstance(parsed, list) and parsed[0].get("type") == "function":
                            tool_calls = parsed
                    except Exception:
                        pass

                if not tool_calls:
                    final_reply = content
                    break

                messages.append({"role": "assistant", "content": content, "tool_calls": tool_calls})

                for tc in tool_calls:
                    fn_name = (tc.get("function") or {}).get("name", "")
                    raw_args = (tc.get("function") or {}).get("arguments", "{}")
                    tc_id    = tc.get("id", fn_name)

                    try:
                        args = json.loads(raw_args) if isinstance(raw_args, str) else raw_args
                    except Exception:
                        args = {}

                    dispatcher = _TOOL_DISPATCH.get(fn_name)
                    if dispatcher is None:
                        result = {"ok": False, "error": f"unknown_tool: {fn_name}"}
                    else:
                        try:
                            result = dispatcher(args)
                        except Exception as exc:
                            result = {"ok": False, "error": str(exc)}

                    print(f"[agent] tool={fn_name} result={str(result)[:120]}", flush=True)
                    yield f"data: {json.dumps({'type': 'tool_call', 'name': fn_name, 'args': args, 'result': result})}\n\n"

                    messages.append({
                        "role":         "tool",
                        "tool_call_id": tc_id,
                        "content":      json.dumps(result)[:1500],
                    })
            else:
                final_reply = "[noir] hit tool-call limit."
        except Exception as exc:
            final_reply = f"[noir] unreachable: {exc}"

        print(f"[agent] reply: {final_reply[:100]!r}", flush=True)
        yield f"data: {json.dumps({'type': 'reply', 'text': final_reply})}\n\n"

        history.append({"role": "user",      "content": req.message})
        history.append({"role": "assistant", "content": final_reply or "…"})
        r.set("agent:history", json.dumps(history[-20:]), ex=1800)

    return StreamingResponse(_stream(), media_type="text/event-stream",
                             headers={"X-Accel-Buffering": "no"})


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


@app.post("/agent/follow")
def agent_follow(req: FollowReq):
    """Enable or disable autonomous face-following."""
    r = _redis()
    if req.on:
        r.set("agent:follow_cfg", json.dumps({
            "on": True, "target": (req.target_name or "").upper(), "started": _now(),
        }))
    else:
        r.delete("agent:follow_cfg")
        _set_vel(0.0, 0.0, 0.0, 0.0)
        ros.stop_robot()
    return {"ok": True, "on": req.on, "target": req.target_name}


# ── Vision describe (on-demand VLM) ───────────────────────────────────────────

@app.post("/vision/describe")
def vision_describe(req: DescribeReq):
    """Answer a visual question about the current camera frame using the VLM."""
    try:
        text = _vlm_describe_image(req.question)
        return {"text": text}
    except RuntimeError as e:
        raise HTTPException(404, str(e))
    except Exception as e:
        raise HTTPException(503, str(e))


# ── Follow-face loop ───────────────────────────────────────────────────────────

_FOLLOW_HZ          = 5
_FOLLOW_K_YAW       = 1.6
_FOLLOW_K_FWD       = 1.4
_FOLLOW_TARGET_H    = 0.20
_FOLLOW_STALE_S     = 2.5
_SEARCH_ROT_SPEED   = 1.5
_SEARCH_ROT_TICKS   = 3
_SEARCH_PAUSE_TICKS = 2
_search_tick        = 0


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

            x_ts_raw = r.get("xbox:last_input_ts")
            if x_ts_raw and (_now() - float(x_ts_raw)) < 1.5:
                continue

            cfg    = json.loads(cfg_raw)
            target = cfg.get("target", "")

            face_raw = r.get("face:latest")
            fd       = json.loads(face_raw) if face_raw else {}
            stale    = (_now() - float(fd.get("ts", 0))) > _FOLLOW_STALE_S if fd else True

            faces = fd.get("faces") or []
            pick  = next((f for f in faces if f.get("name") == target), None) if target \
                    else (faces[0] if faces else None)

            if not pick or stale:
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

            _search_tick = 0

            x1, y1, x2, y2 = pick["bbox"]
            fw      = float(fd.get("frame_w") or 1)
            fh      = float(fd.get("frame_h") or 1)
            cx      = (x1 + x2) / 2.0
            err_x   = (cx - fw / 2.0) / (fw / 2.0)
            box_h_f = (y2 - y1) / fh
            yaw_cmd = max(-1.5, min(1.5, -_FOLLOW_K_YAW * err_x))
            fwd_cmd = max(-0.4, min(0.4,  _FOLLOW_K_FWD * (_FOLLOW_TARGET_H - box_h_f)))

            ok, _ = arbiter_allow(0.0, fwd_cmd, yaw_cmd)
            if not ok:
                continue
            hold_s = dt + 0.1
            _set_vel(0.0, fwd_cmd, yaw_cmd, hold_s)
            ros.publish_twist(0.0, fwd_cmd, yaw_cmd)
            _record_allowed_move()
        except Exception:
            pass


# ── Navigation ─────────────────────────────────────────────────────────────────

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
    """Latest VLM scene caption from kg_builder (TTL 30s)."""
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


# ── Safety state ───────────────────────────────────────────────────────────────

@app.get("/safety/state")
def safety_state():
    r   = _redis()
    now = _now()
    try:
        last_xbox_raw = r.get("xbox:last_input_ts")
        last_xbox = float(last_xbox_raw) if last_xbox_raw else None
        with _allowed_move_lock:
            last_move = _last_allowed_move_ts
        return {
            "xbox_active":              bool(last_xbox and (now - last_xbox) < 2.0),
            "xbox_last_input_age_s":    round(now - last_xbox, 2) if last_xbox else None,
            "last_allowed_move_age_s":  round(now - last_move, 2) if last_move else None,
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
                "ts":       float(f.get("ts", 0)),
                "label":    f.get("label", ""),
                "conf":     float(f.get("conf", 0)),
                "thumb_id": f.get("thumb_id"),
            }
            for _, f in raw
        ]}
    except redis_lib.RedisError:
        return {"events": []}


# ── Knowledge Graph ────────────────────────────────────────────────────────────

def _get_kg_snapshot() -> dict:
    try:
        raw = _redis().get("kg:snapshot")
        return json.loads(raw) if raw else {}
    except Exception:
        return {}


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
        "nodes":  nodes,
        "edges":  snap.get("edges", []),
        "ts":     snap.get("ts", 0),
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

    snap      = _get_kg_snapshot()
    day_start = datetime.datetime.combine(d, datetime.time.min).timestamp()
    day_end   = datetime.datetime.combine(d + datetime.timedelta(days=1), datetime.time.min).timestamp()

    day_nodes = [n for n in snap.get("nodes", [])
                 if day_start <= (n.get("first_seen") or 0) < day_end]
    if not day_nodes:
        return {"date": date, "text": f"Nothing was recorded on {date}.", "cached": False}

    observations = ", ".join(f"{n['type']} '{n['label']}'" for n in day_nodes[:20])
    prompt = (
        f"You are NOIR, a compact wheeled robot. "
        f"Write a diary entry for {date} in 4-6 sentences, first person, dry noir voice. "
        f"Observations: {observations}. "
        f"Diary text only — no labels or headers. /no_think"
    )
    url, model, headers = _provider_config()
    try:
        resp = requests.post(url, headers=headers, json={
            "model":       model,
            "messages":    [{"role": "user", "content": prompt}],
            "temperature": 0.7,
            "max_tokens":  300,
            "stream":      False,
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
