#!/usr/bin/env python3
import asyncio
import base64
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
_OLLAMA_URL       = os.environ.get("OLLAMA_URL",       "http://localhost:11434")  # kept for reference
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

_NOIR_SYSTEM = (
    "You are NOIR, an AI living inside a small wheeled robot. You ARE the robot — not an observer of it. "
    "Always speak in first person: 'I see', 'I moved', 'I found', 'I can hear'. "
    "Be direct, a little dry, and easy to talk to. Keep every reply to 1-2 sentences.\n\n"
    "When asked to do something physical or observe the world, call the right tools. "
    "Tools describe themselves — use your judgment. Call tools in the right order for multi-step requests.\n\n"
    "After tool use, describe what you did or found naturally and briefly, as yourself. "
    "No asterisks, parentheses, or announcing what you're about to do."
)

_TOOLS = [
    {"type": "function", "function": {
        "name": "stop",
        "description": "Halt all motion immediately.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "move",
        "description": (
            "Drives the robot in a straight line. "
            "forward_m: travel forward (positive) or backward (negative), max ±0.6 m. "
            "strafe_m: slide right (positive) or left (negative), max ±0.4 m. "
            "Use the separate rotate tool for turning — do not combine move and rotate in the same request."
        ),
        "parameters": {"type": "object", "properties": {
            "forward_m": {"type": "number"},
            "strafe_m":  {"type": "number"},
        }}}},
    {"type": "function", "function": {
        "name": "rotate",
        "description": (
            "Turns the robot in place. "
            "rotate_deg: degrees to turn — positive = clockwise/right, negative = counter-clockwise/left. Max ±180°."
        ),
        "parameters": {"type": "object", "properties": {
            "rotate_deg": {"type": "number"},
        }, "required": ["rotate_deg"]}}},
    {"type": "function", "function": {
        "name": "look_around",
        "description": "Rotates the robot slowly through a full 360° while capturing frames, giving a complete panoramic survey of the surroundings.",
        "parameters": {"type": "object", "properties": {
            "n": {"type": "integer", "description": "Number of frames to capture (4–16)"}}}}},
    {"type": "function", "function": {
        "name": "describe_scene",
        "description": "Returns the most recent cached description of what the robot's camera sees. May be a few seconds old.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "list_objects",
        "description": "Returns the objects currently detected in the robot's camera view, with confidence scores.",
        "parameters": {"type": "object", "properties": {
            "top_k": {"type": "integer", "description": "Maximum number of objects to return"}}}}},
    {"type": "function", "function": {
        "name": "who_is_here",
        "description": "Returns the names and confidence scores of people currently recognized by the robot's face recognition system.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "set_follow_mode",
        "description": "Enables or disables autonomous face-following. When on, the robot continuously tracks and approaches the named person.",
        "parameters": {"type": "object", "properties": {
            "on":          {"type": "boolean", "description": "true to start following, false to stop"},
            "target_name": {"type": "string",  "description": "Name of the person to follow (must be a recognized face)"},
        }, "required": ["on"]}}},
    {"type": "function", "function": {
        "name": "capture_and_describe",
        "description": (
            "Captures a live camera frame and answers any visual question about it using the robot's vision intelligence. "
            "This can identify specific objects ('is there a bottle?'), colors, positions, text, and scene details that "
            "list_objects cannot — list_objects only returns labeled bounding boxes from a fixed detector. "
            "Always use this after moving when you need to inspect something specific."
        ),
        "parameters": {"type": "object", "properties": {
            "question": {"type": "string", "description": "The visual question to answer about the current frame"},
        }, "required": ["question"]}}},
]


def _tool_stop() -> dict:
    _set_vel(0.0, 0.0, 0.0, 0.0)
    ros.stop_robot()
    return {"ok": True}


def _tool_move(forward_m: float = 0.0, strafe_m: float = 0.0) -> dict:
    forward_m = max(-0.6, min(0.6, float(forward_m)))
    strafe_m  = max(-0.4, min(0.4, float(strafe_m)))
    out: dict = {"forward_m": forward_m, "strafe_m": strafe_m}
    if abs(forward_m) > 1e-3 or abs(strafe_m) > 1e-3:
        # Axis mapping: algo_move(x_dist=strafe, y_dist=forward)
        out["move"] = ros.algo_move(strafe_m, forward_m, 0.3)
    return out


def _tool_rotate(rotate_deg: float) -> dict:
    rotate_deg = max(-180.0, min(180.0, float(rotate_deg)))
    if abs(rotate_deg) < 1e-3:
        return {"rotate_deg": 0, "ok": True}
    # Use Twist-based timing — same mechanism as the controller (algo_roll is unreliable).
    rot_speed = 3.0  # rad/s
    duration_s = abs(math.radians(rotate_deg)) / rot_speed
    # Scout firmware: positive angular.z = left/CCW — negate so positive rotate_deg = right/CW
    direction = -math.copysign(1.0, rotate_deg)
    _set_vel(0.0, 0.0, direction * rot_speed, duration_s)
    ros.publish_twist(0.0, 0.0, direction * rot_speed)
    time.sleep(duration_s + 0.15)  # wait for hold to expire
    return {"rotate_deg": rotate_deg, "ok": True}


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


_TOOL_DISPATCH = {
    "stop":                 _tool_stop,
    "move":                 _tool_move,
    "rotate":               _tool_rotate,
    "look_around":          _tool_look_around,
    "describe_scene":       _tool_describe_scene,
    "list_objects":         _tool_list_objects,
    "who_is_here":          _tool_who_is_here,
    "set_follow_mode":      _tool_set_follow_mode,
    "capture_and_describe": _tool_capture_and_describe,
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
    messages.append({"role": "user", "content": req.message})

    chat_url, chat_model, chat_headers = _provider_config()
    print(f"[agent] provider={_AGENT_PROVIDER} model={chat_model}", flush=True)

    tool_log: list = []
    reply = ""
    for _ in range(4):
        try:
            resp = requests.post(
                chat_url,
                headers=chat_headers,
                json={
                    "model":       chat_model,
                    "messages":    messages,
                    "tools":       _TOOLS,
                    "tool_choice": "auto",
                    "temperature": 0.65,
                    "stream":      False,
                },
                timeout=60,
            )
            resp.raise_for_status()
            data = resp.json()
        except Exception as exc:
            reply = f"[noir] unreachable: {exc}"
            break

        msg   = (data.get("choices") or [{}])[0].get("message", {})
        calls = msg.get("tool_calls") or []

        if not calls:
            reply = (msg.get("content") or "").strip()
            print(f"[agent] reply: {reply!r}", flush=True)
            messages.append({"role": "assistant", "content": reply})
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
            tool_log.append({"name": fn, "args": args, "result": result})
            messages.append({
                "role":         "tool",
                "tool_call_id": tc_id,
                "content":      json.dumps(result)[:1500],
            })
    else:
        if not reply:
            reply = "[noir] hit iteration limit — try again"

    history.append({"role": "user",      "content": req.message})
    history.append({"role": "assistant", "content": reply or "…"})
    r.set("agent:history", json.dumps(history[-20:]), ex=1800)
    return {"reply": reply, "tool_calls": tool_log}


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
_FOLLOW_TARGET_H = 0.30   # target bbox height as fraction of frame
_FOLLOW_STALE_S  = 2.5    # give up if face:latest is older than this


def _follow_loop():
    dt = 1.0 / _FOLLOW_HZ
    while True:
        time.sleep(dt)
        try:
            r = _redis()
            cfg_raw = r.get("agent:follow_cfg")
            if not cfg_raw:
                continue

            # Yield to controller: if a stick moved in the last 1.5 s, step aside
            x_ts_raw = r.get("xbox:last_input_ts")
            if x_ts_raw and (_now() - float(x_ts_raw)) < 1.5:
                continue

            cfg    = json.loads(cfg_raw)
            target = cfg.get("target", "")

            face_raw = r.get("face:latest")
            if not face_raw:
                _set_vel(0.0, 0.0, 0.0, 0.0)
                continue

            fd = json.loads(face_raw)
            if (_now() - float(fd.get("ts", 0))) > _FOLLOW_STALE_S:
                _set_vel(0.0, 0.0, 0.0, 0.0)
                continue

            faces = fd.get("faces") or []
            pick  = next((f for f in faces if f.get("name") == target), None) if target \
                    else (faces[0] if faces else None)
            if not pick:
                _set_vel(0.0, 0.0, 0.0, 0.0)
                continue

            x1, y1, x2, y2 = pick["bbox"]
            fw = float(fd.get("frame_w") or 1)
            fh = float(fd.get("frame_h") or 1)

            cx      = (x1 + x2) / 2.0
            err_x   = (cx - fw / 2.0) / (fw / 2.0)                          # -1..1, +ve = right
            box_h_f = (y2 - y1) / fh
            yaw_cmd = max(-1.5, min(1.5, _FOLLOW_K_YAW * err_x))
            fwd_cmd = max(-0.4, min(0.4, _FOLLOW_K_FWD * (_FOLLOW_TARGET_H - box_h_f)))

            ok, _ = arbiter_allow(0.0, fwd_cmd, yaw_cmd)
            if not ok:
                continue

            hold_s = dt + 0.1   # slightly longer than loop period → no gap between pulses
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
