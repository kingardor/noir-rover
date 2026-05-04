#!/usr/bin/env python3
import asyncio
import base64
import json
import math
import threading
import time
from typing import Optional

import redis as redis_lib
from fastapi import FastAPI, HTTPException, Query, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from scoutros import ScoutROS, CMD_VEL_TOPIC, CAMERA_TOPIC

# ── Redis ─────────────────────────────────────────────────────────────────────

_r: Optional[redis_lib.Redis] = None


def _redis() -> redis_lib.Redis:
    global _r
    if _r is None:
        _r = redis_lib.Redis(host="localhost", port=6379, decode_responses=True)
    return _r


def _now() -> float:
    return time.time()


# ── Request models ────────────────────────────────────────────────────────────

class Vel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    rotate: float = 0.0
    duration_ms: Optional[int] = None
    source: str = "manual"  # "manual" | "agent" | "xbox"


class AlgoAction(BaseModel):
    x_speed: float = 0.0
    y_speed: float = 0.0
    rotated_speed: float = 0.0
    duration_ms: int = 1000  # milliseconds — passed directly to UtilNode/algo_action `time` field
    source: str = "agent"


class AlgoMove(BaseModel):
    x_dist: float = 0.0
    y_dist: float = 0.0
    speed: float = 0.3
    source: str = "agent"


class AlgoRoll(BaseModel):
    angle_rad: float
    speed_rad_s: float = 1.0
    timeout_s: int = 10
    error_rad: float = 0.05
    source: str = "agent"


class PatrolStart(BaseModel):
    name: str
    from_start: bool = True


class PathSave(BaseModel):
    name: str


class MissionStart(BaseModel):
    mode: str   # "voice" | "follow" | "patrol"
    goal: Optional[str] = None


class SpeakRequest(BaseModel):
    text: str


# ── Arbiter ───────────────────────────────────────────────────────────────────

_last_allowed_move_ts: Optional[float] = None
_allowed_move_lock = threading.Lock()


def _record_allowed_move():
    global _last_allowed_move_ts
    with _allowed_move_lock:
        _last_allowed_move_ts = _now()


def arbiter_allow(x: float, y: float, rotate: float, source: str) -> tuple:
    """Return (True, 'ok') or (False, reason). Xbox source bypasses Xbox gate."""
    if source == "agent":
        try:
            r = _redis()
            last_xbox = r.get("xbox:last_input_ts")
            if last_xbox and (_now() - float(last_xbox)) < 2.0:
                return False, "xbox_active"
            last_hb = r.get("agent:heartbeat_ts")
            if not last_hb or (_now() - float(last_hb)) > 1.5:
                return False, "heartbeat_stale"
        except redis_lib.RedisError:
            return False, "redis_unavailable"
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


# ── Watchdog thread ───────────────────────────────────────────────────────────

def _watchdog():
    """Stop robot within 500 ms if mission is active but no move has been allowed recently."""
    while True:
        time.sleep(0.1)
        try:
            mission = _redis().get("mission:active") or "idle"
            if mission == "idle":
                continue
            with _allowed_move_lock:
                last_ts = _last_allowed_move_ts
            if last_ts is None or (_now() - last_ts) > 0.5:
                ros.stop_robot()
        except Exception:
            pass


# ── App ───────────────────────────────────────────────────────────────────────

app = FastAPI(
    title="Moorebot Scout API",
    description="Motion + Perception + Nav + Mission bridge (ScoutROS / rospy)",
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
    threading.Thread(target=_watchdog, daemon=True).start()
    threading.Thread(target=_vel_hold_loop, daemon=True).start()


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
    ok, reason = arbiter_allow(v.x, v.y, v.rotate, v.source)
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
    ok, reason = arbiter_allow(body.x_speed, body.y_speed, body.rotated_speed, body.source)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    _record_allowed_move()
    result = ros.algo_action(body.x_speed, body.y_speed, body.rotated_speed, body.duration_ms)
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "algo_action failed"))
    return result


@app.post("/move/distance")
def move_distance(body: AlgoMove):
    ok, reason = arbiter_allow(body.x_dist, body.y_dist, 0.0, body.source)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    _record_allowed_move()
    result = ros.algo_move(body.x_dist, body.y_dist, abs(body.speed))
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "algo_move failed"))
    return result


@app.post("/move/rotate")
def move_rotate(body: AlgoRoll):
    ok, reason = arbiter_allow(0.0, 0.0, body.angle_rad, body.source)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")
    _record_allowed_move()
    result = ros.algo_roll(body.angle_rad, body.speed_rad_s, body.timeout_s, body.error_rad)
    if not result.get("ok"):
        raise HTTPException(503, result.get("error", "algo_roll failed"))
    return result


# ── Look around ───────────────────────────────────────────────────────────────

@app.post("/look_around")
def look_around(
    n_frames: int = Query(default=8, ge=4, le=16),
    source: str = Query(default="agent"),
):
    """Slow 360° rotation capturing N evenly-spaced frames with available detections."""
    rotation_speed = 0.5   # rad/s
    total_time = (2 * math.pi) / rotation_speed   # ~12.6 s
    interval = total_time / n_frames

    ok, reason = arbiter_allow(0.0, 0.0, rotation_speed, source)
    if not ok:
        raise HTTPException(403, f"Arbiter blocked: {reason}")

    results = []
    # Start rotating via service call (duration covers one inter-frame interval + buffer)
    slot_ms = int((math.ceil(interval) + 1) * 1000)
    ros.algo_action(0.0, 0.0, rotation_speed, slot_ms)
    _record_allowed_move()

    try:
        r = _redis()
        for i in range(n_frames):
            heading_deg = round((i / n_frames) * 360.0, 1)

            # Xbox preemption check
            try:
                lx = r.get("xbox:last_input_ts")
                if lx and (_now() - float(lx)) < 2.0 and source == "agent":
                    raise HTTPException(403, "Xbox preempted look_around")
            except HTTPException:
                raise
            except Exception:
                pass

            time.sleep(interval)
            _record_allowed_move()

            # Keep rotating by refreshing the action (last frame doesn't need refresh)
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


# ── Agent / Mission ───────────────────────────────────────────────────────────

@app.post("/agent/heartbeat")
def agent_heartbeat():
    """Agent calls this ≥ 1 Hz to keep the arbiter's heartbeat gate open."""
    try:
        _redis().set("agent:heartbeat_ts", str(_now()), ex=5)
    except redis_lib.RedisError as e:
        raise HTTPException(503, f"Redis unavailable: {e}")
    return {"ok": True, "ts": _now()}


@app.post("/mission/start")
def mission_start(body: MissionStart):
    r = _redis()
    try:
        acquired = r.set("mission:lock", "1", nx=True, ex=5)
        if not acquired:
            return {"ok": False, "reason": "another_mission_active"}
        r.set("mission:active", body.mode)
        if body.goal:
            r.set("mission:goal", body.goal, ex=3600)
        return {"ok": True, "mode": body.mode}
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


@app.post("/mission/stop")
def mission_stop():
    r = _redis()
    try:
        r.delete("mission:lock")
        r.set("mission:active", "idle")
        ros.stop_robot()
        return {"ok": True}
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


@app.get("/mission/state")
def mission_state():
    r = _redis()
    try:
        return {
            "active": r.get("mission:active") or "idle",
            "goal": r.get("mission:goal"),
            "lock": r.exists("mission:lock") == 1,
        }
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


# ── Audio ─────────────────────────────────────────────────────────────────────

@app.post("/audio/speak")
def audio_speak(body: SpeakRequest):
    """Push text to TTS queue consumed by audio/tts.py."""
    try:
        _redis().rpush("tts:queue", body.text)
        return {"ok": True, "text": body.text}
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


@app.post("/stt/start")
def stt_start():
    """Signal stt.py to begin recording."""
    try:
        _redis().publish("stt:control", "start")
        return {"ok": True}
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


@app.post("/stt/stop")
def stt_stop():
    """Signal stt.py to stop recording and transcribe."""
    try:
        _redis().publish("stt:control", "stop")
        return {"ok": True}
    except redis_lib.RedisError as e:
        raise HTTPException(503, str(e))


# ── Safety state ──────────────────────────────────────────────────────────────

@app.get("/safety/state")
def safety_state():
    r = _redis()
    now = _now()
    try:
        last_xbox_raw = r.get("xbox:last_input_ts")
        last_hb_raw = r.get("agent:heartbeat_ts")
        last_xbox = float(last_xbox_raw) if last_xbox_raw else None
        last_hb = float(last_hb_raw) if last_hb_raw else None
        with _allowed_move_lock:
            last_move = _last_allowed_move_ts
        return {
            "xbox_active": bool(last_xbox and (now - last_xbox) < 2.0),
            "xbox_last_input_age_s": round(now - last_xbox, 2) if last_xbox else None,
            "heartbeat_ok": bool(last_hb and (now - last_hb) < 1.5),
            "heartbeat_age_s": round(now - last_hb, 2) if last_hb else None,
            "last_allowed_move_age_s": round(now - last_move, 2) if last_move else None,
            "mission": r.get("mission:active") or "idle",
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
