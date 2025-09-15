#!/usr/bin/env python3
import time
from typing import Optional

from fastapi import FastAPI, HTTPException, BackgroundTasks, Response
from pydantic import BaseModel

from scoutros import ScoutROS, CMD_VEL_TOPIC, CAMERA_TOPIC

class Vel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    rotate: float = 0.0
    duration_ms: Optional[int] = None

app = FastAPI(
    title="Moorebot Scout API",
    description="Movement Control + JPG Camera (rospy via ScoutROS)",
    version="3.1.0"
)

ros = ScoutROS(node_name="scout_api")

@app.on_event("startup")
def startup():
    ros.init()

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

@app.post("/robot/stop")
def stop_robot():
    if not ros.stop_robot():
        raise HTTPException(503, "Failed to stop robot")
    return {"ok": True, "message": "Robot stopped"}

@app.post("/robot/move")
def move(v: Vel, background_tasks: BackgroundTasks):
    if not ros.is_connected:
        raise HTTPException(503, "Not connected to ROS master")
    if not ros.publish_twist(v.x, v.y, v.rotate):
        raise HTTPException(503, "Failed to publish movement")
    if v.duration_ms:
        def delayed():
            time.sleep(max(0.0, v.duration_ms) / 1000.0)
            ros.stop_robot()
        background_tasks.add_task(delayed)
    return {"ok": True, "message": f"Moving fwd={v.y} strafe={v.x} rot={v.rotate}", "duration_ms": v.duration_ms}

@app.get("/camera/frame")
def frame():
    jpg = ros.get_latest_frame()
    if not jpg:
        raise HTTPException(404, "No camera frame available")
    return Response(content=jpg, media_type="image/jpeg")
