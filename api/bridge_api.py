#!/usr/bin/env python3

import os
import time
import threading
import base64
from typing import Optional
from fastapi import FastAPI, HTTPException, BackgroundTasks, Response
from pydantic import BaseModel
import roslibpy

# Configuration
CMD_VEL_TOPIC = "/cmd_vel"
CAMERA_TOPIC = "/CoreNode/jpg"
ROSBRIDGE_HOST = os.getenv("ROSBRIDGE_HOST", "localhost") 
ROSBRIDGE_PORT = int(os.getenv("ROSBRIDGE_PORT", "9090"))

camera_data = []
camera_lock = threading.Lock()

class Vel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    duration_ms: Optional[int] = None

class ScoutRosBridgeClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self._ros = roslibpy.Ros(host=self.host, port=self.port)
        
        # Movement publisher
        self._cmd_pub = roslibpy.Topic(self._ros, CMD_VEL_TOPIC, "geometry_msgs/Twist")
        
        # Camera subscriber (JPG frames)
        self._camera_sub = roslibpy.Topic(self._ros, CAMERA_TOPIC, "roller_eye/frame")
        self._camera_sub.subscribe(self._on_camera_frame)
        
    def _on_camera_frame(self, message):
        """Handle incoming JPG frames"""
        with camera_lock:
            try:
                # Extract JPG data from message
                data = message.get('data', [])
                if isinstance(data, str):
                    # If base64 encoded string
                    jpg_bytes = base64.b64decode(data)
                else:
                    # If byte array
                    jpg_bytes = bytes(data)
                
                # Store latest frame (keep only latest)
                camera_data.clear()
                camera_data.append(jpg_bytes)
                
            except Exception as e:
                print(f"[CAMERA] Error processing frame: {e}")
        
    def connect(self):
        """Start ROS connection (non-blocking)"""
        print(f"[SCOUT] Connecting to ROS bridge at {self.host}:{self.port}")
        
        def on_ready():
            print("[SCOUT] ROS bridge connected!")
            
        self._ros.on_ready(on_ready)
        self._ros.run()
        
    @property
    def is_connected(self):
        return self._ros.is_connected
        
    def publish_twist(self, x=0.0, y=0.0, yaw=0.0):
        """Publish movement command (Go-style mapping)"""
        if not self.is_connected:
            return False
            
        # Go mapping: Linear.X = strafe, Linear.Y = forward, Angular.Z = rotation
        msg = {
            "linear": {"x": float(y), "y": float(x), "z": 0.0},  # Swap X/Y for Scout
            "angular": {"x": 0.0, "y": 0.0, "z": float(yaw)}
        }
        
        self._cmd_pub.publish(roslibpy.Message(msg))
        print(f"[SCOUT] Move: forward={x}, strafe={y}, rotate={yaw}")
        return True
        
    def stop_robot(self):
        return self.publish_twist()

app = FastAPI(
    title="Moorebot Scout API",
    description="Movement Control + JPG Camera + RTMP Streaming",
    version="2.0.0"
)

ros_client = ScoutRosBridgeClient(ROSBRIDGE_HOST, ROSBRIDGE_PORT)

@app.on_event("startup")
def startup_event():
    """Initialize ROS connection"""
    print("[API] Starting Scout API with JPG camera support...")
    
    def connect_ros():
        ros_client.connect()
    
    ros_thread = threading.Thread(target=connect_ros, daemon=True)
    ros_thread.start()
    print("[API] ROS connection started")

@app.get("/health")
def health():
    with camera_lock:
        has_frames = len(camera_data) > 0
    
    return {
        "ok": True,
        "ros_connected": ros_client.is_connected,
        "camera_active": has_frames
    }

@app.post("/robot/stop")
def stop_robot():
    if not ros_client.stop_robot():
        raise HTTPException(503, "Failed to stop robot")
    return {"ok": True, "message": "Robot stopped"}

@app.post("/robot/move")
def move_robot(v: Vel, background_tasks: BackgroundTasks):
    if not ros_client.is_connected:
        raise HTTPException(503, "Not connected to ROS")
        
    success = ros_client.publish_twist(v.x, v.y, v.yaw)
    if not success:
        raise HTTPException(503, "Failed to publish movement")
    
    if v.duration_ms:
        def delayed_stop():
            time.sleep(v.duration_ms / 1000.0)
            ros_client.stop_robot()
        background_tasks.add_task(delayed_stop)
    
    return {
        "ok": True,
        "message": f"Moving: forward={v.x}, strafe={v.y}, rotate={v.yaw}",
        "duration_ms": v.duration_ms
    }

@app.get("/camera/status")
def camera_status():
    """Camera status info"""
    with camera_lock:
        has_frames = len(camera_data) > 0
        frame_size = len(camera_data[0]) if has_frames else 0
    
    return {
        "available": has_frames,
        "ros_connected": ros_client.is_connected,
        "topic": CAMERA_TOPIC,
        "format": "JPG",
        "latest_frame_size_bytes": frame_size
    }

@app.get("/camera/frame")
def get_camera_frame():
    """Get latest JPG frame as binary response"""
    with camera_lock:
        if not camera_data:
            raise HTTPException(404, "No camera frame available")
        
        jpg_bytes = camera_data[0]
        return Response(content=jpg_bytes, media_type="image/jpeg")