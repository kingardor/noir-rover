import os
import threading
import time
from typing import Optional

from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel, Field
import roslibpy

CMD_VEL_TOPIC = "/cmd_vel"
ROSBRIDGE_HOST = os.getenv("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.getenv("ROSBRIDGE_PORT", "9090"))

# --- ROS Bridge Client Class ---
class RosBridgeClient:
    def __init__(self, host: str, port: int, cmd_vel_topic: str):
        self.host = host
        self.port = port
        self.cmd_vel_topic = cmd_vel_topic
        self._ros = roslibpy.Ros(host=self.host, port=self.port)
        self._cmd_pub = roslibpy.Topic(self._ros, self.cmd_vel_topic, "geometry_msgs/Twist")
        self._connection_started = False

    def connect(self):
        if not self._connection_started:
            threading.Thread(target=self._ros.run, daemon=True).start()
            self._connection_started = True

    @property
    def is_connected(self):
        return self._ros.is_connected

    def publish_twist(self, x=0.0, y=0.0, yaw=0.0):
        if not self.is_connected:
            print("[API] Error: Cannot publish twist, not connected to rosbridge.")
            return
        # WORKAROUND: Swap X and Y for Moorebot Scout mecanum wheels
        msg = {"linear": {"x": float(y), "y": float(x)}, "angular": {"z": float(yaw)}}
        self._cmd_pub.publish(roslibpy.Message(msg))

    def stop_robot(self):
        self.publish_twist()

# --- Robot Controller Class ---
class RobotController:
    def __init__(self, ros_client: RosBridgeClient):
        self.ros_client = ros_client

    def move(self, x: float, y: float, yaw: float, duration_ms: Optional[int], background_tasks: BackgroundTasks):
        if not self.ros_client.is_connected:
            raise HTTPException(503, "Cannot control movement: Not connected to ROS.")
        self.ros_client.publish_twist(x, y, yaw)
        if duration_ms:
            def _delayed_stop(ms: int):
                time.sleep(ms / 1000.0)
                self.ros_client.stop_robot()
            background_tasks.add_task(_delayed_stop, duration_ms)

    def stop(self):
        self.ros_client.stop_robot()

# --- Pydantic Models for Input Validation ---
class Vel(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    duration_ms: Optional[int] = None

class StreamStartRequest(BaseModel):
    url: str = Field("rtmp://localhost:1935/live/scout", description="The destination RTMP server URL.")

# --- FastAPI App and Dependency Injection ---
app = FastAPI(title="Moorebot Scout Bridge")

ros_client = RosBridgeClient(ROSBRIDGE_HOST, ROSBRIDGE_PORT, CMD_VEL_TOPIC)
robot_controller = RobotController(ros_client)

@app.on_event("startup")
def startup_event():
    print("[API] Starting ROS connection...")
    ros_client.connect()

@app.get("/health")
def health():
    return {"ok": True, "ros_connected": ros_client.is_connected}

@app.post("/robot/stop")
def stop():
    robot_controller.stop()
    return {"ok": True}

@app.post("/robot/move")
def move(v: Vel, background_tasks: BackgroundTasks):
    robot_controller.move(v.x, v.y, v.yaw, v.duration_ms, background_tasks)
    return {"ok": True}
