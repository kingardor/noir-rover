#!/usr/bin/env python3
from typing import Optional
import time
import base64
import threading

import rospy
from rostopic import get_topic_class
from geometry_msgs.msg import Twist

CMD_VEL_TOPIC = "/cmd_vel"
CAMERA_TOPIC  = "/CoreNode/jpg"

_camera_data = []
_camera_lock = threading.Lock()
_last_frame_ts: Optional[float] = None


class ScoutROS:
    """
    Minimal ROS wrapper used by both FastAPI and local controllers.
    - publish_twist(x, y, rotate)
    - stop_robot()
    - is_connected (property)
    - get_latest_frame() -> Optional[bytes]
    """
    def __init__(self, node_name: str = "scout_api"):
        self.node_name = node_name
        self._cmd_pub: Optional[rospy.Publisher] = None
        self._camera_sub: Optional[rospy.Subscriber] = None
        self._inited = False
        self._pub_lock = threading.Lock()

    def init(self):
        if self._inited:
            return
        rospy.init_node(self.node_name, anonymous=True, disable_signals=True)
        rospy.loginfo("[SCOUT] rospy node initialized")

        self._cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        rospy.loginfo(f"[SCOUT] Publisher ready on {CMD_VEL_TOPIC} [geometry_msgs/Twist]")

        self._subscribe_camera()
        self._inited = True

        # background: resubscribe if topic type appears later
        def _resub():
            while not rospy.is_shutdown():
                if self._camera_sub is None and self.is_connected:
                    try:
                        self._subscribe_camera()
                    except Exception:
                        pass
                time.sleep(2.0)
        threading.Thread(target=_resub, daemon=True).start()

    @staticmethod
    def _parse_frame_msg(msg) -> Optional[bytes]:
        data = getattr(msg, "data", None)
        if data is None:
            return None
        if isinstance(data, (bytes, bytearray)):
            return bytes(data)
        if isinstance(data, list) and data and isinstance(data[0], int):
            return bytes(bytearray(data))
        if isinstance(data, str):
            try:
                return base64.b64decode(data)
            except Exception:
                return data.encode("utf-8", errors="ignore")
        return None

    def _on_camera_frame(self, msg):
        global _last_frame_ts
        try:
            jpg = self._parse_frame_msg(msg)
            if jpg:
                with _camera_lock:
                    _camera_data.clear()
                    _camera_data.append(jpg)
                    _last_frame_ts = time.time()
        except Exception as e:
            rospy.logwarn(f"[CAMERA] Error processing frame: {e}")

    def _subscribe_camera(self):
        msg_class, real_topic, _ = get_topic_class(CAMERA_TOPIC, blocking=False)
        if msg_class is None:
            rospy.logwarn(f"[SCOUT] Msg type for {CAMERA_TOPIC} not yet available.")
            return
        self._camera_sub = rospy.Subscriber(real_topic, msg_class, self._on_camera_frame, queue_size=1)
        rospy.loginfo(f"[SCOUT] Subscribed to {real_topic} [{msg_class._type}]")

    @property
    def is_connected(self) -> bool:
        if not self._inited or rospy.is_shutdown():
            return False
        try:
            rospy.get_published_topics()
            return True
        except Exception:
            return False

    def publish_twist(self, x: float = 0.0, y: float = 0.0, rotate: float = 0.0) -> bool:
        if not self._cmd_pub or not self.is_connected:
            return False
        t = Twist()
        # mapping: linear.x=strafe (x), linear.y=forward (y), angular.z=rotate
        t.linear.x = float(x)
        t.linear.y = float(y)
        t.angular.z = float(rotate)
        with self._pub_lock:
            self._cmd_pub.publish(t)
        return True

    def stop_robot(self) -> bool:
        return self.publish_twist(0.0, 0.0, 0.0)

    def get_latest_frame(self) -> Optional[bytes]:
        with _camera_lock:
            return _camera_data[0] if _camera_data else None

    def camera_status(self):
        with _camera_lock:
            has = len(_camera_data) > 0
            size = len(_camera_data[0]) if has else 0
        age = None if _last_frame_ts is None else round(time.time() - _last_frame_ts, 3)
        return has, size, age
