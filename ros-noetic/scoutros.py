#!/usr/bin/env python3
from typing import Optional
import os
import time
import base64
import threading

import rospy
from rostopic import get_topic_class
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from nav_msgs.msg import Odometry

CMD_VEL_TOPIC   = "/cmd_vel"
CAMERA_TOPIC    = "/CoreNode/jpg"
TOF_TOPIC       = "/SensorNode/tof"
IMU_TOPIC       = "/SensorNode/imu"
VIO_ODOM_TOPIC  = "/MotorNode/vio_odom_relative"
BATTERY_TOPIC   = "/SensorNode/simple_battery_status"
DETECT_TOPIC    = "/CoreNode/obj"


_camera_data  = []
_camera_lock  = threading.Lock()
_last_frame_ts: Optional[float] = None

_tof_range: Optional[float]  = None
_imu_data:  Optional[dict]   = None
_vio_odom:  Optional[dict]   = None
_battery:   Optional[list]   = None
_sensor_lock = threading.Lock()


class ScoutROS:
    """ROS wrapper used by the FastAPI bridge."""

    def __init__(self, node_name: str = "scout_api"):
        self.node_name = node_name
        self._cmd_vel_pub: Optional[rospy.Publisher] = None
        self._camera_sub  = None
        self._tof_sub     = None
        self._imu_sub     = None
        self._vio_sub     = None
        self._battery_sub = None
        self._inited = False
        self._algo_action_svc = None
        self._svc_lock = threading.Lock()

    def init(self):
        if self._inited:
            return
        rospy.init_node(self.node_name, anonymous=False, disable_signals=True)
        rospy.loginfo("[SCOUT] rospy node initialized")

        self._cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        rospy.loginfo(f"[SCOUT] Publisher ready → {CMD_VEL_TOPIC}")

        self._subscribe_camera()
        self._subscribe_sensors()
        self._inited = True

        def _resub():
            _last_pub_ok = time.time()
            while not rospy.is_shutdown():
                time.sleep(2.0)
                if not self.is_connected:
                    continue
                if self._camera_sub is None:
                    try:
                        self._subscribe_camera()
                    except Exception:
                        pass
                # If MotorNode dropped its /cmd_vel subscription (bridge restart),
                # recreate the publisher so ROS master sends a publisherUpdate to MotorNode.
                if self._cmd_vel_pub is not None:
                    if self._cmd_vel_pub.get_num_connections() > 0:
                        _last_pub_ok = time.time()
                    elif time.time() - _last_pub_ok > 5.0:
                        try:
                            self._cmd_vel_pub.unregister()
                            self._cmd_vel_pub = rospy.Publisher(
                                CMD_VEL_TOPIC, Twist, queue_size=1)
                            _last_pub_ok = time.time()
                            rospy.logwarn("[SCOUT] /cmd_vel publisher recreated — MotorNode reconnect triggered")
                        except Exception:
                            pass
        threading.Thread(target=_resub, daemon=True).start()

    # ── camera ──────────────────────────────────────────────────────────────

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
            rospy.logwarn(f"[CAMERA] {e}")

    def _subscribe_camera(self):
        msg_class, real_topic, _ = get_topic_class(CAMERA_TOPIC, blocking=False)
        if msg_class is None:
            rospy.logwarn(f"[SCOUT] Msg type for {CAMERA_TOPIC} not yet available.")
            return
        self._camera_sub = rospy.Subscriber(real_topic, msg_class,
                                             self._on_camera_frame, queue_size=1)
        rospy.loginfo(f"[SCOUT] Subscribed to {real_topic} [{msg_class._type}]")

    # ── sensors ─────────────────────────────────────────────────────────────

    def _subscribe_sensors(self):
        try:
            self._tof_sub = rospy.Subscriber(TOF_TOPIC, Range, self._on_tof, queue_size=1)
        except Exception as e:
            rospy.logwarn(f"[SCOUT] ToF sub failed: {e}")
        try:
            self._imu_sub = rospy.Subscriber(IMU_TOPIC, Imu, self._on_imu, queue_size=1)
        except Exception as e:
            rospy.logwarn(f"[SCOUT] IMU sub failed: {e}")
        try:
            self._vio_sub = rospy.Subscriber(VIO_ODOM_TOPIC, Odometry,
                                              self._on_vio_odom, queue_size=1)
        except Exception as e:
            rospy.logwarn(f"[SCOUT] VIO sub failed: {e}")
        try:
            bat_class, bat_topic, _ = get_topic_class(BATTERY_TOPIC, blocking=False)
            if bat_class:
                self._battery_sub = rospy.Subscriber(bat_topic, bat_class,
                                                      self._on_battery, queue_size=1)
        except Exception as e:
            rospy.logwarn(f"[SCOUT] Battery sub failed: {e}")

    def _on_tof(self, msg: Range):
        import math
        with _sensor_lock:
            global _tof_range
            _tof_range = None if math.isinf(msg.range) or math.isnan(msg.range) else round(msg.range, 3)

    def _on_imu(self, msg: Imu):
        with _sensor_lock:
            global _imu_data
            _imu_data = {
                "angular_velocity": {
                    "x": round(msg.angular_velocity.x, 4),
                    "y": round(msg.angular_velocity.y, 4),
                    "z": round(msg.angular_velocity.z, 4),
                },
                "linear_acceleration": {
                    "x": round(msg.linear_acceleration.x, 3),
                    "y": round(msg.linear_acceleration.y, 3),
                    "z": round(msg.linear_acceleration.z, 3),
                },
            }

    def _on_vio_odom(self, msg: Odometry):
        with _sensor_lock:
            global _vio_odom
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            v = msg.twist.twist
            _vio_odom = {
                "position": {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4)},
                "orientation": {"x": round(o.x, 4), "y": round(o.y, 4),
                                 "z": round(o.z, 4), "w": round(o.w, 4)},
                "velocity": {
                    "linear": {"x": round(v.linear.x, 4), "y": round(v.linear.y, 4)},
                    "angular": {"z": round(v.angular.z, 4)},
                },
            }

    def _on_battery(self, msg):
        with _sensor_lock:
            global _battery
            try:
                _battery = list(msg.status)
            except Exception:
                pass

    # ── properties ──────────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        if not self._inited or rospy.is_shutdown():
            return False
        try:
            rospy.get_published_topics()
            return True
        except Exception:
            return False

    def get_latest_frame(self) -> Optional[bytes]:
        with _camera_lock:
            return _camera_data[0] if _camera_data else None

    def camera_status(self):
        with _camera_lock:
            has  = len(_camera_data) > 0
            size = len(_camera_data[0]) if has else 0
        age = None if _last_frame_ts is None else round(time.time() - _last_frame_ts, 3)
        return has, size, age

    def get_sensors(self) -> dict:
        with _sensor_lock:
            bat = None
            if _battery and len(_battery) >= 2:
                bat = {
                    "percentage": _battery[1],
                    "charging":   _battery[0] == 0,
                    "full":       _battery[0] == 2,
                    "raw":        _battery,
                }
            return {
                "tof_range_m": _tof_range,
                "imu": _imu_data,
                "vio_odom": _vio_odom,
                "battery": bat,
            }

    # ── motion ──────────────────────────────────────────────────────────────

    def publish_twist(self, x: float = 0.0, y: float = 0.0, rotate: float = 0.0) -> bool:
        if not self._inited or self._cmd_vel_pub is None:
            return False
        msg = Twist()
        msg.linear.x  = float(x)
        msg.linear.y  = float(y)
        msg.angular.z = float(rotate)
        self._cmd_vel_pub.publish(msg)
        return True

    def stop_robot(self) -> bool:
        return self.publish_twist(0.0, 0.0, 0.0)

    def algo_action(self, x_speed: float, y_speed: float,
                    rotated_speed: float, duration_s: int) -> dict:
        """Timed move via UtilNode/algo_action service."""
        try:
            from roller_eye.srv import algo_action as _svc
            with self._svc_lock:
                if self._algo_action_svc is None:
                    rospy.wait_for_service("/UtilNode/algo_action", timeout=2.0)
                    self._algo_action_svc = rospy.ServiceProxy(
                        "/UtilNode/algo_action", _svc, persistent=True)
            resp = self._algo_action_svc(
                xSpeed=float(x_speed), ySpeed=float(y_speed),
                rotatedSpeed=float(rotated_speed), time=int(duration_s))
            return {"ok": resp.ret == 0, "ret": resp.ret}
        except Exception as e:
            with self._svc_lock:
                self._algo_action_svc = None
            return {"ok": False, "error": str(e)}

    def algo_move(self, x_dist: float, y_dist: float, speed: float) -> dict:
        """Move a specific distance (m) at given speed."""
        try:
            from roller_eye.srv import algo_move as _svc
            rospy.wait_for_service("/UtilNode/algo_move", timeout=2.0)
            svc = rospy.ServiceProxy("/UtilNode/algo_move", _svc)
            resp = svc(xDistance=float(x_dist), yDistance=float(y_dist), speed=float(speed))
            return {"ok": resp.ret == 0, "ret": resp.ret}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def algo_roll(self, angle_rad: float, speed_rad_s: float = 1.0,
                  timeout_s: int = 10, error_rad: float = 0.05) -> dict:
        """Rotate to a specific angle (radians)."""
        try:
            from roller_eye.srv import algo_roll as _svc
            rospy.wait_for_service("/UtilNode/algo_roll", timeout=2.0)
            svc = rospy.ServiceProxy("/UtilNode/algo_roll", _svc)
            resp = svc(angle=float(angle_rad), rotatedSpeed=float(speed_rad_s),
                       timeout=int(timeout_s), error=float(error_rad))
            return {"ok": resp.ret == 0, "ret": resp.ret}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # ── nav ─────────────────────────────────────────────────────────────────

    def nav_list_paths(self) -> dict:
        try:
            from roller_eye.srv import nav_list_path as _svc
            rospy.wait_for_service("/NavPathNode/nav_list_path", timeout=2.0)
            svc = rospy.ServiceProxy("/NavPathNode/nav_list_path", _svc)
            resp = svc()
            return {"paths": list(resp.name_list), "created": list(resp.create_time_list)}
        except Exception as e:
            return {"paths": [], "error": str(e)}

    def nav_start_patrol(self, name: str, from_start: bool = True) -> dict:
        try:
            from roller_eye.srv import nav_patrol as _svc
            rospy.wait_for_service("/NavPathNode/nav_patrol", timeout=2.0)
            svc = rospy.ServiceProxy("/NavPathNode/nav_patrol", _svc)
            resp = svc(isFromOutStart=int(from_start), name=name)
            return {"ok": resp.ret == 0, "ret": resp.ret}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def nav_stop_patrol(self) -> dict:
        try:
            from roller_eye.srv import nav_patrol_stop as _svc
            rospy.wait_for_service("/NavPathNode/nav_patrol_stop", timeout=2.0)
            svc = rospy.ServiceProxy("/NavPathNode/nav_patrol_stop", _svc)
            svc()
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def nav_cancel(self) -> dict:
        try:
            from roller_eye.srv import nav_cancel as _svc
            rospy.wait_for_service("/NavPathNode/nav_cancel", timeout=2.0)
            svc = rospy.ServiceProxy("/NavPathNode/nav_cancel", _svc)
            svc()
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def nav_get_status(self) -> dict:
        try:
            from roller_eye.srv import nav_get_status as _svc
            rospy.wait_for_service("/NavPathNode/nav_get_status", timeout=2.0)
            svc = rospy.ServiceProxy("/NavPathNode/nav_get_status", _svc)
            resp = svc()
            return {"status": resp.status}
        except Exception as e:
            return {"status": -1, "error": str(e)}

    def nav_save_path(self, name: str) -> dict:
        try:
            from roller_eye.srv import nav_path_save as _svc
            rospy.wait_for_service("/NavPathNode/nav_path_save", timeout=2.0)
            svc = rospy.ServiceProxy("/NavPathNode/nav_path_save", _svc)
            svc(name=name)
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}
