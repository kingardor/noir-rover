#!/usr/bin/env python
"""
cmd_vel UDP relay -- runs on the robot (10.42.0.1), not in Docker.
Python 2 / ROS Melodic compatible.

The relay owns the rospy.Publisher for /cmd_vel so MotorNode subscribes
via loopback (TCPROS back-connection always works to 127.0.0.1).

Wire format (matches relay_protocol.py):
  struct.pack('<fffI', x, y, rotate, expires_ms)  -- 16 bytes

Deploy with: scripts/deploy_relay.sh
"""
from __future__ import print_function
import socket
import struct
import threading
import time

import rospy
from geometry_msgs.msg import Twist

_FMT   = b'<fffI'
_FMTSZ = struct.calcsize(_FMT)   # 16

BIND_HOST  = '0.0.0.0'
BIND_PORT  = 9999
PUBLISH_HZ = 60

_x        = 0.0
_y        = 0.0
_rotate   = 0.0
_deadline = 0.0
_lock = threading.Lock()


def _recv_loop(sock):
    global _x, _y, _rotate, _deadline
    while True:
        try:
            data, _ = sock.recvfrom(64)
        except OSError:
            break
        if len(data) < _FMTSZ:
            continue
        x, y, rotate, expires_ms = struct.unpack_from(_FMT, data)
        deadline = time.time() + expires_ms / 1000.0
        with _lock:
            _x, _y, _rotate, _deadline = x, y, rotate, deadline


def main():
    rospy.init_node('cmd_vel_relay', anonymous=False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.loginfo('[relay] Publisher ready on /cmd_vel at %d Hz', PUBLISH_HZ)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BIND_HOST, BIND_PORT))
    rospy.loginfo('[relay] Listening UDP %s:%d', BIND_HOST, BIND_PORT)

    t = threading.Thread(target=_recv_loop, args=(sock,))
    t.daemon = True
    t.start()

    rate = rospy.Rate(PUBLISH_HZ)
    last_was_active = False
    log_tick = 0

    while not rospy.is_shutdown():
        now = time.time()
        with _lock:
            active = now < _deadline
            x, y, r = _x, _y, _rotate

        msg = Twist()
        if active:
            msg.linear.x  = float(x)
            msg.linear.y  = float(y)
            msg.angular.z = float(r)
            last_was_active = True
        elif last_was_active:
            last_was_active = False   # publish one trailing zero

        pub.publish(msg)

        log_tick += 1
        if log_tick >= PUBLISH_HZ * 10:
            rospy.loginfo('[relay] alive active=%s x=%.2f y=%.2f r=%.2f', active, x, y, r)
            log_tick = 0

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
