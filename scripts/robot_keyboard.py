#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Keyboard teleoperation - runs on the robot via SSH, not in Docker."""
import os, sys, tty, termios, select, time
import rospy
from geometry_msgs.msg import Twist

SPEED_LIN = 0.4
SPEED_ROT = 4.0

MAP = {
    "w": (0.0,  SPEED_LIN,  0.0),
    "s": (0.0, -SPEED_LIN,  0.0),
    "a": (-SPEED_LIN, 0.0,  0.0),
    "d": ( SPEED_LIN, 0.0,  0.0),
    "q": (0.0,  0.0, -SPEED_ROT),
    "e": (0.0,  0.0,  SPEED_ROT),
    " ": (0.0,  0.0,  0.0),
}

rospy.init_node("keyboard_drive", anonymous=True)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rate = rospy.Rate(30)

# Wait for MotorNode to establish the TCPROS subscriber connection before
# accepting input - without this, the first few publishes are dropped.
rospy.sleep(0.5)

fd = sys.stdin.fileno()
saved = termios.tcgetattr(fd)
tty.setraw(fd)

# Two thresholds to handle macOS TTY key-repeat behaviour over SSH:
#   macOS sends ONE char on keydown, then waits ~300-500ms (initial delay)
#   before sending repeats at ~30ms intervals. We can't see keyup events.
#
# Strategy: once rapid repeats confirm the key is held (in_repeat=True), use
# a short timeout so release feels immediate. Before that, use a longer grace
# period so the initial-delay gap doesn't trigger a false stop.
STOP_IDLE_HOLD  = 0.12   # stop this fast once key-repeat is active
STOP_IDLE_FIRST = 0.65   # grace period for the initial key-repeat delay
REPEAT_GAP_MAX  = 0.20   # two events this close -> we're in auto-repeat

x = y = r = 0.0
last_key_ts = 0.0
last_key = None
in_repeat = False

sys.stdout.write("W/S/A/D/Q/E=move  SPACE=stop  Ctrl+C=quit\r\n")
sys.stdout.flush()

try:
    while not rospy.is_shutdown():
        rdy, _, _ = select.select([fd], [], [], 0.03)
        now = time.time()
        if rdy:
            k = os.read(fd, 1).decode("utf-8", "ignore")
            if k == "\x03":
                break
            if k in MAP:
                if k == last_key and (now - last_key_ts) < REPEAT_GAP_MAX:
                    in_repeat = True
                else:
                    in_repeat = False
                x, y, r = MAP[k]
                last_key = k
                last_key_ts = now

        threshold = STOP_IDLE_HOLD if in_repeat else STOP_IDLE_FIRST
        if now - last_key_ts > threshold:
            x = y = r = 0.0
            in_repeat = False
            last_key = None

        msg = Twist()
        msg.linear.x  = x
        msg.linear.y  = y
        msg.angular.z = r
        pub.publish(msg)
        rate.sleep()
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, saved)
    pub.publish(Twist())
    sys.stdout.write("\r\nstopped.\r\n")
    sys.stdout.flush()
