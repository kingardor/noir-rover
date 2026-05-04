#!/usr/bin/env python3
"""
Native macOS keyboard teleoperation — sends UDP directly to the on-robot relay.
No Docker, no ROS, no TTY gymnastics.

Usage:
  python3 scripts/keyboard_drive_native.py

  W/S = forward / backward
  A/D = strafe left / right
  Q/E = rotate CCW / CW
  SPACE = stop
  Ctrl+C = quit
"""
import curses
import os
import sys
import time

# relay_protocol.py lives in ros-noetic/; find it relative to this script
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros-noetic'))
from relay_protocol import RelayClient

SPEED_LIN = float(os.getenv("KD_SPEED_LIN", "0.4"))
SPEED_ROT = float(os.getenv("KD_SPEED_ROT", "4.0"))
SEND_HZ   = int(os.getenv("KD_HZ", "30"))       # how often to refresh the relay
EXPIRES   = int(os.getenv("KD_EXPIRES_MS", "150"))  # relay timeout per packet

_KEY_MAP = {
    ord('w'): ( 0.0,  SPEED_LIN,  0.0),
    ord('s'): ( 0.0, -SPEED_LIN,  0.0),
    ord('a'): (-SPEED_LIN, 0.0,   0.0),
    ord('d'): ( SPEED_LIN, 0.0,   0.0),
    ord('q'): ( 0.0,  0.0, -SPEED_ROT),
    ord('e'): ( 0.0,  0.0,  SPEED_ROT),
    ord(' '): ( 0.0,  0.0,  0.0),
}


def run(stdscr):
    relay = RelayClient()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)   # non-blocking getch

    stdscr.clear()
    stdscr.addstr(0, 0, "Noir rover keyboard control")
    stdscr.addstr(1, 0, f"  speed {SPEED_LIN} m/s  rot {SPEED_ROT} rad/s  {SEND_HZ} Hz")
    stdscr.addstr(3, 0, "  W/S = fwd/back   A/D = strafe   Q/E = rotate")
    stdscr.addstr(4, 0, "  SPACE = stop     Ctrl+C = quit")
    stdscr.addstr(6, 0, "status: idle")
    stdscr.refresh()

    x = y = r = 0.0
    dt = 1.0 / SEND_HZ
    last_key = "–"

    try:
        while True:
            ch = stdscr.getch()
            if ch == 3:   # Ctrl+C
                break
            if ch in _KEY_MAP:
                x, y, r = _KEY_MAP[ch]
                last_key = chr(ch) if ch != ord(' ') else 'SPACE'
            elif ch == curses.ERR:
                pass  # no key pressed this tick
            else:
                x = y = r = 0.0

            relay.send(x, y, r, EXPIRES)

            stdscr.addstr(6, 0, f"key: {last_key:<6}  x={x:+.2f}  y={y:+.2f}  r={r:+.1f}   ")
            stdscr.refresh()
            time.sleep(dt)
    finally:
        relay.send(0.0, 0.0, 0.0, 0)
        relay.close()


if __name__ == "__main__":
    curses.wrapper(run)
