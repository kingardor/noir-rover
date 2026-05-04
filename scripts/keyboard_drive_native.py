#!/usr/bin/env python3
"""
Native macOS keyboard teleoperation — sends commands to the bridge API.

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
import urllib.request
import json

BRIDGE_URL = os.getenv("BRIDGE_URL", "http://localhost:8012")
SPEED_LIN  = float(os.getenv("KD_SPEED_LIN", "0.4"))
SPEED_ROT  = float(os.getenv("KD_SPEED_ROT", "4.0"))
SEND_HZ    = int(os.getenv("KD_HZ", "10"))
HOLD_MS    = int(os.getenv("KD_HOLD_MS", "200"))   # duration_ms per command

_KEY_MAP = {
    ord('w'): ( 0.0,  SPEED_LIN,  0.0),
    ord('s'): ( 0.0, -SPEED_LIN,  0.0),
    ord('a'): (-SPEED_LIN, 0.0,   0.0),
    ord('d'): ( SPEED_LIN, 0.0,   0.0),
    ord('q'): ( 0.0,  0.0, -SPEED_ROT),
    ord('e'): ( 0.0,  0.0,  SPEED_ROT),
    ord(' '): ( 0.0,  0.0,  0.0),
}


def _post_move(x, y, rotate, duration_ms):
    payload = json.dumps({"x": x, "y": y, "rotate": rotate,
                          "duration_ms": duration_ms}).encode()
    req = urllib.request.Request(
        f"{BRIDGE_URL}/robot/move",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=0.3)
    except Exception:
        pass


def run(stdscr):
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    stdscr.clear()
    stdscr.addstr(0, 0, "Noir rover keyboard control")
    stdscr.addstr(1, 0, f"  bridge: {BRIDGE_URL}")
    stdscr.addstr(2, 0, f"  speed {SPEED_LIN} m/s  rot {SPEED_ROT} rad/s  {SEND_HZ} Hz")
    stdscr.addstr(4, 0, "  W/S = fwd/back   A/D = strafe   Q/E = rotate")
    stdscr.addstr(5, 0, "  SPACE = stop     Ctrl+C = quit")
    stdscr.addstr(7, 0, "status: idle")
    stdscr.refresh()

    x = y = r = 0.0
    dt = 1.0 / SEND_HZ
    last_key = "–"

    try:
        while True:
            ch = stdscr.getch()
            if ch == 3:
                break
            if ch in _KEY_MAP:
                x, y, r = _KEY_MAP[ch]
                last_key = chr(ch) if ch != ord(' ') else 'SPACE'
            elif ch != curses.ERR:
                x = y = r = 0.0

            _post_move(x, y, r, HOLD_MS)

            stdscr.addstr(7, 0, f"key: {last_key:<6}  x={x:+.2f}  y={y:+.2f}  r={r:+.1f}   ")
            stdscr.refresh()
            time.sleep(dt)
    finally:
        _post_move(0.0, 0.0, 0.0, 0)


if __name__ == "__main__":
    curses.wrapper(run)
