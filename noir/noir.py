#!/usr/bin/env python3
import os
import time
import threading
import requests
from xbox import XboxBotDriver

API_BASE = os.getenv("BOT_API_BASE", "http://0.0.0.0:6969")
MOVE_ENDPOINT = f"{API_BASE}/robot/move"
STOP_ENDPOINT = f"{API_BASE}/robot/stop"

SEND_PERIOD = 0.1
REQ_TIMEOUT = 0.25

DEVICE_HINT = os.getenv("CONTROLLER_DEVICE")  # e.g., /dev/input/event20

def post_move(payload: dict):
    try:
        requests.post(MOVE_ENDPOINT, headers={"Content-Type":"application/json"}, json=payload, timeout=REQ_TIMEOUT)
    except requests.exceptions.RequestException as e:
        print(f"[net] move post failed: {e.__class__.__name__}")

def post_stop():
    try:
        requests.post(STOP_ENDPOINT, timeout=REQ_TIMEOUT)
    except requests.RequestException:
        pass
    print("[exit] Stopped and exiting cleanly.")

def send_loop(driver: XboxBotDriver):
    last = 0.0
    try:
        while driver.running:
            now = time.monotonic()
            if now - last >= SEND_PERIOD:
                last = now
                payload = driver.get_payload()
                post_move(payload)
            time.sleep(0.01)
    finally:
        post_stop()

if __name__ == "__main__":
    driver = XboxBotDriver(device_hint=DEVICE_HINT)
    print("[map] LX→strafe, LY→forward/back (preset-limited); RB→+rot, LB→-rot; RT=boost (1.0/10), LT=precision (0.1/1.0)")
    # Start event and control loops
    driver.start_event_and_control_loops()
    # Start send loop (main thread)
    send_loop(driver)
