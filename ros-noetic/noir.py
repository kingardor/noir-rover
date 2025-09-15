#!/usr/bin/env python3
import os
import time
import signal
import threading

from scoutros import ScoutROS
from xbox import XboxBotDriver

SEND_HZ = float(os.getenv("NOIR_SEND_HZ", "60"))
EXIT_ON_ERROR = os.getenv("NOIR_EXIT_ON_ERROR", "0") == "1"

class NoirController:
    def __init__(self, device_hint=None):
        self.ros = ScoutROS(node_name="noir_controller")
        self.ros.init()
        self.controller = XboxBotDriver(device_hint=device_hint)
        self.controller.start_event_and_control_loops()
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.running = False
        try:
            self.controller.stop()
        except Exception:
            pass
        try:
            self.ros.stop_robot()
        except Exception:
            pass

    def _loop(self):
        dt = 1.0 / max(1.0, SEND_HZ)
        while self.running:
            try:
                payload = self.controller.get_payload()  # {"x": strafe, "y": fwd, "rotate": rot}
                ok = self.ros.publish_twist(
                    x=payload["x"],
                    y=payload["y"],
                    rotate=payload["rotate"]
                )
                if not ok:
                    # ROS not connected (master down?) — back off a bit
                    time.sleep(0.2)
            except Exception as e:
                print(f"[noir] control error: {e}", flush=True)
                if EXIT_ON_ERROR:
                    break
            time.sleep(dt)

def main():
    device_hint = os.getenv("CONTROLLER_DEVICE")
    nc = NoirController(device_hint=device_hint)

    def handle_sig(*_):
        print("[noir] stopping...", flush=True)
        nc.stop()

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    nc.start()

    # wait forever
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        handle_sig()

if __name__ == "__main__":
    main()
