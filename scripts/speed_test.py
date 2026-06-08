#!/usr/bin/env python3
"""
Top-speed finder for the Scout robot.

Temporarily raises the bridge arbiter cap, sends short forward bursts at
increasing speeds, then reports the highest speed that the robot accepted
and executed without faulting.

Usage:
    python3 scripts/speed_test.py [--url http://localhost:8012]

The robot will move forward in short ~150ms bursts. Keep it in open space.
The original arbiter cap is restored automatically on exit.
"""
import sys
import time
import argparse
import urllib.request
import urllib.error
import json
import re
import pathlib

ARBITER_CAP = 3.0      # temporary cap for the test
BURST_MS    = 150      # duration of each burst (ms) — barely moves
PAUSE_S     = 1.5      # wait between steps (let robot settle)
TEST_SPEEDS = [1.0, 1.2, 1.4, 1.5, 1.6, 1.7, 1.8, 2.0, 2.2, 2.5]

BRIDGE_API  = pathlib.Path(__file__).parent.parent / "ros-noetic" / "bridge-api.py"


def _post(url, payload):
    data = json.dumps(payload).encode()
    req  = urllib.request.Request(url, data=data,
                                  headers={"Content-Type": "application/json"},
                                  method="POST")
    try:
        with urllib.request.urlopen(req, timeout=2) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, e.reason
    except Exception as e:
        return None, str(e)


def patch_arbiter(cap: float):
    src = BRIDGE_API.read_text()
    patched = re.sub(
        r'(if abs\(x\) > )[\d.]+( or abs\(y\) > )[\d.]+( or abs\(rotate\) > )[\d.]+(:)',
        lambda m: f"{m.group(1)}{cap}{m.group(2)}{cap}{m.group(3)}12.0{m.group(4)}",
        src,
    )
    BRIDGE_API.write_text(patched)


def restore_arbiter():
    patch_arbiter(1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8012")
    args = ap.parse_args()
    base = args.url.rstrip("/")

    # Check bridge is up
    status_code, status = _post(f"{base}/robot/stop", {})
    if status_code is None:
        print(f"Bridge not reachable at {base}: {status}")
        sys.exit(1)

    print(f"Bridge reachable. Patching arbiter to {ARBITER_CAP} m/s cap…")
    original = BRIDGE_API.read_text()
    patch_arbiter(ARBITER_CAP)
    print("NOTE: bridge restart required for patch to take effect.")
    print("Restart the bridge now, then press Enter to begin the speed test.")
    input()

    results = []
    try:
        for speed in TEST_SPEEDS:
            code, resp = _post(f"{base}/robot/move", {
                "x": 0.0, "y": round(speed, 2), "rotate": 0.0,
                "source": "speed_test", "duration_ms": BURST_MS,
            })
            if code == 200:
                status_str = "OK"
            elif code == 403:
                status_str = "ARBITER_BLOCKED"
            elif code == 503:
                status_str = "ROS_FAULT"
            else:
                status_str = f"ERR_{code}"

            results.append((speed, status_str))
            print(f"  {speed:.1f} m/s → {status_str}   ({resp})")

            # Stop after fault — MotorNode may need recovery
            if status_str in ("ROS_FAULT", "ARBITER_BLOCKED"):
                _post(f"{base}/robot/stop", {})
                print("  Stopping test — robot faulted or arbiter blocked.")
                break

            time.sleep(PAUSE_S)

    finally:
        print("\nRestoring arbiter to 1.5 m/s…")
        BRIDGE_API.write_text(original)
        _post(f"{base}/robot/stop", {})

    # Report
    print("\n── Speed test results ──")
    ok_speeds = [s for s, r in results if r == "OK"]
    if ok_speeds:
        print(f"  Max confirmed speed : {max(ok_speeds):.1f} m/s")
    else:
        print("  No successful commands recorded.")
    for speed, result in results:
        marker = "✓" if result == "OK" else "✗"
        print(f"  {marker}  {speed:.1f} m/s  {result}")


if __name__ == "__main__":
    main()
