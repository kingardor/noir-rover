#!/usr/bin/env python3 -u
"""
Native macOS controller driver — Xbox and PS5 DualSense over Bluetooth.

Uses Apple GameController.framework so no USB cable or evdev is needed.
Auto-detects whichever controller is paired; prefers DualSense if both present.

Sends velocity commands to the bridge API (source=xbox) and stamps
xbox:last_input_ts in Redis so the safety arbiter knows a human is driving.

PS5: lightbar reflects driving state, adaptive triggers give resistance.
Xbox: haptic pulse on boost/precision press.

Usage:
    make controller
"""
import sys
sys.stdout.reconfigure(line_buffering=True)

import math
import time
import os
import queue
import threading

import requests
import redis as redis_lib
from Foundation import NSRunLoop, NSDate, NSDefaultRunLoopMode
from GameController import (
    GCController, GCColor, GCHapticsLocalityHandles, GCDualSenseAdaptiveTrigger,
)
import CoreHaptics as CH

# ── Config ────────────────────────────────────────────────────────────────────

BRIDGE_URL  = os.getenv("BRIDGE_URL", "http://localhost:8012")
REDIS_URL   = os.getenv("REDIS_URL",  "redis://localhost:6380")

POLL_HZ     = 60
DEADZONE    = 0.08
TRIG_PRESS  = 0.50
DURATION_MS = 400   # velocity hold per send; covers HTTP RTT spikes; robot auto-stops if sends cease

SPEED = {
    "base":      (0.25,  4.0),
    "boost":     (1.4,  10.0),
    "precision": (0.10,  1.5),
}

# PS5 lightbar colors — GCColor takes 0.0–1.0 floats
_C_IDLE      = (30/255,  30/255,  80/255)   # dim blue
_C_PRECISION = (60/255,   0/255, 180/255)   # purple   — L2 held
_C_BOOST     = (255/255, 100/255,  0/255)   # orange   — R2 held (brightness ∝ speed)
_C_ROTATE    = (200/255, 200/255,  0/255)   # yellow   — L1 / R1

# ── Helpers ───────────────────────────────────────────────────────────────────

def _dz(v):
    if abs(v) < DEADZONE:
        return 0.0
    s = math.copysign(1.0, v)
    return s * (abs(v) - DEADZONE) / (1.0 - DEADZONE)

def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v

def _gc_color(r, g, b):
    return GCColor.alloc().initWithRed_green_blue_(r, g, b)

def _make_engine(ctrl):
    h = ctrl.haptics()
    if h is None:
        return None
    e = h.createEngineWithLocality_(GCHapticsLocalityHandles)
    if e is None:
        return None
    ok, _ = e.startAndReturnError_(None)
    return e if ok else None

def _make_transient(t, intensity, sharpness):
    pi = CH.CHHapticEventParameter.alloc().initWithParameterID_value_(
        CH.CHHapticEventParameterIDHapticIntensity, float(intensity))
    sh = CH.CHHapticEventParameter.alloc().initWithParameterID_value_(
        CH.CHHapticEventParameterIDHapticSharpness, float(sharpness))
    return CH.CHHapticEvent.alloc().initWithEventType_parameters_relativeTime_(
        CH.CHHapticEventTypeHapticTransient, [pi, sh], float(t))


# (time_s, intensity 0-1, sharpness 0=soft 1=sharp)
_HAPTIC_PRECISION = [
    (0.00, 0.40, 0.05),   # soft thud
    (0.14, 0.65, 0.05),   # slightly firmer thud
]
_HAPTIC_BOOST = [
    (0.00, 0.45, 0.85),   # crisp punch
    (0.08, 0.72, 0.88),   # stronger
    (0.16, 1.00, 0.92),   # full power crack
]


def _buzz(engine, style: str):
    if engine is None:
        return
    specs = _HAPTIC_PRECISION if style == "precision" else _HAPTIC_BOOST
    try:
        events = [_make_transient(*s) for s in specs]
        pat, _ = CH.CHHapticPattern.alloc().initWithEvents_parameters_error_(events, [], None)
        if pat is None:
            return
        player, _ = engine.createPlayerWithPattern_error_(pat, None)
        if player is None:
            return
        player.startAtTime_error_(0.0, None)
    except Exception:
        pass

def _is_ps5(ctrl):
    n = ctrl.vendorName() or ""
    return "DualSense" in n or "PS5" in n

# ── Controller session ────────────────────────────────────────────────────────

class Session:
    """One connected controller — input reading, lightbar, haptics."""

    def __init__(self, ctrl):
        self.ctrl   = ctrl
        self.gp     = ctrl.extendedGamepad()
        self.ps5    = _is_ps5(ctrl)
        self.engine = _make_engine(ctrl)
        self._color = None

        if self.ps5:
            self._configure_triggers()
            self._set_light(*_C_IDLE)

        kind    = "DualSense" if self.ps5 else "Xbox"
        haptics = "ok" if self.engine else "none"
        print(f"[ctrl] {kind} connected — haptics: {haptics}", flush=True)

    def _configure_triggers(self):
        try:
            lt = self.gp.leftTrigger()
            rt = self.gp.rightTrigger()
            if isinstance(lt, GCDualSenseAdaptiveTrigger):
                lt.setModeFeedbackWithStartPosition_resistiveStrength_(0.0, 0.8)
            if isinstance(rt, GCDualSenseAdaptiveTrigger):
                rt.setModeVibrationWithStartPosition_amplitude_frequency_(0.2, 0.8, 15.0)
        except Exception:
            pass

    def _set_light(self, r, g, b):
        color = (r, g, b)
        if color == self._color:
            return
        light = self.ctrl.light()
        if light is not None:
            light.setColor_(_gc_color(r, g, b))
        self._color = color

    def update_light(self, lt, rt, rotating, speed_norm):
        """Update PS5 lightbar based on driving state."""
        if not self.ps5:
            return
        if lt:
            self._set_light(*_C_PRECISION)
        elif rt:
            bri = 80/255 + min(speed_norm, 1.0) * 175/255
            self._set_light(bri, bri * 0.39, 0.0)
        elif rotating:
            self._set_light(*_C_ROTATE)
        elif speed_norm > 0.05:
            bri = 80/255 + min(speed_norm, 1.0) * 175/255
            self._set_light(0.0, bri, bri * 0.12)   # green, slight teal tint
        else:
            self._set_light(*_C_IDLE)

    def read(self):
        """Return (LY, RX, LT, RT, LB, RB). Left stick Y = fwd/back, right stick X = strafe."""
        gp = self.gp
        return (
            _dz(_clamp(gp.leftThumbstick().yAxis().value())),
            _dz(_clamp(gp.rightThumbstick().xAxis().value())),
            _clamp(gp.leftTrigger().value(),  0.0, 1.0),
            _clamp(gp.rightTrigger().value(), 0.0, 1.0),
            bool(gp.leftShoulder().isPressed()),
            bool(gp.rightShoulder().isPressed()),
        )

    def buzz(self, style: str):
        _buzz(self.engine, style)

    def teardown(self):
        if self.ps5:
            self._set_light(0.0, 0.0, 0.0)

# ── Bridge I/O ────────────────────────────────────────────────────────────────

# ── Background sender ─────────────────────────────────────────────────────────
# ROS publish inside bridge-api can take 200-400 ms; running HTTP in a background
# thread keeps the controller polling loop at full 60 Hz regardless.
# Queue depth 1: old commands are dropped in favour of the latest.

_send_q: queue.SimpleQueue = queue.SimpleQueue()
_last_err: str = ""
_last_err_ts: float = 0.0


def _sender_loop(rc):
    global _last_err, _last_err_ts
    http = requests.Session()
    while True:
        item = _send_q.get()          # block until there's something to send
        while not _send_q.empty():    # drain to the latest command only
            item = _send_q.get_nowait()
        if item is None:
            break

        endpoint, payload = item
        try:
            r = http.post(f"{BRIDGE_URL}/{endpoint}", json=payload, timeout=0.5)
            if r.status_code != 200:
                msg = f"bridge {r.status_code}: {r.text[:120]}"
                now = time.time()
                if msg != _last_err or now - _last_err_ts > 5.0:
                    print(f"[warn] {msg}", flush=True)
                    _last_err, _last_err_ts = msg, now
            else:
                _last_err = ""
                if endpoint == "robot/move" and rc:
                    try:
                        rc.set("xbox:last_input_ts", str(time.time()), ex=5)
                    except Exception:
                        pass
        except Exception as e:
            now = time.time()
            msg = str(e)[:100]
            if msg != _last_err or now - _last_err_ts > 5.0:
                print(f"[warn] send error: {msg}", flush=True)
                _last_err, _last_err_ts = msg, now


def _send_move(x, y, rotate):
    _send_q.put(("robot/move", {
        "x": round(x, 4), "y": round(y, 4), "rotate": round(rotate, 4),
        "source": "xbox", "duration_ms": DURATION_MS,
    }))


def _send_stop():
    _send_q.put(("robot/stop", {}))

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    GCController.setShouldMonitorBackgroundEvents_(True)
    GCController.startWirelessControllerDiscoveryWithCompletionHandler_(None)
    print(f"Searching for controller…  (press PS/Xbox button to wake)")
    print(f"Bridge : {BRIDGE_URL}")
    print(f"Redis  : {REDIS_URL}")
    print("Ctrl+C to quit.\n")

    try:
        st = requests.get(f"{BRIDGE_URL}/status", timeout=2).json()
        print(f"[bridge] ros_connected={st.get('ros_connected')}  "
              f"camera={st.get('camera_ok')}  mode={st.get('mode','?')}", flush=True)
    except Exception as e:
        print(f"[bridge] unreachable — {e}", flush=True)

    runloop = NSRunLoop.currentRunLoop()
    tick_s  = 1.0 / POLL_HZ

    try:
        rc = redis_lib.from_url(REDIS_URL, decode_responses=True)
        rc.ping()
    except Exception:
        rc = None
        print("[warn] Redis unavailable — arbiter stamp disabled")

    sender = threading.Thread(target=_sender_loop, args=(rc,), daemon=True, name="sender")
    sender.start()

    session    = None
    prev_lt    = False
    prev_rt    = False
    was_moving = False

    try:
        while True:
            runloop.runMode_beforeDate_(
                NSDefaultRunLoopMode,
                NSDate.dateWithTimeIntervalSinceNow_(tick_s),
            )

            controllers = list(GCController.controllers())

            # ── Disconnect ───────────────────────────────────────────────────
            if session is not None and session.ctrl not in controllers:
                print("[ctrl] Disconnected")
                session.teardown()
                session    = None
                prev_lt    = prev_rt = was_moving = False
                _send_stop()
                continue

            # ── Connect ──────────────────────────────────────────────────────
            if session is None and controllers:
                ctrl = next((c for c in controllers if _is_ps5(c)), controllers[0])
                if ctrl.extendedGamepad() is None:
                    continue
                print(f"[ctrl] Found: {ctrl.vendorName()}")
                session = Session(ctrl)
                continue

            if session is None:
                continue

            # ── Inputs ───────────────────────────────────────────────────────
            LY, RX, LT, RT, LB, RB = session.read()

            lt = LT > TRIG_PRESS
            rt = RT > TRIG_PRESS

            if lt and not prev_lt:
                session.buzz("precision")
            if rt and not prev_rt:
                session.buzz("boost")
            prev_lt, prev_rt = lt, rt

            # ── Velocity ─────────────────────────────────────────────────────
            if lt:
                fwdmax, rotmax = SPEED["precision"]
            elif rt:
                fwdmax, rotmax = SPEED["boost"]
            else:
                fwdmax, rotmax = SPEED["base"]

            fwd    =  _clamp(LY) * fwdmax
            strafe =  _clamp(RX) * fwdmax
            rotate = -((1.0 if RB else 0.0) - (1.0 if LB else 0.0)) * rotmax

            moving = abs(fwd) > 1e-3 or abs(strafe) > 1e-3 or abs(rotate) > 1e-3

            # ── Lightbar ─────────────────────────────────────────────────────
            speed_norm = max(abs(fwd), abs(strafe)) / max(fwdmax, 1e-6)
            session.update_light(lt, rt, abs(rotate) > 0.01, speed_norm)

            # ── Send ─────────────────────────────────────────────────────────
            if moving:
                _send_move(strafe, fwd, rotate)
                was_moving = True
            elif was_moving:
                _send_stop()
                was_moving = False

    except KeyboardInterrupt:
        pass
    finally:
        if session:
            session.teardown()
        GCController.stopWirelessControllerDiscovery()
        _send_stop()
        _send_q.put(None)   # signal sender thread to exit
        print("\nGoodbye.")


if __name__ == "__main__":
    main()
