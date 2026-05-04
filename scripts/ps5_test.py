#!/usr/bin/env python3 -u
"""
PS5 DualSense controller tester — Apple GameController.framework (Bluetooth or USB).

Polling mode: ticks NSRunLoop so GameController receives BT events, then reads
button/axis state directly — no ObjC→Python callbacks, works on Python 3.14.

Usage:
  make ps5-test

Press buttons to see name + lightbar colour + haptic buzz.
"""
import sys
sys.stdout.reconfigure(line_buffering=True)
import CoreHaptics as CH
from Foundation import NSRunLoop, NSDate, NSDefaultRunLoopMode
from GameController import (
    GCController,
    GCColor,
    GCHapticsLocalityHandles,
    GCDualSenseAdaptiveTrigger,
)

# Deliver controller input even when this CLI process is not the foreground app
GCController.setShouldMonitorBackgroundEvents_(True)


IDLE_COLOR = (30 / 255, 30 / 255, 80 / 255)
POLL_HZ    = 60
DEADZONE   = 0.08


def _gc_color(r: float, g: float, b: float) -> GCColor:
    return GCColor.alloc().initWithRed_green_blue_(r, g, b)


def _set_light(ctrl, r: float, g: float, b: float):
    light = ctrl.light()
    if light is not None:
        light.setColor_(_gc_color(r, g, b))


def _make_haptic_engine(ctrl):
    haptics = ctrl.haptics()
    if haptics is None:
        return None
    engine = haptics.createEngineWithLocality_(GCHapticsLocalityHandles)
    if engine is None:
        return None
    ok, _ = engine.startAndReturnError_(None)   # NSError** → (bool, error) tuple
    return engine if ok else None


def _buzz(engine, intensity: float):
    if engine is None:
        return
    try:
        pi = CH.CHHapticEventParameter.alloc().initWithParameterID_value_(
            CH.CHHapticEventParameterIDHapticIntensity, intensity
        )
        ps = CH.CHHapticEventParameter.alloc().initWithParameterID_value_(
            CH.CHHapticEventParameterIDHapticSharpness, 0.5
        )
        event = CH.CHHapticEvent.alloc().initWithEventType_parameters_relativeTime_(
            CH.CHHapticEventTypeHapticTransient, [pi, ps], 0.0
        )
        # All NSError** params: pass None explicitly → method returns (result, error) tuple
        pattern, _ = CH.CHHapticPattern.alloc().initWithEvents_parameters_error_(
            [event], [], None
        )
        if pattern is None:
            return
        player, _ = engine.createPlayerWithPattern_error_(pattern, None)
        if player is None:
            return
        player.startAtTime_error_(0.0, None)
    except Exception:
        pass


def _configure_triggers(gp):
    """L2 = rigid feedback, R2 = vibration (DualSense adaptive triggers)."""
    try:
        lt = gp.leftTrigger()
        rt = gp.rightTrigger()
        if isinstance(lt, GCDualSenseAdaptiveTrigger):
            lt.setModeFeedbackWithStartPosition_resistiveStrength_(0.0, 0.8)
        if isinstance(rt, GCDualSenseAdaptiveTrigger):
            rt.setModeVibrationWithStartPosition_amplitude_frequency_(0.2, 0.8, 15.0)
    except Exception:
        pass


def _collect_buttons(gp):
    """Return list of (label, button_obj, haptic_intensity, gc_color)."""
    items = []

    def add(btn, label, intensity, r, g, b):
        if btn is not None:
            items.append((label, btn, intensity, _gc_color(r / 255, g / 255, b / 255)))

    add(gp.buttonA(),              "Cross ✕",       0.8,   0, 180, 255)
    add(gp.buttonB(),              "Circle ○",       0.8, 255,  50,  50)
    add(gp.buttonX(),              "Square □",       0.8, 200,   0, 200)
    add(gp.buttonY(),              "Triangle △",     0.8,   0, 220,  80)
    add(gp.leftShoulder(),         "L1",             0.6,  80, 120, 255)
    add(gp.rightShoulder(),        "R1",             0.6,  80, 120, 255)
    add(gp.leftTrigger(),          "L2",             0.9, 255, 140,   0)
    add(gp.rightTrigger(),         "R2",             0.9, 255, 140,   0)
    add(gp.leftThumbstickButton(), "L3",             0.7,   0, 200, 255)
    add(gp.rightThumbstickButton(),"R3",             0.7,   0, 200, 255)
    add(gp.buttonMenu(),           "Options ☰",      0.4, 200, 200, 200)
    add(gp.buttonOptions(),        "Create ✤",       0.4, 200, 200, 200)
    add(gp.buttonHome(),           "PS ⊙",          1.0, 255, 255, 255)

    tp_btn = getattr(gp, "touchpadButton", None)
    if callable(tp_btn):
        tp_btn = tp_btn()
    add(tp_btn, "Touchpad click", 0.3, 100, 255, 200)

    dpad = gp.dpad()
    if dpad is not None:
        add(dpad.up(),    "D-Pad ↑", 0.4, 255, 255, 0)
        add(dpad.down(),  "D-Pad ↓", 0.4, 255, 255, 0)
        add(dpad.left(),  "D-Pad ←", 0.4, 255, 255, 0)
        add(dpad.right(), "D-Pad →", 0.4, 255, 255, 0)

    return items


def main():
    GCController.startWirelessControllerDiscoveryWithCompletionHandler_(None)
    print("Searching for DualSense…  (press PS button if the controller is off)")
    print("Ctrl+C to quit.\n")

    runloop = NSRunLoop.currentRunLoop()
    tick_s  = 1.0 / POLL_HZ

    ctrl     = None
    gp       = None
    engine   = None
    buttons  = []     # list of (label, btn_obj, intensity, gc_color)
    prev     = {}     # label -> bool (was pressed last frame)
    idle_col = _gc_color(*IDLE_COLOR)

    try:
        while True:
            # Tick the runloop — GameController updates button state during this call
            runloop.runMode_beforeDate_(
                NSDefaultRunLoopMode,
                NSDate.dateWithTimeIntervalSinceNow_(tick_s)
            )

            # ── Controller connect / disconnect ───────────────────────────
            controllers = list(GCController.controllers())

            if ctrl is not None and ctrl not in controllers:
                print("\nController disconnected.  Press PS button to reconnect.\n")
                _set_light(ctrl, 0, 0, 0)
                ctrl = gp = engine = None
                buttons, prev = [], {}
                continue

            if ctrl is None and controllers:
                # Prefer DualSense; fall back to first controller with extendedGamepad
                def _is_dualsense(c):
                    n = c.vendorName() or ""
                    return "DualSense" in n or "PS5" in n
                ctrl = next((c for c in controllers if _is_dualsense(c)), controllers[0])
                gp   = ctrl.extendedGamepad()
                if gp is None:
                    ctrl = None
                    continue
                print(f"Connected: {ctrl.vendorName()}\n")
                _set_light(ctrl, *IDLE_COLOR)
                engine  = _make_haptic_engine(ctrl)
                _configure_triggers(gp)
                buttons = _collect_buttons(gp)
                prev    = {label: False for label, *_ in buttons}
                continue  # read next frame so btn objects are fully initialised

            if ctrl is None:
                continue

            # ── Button state ──────────────────────────────────────────────
            for label, btn, intensity, color in buttons:
                pressed = bool(btn.isPressed())
                was     = prev.get(label, False)
                if pressed and not was:
                    light = ctrl.light()
                    if light is not None:
                        light.setColor_(color)
                    _buzz(engine, intensity)
                    print(f"  PRESSED  {label}")
                elif not pressed and was:
                    print(f"  released {label}")
                    light = ctrl.light()
                    if light is not None:
                        light.setColor_(idle_col)
                prev[label] = pressed

            # ── Thumbsticks ───────────────────────────────────────────────
            for stick, name in ((gp.leftThumbstick(), "L"), (gp.rightThumbstick(), "R")):
                if stick is None:
                    continue
                x = stick.xAxis().value()
                y = stick.yAxis().value()
                if abs(x) > DEADZONE or abs(y) > DEADZONE:
                    print(f"\r  {name}-stick: x={x:+.2f}  y={y:+.2f}   ", end="", flush=True)

            # ── Touchpad finger position (DualSense only) ─────────────────
            tp = getattr(gp, "touchpadPrimary", None)
            if callable(tp):
                tp = tp()
            if tp is not None:
                tx = tp.xAxis().value()
                ty = tp.yAxis().value()
                if abs(tx) > 0.01 or abs(ty) > 0.01:
                    print(f"\r  touchpad: x={tx:+.2f}  y={ty:+.2f}   ", end="", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        if ctrl is not None:
            _set_light(ctrl, 0, 0, 0)
        GCController.stopWirelessControllerDiscovery()
        print("\nGoodbye.")


if __name__ == "__main__":
    main()
