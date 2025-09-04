import os
import sys
import math
import signal
import threading
from dataclasses import dataclass
from evdev import InputDevice, ecodes, list_devices, ff

@dataclass
class Inputs:
    LX: float = 0.0   # used: strafe
    LY: float = 0.0   # used: forward
    RX: float = 0.0   # ignored
    RY: float = 0.0   # ignored
    RT: float = 0.0   # 0..1 (boost)
    LT: float = 0.0   # 0..1 (precision)
    LB: int = 0       # 0/1 rotate left
    RB: int = 0       # 0/1 rotate right

@dataclass
class State:
    forward: float = 0.0  # +forward
    strafe:  float = 0.0  # +right
    rotate:  float = 0.0  # +CW

def deadzone(v: float, dz: float) -> float:
    if abs(v) < dz: return 0.0
    s = math.copysign(1.0, v)
    t = (abs(v) - dz) / (1.0 - dz)
    return s * max(0.0, min(1.0, t))

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

class XboxBotDriver:
    STRAFE_SIGN = +1.0
    FORWARD_SIGN = +1.0
    ROTATE_SIGN = -1.0   # +right turn is positive
    FWD_TO_API = "y"
    RUMBLE_MS = 1000

    def __init__(self, device_hint=None, control_hz=60.0, speed_levels=None, trig_press=0.50, stick_dz=0.08, trig_dz=0.02, fwd_to_api=None):
        self.inputs  = Inputs()
        self.state   = State()
        self.lock    = threading.Lock()
        self.running = True

        self.DEVICE_HINT = device_hint
        self.CONTROL_HZ = control_hz
        self.SPEED_LEVELS = speed_levels or {
            "base":      (0.4,  4.0),   # default
            "boost":     (1.0, 10.0),   # RT pressed
            "precision": (0.15,  1.5),   # LT pressed
        }
        self.TRIG_PRESS = trig_press
        self.STICK_DZ = stick_dz
        self.TRIG_DZ = trig_dz
        self.FWD_TO_API = fwd_to_api or XboxBotDriver.FWD_TO_API

        self.dev = self._open_device()
        print(f"[ok] Using controller: '{self.dev.name}'")

        self.rumble_strong = self.dev.upload_effect(self._make_rumble(1.0, 0.9, -1))
        self.rumble_weak = self.dev.upload_effect(self._make_rumble(0.3, 0.2, -1))
        self._rt_down = False
        self._lt_down = False

        signal.signal(signal.SIGINT,  self._stop_signal)
        signal.signal(signal.SIGTERM, self._stop_signal)

    # ---- device handling ----
    def _open_device(self) -> InputDevice:
        if self.DEVICE_HINT and os.path.exists(self.DEVICE_HINT):
            try:
                return InputDevice(self.DEVICE_HINT)
            except PermissionError:
                sys.exit(f"Permission denied: {self.DEVICE_HINT}\nTry: sudo setfacl -m u:$USER:rw {self.DEVICE_HINT}")

        for path in list_devices():
            try:
                d = InputDevice(path)
            except Exception:
                continue
            name = (d.name or "").lower()
            if any(k in name for k in ("xbox", "microsoft", "controller", "gamepad")):
                try:
                    _ = d.capabilities(verbose=True)
                    return d
                except PermissionError:
                    pass

        sys.exit("No suitable game controller found. Set CONTROLLER_DEVICE=/dev/input/eventX (check permissions).")

    def _make_rumble(self, strong01, weak01, effect_id=-1):
        s = int(max(0, min(1, strong01)) * 0xFFFF)
        w = int(max(0, min(1, weak01)) * 0xFFFF)
        rumble = ff.Rumble(strong_magnitude=s, weak_magnitude=w)
        etype  = ff.EffectType(ff_rumble_effect=rumble)
        return ff.Effect(ecodes.FF_RUMBLE, effect_id, 0,
                        ff.Trigger(0,0), ff.Replay(self.RUMBLE_MS, 0), etype)
    
    def _stop_signal(self, *_):
        self.dev.write(ecodes.EV_FF, self.rumble_strong, 0)
        self.dev.write(ecodes.EV_FF, self.rumble_weak, 0)
        self.running = False

    # ---- normalization ----
    def _normalize_abs(self, code: int, value: int):
        name_raw = ecodes.ABS.get(code)
        if not name_raw:
            return None, None

        if   name_raw in ("ABS_X",):                 label = "LX"
        elif name_raw in ("ABS_Y",):                 label = "LY"
        elif name_raw in ("ABS_RX", "ABS_Z"):        label = "RX"
        elif name_raw in ("ABS_RY", "ABS_RZ"):       label = "RY"
        elif name_raw in ("ABS_BRAKE",):             label = "LT"
        elif name_raw in ("ABS_GAS",):               label = "RT"
        else:
            if name_raw.startswith("ABS_HAT"):       return None, None
            if name_raw in ("ABS_MISC","ABS_RUDDER"):return None, None
            return None, None

        try:
            info = self.dev.absinfo(code)
            vmin, vmax = info.min, info.max
        except Exception:
            vmin, vmax = (0, 255) if label in ("LT","RT") else (-32768, 32767)

        if vmax <= vmin:
            return label, 0.0

        if label in ("LT", "RT"):
            norm = (value - vmin) / float(vmax - vmin)
            norm = 0.0 if norm < self.TRIG_DZ else norm
            return label, clamp(norm, 0.0, 1.0)
        else:
            norm = (2.0 * (value - vmin) / float(vmax - vmin)) - 1.0
            if label in ("LY", "RY"): norm = -norm  # up=+
            norm = deadzone(norm, self.STICK_DZ)
            return label, clamp(norm, -1.0, 1.0)

    # ---- threads ----
    def start_event_and_control_loops(self):
        t_ev   = threading.Thread(target=self._event_loop,  name="events",  daemon=True)
        t_ctl  = threading.Thread(target=self._control_loop, name="control", daemon=True)
        t_ev.start()
        t_ctl.start()
        self._event_thread = t_ev
        self._control_thread = t_ctl

    def _event_loop(self):
        for ev in self.dev.read_loop():
            if not self.running: break

            if ev.type == ecodes.EV_ABS:
                label, val = self._normalize_abs(ev.code, ev.value)
                if label is None: continue
                with self.lock:
                    if   label == "LX": self.inputs.LX = val
                    elif label == "LY": self.inputs.LY = val
                    elif label == "RX": self.inputs.RX = val  # ignored
                    elif label == "RY": self.inputs.RY = val  # ignored
                    elif label == "LT": self.inputs.LT = val
                    elif label == "RT": self.inputs.RT = val

            elif ev.type == ecodes.EV_KEY:
                if ev.code == ecodes.BTN_TL:   # LB
                    with self.lock: self.inputs.LB = 1 if ev.value > 0 else 0
                elif ev.code == ecodes.BTN_TR: # RB
                    with self.lock: self.inputs.RB = 1 if ev.value > 0 else 0

    def _control_loop(self):
        dt = 1.0 / self.CONTROL_HZ
        while self.running:
            with self.lock:
                # --- latch trigger states ---
                rt_down = self.inputs.RT > self.TRIG_PRESS
                lt_down = self.inputs.LT > self.TRIG_PRESS

                # --- one-shot rumble on rising edge ---
                if lt_down and not self._lt_down:
                    self.dev.write(ecodes.EV_FF, self.rumble_weak, 1)
                elif rt_down and not self._rt_down:
                    self.dev.write(ecodes.EV_FF, self.rumble_strong, 1)

                # update latches
                self._lt_down = lt_down
                self._rt_down = rt_down

                # --- speed preset (no rumble writes here) ---
                # LT overrides RT if both pressed
                if lt_down:
                    fwdmax, rotmax = self.SPEED_LEVELS["precision"]
                elif rt_down:
                    fwdmax, rotmax = self.SPEED_LEVELS["boost"]
                else:
                    fwdmax, rotmax = self.SPEED_LEVELS["base"]

                # Apply per-axis limits
                base_fwd    = clamp(self.inputs.LY, -1.0, 1.0) * fwdmax
                base_strafe = clamp(self.inputs.LX, -1.0, 1.0) * fwdmax
                base_rot = ((1.0 if self.inputs.RB else 0.0)
                           - (1.0 if self.inputs.LB else 0.0)) * rotmax

                self.state.forward = base_fwd
                self.state.strafe  = base_strafe
                self.state.rotate  = base_rot

            threading.Event().wait(dt)

    def get_state(self):
        with self.lock:
            return self.state.forward, self.state.strafe, self.state.rotate

    def get_payload(self):
        with self.lock:
            fwd    = self.FORWARD_SIGN * self.state.forward
            strafe = self.STRAFE_SIGN  * self.state.strafe
            rot    = self.ROTATE_SIGN  * self.state.rotate
        fwd, strafe, rot = round(fwd, 4), round(strafe, 4), round(rot, 4)
        if self.FWD_TO_API == "y":
            return {"x": strafe, "y": fwd, "rotate": rot}
        elif self.FWD_TO_API == "x":
            return {"x": fwd, "y": strafe, "rotate": rot}
        return {"x": strafe, "y": fwd, "rotate": rot}

    def get_inputs(self):
        with self.lock:
            return self.inputs

    def stop(self):
        self.running = False
