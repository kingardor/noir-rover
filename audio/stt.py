#!/usr/bin/env python3
"""
Push-to-talk STT using mlx-whisper (Apple Silicon only).
Triggered via Redis pub/sub: publish "start" / "stop" to stt:control channel.
The dashboard (or any client) calls POST /stt/start and /stt/stop on the bridge.

Whisper is lazy-loaded on first use and unloaded after WHISPER_IDLE_S seconds idle.
"""
import io
import os
import threading
import time
import wave

import numpy as np
import redis as redis_lib
import requests
import sounddevice as sd

BRIDGE_URL     = os.getenv("BRIDGE_URL",    "http://localhost:8012")
REDIS_URL      = os.getenv("REDIS_URL",     "redis://localhost:6380")
WHISPER_MODEL  = os.getenv("WHISPER_MODEL", "mlx-community/whisper-small-mlx")
SAMPLE_RATE    = 16000
CHANNELS       = 1
WHISPER_IDLE_S = float(os.getenv("WHISPER_IDLE_S", "30"))

_recording    = threading.Event()
_whisper      = None
_whisper_lock = threading.Lock()
_last_use_ts  = 0.0
_redis_client = None   # set in main(), shared with _dispatch


def _get_whisper():
    global _whisper, _last_use_ts
    with _whisper_lock:
        if _whisper is None:
            print("[stt] Loading Whisper...", flush=True)
            import mlx_whisper
            _whisper = mlx_whisper
            print("[stt] Whisper ready.", flush=True)
        _last_use_ts = time.time()
        return _whisper


def _unload_whisper_after_idle():
    global _whisper
    while True:
        time.sleep(10)
        with _whisper_lock:
            if _whisper is not None and (time.time() - _last_use_ts) > WHISPER_IDLE_S:
                _whisper = None
                print("[stt] Whisper unloaded (idle).", flush=True)


def _record() -> bytes:
    frames: list[np.ndarray] = []

    def cb(indata, frame_count, time_info, status):
        if _recording.is_set():
            frames.append(indata.copy())

    with sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS,
                        dtype="float32", callback=cb):
        while _recording.is_set():
            time.sleep(0.05)

    if not frames:
        return b""

    audio = np.concatenate(frames).flatten()
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes((audio * 32767).astype(np.int16).tobytes())
    return buf.getvalue()


def _transcribe(wav_bytes: bytes) -> str:
    import tempfile
    w = _get_whisper()
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        f.write(wav_bytes)
        tmp = f.name
    try:
        result = w.transcribe(tmp, path_or_hf_repo=WHISPER_MODEL)
        return (result.get("text") or "").strip()
    finally:
        os.unlink(tmp)


def _dispatch(text: str):
    if not text:
        return
    print(f"[stt] → {text!r}", flush=True)
    if _redis_client:
        try:
            _redis_client.set("stt:last_result", text, ex=300)
        except Exception:
            pass
    try:
        requests.post(f"{BRIDGE_URL}/mission/start",
                      json={"mode": "voice", "goal": text}, timeout=5)
    except Exception as e:
        print(f"[stt] Dispatch error: {e}", flush=True)


def _on_start():
    if _recording.is_set():
        return
    print("[stt] Recording...", flush=True)
    _recording.set()

    def _run():
        wav = _record()
        if wav:
            print("[stt] Transcribing...", flush=True)
            text = _transcribe(wav)
            _dispatch(text)

    threading.Thread(target=_run, daemon=True).start()


def _on_stop():
    if not _recording.is_set():
        return
    _recording.clear()
    print("[stt] Stopped recording.", flush=True)


def main():
    global _redis_client
    threading.Thread(target=_unload_whisper_after_idle, daemon=True).start()

    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    _redis_client = r
    pubsub = r.pubsub(ignore_subscribe_messages=True)
    pubsub.subscribe("stt:control")

    print(f"[stt] Ready — listening on Redis stt:control", flush=True)
    print(f"[stt] Bridge: {BRIDGE_URL}", flush=True)

    for msg in pubsub.listen():
        if msg["type"] != "message":
            continue
        cmd = (msg.get("data") or "").strip().lower()
        if cmd == "start":
            _on_start()
        elif cmd == "stop":
            _on_stop()


if __name__ == "__main__":
    main()
