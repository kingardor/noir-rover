#!/usr/bin/env python3
"""
TTS consumer — pops from Redis tts:queue and speaks.
Default engine: macOS `say` (zero dependencies).
Set TTS_ENGINE=piper and PIPER_BIN/PIPER_MODEL to use Piper.
"""
import os
import subprocess
import time

import redis as redis_lib

REDIS_URL   = os.getenv("REDIS_URL",    "redis://localhost:6379")
TTS_ENGINE  = os.getenv("TTS_ENGINE",   "say")        # "say" | "piper"
PIPER_BIN   = os.getenv("PIPER_BIN",    "piper")
PIPER_MODEL = os.getenv("PIPER_MODEL",  "en_US-lessac-medium")
SAY_VOICE   = os.getenv("SAY_VOICE",    "")           # e.g. "Samantha"; empty = system default


def _say(text: str):
    cmd = ["say"]
    if SAY_VOICE:
        cmd += ["-v", SAY_VOICE]
    cmd.append(text)
    subprocess.run(cmd, check=False)


def _piper(text: str):
    try:
        proc = subprocess.Popen(
            [PIPER_BIN, "--model", PIPER_MODEL, "--output-raw"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        audio, _ = proc.communicate(input=text.encode())
        subprocess.run(["afplay", "-"], input=audio, check=False, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"[tts] Piper failed ({e}), falling back to say.", flush=True)
        _say(text)


def speak(text: str):
    if TTS_ENGINE == "piper":
        _piper(text)
    else:
        _say(text)


def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    print(f"[tts] Ready (engine={TTS_ENGINE}). Waiting for messages...", flush=True)
    while True:
        try:
            item = r.blpop("tts:queue", timeout=5)
            if item:
                _, text = item
                print(f"[tts] Speaking: {text!r}", flush=True)
                speak(text)
        except redis_lib.RedisError as e:
            print(f"[tts] Redis error: {e}. Retrying in 2 s...", flush=True)
            time.sleep(2.0)
        except Exception as e:
            print(f"[tts] Error: {e}", flush=True)
            time.sleep(0.5)


if __name__ == "__main__":
    main()
