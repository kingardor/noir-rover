#!/usr/bin/env python3
"""
Audio sidecar — port 8014

POST /audio/stt   multipart file  → {"text": "...", "duration_ms": N}
POST /audio/tts   {"text": "...", "voice": "af_heart"}  → audio/wav
GET  /audio/health → {"stt_ready": bool, "tts_ready": bool}
"""
import io
import os
import tempfile
import time
from contextlib import asynccontextmanager

import numpy as np
from fastapi import FastAPI, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
from pydantic import BaseModel

STT_MODEL = os.getenv("STT_MODEL", "mlx-community/parakeet-tdt-0.6b-v3")
TTS_MODEL = os.getenv("TTS_MODEL", "mlx-community/Kokoro-82M-4bit")
TTS_VOICE = os.getenv("TTS_VOICE", "af_heart")

_stt_model = None
_tts_ready = False


def _load_stt():
    global _stt_model
    print(f"[audio] loading STT {STT_MODEL}…", flush=True)
    from parakeet_mlx import from_pretrained
    _stt_model = from_pretrained(STT_MODEL)
    print("[audio] STT ready", flush=True)


def _load_tts():
    global _tts_ready
    print(f"[audio] loading TTS {TTS_MODEL}…", flush=True)
    # Import to trigger model download / cache warm; actual generation is lazy
    import mlx_audio.tts  # noqa: F401
    _tts_ready = True
    print("[audio] TTS ready", flush=True)


@asynccontextmanager
async def lifespan(app: FastAPI):
    _load_stt()
    _load_tts()
    yield


app = FastAPI(title="Noir Audio", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/audio/health")
def health():
    return {"stt_ready": _stt_model is not None, "tts_ready": _tts_ready}


@app.post("/audio/stt")
async def stt(file: UploadFile):
    if _stt_model is None:
        raise HTTPException(503, "STT model not ready")

    data = await file.read()
    suffix = "." + (file.filename or "audio.webm").rsplit(".", 1)[-1]

    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp.write(data)
        tmp_path = tmp.name

    try:
        t0 = time.monotonic()
        result = _stt_model.transcribe(tmp_path)
        elapsed_ms = int((time.monotonic() - t0) * 1000)
        text = getattr(result, "text", "") or ""
        text = text.strip()
        print(f"[audio/stt] {elapsed_ms}ms  {text[:80]!r}", flush=True)
        return {"text": text, "duration_ms": elapsed_ms}
    finally:
        os.unlink(tmp_path)


class TTSRequest(BaseModel):
    text: str
    voice: str = TTS_VOICE


@app.post("/audio/tts")
def tts(req: TTSRequest):
    if not _tts_ready:
        raise HTTPException(503, "TTS model not ready")

    from mlx_audio.tts.generate import generate
    import soundfile as sf

    print(f"[audio/tts] {req.text[:60]!r}", flush=True)
    t0 = time.monotonic()

    # generate() returns a numpy array of float32 samples at 24000 Hz
    audio: np.ndarray = generate(
        text=req.text,
        model_id=TTS_MODEL,
        voice=req.voice,
        speed=1.0,
        lang_code="a",
    )

    buf = io.BytesIO()
    sf.write(buf, audio, samplerate=24000, format="WAV", subtype="PCM_16")
    wav_bytes = buf.getvalue()

    elapsed_ms = int((time.monotonic() - t0) * 1000)
    print(f"[audio/tts] {elapsed_ms}ms  {len(wav_bytes)} bytes", flush=True)
    return Response(content=wav_bytes, media_type="audio/wav")


if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("AUDIO_PORT", "8014"))
    uvicorn.run("audio.server:app", host="0.0.0.0", port=port, workers=1)
