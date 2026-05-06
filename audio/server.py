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
_tts_model = None


def _load_stt():
    global _stt_model
    print(f"[audio] loading STT {STT_MODEL}…", flush=True)
    from parakeet_mlx import from_pretrained
    _stt_model = from_pretrained(STT_MODEL)
    print("[audio] STT ready", flush=True)


def _load_tts():
    global _tts_model
    print(f"[audio] loading TTS {TTS_MODEL}…", flush=True)
    from mlx_audio.tts.utils import load_model
    _tts_model = load_model(TTS_MODEL)
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
    return {"stt_ready": _stt_model is not None, "tts_ready": _tts_model is not None}


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
async def tts(req: TTSRequest):
    if _tts_model is None:
        raise HTTPException(503, "TTS model not ready")

    import soundfile as sf

    print(f"[audio/tts] {req.text[:60]!r}", flush=True)
    t0 = time.monotonic()

    # model.generate() yields GenerationResult with .audio (1-D float array) and .sample_rate
    chunks = []
    sample_rate = 24000
    for result in _tts_model.generate(
        text=req.text,
        voice=req.voice,
        speed=1.0,
        lang_code="a",
    ):
        chunks.append(np.array(result.audio))
        sample_rate = result.sample_rate

    if not chunks:
        raise HTTPException(500, "TTS produced no audio")

    audio = np.concatenate(chunks) if len(chunks) > 1 else chunks[0]

    buf = io.BytesIO()
    sf.write(buf, audio, samplerate=sample_rate, format="WAV", subtype="PCM_16")
    wav_bytes = buf.getvalue()

    elapsed_ms = int((time.monotonic() - t0) * 1000)
    print(f"[audio/tts] {elapsed_ms}ms  {len(wav_bytes)} bytes", flush=True)
    return Response(content=wav_bytes, media_type="audio/wav")


if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("AUDIO_PORT", "8014"))
    uvicorn.run("audio.server:app", host="0.0.0.0", port=port, workers=1)
