#!/usr/bin/env bash
# Verify the dev environment is in shape. Exit non-zero on any failure.
set -u
fail=0
MAMBA=/opt/homebrew/opt/micromamba/bin/micromamba
HF_CACHE="${HF_HOME:-$HOME/.cache/huggingface}/hub"

check() {
    local label="$1"; shift
    printf "  %-46s " "$label"
    if "$@" >/dev/null 2>&1; then echo "OK"; else echo "FAIL"; fail=1; fi
}

# ── System deps ───────────────────────────────────────────────────────────────
echo "System dependencies:"
check "ffmpeg (audio decode for STT)"     which ffmpeg
check "espeak-ng (Kokoro phonemizer)"     which espeak-ng

# ── ros_env ───────────────────────────────────────────────────────────────────
echo
echo "ros_env (micromamba — ROS bridge):"
check "env exists"              bash -c "$MAMBA env list | grep -q '^ros_env[[:space:]]'"
check "ros-noetic available"    $MAMBA run -n ros_env rosversion -d
check "fastapi installed"       $MAMBA run -n ros_env python -c 'import fastapi'
check "uvicorn installed"       $MAMBA run -n ros_env python -c 'import uvicorn'
check "redis client installed"  $MAMBA run -n ros_env python -c 'import redis'
check "pydantic installed"      $MAMBA run -n ros_env python -c 'import pydantic'
check "roller_eye built"        test -f catkin_ws/devel/setup.bash

# ── noir_env ──────────────────────────────────────────────────────────────────
echo
echo "noir_env (micromamba — vision / controller / mlx):"
check "env exists"              bash -c "$MAMBA env list | grep -q '^noir_env[[:space:]]'"
check "torch installed"         $MAMBA run -n noir_env python -c 'import torch'
check "ultralytics installed"   $MAMBA run -n noir_env python -c 'import ultralytics'
check "insightface installed"   $MAMBA run -n noir_env python -c 'import insightface'
check "pyobjc installed"        $MAMBA run -n noir_env python -c 'import GameController'
check "redis client installed"  $MAMBA run -n noir_env python -c 'import redis'
check "vllm-mlx installed"      $MAMBA run -n noir_env python -c 'import vllm_mlx'
check "parakeet-mlx installed"  $MAMBA run -n noir_env python -c 'import parakeet_mlx'
check "mlx-audio installed"     $MAMBA run -n noir_env python -c 'import mlx_audio'
check "soundfile installed"     $MAMBA run -n noir_env python -c 'import soundfile'
check "huggingface_hub avail."  $MAMBA run -n noir_env python -c 'from huggingface_hub import snapshot_download'

# ── HF model cache ────────────────────────────────────────────────────────────
echo
echo "HuggingFace model cache (run 'make pull-models' if missing):"
check "Qwen3-VL-2B-Instruct-4bit" test -d "${HF_CACHE}/models--mlx-community--Qwen3-VL-2B-Instruct-4bit"
check "parakeet-tdt-0.6b-v3"      test -d "${HF_CACHE}/models--mlx-community--parakeet-tdt-0.6b-v3"
check "Kokoro-82M-4bit"            test -d "${HF_CACHE}/models--mlx-community--Kokoro-82M-4bit"

# ── Robot reachability ────────────────────────────────────────────────────────
echo
echo "Robot reachability:"
check "linaro-alip resolves"    getent hosts linaro-alip
check "robot ping (10.42.0.1)" ping -c1 -t1 10.42.0.1
check "Mac IP is 10.42.0.181"  bash -c "ifconfig | grep -q 'inet 10.42.0.181'"

# ── Services (if stack is running) ────────────────────────────────────────────
echo
echo "Services (if stack is running):"
check "Redis on :6380"          bash -c "printf 'PING\r\n' | nc -w1 localhost 6380 | grep -q PONG"
check "vllm-mlx on :8000"       curl -sf http://localhost:8000/v1/models
check "Bridge on :8012"         curl -sf http://localhost:8012/status
check "Audio sidecar on :8014"  curl -sf http://localhost:8014/audio/health

if [[ $fail -ne 0 ]]; then
    echo
    echo "Some checks failed."
    echo "  System deps missing → make setup-system"
    echo "  ros_env issues      → make sync-bridge  or  make build-bridge"
    echo "  noir_env issues     → make sync"
    echo "  Models missing      → make pull-models"
    exit 1
fi
echo
echo "All good."
