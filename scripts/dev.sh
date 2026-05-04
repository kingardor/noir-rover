#!/usr/bin/env bash
# Fallback plain-bash launcher (prefer: tilt up)
# Boots Docker services then starts native macOS services.
# Ctrl-C stops everything cleanly.
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p logs

echo "[dev] Starting Docker services..."
docker compose up -d
echo "[dev] Waiting for bridge to become ready..."
until curl -sf http://localhost:8012/status > /dev/null 2>&1; do sleep 1; done
echo "[dev] Bridge is up."

pids=()
cleanup() {
  echo ""
  echo "[dev] Stopping native services..."
  for pid in "${pids[@]:-}"; do kill "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]:-}"; do wait "$pid" 2>/dev/null || true; done
  echo "[dev] Done."
}
trap cleanup EXIT INT TERM

echo "[dev] Starting vision service..."
BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 \
  uv run python -u vision/app.py >> logs/vision.log 2>&1 &
pids+=($!)

echo "[dev] Starting TTS service..."
REDIS_URL=redis://localhost:6380 \
  uv run python -u audio/tts.py >> logs/tts.log 2>&1 &
pids+=($!)

echo "[dev] Starting STT service..."
BRIDGE_URL=http://localhost:8012 \
  uv run python -u audio/stt.py >> logs/stt.log 2>&1 &
pids+=($!)

echo ""
echo "[dev] All services running. Logs in logs/"
echo "      vision PID=${pids[0]}  tts PID=${pids[1]}  stt PID=${pids[2]}"
echo "      Run: make logs   or   bash scripts/logs.sh"
echo "      Press Ctrl-C to stop."

wait
