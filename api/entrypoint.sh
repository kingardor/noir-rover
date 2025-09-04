#!/bin/sh
set -e

PORT="${PORT:-6969}"

uvicorn bridge_api:app --host 0.0.0.0 --port "$PORT"
