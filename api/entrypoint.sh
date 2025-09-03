#!/bin/sh
set -e

# Default port to 6969 if not set
PORT="${PORT:-6969}"

uvicorn bridge_api:app --host 0.0.0.0 --port "$PORT"