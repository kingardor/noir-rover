#!/bin/sh
set -e

# Default port to 8000 if not set
PORT="${PORT:-8000}"

exec uvicorn bridge_api:app --host 0.0.0.0 --port "$PORT"
