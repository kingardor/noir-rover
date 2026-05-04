# Fallback Procfile for honcho/foreman (prefer: tilt up)
# Start with: honcho start
#
# BRIDGE_URL:  bridge API proxied to Mac's localhost via noir-api-proxy container
# REDIS_URL:   Redis proxied to Mac's localhost:6380 via noir-redis-proxy container

vision:    BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 uv run python -u vision/app.py
