# Tiltfile — noir-rover dev environment
# Usage: tilt up
#
# Labels:
#   infra  — Docker-based services (Redis, ROS bridge, socat proxies)
#   native — macOS-native services requiring MPS/Metal (vision, audio, MCP)

# ── Docker Compose (infra) ────────────────────────────────────────────────────
docker_compose('docker-compose.yml')

dc_resource('noir-redis',       labels=['infra'])
dc_resource('noir-ros-noetic',  labels=['infra'])
dc_resource('noir-api-proxy',   labels=['infra'], resource_deps=['noir-ros-noetic'])
dc_resource('noir-redis-proxy', labels=['infra'], resource_deps=['noir-redis'])

# One-shot readiness gate — blocks native resources until bridge responds
local_resource(
    'bridge-ready',
    cmd='bash -c "until curl -sf http://localhost:8012/status > /dev/null 2>&1; do sleep 1; done"',
    resource_deps=['noir-api-proxy'],
    labels=['infra'],
)

# Deploy cmd_vel relay to the robot — idempotent, re-runs on relay file changes
local_resource(
    'relay-deploy',
    cmd='bash scripts/deploy_relay.sh',
    deps=['ros-noetic/cmd_vel_relay.py', 'ros-noetic/relay_protocol.py', 'scripts/deploy_relay.sh'],
    labels=['infra'],
)

# ── Native macOS services ─────────────────────────────────────────────────────
# Vision and STT must run natively: Docker Desktop on Mac cannot pass MPS/Metal
# through to containers. YOLOE on CPU is ~10x slower; mlx-whisper is Apple-only.
# Agent, TTS, and MCP share the same venv and are simpler to manage alongside them.

local_resource(
    'vision',
    serve_cmd='BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 uv run python -u vision/app.py',
    deps=['vision/'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

local_resource(
    'tts',
    serve_cmd='REDIS_URL=redis://localhost:6380 uv run python -u audio/tts.py',
    deps=['audio/tts.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

local_resource(
    'stt',
    serve_cmd='BRIDGE_URL=http://localhost:8012 uv run python -u audio/stt.py',
    deps=['audio/stt.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# Mission daemons — idle until their Redis mission:active mode is set by the dashboard/API
local_resource(
    'voice',
    serve_cmd='BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 uv run python -u -m agent.missions.voice',
    deps=['agent/missions/voice.py', 'agent/agent.py', 'agent/tools.py', 'agent/client.py'],
    labels=['native'],
    resource_deps=['bridge-ready', 'vision'],
)

local_resource(
    'follow',
    serve_cmd='BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 uv run python -u -m agent.missions.follow',
    deps=['agent/missions/follow.py'],
    labels=['native'],
    resource_deps=['bridge-ready', 'vision'],
)

local_resource(
    'patrol',
    serve_cmd='BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 uv run python -u -m agent.missions.patrol',
    deps=['agent/missions/patrol.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# MCP is a stdio server invoked by Claude Desktop/Code — not a long-running
# service. Configure it via mcp/main.py docstring instructions instead.
# local_resource('mcp', ...) intentionally omitted.

# Controller — Xbox or PS5 over BT → bridge API. Waits quietly when no controller is paired.
local_resource(
    'controller',
    serve_cmd='BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 uv run python -u controllers/driver.py',
    deps=['controllers/driver.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# Dashboard — static HTML served on :8013, connects to bridge at localhost:8012
local_resource(
    'dashboard',
    serve_cmd='python3 -m http.server 8013 --directory dashboard',
    deps=['dashboard/'],
    labels=['native'],
    resource_deps=['bridge-ready'],
    links=['http://localhost:8013'],
)
