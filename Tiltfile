# Tiltfile — noir-rover dev environment
# Usage: tilt up
#
# Labels:
#   infra  — Docker-based services (Redis)
#   native — macOS-native services (bridge, vision, controller)

# ── Docker Compose (infra) ────────────────────────────────────────────────────
docker_compose('docker-compose.yml')

dc_resource('noir-redis',        labels=['infra'])
dc_resource('noir-redis-proxy',  labels=['infra'], resource_deps=['noir-redis'])

# ── Native macOS services ─────────────────────────────────────────────────────

# Bridge runs natively so ROS XMLRPC/TCPROS bind on Mac's real IP (10.42.0.181),
# directly reachable from the robot — no socat proxies needed.
local_resource(
    'bridge',
    serve_cmd='ROS_MASTER_URI=http://10.42.0.1:11311 ROS_IP=10.42.0.181 REDIS_URL=redis://localhost:6380 OLLAMA_URL=http://localhost:11434 AGENT_MODEL=qwen3-vl:2b-instruct bash scripts/start_bridge.sh',
    deps=['ros-noetic/bridge-api.py', 'ros-noetic/scoutros.py', 'scripts/start_bridge.sh'],
    labels=['native'],
    resource_deps=['noir-redis-proxy'],
)

# One-shot readiness gate — blocks native resources until bridge responds
local_resource(
    'bridge-ready',
    cmd='bash -c "until curl -sf http://localhost:8012/status > /dev/null 2>&1; do sleep 1; done"',
    resource_deps=['bridge'],
    labels=['ready-check'],
)

# Ollama is managed by the macOS menu-bar app — do not call 'ollama serve'.
# This gate simply waits until the already-running Ollama is reachable.
local_resource(
    'ollama-ready',
    cmd='bash -c "until curl -sf http://localhost:11434/api/version > /dev/null 2>&1; do sleep 1; done"',
    labels=['ready-check'],
)

local_resource(
    'vision',
    serve_cmd='REDIS_URL=redis://localhost:6380 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/app.py',
    deps=['vision/app.py', 'vision/memory.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

local_resource(
    'vlm',
    serve_cmd='REDIS_URL=redis://localhost:6380 OLLAMA_URL=http://localhost:11434 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/vlm.py',
    deps=['vision/vlm.py'],
    labels=['native'],
    resource_deps=['bridge-ready', 'ollama-ready'],
)

local_resource(
    'facerec',
    serve_cmd='REDIS_URL=redis://localhost:6380 FACEREC_INTERVAL=0.4 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/facerec.py',
    deps=['vision/facerec.py', 'faces/'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# Controller — Xbox or PS5 over BT → bridge API. Waits quietly when no controller is paired.
local_resource(
    'controller',
    serve_cmd='BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u controllers/driver.py',
    deps=['controllers/driver.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# Dashboard — static HTML served on :8013, connects to bridge at localhost:8012
local_resource(
    'dashboard',
    serve_cmd='bash -c "lsof -ti:8013 | xargs kill -9 2>/dev/null || true; python3 -m http.server 8013 --directory dashboard"',
    deps=['dashboard/'],
    labels=['native'],
    resource_deps=['bridge-ready'],
    links=['http://localhost:8013'],
)
