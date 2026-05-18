# Tiltfile — noir-rover dev environment
# Usage: tilt up
#
# Labels:
#   infra     — Docker-based services (Redis)
#   native    — macOS-native services (bridge, vision, controller)
#   test-mode — video loopback feed for offline development (no robot needed)
#
# Test mode (no robot required):
#   Set TEST_MODE=1 in .env — test-feed auto-starts, bridge skips ROS init,
#   camera endpoints read from Redis (test_feed.py writes recordings/test_feed.mp4).
#   KG writes to data/kg/graph_db_test so prod graph stays clean.

# ── Read TEST_MODE from .env so auto_init can be conditional ─────────────────
def _dotenv(key, default=''):
    content = str(read_file('.env', default=''))
    for line in content.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        k, _, v = line.partition('=')
        if k.strip() == key:
            return v.strip()
    return default

_test_mode = _dotenv('TEST_MODE') in ('1', 'true', 'yes')

# ── Docker Compose (infra) ────────────────────────────────────────────────────
docker_compose('docker-compose.yml')

dc_resource('noir-redis',        labels=['infra'])
dc_resource('noir-redis-proxy',  labels=['infra'], resource_deps=['noir-redis'])

# ── Native macOS services ─────────────────────────────────────────────────────

# mlx-vlm server — Qwen3-VL-2B on :8000 (text + vision).
# vllm-mlx was tried but crashes on Qwen3-VL (GPU stream thread issue in worker pool).
# mlx_vlm.server runs inference in the main thread and works correctly.
# To switch to OpenRouter for agent chat: set AGENT_PROVIDER=openrouter in the bridge serve_cmd.
local_resource(
    'vlm-server',
    serve_cmd='/opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -m mlx_vlm.server --model mlx-community/Qwen3-VL-2B-Instruct-4bit --port 8000',
    labels=['native'],
    resource_deps=['noir-redis-proxy'],
)

local_resource(
    'vlm-server-ready',
    cmd='bash -c "until curl -sf http://localhost:8000/v1/models > /dev/null 2>&1; do sleep 2; done"',
    resource_deps=['vlm-server'],
    labels=['ready-check'],
)

# Video loopback feed — reads recordings/test_feed.mp4 and writes frames to Redis.
# Auto-starts when TEST_MODE=1 in .env. Yields to real bridge frames if the
# robot comes online mid-session.
local_resource(
    'test-feed',
    serve_cmd='REDIS_URL=redis://localhost:6380 TEST_FPS=5 TEST_VIDEO=recordings/test_feed.mp4 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/test_feed.py',
    deps=['vision/test_feed.py'],
    labels=['test-mode'],
    resource_deps=['noir-redis-proxy'],
    auto_init=_test_mode,  # starts automatically when TEST_MODE=1 in .env
)

# Bridge runs natively so ROS XMLRPC/TCPROS bind on Mac's real IP (10.42.0.181),
# directly reachable from the robot — no socat proxies needed.
# Set TEST_MODE=1 (via .env or inline) to run without the robot.
local_resource(
    'bridge',
    serve_cmd='set -a; [ -f .env ] && . .env; set +a; ROS_MASTER_URI=http://10.42.0.1:11311 ROS_IP=10.42.0.181 REDIS_URL=redis://localhost:6380 AGENT_PROVIDER=mlx MLX_VLM_URL=http://localhost:8000 AGENT_MODEL=mlx-community/Qwen3-VL-2B-Instruct-4bit bash scripts/start_bridge.sh',
    deps=['ros-noetic/bridge-api.py', 'ros-noetic/scoutros.py', 'scripts/start_bridge.sh'],
    labels=['native'],
    resource_deps=['noir-redis-proxy', 'vlm-server-ready'],
)

# One-shot readiness gate — blocks native resources until bridge responds
local_resource(
    'bridge-ready',
    cmd='bash -c "until curl -sf http://localhost:8012/status > /dev/null 2>&1; do sleep 1; done"',
    resource_deps=['bridge'],
    labels=['ready-check'],
)

local_resource(
    'vision',
    serve_cmd='REDIS_URL=redis://localhost:6380 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/app.py',
    deps=['vision/app.py', 'vision/memory.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)


# Audio sidecar — Parakeet STT + Kokoro TTS on :8014
local_resource(
    'audio',
    serve_cmd='bash -c "lsof -ti:8014 | xargs kill -9 2>/dev/null || true; REDIS_URL=redis://localhost:6380 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u -m audio.server"',
    deps=['audio/server.py'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# One-shot gate: waits until both STT and TTS models have loaded inside the sidecar
local_resource(
    'audio-ready',
    cmd='bash scripts/wait_audio_ready.sh',
    resource_deps=['audio'],
    labels=['ready-check'],
)

local_resource(
    'facerec',
    serve_cmd='REDIS_URL=redis://localhost:6380 FACEREC_INTERVAL=0.5 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/facerec.py',
    deps=['vision/facerec.py', 'faces/'],
    labels=['native'],
    resource_deps=['bridge-ready'],
)

# kg — combined perception service: caption (→ vlm:latest) + knowledge graph (→ Kuzu).
# Replaces the old separate vlm.py and kg_builder.py services.
local_resource(
    'kg',
    serve_cmd='set -a; [ -f .env ] && . .env; set +a; REDIS_URL=redis://localhost:6380 MLX_VLM_URL=http://localhost:8000 VLM_MODEL=mlx-community/Qwen3-VL-2B-Instruct-4bit KG_SAMPLE_INTERVAL=3.0 KG_FRAMES=3 /opt/homebrew/opt/micromamba/bin/micromamba run -n noir_env python -u vision/kg_builder.py',
    deps=['vision/kg_builder.py', 'vision/kg_store.py'],
    labels=['native'],
    resource_deps=['bridge-ready', 'vlm-server-ready'],
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
    resource_deps=['bridge-ready', 'audio-ready'],
    links=['http://localhost:8013'],
)
