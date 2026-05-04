# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Noir-rover is an autonomous AI rover system built on a **Moorebot Scout** omnidirectional robot. It combines:
- A ROS Noetic bridge (Docker) connecting to the robot's SBC at `10.42.0.1`
- Native macOS ML services (YOLOE vision, LLM agent, audio STT/TTS)
- Safety arbiter with Xbox e-stop

## Running the stack

```bash
# Install / sync all Python dependencies into .venv
make sync            # uv sync --all-groups

# Full stack with live UI
tilt up

# Fallback: plain bash (no Tilt UI)
make dev             # or: bash scripts/dev.sh

# Run a voice mission
make voice GOAL="find the red mug"

# Run follow-me / patrol
make follow
make patrol

# Run e-stop safety test suite
make e2e
```

## Network topology

Docker containers use `network_mode: host` inside Docker Desktop's Linux VM.
Two socat proxy containers forward VM ports to Mac's localhost:

| Service | Mac localhost | Purpose |
|---|---|---|
| Bridge API | `:8012` | All rover control + perception endpoints |
| Redis | `:6380` | State bus (vision, missions, memory) |

Native macOS services must use `BRIDGE_URL=http://localhost:8012` and `REDIS_URL=redis://localhost:6380`.

## Key environment variables (Docker container)

| Variable | Value | Purpose |
|---|---|---|
| `ROS_MASTER_URI` | `http://10.42.0.1:11311` | ROS master on robot |
| `ROS_IP` | `10.42.0.181` | Mac's IP on robot subnet (update if IP changes) |
| `RUN_NOIR` | `0` | Set to `1` to enable Xbox controller loop |
| `RUN_ROSBRIDGE` | `0` | Set to `1` to enable WebSocket bridge |

## Architecture

```
MacBook Air M4 (native Python)          Docker VM (Linux containers)
┌─────────────────────────┐             ┌────────────────────��───────┐
│ vision/app.py  (YOLOE)  │──localhost  │ noir-api-proxy  :8012→8011 │
│ agent/missions/*.py     │──:8012 ──►  │ noir-redis-proxy:6380→6379 │
│ audio/tts.py, stt.py    │──:6380 ──►  │                            │
│ mcp/server.py           │             │ noir-ros-noetic   host mode │
└─────────────────────────┘             │   bridge-api.py   :8011    │
                                        │   scoutros.py  (ROS)       │
                                        │ noir-redis        host mode │
                                        └────────────┬───────────────┘
                                                     │  ROS TCP
                                                     ▼
                                        Scout robot at 10.42.0.1
```

## Module responsibilities

### Docker (ros-noetic/)
- **`scoutros.py`** — Only ROS-touching code. Publishes Twist on `/cmd_vel`; subscribes to camera, ToF, IMU, VIO, battery. Service wrappers for `algo_action`, `algo_move`, `algo_roll`, nav.
- **`bridge-api.py`** — FastAPI bridge (v4). Safety arbiter, watchdog thread, all endpoints. Uses Redis for mission state and Xbox e-stop coordination.
- **`noir.py`** — Xbox controller loop (disabled by default). Stamps `xbox:last_input_ts` in Redis.
- **`xbox.py`** — `XboxBotDriver` reading evdev.

### Native macOS (Python 3.11+)
- **`vision/app.py`** — YOLOE on MPS. Publishes `vision:latest` JSON and `vision:thumb:{id}` to Redis. Runs `vision/memory.py` as thread.
- **`vision/memory.py`** — Debounced detection writer to `memory:events` Redis stream.
- **`agent/agent.py`** — ReAct tool-use loop. Hard caps: 10 steps / 30s / `done()`.
- **`agent/tools.py`** — All tool implementations (`look_around`, `detect`, `drive`, `stop`, `say`, `query_memory`, `done`). Also holds OpenAI tool schemas.
- **`agent/client.py`** — `get_client()` → OpenAI-compatible client for Ollama (default) or LM Studio. Model: `qwen3.5:2b-q4_K_M`.
- **`agent/missions/voice.py`** — Voice goal → agent loop.
- **`agent/missions/follow.py`** — Person follow-me PD controller (no LLM, vision events only).
- **`agent/missions/patrol.py`** — Patrol with VLM diff anomaly detection.
- **`audio/tts.py`** — Pops from `tts:queue` Redis list, speaks via `say` or Piper.
- **`audio/stt.py`** — Push-to-talk (default: F13) → mlx-whisper → POST `/mission/start`.
- **`mcp/server.py`** — MCP server exposing all rover tools to Claude Desktop / Claude Code.

## Bridge API endpoints (port 8011 / proxied to 8012)

| Endpoint | Purpose |
|---|---|
| `GET /status` | ROS connection + camera status |
| `GET /camera/frame` | Latest JPEG from `/CoreNode/jpg` |
| `GET /snapshot` | Latest frame (vision-annotated if vision service is running) |
| `GET /sensors` | ToF distance, IMU, VIO odometry, battery |
| `GET /perception/detections` | YOLOE detections from Redis (empty if vision service down) |
| `POST /robot/move` | Twist publish with arbiter (source: manual/agent/xbox) |
| `POST /robot/stop` | Zero Twist |
| `POST /move/action` | `algo_action` service (speed + duration_s) |
| `POST /move/distance` | `algo_move` service (distance in m) |
| `POST /move/rotate` | `algo_roll` service (angle in rad) |
| `POST /look_around` | 360° rotation, captures N frames |
| `GET /nav/paths` | List saved nav paths |
| `POST /nav/patrol/start` | Start NavPathNode patrol by name |
| `POST /nav/patrol/stop` | Stop patrol |
| `POST /nav/cancel` | Cancel navigation |
| `GET /nav/status` | NavPathNode status code |
| `POST /nav/path/save` | Save current path |
| `POST /agent/heartbeat` | Arbiter liveness (≥1 Hz required) |
| `POST /mission/start` | Acquire mission lock (voice/follow/patrol) |
| `POST /mission/stop` | Release mission lock |
| `GET /mission/state` | Current mode + goal |
| `POST /audio/speak` | Push text to `tts:queue` |
| `GET /safety/state` | Full arbiter diagnostic |

## Safety arbiter

All `source=agent` move requests are blocked when:
1. Xbox was active in the last 2 s (`xbox:last_input_ts` key)
2. Agent heartbeat is stale >1.5 s (`agent:heartbeat_ts` key)
3. Magnitude exceeds limits (|x|>1.5, |y|>1.5, |rotate|>12)

A watchdog thread (10 Hz) publishes zero Twist if a mission is active but no allowed move logged in 500 ms.

Run `make e2e` to verify all four safety cases before any physical demo.

## Twist axis mapping

```
linear.x  = strafe (left/right)
linear.y  = forward/backward
angular.z = rotation (+ = clockwise)
```

## Redis schema

| Key | Type | TTL | Writer |
|---|---|---|---|
| `xbox:last_input_ts` | string | 5s | `noir.py` |
| `agent:heartbeat_ts` | string | 5s | agent loop |
| `mission:active` | string | — | `/mission/start` |
| `mission:lock` | string | 5s | `/mission/start` |
| `mission:goal` | string | 1h | `/mission/start` |
| `vision:latest` | JSON string | 2s | `vision/app.py` |
| `vision:thumb:{frame_id}` | base64 JPEG | 60s | `vision/app.py` |
| `vision:events` (pubsub) | channel | — | `vision/app.py` |
| `memory:events` | Redis stream | — | `vision/memory.py` |
| `patrol:baseline:{pose_id}` | string (caption) | — | `patrol.py` |
| `tts:queue` | list | — | `/audio/speak` |

## Custom ROS messages (roller_eye package)

`roller_eye/` defines 9 message types and 29 service types for the Scout's proprietary API including:
- `frame.msg` — multiplexed A/V (JPG=1, H264=0, AAC=2)
- `detect.msg` — object detection result
- `algo_action.srv`, `algo_move.srv`, `algo_roll.srv` — motion services
- `nav_patrol.srv`, `nav_list_path.srv`, etc. — navigation services
