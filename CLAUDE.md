# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Noir-rover is an autonomous AI rover system built on a **Moorebot Scout** omnidirectional robot. It combines:
- A native macOS ROS Noetic bridge (via RoboStack) connecting to the robot's SBC at `10.42.0.1`
- Native macOS ML services (YOLOE vision, VLM, face recognition)
- Manual controller support (Xbox / PS5 over Bluetooth)

## Running the stack

```bash
# First-time setup: build the catkin workspace for roller_eye messages
make build-bridge    # requires micromamba + ros_env (RoboStack)

# Create / update noir_env conda environment (vision, facerec, controller deps)
make sync            # micromamba env update -f environment.yml

# Full stack with live UI
tilt up

# Fallback: plain bash (no Tilt UI)
make dev             # or: bash scripts/dev.sh
```

## Network topology

All services run natively on macOS or in Docker for Redis only.

| Service | Port | Notes |
|---|---|---|
| Bridge API | `:8012` | Runs natively; all rover control + perception endpoints |
| Redis | `:6380` | Proxied from Docker VM via noir-redis-proxy |

Native macOS services use `BRIDGE_URL=http://localhost:8012` and `REDIS_URL=redis://localhost:6380`.

## Key environment variables (bridge)

| Variable | Value | Purpose |
|---|---|---|
| `ROS_MASTER_URI` | `http://10.42.0.1:11311` | ROS master on robot |
| `ROS_IP` | `10.42.0.181` | Mac's IP on robot subnet (update if IP changes) |
| `XMLRPC_PORT` | `11323` | Fixed XMLRPC port — stable URI prevents master "same name" bump on restart |
| `REDIS_URL` | `redis://localhost:6380` | Redis connection |

## Architecture

```
MacBook Air M4 (all native)             Docker VM (Linux containers)
┌──────────────────────────┐            ┌────────────────────────────┐
│ ros-noetic/bridge-api.py │            │ noir-redis-proxy:6380→6379 │
│ ros-noetic/scoutros.py   │──:6380 ──► │                            │
│   (RoboStack ROS Noetic) │            │ noir-redis        host mode │
│                          │            └────────────────────────────┘
│ vision/app.py  (YOLOE)   │
│ vision/vlm.py  (VLM)     │
│ vision/facerec.py        │
│ controllers/driver.py    │
└──────────┬───────────────┘
           │  ROS TCP (direct — no proxy)
           ▼
Scout robot at 10.42.0.1
```

## RoboStack setup (one-time)

```bash
brew install micromamba
micromamba create -n ros_env -c conda-forge -c robostack-staging \
  ros-noetic-ros-base python=3.11 --yes
make build-bridge    # also runs sync-bridge (installs fastapi/uvicorn/redis/pydantic into ros_env)
```

The `catkin_ws/` directory is gitignored. `make build-bridge` creates it from `ros-noetic/roller_eye/`.

## Module responsibilities

### Bridge (ros-noetic/) — runs natively via RoboStack micromamba
- **`scoutros.py`** — Only ROS-touching code. Publishes Twist on `/cmd_vel`; subscribes to camera, ToF, IMU, VIO, battery. Service wrappers for `algo_action`, `algo_move`, `algo_roll`, nav.
- **`bridge-api.py`** — FastAPI bridge (v4). Magnitude-clamp safety arbiter, all endpoints. Uses Redis for vision state and Xbox activity tracking. Runs on port 8012.

### Native macOS (Python 3.11+)
- **`vision/app.py`** — YOLOE on MPS. Publishes `vision:latest` JSON and `vision:thumb:{id}` to Redis. Runs `vision/memory.py` as thread.
- **`vision/memory.py`** — Debounced detection writer to `memory:events` Redis stream.
- **`vision/vlm.py`** — VLM scene description. Polls `vision:latest` every 2s, calls Ollama (`qwen2.5vl:3b`), stores result in `vlm:latest` (TTL 30s). Set `VLM_MODEL` env to override model.
- **`vision/facerec.py`** — Face recognition using InsightFace (`buffalo_l`). Polls `vision:latest`, detects + identifies faces against enrolled images in `faces/`, publishes to `face:latest` (TTL 10s). Enrolled images: `faces/<Name>.jpg`. Threshold: 0.35 cosine similarity.
- **`controllers/driver.py`** — Xbox / PS5 controller loop via GameController.framework at 60 Hz. Stamps `xbox:last_input_ts` in Redis on input.

## Bridge API endpoints (port 8012)

| Endpoint | Purpose |
|---|---|
| `GET /status` | ROS connection + camera status |
| `GET /camera/frame` | Latest JPEG from `/CoreNode/jpg` |
| `GET /camera/stream` | MJPEG stream (15 fps) |
| `GET /snapshot` | Latest frame (vision-annotated if vision service is running) |
| `GET /sensors` | ToF distance, IMU, VIO odometry, battery |
| `GET /perception/detections` | YOLOE detections from Redis (empty if vision service down) |
| `POST /robot/move` | Twist publish with magnitude clamp arbiter |
| `POST /robot/stop` | Zero Twist |
| `POST /move/action` | `algo_action` service (speed + duration_ms) |
| `POST /move/distance` | `algo_move` service (distance in m) |
| `POST /move/rotate` | `algo_roll` service (angle in rad) |
| `POST /look_around` | 360° rotation, captures N frames |
| `GET /nav/paths` | List saved nav paths |
| `POST /nav/patrol/start` | Start NavPathNode patrol by name |
| `POST /nav/patrol/stop` | Stop patrol |
| `POST /nav/cancel` | Cancel navigation |
| `GET /nav/status` | NavPathNode status code |
| `POST /nav/path/save` | Save current path |
| `GET /vlm/description` | Latest VLM scene description (TTL 30s) |
| `GET /faces/detections` | Latest face recognition results (TTL 10s) |
| `GET /safety/state` | Controller activity + last move age |
| `GET /memory/recent` | Latest N detection events from memory stream |

## Safety arbiter

All move requests are blocked when magnitude exceeds limits: `|x|>1.5`, `|y|>1.5`, `|rotate|>12`.

## Twist axis mapping

```
linear.x  = strafe (left/right)
linear.y  = forward/backward
angular.z = rotation (+ = clockwise)
```

## Redis schema

| Key | Type | TTL | Writer |
|---|---|---|---|
| `xbox:last_input_ts` | string | 5s | `controllers/driver.py` |
| `vision:latest` | JSON string | 2s | `vision/app.py` |
| `vision:thumb:{frame_id}` | base64 JPEG | 60s | `vision/app.py` |
| `vision:events` (pubsub) | channel | — | `vision/app.py` |
| `memory:events` | Redis stream | — | `vision/memory.py` |
| `vlm:latest` | JSON {text,ts,frame_id} | 30s | `vision/vlm.py` |
| `face:latest` | JSON {faces,ts,frame_id,frame_w,frame_h} | 10s | `vision/facerec.py` |

## Custom ROS messages (roller_eye package)

`ros-noetic/roller_eye/` defines 9 message types and 29 service types for the Scout's proprietary API including:
- `frame.msg` — multiplexed A/V (JPG=1, H264=0, AAC=2)
- `detect.msg` — object detection result
- `algo_action.srv`, `algo_move.srv`, `algo_roll.srv` — motion services
- `nav_patrol.srv`, `nav_list_path.srv`, etc. — navigation services

Python classes are generated into `catkin_ws/` by `make build-bridge`.
