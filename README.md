# Noir Rover

Autonomous AI rover system built on a **Moorebot Scout** omnidirectional robot. All compute runs natively on a MacBook Air M4 — no cloud, no heavy server.

## What it does

| Capability | How |
|---|---|
| Live camera feed + MJPEG stream | ROS `/CoreNode/jpg` → bridge API |
| Object detection overlay | YOLOE on MPS (Apple Silicon GPU) |
| Scene description | VLM (Qwen2.5-VL via Ollama) every 2 s |
| Face recognition | InsightFace (`buffalo_l`), enroll by dropping a JPEG in `faces/` |
| Manual drive | Xbox / PS5 over Bluetooth, or WASD in the dashboard |
| Velocity safety arbiter | Magnitude clamp — blocks commands exceeding ±1.5 m/s or ±12 rad/s |
| Memory stream | Debounced detection log, rendered in the dashboard |

---

## Hardware

- **Moorebot Scout** robot (ARM SBC running ROS Melodic at `10.42.0.1`)
- **MacBook Air M4** (or any Apple Silicon Mac) connected to the robot via USB-C Ethernet
- Xbox or PS5 DualSense controller over Bluetooth (optional)

---

## Software prerequisites

Install these once:

```bash
# Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# micromamba — conda package manager (used for all Python envs)
brew install micromamba

# Tilt — dev orchestrator (optional but recommended)
brew install tilt

# Ollama — local LLM runtime (for VLM scene description)
# Install from https://ollama.ai then pull the model:
ollama pull qwen3-vl:2b-instruct
```

---

## First-time setup

### 1. RoboStack ROS Noetic environment

ROS Noetic runs natively on macOS via RoboStack (no Docker needed for ROS).

```bash
micromamba create -n ros_env -c conda-forge -c robostack-staging \
  ros-noetic-ros-base python=3.11 --yes

```

### 2. Build the roller_eye message package

The robot uses custom ROS messages. Build them once (re-run after cloning or after changing `.msg`/`.srv` files):

```bash
make build-bridge
```

This also installs the pinned bridge Python deps (`ros-noetic/requirements.txt`) into `ros_env`. It creates `catkin_ws/` with the generated Python message classes. The directory is gitignored.

### 3. Python dependencies (vision, facerec, controller)

```bash
make sync    # micromamba env update -f environment.yml  →  creates noir_env
```

### 4. Network — add the robot's hostname

The robot's ROS nodes advertise themselves as `linaro-alip`. Add it to `/etc/hosts` so macOS can resolve it:

```bash
sudo sh -c 'echo "10.42.0.1 linaro-alip" >> /etc/hosts'
```

> This is a one-time step. Without it, sensor and camera data will not flow (the robot can't complete the TCPROS handshake back to macOS).

### 5. Enroll faces (optional)

Drop a clear face JPEG named after the person into `faces/`:

```
faces/Alice.jpg
faces/Bob.jpg
```

The face recognition service picks them up automatically on next start.

---

## Running the stack

```bash
tilt up
```

Open the Tilt UI at `http://localhost:10350` to see all services. The dashboard loads at `http://localhost:8013`.

### Service startup order

```
noir-redis → noir-redis-proxy
                              ↓
                           bridge (ROS node /scout_api starts here)
                              ↓
                         bridge-ready
                    ↙         ↓         ↘
               vision        vlm      facerec
                    ↘         ↓         ↙
                         controller
                         dashboard
```

### Fallback (no Tilt)

```bash
make dev        # starts Redis + bridge + vision in one terminal
make logs       # tail all service logs
make stop       # stop everything
```

---

## Architecture

Everything runs natively on macOS. Docker is used only for Redis.

```
MacBook Air M4 (all native)               Docker VM
┌────────────────────────────────┐        ┌─────────────────────────┐
│                                │        │ noir-redis  (port 6379) │
│  ros-noetic/bridge-api.py      │◄─6380─►│ noir-redis-proxy        │
│  ros-noetic/scoutros.py        │        └─────────────────────────┘
│    ROS node: /scout_api        │
│    port 8012 (FastAPI)         │
│    port 11323 (ROS XMLRPC)     │
│                                │
│  vision/app.py   (YOLOE/MPS)   │
│  vision/vlm.py   (Ollama VLM)  │
│  vision/facerec.py             │
│  controllers/driver.py         │
│  dashboard/  (:8013)           │
└───────────────┬────────────────┘
                │  ROS TCP — direct, no proxies
                ▼
   Scout robot at 10.42.0.1 (ROS Melodic)
   /CoreNode, /MotorNode, /SensorNode, ...  ← stock robot firmware
```

**The only ROS node this repo creates is `/scout_api`.** All other nodes in the ROS graph (`/CoreNode`, `/MotorNode`, `/SensorNode`, `/NavPathNode`, etc.) are stock firmware running on the robot's SBC.

---

## Key environment variables

| Variable | Default | Purpose |
|---|---|---|
| `ROS_MASTER_URI` | `http://10.42.0.1:11311` | ROS master on the robot |
| `ROS_IP` | `10.42.0.181` | Mac's IP on the robot subnet — update if it changes |
| `XMLRPC_PORT` | `11323` | Fixed ROS XMLRPC port — keeps URI stable across restarts |
| `REDIS_URL` | `redis://localhost:6380` | Redis (all native services) |
| `BRIDGE_URL` | `http://localhost:8012` | Bridge API (vision, controller, dashboard) |
| `VLM_MODEL` | `qwen2.5vl:3b` | Ollama model for scene description |

---

## Bridge API — port 8012

| Endpoint | Method | Purpose |
|---|---|---|
| `/status` | GET | ROS connection + camera status |
| `/camera/frame` | GET | Latest JPEG |
| `/camera/stream` | GET | MJPEG stream (15 fps) |
| `/snapshot` | GET | Latest frame, vision-annotated if available |
| `/sensors` | GET | ToF distance, IMU, VIO odometry, battery |
| `/perception/detections` | GET | YOLOE object detections from Redis |
| `/vlm/description` | GET | Latest VLM scene description (TTL 30 s) |
| `/faces/detections` | GET | Face recognition results (TTL 10 s) |
| `/robot/move` | POST | `{x, y, rotate, duration_ms}` — Twist with safety arbiter |
| `/robot/stop` | POST | Zero velocity |
| `/move/action` | POST | Timed move via `algo_action` ROS service |
| `/move/distance` | POST | Distance-based move via `algo_move` |
| `/move/rotate` | POST | Angle-based rotation via `algo_roll` |
| `/look_around` | POST | 360° rotation capturing N frames |
| `/nav/paths` | GET | Saved navigation paths |
| `/nav/patrol/start` | POST | Start path patrol by name |
| `/nav/patrol/stop` | POST | Stop patrol |
| `/nav/cancel` | POST | Cancel navigation |
| `/nav/status` | GET | NavPathNode status |
| `/nav/path/save` | POST | Save current path |
| `/safety/state` | GET | Controller activity + last move age |
| `/memory/recent` | GET | Latest N detection events from memory stream |

### Safety arbiter

All move commands are blocked if any axis exceeds: `|x| > 1.5`, `|y| > 1.5`, `|rotate| > 12`.

### Twist axis convention

```
linear.x  = strafe left/right
linear.y  = forward/backward
angular.z = rotation  (+ = clockwise)
```

---

## Controller (Xbox / PS5)

Hold **L2** = precision mode (slow, fine control).  
Hold **R2** = boost mode (full speed).  
No trigger = base speed.

| Mode | Linear | Rotation |
|---|---|---|
| Base | 0.25 m/s | 4.0 rad/s |
| Boost (R2) | 1.4 m/s | 10.0 rad/s |
| Precision (L2) | 0.22 m/s | 3.0 rad/s |

PS5 lightbar: purple = precision, orange = boost, green = base.

---

## Face recognition

Enroll a face by placing `faces/<Name>.jpg` in the repo root. The `facerec` service loads all images at startup. Recognition threshold: 0.35 cosine similarity (InsightFace `buffalo_l` model).

---

## Redis schema

| Key | Type | TTL | Written by |
|---|---|---|---|
| `xbox:last_input_ts` | string | 5 s | `controllers/driver.py` |
| `vision:latest` | JSON | 2 s | `vision/app.py` |
| `vision:thumb:{frame_id}` | base64 JPEG | 60 s | `vision/app.py` |
| `memory:events` | stream | — | `vision/memory.py` |
| `vlm:latest` | JSON | 30 s | `vision/vlm.py` |
| `face:latest` | JSON | 10 s | `vision/facerec.py` |

---

## Troubleshooting

**Camera and sensors show nothing after startup**
Run `ping linaro-alip` — if it fails, add `10.42.0.1 linaro-alip` to `/etc/hosts` (see setup step 4).

**`make build-bridge` fails with "CMake < 3.5" error**
The flag `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` in the Makefile handles this. If running `catkin_make` manually, add that flag yourself.

**Bridge fails to start — "roller_eye not found"**
The catkin workspace hasn't been built yet. Run `make build-bridge`.

**Mac's IP changed (not 10.42.0.181 anymore)**
Update `ROS_IP` in `Tiltfile` (line with `serve_cmd` for `bridge`) and `scripts/start_bridge.sh`. Check current IP with `ifconfig | grep 10.42.`.

**Robot movement feels laggy**
Check ping latency: `ping 10.42.0.1`. Anything above ~5 ms on USB Ethernet indicates a cable or adapter issue. Normal should be < 1 ms.

**Stuck on setup?**
Run `make doctor` to check the environment — it verifies the ros_env conda packages, catkin build, robot reachability, and running services.
