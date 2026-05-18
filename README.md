# Noir Rover

Autonomous AI rover system built on a **Moorebot Scout** omnidirectional robot. All compute runs natively on a MacBook Air M4 — no cloud, no heavy server.

The rover continuously observes its environment through YOLOE object detection and a vision-language model, builds a live knowledge graph of what it sees, recognises enrolled faces, and exposes an **NOIR agent** you can chat with in plain English. The agent uses tool-calling to physically move the robot, describe the scene, and answer questions about what it has observed. Everything streams in real time through a web dashboard.

---

## What it does

| Capability | How |
|---|---|
| Live camera feed + MJPEG stream | ROS `/CoreNode/jpg` → bridge API `:8012` |
| Object detection overlay | YOLOE-11m-seg on MPS (Apple Silicon GPU) |
| VLM scene description | Qwen3-VL-2B-4bit via `mlx_vlm.server` every ~3 s |
| Knowledge graph | Kuzu graph DB, continuously updated from VLM output |
| Face recognition | InsightFace `buffalo_l`, enroll by dropping a JPEG in `faces/` |
| NOIR agent chat | LLM with 5 tool-calling tools for movement + vision; streams SSE |
| Manual drive | Xbox / PS5 DualSense over Bluetooth, or keyboard |
| Dashboard | Single-page web UI at `:8013` — camera, chat, knowledge graph |
| Voice I/O | Parakeet STT + Kokoro TTS via audio sidecar at `:8014` |
| Velocity safety arbiter | Blocks commands exceeding ±1.5 m/s or ±12 rad/s |

---

## Hardware

- **Moorebot Scout** robot (ARM SBC running ROS Melodic at `10.42.0.1`)
- **MacBook Air M4** (or any Apple Silicon Mac) connected to the robot via USB-C Ethernet
- Xbox or PS5 DualSense controller over Bluetooth (optional)

---

## Architecture

All compute runs natively on macOS. Docker is used only for Redis.

```
MacBook Air M4 (all native)
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  vlm-server (:8000)   mlx_vlm.server  Qwen3-VL-2B-Instruct-4bit│
│       ↑ /v1/chat/completions                                    │
│                                                                 │
│  vision/kg_builder.py    (KG build + VLM captions, noir_env)   │
│  vision/app.py           (YOLOE detection, noir_env)            │
│  vision/facerec.py       (face recognition, noir_env)           │
│  audio/server.py  (:8014)(STT/TTS sidecar, noir_env)           │
│  controllers/driver.py   (Xbox/PS5, noir_env)                  │
│                                                                 │
│  ros-noetic/bridge-api.py (:8012)  (FastAPI, ros_env)          │
│  ros-noetic/scoutros.py           (ROS node /scout_api)        │
│                                                                 │
│  dashboard/ (:8013)  (static HTML, Python http.server)         │
│                                                                 │
│  ↕ redis://localhost:6380                                       │
└─────────────────────────────────────────────────────────────────┘
        ↓                              Docker VM
        ↓                    ┌──────────────────────────┐
        └──── :6380 ────────►│ noir-redis-proxy → :6379 │
                             │ noir-redis (host net)     │
                             └──────────────────────────┘

ROS TCP — direct (no proxy)
        ↓
Scout robot at 10.42.0.1 (ROS Melodic)
/CoreNode  /MotorNode  /SensorNode  /NavPathNode  ← stock firmware
```

**The only ROS node this repo creates is `/scout_api`.** All other nodes in the ROS graph are stock firmware running on the robot's SBC.

### Service responsibilities

| Service | Path | Env | Reads | Writes |
|---|---|---|---|---|
| vlm-server | _(mlx_vlm module)_ | `noir_env` | — | HTTP `:8000/v1/chat/completions` |
| bridge | `ros-noetic/bridge-api.py` | `ros_env` | Redis, ROS topics | `camera:frame`, `camera:ts`, `kg:diary:*`, `agent:history`, `agent:follow_cfg` |
| scoutros | `ros-noetic/scoutros.py` | `ros_env` | ROS camera/sensors | `/cmd_vel` Twist |
| vision | `vision/app.py` | `noir_env` | `camera:frame` | `vision:latest`, `vision:thumb:*`, pubsub `vision:events` |
| memory | `vision/memory.py` | `noir_env` (thread in vision) | `vision:latest`, pubsub | `memory:events` stream |
| kg | `vision/kg_builder.py` | `noir_env` | `camera:frame`, `face:latest`, vlm-server | `vlm:latest`, `kg:snapshot`, `kg:last_update`, pubsub `kg:updated` |
| facerec | `vision/facerec.py` | `noir_env` | `camera:frame` | `face:latest` |
| audio | `audio/server.py` | `noir_env` | HTTP upload | HTTP `:8014` (WAV) |
| controller | `controllers/driver.py` | `noir_env` | — | HTTP `/robot/move`, `xbox:last_input_ts` |
| dashboard | `dashboard/index.html` | _(static)_ | HTTP `:8012`, `:8014` | — |

### Service ports

| Service | Port | Notes |
|---|---|---|
| Bridge API | `8012` | All rover control + perception endpoints |
| Dashboard | `8013` | Static HTML served by `python3 -m http.server` |
| Audio sidecar | `8014` | STT + TTS FastAPI |
| VLM server | `8000` | `mlx_vlm.server` OpenAI-compatible endpoint |
| Redis | `6380` | Proxied from Docker VM (host-side Redis is at `6379`) |
| Tilt UI | `10350` | Dev orchestrator UI |

### Startup order

```
noir-redis
    ↓
noir-redis-proxy
    ↓
vlm-server ───────────────────────────── [test-feed if TEST_MODE=1]
    ↓
vlm-server-ready (readiness probe)
    ↓
bridge
    ↓
bridge-ready (readiness probe)
    ↙     ↙      ↓       ↓       ↘
vision   kg   facerec  audio  controller
                          ↓
                    audio-ready (readiness probe)
                          ↓
                       dashboard
```

---

## Software prerequisites

Install these once before the first-time setup:

```bash
# Homebrew
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# micromamba — conda package manager (manages both ros_env and noir_env)
brew install micromamba

# Tilt — dev orchestrator
brew install tilt

# Docker Desktop (or OrbStack/Colima) — needed for the Redis container only
# https://orbstack.dev  or  https://www.docker.com/products/docker-desktop/

# System audio deps (for STT/TTS)
# (also run-able later via: make setup-system)
brew install ffmpeg espeak-ng
```

---

## First-time setup

### 1. Clone and create your `.env`

```bash
git clone <repo>
cd noir-rover
cp .env.example .env
# Edit .env — at minimum set ROS_IP to your Mac's address on the robot subnet
```

The robot subnet is the USB-C Ethernet interface to the Scout. Find your IP with:

```bash
ifconfig | grep 10.42.
```

### 2. RoboStack ROS Noetic environment

ROS runs natively on macOS via RoboStack — no Docker needed for ROS:

```bash
micromamba create -n ros_env -c conda-forge -c robostack-staging \
  ros-noetic-ros-base python=3.11 --yes
```

### 3. Install everything

```bash
make install       # sync + build-bridge + pull-models
make setup-system  # brew install ffmpeg espeak-ng
```

`make install` does three things:
- **`sync`** — creates the `noir_env` conda environment from `environment.yml` (vision, facerec, KG, audio MLX stack).
- **`build-bridge`** — builds the `roller_eye` custom ROS messages into `catkin_ws/devel/` and installs FastAPI/uvicorn into `ros_env`. Re-run after any change to `ros-noetic/roller_eye/*.msg` or `*.srv`.
- **`pull-models`** — downloads `Qwen3-VL-2B-Instruct-4bit`, `parakeet-tdt-0.6b-v3`, and `Kokoro-82M-4bit` from HuggingFace into the local model cache.

### 4. Add the robot's hostname to `/etc/hosts`

The robot's ROS nodes advertise themselves as `linaro-alip`. Without this entry, camera and sensor data will not flow (TCPROS handshake fails):

```bash
sudo sh -c 'echo "10.42.0.1 linaro-alip" >> /etc/hosts'
```

### 5. Enroll faces (optional)

Drop a clear face JPEG named after the person into `faces/`:

```
faces/Alice.jpg
faces/Bob.jpg
```

The face recognition service picks them up at startup. Recognised names are auto-linked into the knowledge graph.

---

## Running the stack

```bash
tilt up
```

- Tilt UI: `http://localhost:10350`
- Dashboard: `http://localhost:8013`

The bridge needs ~10–15 seconds to initialise ROS. The VLM server needs ~20–30 seconds to load the model on first start. The audio service loads both STT and TTS sequentially and runs a warmup pulse — this takes ~60 seconds.

### Fallback (without Tilt)

```bash
# Check environment health
make doctor

# Start the bridge manually (in ros_env)
bash scripts/start_bridge.sh

# Start other services manually (in noir_env)
micromamba run -n noir_env python -m mlx_vlm.server --model mlx-community/Qwen3-VL-2B-Instruct-4bit --port 8000
micromamba run -n noir_env python vision/app.py
micromamba run -n noir_env python vision/kg_builder.py
micromamba run -n noir_env python vision/facerec.py
micromamba run -n noir_env python -m audio.server
micromamba run -n noir_env python controllers/driver.py
python3 -m http.server 8013 --directory dashboard
```

---

## Test mode (no robot required)

Set `TEST_MODE=1` in `.env`, then run `tilt up`. The `test-feed` resource will start automatically.

In test mode:
- The bridge skips ROS init; camera frames come from Redis instead of the robot.
- Motion commands are no-ops (return `{"ok":true,"test_mode":true}`).
- Sensors return synthetic values.
- `vision/test_feed.py` reads `recordings/test_feed.mp4` in a loop at 5 fps and writes frames to `camera:frame` in Redis.
- The KG service redirects its database to `data/kg/graph_db_test` and images to `data/kg/images_test`, keeping the production graph clean.
- The test feed yields back to real bridge frames if the robot comes online mid-session.

---

## Configuration

All env vars have defaults. Copy `.env.example` to `.env` and override what you need.

### Bridge (`ros-noetic/bridge-api.py`)

| Variable | Default | Purpose |
|---|---|---|
| `ROS_MASTER_URI` | `http://10.42.0.1:11311` | ROS master on the robot |
| `ROS_IP` | `10.42.0.181` | Mac's IP on the robot subnet — update if it changes |
| `XMLRPC_PORT` | `11323` | Fixed ROS XMLRPC port — keeps URI stable across restarts |
| `REDIS_URL` | `redis://localhost:6380` | Redis connection |
| `TEST_MODE` | `0` | `1` = no robot; bridge uses Redis camera, synthetic sensors |
| `AGENT_PROVIDER` | `mlx` | `mlx` (local VLM) or `openrouter` (cloud) |
| `MLX_VLM_URL` | `http://localhost:8000` | mlx_vlm.server endpoint |
| `AGENT_MODEL` | `mlx-community/Qwen3-VL-2B-Instruct-4bit` | Local VLM model name |
| `OPENROUTER_MODEL` | `nvidia/nemotron-nano-12b-v2-vl:free` | OpenRouter model for agent |
| `OPENROUTER_API_KEY` | _(empty)_ | Required when `AGENT_PROVIDER=openrouter` |

### Knowledge Graph (`vision/kg_builder.py`)

| Variable | Default | Purpose |
|---|---|---|
| `MLX_VLM_URL` | `http://localhost:8000` | VLM server URL |
| `VLM_MODEL` | `mlx-community/Qwen3-VL-2B-Instruct-4bit` | Model name passed to vlm-server |
| `VLM_SIZE` | `256` | Frame resize (px square) before VLM call |
| `KG_SAMPLE_INTERVAL` | `3.0` | Seconds between frame samples |
| `KG_FRAMES` | `3` | Rolling buffer size (fires VLM when full + fresh) |
| `KG_FUZZY_OBJ_THRESHOLD` | `0.82` | Object dedup similarity threshold |
| `KG_FUZZY_EVT_THRESHOLD` | `0.78` | Event dedup similarity threshold |
| `KG_DB_DIR` | `data/kg/graph_db` | Kuzu database directory |
| `KG_IMAGES_DIR` | `data/kg/images` | KG image store directory |

### Vision (`vision/app.py`)

| Variable | Default | Purpose |
|---|---|---|
| `YOLOE_MODEL` | `ai/yoloe-11m-seg.pt` | YOLOE weights path |
| `YOLOE_LABELS` | _(16 common labels)_ | Comma-separated zero-shot detection labels |
| `VISION_INTERVAL` | `2.0` | Max seconds between detections (triggered by frame change) |

### Face recognition (`vision/facerec.py`)

| Variable | Default | Purpose |
|---|---|---|
| `FACES_DIR` | `faces` | Enrollment image directory |
| `FACEREC_INTERVAL` | `2.0` | Seconds between recognition runs (Tilt sets 0.5) |
| `FACEREC_THRESHOLD` | `0.35` | Cosine similarity match threshold (InsightFace) |

### Audio sidecar (`audio/server.py`)

| Variable | Default | Purpose |
|---|---|---|
| `STT_MODEL` | `mlx-community/parakeet-tdt-0.6b-v3` | Speech-to-text model |
| `TTS_MODEL` | `mlx-community/Kokoro-82M-4bit` | Text-to-speech model |
| `TTS_VOICE` | `af_heart` | Kokoro voice name |
| `AUDIO_PORT` | `8014` | Audio server port |

---

## Bridge API — port 8012

### Status / camera / sensors

| Method | Path | Purpose |
|---|---|---|
| GET | `/status` | ROS connection state + camera status (age, size) |
| GET | `/camera/frame` | Latest JPEG from `/CoreNode/jpg` |
| GET | `/camera/stream` | MJPEG multipart stream at 15 fps |
| GET | `/snapshot` | Latest frame, vision-annotated if available |
| GET | `/sensors` | ToF distance, IMU angular velocity, VIO odometry, battery |
| GET | `/perception/detections` | Latest YOLOE detections from Redis |
| GET | `/vlm/description` | Latest VLM scene caption from kg_builder (TTL 30 s) |
| GET | `/faces/detections` | Latest face recognition results (TTL 10 s) |

### Motion / safety

| Method | Path | Body | Purpose |
|---|---|---|---|
| POST | `/robot/move` | `{x, y, rotate, duration_ms}` | Twist publish with safety arbiter |
| POST | `/robot/stop` | — | Zero velocity |
| POST | `/move/action` | `{x_speed, y_speed, rotate_speed, duration_ms}` | Timed move via `algo_action` ROS service |
| POST | `/move/distance` | `{x_dist, y_dist, speed}` | Distance-based move via `algo_move` |
| POST | `/move/rotate` | `{angle_rad, speed_rad_s}` | Angle-based rotation via `algo_roll` |
| POST | `/look_around` | `{n_frames?}` | 360° rotation capturing N frames (4–16) |
| GET | `/safety/state` | — | Controller activity + last move age |

**Safety arbiter:** All `/robot/move` commands are blocked when any axis exceeds `|x|>1.5`, `|y|>1.5`, or `|rotate|>12.0`.

**Twist axis convention:**
```
linear.x  = strafe left/right
linear.y  = forward/backward
angular.z = rotation  (+ = clockwise)
```

### Agent / vision

| Method | Path | Body | Purpose |
|---|---|---|---|
| POST | `/agent/chat` | `{message}` | Chat with NOIR agent; streams SSE events |
| POST | `/agent/reset` | — | Clear conversation history from Redis |
| GET | `/agent/follow_status` | — | Current face-following config |
| POST | `/agent/follow` | `{on, target_name}` | Enable/disable face-following |
| POST | `/vision/describe` | `{question}` | One-shot VLM answer for a visual question |

### Navigation

| Method | Path | Purpose |
|---|---|---|
| GET | `/nav/paths` | List saved nav paths |
| POST | `/nav/patrol/start` | Start path patrol by name |
| POST | `/nav/patrol/stop` | Stop patrol |
| POST | `/nav/cancel` | Cancel navigation |
| GET | `/nav/status` | NavPathNode status code |
| POST | `/nav/path/save` | Save current path |

### Knowledge graph / memory

| Method | Path | Purpose |
|---|---|---|
| GET | `/kg/graph` | Full KG snapshot `{nodes,edges,counts}`. Optional `?since=unix_ts` |
| GET | `/kg/node/{id}` | Node detail + all attached image metadata |
| GET | `/kg/image/{img_id}` | Serve a KG image JPEG from disk |
| GET | `/kg/diary?date=YYYY-MM-DD` | NOIR-voiced diary entry for the day (cached 24 h) |
| POST | `/kg/reset` | Wipe the graph and all images |
| GET | `/memory/recent` | Latest N events from the detection memory stream |

---

## Agent chat

The NOIR agent is a dry, clipped AI personality that controls the robot via function-calling. It responds in 1–3 sentences, metric units, no markdown.

### Providers

Select via `AGENT_PROVIDER` in `.env`:

| Provider | Model | Notes |
|---|---|---|
| `mlx` (default) | `Qwen3-VL-2B-Instruct-4bit` | Runs locally via `mlx_vlm.server` on `:8000`. No API key. |
| `openrouter` | `nvidia/nemotron-nano-12b-v2-vl:free` | Cloud. Requires `OPENROUTER_API_KEY`. Stronger reasoning. |

### Tools (5 wired in)

| Tool | Parameters | What it does |
|---|---|---|
| `move_forward_back` | `distance_m` (req), `speed_m_s` (0.25) | Drives forward (+) or backward (–) |
| `strafe_left_right` | `distance_m` (req), `speed_m_s` (0.25) | Strafes right (+) or left (–) |
| `turn` | `angle_rad` (req), `speed_rad_s` (1.0) | Rotates; positive = clockwise |
| `stop` | — | Immediately stops all motion |
| `ask_about_scene` | `question` (req) | One-shot VLM answer using the current camera frame |

### SSE event stream

`POST /agent/chat` returns `text/event-stream`. Events arrive in order:

```
data: {"type":"tool_call_start","tc_id":"…","name":"move_forward_back","args":{…}}
data: {"type":"tool_call_result","tc_id":"…","result":"…"}
data: {"type":"reply","text":"Moving forward 0.5 m."}
```

The dispatcher runs up to **3 tool-call iterations** per chat turn. Conversation history persists in Redis (`agent:history`, last 20 messages, 30-minute TTL).

### Face following

`POST /agent/follow {"on":true,"target_name":"Alice"}` enables a 5 Hz proportional controller that keeps the named face centred in frame. It pauses for 1.5 s after any physical controller input. When the target face is not visible, the robot slowly rotates to search.

---

## Knowledge graph

The KG builder continuously observes the camera and builds a structured record of everything the rover has seen.

### How it works

1. One frame is sampled from `camera:frame` every `KG_SAMPLE_INTERVAL` seconds (default 3 s) into a rolling deque of size `KG_FRAMES` (default 3).
2. When a fresh frame just filled the deque, all 3 frames are sent together to `mlx_vlm.server` as a single batched vision prompt.
3. The VLM returns a strict JSON object: `{caption, objects:[{label,attrs}], events:[{description,involves}], relationships:[{subject,predicate,object}]}`.
4. Extracted labels are **fuzzy-merged** against existing Kuzu nodes via `difflib.SequenceMatcher` (thresholds: objects 0.82, events 0.78) to prevent duplicates.
5. All 3 frames in the batch are attached as images to each inserted/matched node, growing per-node image galleries over time.
6. Named faces from `face:latest` are automatically linked as `Person` nodes.
7. The updated graph snapshot is published to Redis `kg:snapshot` (TTL 300 s).

### Kuzu schema

**Node tables**

| Table | Fields |
|---|---|
| `Object` | `id, label, first_seen, last_seen, attrs` |
| `Event` | `id, content, ts` |
| `Person` | `id, name, first_seen, last_seen` |
| `Image` | `id, ts, path` |
| `Diary` | `id, date, text, generated_at` |

**Relationship tables**

| Relationship | From → To | Extra fields |
|---|---|---|
| `INVOLVES` | Event → Object | `ts` |
| `WITNESSED_BY` | Event → Person | `ts` |
| `RELATED` | Object → Object | `predicate, ts` (open-vocabulary) |
| `PICTURED_IN` | Object → Image | — |
| `CAPTURED_AT` | Event → Image | — |
| `APPEARS_IN` | Person → Image | — |

### Disk layout

```
data/kg/
├── graph_db/        # Kuzu database (live)
├── graph_db_test/   # Kuzu database (TEST_MODE=1)
├── images/          # Per-node JPEG images (live)
└── images_test/     # Per-node JPEG images (test mode)
```

### Resetting the graph

```bash
curl -X POST http://localhost:8012/kg/reset
```

This sets a `kg:reset` sentinel in Redis. The kg_builder picks it up within one loop tick, wipes the Kuzu database and images directory, and starts fresh.

### Diary

`GET /kg/diary?date=YYYY-MM-DD` generates a NOIR-voiced first-person diary entry for the day using the graph snapshot. Cached for 24 h in Redis.

---

## Dashboard

Static single-page app at `http://localhost:8013`. No build step — served directly from `dashboard/index.html` by Python's `http.server`.

**Tabs:**

- **CAMERA** — live MJPEG feed, NOIR agent chat panel with SSE tool-call rendering (shows pending tool calls in real time), and voice input/output using the audio sidecar.
- **KNOWLEDGE** — interactive vis-network graph of the knowledge graph. Click a node to see its detail panel: label, first/last seen, attribute tags, and an image gallery of all frames where the object was observed. Includes the diary view.

---

## Audio sidecar — port 8014

`audio/server.py` is a standalone FastAPI service providing speech I/O for the dashboard.

| Endpoint | Method | Body / Response | Purpose |
|---|---|---|---|
| `/audio/health` | GET | `{stt_ready, tts_ready}` | Readiness probe |
| `/audio/stt` | POST | multipart audio file → `{text, duration_ms}` | Transcribe speech (Parakeet 0.6B) |
| `/audio/tts` | POST | `{text, voice?}` → `audio/wav` | Synthesise speech (Kokoro 82M) |

Models are loaded once on startup. The TTS pipeline runs a silent warmup pulse before reporting ready, avoiding a 30–40 s stall on the first real request.

---

## Controller (Xbox / PS5)

The `controllers/driver.py` connects to an Xbox or PS5 DualSense controller via Apple's `GameController.framework` at 60 Hz. All inputs are independent and combine simultaneously.

| Input | Action |
|---|---|
| Left stick Y | Forward / backward |
| Left stick X | Strafe left / right |
| Right stick X | Rotate |
| L1 / R1 | Rotate left / right (digital) |
| Hold **L2** | Precision mode (slow, fine control) |
| Hold **R2** | Boost mode (full speed) |

| Mode | Linear speed | Rotation speed |
|---|---|---|
| Base | 0.25 m/s | 4.0 rad/s |
| Boost (R2) | 1.4 m/s | 10.0 rad/s |
| Precision (L2) | 0.22 m/s | 3.0 rad/s |

PS5 lightbar: purple = precision, orange brightness ∝ speed in boost, green = base. Adaptive triggers are active on L2 (resistive) and R2 (vibration).

Any controller input temporarily pauses face-following for 1.5 s.

---

## Face recognition

Enroll a face by placing `faces/<Name>.jpg` in the repo root. The `facerec` service loads all enrolled images at startup. Recognition threshold: 0.35 cosine similarity (InsightFace `buffalo_l` model, `det_size=(640,640)`).

Recognised names (anything that isn't `"UNKNOWN"`) are automatically linked into the knowledge graph as `Person` nodes by the kg_builder on each VLM cycle.

---

## Redis schema

| Key | Type | TTL | Written by |
|---|---|---|---|
| `xbox:last_input_ts` | string | 5 s | `controllers/driver.py` |
| `camera:frame` | base64 JPEG | 5 s | bridge `_camera_frame_writer` or `vision/test_feed.py` |
| `camera:ts` | string | 5 s | same as above |
| `vision:latest` | JSON | 10 s | `vision/app.py` |
| `vision:thumb:{frame_id}` | base64 JPEG | 60 s | `vision/app.py`, bridge `/look_around` |
| `vision:events` | pubsub | — | `vision/app.py` |
| `memory:events` | stream | — | `vision/memory.py` |
| `vlm:latest` | JSON `{text,ts,frame_id}` | 30 s | `vision/kg_builder.py` |
| `face:latest` | JSON `{faces,ts,frame_id,frame_w,frame_h}` | 10 s | `vision/facerec.py` |
| `kg:snapshot` | JSON `{nodes,edges,counts,node_images,image_index,ts}` | 300 s | `vision/kg_builder.py` |
| `kg:updated` | pubsub | — | `vision/kg_builder.py` |
| `kg:last_update` | unix timestamp | 3600 s | `vision/kg_builder.py` |
| `kg:reset` | sentinel `"1"` | 30 s | `ros-noetic/bridge-api.py` |
| `kg:diary:{date}` | diary text | 86400 s | `ros-noetic/bridge-api.py` |
| `agent:history` | JSON list (last 20 msgs) | 1800 s | `ros-noetic/bridge-api.py` |
| `agent:follow_cfg` | JSON `{on,target,started}` | — | `ros-noetic/bridge-api.py` |

---

## Project layout

```
noir-rover/
├── ros-noetic/                 # ROS bridge (ros_env)
│   ├── bridge-api.py           # FastAPI HTTP server :8012
│   ├── scoutros.py             # ROS subscriber/publisher/service wrappers
│   ├── requirements.txt        # FastAPI/uvicorn/redis deps for ros_env
│   └── roller_eye/             # Custom ROS message and service definitions (9 msgs, 29 srvs)
├── vision/
│   ├── app.py                  # YOLOE detection loop (noir_env)
│   ├── kg_builder.py           # KG builder + VLM captioning (noir_env)
│   ├── kg_store.py             # Kuzu DB wrapper (noir_env)
│   ├── facerec.py              # Face recognition (noir_env)
│   ├── memory.py               # Debounced detection event writer (thread in app.py)
│   └── test_feed.py            # Synthetic camera feed for test mode (noir_env)
├── audio/
│   └── server.py               # STT/TTS FastAPI sidecar :8014 (noir_env)
├── controllers/
│   └── driver.py               # Xbox / PS5 DualSense gamepad driver (noir_env)
├── dashboard/
│   └── index.html              # Single-page dashboard UI served on :8013
├── ai/
│   ├── yoloe-11m-seg.pt        # YOLOE model weights
│   └── mobileclip_blt.pt       # MobileCLIP text projection weights (YOLOE dep)
├── faces/                      # Face enrollment JPEGs (gitignored contents)
├── data/
│   └── kg/                     # Kuzu database + KG images (gitignored)
├── scripts/
│   ├── doctor.sh               # Environment health checker
│   ├── start_bridge.sh         # Bridge launch script (used by Tiltfile)
│   ├── wait_audio_ready.sh     # Audio readiness poll (used by Tiltfile)
│   ├── keyboard_drive_native.py# Curses keyboard teleop (make keyboard)
│   ├── ps5_test.py             # Standalone PS5 DualSense tester
│   └── bench_vlm.py            # VLM latency benchmark
├── recordings/                 # Test feed MP4s (gitignored)
├── .env.example                # Config template (copy to .env)
├── Tiltfile                    # Tilt orchestration config
├── Makefile                    # Dev shortcuts
├── environment.yml             # noir_env conda spec
└── docker-compose.yml          # Redis container
```

---

## Makefile targets

| Target | What it does |
|---|---|
| `make install` | `sync` + `build-bridge` + `pull-models` — full first-time setup |
| `make setup-system` | `brew install ffmpeg espeak-ng` |
| `make sync` | Create/update `noir_env` from `environment.yml` |
| `make pull-models` | Download VLM, STT, TTS models from HuggingFace |
| `make sync-bridge` | Install `ros-noetic/requirements.txt` into `ros_env` |
| `make build-bridge` | `sync-bridge` + `catkin_make` for `roller_eye` messages |
| `make doctor` | Run `scripts/doctor.sh` — verifies envs, models, ports |
| `make keyboard` | Launch curses keyboard teleop |

---

## Troubleshooting

**Camera and sensors show nothing after startup**
Run `ping linaro-alip`. If it fails, add `10.42.0.1 linaro-alip` to `/etc/hosts` (see setup step 4).

**`make build-bridge` fails with CMake version error**
The flag `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` in the Makefile handles this automatically. If running `catkin_make` manually, add that flag yourself.

**Bridge fails to start — "roller_eye not found"**
The catkin workspace hasn't been built. Run `make build-bridge`.

**Mac's IP changed (not 10.42.0.181)**
Update `ROS_IP` in `.env`. Find your current IP: `ifconfig | grep 10.42.`.

**VLM server not responding**
Check: `curl http://localhost:8000/v1/models`. If it fails, the `vlm-server` Tilt resource didn't start. The model download on first boot can take a few minutes. Check the `vlm-server` Tilt logs.

**Audio service not ready**
Check: `curl http://localhost:8014/audio/health`. The audio server loads STT then TTS serially and runs a TTS warmup pulse — allow ~60 s after the service process starts. Check `audio-ready` in the Tilt UI.

**Knowledge graph is empty**
The `kg` service depends on `vlm-server-ready`. If the VLM server started late, kg_builder may have exited early — restart the `kg` Tilt resource. Also check `vlm:latest` in Redis: `redis-cli -p 6380 get vlm:latest`.

**Reset the knowledge graph**
```bash
curl -X POST http://localhost:8012/kg/reset
```

**Robot movement feels laggy**
Check ping latency: `ping 10.42.0.1`. Normal on USB-C Ethernet is < 1 ms. Anything above ~5 ms indicates a cable or adapter issue.

**Stuck on setup?**
Run `make doctor` — it checks `ros_env`/`noir_env` packages, model cache, robot reachability, and all running service ports.
