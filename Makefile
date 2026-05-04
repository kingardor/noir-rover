.PHONY: dev logs stop e2e sync voice patrol follow dashboard keyboard ps5-test controller

# Fallback: plain bash (no Tilt UI) — prefer: tilt up
dev:
	bash scripts/dev.sh

logs:
	bash scripts/logs.sh

stop:
	docker compose down
	pkill -f "vision/app.py" 2>/dev/null || true
	pkill -f "audio/tts.py"  2>/dev/null || true
	pkill -f "audio/stt.py"  2>/dev/null || true
	pkill -f "mcp/main.py"   2>/dev/null || true

e2e:
	bash scripts/e_stop_test.sh

# Install / sync all Python dependencies into .venv
sync:
	uv sync --all-groups

# Open dashboard in the default browser (standalone, no Tilt)
dashboard:
	python3 -m http.server 8013 --directory dashboard &
	open http://localhost:8013

# Run a voice mission
# Usage: make voice GOAL="find the red mug"
voice:
	BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 \
	  uv run python -m agent.missions.voice "$(GOAL)"

patrol:
	BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 \
	  uv run python -m agent.missions.patrol

follow:
	BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 \
	  uv run python -m agent.missions.follow

# Native macOS controller driver — Xbox or PS5 over Bluetooth → bridge API
controller:
	BRIDGE_URL=http://localhost:8012 REDIS_URL=redis://localhost:6380 \
	  uv run python -u controllers/driver.py

# Test PS5 DualSense controller (buttons, axes, rumble, lightbar)
ps5-test:
	uv run python scripts/ps5_test.py

# Keyboard teleoperation — sends UDP directly to relay on robot
keyboard:
	uv run python scripts/keyboard_drive_native.py
