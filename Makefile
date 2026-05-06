.PHONY: up stop sync build-bridge sync-bridge pull-models doctor keyboard ps5-test

MAMBA := /opt/homebrew/opt/micromamba/bin/micromamba
HF_DL  := $(MAMBA) run -n noir_env python -c

# First-time setup: system deps, Python envs, bridge build, model downloads
install: sync build-bridge pull-models

# Install/update system deps needed by the MLX audio stack
setup-system:
	brew install ffmpeg espeak-ng

# Create (first run) or update (subsequent runs) noir_env from environment.yml
sync:
	@if $(MAMBA) env list | grep -q '^noir_env[[:space:]]'; then \
	  $(MAMBA) env update -f environment.yml; \
	else \
	  $(MAMBA) create -f environment.yml -y; \
	fi

# Download all HuggingFace models into the local cache (idempotent)
pull-models:
	@echo "Pulling MLX models into HF cache…"
	$(HF_DL) "from huggingface_hub import snapshot_download; snapshot_download('mlx-community/Qwen3-VL-2B-Instruct-4bit')"
	$(HF_DL) "from huggingface_hub import snapshot_download; snapshot_download('mlx-community/parakeet-tdt-0.6b-v3')"
	$(HF_DL) "from huggingface_hub import snapshot_download; snapshot_download('mlx-community/Kokoro-82M-4bit')"
	@echo "Models ready."

# Install bridge pip deps into ros_env + build roller_eye catkin workspace
sync-bridge:
	$(MAMBA) run -n ros_env pip install -r ros-noetic/requirements.txt

build-bridge: sync-bridge
	mkdir -p catkin_ws/src
	cp -r ros-noetic/roller_eye catkin_ws/src/
	$(MAMBA) run -n ros_env catkin_make -C catkin_ws -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# Verify the dev environment is in shape
doctor:
	bash scripts/doctor.sh

# Keyboard teleoperation (not managed by Tilt)
keyboard:
	$(MAMBA) run -n noir_env python scripts/keyboard_drive_native.py
