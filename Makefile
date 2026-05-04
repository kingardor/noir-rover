.PHONY: up stop sync build-bridge sync-bridge doctor keyboard ps5-test

MAMBA := /opt/homebrew/opt/micromamba/bin/micromamba

# Bootstrap all envs + launch the full stack
install: sync build-bridge

# Create (first run) or update (subsequent runs) noir_env from environment.yml
sync:
	@if $(MAMBA) env list | grep -q '^noir_env[[:space:]]'; then \
	  $(MAMBA) env update -f environment.yml; \
	else \
	  $(MAMBA) create -f environment.yml -y; \
	fi

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