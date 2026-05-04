#!/usr/bin/env bash
# Verify the dev environment is in shape. Exit non-zero on any failure.
set -u
fail=0
MAMBA=/opt/homebrew/opt/micromamba/bin/micromamba

check() {
    local label="$1"; shift
    printf "  %-42s " "$label"
    if "$@" >/dev/null 2>&1; then echo "OK"; else echo "FAIL"; fail=1; fi
}

echo "ros_env (micromamba — ROS bridge):"
check "env exists"              bash -c "$MAMBA env list | grep -q '^ros_env[[:space:]]'"
check "ros-noetic available"    $MAMBA run -n ros_env rosversion -d
check "fastapi installed"       $MAMBA run -n ros_env python -c 'import fastapi'
check "uvicorn installed"       $MAMBA run -n ros_env python -c 'import uvicorn'
check "redis client installed"  $MAMBA run -n ros_env python -c 'import redis'
check "pydantic installed"      $MAMBA run -n ros_env python -c 'import pydantic'
check "roller_eye built"        test -f catkin_ws/devel/setup.bash

echo
echo "noir_env (micromamba — vision / controller):"
check "env exists"              bash -c "$MAMBA env list | grep -q '^noir_env[[:space:]]'"
check "torch installed"         $MAMBA run -n noir_env python -c 'import torch'
check "ultralytics installed"   $MAMBA run -n noir_env python -c 'import ultralytics'
check "insightface installed"   $MAMBA run -n noir_env python -c 'import insightface'
check "pyobjc installed"        $MAMBA run -n noir_env python -c 'import GameController'
check "redis client installed"  $MAMBA run -n noir_env python -c 'import redis'

echo
echo "Robot reachability:"
check "linaro-alip resolves"    getent hosts linaro-alip
check "robot ping (10.42.0.1)" ping -c1 -t1 10.42.0.1
check "Mac IP is 10.42.0.181"  bash -c "ifconfig | grep -q 'inet 10.42.0.181'"

echo
echo "Services (if stack is running):"
check "Redis on :6380"          bash -c "printf 'PING\r\n' | nc -w1 localhost 6380 | grep -q PONG"
check "Bridge on :8012"         curl -sf http://localhost:8012/status

if [[ $fail -ne 0 ]]; then
    echo
    echo "Some checks failed."
    echo "  ros_env issues  → make sync-bridge  or  make build-bridge"
    echo "  noir_env issues → make sync"
    exit 1
fi
echo
echo "All good."
