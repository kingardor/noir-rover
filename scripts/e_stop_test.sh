#!/usr/bin/env bash
# Verify the Xbox e-stop: agent moves must be blocked when Xbox is active.
set -euo pipefail

BRIDGE=${BRIDGE_URL:-http://localhost:8012}
PASS=0
FAIL=0

check() {
  local label="$1" cond="$2"
  if eval "$cond"; then
    echo "  PASS: $label"
    PASS=$((PASS + 1))
  else
    echo "  FAIL: $label"
    FAIL=$((FAIL + 1))
  fi
}

echo "=== noir e-stop test ==="

# Inject heartbeat
curl -s -X POST "$BRIDGE/agent/heartbeat" > /dev/null

# --- Test 1: agent move blocked when Xbox is active ---
echo "--- Test 1: Xbox blocks agent ---"
# Simulate Xbox activity by writing directly to Redis via Python
python3 -c "import redis, time; r=redis.Redis(host='localhost',port=6380); r.set('xbox:last_input_ts', str(time.time()), ex=5)"

RESULT=$(curl -s -X POST "$BRIDGE/robot/move" \
  -H "Content-Type: application/json" \
  -d '{"x": 0.3, "y": 0.0, "source": "agent"}' 2>&1)
echo "  response: $RESULT"
check "Xbox blocks agent move" 'echo "$RESULT" | grep -q "xbox_active"'

# --- Test 2: agent move blocked when heartbeat is stale ---
echo "--- Test 2: Stale heartbeat blocks agent ---"
python3 -c "import redis; r=redis.Redis(host='localhost',port=6380); r.delete('xbox:last_input_ts','agent:heartbeat_ts')"
sleep 0.2

RESULT=$(curl -s -X POST "$BRIDGE/robot/move" \
  -H "Content-Type: application/json" \
  -d '{"x": 0.3, "y": 0.0, "source": "agent"}' 2>&1)
echo "  response: $RESULT"
check "Stale heartbeat blocks agent move" 'echo "$RESULT" | grep -q "heartbeat_stale"'

# --- Test 3: agent move passes after fresh heartbeat ---
echo "--- Test 3: Fresh heartbeat allows agent ---"
curl -s -X POST "$BRIDGE/agent/heartbeat" > /dev/null
RESULT=$(curl -s -X POST "$BRIDGE/robot/move" \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 0.0, "rotate": 0.0, "source": "agent"}' 2>&1)
echo "  response: $RESULT"
check "Agent allowed with fresh heartbeat" 'echo "$RESULT" | grep -q '"'"'"ok":true'"'"''

# --- Test 4: magnitude clamp ---
echo "--- Test 4: Magnitude clamp ---"
RESULT=$(curl -s -X POST "$BRIDGE/robot/move" \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 5.0, "source": "manual"}' 2>&1)
echo "  response: $RESULT"
check "Magnitude clamp blocks y=5.0" 'echo "$RESULT" | grep -q "magnitude_exceeded"'

# --- Summary ---
echo ""
echo "=== Results: $PASS passed, $FAIL failed ==="
[[ $FAIL -eq 0 ]] && exit 0 || exit 1
