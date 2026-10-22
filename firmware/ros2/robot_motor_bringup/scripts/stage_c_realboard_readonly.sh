#!/usr/bin/env bash
# Stage C final step: read-only bring-up against the REAL board.
#
# This script sends NOTHING but Modbus function code 03 reads. It never writes a
# register, never sends a command frame, never moves, never clears a fault and never
# sets a zero. allow_motion and maintenance are both left false, so the driver is in
# read-only mode and even /robot/stop is refused by design.
#
# It also refuses to run if the board's stable serial path is missing or already held
# by another process, and it never touches /dev/ttyUSB1 (that is the door lock gateway).
set -u

WS="${WS:-/data/ros2/robot_motor_ros2_ws}"
OUT="${OUT:-/data/ros2/stage_c_realboard_evidence}"
PORT="${PORT:-/dev/serial/by-path/platform-fc800000.usb-usb-0:1.2:1.0-port0}"
RUN_SECONDS="${RUN_SECONDS:-70}"

mkdir -p "$OUT"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"

echo "=== stage C read-only real-board run, ROS_DOMAIN_ID=$ROS_DOMAIN_ID ==="

echo "--- pre-flight: port identity and occupancy"
{
  date -Is
  ls -l /dev/serial/by-path/ /dev/serial/by-id/
  echo "resolved: $(readlink -f "$PORT" 2>&1)"
  echo "holder:   $(fuser "$PORT" 2>&1 || echo none)"
  echo "door lock (must stay untouched): $(readlink -f /dev/door_lock_uart 2>&1)"
} | tee "$OUT/preflight.txt"

if [ ! -e "$PORT" ]; then
  echo "ABORT: $PORT does not exist; the board is not connected or re-enumerated."
  exit 2
fi
if fuser "$PORT" >/dev/null 2>&1; then
  echo "ABORT: $PORT is held by another process; refusing to fight for the bus."
  exit 2
fi

echo "--- launching read-only (allow_motion:=false, no maintenance)"
timeout "$((RUN_SECONDS + 30))" ros2 launch robot_motor_bringup bringup.launch.py \
  transport:=modbus_rtu allow_motion:=false maintenance:=false \
  > "$OUT/node.log" 2>&1 &
LAUNCH_PID=$!
sleep 20

echo "--- /robot/motor_states --once"
timeout 40 ros2 topic echo /robot/motor_states --once > "$OUT/motor_states.yaml" 2>&1
head -60 "$OUT/motor_states.yaml"

echo "--- /robot/board_identity"
timeout 40 ros2 service call /robot/board_identity std_srvs/srv/Trigger "{}" \
  > "$OUT/identity.log" 2>&1
cat "$OUT/identity.log"

echo "--- /robot/stop must be REFUSED in read-only mode (and must not reach the bus)"
timeout 40 ros2 service call /robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: 'head_joint'}" > "$OUT/stop_refused.log" 2>&1
cat "$OUT/stop_refused.log"

echo "--- /robot/diagnostics_json --once"
timeout 40 ros2 topic echo /robot/diagnostics_json --once > "$OUT/diagnostics.log" 2>&1
head -20 "$OUT/diagnostics.log"

kill "$LAUNCH_PID" 2>/dev/null
wait "$LAUNCH_PID" 2>/dev/null
sleep 2

echo "--- command frames sent during the whole run (must be 0)"
TX_COUNT=$(grep -c "TX " "$OUT/node.log" || true)
echo "TX_COUNT=$TX_COUNT" | tee "$OUT/tx_count.txt"

echo "--- node log highlights"
grep -iE "identified|ready|read-only|refus|SIMULATED|error|warn" "$OUT/node.log" | head -20

echo "--- evidence"
ls -l "$OUT"
if [ "$TX_COUNT" != "0" ]; then
  echo "FAIL: the read-only run transmitted a command frame"
  exit 1
fi
echo "=== done: read-only, zero command frames ==="
