#!/usr/bin/env bash
# Stage D acceptance run against the SIMULATED board (execution plan section 8D).
#
# transport is always "fake" here: no serial port is opened and no frame reaches
# real hardware. Everything is captured under $OUT so the claims in the
# verification record can be checked against raw output instead of prose.
#
# Usage (on the robot host):
#   bash ros2/robot_motor_bringup/scripts/stage_d_fake_acceptance.sh
set -u

WS="${WS:-/data/ros2/robot_motor_ros2_ws}"
OUT="${OUT:-/data/ros2/stage_d_evidence}"
DOMAIN="${ROS_DOMAIN_ID:-71}"

mkdir -p "$OUT"
# colcon's setup.bash references unbound variables (COLCON_TRACE), so `set -u` has
# to be off while sourcing it and can come back afterwards.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN"

echo "=== stage D evidence, ROS_DOMAIN_ID=$DOMAIN, output in $OUT ==="
{
  date -Is
  hostname
  python3 --version
  ros2 pkg prefix robot_motor_driver
  ros2 pkg prefix robot_motor_bringup
} > "$OUT/environment.txt" 2>&1
cat "$OUT/environment.txt"

launch_fake() {
  # $1 = log name, rest = launch arguments
  local log="$1"; shift
  timeout "${LAUNCH_TIMEOUT:-150}" ros2 launch robot_motor_bringup bringup.launch.py \
    transport:=fake "$@" > "$OUT/$log" 2>&1 &
  LAUNCH_PID=$!
  sleep "${SETTLE:-14}"
}

stop_launch() {
  kill "$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
  sleep 2
}

MOVE_RELATIVE="{joint_name: 'left_forearm_joint', mode: 0, angle_rad: 0.1745329252, motor_rpm: 400, timeout_sec: 30.0}"
MOVE_ABSOLUTE="{joint_name: 'left_forearm_joint', mode: 1, angle_rad: 0.2, motor_rpm: 400, timeout_sec: 30.0}"

# ---------------------------------------------------------------------------
echo
echo "=== case 1: default bring-up is read-only (allow_motion:=false) ==="
launch_fake readonly.log allow_motion:=false
{
  echo "--- ros2 topic list"; ros2 topic list
  echo "--- ros2 service list"; ros2 service list
  echo "--- ros2 action list"; ros2 action list
} > "$OUT/graph.txt" 2>&1
cat "$OUT/graph.txt"

ros2 interface show robot_motor_interfaces/action/MoveJoint > "$OUT/interface_movejoint.txt" 2>&1
ros2 interface show robot_motor_interfaces/msg/MotorStates > "$OUT/interface_motorstates.txt" 2>&1

timeout 40 ros2 topic echo /robot/motor_states --once > "$OUT/motor_states.yaml" 2>&1
echo "--- /robot/motor_states --once (first 40 lines)"
head -40 "$OUT/motor_states.yaml"

timeout 40 ros2 action send_goal /robot/move_joint \
  robot_motor_interfaces/action/MoveJoint "$MOVE_RELATIVE" > "$OUT/move_rejected.log" 2>&1
echo "--- move goal in read-only mode (expect rejection)"
cat "$OUT/move_rejected.log"

timeout 40 ros2 service call /robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: 'head_joint'}" > "$OUT/stop_readonly.log" 2>&1
echo "--- /robot/stop in read-only mode (expect refusal naming the hardware e-stop)"
cat "$OUT/stop_readonly.log"

timeout 40 ros2 service call /robot/board_identity std_srvs/srv/Trigger \
  "{}" > "$OUT/identity.log" 2>&1
echo "--- /robot/board_identity"
cat "$OUT/identity.log"
stop_launch

echo "--- node log: writes attempted during the read-only run"
grep -icE "TX |command" "$OUT/readonly.log" || true
grep -iE "read-only|allow_motion|identified|SIMULATED" "$OUT/readonly.log" | head -12

# ---------------------------------------------------------------------------
echo
echo "=== case 2: fake backend with motion explicitly allowed ==="
# allow_unreferenced_relative:=true is an explicit choice for THIS simulated run so
# the plan's +10 deg example actually executes a motion. On a real board it must stay
# false until an operator has decided that moving without a zero reference is wanted.
launch_fake motion.log allow_motion:=true allow_unreferenced_relative:=true

echo "--- relative +10 deg on left_forearm_joint (feedback enabled)"
timeout 90 ros2 action send_goal /robot/move_joint \
  robot_motor_interfaces/action/MoveJoint "$MOVE_RELATIVE" --feedback > "$OUT/move_relative.log" 2>&1
tail -25 "$OUT/move_relative.log"

echo "--- absolute move while position_valid is false (expect refusal, no auto zero)"
timeout 60 ros2 action send_goal /robot/move_joint \
  robot_motor_interfaces/action/MoveJoint "$MOVE_ABSOLUTE" > "$OUT/move_absolute_refused.log" 2>&1
tail -12 "$OUT/move_absolute_refused.log"

echo "--- explicit /robot/set_zero (operator action, then absolute becomes possible)"
timeout 60 ros2 service call /robot/set_zero robot_motor_interfaces/srv/SetZero \
  "{joint_name: 'left_forearm_joint'}" > "$OUT/set_zero.log" 2>&1
tail -12 "$OUT/set_zero.log"

echo "--- absolute move after an explicit zero"
timeout 90 ros2 action send_goal /robot/move_joint \
  robot_motor_interfaces/action/MoveJoint "$MOVE_ABSOLUTE" > "$OUT/move_absolute.log" 2>&1
tail -12 "$OUT/move_absolute.log"

echo "--- goal that quantises to zero steps (expect NO_OP, nothing sent)"
timeout 60 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'left_wrist_joint', mode: 0, angle_rad: 0.0001, motor_rpm: 400, timeout_sec: 10.0}" \
  > "$OUT/move_noop.log" 2>&1
tail -12 "$OUT/move_noop.log"

echo "--- cancel a long move and require a confirmed stop"
timeout 90 ros2 action send_goal /robot/move_joint robot_motor_interfaces/action/MoveJoint \
  "{joint_name: 'head_joint', mode: 0, angle_rad: 6.0, motor_rpm: 400, timeout_sec: 60.0}" \
  > "$OUT/move_cancel.log" 2>&1 &
GOAL_PID=$!
sleep 6
ros2 action list > /dev/null 2>&1
# ros2 CLI cannot cancel a goal it did not send, so cancel through the service and
# record that the driver distinguishes accepted from confirmed.
timeout 40 ros2 service call /robot/stop robot_motor_interfaces/srv/Stop \
  "{joint_name: 'head_joint'}" > "$OUT/stop_confirmed.log" 2>&1
wait "$GOAL_PID" 2>/dev/null
echo "--- /robot/stop while a move was in flight"
cat "$OUT/stop_confirmed.log"
echo "--- the interrupted goal"
tail -15 "$OUT/move_cancel.log"

echo "--- self test on the fake backend (it moves; only run because this is simulated)"
timeout 90 ros2 action send_goal /robot/selftest robot_motor_interfaces/action/RunSelfTest \
  "{joint_name: 'head_joint', scope: 0, direction: 0, timeout_sec: 60.0}" > "$OUT/selftest.log" 2>&1
tail -20 "$OUT/selftest.log"

echo "--- diagnostics"
timeout 40 ros2 topic echo /robot/diagnostics_json --once > "$OUT/diagnostics.log" 2>&1 || true
timeout 40 ros2 topic echo /diagnostics --once > "$OUT/diagnostics_std.log" 2>&1 || true
head -20 "$OUT/diagnostics.log" 2>/dev/null
head -20 "$OUT/diagnostics_std.log" 2>/dev/null
stop_launch

echo
echo "=== evidence files ==="
ls -l "$OUT"
echo "=== done ==="
