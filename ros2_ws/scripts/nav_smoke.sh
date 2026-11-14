#!/bin/bash
# Headless end-to-end check: simulation + SLAM Toolbox + Nav2, then navigate to a goal and verify
# the rover actually got there (odometry within 0.5 m: odom and map frames differ by SLAM correction).
set -u
GX=${GX:-1.2}; GY=${GY:-0.0}
log=$(mktemp)
(ros2 launch rover_gazebo sim.launch.py headless:=true >"$log" 2>&1 &)
cleanup() { pkill -f "ros2 launch" 2>/dev/null; pkill -f gz 2>/dev/null; pkill -f ros2_control 2>/dev/null; }
trap cleanup EXIT

for _ in $(seq 150); do grep -q "Configured and activated diff_drive_controller" "$log" && break; sleep 1; done
grep -q "Configured and activated diff_drive_controller" "$log" || { echo "FAIL: sim did not start"; exit 1; }

# Bring things up in order: localization, then SLAM, then Nav2. Nav2's lifecycle manager gives up for
# good if the odom frame is missing when the costmaps activate, and the EKF only publishes
# odom -> base_footprint once wheel odometry flows.
#
# Wait with single blocking calls and log greps. Polling with a fresh `ros2` CLI process every second
# churns DDS discovery enough to make the lifecycle manager miss services.
# `ros2 topic echo --once` returns at once if the topic is not advertised yet, so retry slowly.
wait_for_message() {  # wait_for_message <topic> [extra echo args...]
  local topic=$1; shift
  for _ in $(seq 40); do
    timeout 20 ros2 topic echo --once "$topic" "$@" --field header.frame_id >/dev/null 2>&1 && return 0
    sleep 5
  done
  return 1
}

echo "waiting for filtered odometry"
wait_for_message /odometry/filtered || { echo "FAIL: EKF never produced odometry"; exit 1; }

(ros2 launch rover_bringup slam.launch.py sim:=true >"$log.slam" 2>&1 &)
echo "waiting for the SLAM map"
wait_for_message /map --qos-durability transient_local --qos-reliability reliable   || { echo "FAIL: SLAM never published a map"; exit 1; }

(ros2 launch rover_bringup nav.launch.py sim:=true >"$log.nav" 2>&1 &)
echo "waiting for Nav2 to activate"
for _ in $(seq 40); do
  grep -q "Managed nodes are active" "$log.nav" && break
  grep -q "Aborting bringup" "$log.nav" && break
  sleep 5
done
grep -q "Managed nodes are active" "$log.nav" || { echo "FAIL: Nav2 did not activate"; tail -20 "$log.nav"; exit 1; }

# Nav2 rejects goals until its lifecycle nodes, costmaps and the map->odom transform are all up, which
# takes longer on a slow machine. Retry until a goal is accepted.
echo "sending goal ($GX, $GY)"
for attempt in $(seq 15); do
  out=$(timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose     "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {w: 1.0}}}}" 2>&1)
  if echo "$out" | grep -qi "rejected"; then
    echo "attempt $attempt: goal rejected, Nav2 not ready yet"; sleep 10; continue
  fi
  echo "$out" | tail -4
  break
done
x=$(timeout 10 ros2 topic echo --once --field pose.pose.position.x /odometry/filtered 2>/dev/null | head -1)
y=$(timeout 10 ros2 topic echo --once --field pose.pose.position.y /odometry/filtered 2>/dev/null | head -1)
echo "final odometry: x=$x y=$y"
python3 - "$x" "$y" "$GX" "$GY" <<'PY'
import math, sys
x, y, gx, gy = map(float, sys.argv[1:5])
d = math.hypot(x - gx, y - gy)
print(f"distance to goal: {d:.2f} m")
sys.exit(0 if d < 0.5 else 1)
PY
