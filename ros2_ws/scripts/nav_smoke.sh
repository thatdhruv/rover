#!/bin/bash
# Headless end-to-end check: simulation + SLAM Toolbox + Nav2, then navigate to a goal and verify
# the rover actually got there (odometry within tolerance of the goal).
set -u
GX=${GX:-1.2}; GY=${GY:-0.0}
log=$(mktemp)
(ros2 launch rover_gazebo sim.launch.py headless:=true >"$log" 2>&1 &)
(ros2 launch rover_bringup slam.launch.py sim:=true >"$log.slam" 2>&1 &)
cleanup() { pkill -f "ros2 launch" 2>/dev/null; pkill -f gz 2>/dev/null; pkill -f ros2_control 2>/dev/null; }
trap cleanup EXIT

for _ in $(seq 150); do grep -q "Configured and activated diff_drive_controller" "$log" && break; sleep 1; done
grep -q "Configured and activated diff_drive_controller" "$log" || { echo "FAIL: sim did not start"; exit 1; }

# let SLAM publish map -> odom before Nav2 starts
sleep 15
(ros2 launch rover_bringup nav.launch.py sim:=true >"$log.nav" 2>&1 &)
for _ in $(seq 120); do
  ros2 action list 2>/dev/null | grep -q navigate_to_pose && break; sleep 1
done
ros2 action list 2>/dev/null | grep -q navigate_to_pose || { echo "FAIL: Nav2 action server never appeared"; tail -30 "$log.nav"; exit 1; }
sleep 10  # lifecycle activation

echo "sending goal ($GX, $GY)"
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {w: 1.0}}}}" 2>&1 | tail -6
x=$(timeout 10 ros2 topic echo --once --field pose.pose.position.x /odometry/filtered 2>/dev/null | head -1)
y=$(timeout 10 ros2 topic echo --once --field pose.pose.position.y /odometry/filtered 2>/dev/null | head -1)
echo "final odometry: x=$x y=$y"
python3 - "$x" "$y" "$GX" "$GY" <<'PY'
import math, sys
x, y, gx, gy = map(float, sys.argv[1:5])
d = math.hypot(x - gx, y - gy)
print(f"distance to goal: {d:.2f} m")
sys.exit(0 if d < 0.35 else 1)
PY
