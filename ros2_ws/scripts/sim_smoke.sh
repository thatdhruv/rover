#!/bin/bash
# Headless Gazebo smoke test: launch the sim, drive forward, and assert that the controllers came
# up, odometry moved, and the lidar produced a scan. Run from a sourced workspace.
set -u
log=$(mktemp)
ros2 launch rover_gazebo sim.launch.py headless:=true >"$log" 2>&1 &
sim=$!
trap 'kill $sim 2>/dev/null; wait $sim 2>/dev/null' EXIT

# wait for the diff drive controller (up to 150 s)
for _ in $(seq 150); do
  grep -q "Configured and activated diff_drive_controller" "$log" && break
  sleep 1
done
grep -q "Configured and activated diff_drive_controller" "$log" || { echo "FAIL: controllers did not start"; tail -30 "$log"; exit 1; }

ros2 topic pub -r 10 -t 60 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}}}" >/dev/null 2>&1

x=$(timeout 15 ros2 topic echo --once --field pose.pose.position.x /odometry/filtered 2>/dev/null | head -1)
n=$(timeout 15 ros2 topic echo --once --field ranges /scan 2>/dev/null | head -1 | grep -c "array")
echo "odometry x = ${x:-none} m, scan received = $n"

python3 - "$x" "$n" <<'PY'
import sys
x, n = sys.argv[1], sys.argv[2]
ok = x not in ("", "none") and float(x) > 0.2 and n == "1"
print("PASS" if ok else "FAIL: rover did not move or no lidar scan")
sys.exit(0 if ok else 1)
PY
