#!/bin/bash
# Copy the rover message definitions into libmicroros so the firmware can use
# rover_msgs/WheelState and rover_msgs/WheelCmd. Run once after `west update`.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
dest="$root/modules/lib/micro_ros_zephyr_module/modules/libmicroros/extra_packages"
mkdir -p "$dest"
rm -rf "$dest/rover_msgs"
cp -r "$root/ros2_ws/src/rover_msgs" "$dest/rover_msgs"
echo "rover_msgs copied to $dest"
