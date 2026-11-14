# Verification status

What has actually been run, and what has not. Updated with each milestone.

## Verified

| Area | How | Result |
|------|-----|--------|
| IMU driver, PID, heading filter, link monitor, encoder math, quadrature decoder, jitter stats, fault log | ztest on `native_sim` (Zephyr 4.4.0), local Docker and CI | 40/40 pass |
| Firmware build for NUCLEO-F429ZI with MCUboot, signed (ECDSA P-256) | `west build --sysbuild`, `imgtool verify` | builds; app 43% of a 256 KB slot, bootloader 84% of 32 KB; signature validates |
| Boot path | Renode: MCUboot validates the signature, boots the app; kernel clock, 1 kHz loop, shell | boots; 2,287 loop ticks in ~2.7 s, max jitter 51 µs *(emulated time, not a hardware measurement)* |
| Automatic rollback | Renode: a signed update that hangs is staged in slot 1; MCUboot swaps it in (test mode), the watchdog resets the hung image, MCUboot reverts, the original image runs and confirms itself | passes with the production bootloader. Emulated, not run on silicon |
| ROS 2 (Lyrical) packages | `colcon build` + `colcon test` in `ros:lyrical` | builds; 17 tests pass, including the hardware interface loaded through `ros2_control` |
| Simulation | Headless Gazebo: controllers activate, rover drives, EKF odometry advances, lidar publishes | passes (`ros2_ws/scripts/sim_smoke.sh`) |
| SLAM + navigation | Headless Gazebo + SLAM Toolbox + Nav2: `NavigateToPose` goal at (1.2, 0.0) | succeeds, ends 0.10 m from the goal (`ros2_ws/scripts/nav_smoke.sh`). One goal in an open area, not a full mapping/obstacle run |
| HIL script logic | pytest against a fake board, including a firmware that fails to cut motors | 7/7 pass |

## Not verified

- **Nothing has run on real hardware.** The pin map, PID gains, feed-forward constant, encoder CPR
  (1320) and wheel geometry are placeholders to be tuned on the bench.
- **The MPU-6050 driver has only been tested against its emulator**, not a physical sensor. Renode has
  no MPU-6050 model, so the Renode runs exercise the IMU-absent (degraded) path.
- **micro-ROS firmware (`CONFIG_ROVER_MICROROS`) has not been compiled.** The upstream Zephyr module
  has no `lyrical` branch and its Zephyr 4.4 support is unproven. See `docs/microros.md`.
- **SLAM/Nav2 parameters are the packaged Nav2 defaults with this robot's frames and radius**, not tuned. Only
  a single short goal has been run; navigating around the room's obstacles has not been exercised.
- **Renode is an emulator.** Its STM32 model needed fixes (vendored platform with the real 168 MHz clock, USART6,
  0xFF-erased flash, reset vector), so passing there shows the boot/update logic works, not that the timing or
  flash behaviour matches silicon.
- **Flash erase vs. watchdog:** erasing the 64 KB fault-log sector (only when the log wraps, after
  about 2,000 entries) can exceed the 200 ms watchdog window on the F4.
- The signing key is MCUboot's public development key.
