# Rover

Autonomous two-wheeled rover: Zephyr RTOS firmware (C) on an STM32 Nucleo plus
ROS 2 (Lyrical) on a Raspberry Pi 5.

| Layer | Path | Status |
|-------|------|--------|
| Firmware (Zephyr) | [`firmware/`](firmware) | in progress |
| ROS 2 workspace | `ros2_ws/` | planned |
| Hardware-in-the-loop | `tools/hil/` | planned |

Build order: board bring-up + IMU driver → motor control → ROS 2 integration.

## Firmware quick start

```sh
west init -l firmware
west update
west build -b nucleo_f446re firmware
west twister -T firmware/tests -p native_sim   # unit tests
```
