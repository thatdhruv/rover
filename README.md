# Rover

Autonomous two-wheeled rover: Zephyr RTOS firmware (C) on an STM32 Nucleo-F429ZI, and ROS 2
(Lyrical) on a Raspberry Pi 5 for control, localization, SLAM and navigation.

```
 Raspberry Pi 5 (ROS 2 Lyrical)                      NUCLEO-F429ZI (Zephyr 4.4)
┌──────────────────────────────────────┐   UART    ┌────────────────────────────────────┐
│ Nav2 ─ cmd_vel ─▶ diff_drive_ctrl    │ 921600    │ micro-ROS client                   │
│ SLAM Toolbox ◀─ /scan   ▲            │◀────────▶ │   wheel_cmd ─▶ link monitor        │
│ robot_localization (EKF) │           │ micro-ROS │ 1 kHz PID ─▶ H-bridge PWM ─▶ motors│
│ rover_hardware (ros2_control)        │  agent    │ encoders (IRQ) ─▶ velocity est.    │
└──────────────────────────────────────┘           │ MPU-6050 driver ─▶ heading filter  │
                                                   │ watchdog · fault log · MCUboot A/B │
                                                   └────────────────────────────────────┘
```

## Layout

| Path | What |
|------|------|
| [`firmware/`](firmware) | Zephyr module (drivers, libraries), application, ztest suites |
| [`firmware/drivers/imu`](firmware/drivers/imu) | Hand-written MPU-6050 driver, DT-configured, with an I2C emulator for tests |
| [`firmware/drivers/{encoder,motor}`](firmware/drivers) | GPIO-IRQ quadrature encoder and H-bridge motor drivers |
| [`firmware/lib`](firmware/lib) | PID, heading filter, link monitor, velocity estimator, jitter stats, flash fault log |
| [`firmware/app`](firmware/app) | 1 kHz control loop, safety supervisor, micro-ROS link, bench shell, board overlay |
| [`ros2_ws/`](ros2_ws) | ROS 2 packages: `rover_msgs`, `rover_hardware`, `rover_description`, `rover_bringup`, `rover_gazebo` |
| [`tools/hil`](tools/hil) | Hardware-in-the-loop script (runs on the Pi against the real board) |
| [`tools/renode`](tools/renode) | Renode platform + Robot tests (boot and rollback) |
| [`.github/workflows`](.github/workflows) | CI: firmware, ROS 2, HIL tool tests |

## Firmware

```sh
west init -l firmware && west update
west twister -T firmware/tests -p native_sim          # unit tests
west build --sysbuild -b nucleo_f429zi firmware/app   # MCUboot + signed app
west flash                                            # flashes merged.hex
```

Pin map (see [`nucleo_f429zi.overlay`](firmware/app/boards/nucleo_f429zi.overlay)):
IMU on I2C1 PB8/PB9 · motors PWM TIM3 PC6/PC7 with direction on PF12–PF15 · encoders on
PE2–PE5 · host link USART6 (PG14/PG9) · shell on the ST-Link virtual COM port.

### Safety behavior

- **Watchdog.** The IWDG (200 ms) is fed only by the supervisor, and only while the control loop
  and IMU task have both checked in. A stalled task makes the supervisor cut the motors, log which
  task stalled, and stop feeding.
- **Link loss.** If no command or heartbeat arrives for 250 ms the control loop cuts the motors and
  zeroes setpoints. Motors re-arm when commands resume. Independent of micro-ROS: it works with the
  host link entirely absent.
- **Fault log.** CRC-protected 32-byte records in a 64 KB flash sector, readable with the `faults`
  shell command. Boot cause, watchdog stalls, link loss, IMU failures are recorded.
- **Updates.** MCUboot verifies an ECDSA P-256 signature and swaps images in test mode. The app
  confirms the new image only after 3 s of continuous health; if it resets first, MCUboot rolls back.

> The repo uses MCUboot's public **development** signing key. Generate your own with
> `imgtool keygen` and set `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE` before deploying; never commit it.

### micro-ROS

Optional; needs the extra west group and a ROS toolchain at build time. See
[`docs/microros.md`](docs/microros.md).

### Bench shell

Connect to the ST-Link virtual COM port at 115200 baud:

```
rover vel <left> <right>   wheel setpoints, rad/s      rover hb      heartbeat
rover state                wheels, heading, IMU        rover jitter  1 kHz loop timing
faults [clear]             flash fault log
```

## ROS 2

```sh
cd ros2_ws && rosdep install --from-paths src --ignore-src -y && colcon build
source install/setup.bash

ros2 launch rover_gazebo sim.launch.py                 # simulation (add headless:=true)
ros2 launch rover_bringup slam.launch.py sim:=true     # map the room
ros2 launch rover_bringup nav.launch.py sim:=true      # navigate to goals

ros2 launch rover_bringup robot.launch.py              # real robot: micro-ROS agent + ros2_control + EKF
```

The simulation and the hardware share the same controller configuration, EKF, SLAM and Nav2
parameters; only the `ros2_control` plugin differs (`gz_ros2_control` vs `rover_hardware`).

## Testing

| Level | What | Where |
|-------|------|-------|
| Unit | ztest on `native_sim`: IMU driver (against an emulator), PID, filters, decoder, fault log on a flash simulator | CI `firmware` |
| Boot | Renode runs MCUboot and the signed app; a second test stages a hanging update and expects automatic rollback | `tools/renode`, CI `renode` |
| Software | ROS 2 build, hardware-interface test, URDF and parameter consistency checks | CI `ros2` |
| Hardware | `tools/hil/hil.py` on the Pi: gravity/IMU sanity, loop jitter, link-loss cutoff, velocity tracking | by hand with the board attached |

```sh
python3 tools/hil/hil.py --port /dev/ttyACM0                 # safe checks
python3 tools/hil/hil.py --port /dev/ttyACM0 --with-motors   # wheels off the ground!
```

See [`docs/status.md`](docs/status.md) for what has and has not been verified.
