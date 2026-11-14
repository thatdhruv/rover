# micro-ROS link

The firmware talks to ROS 2 through [micro-ROS](https://micro.ros.org) over UART (XRCE-DDS with
serial framing) to a `micro_ros_agent` on the Pi.

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `rover/wheel_cmd` | `rover_msgs/WheelCmd` | Pi → MCU | 100 Hz (every `ros2_control` write) |
| `rover/wheel_state` | `rover_msgs/WheelState` | MCU → Pi | 50 Hz |
| `rover/imu` | `sensor_msgs/Imu` | MCU → Pi | 50 Hz |

`wheel_cmd` doubles as the heartbeat. If it stops for 250 ms the firmware cuts the motors itself,
whether or not micro-ROS is up. The firmware reconnects automatically if the agent restarts.

## Wiring

USART6 on the Nucleo (PG14 TX, PG9 RX) to a Pi UART (`/dev/ttyAMA0`, GPIO14/15), 3.3 V, common
ground, 921600 baud. Enable the Pi's UART in `raspi-config` and disable the serial login console.

## Building the firmware with micro-ROS

micro-ROS is an opt-in west group because it pulls a ROS build into the firmware build.

```sh
west config manifest.group-filter -- +microros
west update
firmware/scripts/microros_extra_packages.sh      # makes rover_msgs available to the firmware
west build --sysbuild -b nucleo_f429zi firmware/app -- \
    -DEXTRA_CONF_FILE=microros.conf -DROVER_MICROROS=ON
```

The build needs the host tools libmicroros uses (colcon, catkin_pkg, lark, empy, cmake).

### Known caveats

- Upstream's `micro_ros_zephyr_module` has no `lyrical` branch yet; the manifest tracks `rolling`.
  The wire protocol is the same XRCE-DDS, so a Lyrical agent interoperates, but the module's own
  support for Zephyr 4.4 is the part most likely to need a pin or a patch.
- libmicroros needs newlib and the POSIX API, so `microros.conf` switches off picolibc.
- The module's serial transport hardcodes `usart1`, so this project supplies its own transport on
  the `rover-link-uart` devicetree alias (`firmware/app/src/microros_link.c`).

## Pi side

```sh
ros2 launch rover_bringup robot.launch.py serial_dev:=/dev/ttyAMA0
```

starts the agent, `ros2_control` with `rover_hardware`, the controllers, and the EKF.
