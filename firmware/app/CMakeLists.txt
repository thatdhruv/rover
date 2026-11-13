cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rover_firmware)

target_sources(app PRIVATE
  src/main.c
  src/control.c
  src/imu_task.c
  src/safety.c
  src/shell_cmds.c
)
