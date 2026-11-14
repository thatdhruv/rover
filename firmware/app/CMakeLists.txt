cmake_minimum_required(VERSION 3.20.0)
# micro-ROS is optional: it needs the extra west group and a ROS toolchain at build time.
if(ROVER_MICROROS OR "$ENV{ROVER_MICROROS}")
  list(APPEND ZEPHYR_EXTRA_MODULES
       ${CMAKE_CURRENT_LIST_DIR}/../../modules/lib/micro_ros_zephyr_module/modules/libmicroros)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rover_firmware)

target_sources(app PRIVATE
  src/main.c
  src/control.c
  src/imu_task.c
  src/safety.c
  src/shell_cmds.c
)

target_sources_ifdef(CONFIG_ROVER_MICROROS app PRIVATE src/microros_link.c)
