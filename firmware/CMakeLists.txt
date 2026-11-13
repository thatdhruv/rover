# Rover firmware module: drivers and portable control libraries.
zephyr_library()
zephyr_include_directories(include)

zephyr_library_sources_ifdef(CONFIG_ROVER_MPU6050      drivers/imu/rover_mpu6050.c)
zephyr_library_sources_ifdef(CONFIG_ROVER_ENCODER      drivers/encoder/rover_encoder.c)
zephyr_library_sources_ifdef(CONFIG_ROVER_MOTOR        drivers/motor/rover_motor.c)
zephyr_library_sources_ifdef(CONFIG_ROVER_MPU6050_EMUL drivers/imu/rover_mpu6050_emul.c)

zephyr_library_sources(
  lib/pid.c
  lib/heading_filter.c
  lib/link_monitor.c
  lib/encoder_math.c
  lib/quad_decoder.c
  lib/jitter.c
)
zephyr_library_sources_ifdef(CONFIG_FLASH_MAP lib/faultlog.c)
