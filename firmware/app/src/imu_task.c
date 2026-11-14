/* 200 Hz IMU sampling and heading estimation. */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rover/faultlog.h>
#include <rover/heading_filter.h>

#include "rover_state.h"

LOG_MODULE_REGISTER(imu_task, LOG_LEVEL_INF);

#define IMU_PERIOD_MS    5
#define HEADING_TAU_S    2.0f
#define IMU_FAULT_THRESH 20 /* consecutive failed reads before logging a fault */

static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(rover_imu));

static void imu_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct rover_heading_filter hf;
	uint32_t fails = 0;
	int64_t next = k_uptime_get();

	rover_heading_init(&hf, HEADING_TAU_S);

	for (;;) {
		next += IMU_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));

		const int ret = sensor_sample_fetch(imu);

		if (ret < 0) {
			if (++fails == IMU_FAULT_THRESH) {
				safety_log_fault(ROVER_FAULT_IMU, (uint32_t)-ret, 0);
				LOG_ERR("IMU read failing (%d)", ret);
			}
			safety_task_alive(TASK_IMU); /* a bad sensor is a fault, not a hang */
			continue;
		}
		fails = 0;

		struct sensor_value acc[3], gyr[3];
		float accel[3];

		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr);
		for (int i = 0; i < 3; i++) {
			accel[i] = sensor_value_to_float(&acc[i]);
		}
		const float gyro_z = sensor_value_to_float(&gyr[2]);
		const float heading = rover_heading_update(&hf, gyro_z, state_odom_heading(), true,
							   IMU_PERIOD_MS / 1000.0f);

		state_publish_imu(heading, gyro_z, accel);
		safety_task_alive(TASK_IMU);
	}
}

K_THREAD_STACK_DEFINE(imu_stack, 2048);
static struct k_thread imu_thread;

int imu_task_start(void)
{
	if (!device_is_ready(imu)) {
		/* Degraded mode: wheel odometry still gives heading and the rover can drive. */
		LOG_WRN("IMU not ready; continuing without it");
		safety_log_fault(ROVER_FAULT_IMU, ENODEV, 0);
		safety_task_optional(TASK_IMU);
		state_set_imu_present(false);
		return 0;
	}
	k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_fn, NULL,
			NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread, "imu");
	return 0;
}
