#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

#include <rover/rover_mpu6050_emul.h>
#include <rover/rover_mpu6050_regs.h>

#define IMU_NODE DT_NODELABEL(imu)

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct emul *const imu_emul = EMUL_DT_GET(IMU_NODE);

static double sv_to_double(const struct sensor_value *v)
{
	return (double)v->val1 + (double)v->val2 / 1e6;
}

ZTEST(imu, test_device_ready)
{
	zassert_true(device_is_ready(imu));
}

ZTEST(imu, test_init_programs_config_registers)
{
	/* 4 g -> AFS_SEL 1, 500 dps -> FS_SEL 1 */
	zassert_equal(rover_mpu6050_emul_get_reg(imu_emul, MPU6050_REG_ACCEL_CONFIG), 1 << 3);
	zassert_equal(rover_mpu6050_emul_get_reg(imu_emul, MPU6050_REG_GYRO_CONFIG), 1 << 3);
	zassert_equal(rover_mpu6050_emul_get_reg(imu_emul, MPU6050_REG_CONFIG), 3);
	zassert_equal(rover_mpu6050_emul_get_reg(imu_emul, MPU6050_REG_SMPLRT_DIV), 4);
	/* sleep cleared, PLL clock selected */
	zassert_equal(rover_mpu6050_emul_get_reg(imu_emul, MPU6050_REG_PWR_MGMT_1),
		      MPU6050_CLKSEL_PLL_XGYRO);
}

ZTEST(imu, test_accel_scaling)
{
	/* +-4 g => 8192 LSB/g. 1 g on Z, -0.5 g on X */
	const int16_t raw[7] = {-4096, 0, 8192, 0, 0, 0, 0};
	struct sensor_value acc[3];

	rover_mpu6050_emul_set_raw(imu_emul, raw);
	zassert_ok(sensor_sample_fetch(imu));
	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc));

	zassert_within(sv_to_double(&acc[0]), -4.903325, 1e-4);
	zassert_within(sv_to_double(&acc[1]), 0.0, 1e-6);
	zassert_within(sv_to_double(&acc[2]), 9.80665, 1e-4);
}

ZTEST(imu, test_gyro_scaling)
{
	/* +-500 dps => 65.5 LSB/dps. 655 raw = 10 dps = 0.1745 rad/s */
	const int16_t raw[7] = {0, 0, 0, 0, 655, -655, 0};
	struct sensor_value gyr[3];

	rover_mpu6050_emul_set_raw(imu_emul, raw);
	zassert_ok(sensor_sample_fetch(imu));
	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr));

	zassert_within(sv_to_double(&gyr[0]), 0.174533, 1e-4);
	zassert_within(sv_to_double(&gyr[1]), -0.174533, 1e-4);
	zassert_within(sv_to_double(&gyr[2]), 0.0, 1e-6);
}

ZTEST(imu, test_single_axis_channels_match_xyz)
{
	const int16_t raw[7] = {100, 200, 300, 0, 400, 500, 600};
	struct sensor_value xyz[3], single;

	rover_mpu6050_emul_set_raw(imu_emul, raw);
	zassert_ok(sensor_sample_fetch(imu));

	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, xyz));
	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &single));
	zassert_equal(single.val1, xyz[1].val1);
	zassert_equal(single.val2, xyz[1].val2);

	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, xyz));
	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &single));
	zassert_equal(single.val1, xyz[2].val1);
	zassert_equal(single.val2, xyz[2].val2);
}

ZTEST(imu, test_die_temperature)
{
	/* raw 0 => 36.53 C; raw 3400 => 46.53 C */
	int16_t raw[7] = {0};
	struct sensor_value t;

	rover_mpu6050_emul_set_raw(imu_emul, raw);
	zassert_ok(sensor_sample_fetch(imu));
	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &t));
	zassert_within(sv_to_double(&t), 36.53, 1e-3);

	raw[3] = 3400;
	rover_mpu6050_emul_set_raw(imu_emul, raw);
	zassert_ok(sensor_sample_fetch(imu));
	zassert_ok(sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &t));
	zassert_within(sv_to_double(&t), 46.53, 1e-3);
}

ZTEST(imu, test_bus_error_propagates)
{
	rover_mpu6050_emul_set_fail(imu_emul, true);
	zassert_equal(sensor_sample_fetch(imu), -EIO);
	rover_mpu6050_emul_set_fail(imu_emul, false);
	zassert_ok(sensor_sample_fetch(imu));
}

ZTEST(imu, test_unsupported_channel)
{
	struct sensor_value v;

	zassert_ok(sensor_sample_fetch(imu));
	zassert_equal(sensor_channel_get(imu, SENSOR_CHAN_HUMIDITY, &v), -ENOTSUP);
}

ZTEST_SUITE(imu, NULL, NULL, NULL, NULL, NULL);
