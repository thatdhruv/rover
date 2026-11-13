/*
 * Hand-written driver for MPU-6050 compatible 6-axis IMUs.
 * Exposes the standard Zephyr sensor API; all configuration comes from the
 * device tree (see dts/bindings/sensor/rover,mpu6050.yaml).
 */
#define DT_DRV_COMPAT rover_mpu6050

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <rover/rover_mpu6050_regs.h>

LOG_MODULE_REGISTER(rover_mpu6050, CONFIG_SENSOR_LOG_LEVEL);

struct mpu6050_config {
	struct i2c_dt_spec bus;
	uint8_t accel_fs; /* ACCEL_CONFIG AFS_SEL, 0..3 */
	uint8_t gyro_fs;  /* GYRO_CONFIG FS_SEL, 0..3 */
	uint8_t dlpf;
	uint8_t smplrt_div;
	uint8_t who_am_i;
};

struct mpu6050_data {
	int16_t raw[MPU6050_NUM_RAW]; /* ax ay az temp gx gy gz */
};

/* raw order inside the 14-byte burst */
enum { RAW_AX, RAW_AY, RAW_AZ, RAW_TEMP, RAW_GX, RAW_GY, RAW_GZ };

/* micro-rad/s per (degree/s), scaled by 10 to match the x10 LSB table below */
#define URAD_PER_DPS_X10 174533LL
#define UMS2_PER_G       9806650LL

/* log2(range / first): 2g -> 0, 4g -> 1, ... */
#define RANGE_TO_FS(range, first) (LOG2((range) / (first)))

static void micro_to_sensor_value(struct sensor_value *val, int64_t micro)
{
	val->val1 = (int32_t)(micro / 1000000LL);
	val->val2 = (int32_t)(micro % 1000000LL);
}

static int64_t accel_to_micro_ms2(const struct mpu6050_config *cfg, int16_t raw)
{
	/* 16384 LSB/g at +-2 g, halving per range step */
	const int64_t lsb_per_g = 16384 >> cfg->accel_fs;

	return ((int64_t)raw * UMS2_PER_G) / lsb_per_g;
}

static int64_t gyro_to_micro_rads(const struct mpu6050_config *cfg, int16_t raw)
{
	/* 131 LSB/dps at +-250 dps, halving per step (x10 to stay integral) */
	static const int32_t lsb_per_dps_x10[] = {1310, 655, 328, 164};

	return ((int64_t)raw * URAD_PER_DPS_X10) / lsb_per_dps_x10[cfg->gyro_fs];
}

static int mpu6050_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct mpu6050_config *cfg = dev->config;
	struct mpu6050_data *data = dev->data;
	uint8_t buf[MPU6050_SAMPLE_BYTES];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ACCEL_XYZ &&
	    chan != SENSOR_CHAN_GYRO_XYZ && chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	ret = i2c_burst_read_dt(&cfg->bus, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("sample read failed (%d)", ret);
		return ret;
	}

	for (int i = 0; i < MPU6050_NUM_RAW; i++) {
		data->raw[i] = (int16_t)sys_get_be16(&buf[i * 2]);
	}
	return 0;
}

static int mpu6050_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct mpu6050_config *cfg = dev->config;
	const struct mpu6050_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		for (int i = 0; i < 3; i++) {
			micro_to_sensor_value(&val[i],
					      accel_to_micro_ms2(cfg, data->raw[RAW_AX + i]));
		}
		return 0;
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		micro_to_sensor_value(
			val, accel_to_micro_ms2(cfg, data->raw[RAW_AX + (chan - SENSOR_CHAN_ACCEL_X)]));
		return 0;
	case SENSOR_CHAN_GYRO_XYZ:
		for (int i = 0; i < 3; i++) {
			micro_to_sensor_value(&val[i],
					      gyro_to_micro_rads(cfg, data->raw[RAW_GX + i]));
		}
		return 0;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		micro_to_sensor_value(
			val, gyro_to_micro_rads(cfg, data->raw[RAW_GX + (chan - SENSOR_CHAN_GYRO_X)]));
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		/* degC = raw / 340 + 36.53 */
		micro_to_sensor_value(val,
				      (int64_t)data->raw[RAW_TEMP] * 1000000LL / 340 + 36530000LL);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api mpu6050_api = {
	.sample_fetch = mpu6050_sample_fetch,
	.channel_get = mpu6050_channel_get,
};

static int mpu6050_write(const struct mpu6050_config *cfg, uint8_t reg, uint8_t val)
{
	int ret = i2c_reg_write_byte_dt(&cfg->bus, reg, val);

	if (ret < 0) {
		LOG_ERR("write reg 0x%02x failed (%d)", reg, ret);
	}
	return ret;
}

static int mpu6050_init(const struct device *dev)
{
	const struct mpu6050_config *cfg = dev->config;
	uint8_t id;
	int ret;

	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&cfg->bus, MPU6050_REG_WHO_AM_I, &id);
	if (ret < 0) {
		LOG_ERR("WHO_AM_I read failed (%d)", ret);
		return ret;
	}
	if (id != cfg->who_am_i) {
		LOG_ERR("unexpected WHO_AM_I 0x%02x (want 0x%02x)", id, cfg->who_am_i);
		return -ENODEV;
	}

	/* Reset, then wake with the gyro PLL as clock source. */
	ret = mpu6050_write(cfg, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET);
	if (ret < 0) {
		return ret;
	}
	k_msleep(100);
	ret = mpu6050_write(cfg, MPU6050_REG_PWR_MGMT_1, MPU6050_CLKSEL_PLL_XGYRO);
	if (ret < 0) {
		return ret;
	}
	k_msleep(10);

	ret = mpu6050_write(cfg, MPU6050_REG_CONFIG, cfg->dlpf);
	if (ret < 0) {
		return ret;
	}
	ret = mpu6050_write(cfg, MPU6050_REG_SMPLRT_DIV, cfg->smplrt_div);
	if (ret < 0) {
		return ret;
	}
	ret = mpu6050_write(cfg, MPU6050_REG_GYRO_CONFIG, cfg->gyro_fs << 3);
	if (ret < 0) {
		return ret;
	}
	return mpu6050_write(cfg, MPU6050_REG_ACCEL_CONFIG, cfg->accel_fs << 3);
}

#define MPU6050_INIT(n)                                                                            \
	static struct mpu6050_data mpu6050_data_##n;                                               \
	static const struct mpu6050_config mpu6050_config_##n = {                                  \
		.bus = I2C_DT_SPEC_INST_GET(n),                                                    \
		.accel_fs = RANGE_TO_FS(DT_INST_PROP(n, accel_range_g), 2),                        \
		.gyro_fs = RANGE_TO_FS(DT_INST_PROP(n, gyro_range_dps), 250),                      \
		.dlpf = DT_INST_PROP(n, dlpf_cfg),                                                 \
		.smplrt_div = DT_INST_PROP(n, sample_rate_div),                                    \
		.who_am_i = DT_INST_PROP(n, who_am_i),                                             \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(n, mpu6050_init, NULL, &mpu6050_data_##n,                     \
				     &mpu6050_config_##n, POST_KERNEL,                             \
				     CONFIG_SENSOR_INIT_PRIORITY, &mpu6050_api);

DT_INST_FOREACH_STATUS_OKAY(MPU6050_INIT)
