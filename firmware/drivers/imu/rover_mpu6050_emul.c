/* Register-level emulator for the MPU-6050 driver, for native_sim tests. */
#define DT_DRV_COMPAT rover_mpu6050

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include <rover/rover_mpu6050_emul.h>
#include <rover/rover_mpu6050_regs.h>

#define REG_FILE_SIZE 0x80

struct mpu6050_emul_data {
	uint8_t regs[REG_FILE_SIZE];
	uint8_t ptr;
	bool fail;
};

struct mpu6050_emul_cfg {
	uint16_t addr;
};

static int mpu6050_emul_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				 int addr)
{
	struct mpu6050_emul_data *data = target->data;
	const struct mpu6050_emul_cfg *cfg = target->cfg;

	if (addr != cfg->addr || data->fail) {
		return -EIO;
	}

	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];

		if (msg->flags & I2C_MSG_READ) {
			for (uint32_t j = 0; j < msg->len; j++) {
				msg->buf[j] = data->regs[data->ptr++ % REG_FILE_SIZE];
			}
		} else if (msg->len > 0) {
			data->ptr = msg->buf[0] % REG_FILE_SIZE;
			for (uint32_t j = 1; j < msg->len; j++) {
				data->regs[data->ptr++ % REG_FILE_SIZE] = msg->buf[j];
			}
		}
	}
	return 0;
}

static const struct i2c_emul_api mpu6050_emul_api = {
	.transfer = mpu6050_emul_transfer,
};

void rover_mpu6050_emul_set_raw(const struct emul *target, const int16_t raw[7])
{
	struct mpu6050_emul_data *data = target->data;

	for (int i = 0; i < MPU6050_NUM_RAW; i++) {
		sys_put_be16((uint16_t)raw[i], &data->regs[MPU6050_REG_ACCEL_XOUT_H + 2 * i]);
	}
}

uint8_t rover_mpu6050_emul_get_reg(const struct emul *target, uint8_t reg)
{
	struct mpu6050_emul_data *data = target->data;

	return data->regs[reg % REG_FILE_SIZE];
}

void rover_mpu6050_emul_set_fail(const struct emul *target, bool fail)
{
	struct mpu6050_emul_data *data = target->data;

	data->fail = fail;
}

static int mpu6050_emul_init(const struct emul *target, const struct device *parent)
{
	struct mpu6050_emul_data *data = target->data;

	ARG_UNUSED(parent);
	memset(data->regs, 0, sizeof(data->regs));
	data->regs[MPU6050_REG_WHO_AM_I] = 0x68;
	data->regs[MPU6050_REG_PWR_MGMT_1] = MPU6050_PWR_MGMT_1_SLEEP;
	return 0;
}

#define MPU6050_EMUL(n)                                                                            \
	static struct mpu6050_emul_data mpu6050_emul_data_##n;                                     \
	static const struct mpu6050_emul_cfg mpu6050_emul_cfg_##n = {                              \
		.addr = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, mpu6050_emul_init, &mpu6050_emul_data_##n, &mpu6050_emul_cfg_##n,   \
			    &mpu6050_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(MPU6050_EMUL)
