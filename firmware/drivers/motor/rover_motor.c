/* H-bridge DC motor: PWM on the enable pin, two GPIOs select direction. */
#define DT_DRV_COMPAT rover_dc_motor

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rover/motor.h>

LOG_MODULE_REGISTER(rover_motor, LOG_LEVEL_INF);

struct motor_config {
	struct pwm_dt_spec pwm;
	struct gpio_dt_spec in1;
	struct gpio_dt_spec in2;
	float deadband;
};

struct motor_data {
	atomic_t cut_off;
};

/* Coast: both direction pins low and 0 % duty. Safe to call from an ISR. */
static void motor_coast(const struct motor_config *cfg)
{
	pwm_set_pulse_dt(&cfg->pwm, 0);
	gpio_pin_set_dt(&cfg->in1, 0);
	gpio_pin_set_dt(&cfg->in2, 0);
}

static int motor_set_duty(const struct device *dev, float duty)
{
	const struct motor_config *cfg = dev->config;
	struct motor_data *data = dev->data;
	const float mag = fabsf(duty);

	if (atomic_get(&data->cut_off)) {
		return -EPERM;
	}
	if (mag < cfg->deadband) {
		motor_coast(cfg);
		return 0;
	}

	/* direction pins first, then PWM, so we never drive the wrong way */
	gpio_pin_set_dt(&cfg->in1, duty > 0.0f);
	gpio_pin_set_dt(&cfg->in2, duty < 0.0f);
	return pwm_set_pulse_dt(&cfg->pwm, (uint32_t)(fminf(mag, 1.0f) * (float)cfg->pwm.period));
}

static void motor_cutoff(const struct device *dev)
{
	const struct motor_config *cfg = dev->config;
	struct motor_data *data = dev->data;

	atomic_set(&data->cut_off, 1);
	motor_coast(cfg);
}

static void motor_rearm(const struct device *dev)
{
	struct motor_data *data = dev->data;

	atomic_set(&data->cut_off, 0);
}

static const struct rover_motor_api motor_api = {
	.set_duty = motor_set_duty,
	.cutoff = motor_cutoff,
	.rearm = motor_rearm,
};

static int motor_init(const struct device *dev)
{
	const struct motor_config *cfg = dev->config;
	int ret;

	if (!pwm_is_ready_dt(&cfg->pwm) || !gpio_is_ready_dt(&cfg->in1) ||
	    !gpio_is_ready_dt(&cfg->in2)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&cfg->in1, GPIO_OUTPUT_INACTIVE);
	if (ret == 0) {
		ret = gpio_pin_configure_dt(&cfg->in2, GPIO_OUTPUT_INACTIVE);
	}
	if (ret < 0) {
		return ret;
	}
	motor_coast(cfg);
	return 0;
}

#define MOTOR_INIT(n)                                                                              \
	static struct motor_data motor_data_##n;                                                   \
	static const struct motor_config motor_config_##n = {                                      \
		.pwm = PWM_DT_SPEC_INST_GET(n),                                                    \
		.in1 = GPIO_DT_SPEC_INST_GET(n, in1_gpios),                                        \
		.in2 = GPIO_DT_SPEC_INST_GET(n, in2_gpios),                                        \
		.deadband = DT_INST_PROP(n, deadband_permille) / 1000.0f,                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, motor_init, NULL, &motor_data_##n, &motor_config_##n,             \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &motor_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_INIT)
