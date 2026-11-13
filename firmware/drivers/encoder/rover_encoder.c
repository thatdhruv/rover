/* GPIO-interrupt quadrature encoder: both channels interrupt on either edge. */
#define DT_DRV_COMPAT rover_quad_encoder

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rover/encoder.h>
#include <rover/quad_decoder.h>

LOG_MODULE_REGISTER(rover_encoder, LOG_LEVEL_INF);

struct encoder_config {
	struct gpio_dt_spec a;
	struct gpio_dt_spec b;
	uint32_t counts_per_rev;
	bool invert;
};

struct encoder_data {
	const struct device *dev;
	struct gpio_callback cb_a;
	struct gpio_callback cb_b;
	struct rover_quad_decoder quad;
	struct k_spinlock lock;
};

/* Re-read both pins and feed the decoder; the pin state, not the edge that
 * fired, is the source of truth, so a late ISR still decodes correctly.
 */
static void encoder_sample(struct encoder_data *data)
{
	const struct encoder_config *cfg = data->dev->config;
	const int a = gpio_pin_get_raw(cfg->a.port, cfg->a.pin);
	const int b = gpio_pin_get_raw(cfg->b.port, cfg->b.pin);
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	rover_quad_update(&data->quad, a, b);
	k_spin_unlock(&data->lock, key);
}

static void encoder_isr_a(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	encoder_sample(CONTAINER_OF(cb, struct encoder_data, cb_a));
}

static void encoder_isr_b(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	encoder_sample(CONTAINER_OF(cb, struct encoder_data, cb_b));
}

static int32_t encoder_get_count(const struct device *dev)
{
	const struct encoder_config *cfg = dev->config;
	struct encoder_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	const int32_t count = data->quad.count;

	k_spin_unlock(&data->lock, key);
	return cfg->invert ? -count : count;
}

static uint32_t encoder_get_errors(const struct device *dev)
{
	struct encoder_data *data = dev->data;

	return data->quad.errors;
}

static void encoder_reset(const struct device *dev)
{
	const struct encoder_config *cfg = dev->config;
	struct encoder_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	rover_quad_init(&data->quad, gpio_pin_get_raw(cfg->a.port, cfg->a.pin),
			gpio_pin_get_raw(cfg->b.port, cfg->b.pin));
	k_spin_unlock(&data->lock, key);
}

static uint32_t encoder_counts_per_rev(const struct device *dev)
{
	const struct encoder_config *cfg = dev->config;

	return cfg->counts_per_rev;
}

static const struct rover_encoder_api encoder_api = {
	.get_count = encoder_get_count,
	.get_errors = encoder_get_errors,
	.reset = encoder_reset,
	.counts_per_rev = encoder_counts_per_rev,
};

static int encoder_init(const struct device *dev)
{
	const struct encoder_config *cfg = dev->config;
	struct encoder_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->a) || !gpio_is_ready_dt(&cfg->b)) {
		return -ENODEV;
	}
	data->dev = dev;

	ret = gpio_pin_configure_dt(&cfg->a, GPIO_INPUT);
	if (ret == 0) {
		ret = gpio_pin_configure_dt(&cfg->b, GPIO_INPUT);
	}
	if (ret < 0) {
		return ret;
	}
	encoder_reset(dev);

	gpio_init_callback(&data->cb_a, encoder_isr_a, BIT(cfg->a.pin));
	gpio_init_callback(&data->cb_b, encoder_isr_b, BIT(cfg->b.pin));
	ret = gpio_add_callback(cfg->a.port, &data->cb_a);
	if (ret == 0) {
		ret = gpio_add_callback(cfg->b.port, &data->cb_b);
	}
	if (ret == 0) {
		ret = gpio_pin_interrupt_configure_dt(&cfg->a, GPIO_INT_EDGE_BOTH);
	}
	if (ret == 0) {
		ret = gpio_pin_interrupt_configure_dt(&cfg->b, GPIO_INT_EDGE_BOTH);
	}
	if (ret < 0) {
		LOG_ERR("interrupt setup failed (%d)", ret);
	}
	return ret;
}

#define ENCODER_INIT(n)                                                                            \
	static struct encoder_data encoder_data_##n;                                               \
	static const struct encoder_config encoder_config_##n = {                                  \
		.a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                            \
		.b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                            \
		.counts_per_rev = DT_INST_PROP(n, counts_per_rev),                                 \
		.invert = DT_INST_PROP(n, invert),                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, encoder_init, NULL, &encoder_data_##n, &encoder_config_##n,       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &encoder_api);

DT_INST_FOREACH_STATUS_OKAY(ENCODER_INIT)
