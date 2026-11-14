/*
 * Rover firmware entry point.
 * Starts sensing and the control loop, then the safety supervisor, which owns
 * the watchdog.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "rover_state.h"

LOG_MODULE_REGISTER(rover, LOG_LEVEL_INF);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	LOG_INF("rover firmware starting");
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	/* The watchdog must not start until every task that feeds it exists. */
	if (control_start() < 0 || imu_task_start() < 0 || safety_start() < 0) {
		LOG_ERR("startup failed; motors stay off");
		for (;;) {
			gpio_pin_toggle_dt(&led); /* fast blink = fault */
			k_msleep(100);
		}
	}
	LOG_INF("rover firmware running");

	if (IS_ENABLED(CONFIG_ROVER_TEST_BAD_IMAGE)) {
		k_msleep(1000);
		LOG_ERR("test: simulating a hung image");
		(void)irq_lock();
		for (;;) {
		}
	}

	for (;;) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}
	return 0;
}
