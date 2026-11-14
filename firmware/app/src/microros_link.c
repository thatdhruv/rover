/*
 * micro-ROS link to the Raspberry Pi over UART (XRCE-DDS, serial framing).
 *
 * Publishes  rover/wheel_state (rover_msgs/WheelState) and rover/imu (sensor_msgs/Imu) at 50 Hz.
 * Subscribes rover/wheel_cmd   (rover_msgs/WheelCmd), which also acts as the heartbeat for
 * the link monitor: if commands stop, the control loop cuts the motors on its own.
 *
 * The transport is our own (not the module's) so it can use the dedicated link UART instead
 * of a hardcoded USART.
 */
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rover_msgs/msg/wheel_cmd.h>
#include <rover_msgs/msg/wheel_state.h>
#include <sensor_msgs/msg/imu.h>
#include <uxr/client/transport.h>

#include "rover_state.h"

LOG_MODULE_REGISTER(microros, LOG_LEVEL_INF);

#define PUBLISH_PERIOD_MS 20
#define PING_TIMEOUT_MS   100
#define MAX_MISSED_PINGS  5

static const struct device *const link_uart = DEVICE_DT_GET(DT_ALIAS(rover_link_uart));
static const float rad_per_count[NUM_WHEELS] = {
	6.2831853f / DT_PROP(DT_ALIAS(rover_enc_left), counts_per_rev),
	6.2831853f / DT_PROP(DT_ALIAS(rover_enc_right), counts_per_rev),
};

/* ---- UART transport ---- */

RING_BUF_DECLARE(rx_ring, 512);

static void uart_isr(const struct device *dev, void *user)
{
	uint8_t buf[32];

	ARG_UNUSED(user);
	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		const int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		ring_buf_put(&rx_ring, buf, n);
	}
}

static bool transport_open(struct uxrCustomTransport *t)
{
	ARG_UNUSED(t);
	if (!device_is_ready(link_uart)) {
		return false;
	}
	ring_buf_reset(&rx_ring);
	uart_irq_callback_user_data_set(link_uart, uart_isr, NULL);
	uart_irq_rx_enable(link_uart);
	return true;
}

static bool transport_close(struct uxrCustomTransport *t)
{
	ARG_UNUSED(t);
	uart_irq_rx_disable(link_uart);
	return true;
}

static size_t transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len,
			      uint8_t *err)
{
	ARG_UNUSED(t);
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(link_uart, buf[i]);
	}
	*err = 0;
	return len;
}

static size_t transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout,
			     uint8_t *err)
{
	const int64_t deadline = k_uptime_get() + timeout;
	size_t got = 0;

	ARG_UNUSED(t);
	*err = 0;
	for (;;) {
		got += ring_buf_get(&rx_ring, buf + got, len - got);
		if (got == len || k_uptime_get() >= deadline) {
			return got;
		}
		k_msleep(1);
	}
}

/* ---- ROS entities ---- */

static rcl_publisher_t state_pub, imu_pub;
static rcl_subscription_t cmd_sub;
static rcl_timer_t timer;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;

static rover_msgs__msg__WheelState state_msg;
static rover_msgs__msg__WheelCmd cmd_msg;
static sensor_msgs__msg__Imu imu_msg;
static char imu_frame[] = "imu_link";

static void cmd_cb(const void *msgin)
{
	const rover_msgs__msg__WheelCmd *m = msgin;

	rover_cmd_set_wheel_velocity(m->left, m->right);
}

static void timer_cb(rcl_timer_t *t, int64_t last_call)
{
	struct rover_state s;

	ARG_UNUSED(last_call);
	if (t == NULL) {
		return;
	}
	rover_state_get(&s);

	state_msg.left_pos = s.wheel_count[WHEEL_LEFT] * rad_per_count[WHEEL_LEFT];
	state_msg.right_pos = s.wheel_count[WHEEL_RIGHT] * rad_per_count[WHEEL_RIGHT];
	state_msg.left_vel = s.wheel_vel[WHEEL_LEFT];
	state_msg.right_vel = s.wheel_vel[WHEEL_RIGHT];
	state_msg.heading = s.heading;
	state_msg.link_alive = s.link_alive;
	state_msg.motors_cut_off = s.motors_cut_off;
	state_msg.uptime_ms = s.uptime_ms;
	rcl_publish(&state_pub, &state_msg, NULL);

	const int64_t ms = k_uptime_get();

	imu_msg.header.stamp.sec = (int32_t)(ms / 1000);
	imu_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000);
	imu_msg.orientation.z = sinf(s.heading * 0.5f);
	imu_msg.orientation.w = cosf(s.heading * 0.5f);
	imu_msg.angular_velocity.z = s.gyro_z;
	imu_msg.linear_acceleration.x = s.accel[0];
	imu_msg.linear_acceleration.y = s.accel[1];
	imu_msg.linear_acceleration.z = s.accel[2];
	rcl_publish(&imu_pub, &imu_msg, NULL);
}

static void imu_msg_init(void)
{
	sensor_msgs__msg__Imu__init(&imu_msg);
	imu_msg.header.frame_id.data = imu_frame;
	imu_msg.header.frame_id.size = sizeof(imu_frame) - 1;
	imu_msg.header.frame_id.capacity = sizeof(imu_frame);
	/* diagonal covariances: roll/pitch unobserved, yaw from the heading filter */
	imu_msg.orientation_covariance[0] = 1e3;
	imu_msg.orientation_covariance[4] = 1e3;
	imu_msg.orientation_covariance[8] = 0.02;
	imu_msg.angular_velocity_covariance[0] = 0.01;
	imu_msg.angular_velocity_covariance[4] = 0.01;
	imu_msg.angular_velocity_covariance[8] = 0.0004;
	imu_msg.linear_acceleration_covariance[0] = 0.05;
	imu_msg.linear_acceleration_covariance[4] = 0.05;
	imu_msg.linear_acceleration_covariance[8] = 0.05;
}

static bool entities_create(void)
{
	allocator = rcl_get_default_allocator();
	if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK ||
	    rclc_node_init_default(&node, "rover_fw", "", &support) != RCL_RET_OK) {
		return false;
	}
	if (rclc_publisher_init_best_effort(&state_pub, &node,
					    ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, WheelState),
					    "rover/wheel_state") != RCL_RET_OK ||
	    rclc_publisher_init_best_effort(&imu_pub, &node,
					    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
					    "rover/imu") != RCL_RET_OK ||
	    rclc_subscription_init_best_effort(&cmd_sub, &node,
					       ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, WheelCmd),
					       "rover/wheel_cmd") != RCL_RET_OK ||
	    rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_cb,
				     true) != RCL_RET_OK) {
		return false;
	}
	executor = rclc_executor_get_zero_initialized_executor();
	return rclc_executor_init(&executor, &support.context, 2, &allocator) == RCL_RET_OK &&
	       rclc_executor_add_timer(&executor, &timer) == RCL_RET_OK &&
	       rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, cmd_cb, ON_NEW_DATA) ==
		       RCL_RET_OK;
}

static void entities_destroy(void)
{
	rmw_context_t *rmw = rcl_context_get_rmw_context(&support.context);

	(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
	rcl_publisher_fini(&state_pub, &node);
	rcl_publisher_fini(&imu_pub, &node);
	rcl_subscription_fini(&cmd_sub, &node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

static void microros_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	rover_msgs__msg__WheelState__init(&state_msg);
	rover_msgs__msg__WheelCmd__init(&cmd_msg);
	imu_msg_init();

	rmw_uros_set_custom_transport(true, NULL, transport_open, transport_close, transport_write,
				      transport_read);

	for (;;) {
		/* wait for the agent on the Pi; the control loop is safe either way */
		while (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
			k_msleep(200);
		}
		if (!entities_create()) {
			LOG_ERR("micro-ROS entity creation failed; retrying");
			entities_destroy();
			k_msleep(1000);
			continue;
		}
		LOG_INF("micro-ROS agent connected");

		int missed = 0;

		while (missed < MAX_MISSED_PINGS) {
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
			missed = (rmw_uros_ping_agent(20, 1) == RMW_RET_OK) ? 0 : missed + 1;
		}
		LOG_WRN("micro-ROS agent lost; reconnecting");
		entities_destroy();
	}
}

K_THREAD_STACK_DEFINE(microros_stack, 16384);
static struct k_thread microros_thread;

int microros_start(void)
{
	k_thread_create(&microros_thread, microros_stack, K_THREAD_STACK_SIZEOF(microros_stack),
			microros_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
	k_thread_name_set(&microros_thread, "microros");
	return 0;
}
