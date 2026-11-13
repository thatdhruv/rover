/*
 * 1 kHz wheel-velocity control loop.
 *
 * A k_timer releases a semaphore every millisecond; the control thread
 * (highest cooperative-safe priority) wakes, samples the encoders, runs a PID
 * per wheel and writes the motor duty. Wake-up latency against an ideal
 * 1 ms grid (derived from the cycle counter) is recorded as jitter.
 */
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <rover/encoder.h>
#include <rover/encoder_math.h>
#include <rover/faultlog.h>
#include <rover/heading_filter.h>
#include <rover/jitter.h>
#include <rover/link_monitor.h>
#include <rover/motor.h>
#include <rover/pid.h>

#include "rover_state.h"

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

#define WHEEL_RADIUS_M   0.033f
#define TRACK_WIDTH_M    0.160f
#define LOOP_HZ          1000
#define LOOP_DT          (1.0f / LOOP_HZ)
#define VEL_WINDOW       20
#define LINK_TIMEOUT_MS  250
#define MAX_WHEEL_RADS   30.0f
#define PID_KP           0.08f
#define PID_KI           0.9f
#define PID_KD           0.0f
/* feed-forward: duty per rad/s, roughly 1 / (no-load speed) */
#define FF_DUTY_PER_RADS (1.0f / 35.0f)

static const struct device *const motors[NUM_WHEELS] = {
	DEVICE_DT_GET(DT_ALIAS(rover_motor_left)),
	DEVICE_DT_GET(DT_ALIAS(rover_motor_right)),
};
static const struct device *const encoders[NUM_WHEELS] = {
	DEVICE_DT_GET(DT_ALIAS(rover_enc_left)),
	DEVICE_DT_GET(DT_ALIAS(rover_enc_right)),
};

static struct {
	struct k_spinlock lock;
	float cmd[NUM_WHEELS];
	struct rover_state state;
	struct rover_jitter jitter;
	struct rover_link_monitor link;
	float odom_heading;
} ctl;

K_SEM_DEFINE(tick_sem, 0, 1);
static struct k_timer tick_timer;

/* ---- command / state API ---- */

void rover_cmd_set_wheel_velocity(float left, float right)
{
	k_spinlock_key_t key = k_spin_lock(&ctl.lock);

	ctl.cmd[WHEEL_LEFT] = CLAMP(left, -MAX_WHEEL_RADS, MAX_WHEEL_RADS);
	ctl.cmd[WHEEL_RIGHT] = CLAMP(right, -MAX_WHEEL_RADS, MAX_WHEEL_RADS);
	rover_link_feed(&ctl.link, k_uptime_get_32());
	k_spin_unlock(&ctl.lock, key);
}

void rover_cmd_heartbeat(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctl.lock);

	rover_link_feed(&ctl.link, k_uptime_get_32());
	k_spin_unlock(&ctl.lock, key);
}

void rover_state_get(struct rover_state *out)
{
	k_spinlock_key_t key = k_spin_lock(&ctl.lock);

	*out = ctl.state;
	k_spin_unlock(&ctl.lock, key);
}

void rover_jitter_get(struct rover_jitter *out)
{
	k_spinlock_key_t key = k_spin_lock(&ctl.lock);

	*out = ctl.jitter;
	k_spin_unlock(&ctl.lock, key);
}

void state_publish_imu(float heading, float gyro_z, const float accel[3])
{
	k_spinlock_key_t key = k_spin_lock(&ctl.lock);

	ctl.state.heading = heading;
	ctl.state.gyro_z = gyro_z;
	for (int i = 0; i < 3; i++) {
		ctl.state.accel[i] = accel[i];
	}
	k_spin_unlock(&ctl.lock, key);
}

float state_odom_heading(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctl.lock);
	const float h = ctl.odom_heading;

	k_spin_unlock(&ctl.lock, key);
	return h;
}

/* ---- the loop ---- */

static void tick_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_sem_give(&tick_sem);
}

static void control_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct rover_pid pid[NUM_WHEELS];
	struct rover_vel_est est[NUM_WHEELS];
	const uint32_t cyc_per_loop = sys_clock_hw_cycles_per_sec() / LOOP_HZ;
	bool link_was_alive = false;
	bool cutoff = false;
	int32_t prev_counts[NUM_WHEELS] = {0};
	float odom_heading = 0.0f;
	uint32_t expected;

	for (int i = 0; i < NUM_WHEELS; i++) {
		rover_pid_init(&pid[i], PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
		rover_vel_est_init(&est[i], VEL_WINDOW);
		rover_encoder_reset(encoders[i]);
	}
	rover_jitter_reset(&ctl.jitter);

	k_timer_init(&tick_timer, tick_expiry, NULL);
	expected = k_cycle_get_32() + cyc_per_loop;
	k_timer_start(&tick_timer, K_MSEC(1), K_MSEC(1));

	for (;;) {
		k_sem_take(&tick_sem, K_FOREVER);
		const uint32_t now_cyc = k_cycle_get_32();
		const int32_t err_cyc = (int32_t)(now_cyc - expected);

		expected += cyc_per_loop;

		float meas[NUM_WHEELS], cmd[NUM_WHEELS], duty[NUM_WHEELS] = {0};
		int32_t counts[NUM_WHEELS];
		const uint32_t now_ms = k_uptime_get_32();

		for (int i = 0; i < NUM_WHEELS; i++) {
			counts[i] = rover_encoder_get_count(encoders[i]);
			meas[i] = rover_vel_est_update(&est[i], counts[i],
						       (float)rover_encoder_counts_per_rev(encoders[i]),
						       LOOP_DT);
		}

		/* differential-drive heading from wheel travel: dtheta = (dR - dL) / track */
		const float rad_per_count = 6.28318531f / (float)rover_encoder_counts_per_rev(encoders[0]);

		odom_heading = rover_wrap_pi(
			odom_heading + (float)(rover_enc_delta(prev_counts[WHEEL_RIGHT], counts[WHEEL_RIGHT]) -
					       rover_enc_delta(prev_counts[WHEEL_LEFT], counts[WHEEL_LEFT])) *
					       rad_per_count * WHEEL_RADIUS_M / TRACK_WIDTH_M);
		for (int i = 0; i < NUM_WHEELS; i++) {
			prev_counts[i] = counts[i];
		}

		k_spinlock_key_t key = k_spin_lock(&ctl.lock);
		ctl.odom_heading = odom_heading;
		const bool alive = rover_link_alive(&ctl.link, now_ms);

		if (!alive) {
			/* host gone: zero the setpoints so recovery starts from rest */
			ctl.cmd[WHEEL_LEFT] = 0.0f;
			ctl.cmd[WHEEL_RIGHT] = 0.0f;
		}
		cmd[WHEEL_LEFT] = ctl.cmd[WHEEL_LEFT];
		cmd[WHEEL_RIGHT] = ctl.cmd[WHEEL_RIGHT];
		k_spin_unlock(&ctl.lock, key);

		if (link_was_alive && !alive) {
			for (int i = 0; i < NUM_WHEELS; i++) {
				rover_motor_cutoff(motors[i]);
				rover_pid_reset(&pid[i]);
			}
			cutoff = true;
			safety_log_fault(ROVER_FAULT_LINK_LOST, LINK_TIMEOUT_MS, 0);
			LOG_WRN("link lost: motors cut off");
		} else if (alive && cutoff) {
			for (int i = 0; i < NUM_WHEELS; i++) {
				rover_motor_rearm(motors[i]);
			}
			cutoff = false;
			LOG_INF("link restored: motors re-armed");
		}
		link_was_alive = alive;

		for (int i = 0; i < NUM_WHEELS; i++) {
			if (cutoff) {
				break;
			}
			duty[i] = rover_pid_update(&pid[i], cmd[i], meas[i], LOOP_DT) +
				  cmd[i] * FF_DUTY_PER_RADS;
			duty[i] = CLAMP(duty[i], -1.0f, 1.0f);
			if (cmd[i] == 0.0f && fabsf(meas[i]) < 0.2f) {
				duty[i] = 0.0f; /* hold still without hunting */
				rover_pid_reset(&pid[i]);
			}
			rover_motor_set_duty(motors[i], duty[i]);
		}

		key = k_spin_lock(&ctl.lock);
		rover_jitter_record(&ctl.jitter,
				    (int32_t)((int64_t)err_cyc * 1000000000LL /
					      sys_clock_hw_cycles_per_sec()));
		for (int i = 0; i < NUM_WHEELS; i++) {
			ctl.state.wheel_vel[i] = meas[i];
			ctl.state.wheel_cmd[i] = cmd[i];
			ctl.state.wheel_duty[i] = duty[i];
			ctl.state.wheel_count[i] = counts[i];
		}
		ctl.state.link_alive = alive;
		ctl.state.motors_cut_off = cutoff;
		ctl.state.uptime_ms = now_ms;
		k_spin_unlock(&ctl.lock, key);

		safety_task_alive(TASK_CONTROL);
	}
}

K_THREAD_STACK_DEFINE(control_stack, 2048);
static struct k_thread control_thread;

int control_start(void)
{
	for (int i = 0; i < NUM_WHEELS; i++) {
		if (!device_is_ready(motors[i]) || !device_is_ready(encoders[i])) {
			LOG_ERR("wheel %d hardware not ready", i);
			return -ENODEV;
		}
	}
	rover_link_init(&ctl.link, LINK_TIMEOUT_MS);

	k_thread_create(&control_thread, control_stack, K_THREAD_STACK_SIZEOF(control_stack),
			control_fn, NULL, NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&control_thread, "control");
	return 0;
}
