#include <math.h>

#include <zephyr/ztest.h>

#include <rover/encoder_math.h>
#include <rover/heading_filter.h>
#include <rover/link_monitor.h>
#include <rover/pid.h>

#define ZNEAR(a, b, d) zassert_within((double)(a), (double)(b), (double)(d))
#include <rover/quad_decoder.h>

/* ---- PID ---- */

ZTEST(pid, test_proportional_only)
{
	struct rover_pid pid;

	rover_pid_init(&pid, 2.0f, 0.0f, 0.0f, -10.0f, 10.0f);
	ZNEAR(rover_pid_update(&pid, 3.0f, 1.0f, 0.001f), 4.0f, 1e-6);
}

ZTEST(pid, test_output_clamped)
{
	struct rover_pid pid;

	rover_pid_init(&pid, 100.0f, 0.0f, 0.0f, -1.0f, 1.0f);
	ZNEAR(rover_pid_update(&pid, 10.0f, 0.0f, 0.001f), 1.0f, 1e-6);
	ZNEAR(rover_pid_update(&pid, -10.0f, 0.0f, 0.001f), -1.0f, 1e-6);
}

ZTEST(pid, test_integral_accumulates)
{
	struct rover_pid pid;
	float out = 0.0f;

	rover_pid_init(&pid, 0.0f, 10.0f, 0.0f, -100.0f, 100.0f);
	for (int i = 0; i < 100; i++) {
		out = rover_pid_update(&pid, 1.0f, 0.0f, 0.01f);
	}
	/* integral of 10 * 1 over 1 s */
	ZNEAR(out, 10.0f, 0.2f);
}

ZTEST(pid, test_anti_windup_recovers_quickly)
{
	struct rover_pid pid;

	rover_pid_init(&pid, 0.0f, 100.0f, 0.0f, -1.0f, 1.0f);
	/* saturate hard for 10 s of simulated time */
	for (int i = 0; i < 10000; i++) {
		rover_pid_update(&pid, 100.0f, 0.0f, 0.001f);
	}
	/* with a wound-up integrator this would stay pinned at +1 for a long time;
	 * with conditional integration the integrator stays near the limit
	 */
	zassert_true(pid.integral < 1.5f, "integral wound up: %f", (double)pid.integral);
	/* a reversed error must pull the output off the rail within ~20 ms */
	float out = 1.0f;

	for (int i = 0; i < 20; i++) {
		out = rover_pid_update(&pid, -100.0f, 0.0f, 0.001f);
	}
	zassert_true(out < 1.0f, "output still pinned: %f", (double)out);
}

ZTEST(pid, test_no_derivative_kick_on_setpoint_step)
{
	struct rover_pid pid;

	rover_pid_init(&pid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
	rover_pid_update(&pid, 0.0f, 5.0f, 0.001f);
	/* measurement unchanged, setpoint jumps: derivative term must stay zero */
	ZNEAR(rover_pid_update(&pid, 50.0f, 5.0f, 0.001f), 0.0f, 1e-6);
}

ZTEST(pid, test_closed_loop_first_order_plant)
{
	struct rover_pid pid;
	float vel = 0.0f;

	/* plant: tau dv/dt = u - v ; check PI drives velocity to the setpoint */
	rover_pid_init(&pid, 2.0f, 8.0f, 0.0f, -12.0f, 12.0f);
	for (int i = 0; i < 3000; i++) {
		const float u = rover_pid_update(&pid, 5.0f, vel, 0.001f);

		vel += (u - vel) / 0.1f * 0.001f;
	}
	ZNEAR(vel, 5.0f, 0.05f);
}

ZTEST_SUITE(pid, NULL, NULL, NULL, NULL, NULL);

/* ---- heading filter ---- */

ZTEST(heading, test_integrates_gyro_without_reference)
{
	struct rover_heading_filter f;
	float h = 0.0f;

	rover_heading_init(&f, 1.0f);
	rover_heading_update(&f, 0.0f, 0.0f, true, 0.01f); /* prime at 0 */
	for (int i = 0; i < 100; i++) {
		h = rover_heading_update(&f, 0.5f, 0.0f, false, 0.01f);
	}
	ZNEAR(h, 0.5f, 1e-3);
}

ZTEST(heading, test_reference_removes_gyro_bias)
{
	struct rover_heading_filter f;
	float h = 0.0f;

	rover_heading_init(&f, 0.5f);
	rover_heading_update(&f, 0.0f, 0.0f, true, 0.01f);
	/* gyro reports 0.05 rad/s while the robot is actually stationary */
	for (int i = 0; i < 3000; i++) {
		h = rover_heading_update(&f, 0.05f, 0.0f, true, 0.01f);
	}
	/* steady-state error is bias * tau = 0.025 rad, not the 1.5 rad of pure integration */
	ZNEAR(h, 0.025f, 0.005f);
}

ZTEST(heading, test_wraps_across_pi)
{
	struct rover_heading_filter f;
	const float pi = 3.14159265f;
	float h;

	rover_heading_init(&f, 0.5f);
	rover_heading_update(&f, 0.0f, pi - 0.01f, true, 0.01f);
	h = rover_heading_update(&f, 1.0f, pi - 0.01f + 0.01f, true, 0.01f);
	/* crossed the seam: result must be near -pi, not 0 */
	zassert_true(fabsf(h) > 3.0f, "h=%f", (double)h);
}

ZTEST(heading, test_wrap_pi_range)
{
	ZNEAR(rover_wrap_pi(4.0f), 4.0f - 6.2831853f, 1e-5);
	ZNEAR(rover_wrap_pi(-4.0f), -4.0f + 6.2831853f, 1e-5);
	ZNEAR(rover_wrap_pi(0.3f), 0.3f, 1e-6);
}

ZTEST_SUITE(heading, NULL, NULL, NULL, NULL, NULL);

/* ---- link monitor ---- */

ZTEST(link, test_not_alive_before_first_frame)
{
	struct rover_link_monitor m;

	rover_link_init(&m, 100);
	zassert_false(rover_link_alive(&m, 0));
}

ZTEST(link, test_alive_then_times_out)
{
	struct rover_link_monitor m;

	rover_link_init(&m, 100);
	rover_link_feed(&m, 1000);
	zassert_true(rover_link_alive(&m, 1050));
	zassert_true(rover_link_alive(&m, 1100));
	zassert_false(rover_link_alive(&m, 1101));
	rover_link_feed(&m, 1200);
	zassert_true(rover_link_alive(&m, 1250));
}

ZTEST(link, test_millisecond_counter_wrap)
{
	struct rover_link_monitor m;

	rover_link_init(&m, 100);
	rover_link_feed(&m, UINT32_MAX - 20);
	zassert_true(rover_link_alive(&m, 30));   /* 51 ms later, counter wrapped */
	zassert_false(rover_link_alive(&m, 200));
}

ZTEST_SUITE(link, NULL, NULL, NULL, NULL, NULL);

/* ---- encoder math ---- */

ZTEST(encoder, test_delta_handles_wrap)
{
	zassert_equal(rover_enc_delta(10, 25), 15);
	zassert_equal(rover_enc_delta(25, 10), -15);
	zassert_equal(rover_enc_delta(0xFFFFFFF0u, 0x10u), 32);
	zassert_equal(rover_enc_delta(0x10u, 0xFFFFFFF0u), -32);
}

ZTEST(encoder, test_velocity)
{
	/* one revolution of 1000 ticks in 1 s = 2*pi rad/s */
	ZNEAR(rover_enc_velocity(1000, 1000.0f, 1.0f), 6.2831853f, 1e-4);
	ZNEAR(rover_enc_velocity(-500, 1000.0f, 0.5f), -6.2831853f, 1e-4);
	zassert_equal(rover_enc_velocity(10, 1000.0f, 0.0f), 0.0f);
}

ZTEST(encoder, test_lowpass_converges)
{
	float y = 0.0f;

	for (int i = 0; i < 100; i++) {
		y = rover_lowpass(y, 1.0f, 0.2f);
	}
	ZNEAR(y, 1.0f, 1e-3);
	ZNEAR(rover_lowpass(0.0f, 1.0f, 1.0f), 1.0f, 1e-6);
}

ZTEST_SUITE(encoder, NULL, NULL, NULL, NULL, NULL);

/* ---- quadrature decoder ---- */

ZTEST(quad, test_forward_full_cycle_is_four_counts)
{
	struct rover_quad_decoder q;

	rover_quad_init(&q, 0, 0);
	rover_quad_update(&q, 0, 1);
	rover_quad_update(&q, 1, 1);
	rover_quad_update(&q, 1, 0);
	rover_quad_update(&q, 0, 0);
	zassert_equal(q.count, 4);
	zassert_equal(q.errors, 0);
}

ZTEST(quad, test_reverse_counts_down)
{
	struct rover_quad_decoder q;

	rover_quad_init(&q, 0, 0);
	zassert_equal(rover_quad_update(&q, 1, 0), -1);
	zassert_equal(rover_quad_update(&q, 1, 1), -1);
	zassert_equal(rover_quad_update(&q, 0, 1), -1);
	zassert_equal(rover_quad_update(&q, 0, 0), -1);
	zassert_equal(q.count, -4);
}

ZTEST(quad, test_illegal_jump_is_flagged_not_counted)
{
	struct rover_quad_decoder q;

	rover_quad_init(&q, 0, 0);
	zassert_equal(rover_quad_update(&q, 1, 1), 0);
	zassert_equal(q.count, 0);
	zassert_equal(q.errors, 1);
}

ZTEST(quad, test_bounce_on_one_edge_nets_zero)
{
	struct rover_quad_decoder q;

	rover_quad_init(&q, 0, 0);
	rover_quad_update(&q, 0, 1);
	rover_quad_update(&q, 0, 0);
	rover_quad_update(&q, 0, 1);
	rover_quad_update(&q, 0, 0);
	zassert_equal(q.count, 0);
}

ZTEST(quad, test_unchanged_pins_do_nothing)
{
	struct rover_quad_decoder q;

	rover_quad_init(&q, 1, 1);
	zassert_equal(rover_quad_update(&q, 1, 1), 0);
	zassert_equal(q.errors, 0);
}

ZTEST_SUITE(quad, NULL, NULL, NULL, NULL, NULL);

/* ---- jitter statistics ---- */

#include <rover/jitter.h>

ZTEST(jitter, test_stats)
{
	struct rover_jitter j;

	rover_jitter_reset(&j);
	rover_jitter_record(&j, 5000);
	rover_jitter_record(&j, -20000);
	rover_jitter_record(&j, 1500000);
	zassert_equal(j.count, 3);
	zassert_equal(j.min_ns, -20000);
	zassert_equal(j.max_ns, 1500000);
	zassert_equal(j.max_abs_ns, 1500000u);
	zassert_equal(rover_jitter_mean_ns(&j), (5000 - 20000 + 1500000) / 3);
	zassert_equal(j.hist[0], 1); /* 5 us */
	zassert_equal(j.hist[1], 1); /* 20 us */
	zassert_equal(j.hist[6], 1); /* 1.5 ms */
}

ZTEST(jitter, test_empty_mean_is_zero)
{
	struct rover_jitter j;

	rover_jitter_reset(&j);
	zassert_equal(rover_jitter_mean_ns(&j), 0);
}

ZTEST_SUITE(jitter, NULL, NULL, NULL, NULL, NULL);

ZTEST(encoder, test_window_velocity_resolution)
{
	struct rover_vel_est e;
	float v = 0.0f;

	/* 1000 cpr wheel turning at 1 count per 4 ms = 0.25 count/ms */
	rover_vel_est_init(&e, 20);
	for (int i = 0; i < 200; i++) {
		v = rover_vel_est_update(&e, i / 4, 1000.0f, 0.001f);
	}
	/* 250 counts/s * 2pi/1000 = 1.5708 rad/s, +-1 count over 19 ms */
	ZNEAR(v, 1.5708f, 0.35f);
}

ZTEST(encoder, test_window_first_sample_is_zero_then_converges)
{
	struct rover_vel_est e;

	rover_vel_est_init(&e, 4);
	zassert_equal(rover_vel_est_update(&e, 100, 1000.0f, 0.001f), 0.0f);
	/* 10 counts per ms */
	rover_vel_est_update(&e, 110, 1000.0f, 0.001f);
	rover_vel_est_update(&e, 120, 1000.0f, 0.001f);
	rover_vel_est_update(&e, 130, 1000.0f, 0.001f);
	ZNEAR(rover_vel_est_update(&e, 140, 1000.0f, 0.001f), 62.83f, 0.1f);
}

ZTEST(encoder, test_window_handles_negative_and_wrap)
{
	struct rover_vel_est e;
	float v = 0.0f;

	rover_vel_est_init(&e, 5);
	for (int i = 0; i < 10; i++) {
		v = rover_vel_est_update(&e, (int32_t)(0x7FFFFFFE - 0) - i * 10, 1000.0f, 0.001f);
	}
	ZNEAR(v, -62.83f, 0.1f);
}
