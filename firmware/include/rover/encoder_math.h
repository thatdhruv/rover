#ifndef ROVER_ENCODER_MATH_H_
#define ROVER_ENCODER_MATH_H_

#include <stdint.h>

/* Signed tick delta between two free-running 32-bit counters (wrap safe). */
static inline int32_t rover_enc_delta(uint32_t prev, uint32_t cur)
{
	return (int32_t)(cur - prev);
}

/* Wheel angular velocity in rad/s from a tick delta over dt seconds. */
float rover_enc_velocity(int32_t delta_ticks, float ticks_per_rev, float dt);

/* First-order low-pass: y += alpha * (x - y), alpha in (0, 1]. */
float rover_lowpass(float prev, float sample, float alpha);

#endif /* ROVER_ENCODER_MATH_H_ */

#define ROVER_VEL_WINDOW_MAX 32

/*
 * Sliding-window velocity estimator. A single 1 ms tick only sees a handful of
 * counts, so velocity is taken as the count change across the last `window`
 * samples, which gives window-times finer resolution at the cost of a
 * (window/2) sample delay.
 */
struct rover_vel_est {
	int32_t hist[ROVER_VEL_WINDOW_MAX];
	uint8_t window;
	uint8_t idx;
	uint8_t filled;
};

void rover_vel_est_init(struct rover_vel_est *e, uint8_t window);
/* Push the latest absolute count; returns rad/s. */
float rover_vel_est_update(struct rover_vel_est *e, int32_t count, float ticks_per_rev,
			   float sample_dt);
