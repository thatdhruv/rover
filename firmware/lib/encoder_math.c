#include <rover/encoder_math.h>

#define TWO_PI_F 6.28318530717959f

float rover_enc_velocity(int32_t delta_ticks, float ticks_per_rev, float dt)
{
	if (dt <= 0.0f || ticks_per_rev <= 0.0f) {
		return 0.0f;
	}
	return (float)delta_ticks * TWO_PI_F / (ticks_per_rev * dt);
}

float rover_lowpass(float prev, float sample, float alpha)
{
	return prev + alpha * (sample - prev);
}

void rover_vel_est_init(struct rover_vel_est *e, uint8_t window)
{
	e->window = window < 2 ? 2 : (window > ROVER_VEL_WINDOW_MAX ? ROVER_VEL_WINDOW_MAX : window);
	e->idx = 0;
	e->filled = 0;
}

float rover_vel_est_update(struct rover_vel_est *e, int32_t count, float ticks_per_rev,
			   float sample_dt)
{
	int32_t oldest;
	uint8_t span;

	e->hist[e->idx] = count;
	if (e->filled < e->window) {
		e->filled++;
	}
	/* oldest sample still in the window */
	oldest = e->hist[(e->idx + e->window + 1 - e->filled) % e->window];
	span = e->filled - 1;
	e->idx = (e->idx + 1) % e->window;

	if (span == 0) {
		return 0.0f;
	}
	return rover_enc_velocity(rover_enc_delta((uint32_t)oldest, (uint32_t)count), ticks_per_rev,
				  sample_dt * (float)span);
}
