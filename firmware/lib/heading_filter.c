#include <rover/heading_filter.h>

#define PI_F  3.14159265358979f
#define TWO_PI_F (2.0f * PI_F)

float rover_wrap_pi(float angle)
{
	while (angle > PI_F) {
		angle -= TWO_PI_F;
	}
	while (angle < -PI_F) {
		angle += TWO_PI_F;
	}
	return angle;
}

void rover_heading_init(struct rover_heading_filter *f, float tau)
{
	f->tau = tau;
	f->heading = 0.0f;
	f->primed = false;
}

float rover_heading_update(struct rover_heading_filter *f, float gyro_z, float ref_heading,
			   bool ref_valid, float dt)
{
	if (!f->primed && ref_valid) {
		f->heading = rover_wrap_pi(ref_heading);
		f->primed = true;
		return f->heading;
	}

	f->heading = rover_wrap_pi(f->heading + gyro_z * dt);

	if (ref_valid && f->tau > 0.0f) {
		/* correct along the shortest arc so the +-pi seam does not glitch */
		const float gain = dt / (f->tau + dt);

		f->heading = rover_wrap_pi(f->heading +
					   gain * rover_wrap_pi(ref_heading - f->heading));
	}
	return f->heading;
}
