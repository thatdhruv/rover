#ifndef ROVER_HEADING_FILTER_H_
#define ROVER_HEADING_FILTER_H_

#include <stdbool.h>

/*
 * Complementary filter for heading (yaw, radians, wrapped to [-pi, pi]).
 * The gyro z-rate is integrated for short-term accuracy; a slow absolute
 * reference (wheel-odometry heading, magnetometer) corrects long-term drift.
 * `tau` is the crossover time constant in seconds.
 */
struct rover_heading_filter {
	float tau;
	float heading;
	bool primed;
};

void rover_heading_init(struct rover_heading_filter *f, float tau);
float rover_heading_update(struct rover_heading_filter *f, float gyro_z, float ref_heading,
			   bool ref_valid, float dt);
float rover_wrap_pi(float angle);

#endif /* ROVER_HEADING_FILTER_H_ */
