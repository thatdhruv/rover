#ifndef ROVER_PID_H_
#define ROVER_PID_H_

#include <stdbool.h>

/* PID with derivative-on-measurement and conditional-integration anti-windup. */
struct rover_pid {
	float kp, ki, kd;
	float out_min, out_max;
	float integral;
	float prev_meas;
	bool primed;
};

void rover_pid_init(struct rover_pid *pid, float kp, float ki, float kd, float out_min,
		    float out_max);
void rover_pid_reset(struct rover_pid *pid);
float rover_pid_update(struct rover_pid *pid, float setpoint, float measurement, float dt);

#endif /* ROVER_PID_H_ */
