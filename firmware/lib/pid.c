#include <rover/pid.h>

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

void rover_pid_init(struct rover_pid *pid, float kp, float ki, float kd, float out_min,
		    float out_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->out_min = out_min;
	pid->out_max = out_max;
	rover_pid_reset(pid);
}

void rover_pid_reset(struct rover_pid *pid)
{
	pid->integral = 0.0f;
	pid->prev_meas = 0.0f;
	pid->primed = false;
}

float rover_pid_update(struct rover_pid *pid, float setpoint, float measurement, float dt)
{
	const float err = setpoint - measurement;
	float deriv = 0.0f;

	if (dt <= 0.0f) {
		return clampf(pid->kp * err + pid->integral, pid->out_min, pid->out_max);
	}
	if (pid->primed) {
		/* derivative on measurement avoids a kick on setpoint steps */
		deriv = -(measurement - pid->prev_meas) / dt;
	}
	pid->prev_meas = measurement;
	pid->primed = true;

	const float unsat = pid->kp * err + pid->integral + pid->ki * err * dt + pid->kd * deriv;
	const float out = clampf(unsat, pid->out_min, pid->out_max);

	/* conditional integration: freeze the integrator while saturated and
	 * the error would push it further into saturation
	 */
	if (unsat == out || (unsat > pid->out_max && err < 0.0f) ||
	    (unsat < pid->out_min && err > 0.0f)) {
		pid->integral += pid->ki * err * dt;
	}
	return out;
}
