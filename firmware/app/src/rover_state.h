#ifndef ROVER_STATE_H_
#define ROVER_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include <rover/jitter.h>

#define NUM_WHEELS 2
#define WHEEL_LEFT  0
#define WHEEL_RIGHT 1

/* Snapshot of everything the host link reports. */
struct rover_state {
	float wheel_vel[NUM_WHEELS]; /* measured, rad/s */
	float wheel_cmd[NUM_WHEELS]; /* setpoint, rad/s */
	float wheel_duty[NUM_WHEELS];
	int32_t wheel_count[NUM_WHEELS];
	float heading;  /* rad, complementary-filtered */
	float gyro_z;   /* rad/s */
	float accel[3]; /* m/s^2 */
	bool link_alive;
	bool motors_cut_off;
	uint32_t uptime_ms;
};

/* ---- command side (called from the host-link thread or shell) ---- */

/* Set wheel velocity setpoints (rad/s). Also counts as a link heartbeat. */
void rover_cmd_set_wheel_velocity(float left, float right);
/* Heartbeat without a new setpoint. */
void rover_cmd_heartbeat(void);

/* ---- state side ---- */

void rover_state_get(struct rover_state *out);
void rover_jitter_get(struct rover_jitter *out);

/* ---- subsystem entry points ---- */

int control_start(void);
int imu_task_start(void);
int safety_start(void);

/* Used by the tasks to report liveness to the watchdog supervisor. */
enum task_id {
	TASK_CONTROL = 0,
	TASK_IMU = 1,
	TASK_COUNT
};
void safety_task_alive(enum task_id id);

/* Record a fault in the flash log (no-op if the log is unavailable). */
void safety_log_fault(uint16_t code, uint32_t arg0, uint32_t arg1);

/* Used by the imu task to publish its latest measurements. */
void state_publish_imu(float heading, float gyro_z, const float accel[3]);
/* Latest wheel-odometry heading for the complementary filter reference (rad). */
float state_odom_heading(void);

#endif /* ROVER_STATE_H_ */
