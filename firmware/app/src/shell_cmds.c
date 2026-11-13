/* Bench-test shell: drive wheels and inspect state without the host link. */
#include <stdlib.h>

#include <zephyr/shell/shell.h>

#include "rover_state.h"

static int cmd_vel(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "usage: rover vel <left rad/s> <right rad/s>");
		return -EINVAL;
	}
	rover_cmd_set_wheel_velocity(strtof(argv[1], NULL), strtof(argv[2], NULL));
	return 0;
}

static int cmd_hb(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	rover_cmd_heartbeat();
	return 0;
}

static int cmd_state(const struct shell *sh, size_t argc, char **argv)
{
	struct rover_state s;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	rover_state_get(&s);
	shell_print(sh, "link=%d cutoff=%d up=%ums", s.link_alive, s.motors_cut_off, s.uptime_ms);
	for (int i = 0; i < NUM_WHEELS; i++) {
		shell_print(sh, "wheel%d cmd=%.2f meas=%.2f duty=%.2f count=%d", i,
			    (double)s.wheel_cmd[i], (double)s.wheel_vel[i], (double)s.wheel_duty[i],
			    s.wheel_count[i]);
	}
	shell_print(sh, "heading=%.3f rad gyro_z=%.3f rad/s", (double)s.heading, (double)s.gyro_z);
	return 0;
}

static int cmd_jitter(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const labels[] = {"<10us",  "<50us", "<100us", "<250us",
					     "<500us", "<1ms",  ">=1ms"};
	struct rover_jitter j;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	rover_jitter_get(&j);
	shell_print(sh, "samples=%u mean=%dns min=%dns max=%dns max|e|=%uns", j.count,
		    rover_jitter_mean_ns(&j), j.min_ns, j.max_ns, j.max_abs_ns);
	for (int i = 0; i < ROVER_JITTER_BUCKETS; i++) {
		shell_print(sh, "  %-7s %u", labels[i], j.hist[i]);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(rover_sub,
	SHELL_CMD(vel, NULL, "vel <left> <right>: wheel setpoints in rad/s", cmd_vel),
	SHELL_CMD(hb, NULL, "send a link heartbeat", cmd_hb),
	SHELL_CMD(state, NULL, "print wheel/IMU state", cmd_state),
	SHELL_CMD(jitter, NULL, "1 kHz loop timing statistics", cmd_jitter),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(rover, &rover_sub, "rover bench commands", NULL);
