/*
 * Safety supervisor: hardware watchdog, fault log, and boot-cause reporting.
 *
 * Every critical task calls safety_task_alive() each cycle. The supervisor
 * feeds the watchdog only while all of them are checking in; if one stops, it
 * cuts the motors, records which task stalled, and lets the watchdog reset the
 * chip.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>
#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/dfu/mcuboot.h>
#endif

#include <rover/faultlog.h>
#include <rover/motor.h>

#include "rover_state.h"

LOG_MODULE_REGISTER(safety, LOG_LEVEL_INF);

#define WDT_TIMEOUT_MS 200
#define SUPERVISOR_MS  20
#define STALL_LIMIT_MS 100
#define FAULT_Q_DEPTH  8
#define FAULT_LOG_AREA PARTITION_ID(storage_partition)

struct fault_msg {
	uint16_t code;
	uint32_t arg0;
	uint32_t arg1;
	uint32_t uptime_ms;
};

static const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static const struct device *const motors[] = {
	DEVICE_DT_GET(DT_ALIAS(rover_motor_left)),
	DEVICE_DT_GET(DT_ALIAS(rover_motor_right)),
};

K_MSGQ_DEFINE(fault_q, sizeof(struct fault_msg), FAULT_Q_DEPTH, 4);
static atomic_t alive_bits;
static int64_t last_seen_ms[TASK_COUNT];
static struct rover_faultlog flog;
static bool flog_ok;
static int wdt_channel = -1;

void safety_task_alive(enum task_id id)
{
	atomic_or(&alive_bits, BIT(id));
}

/* Queued so a slow flash write never blocks the 1 kHz loop. */
void safety_log_fault(uint16_t code, uint32_t arg0, uint32_t arg1)
{
	const struct fault_msg m = {code, arg0, arg1, k_uptime_get_32()};

	if (k_msgq_put(&fault_q, &m, K_NO_WAIT) != 0) {
		LOG_WRN("fault queue full, dropped code %u", code);
	}
}

static void drain_faults(void)
{
	struct fault_msg m;

	while (k_msgq_get(&fault_q, &m, K_NO_WAIT) == 0) {
		if (flog_ok && rover_faultlog_append(&flog, m.code, m.arg0, m.arg1, m.uptime_ms) < 0) {
			LOG_ERR("fault log write failed");
		}
	}
}

static void cut_all_motors(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(motors); i++) {
		rover_motor_cutoff(motors[i]);
	}
}

static int watchdog_setup(void)
{
	const struct wdt_timeout_cfg cfg = {
		.window = {.min = 0, .max = WDT_TIMEOUT_MS},
		.flags = WDT_FLAG_RESET_SOC,
	};

	if (!device_is_ready(wdt)) {
		LOG_ERR("watchdog not ready");
		return -ENODEV;
	}
	wdt_channel = wdt_install_timeout(wdt, &cfg);
	if (wdt_channel < 0) {
		return wdt_channel;
	}
	return wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
}

static void report_boot_cause(void)
{
	uint32_t cause = 0;

	hwinfo_get_reset_cause(&cause);
	hwinfo_clear_reset_cause();
	safety_log_fault(ROVER_FAULT_BOOT, cause, 0);
	if (cause & RESET_WATCHDOG) {
		safety_log_fault(ROVER_FAULT_WATCHDOG, 0, 0);
		LOG_WRN("previous run ended in a watchdog reset");
	}
}

static void supervisor_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	bool stalled_logged = false;
	int64_t healthy_since = k_uptime_get();
	bool confirmed = false;
	ARG_UNUSED(confirmed);

	for (int i = 0; i < TASK_COUNT; i++) {
		last_seen_ms[i] = k_uptime_get();
	}

	for (;;) {
		k_msleep(SUPERVISOR_MS);
		drain_faults();

		const atomic_val_t seen = atomic_clear(&alive_bits);
		const int64_t now = k_uptime_get();
		bool healthy = true;

		for (int i = 0; i < TASK_COUNT; i++) {
			if (seen & BIT(i)) {
				last_seen_ms[i] = now;
			} else if (now - last_seen_ms[i] > STALL_LIMIT_MS) {
				healthy = false;
				if (!stalled_logged) {
					stalled_logged = true;
					cut_all_motors();
					if (flog_ok) {
						rover_faultlog_append(&flog, ROVER_FAULT_WATCHDOG, i,
								      0, (uint32_t)now);
					}
					LOG_ERR("task %d stalled; letting watchdog reset", i);
				}
			}
		}

		if (healthy) {
			wdt_feed(wdt, wdt_channel);
		} else {
			healthy_since = now;
		}

#ifdef CONFIG_BOOTLOADER_MCUBOOT
		/* Confirm the running image only after a stretch of proven health;
		 * until then a reset makes MCUboot roll back to the previous image.
		 */
		if (!confirmed && now - healthy_since >= CONFIG_ROVER_CONFIRM_AFTER_MS) {
			confirmed = true;
			if (!boot_is_img_confirmed() && boot_write_img_confirmed() == 0) {
				LOG_INF("image confirmed");
			}
		}
#endif
		/* unhealthy: stop feeding, the IWDG resets the chip within WDT_TIMEOUT_MS */
	}
}

K_THREAD_STACK_DEFINE(supervisor_stack, 2048);
static struct k_thread supervisor_thread;

int safety_start(void)
{
	int ret = rover_faultlog_init(&flog, FAULT_LOG_AREA);

	flog_ok = (ret == 0);
	if (!flog_ok) {
		LOG_ERR("fault log init failed (%d); continuing without it", ret);
	}
	report_boot_cause();
	drain_faults();

	ret = watchdog_setup();
	if (ret < 0) {
		LOG_ERR("watchdog setup failed (%d)", ret);
		return ret;
	}

	k_thread_create(&supervisor_thread, supervisor_stack,
			K_THREAD_STACK_SIZEOF(supervisor_stack), supervisor_fn, NULL, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	k_thread_name_set(&supervisor_thread, "supervisor");
	return 0;
}

/* ---- shell access to the fault log ---- */

static int cmd_faults(const struct shell *sh, size_t argc, char **argv)
{
	struct rover_fault_rec r;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!flog_ok) {
		shell_error(sh, "fault log unavailable");
		return -ENODEV;
	}
	for (uint32_t age = 0; age < 16 && rover_faultlog_read(&flog, age, &r) == 0; age++) {
		shell_print(sh, "#%u t=%ums code=%u arg0=%u arg1=%u", r.seq, r.uptime_ms, r.code,
			    r.arg0, r.arg1);
	}
	return 0;
}

static int cmd_faults_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!flog_ok) {
		return -ENODEV;
	}
	shell_print(sh, "fault log cleared");
	return rover_faultlog_clear(&flog);
}

SHELL_STATIC_SUBCMD_SET_CREATE(faults_sub,
	SHELL_CMD(clear, NULL, "erase the fault log", cmd_faults_clear),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(faults, &faults_sub, "show newest fault log records", cmd_faults);
