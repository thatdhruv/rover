#include <zephyr/storage/flash_map.h>
#include <zephyr/ztest.h>

#include <rover/faultlog.h>

#define AREA_ID PARTITION_ID(storage_partition)

static struct rover_faultlog log;

static void before(void *unused)
{
	ARG_UNUSED(unused);
	const struct flash_area *fa;

	zassert_ok(flash_area_open(AREA_ID, &fa));
	zassert_ok(flash_area_erase(fa, 0, fa->fa_size));
	flash_area_close(fa);
	zassert_ok(rover_faultlog_init(&log, AREA_ID));
}

ZTEST(faultlog, test_empty_log)
{
	struct rover_fault_rec r;

	zassert_equal(rover_faultlog_count(&log), 0);
	zassert_equal(rover_faultlog_read(&log, 0, &r), -ENOENT);
}

ZTEST(faultlog, test_append_and_read_newest_first)
{
	struct rover_fault_rec r;

	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_BOOT, 1, 0, 10));
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_LINK_LOST, 250, 0, 20));
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_IMU, 5, 6, 30));

	zassert_equal(rover_faultlog_count(&log), 3);
	zassert_ok(rover_faultlog_read(&log, 0, &r));
	zassert_equal(r.code, ROVER_FAULT_IMU);
	zassert_equal(r.arg0, 5);
	zassert_equal(r.arg1, 6);
	zassert_equal(r.uptime_ms, 30);
	zassert_ok(rover_faultlog_read(&log, 2, &r));
	zassert_equal(r.code, ROVER_FAULT_BOOT);
	zassert_equal(rover_faultlog_read(&log, 3, &r), -ENOENT);
}

ZTEST(faultlog, test_survives_reinit)
{
	struct rover_fault_rec r;

	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_WATCHDOG, 7, 0, 100));
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_MOTOR_CUTOFF, 2, 0, 200));

	/* simulate a reboot */
	zassert_ok(rover_faultlog_init(&log, AREA_ID));
	zassert_equal(rover_faultlog_count(&log), 2);
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_BOOT, 0, 0, 5));
	zassert_ok(rover_faultlog_read(&log, 0, &r));
	zassert_equal(r.code, ROVER_FAULT_BOOT);
	zassert_equal(r.seq, 3);
	zassert_ok(rover_faultlog_read(&log, 2, &r));
	zassert_equal(r.code, ROVER_FAULT_WATCHDOG);
}

ZTEST(faultlog, test_wraps_and_keeps_recent_history)
{
	struct rover_fault_rec r;
	/* storage_partition is 4 sectors of 4 KiB = 512 slots */
	const uint32_t total = 1300;

	for (uint32_t i = 0; i < total; i++) {
		zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_ENCODER, i, 0, i));
	}
	zassert_true(rover_faultlog_count(&log) >= 3 * 128, "count %u",
		     rover_faultlog_count(&log));
	zassert_ok(rover_faultlog_read(&log, 0, &r));
	zassert_equal(r.arg0, total - 1);
	zassert_ok(rover_faultlog_read(&log, 100, &r));
	zassert_equal(r.arg0, total - 101);

	/* reinit finds the right head after a wrap */
	zassert_ok(rover_faultlog_init(&log, AREA_ID));
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_BOOT, 0, 0, 0));
	zassert_ok(rover_faultlog_read(&log, 0, &r));
	zassert_equal(r.seq, total + 1);
}

ZTEST(faultlog, test_corrupt_record_is_skipped)
{
	const struct flash_area *fa;
	struct rover_fault_rec r;
	const uint8_t junk = 0x00;

	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_BOOT, 1, 0, 1));
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_IMU, 2, 0, 2));

	/* flip a payload byte of record 0 from 0xFF-able state: clear bits only */
	zassert_ok(flash_area_open(AREA_ID, &fa));
	zassert_ok(flash_area_write(fa, 8, &junk, 1)); /* uptime byte -> CRC mismatch */
	flash_area_close(fa);

	zassert_equal(rover_faultlog_count(&log), 1);
	zassert_ok(rover_faultlog_read(&log, 0, &r));
	zassert_equal(r.code, ROVER_FAULT_IMU);
}

ZTEST(faultlog, test_clear)
{
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_BOOT, 1, 0, 1));
	zassert_ok(rover_faultlog_clear(&log));
	zassert_equal(rover_faultlog_count(&log), 0);
	zassert_ok(rover_faultlog_append(&log, ROVER_FAULT_BOOT, 1, 0, 1));
	zassert_equal(rover_faultlog_count(&log), 1);
}

ZTEST_SUITE(faultlog, NULL, NULL, before, NULL, NULL);
