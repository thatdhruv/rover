#ifndef ROVER_FAULTLOG_H_
#define ROVER_FAULTLOG_H_

#include <stdint.h>

/*
 * Persistent fault log in a flash partition.
 *
 * Fixed 32-byte records, CRC protected, written append-only in a ring over the
 * erase sectors of the partition. Entering a new sector erases it first, so at
 * least (sectors - 1) sectors of history survive a wrap. A record torn by
 * power loss fails its CRC and is skipped.
 */

enum rover_fault_code {
	ROVER_FAULT_BOOT = 1,         /* arg0 = hwinfo reset cause flags */
	ROVER_FAULT_WATCHDOG = 2,     /* arg0 = name of the task that stopped feeding */
	ROVER_FAULT_LINK_LOST = 3,    /* arg0 = ms since last valid frame */
	ROVER_FAULT_IMU = 4,          /* arg0 = errno */
	ROVER_FAULT_ENCODER = 5,      /* arg0 = wheel index, arg1 = illegal transitions */
	ROVER_FAULT_MOTOR_CUTOFF = 6, /* arg0 = reason code */
	ROVER_FAULT_CPU_EXCEPTION = 7 /* arg0 = zephyr fatal reason, arg1 = pc */
};

struct rover_fault_rec {
	uint32_t seq;
	uint32_t uptime_ms;
	uint16_t code;
	uint32_t arg0;
	uint32_t arg1;
};

struct rover_faultlog {
	const struct flash_area *fa;
	uint32_t sector_size;
	uint32_t slots;
	uint32_t slots_per_sector;
	uint32_t next_slot;
	uint32_t next_seq;
};

int rover_faultlog_init(struct rover_faultlog *l, uint8_t flash_area_id);
int rover_faultlog_append(struct rover_faultlog *l, uint16_t code, uint32_t arg0, uint32_t arg1,
			  uint32_t uptime_ms);
/* Number of valid records currently stored. */
uint32_t rover_faultlog_count(struct rover_faultlog *l);
/* age 0 = newest. Returns -ENOENT if there is no such record. */
int rover_faultlog_read(struct rover_faultlog *l, uint32_t age, struct rover_fault_rec *out);
int rover_faultlog_clear(struct rover_faultlog *l);

#endif /* ROVER_FAULTLOG_H_ */
