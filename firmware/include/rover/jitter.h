#ifndef ROVER_JITTER_H_
#define ROVER_JITTER_H_

#include <stdint.h>

#define ROVER_JITTER_BUCKETS 7

/* Running statistics of loop timing error (actual wake - ideal wake), in ns. */
struct rover_jitter {
	int32_t min_ns;
	int32_t max_ns;
	int64_t sum_ns;
	uint32_t count;
	uint32_t max_abs_ns;
	/* |error| histogram: <10us <50us <100us <250us <500us <1ms >=1ms */
	uint32_t hist[ROVER_JITTER_BUCKETS];
};

void rover_jitter_reset(struct rover_jitter *j);
void rover_jitter_record(struct rover_jitter *j, int32_t error_ns);
int32_t rover_jitter_mean_ns(const struct rover_jitter *j);

#endif /* ROVER_JITTER_H_ */
