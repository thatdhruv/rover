#include <rover/jitter.h>

static const uint32_t bucket_limit_ns[ROVER_JITTER_BUCKETS - 1] = {
	10000, 50000, 100000, 250000, 500000, 1000000,
};

void rover_jitter_reset(struct rover_jitter *j)
{
	*j = (struct rover_jitter){.min_ns = INT32_MAX, .max_ns = INT32_MIN};
}

void rover_jitter_record(struct rover_jitter *j, int32_t error_ns)
{
	const uint32_t mag = error_ns < 0 ? (uint32_t)(-(int64_t)error_ns) : (uint32_t)error_ns;
	int b = 0;

	if (error_ns < j->min_ns) {
		j->min_ns = error_ns;
	}
	if (error_ns > j->max_ns) {
		j->max_ns = error_ns;
	}
	if (mag > j->max_abs_ns) {
		j->max_abs_ns = mag;
	}
	j->sum_ns += error_ns;
	j->count++;

	while (b < ROVER_JITTER_BUCKETS - 1 && mag >= bucket_limit_ns[b]) {
		b++;
	}
	j->hist[b]++;
}

int32_t rover_jitter_mean_ns(const struct rover_jitter *j)
{
	return j->count ? (int32_t)(j->sum_ns / (int64_t)j->count) : 0;
}
