#ifndef ROVER_ENCODER_H_
#define ROVER_ENCODER_H_

#include <stdint.h>
#include <zephyr/device.h>

struct rover_encoder_api {
	int32_t (*get_count)(const struct device *dev);
	uint32_t (*get_errors)(const struct device *dev);
	void (*reset)(const struct device *dev);
	uint32_t (*counts_per_rev)(const struct device *dev);
};

/* Signed 4x edge count since boot or the last reset. */
static inline int32_t rover_encoder_get_count(const struct device *dev)
{
	return ((const struct rover_encoder_api *)dev->api)->get_count(dev);
}

static inline uint32_t rover_encoder_get_errors(const struct device *dev)
{
	return ((const struct rover_encoder_api *)dev->api)->get_errors(dev);
}

static inline void rover_encoder_reset(const struct device *dev)
{
	((const struct rover_encoder_api *)dev->api)->reset(dev);
}

static inline uint32_t rover_encoder_counts_per_rev(const struct device *dev)
{
	return ((const struct rover_encoder_api *)dev->api)->counts_per_rev(dev);
}

#endif /* ROVER_ENCODER_H_ */
