#ifndef ROVER_MOTOR_H_
#define ROVER_MOTOR_H_

#include <zephyr/device.h>

struct rover_motor_api {
	int (*set_duty)(const struct device *dev, float duty);
	void (*cutoff)(const struct device *dev);
	void (*rearm)(const struct device *dev);
};

/* duty in [-1, 1]; sign selects direction. Returns -EPERM while cut off. */
static inline int rover_motor_set_duty(const struct device *dev, float duty)
{
	return ((const struct rover_motor_api *)dev->api)->set_duty(dev, duty);
}

/* Immediately coast the motor and latch it off until rearmed. ISR-safe. */
static inline void rover_motor_cutoff(const struct device *dev)
{
	((const struct rover_motor_api *)dev->api)->cutoff(dev);
}

static inline void rover_motor_rearm(const struct device *dev)
{
	((const struct rover_motor_api *)dev->api)->rearm(dev);
}

#endif /* ROVER_MOTOR_H_ */
