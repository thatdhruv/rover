#ifndef ROVER_LINK_MONITOR_H_
#define ROVER_LINK_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

/* Detects a dropped host link: no valid frame within `timeout_ms`. */
struct rover_link_monitor {
	uint32_t timeout_ms;
	uint32_t last_ms;
	bool seen;
};

void rover_link_init(struct rover_link_monitor *m, uint32_t timeout_ms);
void rover_link_feed(struct rover_link_monitor *m, uint32_t now_ms);
/* True while the link is alive. False before the first frame and after timeout. */
bool rover_link_alive(const struct rover_link_monitor *m, uint32_t now_ms);

#endif /* ROVER_LINK_MONITOR_H_ */
