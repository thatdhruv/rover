#include <rover/link_monitor.h>

void rover_link_init(struct rover_link_monitor *m, uint32_t timeout_ms)
{
	m->timeout_ms = timeout_ms;
	m->last_ms = 0;
	m->seen = false;
}

void rover_link_feed(struct rover_link_monitor *m, uint32_t now_ms)
{
	m->last_ms = now_ms;
	m->seen = true;
}

bool rover_link_alive(const struct rover_link_monitor *m, uint32_t now_ms)
{
	/* unsigned subtraction keeps this correct across millisecond wrap */
	return m->seen && (uint32_t)(now_ms - m->last_ms) <= m->timeout_ms;
}
