#include <rover/quad_decoder.h>

/* index = (prev << 2) | next; Gray sequence 00 -> 01 -> 11 -> 10 counts up */
static const int8_t step_table[16] = {
	0, +1, -1, 0,
	-1, 0, 0, +1,
	+1, 0, 0, -1,
	0, -1, +1, 0,
};

void rover_quad_init(struct rover_quad_decoder *q, int a, int b)
{
	q->prev = (uint8_t)(((a ? 1 : 0) << 1) | (b ? 1 : 0));
	q->count = 0;
	q->errors = 0;
}

int rover_quad_update(struct rover_quad_decoder *q, int a, int b)
{
	const uint8_t next = (uint8_t)(((a ? 1 : 0) << 1) | (b ? 1 : 0));
	const uint8_t idx = (uint8_t)((q->prev << 2) | next);
	const int step = step_table[idx];

	if (step == 0 && next != q->prev) {
		q->errors++;
	}
	q->count += step;
	q->prev = next;
	return step;
}
