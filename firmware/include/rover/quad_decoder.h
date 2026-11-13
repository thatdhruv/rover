#ifndef ROVER_QUAD_DECODER_H_
#define ROVER_QUAD_DECODER_H_

#include <stdint.h>

/* 4x quadrature decoder: one count per edge on either channel. */
struct rover_quad_decoder {
	uint8_t prev; /* (a << 1) | b */
	int32_t count;
	uint32_t errors; /* illegal transitions (two bits changed at once) */
};

void rover_quad_init(struct rover_quad_decoder *q, int a, int b);
/* Feed the new pin levels; returns the step taken (-1, 0 or +1). */
int rover_quad_update(struct rover_quad_decoder *q, int a, int b);

#endif /* ROVER_QUAD_DECODER_H_ */
