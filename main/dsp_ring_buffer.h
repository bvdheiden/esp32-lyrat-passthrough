#ifndef _DSP_RING_BUFFER_H_
#define _DSP_RING_BUFFER_H_

#include <stdint.h>

typedef struct dsp_ring_buffer
{
    int16_t *buffer;
    int16_t buffer_length;
    uint16_t index;
} dsp_ring_buffer_t;

dsp_ring_buffer_t dsp_ring_buffer_init(int16_t *raw_buffer, uint16_t buffer_len);

void dsp_ring_buffer_put(dsp_ring_buffer_t *buffer, int16_t x);

uint8_t dsp_ring_buffer_read(dsp_ring_buffer_t *buffer, int16_t *x);

#endif