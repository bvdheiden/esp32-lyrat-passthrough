#include "dsp_ring_buffer.h"

#include <string.h>

dsp_ring_buffer_t dsp_ring_buffer_init(int16_t *raw_buffer, uint16_t buffer_len)
{
    memset(raw_buffer, 0, buffer_len);

    dsp_ring_buffer_t buffer;
    buffer.buffer = raw_buffer;
    buffer.buffer_length = buffer_len;
    buffer.index = 0;
    return buffer;
}

void dsp_ring_buffer_put(dsp_ring_buffer_t *buffer, int16_t x)
{
    buffer->buffer[buffer->index] = x;

    uint16_t new_index = buffer->index + 1;
    if (new_index >= buffer->buffer_length)
    {
        new_index = 0;
    }

    buffer->index = new_index;
}

uint8_t dsp_ring_buffer_read(dsp_ring_buffer_t *buffer, int16_t *x)
{
    uint16_t read_index = buffer->index + 1;
    if (read_index >= buffer->buffer_length)
    {
        read_index = 0;
    }

    *x = buffer->buffer[read_index];

    return 1;
}
