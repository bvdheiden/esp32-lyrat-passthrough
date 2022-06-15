#include "dsp_filter.h"
#include "dsp_ring_buffer.h"

#include <stddef.h>
#include <stdio.h>
#include <math.h>

#define BANDWIDTH_GAIN 0.995f
#define DAMPING_GAIN 1.0f - BANDWIDTH_GAIN

#define INPUT_DIFFUSION_1_2_GAIN 0.750f
#define INPUT_DIFFUSION_3_4_GAIN 0.625f

// #define DECAY_DIFFUSION_1_GAIN 0.7f
#define DECAY_DIFFUSION_1_GAIN 0.5f
// #define DECAY_DIFFUSION_2_GAIN 0.5f
#define DECAY_DIFFUSION_2_GAIN 0.4f

// use smaller buffer lengths
#define INPUT_DIFFUSION_1_BUFFER_LENGTH 142
#define INPUT_DIFFUSION_2_BUFFER_LENGTH 107
#define INPUT_DIFFUSION_3_BUFFER_LENGTH 379
#define INPUT_DIFFUSION_4_BUFFER_LENGTH 277

#define DECAY_DIFFUSION_1_BUFFER_LENGTH 672
#define DECAY_DIFFUSION_2_BUFFER_LENGTH 1800

#define DAMPING_DELAY_1_BUFFER_LENGTH 4453
#define DAMPING_DELAY_2_BUFFER_LENGTH 3720

int16_t BANDWIDTH_FILTER_BUFFER = 0;
int16_t DAMPING_FILTER_BUFFER = 0;
int16_t REVERBERANCE_BUFFER = 0;

int16_t INPUT_DIFFUSION_1_BUFFER[INPUT_DIFFUSION_1_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_1_RINGBUFFER;
int16_t INPUT_DIFFUSION_2_BUFFER[INPUT_DIFFUSION_2_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_2_RINGBUFFER;
int16_t INPUT_DIFFUSION_3_BUFFER[INPUT_DIFFUSION_3_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_3_RINGBUFFER;
int16_t INPUT_DIFFUSION_4_BUFFER[INPUT_DIFFUSION_4_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_4_RINGBUFFER;

int16_t DECAY_DIFFUSION_1_BUFFER[DECAY_DIFFUSION_1_BUFFER_LENGTH];
dsp_ring_buffer_t DECAY_DIFFUSION_1_RINGBUFFER;
int16_t DECAY_DIFFUSION_2_BUFFER[DECAY_DIFFUSION_2_BUFFER_LENGTH];
dsp_ring_buffer_t DECAY_DIFFUSION_2_RINGBUFFER;
int16_t DAMPING_DELAY_1_BUFFER[DAMPING_DELAY_1_BUFFER_LENGTH];
dsp_ring_buffer_t DAMPING_DELAY_1_RINGBUFFER;
int16_t DAMPING_DELAY_2_BUFFER[DAMPING_DELAY_2_BUFFER_LENGTH];
dsp_ring_buffer_t DAMPING_DELAY_2_RINGBUFFER;

float decay_values[3] = {0.25f, 0.5f, 0.5f};

inline int16_t delay_ringbuffer(int16_t x, dsp_ring_buffer_t *buffer)
{
	if (buffer->buffer_length == 0)
	{
		return x;
	}

	int16_t y;

	dsp_ring_buffer_read(buffer, &y);
	dsp_ring_buffer_put(buffer, x);

	return y;
}

inline int16_t delay(int16_t x, int16_t *delay_line, int delay_len)
{
	if (delay_len == 0)
	{
		return x;
	}

	for (size_t i = delay_len - 1; i > 1; i--)
	{
		delay_line[i] = delay_line[i - 1];
	}

	delay_line[0] = x;

	return delay_line[delay_len - 1];
}

inline int16_t bandwith_filter(int16_t x, int16_t *prev_sample, float gain)
{
	*prev_sample = (gain * x) + ((1.0f - gain) * (*prev_sample));

	return *prev_sample;
}

inline int16_t damping_filter(int16_t x, int16_t *prev_sample, float gain)
{
	*prev_sample = (x * (1.0f - gain)) + ((*prev_sample) * gain);

	return *prev_sample;
}

inline int16_t allpass_iir_filter(int16_t x, int16_t *delay_line, int delay_len, float gain)
{
	int16_t delayed_x = delay_line[delay_len - 1];

	for (size_t i = delay_len - 1; i > 1; i--)
	{
		delay_line[i] = delay_line[i - 1];
	}

	x = x - (delayed_x * gain);

	delay_line[0] = x;

	return (x * gain) + delayed_x;
}

inline int16_t allpass_iir_filter_ringbuffer(int16_t x, dsp_ring_buffer_t *buffer, float gain)
{
	int16_t delayed_x;
	dsp_ring_buffer_read(buffer, &delayed_x);

	x = x - (delayed_x * gain);

	dsp_ring_buffer_put(buffer, x);

	return (x * gain) + delayed_x;
}

inline int16_t allpass_iir_filter_tap(int16_t x, int16_t *x_out, int16_t *delay_line, int delay_len, float gain)
{
	int16_t delayed_x = delay_line[delay_len - 1];

	*x_out = delayed_x;

	for (size_t i = delay_len - 1; i > 1; i--)
	{
		delay_line[i] = delay_line[i - 1];
	}

	x = x + (delayed_x * gain);

	delay_line[0] = x;

	return delayed_x - (x * gain);
}

inline int16_t allpass_iir_filter_tap_ringbuffer(int16_t x, int16_t *x_out, dsp_ring_buffer_t *buffer, float gain)
{
	int16_t delayed_x;
	dsp_ring_buffer_read(buffer, &delayed_x);

	*x_out = delayed_x;

	x = x + (delayed_x * gain);

	dsp_ring_buffer_put(buffer, x);

	return delayed_x - (x * gain);
}

uint32_t samples = 0;

void reverberance_filter_init()
{
    INPUT_DIFFUSION_1_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_1_BUFFER, INPUT_DIFFUSION_1_BUFFER_LENGTH);
    INPUT_DIFFUSION_2_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_2_BUFFER, INPUT_DIFFUSION_2_BUFFER_LENGTH);
    INPUT_DIFFUSION_3_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_3_BUFFER, INPUT_DIFFUSION_3_BUFFER_LENGTH);
    INPUT_DIFFUSION_4_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_4_BUFFER, INPUT_DIFFUSION_4_BUFFER_LENGTH);

    DECAY_DIFFUSION_1_RINGBUFFER = dsp_ring_buffer_init(DECAY_DIFFUSION_1_BUFFER, DECAY_DIFFUSION_1_BUFFER_LENGTH);
    DECAY_DIFFUSION_2_RINGBUFFER = dsp_ring_buffer_init(DECAY_DIFFUSION_2_BUFFER, DECAY_DIFFUSION_2_BUFFER_LENGTH);

    DAMPING_DELAY_1_RINGBUFFER = dsp_ring_buffer_init(DAMPING_DELAY_1_BUFFER, DAMPING_DELAY_1_BUFFER_LENGTH);
    DAMPING_DELAY_2_RINGBUFFER = dsp_ring_buffer_init(DAMPING_DELAY_2_BUFFER, DAMPING_DELAY_2_BUFFER_LENGTH);
}

int16_t reverb_init, reverb_feedback, reverb_out;

int16_t reverberance_filter_process(int16_t x, uint8_t decay_mode)
{
	if (decay_mode == 0 || decay_mode > 3)
		return x;

	reverb_init = bandwith_filter(x * 0.5f, &BANDWIDTH_FILTER_BUFFER, BANDWIDTH_GAIN);
	reverb_init = allpass_iir_filter_ringbuffer(reverb_init, &INPUT_DIFFUSION_1_RINGBUFFER, INPUT_DIFFUSION_1_2_GAIN);
	reverb_init = allpass_iir_filter_ringbuffer(reverb_init, &INPUT_DIFFUSION_2_RINGBUFFER, INPUT_DIFFUSION_1_2_GAIN);
	reverb_init = allpass_iir_filter_ringbuffer(reverb_init, &INPUT_DIFFUSION_3_RINGBUFFER, INPUT_DIFFUSION_3_4_GAIN);
	reverb_init = allpass_iir_filter_ringbuffer(reverb_init, &INPUT_DIFFUSION_4_RINGBUFFER, INPUT_DIFFUSION_3_4_GAIN);

	reverb_feedback = reverb_init + REVERBERANCE_BUFFER;

	reverb_feedback = allpass_iir_filter_tap_ringbuffer(reverb_feedback, &reverb_out, &DECAY_DIFFUSION_1_RINGBUFFER, DECAY_DIFFUSION_1_GAIN);
	reverb_feedback = delay_ringbuffer(reverb_feedback, &DAMPING_DELAY_1_RINGBUFFER);
	reverb_feedback = damping_filter(reverb_feedback, &DAMPING_FILTER_BUFFER, DAMPING_GAIN);
	reverb_feedback *= decay_values[decay_mode - 1];
	reverb_feedback = allpass_iir_filter_ringbuffer(reverb_feedback, &DECAY_DIFFUSION_2_RINGBUFFER, DECAY_DIFFUSION_2_GAIN);
	reverb_feedback = delay_ringbuffer(reverb_feedback, &DAMPING_DELAY_2_RINGBUFFER);
	reverb_feedback *= decay_values[decay_mode - 1];

	REVERBERANCE_BUFFER = reverb_feedback;

	// cap reverb out to prevent overflow
	reverb_out = -((INT16_MAX / 2) + 1) >= reverb_out ? -((INT16_MAX / 2) + 1) : reverb_out;
	reverb_out = ((INT16_MAX / 2) + 1) <= reverb_out ? ((INT16_MAX / 2) + 1) : reverb_out;

	// dry / wet ratio
	if (decay_mode == 1) {
		return (x * 0.75f) + (reverb_out * 0.25f);
	} else if (decay_mode == 2) {
		return (x * 0.5f) + (reverb_out * 0.5f);
	}

	return reverb_out;
}