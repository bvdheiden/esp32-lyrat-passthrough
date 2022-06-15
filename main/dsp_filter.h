#ifndef _DSP_FILTER_H_
#define _DSP_FILTER_H_

#include <stdint.h>

void reverberance_filter_init();

int16_t reverberance_filter_process(int16_t x, uint8_t decay_mode);

#endif
