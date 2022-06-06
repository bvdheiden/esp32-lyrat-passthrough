#include <esp_log.h>
#include <driver/i2s.h>
#include <driver/i2c.h>

#include "es8388_registers.h"
#include "dsp_ring_buffer.h"

/*
 * Basic I2S and I2C Configuration
 */
#define I2S_NUM I2S_NUM_0
#define I2S_READLEN 50 * 4

#define I2C_NUM I2C_NUM_0
#define ES8388_ADDR 0x20

/*
 * ES8388 Configuration Code
 * Configure ES8388 audio codec over I2C for AUX IN input and headphone jack output
 */
static esp_err_t es_write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
	esp_err_t res = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	res |= i2c_master_start(cmd);
	res |= i2c_master_write_byte(cmd, slave_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_stop(cmd);
	res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return res;
}

static esp_err_t es8388_init()
{
	esp_err_t res = ESP_OK;

	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		//.sda_io_num = GPIO_NUM_18, // Espressif LyraT
		.sda_io_num = GPIO_NUM_33, // AI thinker ESP32-A1S
		.sda_pullup_en = true,
		// .scl_io_num = GPIO_NUM_23, // Espressif LyraT
		.scl_io_num = GPIO_NUM_32, // AI thinker ESP32-A1S
		.scl_pullup_en = true,
		.master.clk_speed = 100000};

	res |= i2c_param_config(I2C_NUM, &i2c_config);
	res |= i2c_driver_install(I2C_NUM, i2c_config.mode, 0, 0, 0);

	/* mute DAC during setup, power up all systems, slave mode */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

	/* power up DAC and enable only LOUT1 / ROUT1, ADC sample rate = DAC sample rate */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x30);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

	/* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

	/* DAC to output route mixer configuration */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

	/* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

	/* DAC volume control: 0dB (maximum, unattenuated)  */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

	/* power down ADC while configuring; volume: +9dB for both channels */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x33);

	/* select LINPUT2 / RINPUT2 as ADC input; stereo; 16 bit word length, format right-justified, MCLK / Fs = 256 */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0e);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);

	/* set ADC volume */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x20);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x20);

	/* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

	/* power up and enable DAC; power up ADC (no MIC bias) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);

	return res;
}

// #define BANDWIDTH_VAL 0.9995f
#define BANDWIDTH_VAL 0.995f

#define INPUT_DIFFUSION_1_2_GAIN 0.750f
#define INPUT_DIFFUSION_3_4_GAIN 0.625f

#define DECAY_DIFFUSION_1_GAIN 0.7f
#define DECAY_DIFFUSION_2_GAIN 0.5f

#define DAMPING_GAIN 0.0005f

#define DECAY_GAIN 0.7f

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

int16_t INPUT_DIFFUSION_1_BUFFER[INPUT_DIFFUSION_1_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_1_RINGBUFFER;
int16_t INPUT_DIFFUSION_2_BUFFER[INPUT_DIFFUSION_2_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_2_RINGBUFFER;
int16_t INPUT_DIFFUSION_3_BUFFER[INPUT_DIFFUSION_3_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_3_RINGBUFFER;
int16_t INPUT_DIFFUSION_4_BUFFER[INPUT_DIFFUSION_4_BUFFER_LENGTH];
dsp_ring_buffer_t INPUT_DIFFUSION_4_RINGBUFFER;

dsp_ring_buffer_t DECAY_DIFFUSION_1_RINGBUFFER;
int16_t DECAY_DIFFUSION_1_BUFFER[DECAY_DIFFUSION_1_BUFFER_LENGTH];
dsp_ring_buffer_t DECAY_DIFFUSION_2_RINGBUFFER;
int16_t DECAY_DIFFUSION_2_BUFFER[DECAY_DIFFUSION_2_BUFFER_LENGTH];

int16_t DAMPING_FILTER_BUFFER = 0;

dsp_ring_buffer_t DAMPING_DELAY_1_RINGBUFFER;
int16_t DAMPING_DELAY_1_BUFFER[DAMPING_DELAY_1_BUFFER_LENGTH];
dsp_ring_buffer_t DAMPING_DELAY_2_RINGBUFFER;
int16_t DAMPING_DELAY_2_BUFFER[DAMPING_DELAY_2_BUFFER_LENGTH];

int16_t REVERBERANCE_BUFFER = 0;

int16_t delay_ringbuffer(int16_t x, dsp_ring_buffer_t *buffer)
{
	if (buffer->buffer_length == 0)
	{
		return x;
	}

	dsp_ring_buffer_put(buffer, x);
	dsp_ring_buffer_read(buffer, &x);

	return x;
}

int16_t delay(int16_t x, int16_t *delay_line, int delay_len)
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

int16_t bandwith_filter(int16_t x, int16_t *prev_sample, float gain)
{
	*prev_sample = (gain * x) + ((1.0f - gain) * (*prev_sample));

	return *prev_sample;
}

int16_t damping_filter(int16_t x, int16_t *prev_sample, float gain)
{
	*prev_sample = (x * (1.0f - gain)) + ((*prev_sample) * gain);

	return *prev_sample;
}

int16_t allpass_iir_filter(int16_t x, int16_t *delay_line, int delay_len, float gain)
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

int16_t allpass_iir_filter_ringbuffer(int16_t x, dsp_ring_buffer_t *buffer, float gain)
{
	int16_t delayed_x;
	dsp_ring_buffer_read(buffer, &delayed_x);

	x = x - (delayed_x * gain);

	dsp_ring_buffer_put(buffer, x);

	return (x * gain) + delayed_x;
}

int16_t allpass_iir_filter_tap(int16_t x, int16_t *x_out, int16_t *delay_line, int delay_len, float gain)
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

int16_t allpass_iir_filter_tap_ringbuffer(int16_t x, int16_t *x_out, dsp_ring_buffer_t *buffer, float gain)
{
	int16_t delayed_x;
	dsp_ring_buffer_read(buffer, &delayed_x);

	*x_out = delayed_x;

	x = x + (delayed_x * gain);

	dsp_ring_buffer_put(buffer, x);

	return delayed_x - (x * gain);
}

uint32_t samples = 0;

static inline int16_t reverberance_filter(int16_t x)
{
	int16_t pre_feedback, feedback, pre_mix;

	samples++;

	pre_feedback = bandwith_filter(x, &BANDWIDTH_FILTER_BUFFER, BANDWIDTH_VAL);
	pre_feedback = allpass_iir_filter_ringbuffer(pre_feedback, &INPUT_DIFFUSION_1_RINGBUFFER, INPUT_DIFFUSION_1_2_GAIN);
	pre_feedback = allpass_iir_filter_ringbuffer(pre_feedback, &INPUT_DIFFUSION_2_RINGBUFFER, INPUT_DIFFUSION_1_2_GAIN);
	pre_feedback = allpass_iir_filter_ringbuffer(pre_feedback, &INPUT_DIFFUSION_3_RINGBUFFER, INPUT_DIFFUSION_3_4_GAIN);
	pre_feedback = allpass_iir_filter_ringbuffer(pre_feedback, &INPUT_DIFFUSION_4_RINGBUFFER, INPUT_DIFFUSION_3_4_GAIN);

	feedback = pre_feedback + REVERBERANCE_BUFFER;

	feedback = allpass_iir_filter_tap_ringbuffer(feedback, &pre_mix, &DECAY_DIFFUSION_1_RINGBUFFER, DECAY_DIFFUSION_1_GAIN);
	feedback = delay_ringbuffer(feedback, &DAMPING_DELAY_1_RINGBUFFER);
	feedback = damping_filter(feedback, &DAMPING_FILTER_BUFFER, DAMPING_GAIN);
	feedback *= DECAY_GAIN;
	feedback = allpass_iir_filter_ringbuffer(feedback, &DECAY_DIFFUSION_2_RINGBUFFER, DECAY_DIFFUSION_2_GAIN);
	feedback = delay_ringbuffer(feedback, &DAMPING_DELAY_2_RINGBUFFER);
	feedback *= DECAY_GAIN;

	REVERBERANCE_BUFFER = feedback;

	// dry / wet ratio
	int16_t out = (x * 0.3f) + (pre_mix * 0.7f);

	// if ((samples % 1000) == 0) {
	// 	printf("%d %d %d %d %d\n", x, pre_feedback, feedback, pre_mix, out);
	// }

	return out;
}

/*
 * Main
 */
void app_main(void)
{
	printf("[filter-dsp] Initializing audio codec via I2C...\r\n");

	if (es8388_init() != ESP_OK)
	{
		printf("[filter-dsp] Audio codec initialization failed!\r\n");
	}
	else
	{
		printf("[filter-dsp] Audio codec initialization OK\r\n");
	}

	INPUT_DIFFUSION_1_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_1_BUFFER, INPUT_DIFFUSION_1_BUFFER_LENGTH);
	INPUT_DIFFUSION_2_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_2_BUFFER, INPUT_DIFFUSION_2_BUFFER_LENGTH);
	INPUT_DIFFUSION_3_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_3_BUFFER, INPUT_DIFFUSION_3_BUFFER_LENGTH);
	INPUT_DIFFUSION_4_RINGBUFFER = dsp_ring_buffer_init(INPUT_DIFFUSION_4_BUFFER, INPUT_DIFFUSION_4_BUFFER_LENGTH);

	DECAY_DIFFUSION_1_RINGBUFFER = dsp_ring_buffer_init(DECAY_DIFFUSION_1_BUFFER, DECAY_DIFFUSION_1_BUFFER_LENGTH);
	DECAY_DIFFUSION_2_RINGBUFFER = dsp_ring_buffer_init(DECAY_DIFFUSION_2_BUFFER, DECAY_DIFFUSION_2_BUFFER_LENGTH);

	DAMPING_DELAY_1_RINGBUFFER = dsp_ring_buffer_init(DAMPING_DELAY_1_BUFFER, DAMPING_DELAY_1_BUFFER_LENGTH);
	DAMPING_DELAY_2_RINGBUFFER = dsp_ring_buffer_init(DAMPING_DELAY_2_BUFFER, DAMPING_DELAY_2_BUFFER_LENGTH);

	/*******************/

	printf("[filter-dsp] Initializing input I2S...\r\n");

	i2s_config_t i2s_read_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
		.sample_rate = 44100,
		.bits_per_sample = 16,
		.communication_format = I2S_COMM_FORMAT_I2S,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
		.dma_buf_count = 3,
		.dma_buf_len = I2S_READLEN,
		.use_apll = 1,
		.tx_desc_auto_clear = 1,
		.fixed_mclk = 0};

	i2s_pin_config_t i2s_read_pin_config = {
		//.bck_io_num = GPIO_NUM_5,
		.bck_io_num = GPIO_NUM_27, // AI thinker ESP32-A1S
		.ws_io_num = GPIO_NUM_25,
		.data_out_num = GPIO_NUM_26,
		.data_in_num = GPIO_NUM_35};

	i2s_driver_install(I2S_NUM, &i2s_read_config, 0, NULL);
	i2s_set_pin(I2S_NUM, &i2s_read_pin_config);

	/*******************/

	printf("[filter-dsp] Initializing MCLK output...\r\n");

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	WRITE_PERI_REG(PIN_CTRL, 0xFFF0);

	/*******************/

	printf("[filter-dsp] Enabling Passthrough mode...\r\n");

	size_t i2s_bytes_read = 0;
	size_t i2s_bytes_written = 0;

	int16_t i2s_buffer_read[I2S_READLEN / sizeof(int16_t)];
	int16_t i2s_buffer_write[I2S_READLEN / sizeof(int16_t)];

	/* continuously read data over I2S, pass it through the filtering function and write it back */
	while (true)
	{
		i2s_bytes_read = I2S_READLEN;
		i2s_read(I2S_NUM, i2s_buffer_read, I2S_READLEN, &i2s_bytes_read, 100);

		/* Both channels filter */
		for (uint32_t i = 0; i < i2s_bytes_read / 2; i += 2)
		{
			i2s_buffer_write[i] = reverberance_filter(i2s_buffer_read[i + 1]);
			i2s_buffer_write[i + 1] = i2s_buffer_write[i];
		}

		i2s_write(I2S_NUM, i2s_buffer_write, i2s_bytes_read, &i2s_bytes_written, 100);
	}
}
