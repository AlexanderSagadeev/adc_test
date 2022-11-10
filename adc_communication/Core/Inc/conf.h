#ifndef _CONF_
#define _CONF_

#include "main.h"

typedef struct __rx_data{
	uint16_t adc_channel;
	uint16_t adc_samples;
	uint16_t timer_period;
	uint16_t send_num;
}RX_data;

typedef struct __frame{
	uint16_t frame_len;
	uint16_t num_of_frames;
	uint16_t total_data;
	uint8_t *buff_ptr[16];
}Frame;

void adc_set_channel(ADC_HandleTypeDef *hadc, uint16_t channel);
void set_period(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t val);
uint16_t set_frame_len(uint16_t samples);
void fill_data(RX_data *recv, uint8_t *buf_rx, Frame *frame);
void frame_creation(uint16_t *dest, volatile uint16_t *src, uint16_t d_len, uint16_t s_len, volatile uint16_t offset, RX_data *recv_data, Frame *frame);
void clear_data(Frame *frame, RX_data *recv_data, int i);
#endif
