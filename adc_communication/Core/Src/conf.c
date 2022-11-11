#include "conf.h"

void adc_set_channel(ADC_HandleTypeDef *hadc, uint16_t channel)
{
	hadc->Instance->SQR3 = channel;
}
EGOR TUT DOBAVIL
void set_period(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t val)
{
	// adc start every second period of timer interrupt
	// this freq using only for timer settings
	uint32_t freq = (HAL_RCC_GetSysClockFreq() / htim->Init.Prescaler) / 2;
	if(val > 0 && val < 65535){
		HAL_TIM_OC_Stop_IT(htim, channel);
		uint32_t period = freq / val;
		htim->Instance->ARR = period;

		//htim->Instance->CCR4 = period; // pulse
		HAL_TIM_OC_Start_IT(htim, channel);
	}
}

uint16_t set_frame_len(uint16_t samples)
{
	/* resolution of ADC - 12 => one sample = 2 bytes
	 * 1024 + 16 max length for transferring without fragmenting */
	return samples >= 512 ? 1024 + 16 : 512 + 16;
}


static void frame_service_info(uint16_t *dest, int *dest_pos, Frame *frame, RX_data *recv_data)
{
	if(*dest_pos == 0 || !(*dest_pos % (frame->frame_len / 2))){
		if(frame->num_of_frames < 16)
			frame->buff_ptr[frame->num_of_frames] = (uint8_t *)(dest + *dest_pos);
		dest[(*dest_pos)++] = recv_data->adc_channel;
		dest[(*dest_pos)++] = frame->num_of_frames++;
		for(int i = 0 ; i < 6; i++)
			dest[(*dest_pos)++] = 0xffff;
	}
}

void frame_creation(uint16_t *dest, volatile uint16_t *src, uint16_t d_len, uint16_t s_len, volatile uint16_t offset, RX_data *recv_data, Frame *frame)
{
	if(offset > s_len || recv_data->adc_samples > s_len)
		return;

	for(int i = offset, dest_pos = 0; i < s_len && i < recv_data->adc_samples && dest_pos < d_len; i++){
		frame_service_info(dest, &dest_pos, frame, recv_data);
		dest[dest_pos++] = src[i];
	}
	for(int i = 0, dest_pos = s_len - offset; i < offset && i < recv_data->adc_samples && dest_pos < d_len; i++){
		frame_service_info(dest, &dest_pos, frame, recv_data);
		dest[dest_pos++] = src[i];
	}
	frame->total_data = frame->num_of_frames * frame->frame_len;
}

void fill_data(RX_data *recv, uint8_t *buf_rx, Frame *frame)
{
	recv->adc_channel = buf_rx[0];
	recv->adc_channel = (recv->adc_channel << 8) + buf_rx[1];
	if(recv->adc_channel > 4 || recv->adc_channel < 1){
		recv->send_num = 4;
		recv->adc_channel = 0;
	}else
		recv->send_num = recv->adc_channel + 1;

	recv->adc_samples = buf_rx[2];
	recv->adc_samples = (recv->adc_samples << 8) + buf_rx[3];

	recv->timer_period = buf_rx[4];
	recv->timer_period = (recv->timer_period << 8) + buf_rx[5];

	frame->frame_len = recv->adc_samples >= 512 ? 1024 + 16 : 512 + 16;
	frame->num_of_frames = 0;
	frame->total_data = 0;
}

void clear_data(Frame *frame, RX_data *recv_data, int i)
{
	frame->num_of_frames = 0;
	frame->total_data = 0;
	recv_data->adc_channel = i;
}
