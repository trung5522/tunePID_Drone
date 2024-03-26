/*
 * sbus.c
 *
 *  Created on: Mar 21, 2024
 *      Author: phamv
 */


#include "sbus.h"
#include "main.h"

extern volatile ppm_ch ch;
uint8_t sbus_original_data[25] = {0x00};
uint16_t sbus_ch[16] = {0};
uint8_t send_channels_data[21] = {0xFE, 0xEF, 0x15, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void sbus_update(void)
{
	sbus_ch[ 0] = ((int16_t)sbus_original_data[ 1] >> 0 | ((int16_t)sbus_original_data[ 2] << 8 )) & 0x07FF;
	ch.ch[0]=sbus_ch[0];
	sbus_ch[ 1] = ((int16_t)sbus_original_data[ 2] >> 3 | ((int16_t)sbus_original_data[ 3] << 5 )) & 0x07FF;
	ch.ch[1]=sbus_ch[1];
	sbus_ch[ 2] = ((int16_t)sbus_original_data[ 3] >> 6 | ((int16_t)sbus_original_data[ 4] << 2 )  | (int16_t)sbus_original_data[ 5] << 10 ) & 0x07FF;
	ch.ch[2]=sbus_ch[2];
	sbus_ch[ 3] = ((int16_t)sbus_original_data[ 5] >> 1 | ((int16_t)sbus_original_data[ 6] << 7 )) & 0x07FF;
	ch.ch[3]=sbus_ch[3];
	sbus_ch[ 4] = ((int16_t)sbus_original_data[ 6] >> 4 | ((int16_t)sbus_original_data[ 7] << 4 )) & 0x07FF;
	ch.ch[4]=sbus_ch[4];
	sbus_ch[ 5] = ((int16_t)sbus_original_data[ 7] >> 7 | ((int16_t)sbus_original_data[ 8] << 1 )  | (int16_t)sbus_original_data[ 9] <<  9 ) & 0x07FF;
	ch.ch[5]=sbus_ch[5];
	sbus_ch[ 6] = ((int16_t)sbus_original_data[ 9] >> 2 | ((int16_t)sbus_original_data[10] << 6 )) & 0x07FF;
	ch.ch[6]=sbus_ch[6];
	sbus_ch[ 7] = ((int16_t)sbus_original_data[10] >> 5 | ((int16_t)sbus_original_data[11] << 3 )) & 0x07FF;
	ch.ch[7]=sbus_ch[7];
	sbus_ch[ 8] = ((int16_t)sbus_original_data[12] << 0 | ((int16_t)sbus_original_data[13] << 8 )) & 0x07FF;
	sbus_ch[ 9] = ((int16_t)sbus_original_data[13] >> 3 | ((int16_t)sbus_original_data[14] << 5 )) & 0x07FF;
	sbus_ch[10] = ((int16_t)sbus_original_data[14] >> 6 | ((int16_t)sbus_original_data[15] << 2 )  | (int16_t)sbus_original_data[16] << 10 ) & 0x07FF;
	sbus_ch[11] = ((int16_t)sbus_original_data[16] >> 1 | ((int16_t)sbus_original_data[17] << 7 )) & 0x07FF;
	sbus_ch[12] = ((int16_t)sbus_original_data[17] >> 4 | ((int16_t)sbus_original_data[18] << 4 )) & 0x07FF;
	sbus_ch[13] = ((int16_t)sbus_original_data[18] >> 7 | ((int16_t)sbus_original_data[19] << 1 )  | (int16_t)sbus_original_data[20] <<  9 ) & 0x07FF;
	sbus_ch[14] = ((int16_t)sbus_original_data[20] >> 2 | ((int16_t)sbus_original_data[21] << 6 )) & 0x07FF;
	sbus_ch[15] = ((int16_t)sbus_original_data[21] >> 5 | ((int16_t)sbus_original_data[22] << 3 )) & 0x07FF;
}
