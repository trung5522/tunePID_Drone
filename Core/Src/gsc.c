/*
 * gsc.c
 *
 *  Created on: Mar 13, 2024
 *      Author: phamv
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "gsc.h"

extern UART_HandleTypeDef huart2;


void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf)
{


			  telemetry_tx_buf[0] = 0x46;
			  telemetry_tx_buf[1] = 0x43;

			  telemetry_tx_buf[2] = 0x10;

			  telemetry_tx_buf[3] = (short)(KalmanAngleRoll*100);
			  telemetry_tx_buf[4] = ((short)(KalmanAngleRoll*100))>>8;

			  telemetry_tx_buf[5] = (short)(KalmanAnglePitch*100);
			  telemetry_tx_buf[6] = ((short)(KalmanAnglePitch*100))>>8;

			  telemetry_tx_buf[7] = (unsigned short)(AngleYaw*100);
			  telemetry_tx_buf[8] = ((unsigned short)(AngleYaw*100))>>8;

			  telemetry_tx_buf[9] = 0x00;
			  telemetry_tx_buf[10] = 0x00;

			  telemetry_tx_buf[11] = (short)((ch.ch[0]-1500)*0.1f*100);
			  telemetry_tx_buf[12] = ((short)((ch.ch[0]-1500)*0.1f*100))>>8;

			  telemetry_tx_buf[13] = (short)((ch.ch[1]-1500)*0.1f*100);
			  telemetry_tx_buf[14] = ((short)((ch.ch[1]-1500)*0.1f*100))>>8;

			  telemetry_tx_buf[15] = (unsigned short)((ch.ch[3]-1000)*0.36f*100);
			  telemetry_tx_buf[16] = ((unsigned short)((ch.ch[3]-1000)*0.36f*100))>>8;

			  telemetry_tx_buf[17] = 0x00;
			  telemetry_tx_buf[18] = 0x00;

			  telemetry_tx_buf[19] = 0xff;

			  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];

}
//void Encode_Msg_GPS(unsigned char* telemetry_tx_buf,ppm_ch *ch)
//{
//	  telemetry_tx_buf[0] = 0x46;
//	  telemetry_tx_buf[1] = 0x43;
//
//	  telemetry_tx_buf[2] = 0x11;
//
//	  telemetry_tx_buf[3] = posllh.lat;
//	  telemetry_tx_buf[4] = posllh.lat>>8;
//	  telemetry_tx_buf[5] = posllh.lat>>16;
//	  telemetry_tx_buf[6] = posllh.lat>>24;
//
//	  telemetry_tx_buf[7] = posllh.lon;
//	  telemetry_tx_buf[8] = posllh.lon>>8;
//	  telemetry_tx_buf[9] = posllh.lon>>16;
//	  telemetry_tx_buf[10] = posllh.lon>>24;
//
//	  telemetry_tx_buf[11] = (unsigned short)(batVolt*100);
//	  telemetry_tx_buf[12] = ((unsigned short)(batVolt*100))>>8;
//
//	  telemetry_tx_buf[13] = ch->ch[5] == 1000 ? 0 : 1;
//	  telemetry_tx_buf[14] = ch->ch[6] == 1000 ? 0 : ch->ch[6] == 1500 ? 1 : 2;
//
//	  telemetry_tx_buf[15] = iBus_isActiveFailsafe(&iBus);
//
//	  telemetry_tx_buf[16] = 0x00;
//	  telemetry_tx_buf[17] = 0x00;
//	  telemetry_tx_buf[18] = 0x00;
//
//	  telemetry_tx_buf[19] = 0xff;
//
//	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
//}

void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;

	  telemetry_tx_buf[2] = id;

//	  memcpy(&telemetry_tx_buf[3], &p, 4);
//	  memcpy(&telemetry_tx_buf[7], &i, 4);
//	  memcpy(&telemetry_tx_buf[11], &d, 4);

	  *(float*)&telemetry_tx_buf[3] = p;
	  *(float*)&telemetry_tx_buf[7] = i;
	  *(float*)&telemetry_tx_buf[11] = d;

	  telemetry_tx_buf[15] = 0x00;
	  telemetry_tx_buf[16] = 0x00;
	  telemetry_tx_buf[17] = 0x00;
	  telemetry_tx_buf[18] = 0x00;

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
}

