/*
 * gsc.h
 *
 *  Created on: Mar 13, 2024
 *      Author: phamv
 */
#include "main.h"
#ifndef INC_GSC_H_
#define INC_GSC_H_

//extern uint8_t telemetry_tx_buf[40];
extern uint8_t telemetry_tx_buf[40];
extern uint8_t telemetry_rx_buf[20];
extern uint8_t telemetry_rx_cplt_flag;
extern float KalmanAngleRoll;
extern float KalmanAnglePitch;
extern float AngleYaw;
extern volatile ppm_ch ch;

void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf);
void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d);

#endif /* INC_GSC_H_ */
