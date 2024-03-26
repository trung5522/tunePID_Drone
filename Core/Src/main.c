/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu.h"
#include "math.h"
#include "gsc.h"
#include "stdlib.h"
#include "mpu.h"
#include "sbus.h"
#include "pid.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int mapValue(int value, int inMin, int inMax, int outMin, int outMax);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//check
uint8_t flag_ppm = 0;

//GSC and FC
uint8_t uart2_rx_data;
uint8_t telemetry_tx_buf[40];
uint8_t telemetry_rx_buf[20];
uint8_t telemetry_rx_cplt_flag;
float batVolt;
//PID
//float PID[6][3]; //Lần lượt roll pitch yaw và inter outter
extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;

float roll_in_kp;
float roll_in_ki;
float roll_in_kd;

float roll_out_kp;
float roll_out_ki;
float roll_out_kd;

float pitch_in_kp;
float pitch_in_ki;
float pitch_in_kd;

float pitch_out_kp;
float pitch_out_ki;
float pitch_out_kd;

float yaw_heading_kp;
float yaw_heading_ki;
float yaw_heading_kd;

float yaw_rate_kp;
float yaw_rate_ki;
float yaw_rate_kd;
//time
uint32_t loop_timer,cuoi2,time1,time2;
//mpu
long gyro_x_cal;
long gyro_y_cal;
long gyro_z_cal;

float acc_x;
float acc_y;
float acc_z;
float gyro_x;
float gyro_y;
float gyro_z;
float AccX, AccY, AccZ;

float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;
float angle_roll_output;
float angle_pitch_output;
float angle_yaw_output;
float angle_roll;
float angle_pitch;
float angle_yaw;
MPU9250_t mpu;
//cotrol
volatile ppm_ch ch;
uint8_t pulse=0;
volatile long tick;
int receiver_input_channel_1;
int receiver_input_channel_2;
int receiver_input_channel_3;
int receiver_input_channel_4;
int receiver_input_channel_5;
int receiver_input_channel_6;

//kalman
//kalman
float Kalman1DOutput[]={0,0};
float AngleRoll, AnglePitch,AngleYaw;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float DesiredAngleRoll, DesiredAnglePitch,DesiredRateYaw;

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;


//
//Timer
 uint8_t tim7_20ms_flag;
 uint8_t tim7_100ms_flag;
//
 //esc

 unsigned short ccr1, ccr2, ccr3, ccr4;

 //sbus
 extern uint8_t sbus_original_data[25];
 uint8_t rx_data4[128] = {0x00};
 uint8_t rx_size4 = 0;
 extern uint16_t sbus_ch[16];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static unsigned char cnt = 0;

	if(huart->Instance == USART2)
	{
		HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);

		switch(cnt)
		{
		case 0:
			if(uart2_rx_data == 0x47)
			{
				telemetry_rx_buf[cnt] = uart2_rx_data;
				cnt++;
			}
			break;
		case 1:
			if(uart2_rx_data == 0x53)
			{
				telemetry_rx_buf[cnt] = uart2_rx_data;
				cnt++;
			}
			else
				cnt = 0;
			break;
		case 19:
			telemetry_rx_buf[cnt] = uart2_rx_data;
			cnt = 0;
			telemetry_rx_cplt_flag = 1;
			break;
		default:
			telemetry_rx_buf[cnt] = uart2_rx_data;
			cnt++;
			break;
		}
	}
//	 else if (huart->Instance == USART1)
//	  {
//	    if(rx_size4 == 0)
//		{
//		  if(rx_data4[rx_size4] == 0x0F)
//			  rx_size4++;
//		  else
//			  rx_size4 = 0;
//		}
//		else
//		{
//			rx_size4++;
//			if(rx_size4 == 25)
//			{
//				for(int i = 0; i < 25; i++)
//				{
//					sbus_original_data[i] = rx_data4[i];
//				}
//
//				rx_size4 = 0;
//			}
//		}
//	    sbus_update();
//	    HAL_UART_Receive_IT(&huart1, rx_data4+rx_size4, 1);
//	  }
//	  else
//	  {
//
//	  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static unsigned char tim7_20ms_count = 0;
	static unsigned char tim7_100ms_count = 0;
	if(htim->Instance == htim4.Instance)
	 {

				tim7_20ms_count++;
				if(tim7_20ms_count == 20)
				{
					tim7_20ms_count = 0;
					tim7_20ms_flag = 1;
				}

				tim7_100ms_count++;
				if(tim7_100ms_count == 100)
				{
					tim7_100ms_count = 0;
					tim7_100ms_flag = 1;
				}
	 }
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.006*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState;
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_get_data(){
	 update_accel_gyro(&mpu);
	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	 gyro_x=mpu.g[0]-(2.9);
	 gyro_y=mpu.g[1]-(-2.15);
	 gyro_z=mpu.g[2]-(0.1);


	  AccX=mpu.a[0];
	  AccY=mpu.a[1];
	  AccZ=mpu.a[2];
	  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
	  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
	  AngleYaw=atan(AccZ/sqrt(AccY*AccY+AccX*AccX))*1/(3.142/180);
	  AnglePitch-=2.5;
	  AngleRoll-=(-0.7);

	//gyro_x *= -1;
//	gyro_y *= -1;
	//gyro_z *= -1;

}
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		flag_ppm = 1;
		tick = __HAL_TIM_GET_COUNTER(&htim5);
		__HAL_TIM_SET_COUNTER(&htim5,0);
		if(tick < 2108){

			if(pulse==2){
				//if(tick<1000 || abs(tick - ch.ch[pulse])>200) ch.ch[pulse]=ch.ch[pulse];
				//else
					ch.ch[pulse]= tick;
			}
			else if((pulse ==4) || (pulse ==5)){
				ch.ch[pulse]= tick;
			}
			else{
//				if(tick<1200 || tick >1700 ) ch[pulse]=ch[pulse];
//				else
				//if(tick<1000 || abs(tick - ch.ch[pulse])>200) ch.ch[pulse]=ch.ch[pulse];
				//else
				//	ch.ch[pulse]= mapValue(tick, 1200, 1700, 1000, 2000);
				ch.ch[pulse]= tick;
			}

			//ch[pulse]= tick;
			pulse++;
		}
		else{
			__HAL_TIM_SET_COUNTER(&htim5,0);
			pulse = 0;
		}

	}

}

int mapValue(int value, int inMin, int inMax, int outMin, int outMax) {
if (value < inMin) return outMin;
else if (value > inMax) return outMax;

return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim5);
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
  	//HAL_UART_Receive_IT(&huart1, rx_data4, 1);

  	HAL_TIM_Base_Start_IT(&htim4);
	while(flag_ppm==0) {
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 printf("\nPPM failed. Program shutting down...\n");

   	      }
   	printf("PPM OK!\n\n");

  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  	//HAL_Delay(10);
  	if(ch.ch[4] < 1100){
  			TIM3->CCR1 = 21000;
  	    	TIM3->CCR2 = 21000;
  	    	TIM3->CCR3 = 21000;
  	    	TIM3->CCR4 = 21000;
  	      HAL_Delay(7000);
  	      	TIM3->CCR1 = 10500;
  	      	TIM3->CCR2 = 10500;
  	      	TIM3->CCR3 = 10500;
  	      	TIM3->CCR4 = 10500;
  	      HAL_Delay(8000);
  		}
//      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
//      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
//      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
//      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);

   	 HAL_Delay(1000);
   	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
   	 HAL_Delay(1000);
   	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
   	 HAL_Delay(1000);
   	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
   	 HAL_Delay(1000);
   	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
   	 printf("Checking sensor connection..\n");

   	 MPU9250SetDefault(&mpu);
   	while(!(setupMPU(&mpu, MPU9250_ADDRESS)==1)) {
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 HAL_Delay(100);
      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
      	 printf("\nMPU9250 failed. Program shutting down...\n");

   	      }
   	printf("All sensors OK!\n\n");


      HAL_Delay(10);


  	  Encode_Msg_PID_Gain(&telemetry_tx_buf[0],0,roll_in_kp,roll_in_ki,roll_in_kd);
  	  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);

  	  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll_out_kp, roll_out_ki, roll_out_kd);
  	  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);

  	  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch_in_kp, pitch_in_ki, pitch_in_kd);
  	  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);

  	  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch_out_kp, pitch_out_ki, pitch_out_kd);
  	  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);

  	  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading_kp, yaw_heading_ki, yaw_heading_kd);
  	  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);

  	  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate_kp, yaw_rate_ki, yaw_rate_kd);
  	  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
      //HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);
      HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);
      HAL_Delay(10);

      for(int i=0;i<100;i++){
           if(updateMPU(&mpu)==1){
          	 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,0);
          	 HAL_Delay(100);
          	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
           }
       }
      while(!(ch.ch[4] > 1100 && ch.ch[2] <1010 )){
    	  	  	  	 HAL_Delay(100);
    	        	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    	        	 HAL_Delay(100);
    	        	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    	        	 HAL_Delay(100);
    	        	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    	        	 HAL_Delay(100);
    	        	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    	        	 printf("\nControl don't ready. Program shutting down...\n");

      }
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_TIM_Base_Start(&htim2);
      loop_timer = __HAL_TIM_GET_COUNTER(&htim2);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time1 = __HAL_TIM_GET_COUNTER(&htim2);
	  //ch.ch[5]=1000;
//tranfer
	  if((ch.ch[4] < 1600 && ch.ch[4] > 1400) && ch.ch[2] < 1005 )
	  {
			TIM3->CCR1 = 10500;
			TIM3->CCR2 = 10500;
			TIM3->CCR3 = 10500;
			TIM3->CCR4 = 10500;
			if(telemetry_rx_cplt_flag == 1)
			{
				telemetry_rx_cplt_flag = 0;



			  unsigned char chksum = 0xff;
			  for(int i=0;i<19;i++) chksum = chksum - telemetry_rx_buf[i];

			  if(chksum == telemetry_rx_buf[19])
			  {
//				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//
//				  TIM3->PSC = 1000;
//				  HAL_Delay(10);
//
//				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

				  switch(telemetry_rx_buf[2])
				  {
				  case 0:
					  roll_in_kp = *(float*)&telemetry_rx_buf[3];
					  roll_in_ki = *(float*)&telemetry_rx_buf[7];
					  roll_in_kd = *(float*)&telemetry_rx_buf[11];
					  roll.in.kp = *(float*)&telemetry_rx_buf[3];
					  roll.in.ki = *(float*)&telemetry_rx_buf[7];
					  roll.in.kd = *(float*)&telemetry_rx_buf[11];
					  //EP_PIDGain_Write(telemetry_rx_buf[2], roll_in_kp, roll_in_ki, roll_in_kd);
					  //EP_PIDGain_Read(telemetry_rx_buf[2], &roll_in_kp, &roll_in_ki, &roll_in_kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll_in_kp, roll_in_ki, roll_in_kd);
					  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
					  break;
				  case 1:
					  roll_out_kp = *(float*)&telemetry_rx_buf[3];
					  roll_out_ki = *(float*)&telemetry_rx_buf[7];
					  roll_out_kd = *(float*)&telemetry_rx_buf[11];
					  roll.out.kp = *(float*)&telemetry_rx_buf[3];
					  roll.out.ki = *(float*)&telemetry_rx_buf[7];
					  roll.out.kd = *(float*)&telemetry_rx_buf[11];
					  //EP_PIDGain_Write(telemetry_rx_buf[2], roll_out_kp, roll_out_ki, roll_out_kd);
					 // EP_PIDGain_Read(telemetry_rx_buf[2], &roll_out_kp, &roll_out_ki, &roll_out_kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll_out_kp, roll_out_ki, roll_out_kd);
					  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
					  break;
				  case 2:
					  pitch_in_kp = *(float*)&telemetry_rx_buf[3];
					  pitch_in_ki = *(float*)&telemetry_rx_buf[7];
					  pitch_in_kd = *(float*)&telemetry_rx_buf[11];
					  pitch.in.kp = *(float*)&telemetry_rx_buf[3];
					  pitch.in.ki = *(float*)&telemetry_rx_buf[7];
					  pitch.in.kd = *(float*)&telemetry_rx_buf[11];
					  //EP_PIDGain_Write(telemetry_rx_buf[2], pitch_in_kp, pitch_in_ki, pitch_in_kd);
					  //EP_PIDGain_Read(telemetry_rx_buf[2], &pitch_in_kp, &pitch_in_ki, &pitch_in_kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch_in_kp, pitch_in_ki, pitch_in_kd);
					  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
					  break;
				  case 3:
					  pitch_out_kp = *(float*)&telemetry_rx_buf[3];
					  pitch_out_ki = *(float*)&telemetry_rx_buf[7];
					  pitch_out_kd = *(float*)&telemetry_rx_buf[11];
					  pitch.out.kp = *(float*)&telemetry_rx_buf[3];
					  pitch.out.ki = *(float*)&telemetry_rx_buf[7];
					  pitch.out.kd = *(float*)&telemetry_rx_buf[11];
					  //EP_PIDGain_Write(telemetry_rx_buf[2], pitch_out_kp, pitch_out_ki, pitch_out_kd);
					  //EP_PIDGain_Read(telemetry_rx_buf[2], &pitch_out_kp, &pitch_out_ki, &pitch_out_kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch_out_kp, pitch_out_ki, pitch_out_kd);
					  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
					  break;
				  case 4:
					  yaw_heading_kp = *(float*)&telemetry_rx_buf[3];
					  yaw_heading_ki = *(float*)&telemetry_rx_buf[7];
					  yaw_heading_kd = *(float*)&telemetry_rx_buf[11];
					  yaw_heading.kp = *(float*)&telemetry_rx_buf[3];
					  yaw_heading.ki = *(float*)&telemetry_rx_buf[7];
					  yaw_heading.kd = *(float*)&telemetry_rx_buf[11];
					  //EP_PIDGain_Write(telemetry_rx_buf[2], yaw_heading_kp, yaw_heading_ki, yaw_heading_kd);
					  //EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_heading_kp, &yaw_heading_ki, &yaw_heading_kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_heading_kp, yaw_heading_ki, yaw_heading_kd);
					  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
					  break;
				  case 5:
					  yaw_rate_kp = *(float*)&telemetry_rx_buf[3];
					  yaw_rate_ki = *(float*)&telemetry_rx_buf[7];
					  yaw_rate_kd = *(float*)&telemetry_rx_buf[11];
					  yaw_rate.kp = *(float*)&telemetry_rx_buf[3];
					  yaw_rate.ki = *(float*)&telemetry_rx_buf[7];
					  yaw_rate.kd = *(float*)&telemetry_rx_buf[11];
					  //EP_PIDGain_Write(telemetry_rx_buf[2], yaw_rate_kp, yaw_rate_ki, yaw_rate_kd);
					  //EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_rate_kp, &yaw_rate_ki, &yaw_rate_kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_rate_kp, yaw_rate_ki, yaw_rate_kd);
					  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
					  break;
				  case 0x10:
					  switch(telemetry_rx_buf[3])
					  {

					  case 0:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], roll.in.kp, roll.in.ki, roll.in.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 1:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], roll.out.kp, roll.out.ki, roll.out.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 2:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], pitch.in.kp, pitch.in.ki, pitch.in.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 3:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], pitch.out.kp, pitch.out.ki, pitch.out.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 4:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 5:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 6:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
						  HAL_UART_Transmit(&huart2, &telemetry_tx_buf[0], 20, 10);
						  break;
					  }
					  break;

				  }
			  }
		  }
	  }

	  if(tim7_20ms_flag == 1 && tim7_100ms_flag != 1)
	  {
		  tim7_20ms_flag = 0;

		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);

		  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 20);
	  }

	  else if(tim7_20ms_flag == 1 && tim7_100ms_flag == 1)
	  {
		  tim7_20ms_flag = 0;
		  tim7_100ms_flag = 0;

		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);
		  //Encode_Msg_GPS(&telemetry_tx_buf[20]);

		  HAL_UART_Transmit_IT(&huart2, &telemetry_tx_buf[0], 40);
	  }

//end trans
	  //time1 = __HAL_TIM_GET_COUNTER(&htim4);
	 // if(updateMPU(&mpu)==1){}
	  gyro_get_data();
  	  //gyro_get_data();
	  //time2 = __HAL_TIM_GET_COUNTER(&htim2)-time1;
//	  receiver_input_channel_1 = ch.ch[2]; //thr
// 	  receiver_input_channel_2 = 1500;//ch[0]; //roll
// 	  receiver_input_channel_3 = 1500;//ch[1];	//pitch
// 	  receiver_input_channel_4 = 1500;//ch[3]; //yaw
// 	  receiver_input_channel_5 = ch.ch[4];	 //sw left
// 	  receiver_input_channel_6 = ch.ch[5]; //sw right

// 	  	gyro_pitch_input 	= ( gyro_pitch_input * 0.7 ) + (float)( gyro_y  * 0.3);
// 		gyro_roll_input 	= ( gyro_roll_input * 0.7 ) + (float)( gyro_x  * 0.3);
// 		gyro_yaw_input 	= ( gyro_yaw_input * 0.7 ) + (float)( gyro_z  * 0.3);
		//kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, gyro_x, AngleRoll);
		//KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
		KalmanAngleRoll=AngleRoll;
		//kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, gyro_y, AnglePitch);
		//KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
		KalmanAnglePitch=AnglePitch;
		//PID

		if(ch.ch[4] > 1900){
			Double_Roll_Pitch_PID_Calculation(&pitch, (ch.ch[1] - 1500) * 0.1f, KalmanAnglePitch, gyro_y);
			Double_Roll_Pitch_PID_Calculation(&roll, (ch.ch[0] - 1500) * 0.1f, KalmanAngleRoll, gyro_x);

			if(ch.ch[2] < 1010)
			{
				Reset_All_PID_Integrator();
			}
			  ccr1 = 10500 + 500 + (ch.ch[2] - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result;// - (iBus.LH - 1500) * 5;
			  ccr2 = 10500 + 500 + (ch.ch[2] - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result;// + (iBus.LH - 1500) * 5;
			  ccr3 = 10500 + 500 + (ch.ch[2] - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result;// - (iBus.LH - 1500) * 5;
			  ccr4 = 10500 + 500 + (ch.ch[2] - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result;// + (iBus.LH - 1500) * 5;
			  if(ch.ch[2] > 1010)
			  {
				  TIM3->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 11000 ? 11000 : ccr1;
				  TIM3->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 11000 ? 11000 : ccr2;
				  TIM3->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 11000 ? 11000 : ccr3;
				  TIM3->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 11000 ? 11000 : ccr4;
			  }
			  else
			  {
				  TIM3->CCR1 = 11000;
				  TIM3->CCR2 = 11000;
				  TIM3->CCR3 = 11000;
				  TIM3->CCR4 = 11000;
			  }
		}
		else{

		}






		time2 = __HAL_TIM_GET_COUNTER(&htim2)-time1;
		while(abs(__HAL_TIM_GET_COUNTER(&htim2) - loop_timer) < 4000 );
		cuoi2 = abs(__HAL_TIM_GET_COUNTER(&htim2) - loop_timer);
		__HAL_TIM_SET_COUNTER(&htim2,0);
		loop_timer = __HAL_TIM_GET_COUNTER(&htim2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 41999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
