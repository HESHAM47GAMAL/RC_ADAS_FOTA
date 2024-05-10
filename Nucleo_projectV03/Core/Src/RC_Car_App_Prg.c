
/************************************************************************************************/
/*************  		Author : Eslam Elwey       **********************************************/
/*************  		Date   : 28/4/2024         **********************************************/
/*************  		File   : RC_Car_App_Prg.c  **********************************************/
/************************************************************************************************/



/******************************** Includes ********************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include <machine/_default_types.h>
#include "Motor_Driver.h"
#include "Ultrasonic.h"
#include "RC_Car_App_Int.h"

/************************************ Private Functions *****************************************/

static void Car_EngineFunc(void) ;
static void Car_RightSignalFunc(void) ;
static void Car_LeftSignalFunc(void) ;
static void Car_WaitSignalFunc(void) ;
static void Car_IdleSignalFunc(void) ;
static void Car_NoneSignalFunc(void) ;
static void Car_FlashingFunc (flashing_t state) ;
static void Car_Gear_Func (gear_t state) ;
static void Car_Horn_Func (horn_t state) ;

/*************************************************************************************************/

static volatile state_t			    Current_state   = 	STATE_NORMAL ;
static volatile state_t 			Previous_state  = 	STATE_NORMAL ;
static volatile transition_t 	    transition      = 	SIG_NONE	 ;
static volatile engine_t			engine_state    =  ENGINE_DISABLE ;
static volatile uint8_t 			sync_flag       =  0  ;
static volatile break_assiat_t      assist_state    = BREAK_ASSIST_DISABLE ;
static volatile right_park_t		right_park_stat = RIGHT_PARK_DISABLE ;
static volatile left_park_t		    left_park_stat  = LEFT_PARK_DISABLE ;
static SemaphoreHandle_t 			flashing_smphr  =  NULL ;
static SemaphoreHandle_t 			horn_smphr  	=  NULL ;
static SemaphoreHandle_t			Gear_smphr		=  NULL ;
static SemaphoreHandle_t			speedInc_smphr	=  NULL ;
static SemaphoreHandle_t			brake_smphr	    =  NULL ;
static SemaphoreHandle_t			goRight_smphr   =  NULL ;
static SemaphoreHandle_t			goLeft_smphr	=  NULL ;
static SemaphoreHandle_t			assist_smphr	=  NULL ;
static SemaphoreHandle_t			right_park_smphr=  NULL ;
static SemaphoreHandle_t			left_park_smphr =  NULL ;
static volatile gear_t 				gear_current    =  GEAR_N  ;
static volatile int8_t				Motor_Speed		= SPEED_INIT_VALUE ;
static volatile int8_t				servo_angle		= SERVO_INIT_ANGLE ;
static volatile park_t				park_state      = PARK_DISABLE ;
static volatile uint16_t 			Ultrasonic_Buffer[6] = {0};
static volatile uint16_t 			Important_Buffer[6] = {0};
static volatile ULTRASONIC_t 		current_ultrasonic = USF ;

/********************************* RTOS Tasks Implementation *************************************/

static void string_toIint(uint16_t num_o)
{
	uint16_t Num  = num_o;
			uint16_t reversed_Num = 0 ;
			uint8_t I2S[6] = {};
			uint8_t Count_Num = 0;
			uint8_t ten_multiple = 0 ;
			uint8_t firstZeros = 0 ;
			while(Num > 0)
			{
			  reversed_Num *= 10 ;
			  if(firstZeros == 0)
			  {
				  if( (Num %10 ) == 0)
				  {
					  ten_multiple++;
				  }
				  else
				  {
					  firstZeros = 1 ;
				  }
			  }
			  reversed_Num += Num %10 ;
			  Num /= 10;
			}
			while(reversed_Num > 0)
			{
			  I2S[Count_Num] = (reversed_Num % 10) + '0';
			  reversed_Num /= 10;
			  Count_Num++;
			}
			while(ten_multiple)
			{
				I2S[Count_Num] = '0';
			    Count_Num++;
			    ten_multiple--;
			}
			I2S[Count_Num] = '\0';

			HAL_UART_Transmit(&huart3 ,I2S,Count_Num,50);
			HAL_UART_Transmit(&huart3 ,"\r\n",2,50);
}


void RC_SystemInit (void)
{
	flashing_smphr     =  xSemaphoreCreateBinary() ;
	horn_smphr         =  xSemaphoreCreateBinary() ;
	Gear_smphr         =  xSemaphoreCreateBinary() ;
	speedInc_smphr     =  xSemaphoreCreateBinary() ;
	brake_smphr        =  xSemaphoreCreateBinary() ;
	goRight_smphr  	   =  xSemaphoreCreateBinary() ;
	goLeft_smphr       =  xSemaphoreCreateBinary() ;
	assist_smphr       =  xSemaphoreCreateBinary() ;
	right_park_smphr   =  xSemaphoreCreateBinary() ;
	left_park_smphr    =  xSemaphoreCreateBinary() ;

	HAL_ServoMotor_Init() ;
	HAL_ServoMotor_Set_Angel(0) ;

	HAL_DC_Motor_Init() ;
	HAL_DC_Motors_Stop() ;
	HAL_DC_Motors_Set_Speed(0) ;

	UltrasonicInit() ;



	HAL_GPIO_WritePin(LEDS_START_ENGINE_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_YELLOW_RIGHT_SIDE,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_YELLOW_LEFT_SIDE,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_FRONT_WHITE_FLASHING,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_BACK_WHITE_PARK,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_PIN,GPIO_PIN_SET);

}


void T1_HandleUartReceive (void* pvarg)
{
	HAL_StatusTypeDef l_retval = HAL_OK ;
	uint8_t data ;

	while (1)
	{
		HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
		HAL_DC_Motors_Set_Speed(Motor_Speed) ;
		HAL_UART_Transmit(&huart3 ,"Speed:",6,50);
		string_toIint(Motor_Speed) ;
		HAL_UART_Transmit(&huart3 ,"\r\n",2,50);
		HAL_UART_Transmit(&huart3 ,&data,1,50);
		HAL_UART_Transmit(&huart3 ,"\r\n",2,50);
		/* handle break assist */
		if ((ENGINE_ENABLE==engine_state)&&(BREAK_ASSIST_ENABLE==assist_state))
		{
			/* Compare current front ultrasonic distance with maximum accepted value
			  if current val is less than maximum accepted value stop car you are crashing    */
			if (gear_current==GEAR_D)
			{
				Ultrasonic_Distance(USF,&Ultrasonic_Buffer[USF]) ;
				if (Ultrasonic_Buffer[USF]<BREAK_ASSIST_MAXIMUM_ACCEPTED_DIST)
				{
					HAL_DC_Motors_Stop() ;
				}
				else
				{
					HAL_DC_Motors_Forward() ;
				}
			}

			/* Compare current Back ultrasonic distance with maximum accepted value
				if current val is less than maximum accepted value stop car you are crashing    */

			else if (gear_current==GEAR_R)
			{
				Ultrasonic_Distance(USB,&Ultrasonic_Buffer[USB]) ;
				if (Ultrasonic_Buffer[USB]<BREAK_ASSIST_MAXIMUM_ACCEPTED_DIST)
				{
					HAL_DC_Motors_Stop() ;
				}
				else
				{
					HAL_DC_Motors_Backword() ;
				}
			}

		}
		/*string_toIint(Motor_Speed) ;
		string_toIint(servo_angle) ;*/
		/* handle UART receive */
		l_retval = HAL_UART_Receive(&huart4 ,&data,1,100 ) ;
		if (l_retval==HAL_OK)
		{
			switch (data)
			{
				case MSG_START_ENGINE :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					engine_state ^= 0x01 ;
					Car_EngineFunc() ;
				break ;

				case MSG_LEFT_SIGNAL :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					transition = SIG_LEFT ;
				break ;

				case MSG_RIGHT_SIGNAL :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					transition = SIG_RIGHT ;
				break ;

				case MSG_WAITING_SIGNAL :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					transition = SIG_W8 ;
				break ;

				case MSG_FLASHING_LIGHT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(flashing_smphr) ;
				break ;

				case MSG_HORN_BEEPING :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(horn_smphr) ;
				break ;

				case MSG_GEAR_CHANGE :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(Gear_smphr) ;
				break ;

				case MSG_GO_FWD :
					HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(speedInc_smphr) ;
				break ;

				case MSG_BRAKE_BUTTON :
					HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_RESET);
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(brake_smphr) ;
				break ;

				case MSG_GO_RIGHT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(goRight_smphr) ;
				break ;

				case MSG_GO_LEFT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(goLeft_smphr) ;
				break ;

				case MSG_BRAKE_ASSIST :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(assist_smphr) ;
				break ;

				case MSG_AUTO_PARKING_RIGHT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(right_park_smphr) ;
				break ;

				case MSG_AUTO_PARKING_LEFT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(left_park_smphr) ;
				break ;

				default :
					HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
				break ;


			}

		}

		vTaskDelay(20);
	}
}


void T2_SignalsAction (void* pvarg)
{
	while(1)
	{

		switch (Current_state)
			{
				case STATE_NORMAL :
					switch (transition)
					{
						case SIG_RIGHT :
							if (ENGINE_ENABLE==engine_state)
							{
								Car_RightSignalFunc() ;
							}
							else
							{
								Car_IdleSignalFunc() ;
							}
							Current_state = STATE_RIGHT ;
							transition = SIG_NONE ;
						break ;

						case SIG_LEFT :
							if (ENGINE_ENABLE==engine_state)
							{
								Car_LeftSignalFunc() ;
							}
							else
							{
								Car_IdleSignalFunc() ;
							}

							Current_state = STATE_LEFT ;
							transition = SIG_NONE ;
						break ;

						case SIG_W8 :
							Car_WaitSignalFunc() ;
							Previous_state = STATE_NORMAL ;
							Current_state = STATE_WAIT ;
							transition = SIG_NONE ;
						break ;

						case SIG_NONE :
							Car_NoneSignalFunc() ;
						break ;
					}

					break ;

				case STATE_RIGHT :
					switch (transition)
					{
						case SIG_RIGHT :
							if (ENGINE_ENABLE==engine_state)
							{
								Car_RightSignalFunc() ;
							}
							else
							{
								Car_IdleSignalFunc() ;
							}
							Current_state = STATE_RIGHT ;
							transition = SIG_NONE ;
						break ;

						case SIG_LEFT :
							Car_IdleSignalFunc() ;
							Current_state   = 	STATE_NORMAL ;
							transition = SIG_NONE ;
						break ;

						case SIG_W8 :
							Car_WaitSignalFunc() ;
							Previous_state = STATE_RIGHT ;
							Current_state   = 	STATE_WAIT ;
							transition = SIG_NONE ;
						break ;

						case SIG_NONE :
							Car_NoneSignalFunc() ;
						break ;
					}
					break ;

				case STATE_LEFT :
					switch (transition)
					{
						case SIG_RIGHT :
							Car_IdleSignalFunc() ;
							Current_state   = 	STATE_NORMAL ;
							transition = SIG_NONE ;
						break ;

						case SIG_LEFT :
							if (ENGINE_ENABLE==engine_state)
							{
								Car_LeftSignalFunc() ;
							}
							else
							{
								Car_IdleSignalFunc() ;
							}
							Current_state = STATE_LEFT ;
							transition = SIG_NONE ;
						break ;

						case SIG_W8 :
							Car_WaitSignalFunc() ;
							Previous_state = STATE_LEFT ;
							Current_state   = 	STATE_WAIT ;
							transition = SIG_NONE ;
						break ;

						case SIG_NONE :
							Car_NoneSignalFunc() ;
						break ;
					}
					break ;

				case STATE_WAIT :
					switch (transition)
					{
						case SIG_RIGHT :
							if (STATE_LEFT==Previous_state)
							{
								Previous_state = STATE_NORMAL ;
							}
							else
							{
								Previous_state = STATE_RIGHT ;
							}
							Current_state = STATE_WAIT ;
							transition = SIG_NONE ;
						break ;

						case SIG_LEFT :
							if (STATE_RIGHT==Previous_state)
							{
								Previous_state = STATE_NORMAL ;
							}
							else
							{
								Previous_state = STATE_LEFT ;
							}

							Current_state = STATE_WAIT ;
							transition = SIG_NONE ;
						break ;

						case SIG_W8 :
							if (STATE_NORMAL==Previous_state)
							{
								Car_IdleSignalFunc() ;
								Current_state = STATE_NORMAL ;
							}
							else if (STATE_RIGHT==Previous_state)
							{
								if (ENGINE_ENABLE==engine_state)
								{
									Car_RightSignalFunc() ;
								}
								else
								{
									Car_IdleSignalFunc() ;
								}
								Current_state = STATE_RIGHT ;
							}
							else if (STATE_LEFT==Previous_state)
							{
								if (ENGINE_ENABLE==engine_state)
								{
									Car_LeftSignalFunc() ;
								}
								else
								{
									Car_IdleSignalFunc() ;
								}
								Current_state = STATE_LEFT ;
							}
							transition = SIG_NONE ;
						break ;

						case SIG_NONE :
							Car_NoneSignalFunc() ;
						break ;
					}
					break ;
			}

		vTaskDelay(250);
	}

}

void T3_WhiteFrontLightFlashing (void* pvarg)
{
	flashing_t flash_state = FLASHING_DISABLE ;
	while (1)
	{
		if (xSemaphoreTake(flashing_smphr,portMAX_DELAY))
		{
			flash_state ^= 0x01 ;
		}

		Car_FlashingFunc(flash_state) ;
	}
}

void T4_HornBeeping (void* pvarg)
{

	horn_t horn_status = HORN_DISABLE ;
	while (1)
	{
		if (xSemaphoreTake(horn_smphr,portMAX_DELAY))
		{
			horn_status ^= 0x01 ;
		}

		Car_Horn_Func(horn_status) ;

	}
}

void T5_ChangeGear (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(Gear_smphr,portMAX_DELAY))
		{
			if  (ENGINE_ENABLE==engine_state)
			{

				gear_current ++ ;

				if (GEAR_NONE==gear_current)
				{
					gear_current = GEAR_N ;
				}

				Car_Gear_Func (gear_current) ;

			}

		}

	}
}

void T6_IncSpeed (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(speedInc_smphr,portMAX_DELAY))
		{
			if  (ENGINE_ENABLE == engine_state)
			{
				if (gear_current==GEAR_D)
				{
					HAL_DC_Motors_Forward() ;
				}
				else if (gear_current==GEAR_R)
				{
					HAL_DC_Motors_Backword() ;
				}
				Motor_Speed = Motor_Speed + 5 ;
				if (Motor_Speed>100)
				{
					Motor_Speed = 100 ;
				}
				HAL_DC_Motors_Set_Speed(Motor_Speed) ;
			}
			else
			{
				Motor_Speed = SPEED_INIT_VALUE ;
			}


		}

	}
}

void T7_BrakeWheel (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(brake_smphr,portMAX_DELAY))
		{
			if  (ENGINE_ENABLE==engine_state)
			{
				if (gear_current==GEAR_D)
				{
					HAL_DC_Motors_Forward() ;
				}
				else if (gear_current==GEAR_R)
				{
					HAL_DC_Motors_Backword() ;
				}
				Motor_Speed =Motor_Speed - 5 ;

				if (Motor_Speed<SPEED_INIT_VALUE)
				{
					Motor_Speed = SPEED_INIT_VALUE ;
					HAL_DC_Motors_Stop() ;
				}

				HAL_DC_Motors_Set_Speed(Motor_Speed) ;
			}
			else
			{
				Motor_Speed = SPEED_INIT_VALUE ;
				HAL_DC_Motors_Set_Speed(Motor_Speed) ;
			}

		}

	}
}

void T8_CarMoveRight (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(goRight_smphr,portMAX_DELAY))
		{
			servo_angle += 9 ;

			if (servo_angle>45)
			{
				servo_angle = 45 ;
			}

			HAL_ServoMotor_Set_Angel(servo_angle) ;

		}
	}
}
void T9_CarMoveLeft (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(goLeft_smphr,portMAX_DELAY))
		{
			servo_angle -= 9 ;

			if (servo_angle<-45)
			{
				servo_angle = -45 ;
			}

			HAL_ServoMotor_Set_Angel(servo_angle) ;

		}
	}
}

void T10_BreakAssistRun (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(assist_smphr,portMAX_DELAY))
		{
			assist_state ^= 0x01 ;
		}
	}
}

void T11_RightParkRun (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(right_park_smphr,portMAX_DELAY))
		{
			if (LEFT_PARK_DISABLE == left_park_stat)
			{
				right_park_stat ^= 0x01 ;
				park_state ^= 0x01 ;
				HAL_ServoMotor_Set_Angel(SERVO_INIT_ANGLE) ;
				Motor_Speed = SPEED_INIT_VALUE+CAR_SPEED_TRIAL ;
				HAL_DC_Motors_Set_Speed(Motor_Speed);
				HAL_DC_Motors_Forward() ;


			}

		}
	}
}

void T12_LeftParkRun (void *pvarg)
{
	while (1)
	{
		if (xSemaphoreTake(left_park_smphr,portMAX_DELAY))
		{
			if (RIGHT_PARK_DISABLE == right_park_stat)
			{
				left_park_stat ^= 0x01 ;
				park_state ^= 0x01 ;
				HAL_ServoMotor_Set_Angel(SERVO_INIT_ANGLE) ;
				Motor_Speed = SPEED_INIT_VALUE+CAR_SPEED_TRIAL ;
				HAL_DC_Motors_Set_Speed(Motor_Speed);
				HAL_DC_Motors_Forward() ;

			}

		}

	}
}

void T13_ParkAlgorithmRun (void *pvarg)
{
	static uint8_t park_slot_flag = 0 ;
	static uint8_t servo_counter = 0 ;
	static uint8_t servo_vice_counter = 0 ;
	static uint8_t fwd_flag = 0  ;
	while (1)
	{

		if (park_state == PARK_ENABLE)
		{
			/* Right Park Procedure  */
			if (RIGHT_PARK_ENABLE==right_park_stat)
			{
				Ultrasonic_Distance(USF,&Ultrasonic_Buffer[USF]) ;
				Ultrasonic_Distance(USFR,&Ultrasonic_Buffer[USFR]) ;
				Ultrasonic_Distance(USBR,&Ultrasonic_Buffer[USBR]) ;
				if (1==park_slot_flag)
				{
					servo_counter++ ;
					if (servo_counter<SERVO_COUNTER_TRIAL)
					{

						HAL_ServoMotor_Set_Angel(45);
						Motor_Speed = SPEED_INIT_VALUE + CAR_SPEED_TRIAL + 5 ;
						HAL_DC_Motors_Set_Speed(Motor_Speed) ;
						HAL_DC_Motors_Backword() ;
						HAL_UART_Transmit(&huart3 ,"rotateR\r\n",9,50);

					}
					else
					{
						servo_vice_counter++ ;
						if ((servo_vice_counter>SERVO_VICE_COUNTER_TRIAL)||(Ultrasonic_Buffer[USF] <10))
						{
							/*Forward Right */
							HAL_DC_Motors_Set_Speed(Motor_Speed);
							HAL_DC_Motors_Forward() ;
							HAL_ServoMotor_Set_Angel(30);
							HAL_UART_Transmit(&huart3 ,"FWDR\r\n",6,50);
							vTaskDelay(500);


							HAL_UART_Transmit(&huart3 ,"Finish\r\n",8,50);
							park_slot_flag = 0 ;
							right_park_stat ^= 0x01 ;
							park_state ^= 0x01 ;
							HAL_ServoMotor_Set_Angel(0) ;
							HAL_DC_Motors_Stop() ;
							Motor_Speed = SPEED_INIT_VALUE ;
							HAL_DC_Motors_Set_Speed(Motor_Speed) ;


						}
						else
						{
							Motor_Speed = SPEED_INIT_VALUE+CAR_SPEED_TRIAL  ;
							HAL_DC_Motors_Set_Speed(Motor_Speed);
							HAL_DC_Motors_Backword() ;
							HAL_UART_Transmit(&huart3 ,"rotateL\r\n",9,50);
							HAL_ServoMotor_Set_Angel(-45);

						}

					}

				}
				else if (Ultrasonic_Buffer[USF] >BREAK_ASSIST_MAXIMUM_ACCEPTED_DIST)
				{
					HAL_DC_Motors_Set_Speed(SPEED_INIT_VALUE+CAR_SPEED_TRIAL);

					HAL_UART_Transmit(&huart3 ,"USFR:",5,50);
					string_toIint(Ultrasonic_Buffer[USFR]) ;
					HAL_UART_Transmit(&huart3 ,"\r\n",2,50);

					HAL_UART_Transmit(&huart3 ,"USBR:",5,50);
					string_toIint(Ultrasonic_Buffer[USBR]) ;
					HAL_UART_Transmit(&huart3 ,"\r\n",2,50);

					if ((Ultrasonic_Buffer[USFR]>CAR_ACCEPTED_PARK_DIST_WIDTH)&&(Ultrasonic_Buffer[USBR]>CAR_ACCEPTED_PARK_DIST_WIDTH))
					{
						vTaskDelay(TIME_OUT_COUNTER) ;
						HAL_DC_Motors_Stop() ;
						vTaskDelay(STOPPAGE_TIME) ;

						park_slot_flag = 1 ;

						HAL_UART_Transmit(&huart3 ,"Slot founded\r\n",14,50);
					}
				}
				else
				{
					HAL_DC_Motors_Stop() ;
				}
			}



			/* Left Park Procedure  */
			else if (LEFT_PARK_ENABLE==left_park_stat)
			{

			}
		}
		vTaskDelay(50);


	}
}

void T14_GetUltrasonicRead (void *pvarg)
{
	while (1)
	{
		Ultrasonic_Distance(current_ultrasonic ,&Ultrasonic_Buffer[current_ultrasonic] );
		current_ultrasonic++ ;
		if (current_ultrasonic == 6 )
		{
			current_ultrasonic = USF ;

			for (uint8_t index = 0 ; index < 6 ; index++)
			{
				Important_Buffer[index] = Ultrasonic_Buffer[index] ;
			}
		}
		vTaskDelay(50) ;
	}
}

void T15_GSMSendCurrentLocation (void *pvarg)
{
	uint8_t GPS_Buffer[GPS_BUFFER_LENGTH] ;
	uint8_t Lattitude_buffer[LATTITUDE_BUFFER_LENGTH];
	uint8_t Longtittude_buffer[LONGTITUDE_BUFFER_LENGTH];
	HAL_StatusTypeDef l_retval = HAL_OK ;
	uint8_t index  ;
	uint8_t lat_index  ;
	uint8_t long_index  ;



	while (1)
	{
		long_index = 0 ;
		lat_index = 0 ;
		index = LATITTUDE_INDEX_START ;

		l_retval = HAL_UART_Receive(&huart3 ,GPS_Buffer,GPS_BUFFER_LENGTH,HAL_MAX_DELAY ) ;

		while ((GPS_Buffer[index]!='.')&&(GPS_Buffer[index]!=','))
		{
			if ((GPS_Buffer[index]>='0')&&(GPS_Buffer[index]<='9'))
			{
				Lattitude_buffer[lat_index]  = GPS_Buffer[LATITTUDE_INDEX_START+index] ;
				lat_index++ ;
			}
			index++ ;


		}

		index = LONGTITUDE_INDEX_START ;


		while ((GPS_Buffer[index]!='.')&&(GPS_Buffer[index]!=','))
		{
			if ((GPS_Buffer[index]>='0')&&(GPS_Buffer[index]<='9'))
			{
				Lattitude_buffer[lat_index]  = GPS_Buffer[LONGTITUDE_INDEX_START+index] ;
				long_index++ ;
			}
			index++ ;
		}

	}
}



/*********************************** static function Implementation *********************************/
static void Car_EngineFunc(void)
{
	if (ENGINE_ENABLE==engine_state)
	{
		/* turn on engine indication led */
		HAL_GPIO_WritePin(LEDS_START_ENGINE_PIN,GPIO_PIN_RESET);
	}
	else if (ENGINE_DISABLE==engine_state)
	{
		/* turn off engine indication led */
		HAL_GPIO_WritePin(LEDS_START_ENGINE_PIN,GPIO_PIN_SET);
	}
	else
	{
		/* Nothing */
	}
}

static void Car_RightSignalFunc(void)
{
	HAL_GPIO_TogglePin(LEDS_YELLOW_RIGHT_SIDE) ;
	HAL_GPIO_WritePin(LEDS_YELLOW_LEFT_SIDE,GPIO_PIN_SET);
}

static void Car_LeftSignalFunc(void)
{
	HAL_GPIO_TogglePin(LEDS_YELLOW_LEFT_SIDE) ;
	HAL_GPIO_WritePin(LEDS_YELLOW_RIGHT_SIDE,GPIO_PIN_SET);
}

static void Car_WaitSignalFunc(void)
{
	if (0==sync_flag)
	{
		HAL_GPIO_WritePin(LEDS_YELLOW_RIGHT_SIDE,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDS_YELLOW_LEFT_SIDE,GPIO_PIN_SET);
		sync_flag = 1 ;
	}
	/*Toggle Both Right and left signal leds */
	HAL_GPIO_TogglePin(LEDS_YELLOW_RIGHT_SIDE) ;
	HAL_GPIO_TogglePin(LEDS_YELLOW_LEFT_SIDE) ;
}

static void Car_IdleSignalFunc(void)
{
	/*Turn off Right yellow signal leds*/
	HAL_GPIO_WritePin(LEDS_YELLOW_RIGHT_SIDE,GPIO_PIN_SET);
	/*Turn on left yellow signal leds*/
	HAL_GPIO_WritePin(LEDS_YELLOW_LEFT_SIDE,GPIO_PIN_SET);
}

static void Car_NoneSignalFunc(void)
{
	switch (Current_state)
	{
		case STATE_NORMAL :
			Car_IdleSignalFunc() ;
			sync_flag = 0 ;
		break ;

		case STATE_RIGHT :
			if (ENGINE_ENABLE==engine_state)
			{
				Car_RightSignalFunc() ;
			}
			else
			{
				Car_IdleSignalFunc() ;
			}
			sync_flag = 0 ;
		break ;

		case SIG_LEFT :
			if (ENGINE_ENABLE==engine_state)
			{
				Car_LeftSignalFunc() ;
			}
			else
			{
				Car_IdleSignalFunc() ;
			}
			sync_flag = 0 ;
		break ;

		case SIG_W8 :
			Car_WaitSignalFunc() ;
		break ;
	}
}


static void Car_FlashingFunc (flashing_t state)
{
	if (FLASHING_DISABLE==state)
	{
		HAL_GPIO_WritePin(LEDS_FRONT_WHITE_FLASHING , GPIO_PIN_SET ) ;
	}
	else if(FLASHING_ENABLE==state)
	{
		HAL_GPIO_WritePin(LEDS_FRONT_WHITE_FLASHING , GPIO_PIN_RESET ) ;
	}
	else
	{
		/* Nothing */
	}
}


static void Car_Gear_Func (gear_t state)
{
	HAL_DC_Motors_Stop() ;
	Motor_Speed = SPEED_INIT_VALUE ;
	switch (state)
	{
		case GEAR_N :
			HAL_DC_Motors_Stop() ;
			HAL_GPIO_WritePin(LEDS_BACK_WHITE_PARK,GPIO_PIN_SET);
		break ;

		case GEAR_D :
			HAL_DC_Motors_Forward() ;
			HAL_GPIO_WritePin(LEDS_BACK_WHITE_PARK,GPIO_PIN_SET);
		break ;

		case GEAR_R :
			HAL_DC_Motors_Backword() ;
			HAL_GPIO_WritePin(LEDS_BACK_WHITE_PARK,GPIO_PIN_RESET);
		break ;

		default :
			HAL_DC_Motors_Stop() ;
		break ;

	}
}


static void Car_Horn_Func (horn_t state)
{
	if (HORN_DISABLE==state)
	{
		HAL_GPIO_WritePin(BUZZER_PIN , GPIO_PIN_SET ) ;
	}
	else if(HORN_ENABLE==state)
	{
		HAL_GPIO_WritePin(BUZZER_PIN , GPIO_PIN_RESET ) ;
	}
	else
	{
		/* Nothing */
	}
}



