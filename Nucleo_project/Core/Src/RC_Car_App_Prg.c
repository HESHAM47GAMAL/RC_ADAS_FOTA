
/************************************************************************************************/
/*************  		Author : Eslam Elwey       **********************************************/
/*************  		Date : 28/4/2024           **********************************************/
/*************  		File : RC_Car_App_Prg.c    **********************************************/
/************************************************************************************************/



/******************************** Includes ********************************************/
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
static SemaphoreHandle_t 			flashing_smphr  =  NULL ;
static SemaphoreHandle_t 			horn_smphr  	=  NULL ;
static SemaphoreHandle_t			Gear_smphr		=  NULL ;
static SemaphoreHandle_t			speedInc_smphr	=  NULL ;
static SemaphoreHandle_t			brake_smphr	    =  NULL ;
static SemaphoreHandle_t			goRight_smphr   =  NULL ;
static SemaphoreHandle_t			goLeft_smphr	=  NULL ;
static gear_t 						gear_current    =  GEAR_N  ;
static int8_t						Motor_Speed		= SPEED_INIT_VALUE ;
static int8_t						servo_angle		= SERVO_INIT_ANGLE ;

/********************************* RTOS Tasks Implementation *************************************/

void RC_SystemInit (void)
{
	flashing_smphr =  xSemaphoreCreateBinary() ;
	horn_smphr     =  xSemaphoreCreateBinary() ;
	Gear_smphr     =  xSemaphoreCreateBinary() ;
	speedInc_smphr =  xSemaphoreCreateBinary() ;
	brake_smphr    =  xSemaphoreCreateBinary() ;
	goRight_smphr  =  xSemaphoreCreateBinary() ;
	goLeft_smphr   =  xSemaphoreCreateBinary() ;

	HAL_DC_Motor_Init() ;
	HAL_DC_Motors_Stop() ;
	HAL_DC_Motors_Set_Speed(0) ;
	HAL_ServoMotor_Init() ;
	HAL_ServoMotor_Set_Angel(0) ;

	HAL_GPIO_WritePin(LEDS_START_ENGINE_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_YELLOW_RIGHT_SIDE,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_YELLOW_LEFT_SIDE,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_FRONT_WHITE_FLASHING,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_BACK_WHITE_PARK,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BUZZER_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);

}


void T1_UartReceive (void* pvarg)
{
	HAL_StatusTypeDef l_retval = HAL_OK ;
	uint8_t data ;

	while (1)
	{
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
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(speedInc_smphr) ;
				break ;

				case MSG_BRAKE_ASSIST :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(brake_smphr) ;
				break ;

				case MSG_GO_RIGHT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(goRight_smphr) ;
				break ;

				case MSG_GO_LEFT :
					HAL_UART_Transmit(&huart4 ,&data,1,50);
					xSemaphoreGive(MSG_GO_LEFT) ;
				break ;

			}

		}

		vTaskDelay(50);
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
				Motor_Speed+= 5 ;
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
				Motor_Speed-= 5 ;

				if (Motor_Speed<0)
				{
					Motor_Speed = 0 ;
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
			HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
		break ;

		case GEAR_D :
			HAL_DC_Motors_Forward() ;
			HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_SET);
		break ;

		case GEAR_R :
			HAL_DC_Motors_Backword() ;
			HAL_GPIO_WritePin(LEDS_BACK_RED_BRAKE,GPIO_PIN_RESET);
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
