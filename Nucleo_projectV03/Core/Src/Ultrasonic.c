/*
 * Ultrasonic.c
 *
 *  Created on: May 4, 2024
 *      Author: moham
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "Ultrasonic.h"





extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;




void UltrasonicInit(void)
{
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}


void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

volatile uint32_t g_TimeOfEcho ;
volatile uint8_t g_edgeDetect = 0;  // is the first value captured ?



// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM1) {
		// This interrupt is from TIM1

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			g_edgeDetect++;
			if (g_edgeDetect==1) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
			}

			else if (g_edgeDetect==2)   // if the first is already captured
			{
				g_TimeOfEcho = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  // read second value
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);


			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
		{
			g_edgeDetect++;
			if (g_edgeDetect==1) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
			}

			else if (g_edgeDetect==2)   // if the first is already captured
			{
				g_TimeOfEcho = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);  // read second value
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);


			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
		{
			g_edgeDetect++;
			if (g_edgeDetect==1) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);

				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
			}

			else if (g_edgeDetect==2)   // if the first is already captured
			{
				g_TimeOfEcho = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);  // read second value
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);


			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel1
		{
			g_edgeDetect++;
			if (g_edgeDetect==1) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);

				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
			}

			else if (g_edgeDetect==2)   // if the first is already captured
			{
				g_TimeOfEcho = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);  // read second value
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);


			}
		}


	}
	else if (htim->Instance == TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			g_edgeDetect++;
			if (g_edgeDetect==1) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim2, 0);  // reset the counter

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

				__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
			}

			else if (g_edgeDetect==2)   // if the first is already captured
			{
				g_TimeOfEcho = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);  // read second value
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);


			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
		{
			g_edgeDetect++;
			if (g_edgeDetect==1) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim2, 0);  // reset the counter

				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

				__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
			}

			else if (g_edgeDetect==2)   // if the first is already captured
			{
				g_TimeOfEcho = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);  // read second value
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);


			}
		}
	}
}


void UltrasonicTrigger(ULTRASONIC_t Value)
{
	switch(Value)
	{
	case USF:
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
		HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin low

		break;

	case USB:
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
		HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);  // pull the TRIG pin low

		break;

	case USFR:
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
		HAL_GPIO_WritePin(TRIG_PORT_3, TRIG_PIN_3, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		HAL_GPIO_WritePin(TRIG_PORT_3, TRIG_PIN_3, GPIO_PIN_RESET);  // pull the TRIG pin low

		break;

	case USBR:
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
		HAL_GPIO_WritePin(TRIG_PORT_4, TRIG_PIN_4, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		HAL_GPIO_WritePin(TRIG_PORT_4, TRIG_PIN_4, GPIO_PIN_RESET);  // pull the TRIG pin low

		break;

	case USFL:
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
		HAL_GPIO_WritePin(TRIG_PORT_5, TRIG_PIN_5, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_GPIO_WritePin(TRIG_PORT_5, TRIG_PIN_5, GPIO_PIN_RESET);  // pull the TRIG pin low

		break;

	case USBL:
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
		HAL_GPIO_WritePin(TRIG_PORT_6, TRIG_PIN_6, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		delay(10);  // wait for 10 us
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_GPIO_WritePin(TRIG_PORT_6, TRIG_PIN_6, GPIO_PIN_RESET);  // pull the TRIG pin low

		break;
	}
}



void Ultrasonic_Distance(ULTRASONIC_t Value , uint16_t * DisUltrasonic)
{

	uint8_t _2_edgeCatched = 1 ;

	UltrasonicTrigger(Value);

	/*  wait until catch riging and failing edge for Echo */
	if((Value <= USBR) && (Value >= USF))
	{
		while(g_edgeDetect != 2)
		{
			if(__HAL_TIM_GET_COUNTER(&htim1) > 20000)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 0);
				_2_edgeCatched = 0 ;
				g_edgeDetect = 0 ;
				switch(Value)
				{
				case USF :
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
					break ;
				case USB :
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
					break ;
				case USFR :
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
					break ;
				case USBR :
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
					break ;
				}
				break;
			}

		}
	}
	else if((Value == USBL) || (Value == USFL))
	{
		while(g_edgeDetect != 2)
		{
			if(__HAL_TIM_GET_COUNTER(&htim2) > 20000)
			{
				__HAL_TIM_SET_COUNTER(&htim2, 0);
				_2_edgeCatched = 0 ;
				g_edgeDetect = 0 ;
				switch(Value)
				{
				case USFL :
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
					break ;
				case USBL :
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
					break ;
				}

				break;
			}

		}
	}


	/*  Make it equal zero to be able to perform same operation*/
    g_edgeDetect = 0 ;
    if(_2_edgeCatched == 1)
    {
    	*DisUltrasonic = (uint32_t)g_TimeOfEcho / 57.5;
    }
    else
    {
    	*DisUltrasonic = 400;
    }




}


