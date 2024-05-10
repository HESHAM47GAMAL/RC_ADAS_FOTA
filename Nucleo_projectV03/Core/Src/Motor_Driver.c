/***********************************************************
 *Module Name: Motor_Driver.h
 *Module Desc: Init Module
 *Author: Omar Ahmed
 ***************************************************************/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "tim.h"
#include "stm32f4xx_hal_tim.h"
#include "Motor_Driver.h"


/************************************************************
 *Function Name : HAL_DC_Motors_Forward
 *Description 	: Rotate the DC Motors forward
 *
 ****************************************************/
void HAL_DC_Motors_Forward(void ){
	//----IN1 = H
	//----IN2 = L
	HAL_GPIO_WritePin(HAL_DC_Motor_Direc_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HAL_DC_Motor_Direc_2, GPIO_PIN_SET);

}
/**************************************************************
 *Function Name : HAL_DC_Motors_Backword
 *Description 	: Rotate the DC Motors forward
 *
 ****************************************************/
void HAL_DC_Motors_Backword(void ){
	//----IN1 = H
	//----IN2 = L
	HAL_GPIO_WritePin(HAL_DC_Motor_Direc_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HAL_DC_Motor_Direc_2, GPIO_PIN_RESET);

}
void HAL_DC_Motors_Stop (void ){

	HAL_GPIO_WritePin(HAL_DC_Motor_Direc_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HAL_DC_Motor_Direc_2, GPIO_PIN_SET);

}
void HAL_DC_Motor_Init (void ){
	HAL_TIM_Base_Start(&HAL_DC_Motor_Timer);
	HAL_TIM_PWM_Init(&HAL_DC_Motor_Timer);

	HAL_TIM_PWM_Start(&HAL_DC_Motor_Timer, HAL_DC_Motor_Channel);
}
void HAL_DC_Motors_Set_Speed (uint8_t Copy_DC_MotorSpeed_percentage){ // Percent


	__HAL_TIM_SET_COMPARE(&HAL_DC_Motor_Timer,HAL_DC_Motor_Channel, Copy_DC_MotorSpeed_percentage);





}

void HAL_ServoMotor_Init (void){

	HAL_TIM_PWM_Init(&HAL_Servo_Motor_Timer);

}

void HAL_ServoMotor_Set_Angel (int8_t Copy_ServoMotor_Angel){


	uint16_t  Campare_value;
	if (Copy_ServoMotor_Angel < 0 ){

		Campare_value = ((((int32_t)50*Copy_ServoMotor_Angel)/9) + 1000);

		__HAL_TIM_SET_COMPARE(&HAL_Servo_Motor_Timer,HAL_Servo_Motor_Channel, Campare_value);

		HAL_TIM_PWM_Start(&HAL_Servo_Motor_Timer, HAL_Servo_Motor_Channel);

	}

	else if (Copy_ServoMotor_Angel > 0){
		Campare_value = ((((int32_t)70*Copy_ServoMotor_Angel)/9) + 1000);

		__HAL_TIM_SET_COMPARE(&HAL_Servo_Motor_Timer,HAL_Servo_Motor_Channel, Campare_value);

		HAL_TIM_PWM_Start(&HAL_Servo_Motor_Timer, HAL_Servo_Motor_Channel);

	}

	else if (Copy_ServoMotor_Angel == 0)
	{


		__HAL_TIM_SET_COMPARE(&HAL_Servo_Motor_Timer,HAL_Servo_Motor_Channel, 1000);

		HAL_TIM_PWM_Start(&HAL_Servo_Motor_Timer, HAL_Servo_Motor_Channel);
	}



}

void HAL_DC_Motor_FWD_Right ()
{


	HAL_GPIO_WritePin(FWD_DC_Motor_Direc_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FWD_DC_Motor_Direc_2, GPIO_PIN_SET);


}

void HAL_DC_Motor_FWD_Left (void) {


	HAL_GPIO_WritePin(FWD_DC_Motor_Direc_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FWD_DC_Motor_Direc_2, GPIO_PIN_RESET);


}

void HAL_DC_Motor_FWD_Return_pos (void) {


	HAL_GPIO_WritePin(FWD_DC_Motor_Direc_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FWD_DC_Motor_Direc_2, GPIO_PIN_RESET);

}
