
/***********************************************************
 *Module Name: Motor_Driver.h
 *Module Desc: Init Module
 *Author: Omar Ahmed
 ***************************************************************/

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

/*********************************************
 * 		Macros For Motor Pins and Port		*
 **********************************************/

#define HAL_DC_Motor_Direc_1		GPIOB , GPIO_PIN_8
#define HAL_DC_Motor_Direc_2	    GPIOB , GPIO_PIN_9

#define FWD_DC_Motor_Direc_1		GPIOB , GPIO_PIN_4
#define FWD_DC_Motor_Direc_2	    GPIOB , GPIO_PIN_5

#define HAL_DC_Motor_Timer				htim4
#define HAL_Servo_Motor_Timer			htim3

#define HAL_DC_Motor_Channel 			TIM_CHANNEL_1
#define HAL_Servo_Motor_Channel			TIM_CHANNEL_1

/*******************************************
 * Macros For Motor Speed
 *******************************************/
#define Maxium_ServoMotor_Angle  180
#define Minium_ServoMotor_Angle  0


#define Maxium_DC_Motor_Speed_Percentage  100
#define Minium_DC_Motor_Speed_Percentage  50

/*******************************************************
 * Motors prototypes
 ******************************************************/
void HAL_DC_Motors_Forward(void );
void HAL_DC_Motors_Backword(void );
void HAL_DC_Motors_Stop (void );
void HAL_DC_Motor_Init (void );
void HAL_DC_Motors_Set_Speed (uint8_t Copy_DC_MotorSpeed_percentage);
void HAL_ServoMotor_Init (void);
void HAL_ServoMotor_Set_Angel (int8_t Copy_ServoMotor_Angel);
void HAL_DC_Motor_FWD_Right (void);
void HAL_DC_Motor_FWD_Left (void);
void HAL_DC_Motor_FWD_Return_pos (void);
#endif /* INC_MOTOR_DRIVER_H_ */
