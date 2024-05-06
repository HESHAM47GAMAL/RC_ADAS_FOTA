/************************************************************************************************/
/*************  		Author : Eslam Elwey       **********************************************/
/*************  		Date : 28/4/2024           **********************************************/
/*************  		File : RC_Car_App_Int.h    **********************************************/
/************************************************************************************************/

#ifndef RC_CAR_APP_INT_H
#define RC_CAR_APP_INT_H

/******************* Macros For Leds And Buzzer *********************************/
											/*  Port    ,  Pin Number*/
#define LEDS_YELLOW_RIGHT_SIDE					GPIOB   ,  GPIO_PIN_15
#define LEDS_YELLOW_LEFT_SIDE					GPIOB   ,  GPIO_PIN_14
#define LEDS_FRONT_WHITE_FLASHING				GPIOB   ,  GPIO_PIN_12
#define LEDS_BACK_WHITE_PARK					GPIOB   ,  GPIO_PIN_0
#define LEDS_BACK_RED_BRAKE						GPIOB   ,  GPIO_PIN_1
#define BUZZER_PIN								GPIOB   ,  GPIO_PIN_2
#define LEDS_START_ENGINE_PIN					GPIOB   ,  GPIO_PIN_13

/**************   uart receive Messsage ***********************/

#define MSG_START_ENGINE						'E'
#define MSG_LEFT_SIGNAL							'L'
#define MSG_RIGHT_SIGNAL						'R'
#define MSG_WAITING_SIGNAL						'W'
#define MSG_HORN_BEEPING						'H'
#define MSG_GEAR_CHANGE							'G'
#define MSG_GO_FWD								'f'
#define MSG_GO_RIGHT							'r'
#define MSG_GO_LEFT								'l'
#define MSG_BRAKE_BUTTON						'B'
#define MSG_FLASHING_LIGHT						'F'
#define MSG_AUTO_PARKING_RIGHT					'D'
#define MSG_AUTO_PARKING_LEFT					'A'
#define MSG_BRAKE_ASSIST						'S'

/********************************************************************************/
#define SPEED_INIT_VALUE						55
#define SERVO_INIT_ANGLE						0

/********************************************************************************/


/*state type */
typedef enum
{
	STATE_NORMAL ,
	STATE_RIGHT ,
	STATE_LEFT ,
	STATE_WAIT
}state_t;

/*transition type */
typedef enum
{
	SIG_NONE ,
	SIG_RIGHT ,
	SIG_LEFT ,
	SIG_W8
}transition_t;


/*Gear type */
typedef enum
{
	GEAR_N = 0  ,
	GEAR_D ,
	GEAR_R ,
	GEAR_NONE
}gear_t;

/*Engine type */
typedef enum
{
	ENGINE_ENABLE ,
	ENGINE_DISABLE
}engine_t;

/*flashing type */
typedef enum
{
	FLASHING_ENABLE ,
	FLASHING_DISABLE
}flashing_t;

/*Horn type */
typedef enum
{
	HORN_ENABLE ,
	HORN_DISABLE
}horn_t;






/************************ Global Functions *****************************************/

void RC_SystemInit (void) ;
void T1_UartReceive (void* pvarg) ;
void T2_SignalsAction (void* pvarg) ;
void T3_WhiteFrontLightFlashing (void* pvarg) ;
void T4_HornBeeping (void* pvarg) ;
void T5_ChangeGear (void *pvarg) ;
void T6_IncSpeed (void *pvarg) ;
void T7_BrakeWheel (void *pvarg) ;
void T8_CarMoveRight (void *pvarg) ;
void T9_CarMoveLeft (void *pvarg) ;

#endif //RC_CAR_APP_INT_H
