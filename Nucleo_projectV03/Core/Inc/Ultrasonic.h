/*
 * Ultrasonic.h
 *
 *  Created on: May 4, 2024
 *      Author: moham
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_



#include "stm32f4xx_hal.h"


#define TRIG_PIN_1			GPIO_PIN_0
#define TRIG_PORT_1			GPIOC


#define TRIG_PIN_2			GPIO_PIN_1
#define TRIG_PORT_2			GPIOC


#define TRIG_PIN_3			GPIO_PIN_2
#define TRIG_PORT_3			GPIOC

#define TRIG_PIN_4			GPIO_PIN_3
#define TRIG_PORT_4			GPIOC

#define TRIG_PIN_5			GPIO_PIN_7
#define TRIG_PORT_5			GPIOA

#define TRIG_PIN_6			GPIO_PIN_4
#define TRIG_PORT_6			GPIOC


typedef enum
{
	USF	 = 0,
	USB,
	USFR,
	USBR,
	USFL,
	USBL
	}ULTRASONIC_t;

void UltrasonicInit(void);

void UltrasonicTrigger(ULTRASONIC_t Value);

void Ultrasonic_Distance(ULTRASONIC_t Value , uint16_t * DisUltrasonic);


#endif /* INC_ULTRASONIC_H_ */
