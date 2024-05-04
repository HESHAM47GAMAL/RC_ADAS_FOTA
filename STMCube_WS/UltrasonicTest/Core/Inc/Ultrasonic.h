/*
 * Ultrasonic.h
 *
 *  Created on: May 4, 2024
 *      Author: moham
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_



#include "stm32f4xx_hal.h"


#define TRIG_PIN_1			GPIO_PIN_9
#define TRIG_PORT_1			GPIOA

uint16_t  Ultrasonic_Distance (void);

#endif /* INC_ULTRASONIC_H_ */
