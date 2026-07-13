/*
 * hardware.h
 *
 *  Created on: 12 jul 2026
 *      Author: kevin
 */

#ifndef INC_HARDWARE_H_
#define INC_HARDWARE_H_

#include "stm32g4xx.h"

void Hardware_Init(void);
void TIM2_Init(void);
void TIM6_Init(void);
void Hardware_SetMotors(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4);

#endif /* INC_HARDWARE_H_ */
