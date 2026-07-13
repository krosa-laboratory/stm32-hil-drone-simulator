/*
 * mixer.c
 *
 *  Created on: 13 jul 2026
 *      Author: kevin
 */

#include "mixer.h"
#include "config.h"
#include "hardware.h"

void Mixer_Compute(float U1, float U2, float U3, float U4)
{

	// Extract the dynamic constant from the config
	float k = config.k_pwm;
	// Convert U1 to PWM base, we assume U1 = 0 is the iddle motor rotation defined in configuration
	float base_throttle = PWM_MIN_US + (U1 * k);
	// Apply Quad-X geometrics expressions
	float m1 = base_throttle + (U2 * k) - (U3 * k) - (U4 + k);
	float m2 = base_throttle - (U2 * k) - (U3 * k) + (U4 + k);
	float m3 = base_throttle - (U2 * k) + (U3 * k) - (U4 + k);
	float m4 = base_throttle + (U2 * k) + (U3 * k) + (U4 + k);
	// Strict security saturations using config macros
	if(m1 < PWM_MIN_US) m1 = PWM_MIN_US; else if(m1 > PWM_MAX_US) m1 = PWM_MAX_US;
	if(m2 < PWM_MIN_US) m2 = PWM_MIN_US; else if(m2 > PWM_MAX_US) m2 = PWM_MAX_US;
	if(m3 < PWM_MIN_US) m3 = PWM_MIN_US; else if(m3 > PWM_MAX_US) m3 = PWM_MAX_US;
	if(m4 < PWM_MIN_US) m4 = PWM_MIN_US; else if(m4 > PWM_MAX_US) m4 = PWM_MAX_US;
	// Inyect values directly to the TIM2 registers
	Hardware_SetMotors((uint32_t)m1, (uint32_t)m2, (uint32_t)m3, (uint32_t)m4);

}

