/*
 * config.h
 *
 *  Created on: 13 jul 2026
 *      Author: kevin
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define PWM_MIN_US   1000.0f
#define PWM_MAX_US   2000.0f
#define TIM6_LOOP_HZ 1000.0f
#define TIM6_DT      (1.0f / TIM6_LOOP_HZ)

// Flight params (Adjustables in real time)
typedef struct {
	float weight_kg;	 // Total Drone weight in kilograms
	float xx_intertia;	 // Roll moment of inertia
	float yy_inertia;	 // Pitch moment of inertia
	float zz_inertia;	 // Yaw moment of inertia
	float k_pwm;		 // Conversion factor Newtons -> Ticks PWM
	float max_angle_rad; // Maximum tilt angle permitted for safety reasons
} DroneConfig_t;

// Global variable with the active configuration
extern DronConfig_t config;

#endif /* INC_CONFIG_H_ */
