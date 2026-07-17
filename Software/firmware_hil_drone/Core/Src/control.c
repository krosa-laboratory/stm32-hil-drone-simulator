/*
 * control.c
 *
 *  Created on: 15 jul 2026
 *      Author: kevin
 */

#include "control.h"
#include "pid.h"
#include "mixer.h"
#include "hardware.h"
#include "config.h"
#include "physics.h"

// PIDs instances
PID_Controller_t pid_z;
PID_Controller_t pid_roll;
PID_Controller_t pid_pitch;
PID_Controller_t pid_yaw;

// Intermediate state variables
extern float actual_roll, actual_pitch, actual_yaw, actual_z;
extern float desired_roll, desired_pitch, desired_yaw, desired_z;

// Local variables of forces
float U1 = 0.0f; // Total throttle
float U2 = 0.0f; // Roll force
float U3 = 0.0f; // Pitch force
float U4 = 0.0f; // Yaw force

void Control_Init(void)
{

	PID_Init(&pid_z, 2.5f, 0.1f, 1.0f, 10.0f);
	PID_Init(&pid_roll, 1.2f, 0.0f, 0.3f, 5.0f);
	PID_Init(&pid_pitch, 1.2f, 0.0f, 0.3f, 5.0f);
	PID_Init(&pid_yaw, 2.0f, 0.0f, 0.0f, 5.0f);

}

void TIM6_DAC_IRQHandler(void)
{

	// Check the interruption flag
	if(TIM6->SR & TIM_SR_UIF)
	{

		// Clear the flag
		TIM6->SR &= ~TIM_SR_UIF;

		static uint32_t tick = 0;
		tick ++;

		// Update state (SIMULATED)
		Physics_Update(TIM6_DT, U1, U2, U3, U4);

		// External loop; Navigation (100Hz -> each 10 ms)
		if(tick % 10 == 0) U1 = PID_Compute(&pid_z, desired_z, actual_z, 0.01f);

		// Internal loop; Actitude (1000Hz -> each 1 ms)
		U2 = PID_Compute(&pid_roll,  desired_roll,  actual_roll,  TIM6_DT);
		U3 = PID_Compute(&pid_pitch, desired_pitch, actual_pitch, TIM6_DT);
		U4 = PID_Compute(&pid_yaw,   desired_yaw,   actual_yaw,   TIM6_DT);

		// Mixer to transform forces to PWM
		Mixer_Compute(U1, U2, U3, U4);

	}

}
