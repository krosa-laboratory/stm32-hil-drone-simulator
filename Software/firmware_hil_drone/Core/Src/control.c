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
#include "state.h"
#include "physics.h"

// PIDs instances
PID_Controller_t pid_x;
PID_Controller_t pid_y;
PID_Controller_t pid_z;
PID_Controller_t pid_roll;
PID_Controller_t pid_pitch;
PID_Controller_t pid_yaw;

// Local variables of forces
float real_U1   = 0.0f; // To simulate delay from motors
float desire_U1 = 0.0f;
float U2 = 0.0f; // Roll force
float U3 = 0.0f; // Pitch force
float U4 = 0.0f; // Yaw force

void Control_Init(void)
{

	PID_Init(&pid_x,     0.4f, 0.0f, 0.4f, config.max_angle_rad);
	PID_Init(&pid_y,     0.4f, 0.0f, 0.4f, config.max_angle_rad);
	PID_Init(&pid_z,     2.5f, 0.1f, 1.0f, 10.0f);
	PID_Init(&pid_roll,  1.2f, 0.0f, 0.3f, 5.0f );
	PID_Init(&pid_pitch, 1.2f, 0.0f, 0.3f, 5.0f );
	PID_Init(&pid_yaw,   2.0f, 0.0f, 0.0f, 5.0f );

	real_U1 = config.weight_kg * 9.81f; // Initial force needed

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
		Physics_Update(TIM6_DT, real_U1, U2, U3, U4, &actual_state);

		// External loop; Navigation (100Hz -> each 10 ms)
		if(tick % NAV_DIVIDER == 0)
		{
			// Z Navigation === Altitude
			float dt_nav = NAV_DT;
			// Calculate desire pushing force
			float force_pid_z = PID_Compute(&pid_z, desire_state.z, actual_state.z, NAV_DT);
			desire_U1 = (config.weight_kg * 9.81f) + force_pid_z;
			// Upper limit for motors
			float max_U1 = 3.0f * config.weight_kg * 9.81f;
			if(desire_U1 < 0.0f) desire_U1 = 0.0f;
			else if(desire_U1 > max_U1) desire_U1 = max_U1;
			// X Navigation
			desire_state.pitch = PID_Compute(&pid_x, desire_state.x, actual_state.x, NAV_DT);
			// Y Navigation
			desire_state.pitch = PID_Compute(&pid_y, desire_state.y, actual_state.y, NAV_DT);
		}

		// Motors delay
		float alpha = TIM6_DT / (0.05f + TIM6_DT);
		real_U1 += alpha * (desire_U1 - real_U1);

		// Internal loop; Actitude (1000Hz -> each 1 ms)
		U2 = PID_Compute(&pid_roll,  desire_state.roll,  actual_state.roll,  TIM6_DT);
		U3 = PID_Compute(&pid_pitch, desire_state.pitch, actual_state.pitch, TIM6_DT);
		U4 = PID_Compute(&pid_yaw,   desire_state.yaw,   actual_state.yaw,   TIM6_DT);

		// Mixer to transform forces to PWM
		Mixer_Compute(real_U1, U2, U3, U4);

	}

}
