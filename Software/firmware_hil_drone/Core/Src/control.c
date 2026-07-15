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
//#include "physics.h"

// PIDs instances
PID_Controller_t pid_z;
PID_Controller_t pid_roll;
PID_Controller_t pid_pitch;
PID_Controller_t pid_yaw;

// State intermediate variables
float desired_roll;
float desired_pitch;
float desired_yaw;

void Control_Init(void)
{

	PID_Init(&pid_z, 2.5f, 0.1f, 1.0f, 10.0f);
	PID_Init(&pid_roll, 1.2f, 0.0f, 0.3f, 5.0f);
	PID_Init(&pid_pitch, 1.2f, 0.0f, 0.3f, 5.0f);
	PID_Init(&pid_yaw, 2.0f, 0.0f, 0.0f, 5.0f);

}
