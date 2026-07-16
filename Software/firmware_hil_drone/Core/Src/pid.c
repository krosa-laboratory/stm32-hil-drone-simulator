/*
 * pid.c
 *
 *  Created on: 15 jul 2026
 *      Author: kevin
 */

#include "pid.h"

void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float limit)
{

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->accumulated_error = 0.0f;
	pid->previous_error = 0.0f;
	pid->output_limit = limit;

}

float PID_Compute(PID_Controller_t* pid, float setpoint, float measure, float dt)
{

	// Calculate the actual error
	float error = setpoint - measure;
	// Proportional term
	float p_output = pid->kp * error;
	// Integrative term
	pid->accumulated_error += error * dt;
	float i_output = pid->ki * pid->accumulated_error;
	// Derivative term
	float derivative = (error - pid->previous_error) / dt;
	float d_output = pid->kd * derivative;
	// Construct the output
	float output = p_output + i_output + d_output;
	// Store actual error for next iteration
	pid->previous_error = error;
	// Saturate the output
	if(output > pid->output_limit) output = pid->output_limit;
	else if(output < -pid->output_limit) output = -pid->output_limit;

	return output;

}
