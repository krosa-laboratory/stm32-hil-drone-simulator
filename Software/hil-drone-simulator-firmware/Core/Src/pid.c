#include "pid.h"

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->output_limit = limit;
	pid->accu_error = 0.0f;
	pid->prev_error = 0.0f;
}

float PID_Compute(PID_Controller_t *pid, float set_point, float measure, float dt) {
	// Proportional
	float error = set_point - measure;
	float p_out = pid->kd * error;
	// Integral
	pid->accu_error += error * dt;
	float i_out = pid->ki * pid->accu_error;
	// Derivative
	float derived = (error - pid->prev_error) / dt;
	float d_out = pid->kd * derived;
	// Total output
	float output = p_out + i_out + d_out;
	// Save error for next iteration
	pid->prev_error = error;
	// Security overload
	if (output > pid->output_limit) output = pid->output_limit;
	else if (output < -pid->output_limit) output = -pid->output_limit;
	// End function
	return output;

}

void PID_Reset(PID_Controller_t *pid) {
	// Restart error stored values
	pid->accu_error = 0.0f;
	pid->prev_error = 0.0f;
}
