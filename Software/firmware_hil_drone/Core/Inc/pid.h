/*
 * pid.h
 *
 *  Created on: 15 jul 2026
 *      Author: kevin
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// This struct contains the "memory" and configuration of a single PID
typedef struct {

	// Gains
	float kp; // Proportional gain
	float ki; // Integrative gain
	float kd; // Derivative gain

	// Internal memory
	float accumulated_error;
	float previous_error;

	// Security limit
	float output_limit;

}; PID_Controller_t;

void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float limit);
float PID_Compute(PID_Controller_t* pid, float setpoint, float measure, float dt);

#endif /* INC_PID_H_ */
