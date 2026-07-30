/*
 * state.h
 *
 *  Created on: 29 jul 2026
 *      Author: kevin
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

typedef struct {
	float x;	 // Meters
	float y;	 // Meters
	float z;	 // Meters
	float roll;  // Degrees
	float pitch; // Degrees
	float yaw;   // Degrees
} FlightState_t;

// Public exposition of the system state variables
extern FlightState_t actual_state;
extern FlightState_t desire_state;

#endif /* INC_STATE_H_ */
