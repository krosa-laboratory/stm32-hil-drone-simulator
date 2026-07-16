/*
 * physics.c
 *
 *  Created on: 16 jul 2026
 *      Author: kevin
 */

#include "config.h"
#include "physics.h"

extern float actual_roll, actual_pitch, actual_yaw, actual_z;
static float velocity_z, velocity_roll, velocity_pitch, velocity_yaw;

void Physics_Update(float dt, float u1, float u2, float u3, float u4)
{

	// --- Altitude dynamics
	float acceleration_z = (u1 / config.weight_kg) - 9.81;
	velocity_z = velocity_z + (acceleration_z * dt);
	actual_z = actual_z + (velocity_z * dt);

	// --- Rotational dynamics
	// Angular accelerations
	float alpha_roll  = u2 / config.xx_inertia;
	float alpha_pitch = u3 / config.yy_inertia;
	float alpha_yaw   = u4 / config.zz_inertia;
	// Angular velocity
	velocity_roll  = velocity_roll + (alpha_roll * dt);
	velocity_pitch = velocity_pitch + (alpha_pitch * dt);
	velocity_yaw   = velocity_yaw + (alpha_yaw * dt);
	// SIMULATED air friction
	velocity_roll  *= 0.99f;
	velocity_pitch *= 0.99f;
	velocity_yaw   *= 0.99f;
	// Actual angles
	actual_roll  = actual_roll + (velocity_roll * dt);
	actual_pitch = actual_pitch + (velocity_pitch * dt);
	actual_yaw   = actual_yaw + (velocity_yaw * dt);

}
