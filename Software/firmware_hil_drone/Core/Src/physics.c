/*
 * physics.c
 *
 *  Created on: 16 jul 2026
 *      Author: kevin
 */

#include "config.h"
#include "physics.h"
#include <math.h>

static float velocity_x = 0.0f, velocity_y = 0.0f, velocity_z = 0.0f;
static float velocity_roll = 0.0f, velocity_pitch = 0.0f, velocity_yaw = 0.0f;

void Physics_Update(float dt, float u1, float u2, float u3, float u4, FlightState_t* state)
{

	// --- Rotational dynamics
	float alpha_roll  = u2 / config.xx_inertia;
	float alpha_pitch = u3 / config.yy_inertia;
	float alpha_yaw   = u4 / config.zz_inertia;
	// Angular velocity
	velocity_roll  += alpha_roll  * dt;
	velocity_pitch += alpha_pitch * dt;
	velocity_yaw   += alpha_yaw   * dt;
	// SIMULATED air friction
	float damping = (1.0f - (10.0f * dt));
	if(damping < 0.0f) damping = 0.0f;
	velocity_roll  *= damping;
	velocity_pitch *= damping;
	velocity_yaw   *= damping;
	// Angles update in global state
	state->roll  += velocity_roll  * dt;
	state->pitch += velocity_pitch * dt;
	state->yaw   += velocity_yaw   * dt;

	// --- 3D dynamics
	float thrust_accel = u1 / config.weight_kg;
	float drag_coeff   = config.f_aero / config.weight_kg;
	// Calculate the needed accelerations
	float ax = thrust_accel * (cosf(state->roll) * sinf(state->pitch) * cosf(state->yaw) + sinf(state->roll) * sinf(state->yaw)) - drag_coeff * velocity_x;
	float ay = thrust_accel * (cosf(state->roll) * sinf(state->pitch) * sinf(state->yaw) - sinf(state->roll) * cosf(state->yaw)) - drag_coeff * velocity_y;
	float az = thrust_accel * (cosf(state->roll) * cosf(state->pitch)) - 9.81f - drag_coeff * velocity_z;
	// Velocity integrations with accelerations
	velocity_x += ax * dt;
	velocity_y += ay * dt;
	velocity_z += az * dt;
	// Coordinates update in global state
	state->x += velocity_x * dt;
	state->y += velocity_y * dt;
	state->z += velocity_z * dt;

	// --- Ground protection
	if(state->z <= 0.0f)
	{
		state->z = 0.0f;
		velocity_z = 0.0f;
	}
}
