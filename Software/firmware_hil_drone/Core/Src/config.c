/*
 * config.c
 *
 *  Created on: 13 jul 2026
 *      Author: kevin
 */

#include "config.h"

DroneConfig_t config;

void Config_LoadStatus(void)
{
	config.weight_kg     = 1.2f;
	config.xx_inertia    = 0.015f;
	config.yy_inertia    = 0.015f;
	config.zz_inertia    = 0.025f;
	config.k_pwm	     = 28.5f;
	config.max_angle_rad = 0.5f;
}
