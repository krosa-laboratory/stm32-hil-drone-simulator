/*
 * telemetry.h
 *
 *  Created on: 11 ago 2026
 *      Author: kevin
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "state.h"

void Telemetry_Init(void);

void Telemetry_SendState(const FlightState_t* actual, const FlightState_t* desire, float u1);
void Telemetry_StoreCommand(uint8_t cmd);
void Telemetry_ProcessCommands(FlightState_t* desire);

#endif /* INC_TELEMETRY_H_ */
