/*
 * telemetry.c
 *
 *  Created on: 11 ago 2026
 *      Author: kevin
 */

#include "telemetry.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

static char tx_buffer[128];				// Static transmission buffer of 128 bytes
static volatile uint8_t rx_command = 0; // Place to store the incoming command asynchronously

void Telemetry_Init(void)
{
	// Clean the transmission buffer in the initialization
	memset(tx_buffer, 0, sizeof(tx_buffer));
}

void Telemetry_Sync(float kp, float ki, float kd)
{
	char sync_msg[64];
	sprintf(sync_msg, "INIT_PID:%.3f,%.3f,%.3f\n", kp, ki, kd);
	CDC_Transmit_FS((uint8_t*)sync_msg, strlen(sync_msg));
}

void Telemetry_SendState(const FlightState_t* actual, const FlightState_t* desire, float u1)
{

	// Format the message in CSV Serial Plotter compatible
	int len = snprintf(tx_buffer, sizeof(tx_buffer),
			"R:%.2f,R_ref:%.2f,P:%.2f,P_ref:%.2f,Z:%.2f,U1:%.2f,X:%.2f,X_ref:%.2f,Y:%.2f,Y_ref:%.2f\r\n",
			actual->roll, desire->roll,
			actual->pitch, desire->pitch,
			actual->z, u1,
			actual->x, desire->x,
			actual->y, desire->y
			);

	// Asynchronous transmission if the message is correct
	if(len > 0 && len < (int)sizeof(tx_buffer))
	{
		CDC_Transmit_FS((uint8_t*)tx_buffer, (uint16_t)len);
	}

}

void Telemetry_StoreCommand(uint8_t cmd)
{
    rx_command = cmd;
}

void Telemetry_ProcessCommands(FlightState_t* desire)
{
    if (rx_command != 0)
    {
        switch(rx_command)
        {
            case 'w': case 'W': desire->x += 1.0f; break; // Forward
            case 's': case 'S': desire->x -= 1.0f; break; // Backward
            case 'a': case 'A': desire->y += 1.0f; break; // Left
            case 'd': case 'D': desire->y -= 1.0f; break; // Right
            case 'r': case 'R': desire->z += 1.0f; break; // Up
            case 'f': case 'F': desire->z -= 1.0f; break; // Down
        }
        rx_command = 0; // Empty the incoming command once processed
    }
}

int Telemetry_ParseCommand(const char* rx_buffer, float* new_kp, float* new_ki, float* new_kd, char* key_cmd)
{
	if (strncmp(rx_buffer, "GET_PID", 7) == 0) return 3;
    if (sscanf(rx_buffer, "P:%f,I:%f,D:%f", new_kp, new_ki, new_kd) == 3) return 1;
    else if (rx_buffer[0] != '\0')
    {
        *key_cmd = rx_buffer[0];
        return 2;
    }
    return 0;
}
