/*
 * control.h
 *
 *  Created on: 15 jul 2026
 *      Author: kevin
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

void Control_Init(void);

float Control_GetRealU1(void);
void Control_UpdatePID(float new_kp, float new_ki, float new_kd);

#endif /* INC_CONTROL_H_ */
