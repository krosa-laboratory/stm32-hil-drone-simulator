#ifndef PID_H
#define PID_H

typedef struct {
	float kp;
	float ki;
	float kd;
	float accu_error;
	float prev_error;
	float output_limit;
} PID_Controllar_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float limit);
float PID_Compute(PID_Controller_t *pid, float set_point, float measure, float dt);
void PID_Reset(PID_Controller_t *pid);

#endif
