#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"
typedef struct PID
{
	float P,pout,I,iout,D,dout,IMAX,OUT;
}PID;

extern int16_t MOTO1_PWM;
extern int16_t MOTO2_PWM;
extern int16_t MOTO3_PWM;
extern int16_t MOTO4_PWM;

extern uint8_t ARMED;

extern PID PID_ROL_OUTER,PID_PIT_OUTER,PID_YAW_OUTER;
extern PID PID_ROL_INNER,PID_PIT_INNER,PID_YAW_INNER;

void CONTROL(float rol, float pit, float yaw);
void Pid_init(void);

#endif
