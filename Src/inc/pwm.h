#ifndef Moto_H_
#define Moto_H_

#include "stm32f10x.h"

struct _target{
      float Pitch;    
	    float Roll;  
	    float Yaw;   
      int32_t Altiude; 
};

extern struct _target Target;

#define Moto_PwmMax 1800
#define Moto_PwmMin 1000
void Moto_PwmRflash(int16_t Moto1_PWM,int16_t Moto2_PWM,int16_t Moto3_PWM,int16_t Moto4_PWM);
void PWM_Init(void); 

#endif
