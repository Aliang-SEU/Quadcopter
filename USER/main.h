#include "stm32f10x.h"
#include "uart1.h"
#include "Timer.h"
#include "led.h"
#include "math.h"
#include "delay.h"
#include "stdio.h"
#include "oled.h"
#include "mpu9250.h"
#include "Attitude_Calc.h"
#include "rc.h"
#include "kalman.h"
#include "control.h"
#include "pwm.h"
//**********º¯ÊýÉùÃ÷******************
void Init_All(void);
void Nvic_Init(void);
int fputc(int ch, FILE *f);
int GetKey(void) ;

extern uint8_t SYS_INIT_OK;
