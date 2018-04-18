#ifndef LED_H_
#define LED_H_

#include "stm32f10x.h"

#define LED1 PDout(13)
#define LED2 PGout(14)
#define LED1_ON  	GPIO_SetBits(GPIOD, GPIO_Pin_13)
#define LED1_OFF 	GPIO_ResetBits(GPIOD, GPIO_Pin_13)
#define LED2_ON  	GPIO_SetBits(GPIOG, GPIO_Pin_14)
#define LED2_OFF 		GPIO_ResetBits(GPIOG, GPIO_Pin_14)
#define LEDALL_OFF  LED1_OFF;LED2_OFF;
#define LEDALL_ON 	LED1_ON;LED2_ON;

void LED_Init(void);
void LED_FLASH(void);
void LED_FLASH1(void);
#endif
