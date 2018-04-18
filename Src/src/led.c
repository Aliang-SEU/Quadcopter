#include "led.h"

void Delay_ms_led(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
} 

// D13 G14������LED��
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE); //������D��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG , ENABLE); //������G��ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}
//������ͬʱ��˸
void LED_FLASH(void)
{
	LEDALL_ON;
	Delay_ms_led(100);
	LEDALL_OFF;
	Delay_ms_led(100);
	LEDALL_ON;
	Delay_ms_led(100);
	LEDALL_OFF;
	Delay_ms_led(100);
	LEDALL_ON;
	Delay_ms_led(100);
	LEDALL_OFF;
	Delay_ms_led(100);
	LEDALL_ON;
	Delay_ms_led(100);
	LEDALL_OFF;
	Delay_ms_led(100);
	LEDALL_ON;
	Delay_ms_led(100);
	LEDALL_OFF;
	Delay_ms_led(100);
	LEDALL_ON;
	Delay_ms_led(100);
	LEDALL_OFF;
	Delay_ms_led(100);
}
void LED_FLASH1(void)
{
	LEDALL_ON;
	Delay_ms_led(1000);
	LEDALL_OFF;
	Delay_ms_led(100);
}
