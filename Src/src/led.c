#include "led.h"

void Delay_ms_led(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
} 

// D13 G14口两个LED灯
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE); //打开外设D的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG , ENABLE); //打开外设G的时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}
//两个灯同时闪烁
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
