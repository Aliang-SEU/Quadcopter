/**
  ******************************************************************************
  * @file    ADC/ADC1_DMA/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "led.h"
#include "main.h"
#include "uart_communication.h"


//定时中断服务程序
void TIM2_IRQHandler(void)
{
	static uint32_t TIM3_IRQCNT = 0;
	
	if(TIM2->SR & TIM_IT_Update)		 
	{     
		TIM2->SR = ~TIM_FLAG_Update;	   //清除中断标志
			TIM3_IRQCNT ++;
		//等待系统所有的器件初始化成功
		if(!SYS_INIT_OK)		
			return;

			//RC_directive();			//遥控器数据提炼
		//	RC_Data_Refine();		//
			Calculate_Target(); 
			MPU9250_ReadValue();//9250读取数据		
			Prepare_Data();			//修正数据
			Get_Attitude();			//姿态计算
			UART_Send_Sensor();
			UART_Send_Status();
			UART_Send_RCDATA();
			CONTROL( Q_ANGLE.roll, Q_ANGLE.pitch, Q_ANGLE.yaw);//控制电机
	}
}
