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


//��ʱ�жϷ������
void TIM2_IRQHandler(void)
{
	static uint32_t TIM3_IRQCNT = 0;
	
	if(TIM2->SR & TIM_IT_Update)		 
	{     
		TIM2->SR = ~TIM_FLAG_Update;	   //����жϱ�־
			TIM3_IRQCNT ++;
		//�ȴ�ϵͳ���е�������ʼ���ɹ�
		if(!SYS_INIT_OK)		
			return;

			//RC_directive();			//ң������������
		//	RC_Data_Refine();		//
			Calculate_Target(); 
			MPU9250_ReadValue();//9250��ȡ����		
			Prepare_Data();			//��������
			Get_Attitude();			//��̬����
			UART_Send_Sensor();
			UART_Send_Status();
			UART_Send_RCDATA();
			CONTROL( Q_ANGLE.roll, Q_ANGLE.pitch, Q_ANGLE.yaw);//���Ƶ��
	}
}
