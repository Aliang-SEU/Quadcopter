#ifndef __SPI_H
#define __SPI_H
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
										  
void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(uint8_t SpeedSet); //����SPI�ٶ�   
uint8_t SPI2_ReadWriteByte(uint8_t TxData);//SPI���߶�дһ���ֽ�
		 
#endif