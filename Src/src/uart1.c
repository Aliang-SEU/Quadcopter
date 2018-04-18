/****************************************
						����V1.0�������� 
						�޸�ʱ��2015/07/29
***************************************/
#include "uart1.h"
#include "delay.h"

//*****UART1���ڳ�ʼ��***************************
void USART1_Configuration(void)
{
		//���ų�ʼ��PA9-TXD PA10-RXD
		GPIO_InitTypeDef GPIO_InitStructure;
			//���ڳ�ʼ��
		USART_InitTypeDef USART_InitStructure;
		USART_ClockInitTypeDef  USART_ClockInitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |RCC_APB2Periph_USART1, ENABLE  );
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);//ʹ��A�˿�ʱ��
		
		 /* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //	ѡ�йܽ�9
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 // �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // ����������50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);				 // ѡ��A�˿�
			
		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  //ѡ�йܽ�10
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);				  //ѡ��A�˿�
	
		//**************************************************************
		
		USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;			// ʱ�ӵ͵�ƽ�
		USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;				// ʱ�ӵ͵�ƽ
		USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;				// ʱ�ӵڶ������ؽ������ݲ���
		USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;		// ���һλ���ݵ�ʱ�����岻��SCLK���
		/* Configure the USART1 synchronous paramters */
		USART_ClockInit(USART1, &USART_ClockInitStructure);					// ʱ�Ӳ�����ʼ������
																			 
		USART_InitStructure.USART_BaudRate = 115200;						  // ������Ϊ��115200
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;			  // 8λ����
		USART_InitStructure.USART_StopBits = USART_StopBits_1;				  // ��֡��β����1��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No ;				  // ��żʧ��
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// Ӳ��������ʧ��

		USART_InitStructure.USART_Mode =  USART_Mode_Tx | USART_Mode_Rx;		  // ����ʹ��+����ʹ��
		/* Configure USART1 basic and asynchronous paramters */
		USART_Init(USART1, &USART_InitStructure);
				
			/* Enable USART1 */
		USART_ClearFlag(USART1, USART_IT_RXNE); 			//���жϣ�����һ�����жϺ����������ж�
		USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);		//ʹ��USART1�ж�Դ
		USART_Cmd(USART1, ENABLE);							//USART1�ܿ��أ����� 
}

//***********�������ݣ�1���ֽڣ�*************************************
void USART1_SendData(uint8_t SendData)
{
	USART_SendData(USART1, SendData);
	delay_ms(1);
}

/*********���ڷ�������*************************************
//��ʽ  AX��xxx  BX��xxx
void Send_data(uint8_t MAG,uint8_t axis)
{
	uint8_t i;
	USART1_SendData(MAG);
  USART1_SendData(axis);
  USART1_SendData(':');
  for(i=0;i<4;i++)USART1_SendData(TX_DATA[i]);	//���ͽ��յ�����
  USART1_SendData(' ');
  USART1_SendData(' ');
}*/

//ת������
void DATA_Trans(uint8_t *s,short temp_data)
{
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s=' ';
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //ȡ������
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //ȡ������
    *++s =temp_data+0x30; 	
}
void PrintChar(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		UsartSend(*p);
		p++;
	}
}
void UART1_Send(uint8_t *s,uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
		USART_SendData(USART1, *(s+i));                              //??????:USART1->DR = '0';  
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){} //????????  
	}
}
void UsartSend(uint16_t ch)
{
	USART1->DR = (ch&(u16)0x01FF);   
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
}
