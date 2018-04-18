#include "main.h"

uint8_t SYS_INIT_OK=0;

int main(void)
{  
	SYS_INIT_OK=0;
	Init_All();	
	Nvic_Init();						//�ж�ʹ��
	Tim2_Init(2000);				//2ms�ж�һ��
	SYS_INIT_OK=1;		
	
	while(1) 
	{	
		
	}				
}
void Init_All()
{
	static uint8_t flag=0;
	SystemInit();
	//Nvic_Init();
	delay_init(72);					//Systic��ʱ��ʼ��
	delay_ms(1500);
	PWM_Init();							//��ʼ���������
	USART1_Configuration();	//PA9 TXD PA10 RXD
	OLED_Init();						//OLED��ʼ��
	LED_Init();
	
	flag=MPU9250_Init();
	
	if(flag==0)
	{
		OLED_ShowString(0,0,"MPU9250 Init Success!");
		OLED_ShowString(0,10,"Wait for Gyro SelfTest!");
		OLED_Refresh_Gram();
		Clac_GYRO_OFFECT();		//�������������ݵ�У׼
		delay_ms(1500);
		OLED_ShowString(0,22,"Gyro SelfTest Success!");
		OLED_Refresh_Gram();
		delay_ms(1500);
		OLED_Clear();  
	}
	else
	{
		OLED_ShowString(0,0,"MPU9250 Init Fail!");
		OLED_Refresh_Gram();
		while(1);				//��ʼ��ʧ�ܽ�����ѭ��
	}
	
	//��������ƫУ׼
	Pid_init();							//pid����У׼
	TIM4_Cap_Init(0xffff,72-1);	//ң�������벶׽�ж�
}

//�жϹ�����
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//TIM2 ��ʱ��
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//TIM4 ң����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

}

//printf�����ض��� fputc���� GetKey ����
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (unsigned char) ch);// USART1 ???? USART2 ?
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}
int GetKey(void) 
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return ((int)(USART1->DR & 0x1FF));
}
