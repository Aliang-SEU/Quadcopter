#include "main.h"

uint8_t SYS_INIT_OK=0;

int main(void)
{  
	SYS_INIT_OK=0;
	Init_All();	
	Nvic_Init();						//中断使能
	Tim2_Init(2000);				//2ms中断一次
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
	delay_init(72);					//Systic延时初始化
	delay_ms(1500);
	PWM_Init();							//初始化电调设置
	USART1_Configuration();	//PA9 TXD PA10 RXD
	OLED_Init();						//OLED初始化
	LED_Init();
	
	flag=MPU9250_Init();
	
	if(flag==0)
	{
		OLED_ShowString(0,0,"MPU9250 Init Success!");
		OLED_ShowString(0,10,"Wait for Gyro SelfTest!");
		OLED_Refresh_Gram();
		Clac_GYRO_OFFECT();		//进行陀螺仪数据的校准
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
		while(1);				//初始化失败进入死循环
	}
	
	//传感器零偏校准
	Pid_init();							//pid参数校准
	TIM4_Cap_Init(0xffff,72-1);	//遥控器输入捕捉中断
}

//中断管理器
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//TIM2 定时器
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//TIM4 遥控器
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

}

//printf函数重定向 fputc发送 GetKey 接收
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
