//ʹ��TIM3����������PWM��
#include "pwm.h"
#include "control.h"
uint16_t PrescalerValue = 0;

//����ٶȸ���
void Moto_PwmRflash(int16_t Moto1_PWM,int16_t Moto2_PWM,int16_t Moto3_PWM,int16_t Moto4_PWM)
{		
	if(Moto1_PWM>Moto_PwmMax)	Moto1_PWM = Moto_PwmMax;
	if(Moto2_PWM>Moto_PwmMax)	Moto2_PWM = Moto_PwmMax;
	if(Moto3_PWM>Moto_PwmMax)	Moto3_PWM = Moto_PwmMax;
	if(Moto4_PWM>Moto_PwmMax)	Moto4_PWM = Moto_PwmMax;
	if(Moto1_PWM<Moto_PwmMin)	Moto1_PWM = Moto_PwmMin;
	if(Moto2_PWM<Moto_PwmMin)	Moto2_PWM = Moto_PwmMin;
	if(Moto3_PWM<Moto_PwmMin)	Moto3_PWM = Moto_PwmMin;
	if(Moto4_PWM<Moto_PwmMin)	Moto4_PWM = Moto_PwmMin;

	TIM5->CCR1 = Moto1_PWM;
	TIM5->CCR2 = Moto2_PWM;
	TIM5->CCR3 = Moto3_PWM;
	TIM5->CCR4 = Moto4_PWM;
}

//pwm��ʼ��
void PWM_Init()
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
	
		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5,ENABLE);//ʹ��TIM1�˿�ʱ��
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);//ʹ��A�˿�ʱ��
		
		 //PORTA.8\9\10\11
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;				 //	ѡ�йܽ�6 7
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 // �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // ����������50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);				 // ѡ��A�˿�
			
	
		/*TIM1ʱ��Ƶ��Ϊ10khz  SysClk/(PrescalerValue+1)/(TIM_Period+1)=100hz
		----------------------------------------------------------------------- */
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;	//PrescalerValue=71
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = 9999;			//����ֵ					
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	//Ԥ��Ƶֵ ��ֵ+1Ϊ��Ƶʱ�ĳ���
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;					//����Ƶ ��λ������
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//���ϼ���

		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 1000;				//��ʼ��ռ�ձ�Ϊ1ms �ߵ�ƽ
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		
		/* PWM1 Mode configuration: Channel1 */
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
		/* PWM1 Mode configuration: Channel2 */
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel3 */
		TIM_OC3Init(TIM5, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel4 */
		TIM_OC4Init(TIM5, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM5, ENABLE);

		/* TIM3 enable counter */
		TIM_Cmd(TIM5, ENABLE);
}
