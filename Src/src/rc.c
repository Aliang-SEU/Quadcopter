//   ����ң���� ���벶��������
//****************************************************

#include "rc.h"
#include "main.h"
#include "pwm.h"

uint8_t TIM4CH1_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
uint16_t TIM4CH1_CAPTURE_UPVAL;
uint16_t TIM4CH1_CAPTURE_DOWNVAL;

uint8_t TIM4CH2_CAPTURE_STA = 0;	//ͨ��2���벶���־������λ�������־����6λ�������־		
uint16_t TIM4CH2_CAPTURE_UPVAL;
uint16_t TIM4CH2_CAPTURE_DOWNVAL;

uint8_t TIM4CH3_CAPTURE_STA = 0;	//ͨ��3���벶���־������λ�������־����6λ�������־		
uint16_t TIM4CH3_CAPTURE_UPVAL;
uint16_t TIM4CH3_CAPTURE_DOWNVAL;

uint8_t TIM4CH4_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־		
uint16_t TIM4CH4_CAPTURE_UPVAL;
uint16_t TIM4CH4_CAPTURE_DOWNVAL;

//ң�������ݱ���
RC_GETDATA Rc_Get;
uint16_t  RC_Pwm_In[8];
uint16_t  RC_Pwm_In_his[8];

//��ʱ��������
uint16_t tempup[4] = {0};	//�����ܸߵ�ƽ��ʱ��
uint32_t tim4_T1;
uint32_t tim4_T2;
uint32_t tim4_T3;
uint32_t tim4_T4;

int pwmout1, pwmout2, pwmout3, pwmout4; 				//���ռ�ձ�

//��ʱ��4ͨ��1���벶������

TIM_ICInitTypeDef TIM4_ICInitStructure;

void TIM4_Cap_Init(uint16_t arr, uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ��TIM4ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
			| GPIO_Pin_9;  //PB6,7,8,9 ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 ���� 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  ����

	//��ʼ����ʱ��4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM4���벶����� ͨ��1
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//��ʼ��TIM4���벶����� ͨ��2
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//��ʼ��TIM4���벶����� ͨ��3
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//��ʼ��TIM4���벶����� ͨ��4
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
			ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�	
	TIM_Cmd(TIM4, ENABLE); 		//ʹ�ܶ�ʱ��4

}

//��ʱ��4�жϷ������
void TIM4_IRQHandler(void)
{
	if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) 		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 		//����1���������¼�
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); 		//����жϱ�־λ
			if (TIM4CH1_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
				{
					tim4_T1 = 65535;
				}
				else
					tim4_T1 = 0;
				tempup[0] = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL + tim4_T1;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout1 = tempup[0];		//�ܵĸߵ�ƽ��ʱ��
				TIM4CH1_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);		//��ȡ����������
				TIM4CH1_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}

	if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)		//����2���������¼�
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);		//����жϱ�־λ
			if (TIM4CH2_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
				{
					tim4_T2 = 65535;
				}
				else
					tim4_T2 = 0;
				tempup[1] = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL
						+ tim4_T2;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout2 = tempup[1] ;		//�ܵĸߵ�ƽ��ʱ��
				TIM4CH2_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);		//��ȡ����������
				TIM4CH2_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}

	if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)		//����3���������¼�
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);		//����жϱ�־λ
			if (TIM4CH3_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
				{
					tim4_T3 = 65535;
				}
				else
					tim4_T3 = 0;
				tempup[2] = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL
						+ tim4_T3;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout3 = tempup[2];		//�ܵĸߵ�ƽ��ʱ��
				TIM4CH3_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);		//��ȡ����������
				TIM4CH3_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}

	if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)		//����4���������¼�
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);		//����жϱ�־λ
			if (TIM4CH4_CAPTURE_STA & 0X40)		//����һ���½���
			{
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
				{
					tim4_T4 = 65535;
				}
				else
					tim4_T4 = 0;
				tempup[3] = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL
						+ tim4_T4;		//�õ��ܵĸߵ�ƽ��ʱ��
				pwmout4 = tempup[3] ;		//�ܵĸߵ�ƽ��ʱ��
				TIM4CH4_CAPTURE_STA = 0;		//�����־λ����
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���		  
			}
			else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);		//��ȡ����������
				TIM4CH4_CAPTURE_STA |= 0X40;		//����Ѳ���������
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
			}
		}
	}
}

//ң������������
void RC_Data_Refine(void)
{
  uint8_t chan,a;	

	uint16_t rcDataMax[4], rcDataMin[4];
	static int16_t rcDataCache[4][4], rcDataMean[4];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	
	for (chan = 0; chan < 4; chan++) 
	{
		  //����ƽ��ֵ�˲���4��
		  if(tempup[chan]>2800 || tempup[chan]<800)  RC_Pwm_In[chan] = RC_Pwm_In_his[chan];	//�����쳣����
			rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan] ;													//���ݻ���
		  RC_Pwm_In_his[chan] = RC_Pwm_In[chan];																						//��һ�ε����ݼ�¼
			
			rcDataMean[chan] = 0;								//��ʼ������
		  rcDataMax[chan] = 0;
		  rcDataMin[chan] = 25000;
		
			for (a = 0; a < 4; a++) 
			{	
				  // ��¼���������ֵ && ��Сֵ
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // ���
					rcDataMean[chan] += rcDataCache[chan][a];  
      }
			// �޳������� ���ֵ && ��Сֵ 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	 } 
	 Rc_Get.YAW   = tempup[0];		 //Rc_Get.rc_data[0] = rcDataMean[0];			//������������
	 Rc_Get.THROTTLE  = tempup[2]; //Rc_Get.rc_data[2] = rcDataMean[2];
	 Rc_Get.ROLL  = tempup[3];		 //Rc_Get.rc_data[3] = rcDataMean[3];
	 Rc_Get.PITCH = tempup[1];		 //Rc_Get.rc_data[1] = rcDataMean[1];
}

//ң����ָ�����
void RC_directive(void)
{
  uint8_t stTmp = 0,i;
	static uint8_t  rcSticks;
	static uint8_t  rcDelayCommand;
  static uint16_t seltLockCommend;	
	
	for (i = 0; i < 4; i++) {
			stTmp >>= 2;
			if (Rc_Get.rc_data[i] > RC_MINCHECK)
					stTmp |= 0x80;  // check for MIN
			if (Rc_Get.rc_data[i] < RC_MAXCHECK)
					stTmp |= 0x40;  // check for MAX
	}
	if (stTmp == rcSticks) {
			if (rcDelayCommand < 250)
					rcDelayCommand++;
	} else
			rcDelayCommand = 0;
	rcSticks = stTmp;
	
	if (rcDelayCommand == 150) {
		if (ARMED){
			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //���� 
			 {
				 LED_FLASH();
				  ARMED=0;
			 }
		}
		else{
      if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)    //����   
			{
				 LED_FLASH1();
					ARMED=1;
			}
    }
	}
	//��װ֮��һ��ʱ�����ű������  ���Զ������װ
	if (ARMED){
	   if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_CE) {
		    if (seltLockCommend < AUTODISARMDE_TIME)
					 seltLockCommend++;
				else 
					 ARMED=0;
		 }
		 else 
        seltLockCommend = 0;			 
	}
}
//����ң������������Ҫ������
void Calculate_Target(void) 
{
	int16_t ftemp=0;
	
	Rc_Get.YAW   = tempup[0];//Rc_Get.rc_data[2] =rcDataMean[3];			//������������
	Rc_Get.THROTTLE  = tempup[2]; //Rc_Get.rc_data[3] =rcDataMean[2];
	Rc_Get.ROLL  = tempup[3];//Rc_Get.rc_data[0] = rcDataMean[0];
	Rc_Get.PITCH = tempup[1];//Rc_Get.rc_data[1] = rcDataMean[1];
	
	Target.Pitch = (1500-Rc_Get.PITCH)/20;
	Target.Roll = (Rc_Get.ROLL-1500)/20;

  //Ŀ�꺽����ơ������Ŵ�����С���ֵʱ����Ϊ�û�ϣ����ɡ���ô��ʱ�ĺ�����ΪĿ�꺽��
   if(Rc_Get.THROTTLE > RC_MINCHECK ) 
	 {
	    Target.Yaw = Q_ANGLE.yaw; //����ǰ�ĺ�����ΪĿ�꺽��
   }
	//�������е�����һ������
	if((Rc_Get.YAW > 1700)||(Rc_Get.YAW < 1300))
	{
		ftemp = 1500 - Rc_Get.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.05f; 
		
		//ת[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
}
