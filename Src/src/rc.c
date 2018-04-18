//   极光遥控器 输入捕获解算程序
//****************************************************

#include "rc.h"
#include "main.h"
#include "pwm.h"

uint8_t TIM4CH1_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
uint16_t TIM4CH1_CAPTURE_UPVAL;
uint16_t TIM4CH1_CAPTURE_DOWNVAL;

uint8_t TIM4CH2_CAPTURE_STA = 0;	//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志		
uint16_t TIM4CH2_CAPTURE_UPVAL;
uint16_t TIM4CH2_CAPTURE_DOWNVAL;

uint8_t TIM4CH3_CAPTURE_STA = 0;	//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
uint16_t TIM4CH3_CAPTURE_UPVAL;
uint16_t TIM4CH3_CAPTURE_DOWNVAL;

uint8_t TIM4CH4_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
uint16_t TIM4CH4_CAPTURE_UPVAL;
uint16_t TIM4CH4_CAPTURE_DOWNVAL;

//遥控器数据保存
RC_GETDATA Rc_Get;
uint16_t  RC_Pwm_In[8];
uint16_t  RC_Pwm_In_his[8];

//临时测量变量
uint16_t tempup[4] = {0};	//捕获总高电平的时间
uint32_t tim4_T1;
uint32_t tim4_T2;
uint32_t tim4_T3;
uint32_t tim4_T4;

int pwmout1, pwmout2, pwmout3, pwmout4; 				//输出占空比

//定时器4通道1输入捕获配置

TIM_ICInitTypeDef TIM4_ICInitStructure;

void TIM4_Cap_Init(uint16_t arr, uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
			| GPIO_Pin_9;  //PB6,7,8,9 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 输入 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  下拉

	//初始化定时器4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM4输入捕获参数 通道1
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道2
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道3
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道4
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
			ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	
	TIM_Cmd(TIM4, ENABLE); 		//使能定时器4

}

//定时器4中断服务程序
void TIM4_IRQHandler(void)
{
	if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) 		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 		//捕获1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); 		//清除中断标志位
			if (TIM4CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//记录下此时的定时器计数值
				if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
				{
					tim4_T1 = 65535;
				}
				else
					tim4_T1 = 0;
				tempup[0] = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL + tim4_T1;		//得到总的高电平的时间
				pwmout1 = tempup[0];		//总的高电平的时间
				TIM4CH1_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);		//获取上升沿数据
				TIM4CH1_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)		//捕获2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);		//清除中断标志位
			if (TIM4CH2_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//记录下此时的定时器计数值
				if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
				{
					tim4_T2 = 65535;
				}
				else
					tim4_T2 = 0;
				tempup[1] = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL
						+ tim4_T2;		//得到总的高电平的时间
				pwmout2 = tempup[1] ;		//总的高电平的时间
				TIM4CH2_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);		//获取上升沿数据
				TIM4CH2_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)		//捕获3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);		//清除中断标志位
			if (TIM4CH3_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//记录下此时的定时器计数值
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
				{
					tim4_T3 = 65535;
				}
				else
					tim4_T3 = 0;
				tempup[2] = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL
						+ tim4_T3;		//得到总的高电平的时间
				pwmout3 = tempup[2];		//总的高电平的时间
				TIM4CH3_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);		//获取上升沿数据
				TIM4CH3_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)		//捕获4发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);		//清除中断标志位
			if (TIM4CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//记录下此时的定时器计数值
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
				{
					tim4_T4 = 65535;
				}
				else
					tim4_T4 = 0;
				tempup[3] = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL
						+ tim4_T4;		//得到总的高电平的时间
				pwmout4 = tempup[3] ;		//总的高电平的时间
				TIM4CH4_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);		//获取上升沿数据
				TIM4CH4_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}
}

//遥控器数据提炼
void RC_Data_Refine(void)
{
  uint8_t chan,a;	

	uint16_t rcDataMax[4], rcDataMin[4];
	static int16_t rcDataCache[4][4], rcDataMean[4];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	
	for (chan = 0; chan < 4; chan++) 
	{
		  //滑动平均值滤波，4次
		  if(tempup[chan]>2800 || tempup[chan]<800)  RC_Pwm_In[chan] = RC_Pwm_In_his[chan];	//数据异常处理
			rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan] ;													//数据缓存
		  RC_Pwm_In_his[chan] = RC_Pwm_In[chan];																						//上一次的数据记录
			
			rcDataMean[chan] = 0;								//初始化参数
		  rcDataMax[chan] = 0;
		  rcDataMin[chan] = 25000;
		
			for (a = 0; a < 4; a++) 
			{	
				  // 记录缓存中最大值 && 最小值
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // 求和
					rcDataMean[chan] += rcDataCache[chan][a];  
      }
			// 剔除缓存中 最大值 && 最小值 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	 } 
	 Rc_Get.YAW   = tempup[0];		 //Rc_Get.rc_data[0] = rcDataMean[0];			//保存解算的数据
	 Rc_Get.THROTTLE  = tempup[2]; //Rc_Get.rc_data[2] = rcDataMean[2];
	 Rc_Get.ROLL  = tempup[3];		 //Rc_Get.rc_data[3] = rcDataMean[3];
	 Rc_Get.PITCH = tempup[1];		 //Rc_Get.rc_data[1] = rcDataMean[1];
}

//遥控器指令设计
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
			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //上锁 
			 {
				 LED_FLASH();
				  ARMED=0;
			 }
		}
		else{
      if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)    //解锁   
			{
				 LED_FLASH1();
					ARMED=1;
			}
    }
	}
	//武装之后一段时间油门保持最低  则自动解除武装
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
//计算遥控器数据所需要的数据
void Calculate_Target(void) 
{
	int16_t ftemp=0;
	
	Rc_Get.YAW   = tempup[0];//Rc_Get.rc_data[2] =rcDataMean[3];			//保存解算的数据
	Rc_Get.THROTTLE  = tempup[2]; //Rc_Get.rc_data[3] =rcDataMean[2];
	Rc_Get.ROLL  = tempup[3];//Rc_Get.rc_data[0] = rcDataMean[0];
	Rc_Get.PITCH = tempup[1];//Rc_Get.rc_data[1] = rcDataMean[1];
	
	Target.Pitch = (1500-Rc_Get.PITCH)/20;
	Target.Roll = (Rc_Get.ROLL-1500)/20;

  //目标航向控制。当油门大于最小检查值时，认为用户希望起飞。那么此时的航向做为目标航向
   if(Rc_Get.THROTTLE > RC_MINCHECK ) 
	 {
	    Target.Yaw = Q_ANGLE.yaw; //将当前的航向做为目标航向
   }
	//航向在中点设置一个死区
	if((Rc_Get.YAW > 1700)||(Rc_Get.YAW < 1300))
	{
		ftemp = 1500 - Rc_Get.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.05f; 
		
		//转[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
}
