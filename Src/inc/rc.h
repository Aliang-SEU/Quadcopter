#ifndef _RC_RC_H_
#define _RC_RC_H_
#include "stm32f10x.h"

/*----------------油门检查----------------------*/
#define RC_MINCHECK   1200
#define RC_MAXCHECK   1800

/*-------------自动解除武装时间-----------------*/
#define AUTODISARMDE_TIME 2500  

typedef struct uint16_rcget
{
				int16_t rc_data[4];
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;
}RC_GETDATA;

/*********** RC alias *****************/
enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
};

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

extern uint16_t  RC_Pwm_In[8];
extern uint16_t  RC_Pwm_In_his[8];
extern RC_GETDATA Rc_Get;
extern uint16_t tempup[4] ;	//捕获总高电平的时间
extern void RC_directive(void);
extern void RC_Data_Refine(void);
extern void Calculate_Target(void); 
extern void TIM4_Cap_Init(u16 arr, u16 psc);
#endif
