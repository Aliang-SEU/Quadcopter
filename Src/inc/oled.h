#include "stm32f10x.h"

//ʹ��4�ߴ��нӿ�ʱʹ�� 
#define OLED_SCLK PFout(4)
#define OLED_SDIN PFout(3)
#define OLED_RST 	PFout(2)		  
#define OLED_DC 	PFout(1)    
#define OLED_CS 	PFout(0)     

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

//OLED�����ú���
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
void OLED_Fill1(void);						   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowNum(uint8_t x,uint8_t y,u32 num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p);	 
