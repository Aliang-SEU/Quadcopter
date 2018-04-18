#include "stm32f10x.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
 	
extern 	uint8_t 	Data_to_Receive[32];//接收到的数据
extern 	uint8_t 	Data_to_Send[32];		//需要发送的数据

extern void UART_Send_Sensor(void);
extern void UART_Send_Status(void);
extern void UART_Send_RCDATA(void);
extern void UART_Send_MOTO(void);
extern void UART_Send_PID(void);
