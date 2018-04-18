#include "stm32f10x.h"

void DATA_Trans(uint8_t *s,short temp_data);
void USART1_Configuration(void);
void Send_data(uint8_t MAG,uint8_t axis);
void USART1_SendData(uint8_t SendData);
void UsartSend(uint16_t ch);
void PrintChar(char *s);
void UART1_Send(uint8_t *s,uint8_t len);
