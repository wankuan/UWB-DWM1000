#ifndef __HA_USART_DMA_H
#define __HA_USART_DMA_H

#include "sys.h"

extern UART_HandleTypeDef USART_Handler; //UART¾ä±ú
extern DMA_HandleTypeDef  USARTRxDMA_Handler;      //DMA¾ä±ú
void UART2_Init(void);
void DMA_IT_Re_Init(void);
#endif 
