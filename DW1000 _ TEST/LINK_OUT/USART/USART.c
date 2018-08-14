#include "USART.h"


///串口DMA笔记
/**
*@brief : 使用DMA进行串口接收
1.串口初始化配置
2.DMA初始化配置（每次DMA初始化配置前需要进行反初始化Deinit）
3.HAL_UART_Receive_DMA(&UART1_Handler,RX_BUF,Uart_size);打开串口DMA接收


**/



UART_HandleTypeDef UART_Handler; //UART句柄
DMA_HandleTypeDef  UARTRxDMA_Handler;      //DMA句柄
#define Uart_size 8

u8 RX_BUF[Uart_size];
void UART_Init(void)
{
	UART_Handler.Instance=USART2;					    //USART1
	UART_Handler.Init.BaudRate=115200;				    //波特率
	UART_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	UART_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&UART_Handler);					    //HAL_UART_Init()会使能UART1
}

 void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART2)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_GPIOC_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_GPIOD_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_GPIOE_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_GPIOF_CLK_ENABLE();			//使能GPIOA时钟
		
		__HAL_RCC_USART2_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3;		
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART2;	//复用为USART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

	
	}
	
}

struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}

