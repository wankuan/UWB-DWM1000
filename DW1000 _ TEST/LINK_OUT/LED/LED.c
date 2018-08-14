#include "LED.h"

void LED_Init(void)
{
		GPIO_InitTypeDef GPIO_Initure;
	
		__HAL_RCC_GPIOC_CLK_ENABLE();  		         
		
		GPIO_Initure.Pin = GPIO_PIN_11|GPIO_PIN_12; 
		GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Initure.Pull = GPIO_NOPULL;        
		GPIO_Initure.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOC,&GPIO_Initure);
		
	 LED_R=1;
		LED_G=1;
}

