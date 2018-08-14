#include "KEY.h"

void KEY_Init(void)
{

		GPIO_InitTypeDef GPIO_Initure;
		
		__HAL_RCC_GPIOA_CLK_ENABLE();           
		__HAL_RCC_GPIOE_CLK_ENABLE();
		
		GPIO_Initure.Pin=GPIO_PIN_0; 
		GPIO_Initure.Mode=GPIO_MODE_IT_RISING ;
		GPIO_Initure.Pull=GPIO_PULLDOWN;        
		GPIO_Initure.Speed=GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		
    HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);       //抢占优先级为2，子优先级为2
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);             //使能中断线2
		
}

