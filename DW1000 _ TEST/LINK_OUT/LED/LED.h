#ifndef __LED_H
#define __LED_H

#include "sys.h"

#define LED_R      	PCout(11) 
#define LED_G      	PCout(12) 

void LED_Init(void);

#endif

