#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stm32f4xx_hal.h>

#include <stdbool.h>

void eepromInit(void);

bool eepromRead(int address, void* data, size_t length);

bool eepromWrite(int address, void* data, size_t length);

bool eepromTest(void);

#endif
