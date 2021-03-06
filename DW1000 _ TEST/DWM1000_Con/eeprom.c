/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stm32f4xx_hal.h>
#include <stdio.h>

#include <stdbool.h>
#include <string.h>

#include "eeprom.h"
#include "24C02.h"
#include "delay.h"


void eepromInit()
{
  AT24CXX_Init();
}

bool eepromTest()
{
  uint8_t test;

  return eepromRead(0, &test, 1);
}

bool eepromRead(int address, void* data, size_t length)
{
	  u8* p;
			p=data;
  for(int i=0;i<length;i++)
	{
			*p++=AT24CXX_ReadOneByte(address++);
	}
    return true;
}

bool eepromWrite(int address, void* data, size_t length)
{
	  u8* p;
			p=data;
  for(int i=0;i<length;i++)
	{
			AT24CXX_WriteOneByte(address++,*p++);
	}
    return true;

}
