#ifndef __CAN_H
#define __CAN_H

#include "sys.h"


#define CAN1_STB PCout(2)

u8 CAN1_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode);
u8 CAN1_Send_Msg(u8* msg,u8 size);
u8 CAN1_Receive_Msg(u8 *buf);

#endif

