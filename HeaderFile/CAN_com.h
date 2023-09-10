#ifndef CAN_COM_H
#define CAN_COM_H

#include "structs.h"
#include "user_config.h"
#include "math_ops.h"
#include "driverlib.h"
#include "device.h"
#include "board.h" //For System configuration, generated codes.

void pack_reply(uint8_t * txMsgData, float p, float v, float t);
void unpack_cmd(uint8_t * rxMsgData, ControllerStruct * controller);


#endif
