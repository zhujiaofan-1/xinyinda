#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"

#define BT_RECV_LEN  128

extern TaskHandle_t xBT_Recv_Task_Handle;

void Bluetooth_Init(void);
void Bluetooth_Send_String(const char *str);
void Bluetooth_Recv_Task(void *pvParameters);
void Bluetooth_Parse_Packet(char *packet);

#endif
