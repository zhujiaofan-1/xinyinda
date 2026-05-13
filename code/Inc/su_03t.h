#ifndef __SU_03T_H__
#define __SU_03T_H__

#include "main.h"
#include "freertos.h"
#include "task.h"

void su03t_Init(void);
void su03t_Send_Task(void *pvParameters);
void su03t_Receive_Task(void *pvParameters);
void Process_UART2_Recv_Data(uint8_t* data);

extern TaskHandle_t xUART2_Recv_Task_Handle;

#endif
