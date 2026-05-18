#ifndef __MYTASK_H__
#define __MYTASK_H__

#include "FreeRTOS.h"
#include "semphr.h"

void Avoidance_Task(void *pvParameters);
void irtracking_Task(void *pvParameters);
void CardWriteTask(void *pvParameters);
void CardNavTask(void *pvParameters);
void MotorTestTask(void *pvParameters);
void car_walking(void *pvParameters);
void Avoidance_Semaphore_Init(void);

extern SemaphoreHandle_t xAvoidSemaphore;

#endif
