#ifndef __SERVO_H__
#define __SERVO_H__

#include "Headfile.h"

#define SERVO_PWM_PIN           GPIO_PIN_12

#define SERVO_PWM_PORT          GPIOD


void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);

#endif
