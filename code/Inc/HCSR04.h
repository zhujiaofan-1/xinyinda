#ifndef __HCSR04_H__
#define __HCSR04_H__

#define HCSR04_TRIG_PIN GPIO_PIN_5
#define HCSR04_ECHO_PIN GPIO_PIN_6

#define HCSR04_GPIO_PORT GPIOF


void HCSR04_Init(void);

float HCSR04_Get_Distance(void);

extern volatile float g_ultrasonic_distance;

#endif
