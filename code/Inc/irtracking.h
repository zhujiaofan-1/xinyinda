#ifndef __IRTRACKING_H__
#define __IRTRACKING_H__

#include "Headfile.h"


#define IR_DEV_ADDR       0x12
#define IR_READ_ADDR      0x30

#define TURN_ENCODER_90         20
#define TURN_ENCODER_180        40
#define TURN_ENCODER_STRAIGHT   50

void irtacking_Read(uint8_t *s1,uint8_t *s2,uint8_t *s3,uint8_t *s4,uint8_t *s5,uint8_t *s6,uint8_t *s7,uint8_t *s8);
float APP_ELE_PID_Calc(int8_t actual_value);
uint8_t LineWalking(void);

#endif
