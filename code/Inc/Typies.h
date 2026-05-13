#ifndef __TYPES_H__
#define __TYPES_H__
#include <stdint.h>


//电机速度设置参数结构体 
typedef struct 
{
    int8_t speed_m1;
    int8_t speed_m2;
    int8_t speed_m3;
    int8_t speed_m4;
}motor_speed_t;

//电机编码器反馈参数结构体
typedef struct 
{
    int32_t encoder_m1;
    int32_t encoder_m2;
    int32_t encoder_m3;
    int32_t encoder_m4;
}motor_encoder_t;



#endif
