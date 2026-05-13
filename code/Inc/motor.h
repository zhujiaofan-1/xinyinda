#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stdint.h"


#define CAM_DEFAULT_I2C_ADDRESS       (0x34)			//I2C地址
#define MOTOR_TYPE_ADDR               20 				//编码电机类型设置寄存器地址
#define MOTOR_FIXED_SPEED_ADDR       	51				//速度寄存器地址。属于闭环控制
#define MOTOR_ENCODER_POLARITY_ADDR   21				//电机编码方向极性地址
#define MOTOR_FIXED_PWM_ADDR      		31				//固定PWM控制地址，属于开环控制
#define MOTOR_ENCODER_TOTAL_ADDR  		60				//4个编码电机各自的总脉冲值
#define ADC_BAT_ADDR                  0					//电压地址
#include "Typies.h"

//电机类型具体地址
#define MOTOR_TYPE_WITHOUT_ENCODER        0 		//无编码器的电机,磁环每转是44个脉冲减速比:90  默认
#define MOTOR_TYPE_TT                     1 		//TT编码电机
#define MOTOR_TYPE_N20                    2 		//N20编码电机
#define MOTOR_TYPE_JGB                    3 		//磁环每转是44个脉冲   减速比:90  默认

void Motor_Set_Speed(motor_speed_t speed);
void Motor_Init(void);
uint16_t Motor_Get_Vol(void);
void Motor_Get_Encoder(motor_encoder_t* Encoder);
void Motor_Reset_Encoder(void);
void Motion_Ctrl(int Vx, int Vy, int W, int Vz);

#endif
