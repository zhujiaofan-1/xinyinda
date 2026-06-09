#include "motor.h"
#include "Headfile.h"
#include <stdint.h>

SemaphoreHandle_t xMotorMutex = NULL;

/**
 * @brief  设置四个电机的速度
 * @param  speed: 包含四个电机速度值的结构体
 * @retval 无
 */
void Motor_Set_Speed(motor_speed_t speed)
{
    int8_t Speed_buf[4] = {speed.speed_m1, speed.speed_m2, speed.speed_m3, speed.speed_m4};
    if(xMotorMutex != NULL) xSemaphoreTake(xMotorMutex, portMAX_DELAY);  // 获取互斥锁
    I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,Speed_buf,4);
    if(xMotorMutex != NULL) xSemaphoreGive(xMotorMutex);  // 释放互斥锁
}

/**
 * @brief  电机初始化
 * @param  无
 * @retval 无
 */
void Motor_Init(void)
{
    int8_t MotorType = MOTOR_TYPE_JGB;  // 设置电机类型为JGB
    int8_t MotorEncoderPolarity = 0;    // 设置编码器极性为默认值

    I2C_Write_Len(MOTOR_TYPE_ADDR,&MotorType,1);  // 写入电机类型配置
    DelayMs(5);  // 延时等待配置生效
    I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR,&MotorEncoderPolarity,1);  // 写入编码器极性配置
	DelayMs(5);  // 延时等待配置生效
}

/**
 * @brief  获取电池电压
 * @param  无
 * @retval 电压值，单位mV
 */
uint16_t Motor_Get_Vol(void)
{
    uint8_t data[2] = {0x01, 0x02};
    uint16_t result;
    if(xMotorMutex != NULL) xSemaphoreTake(xMotorMutex, portMAX_DELAY);
    uint8_t ret = I2C_Read_Len(ADC_BAT_ADDR,data,2);
    if(ret != 0)
    {
        if(xMotorMutex != NULL) xSemaphoreGive(xMotorMutex);
        return 0;
    }
    result = (data[0]|(data[1]<<8));  // 拼接高低字节为电压值
    if(xMotorMutex != NULL) xSemaphoreGive(xMotorMutex);
    return result;
}

/**
 * @brief  获取电机编码器值
 * @param  Encoder: 电机编码器结构体指针，用于存储4个电机的编码器值
 * @retval 无
 */
void Motor_Get_Encoder(motor_encoder_t* Encoder)
{
    uint8_t buf[16];
    if(xMotorMutex != NULL) xSemaphoreTake(xMotorMutex, portMAX_DELAY);
    I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR,buf,16);

    // 将4字节小端数据拼接为32位有符号编码器值
    Encoder->encoder_m1 = (int32_t)(buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24));
    Encoder->encoder_m2 = (int32_t)(buf[4] | (buf[5]<<8) | (buf[6]<<16) | (buf[7]<<24));
    Encoder->encoder_m3 = (int32_t)(buf[8] | (buf[9]<<8) | (buf[10]<<16) | (buf[11]<<24));
    Encoder->encoder_m4 = (int32_t)(buf[12] | (buf[13]<<8) | (buf[14]<<16) | (buf[15]<<24));
    if(xMotorMutex != NULL) xSemaphoreGive(xMotorMutex);
}

/**
 * @brief  重置电机编码器计数
 * @param  无
 * @retval 无
 */
void Motor_Reset_Encoder(void)
{
    int8_t EncodeReset[16]={0};
    if(xMotorMutex != NULL) xSemaphoreTake(xMotorMutex, portMAX_DELAY);
    I2C_Write_Len(MOTOR_ENCODER_TOTAL_ADDR,EncodeReset,16);
    if(xMotorMutex != NULL) xSemaphoreGive(xMotorMutex);
}



/**
 * @brief  麦克纳姆轮运动控制，根据速度分量计算各轮速度
 * @param  Vx: 前进速度
 * @param  Vy: 横移速度
 * @param  W:  旋转量
 * @param  Vz: 附加速度
 * @retval 无
 */
void Motion_Ctrl(int Vx, int Vy, int W, int Vz)
{
    motor_speed_t speed;
    int temp;

    // 麦克纳姆轮逆运动学公式
    // m1(右前)  m2(左前)
    // m3(右后)  m4(左后)
    temp = -W + Vy + Vx;
    speed.speed_m1 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);  // 限幅 [-100, 100]

    temp = W + Vy + Vx;
    speed.speed_m2 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);

    temp = -W - Vy + Vx;
    speed.speed_m3 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);

    temp = W - Vy + Vx;
    speed.speed_m4 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);

    Motor_Set_Speed(speed);
}
