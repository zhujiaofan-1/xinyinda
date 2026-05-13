#include "motor.h"
#include "Headfile.h"
#include <stdint.h>

/**
 * @brief  设置电机速度
 * @param  speed: 电机速度结构体，包含4个电机的速度值
 * @retval 无
 */
void Motor_Set_Speed(motor_speed_t speed)
{
    // 将4个电机的速度值打包到缓冲区
    int8_t Speed_buf[4] = {speed.speed_m1, speed.speed_m2, speed.speed_m3, speed.speed_m4};
    // 通过I2C批量写入电机速度寄存器
    I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,Speed_buf,4);
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
    // 通过I2C读取电池电压寄存器（2字节）
    uint8_t ret = I2C_Read_Len(ADC_BAT_ADDR,data,2);
    if(ret != 0)  // 读取失败返回0
    {
        return 0;
    }
    // 将低字节和高字节组合成16位电压值（单位：0.1mV）
    return (data[0]|(data[1]<<8))	;
}

/**
 * @brief  获取电机编码器值
 * @param  Encoder: 电机编码器结构体指针，用于存储4个电机的编码器值
 * @retval 无
 */
void Motor_Get_Encoder(motor_encoder_t* Encoder)
{
    uint8_t buf[16];
    // 通过I2C读取16字节编码器数据（每个电机4字节，共4个电机）
    I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR,buf,16);

    // 将每个电机的4字节数据组合成32位有符号整数（小端序）
    Encoder->encoder_m1 = (int32_t)(buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24));
    Encoder->encoder_m2 = (int32_t)(buf[4] | (buf[5]<<8) | (buf[6]<<16) | (buf[7]<<24));
    Encoder->encoder_m3 = (int32_t)(buf[8] | (buf[9]<<8) | (buf[10]<<16) | (buf[11]<<24));
    Encoder->encoder_m4 = (int32_t)(buf[12] | (buf[13]<<8) | (buf[14]<<16) | (buf[15]<<24));
}

/**
 * @brief  重置电机编码器计数
 * @param  无
 * @retval 无
 */
void Motor_Reset_Encoder(void)
{
    int8_t EncodeReset[16]={0};   								//用于重置脉冲值
    I2C_Write_Len(MOTOR_ENCODER_TOTAL_ADDR,EncodeReset,16);		//重置脉冲值
}



// 麦克纳姆轮运动控制
// Vx: 前进速度, Vy: 横移速度(巡线不用), W: 旋转量(PID输出), Vz: 附加速度
void Motion_Ctrl(int Vx, int Vy, int W, int Vz)
{
    motor_speed_t speed;
    int temp;
    
    // 麦克纳姆轮逆运动学公式（根据你的电机布局）
    // m1(A右前)  m2(B左前)
    // m3(A右后)  m4(B左后)
    temp = Vx - Vy + W;
    speed.speed_m1 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);  // 右前
    
    temp = Vx + Vy - W;
    speed.speed_m2 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);  // 左前
    
    temp = Vx + Vy + W;
    speed.speed_m3 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);  // 右后
    
    temp = Vx - Vy - W;
    speed.speed_m4 = (temp > 100) ? 100 : ((temp < -100) ? -100 : temp);  // 左后
    
    Motor_Set_Speed(speed);
}
