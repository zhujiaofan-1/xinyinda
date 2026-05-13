#include "Servo.h"
#include "Headfile.h"

/**
 * @brief  舵机初始化
 * @param  无
 * @retval 无
 */
void Servo_Init(void)
{
    // 启动TIM4定时器基频，TIM4配置为50Hz PWM频率（周期20ms）
    // 预分频器84-1，计数周期84MHz/84=1MHz，自动重装载值20000-1，得到20ms周期
    HAL_TIM_Base_Start(&htim4);
    // 启动PWM输出通道3，舵机回到中间位置（90度）
    // 通道3对应引脚输出PWM波形控制舵机角度
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    Servo_SetAngle(90);
}

/**
 * @brief  设置舵机角度
 * @param  angle: 舵机角度，范围0-180度
 * @retval 无
 */
void Servo_SetAngle(uint16_t angle)
{
    // 限制角度范围在0-180度之间，防止超出舵机物理极限
    if (angle > 180) angle = 180;
    // PWM脉宽计算公式：0度对应0.5ms(500us)，180度对应2.5ms(2500us)
    // 线性插值：compare = 500 + angle * (2500-500) / 180 = 500 + angle * 2000 / 180
    // TIM3计数频率1MHz，每个计数值1us，所以500=0.5ms，2500=2.5ms
    uint16_t compare = 500 + angle * 2000 / 180;
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, compare);
}
