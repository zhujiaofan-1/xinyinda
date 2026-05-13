#include "Headfile.h"
#include "HCSR04.h"

uint16_t TIM_CNT = 0;

/**
 * @brief  获取超声波测距距离值
 * @param  无
 * @retval 距离值，单位cm，失败时返回-1.0f
 */
float HCSR04_Get_Distance(void)
{
    // 步骤1：发送触发信号 - 需要至少10us的高电平脉冲来启动测距
    // 先拉低确保初始状态，然后拉高20us，最后拉低完成触发
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    DelayUs(5);
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    DelayUs(20);
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    // 步骤2：清零定时器计数器，准备测量ECHO高电平持续时间
    // TIM3配置为1MHz计数频率，每个计数值代表1us
    __HAL_TIM_SetCounter(&htim3, 0);

    // 步骤3：等待ECHO引脚变为高电平（模块开始发射超声波）
    // ECHO变高表示超声波已发出，开始计时
    while(HAL_GPIO_ReadPin(HCSR04_GPIO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        // 超时保护：30ms内未检测到高电平说明模块无响应或接线错误
        // 30000us = 30ms，超过此时间认为测距失败
        if(__HAL_TIM_GetCounter(&htim3) > 30000) return -1.0f;
    }

    // 步骤4：重新清零计数器，开始精确测量高电平持续时间
    // 此时超声波正在传播，ECHO保持高电平直到接收到回波
    __HAL_TIM_SetCounter(&htim3, 0);

    // 步骤5：等待ECHO引脚变为低电平（超声波回波接收完毕）
    // 高电平持续时间 = 超声波往返时间
    while(HAL_GPIO_ReadPin(HCSR04_GPIO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        // 超时保护：30ms对应最大测量距离约5米（超出模块规格）
        // 超时跳出避免程序卡死
        if(__HAL_TIM_GetCounter(&htim3) > 30000) break;
    }

    // 步骤6：获取定时器计数值，即超声波往返时间（单位：us）
    TIM_CNT = __HAL_TIM_GetCounter(&htim3);

    // 步骤7：距离计算公式推导：
    // 声速 = 340m/s = 34000cm/s = 0.034cm/us
    // 距离 = (时间 × 声速) / 2  （除以2因为是往返时间）
    // 例如：1000us对应距离 = (1000 × 0.034) / 2 = 17cm
    float distance = (TIM_CNT * 0.034f) / 2.0f;

    // 步骤8：延时100ms给模块恢复时间
    // HCSR04测量周期建议大于60ms，避免前后测量干扰
    vTaskDelay(pdMS_TO_TICKS(100));

    return distance;
}
