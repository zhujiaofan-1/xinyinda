#include "Headfile.h"
#include "HCSR04.h"

volatile float g_ultrasonic_distance = -1.0f;
static volatile uint8_t capture_done = 1;  // 初始为1，允许首次触发
static volatile uint32_t capture_value = 0;

/**
 * @brief  TIM5输入捕获回调，CH2下降沿捕获完成时读取脉宽值
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        // CH2捕获到下降沿，读取CH2的捕获值即为ECHO高电平时间(us)
        capture_value = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);
        capture_done = 1;
    }
}

/**
 * @brief  触发一次超声波测距（非阻塞）
 *         由任务周期性调用，结果存入 g_ultrasonic_distance
 */
void HCSR04_Trigger(void)
{
    if(capture_done == 0) return; // 上次测量未完成，跳过

    capture_done = 0;

    // 发送10us以上高电平触发信号
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    DelayUs(5);
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    DelayUs(15);
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  获取超声波测距距离值
 * @retval 距离值，单位cm，失败时返回-1.0f
 */
float HCSR04_Get_Distance(void)
{
    if(capture_done && capture_value > 0)
    {
        // 距离 = (时间us * 0.034) / 2
        g_ultrasonic_distance = (capture_value * 0.034f) / 2.0f;
    }
    return g_ultrasonic_distance;
}

/**
 * @brief  超声波模块初始化，启动TIM5输入捕获
 * @retval 无
 */
void HCSR04_Init(void)
{
    MX_TIM5_Init();
    // 启动PWM输入模式：CH1捕获上升沿(复位计数器)，CH2捕获下降沿
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
}
