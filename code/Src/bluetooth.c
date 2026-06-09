#include "bluetooth.h"
#include "Headfile.h"
#include "freertos.h"
#include "task.h"
#include "string.h"
#include "Turn.h"
#include "usart.h"
#include "stdlib.h"
#include "stdio.h"

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

TaskHandle_t xBT_Recv_Task_Handle;

static char BT_Recv_buf[BT_RECV_LEN] = {0};
char BT_Packet_buf[BT_RECV_LEN] = {0};
static uint16_t BT_recv_len = 0;

extern int16_t g_turn_delay_90;
extern int16_t g_turn_delay_180;
extern int16_t g_translate_encoder_target;
extern int16_t g_forward_encoder;
extern int16_t g_obstacle_distance;
extern int16_t g_ir_speed;
extern int16_t g_ir_pid_kp;
extern int16_t g_ir_pid_ki;
extern int16_t g_ir_pid_kd;

/**
 * @brief  USART1中断处理函数，检测空闲中断并通知接收任务
 * @param  无
 * @retval 无
 */
void USART1_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);   // 清除空闲标志
        HAL_UART_DMAStop(&huart1);            // 停止DMA接收
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xBT_Recv_Task_Handle, &xHigherPriorityTaskWoken);  // 唤醒接收任务
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // 必要时触发任务切换
        return;
    }
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief  蓝牙模块初始化，配置USART1、DMA、GPIO及接收任务
 * @param  无
 * @retval 无
 */
void Bluetooth_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();   // 使能USART1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();    // 使能GPIOA时钟
    __HAL_RCC_DMA2_CLK_ENABLE();     // 使能DMA2时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;  // PA9:TX  PA10:RX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置DMA2_Stream2用于USART1接收
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart1_rx);

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);  // 关联DMA到USART1

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    // 配置USART1：9600波特率，8N1
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // 使能空闲中断

    xTaskCreate(Bluetooth_Recv_Task, "BT_Recv_Task", 256, NULL, 1, &xBT_Recv_Task_Handle);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)BT_Recv_buf, BT_RECV_LEN);  // 启动DMA接收
}

/**
 * @brief  通过蓝牙发送字符串
 * @param  str: 待发送的字符串指针
 * @retval 无
 */
void Bluetooth_Send_String(const char *str)
{
    uint16_t len = strlen(str);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, HAL_MAX_DELAY);
}

/**
 * @brief  解析蓝牙数据包，格式为 "slider,名称,数值"
 * @param  packet: 已提取的数据包字符串（不含方括号）
 * @retval 无
 */
void Bluetooth_Parse_Packet(char *packet)
{
    char *Tag = strtok(packet, ",");   // 提取指令类型
    if(Tag == NULL) return;

    if(strcmp(Tag, "slider") == 0)
    {
        char *Name = strtok(NULL, ",");   // 提取参数名
        char *Value = strtok(NULL, ",");  // 提取参数值
        if(Name == NULL || Value == NULL) return;

        int16_t val = (int16_t)atoi(Value);

        // 根据参数名更新对应的全局变量
        if(strcmp(Name, "Turn90") == 0)
        {
            g_turn_delay_90 = val;
        }
        else if(strcmp(Name, "Turn180") == 0)
        {
            g_turn_delay_180 = val;
        }
        else if(strcmp(Name, "TranslateTarget") == 0)
        {
            g_translate_encoder_target = val;
        }
        else if(strcmp(Name, "ObstacleDist") == 0)
        {
            g_obstacle_distance = val;
        }
        else if(strcmp(Name, "ForwardEnc") == 0)
        {
            g_forward_encoder = val;
        }
        else if(strcmp(Name, "IRSpeed") == 0)
        {
            g_ir_speed = val;
        }
        else if(strcmp(Name, "IRKp") == 0)
        {
            g_ir_pid_kp = val;
        }
        else if(strcmp(Name, "IRKi") == 0)
        {
            g_ir_pid_ki = val;
        }
        else if(strcmp(Name, "IRKd") == 0)
        {
            g_ir_pid_kd = val;
        }
        else if(strcmp(Name, "GoRoom") == 0)
        {
            if(val >= 0 && val <= 9)
            {
                Turn_GoRoom((uint16_t)val);  // 执行导航到指定房间
            }
        }
    }
}

/**
 * @brief  蓝牙接收任务，等待空闲中断通知后提取并解析数据包
 * @param  pvParameters: 任务参数（未使用）
 * @retval 无
 */
void Bluetooth_Recv_Task(void *pvParameters)
{
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 阻塞等待中断通知

        // 计算实际接收到的数据长度
        BT_recv_len = BT_RECV_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        if(BT_recv_len > 0)
        {
            BT_Recv_buf[BT_recv_len] = '\0';  // 添加字符串结束符

            // 提取方括号内的数据包内容
            char *start = strchr(BT_Recv_buf, '[');
            char *end = strchr(BT_Recv_buf, ']');

            if(start != NULL && end != NULL && end > start)
            {
                uint16_t len = end - start - 1;
                if(len < BT_RECV_LEN)
                {
                    strncpy(BT_Packet_buf, start + 1, len);
                    BT_Packet_buf[len] = '\0';
                    Bluetooth_Parse_Packet(BT_Packet_buf);
                }
            }

            memset(BT_Recv_buf, 0, BT_RECV_LEN);  // 清空接收缓冲区
        }

        // 重新启动DMA接收和空闲中断
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)BT_Recv_buf, BT_RECV_LEN);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }
}
