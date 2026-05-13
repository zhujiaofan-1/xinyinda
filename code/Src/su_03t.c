#include "su_03t.h"
#include "Headfile.h"
#include "string.h"
#include "stdint.h"
#include "task.h"
#include "freertos.h"

#define UART2_RECV_LEN          5

TaskHandle_t xUART2_Recv_Task_Handle; // UART2接收任务句柄

static uint8_t UART2_Recv_dma_buf[UART2_RECV_LEN] = {0};

static uint8_t UART2_Recv_buf[UART2_RECV_LEN] = {0};
static uint16_t UART2_recv_len = 0;

/**
 * @brief  SU03T模块初始化
 * @param  无
 * @retval 无
 */
void su03t_Init(void)
{
    // 创建发送任务：周期性发送数据到SU03T语音模块
    xTaskCreate(su03t_Send_Task, "su03t_Send_Task", 256, NULL, 1, NULL);
    // 创建接收任务：通过DMA接收SU03T返回的语音识别结果
    // 将任务句柄保存到xUART2_Recv_Task_Handle，用于中断中发送通知
    xTaskCreate(su03t_Receive_Task, "su03t_Receive_Task", 256, NULL, 1, &xUART2_Recv_Task_Handle);

    // 启动DMA接收，将UART2接收到的数据直接存储到DMA缓冲区
    // DMA方式不需要CPU干预，数据自动搬运到指定内存
    HAL_UART_Receive_DMA(&huart2, UART2_Recv_dma_buf, UART2_RECV_LEN);
    // 开启串口空闲中断：当一帧数据接收完成后（总线空闲超过1字节时间）触发中断
    // 在中断中发送任务通知，唤醒接收任务处理数据
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/**
 * @brief  处理UART2接收到的数据
 * @param  data: 接收到的数据缓冲区指针
 * @retval 无
 */
void Process_UART2_Recv_Data(uint8_t* data)
{
    // 处理数据
    // ...
}

/**
 * @brief  SU03T发送任务
 * @param  pvParameters: 任务参数（未使用）
 * @retval 无
 */
void su03t_Send_Task(void *pvParameters)
{
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief  SU03T接收任务
 * @param  pvParameters: 任务参数（未使用）
 * @retval 无
 */
void su03t_Receive_Task(void *pvParameters)
{
    while(1)
    {
        // 等待任务通知（阻塞状态，不占用CPU）
        // ulTaskNotifyTake会挂起当前任务，直到收到通知
        // pdTRUE表示取出通知后清零，portMAX_DELAY表示无限期等待
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 计算实际接收到的数据长度：总长度 - DMA剩余计数
        // DMA计数器递减，初始值为UART2_RECV_LEN，接收数据后剩余未接收的字节数
        UART2_recv_len = UART2_RECV_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
		
		// 将DMA缓冲区数据拷贝到接收缓冲区
		// DMA缓冲区是循环使用的，需要及时拷贝出来避免被覆盖
		memcpy(UART2_Recv_buf, UART2_Recv_dma_buf, UART2_recv_len);
		
		// 进入数据处理函数，解析语音识别结果
		Process_UART2_Recv_Data(UART2_Recv_buf);
		
		// 清空DMA缓冲区和接收缓冲区，准备接收下一帧数据
		// 清零避免残留数据干扰下次接收
		memset(UART2_Recv_dma_buf, 0, UART2_RECV_LEN);
		memset(UART2_Recv_buf, 0, UART2_RECV_LEN);
        
		// 重新启动 DMA 接收，准备接收下一帧数据
		// 每次处理完数据后必须重新开启DMA，否则会丢失后续数据
        HAL_UART_Receive_DMA(&huart2, UART2_Recv_dma_buf, UART2_RECV_LEN);
        
        // 延时1秒，给系统留出处理时间
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
