#include "su_03t.h"
#include "Headfile.h"
#include "string.h"
#include "stdint.h"
#include "task.h"
#include "freertos.h"
#include "Turn.h"
#include "usart.h"

extern volatile uint8_t system_mode;

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

    xTaskCreate(su03t_Receive_Task, "su03t_Receive_Task", 256, NULL, 1, &xUART2_Recv_Task_Handle);

    // 启动DMA接收，将UART2接收到的数据直接存储到DMA缓冲区

    HAL_UART_Receive_DMA(&huart2, UART2_Recv_dma_buf, UART2_RECV_LEN);
    // 开启串口空闲中断：当一帧数据接收完成后（总线空闲超过1字节时间）触发中断

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/**
 * @brief  处理UART2接收到的数据
 * @param  data: 接收到的数据缓冲区指针
 * @retval 无
 */
void Process_UART2_Recv_Data(uint8_t* data)
{
    uint16_t new_room = 0;

    if (data[0] == 'w' || data[0] == 'W') {
        system_mode = 0;
        UART_Send_String("\r\nSwitched to WRITE MODE\r\n");
        return;
    } else if (data[0] == 'n' || data[0] == 'N') {
        system_mode = 1;
        UART_Send_String("\r\nSwitched to NAVIGATION MODE\r\n");
        return;
    } else if (data[0] == 'h' || data[0] == 'H') {
        UART_Send_String("\r\n=== Help ===\r\n");
        UART_Send_String("w - Switch to Write Mode\r\n");
        UART_Send_String("n - Switch to Navigation Mode\r\n");
        UART_Send_String("h - Show this help\r\n");
        UART_Send_String("============\r\n");
        return;
    }

    switch ((uint8_t)data[0])
    {
    case 0x1A:
        new_room = 1;
        break;
    
    case 0x2B:
        new_room = 2;
        break;

    case 0x3C:
        new_room = 3;
        break;
    
    case 0x4D:
        new_room = 4;
        break;
    
    case 0x5E:
        new_room = 5;
        break;

    case 0x9F:
        new_room = 0;
        break;
    
    default:
        return;
    }

    if (nav_state == NAV_IDLE || nav_state == NAV_RETURNING) {
        Turn_GoRoom(new_room);
    }
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        UART2_recv_len = UART2_RECV_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
	
		memcpy(UART2_Recv_buf, UART2_Recv_dma_buf, UART2_recv_len);
		
		Process_UART2_Recv_Data(UART2_Recv_buf);
		
		memset(UART2_Recv_dma_buf, 0, UART2_RECV_LEN);
		memset(UART2_Recv_buf, 0, UART2_RECV_LEN);
        
        HAL_UART_Receive_DMA(&huart2, UART2_Recv_dma_buf, UART2_RECV_LEN);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    }
}
