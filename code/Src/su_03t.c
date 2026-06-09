#include "su_03t.h"
#include "Headfile.h"
#include "string.h"
#include "stdint.h"
#include "task.h"
#include "freertos.h"
#include "Turn.h"
#include "usart.h"

#define UART2_RECV_LEN          5

TaskHandle_t xUART2_Recv_Task_Handle; // UART2接收任务句柄

static uint8_t UART2_Recv_dma_buf[UART2_RECV_LEN] = {0};

uint8_t g_su03t_recv_data[UART2_RECV_LEN] = {0};  // SU03T接收数据，供LCD显示
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

    // 导航去房间时不接收新指令，返航和空闲时可以接收
    if (nav_state == NAV_GOING) {
        return;
    }

    // ASCII数字格式：'0'~'5' 直接对应房间号
    if (data[0] >= '0' && data[0] <= '5') {
        new_room = data[0] - '0';
    }
    else
    {
        // SU03T语音模块自定义编码格式
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
    }

    if (new_room != 0) {
        Turn_GoRoom(new_room);          // 导航到指定房间
        char msg[] = "\r\nGo to room: X\r\n";
        msg[13] = new_room + '0';
        UART_Send_String(msg);
    } else {
        Turn_GoRoom(0);                 // 返航回起点
        UART_Send_String("\r\nGo to room: RETURN HOME\r\n");
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 阻塞等待空闲中断通知

        // 计算实际接收数据长度
        UART2_recv_len = UART2_RECV_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

		memcpy(g_su03t_recv_data, UART2_Recv_dma_buf, UART2_recv_len);  // 拷贝到全局缓冲区供LCD显示

		Process_UART2_Recv_Data(g_su03t_recv_data);  // 解析并执行语音指令

		memset(UART2_Recv_dma_buf, 0, UART2_RECV_LEN);  // 清空DMA缓冲区

        // 重新启动DMA接收和空闲中断
        HAL_UART_Receive_DMA(&huart2, UART2_Recv_dma_buf, UART2_RECV_LEN);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    }
}
