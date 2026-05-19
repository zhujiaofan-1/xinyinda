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

extern int16_t g_turn_encoder_90;
extern int16_t g_turn_encoder_180;
extern int16_t g_turn_encoder_straight;
extern int16_t g_translate_encoder_target;

void USART1_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xBT_Recv_Task_Handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }
    HAL_UART_IRQHandler(&huart1);
}

void Bluetooth_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    xTaskCreate(Bluetooth_Recv_Task, "BT_Recv_Task", 256, NULL, 1, &xBT_Recv_Task_Handle);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)BT_Recv_buf, BT_RECV_LEN);
}

void Bluetooth_Send_String(const char *str)
{
    uint16_t len = strlen(str);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, HAL_MAX_DELAY);
}

void Bluetooth_Parse_Packet(char *packet)
{
    char *Tag = strtok(packet, ",");
    if(Tag == NULL) return;

    if(strcmp(Tag, "slider") == 0)
    {
        char *Name = strtok(NULL, ",");
        char *Value = strtok(NULL, ",");
        if(Name == NULL || Value == NULL) return;

        int16_t val = (int16_t)atoi(Value);

        if(strcmp(Name, "Turn90") == 0)
        {
            g_turn_encoder_90 = val;
        }
        else if(strcmp(Name, "Turn180") == 0)
        {
            g_turn_encoder_180 = val;
        }
        else if(strcmp(Name, "TurnStraight") == 0)
        {
            g_turn_encoder_straight = val;
        }
        else if(strcmp(Name, "TranslateTarget") == 0)
        {
            g_translate_encoder_target = val;
        }
    }
}

void Bluetooth_Recv_Task(void *pvParameters)
{
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        BT_recv_len = BT_RECV_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        if(BT_recv_len > 0)
        {
            BT_Recv_buf[BT_recv_len] = '\0';

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

            memset(BT_Recv_buf, 0, BT_RECV_LEN);
        }

        HAL_UART_Receive_DMA(&huart1, (uint8_t *)BT_Recv_buf, BT_RECV_LEN);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }
}
