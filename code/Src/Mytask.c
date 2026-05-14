#include "Mytask.h"
#include "Headfile.h"

SemaphoreHandle_t xAvoidSemaphore = NULL;

// 避障状态机
enum {
    AVOID_NORMAL = 0,      // 正常巡线
    AVOID_CHECK_LEFT,      // 检测左边距离
    AVOID_CHECK_RIGHT,     // 检测右边距离
    AVOID_TRANSLATE_OUT,   // 平移出线
    AVOID_FORWARD,         // 向前走绕过障碍物
    AVOID_FORWARD_EXTRA,   // 再向前走11个编码器值
    AVOID_TRANSLATE_BACK   // 平移回线上
};

#define TRANSLATE_ENCODER_TARGET 200  // 平移出线的编码器目标值，需根据实际调整
#define FORWARD_EXTRA_ENCODER    11   // 走过障碍物后额外前进的编码器值
#define OBSTACLE_DISTANCE        20   // 障碍物判定距离(cm)

void Avoidance_Task(void* param)
{
    Servo_Init();
    HCSR04_Init();
    xAvoidSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xAvoidSemaphore);

    static float distance;
    static uint8_t avoid_state = AVOID_NORMAL;
    static float left_dist = 0, right_dist = 0;
    static int32_t translate_encoder = 0;  // 记录平移出线时的编码器值，用于回线时精确返回
    static uint8_t obstacle_side = 0;      // 0=障碍物在左边(向右平移避障)，1=障碍物在右边(向左平移避障)
    static motor_encoder_t encoder;
    static int32_t avg_enc;

    while(1)
    {
        distance = HCSR04_Get_Distance();

        switch(avoid_state)
        {
            case AVOID_NORMAL:
                if(distance < OBSTACLE_DISTANCE && distance > 0)
                {
                    // 检测到前方障碍物，暂停巡线
                    xSemaphoreTake(xAvoidSemaphore, portMAX_DELAY);
                    Motion_Ctrl(0, 0, 0, 0);
                    // 舵机转左，准备检测左边距离
                    Servo_SetAngle(0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    avoid_state = AVOID_CHECK_LEFT;
                }
                break;

            case AVOID_CHECK_LEFT:
                left_dist = distance;
                // 舵机转右，准备检测右边距离
                Servo_SetAngle(180);
                vTaskDelay(pdMS_TO_TICKS(500));
                avoid_state = AVOID_CHECK_RIGHT;
                break;

            case AVOID_CHECK_RIGHT:
                right_dist = distance;
                Motor_Reset_Encoder();

                // 根据左右距离选择平移方向：哪边距离远就往哪边平移
                if(left_dist > right_dist && left_dist > OBSTACLE_DISTANCE)
                {
                    obstacle_side = 1;  // 障碍物在右边，向左平移
                    Motion_Ctrl(0, 60, 0, 0);
                }
                else if(right_dist > OBSTACLE_DISTANCE)
                {
                    obstacle_side = 0;  // 障碍物在左边，向右平移
                    Motion_Ctrl(0, -60, 0, 0);
                }
                else
                {
                    // 两边都有障碍物，默认向左平移
                    obstacle_side = 1;
                    Motion_Ctrl(0, 60, 0, 0);
                }
                avoid_state = AVOID_TRANSLATE_OUT;
                break;

            case AVOID_TRANSLATE_OUT:               //平移出线
                Motor_Get_Encoder(&encoder);
                // 计算平均编码器值，取绝对值
                avg_enc = (abs(encoder.encoder_m1) + abs(encoder.encoder_m2)
                         + abs(encoder.encoder_m3) + abs(encoder.encoder_m4)) / 4;

                if(avg_enc >= TRANSLATE_ENCODER_TARGET)         //平移出线的编码值大于目标值，说明已平移完成
                {
                    // 记录平移编码器值，回线时使用相同值精确返回
                    translate_encoder = avg_enc;
                    Motion_Ctrl(0, 0, 0, 0);                    // 停车
                    Motor_Reset_Encoder();

                    // 舵机转向障碍物那侧，监测是否已走过障碍物
                    if(obstacle_side == 0)
                        Servo_SetAngle(0);      // 障碍物在左边，舵机转左监测
                    else
                        Servo_SetAngle(180);    // 障碍物在右边，舵机转右监测
                    vTaskDelay(pdMS_TO_TICKS(500));

                    Motion_Ctrl(50, 0, 0, 0);       //向前走
                    avoid_state = AVOID_FORWARD;
                }
                break;

            case AVOID_FORWARD:               //向前走
                // 舵机监测障碍物侧，距离>20cm说明已走过障碍物
                if(distance > OBSTACLE_DISTANCE || distance < 0)
                {
                    Motor_Reset_Encoder();
                    Motion_Ctrl(50, 0, 0, 0);
                    avoid_state = AVOID_FORWARD_EXTRA;
                }
                break;

            case AVOID_FORWARD_EXTRA:           // 过了障碍物，向前走额外距离
                Motor_Get_Encoder(&encoder);
                avg_enc = (abs(encoder.encoder_m1) + abs(encoder.encoder_m2)
                         + abs(encoder.encoder_m3) + abs(encoder.encoder_m4)) / 4;

                if(avg_enc >= FORWARD_EXTRA_ENCODER)         // 前向走额外距离的编码值大于目标值，说明已走完额外距离
                {
                    Motion_Ctrl(0, 0, 0, 0);   
                    Motor_Reset_Encoder();

                    // 反向平移回线上（与出线方向相反）
                    if(obstacle_side == 0)
                        Motion_Ctrl(0, 60, 0, 0);   // 之前向右出线，现在向左回线
                    else
                        Motion_Ctrl(0, -60, 0, 0);  // 之前向左出线，现在向右回线

                    avoid_state = AVOID_TRANSLATE_BACK;
                }
                break;

            case AVOID_TRANSLATE_BACK:               //平移回线
                Motor_Get_Encoder(&encoder);
                avg_enc = (abs(encoder.encoder_m1) + abs(encoder.encoder_m2)
                         + abs(encoder.encoder_m3) + abs(encoder.encoder_m4)) / 4;

                // 使用出线时记录的编码器值，确保精确回到线上
                if(avg_enc >= translate_encoder)        //判断是否回到线上
                {
                    Motion_Ctrl(0, 0, 0, 0);
                    Servo_SetAngle(90);
                    vTaskDelay(pdMS_TO_TICKS(500));

                    // 重置所有状态
                    translate_encoder = 0;
                    left_dist = 0;
                    right_dist = 0;
                    avoid_state = AVOID_NORMAL;
                    // 释放信号量，恢复巡线
                    xSemaphoreGive(xAvoidSemaphore);
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void irtracking_Task(void* param)
{
    uint8_t s1,s2,s3,s4,s5,s6,s7,s8;

    while(1)
    {
        // 非阻塞获取信号量：获取成功执行巡线，获取失败说明正在避障则跳过
        if(xSemaphoreTake(xAvoidSemaphore, 0) == pdPASS)
        {
            LineWalking();
            xSemaphoreGive(xAvoidSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void MFRC522_Task(void *pvParameters)
{
  uint8_t uid[5];
  uint8_t last_uid[5] = {0};
  uint8_t status;
  uint8_t i;
  uint8_t same_card;
  
  for(;;)
  {
    status = MFRC522_ReadIDCard(uid, NULL, NULL);
    
    if (status == MI_OK)
    {
      same_card = 1;
      for (i = 0; i < 4; i++)
      {
        if (uid[i] != last_uid[i])
        {
          same_card = 0;
          break;
        }
      }
      
      if (!same_card)
      {
        for (i = 0; i < 4; i++)
        {
          last_uid[i] = uid[i];
        }
        
        UART_Send_UID(uid, 4);
      }
    }
    else if (status == MI_NOTAGERR)
    {
      for (i = 0; i < 4; i++)
      {
        last_uid[i] = 0;
      }
    }
    
    vTaskDelay(100);
  }
}
