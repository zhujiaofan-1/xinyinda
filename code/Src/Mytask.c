#include "Mytask.h"
#include "Headfile.h"
#include "Turn.h"
#include "usart.h"

SemaphoreHandle_t xAvoidSemaphore = NULL;

// 系统模式：0=写卡模式，1=导航模式
volatile uint8_t system_mode = 1;  // 默认导航模式

void Avoidance_Semaphore_Init(void)
{
    xAvoidSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xAvoidSemaphore);
}

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
    static float distance;
    static uint8_t avoid_state = AVOID_NORMAL;
    static float left_dist = 0, right_dist = 0;
    static int32_t translate_encoder = 0;  // 记录平移出线时的编码器值，用于回线时精确返回
    static uint8_t obstacle_side = 0;      // 0=障碍物在左边(向右平移避障)，1=障碍物在右边(向左平移避障)
    static motor_encoder_t encoder;
    static int32_t avg_enc;

    while(1)
    {
        if(nav_state != NAV_GOING && nav_state != NAV_RETURNING) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        distance = HCSR04_Get_Distance();
        g_ultrasonic_distance = distance;

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


//巡线任务
void irtracking_Task(void* param)
{

    while(1)
    {
        if((nav_state == NAV_GOING || nav_state == NAV_RETURNING) &&
           xSemaphoreTake(xAvoidSemaphore, 0) == pdPASS)
        {
            LineWalking();
            xSemaphoreGive(xAvoidSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


//RFID导航联动
void car_walking(void *param)
{
  uint8_t uid[5];
  uint8_t last_uid[5] = {0};
  uint8_t status;
  uint8_t i;
  uint8_t same_card;
  uint8_t block_data[16];
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  TurnInfo_t turn_info[MAX_TURN_DATA];
  uint8_t turn_count = 0;
  uint16_t room_num;
  uint8_t is_room_card;
  motor_encoder_t enc;
  int32_t avg_enc;
  
  xNavSemaphore = xSemaphoreCreateBinary();
  
  Turn_Init(DIR_S, 0);
  nav_state = NAV_IDLE;
  
  while(1)
  {
    for (i = 0; i < 5; i++) uid[i] = 0;
    
    status = MFRC522_Request(PICC_REQALL, uid);
    if (status != MI_OK) {
      for (i = 0; i < 4; i++) last_uid[i] = 0;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    status = MFRC522_Anticoll(uid);
    if (status != MI_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    same_card = 1;
    for (i = 0; i < 4; i++) {
      if (uid[i] != last_uid[i]) {
        same_card = 0;
        break;
      }
    }
    if (same_card) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    for (i = 0; i < 4; i++) last_uid[i] = uid[i];
    
    uid[4] = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
    if (MFRC522_SelectTag(uid) == 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    status = MFRC522_Auth(PICC_AUTHENT1A, 1, key, uid);
    if (status != MI_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    status = MFRC522_Read(1, block_data);
    if (status != MI_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    for (i = 0; i < 16; i++) {
      turn_card_content[i] = block_data[i];
      if (block_data[i] == 0) break;
    }
    turn_card_content[i] = '\0';

    // 尝试解析为路口卡(N34S0W2E15格式)
    if (Turn_ParseData(block_data, turn_info, &turn_count) == MI_OK) {
      if (turn_target_room == 0) {
        // 返航模式：计算返回南方的转向
        turn_action = Turn_CalculateReturn(turn_current_dir);
      } else {
        // 正常导航：根据目标房间计算转向
        turn_action = Turn_CalculateTurn(turn_current_dir, turn_target_room, turn_info, turn_count);
      }
      turn_action_valid = 1;

      if (turn_action != TURN_ERROR) {
        turn_current_dir = Turn_UpdateDirection(turn_current_dir, turn_action);
      }
      if(xNavSemaphore != NULL) {
        xSemaphoreGive(xNavSemaphore);
      }
    } else {
      // 非路口卡，检查是否为房间卡(纯数字)
      is_room_card = 1;
      room_num = 0;
      for (i = 0; i < 16 && block_data[i] != 0; i++) {
        if (block_data[i] >= '0' && block_data[i] <= '9') {
          room_num = room_num * 10 + (block_data[i] - '0');
        } else {
          is_room_card = 0;
          break;
        }
      }
      
      if (is_room_card) {
        if (nav_state == NAV_GOING && room_num == turn_target_room) {
          nav_state = NAV_IDLE;
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            Motion_Ctrl(0, 0, 0, 0);
            xSemaphoreGive(xAvoidSemaphore);
          }
          vTaskDelay(pdMS_TO_TICKS(5000));
          
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            Motor_Reset_Encoder();
            Motion_Ctrl(0, 0, -60, 0);
            do {
              Motor_Get_Encoder(&enc);
              avg_enc = (abs(enc.encoder_m1) + abs(enc.encoder_m2)
                       + abs(enc.encoder_m3) + abs(enc.encoder_m4)) / 4;
              vTaskDelay(pdMS_TO_TICKS(10));
            } while(avg_enc < TURN_ENCODER_180);
            Motion_Ctrl(0, 0, 0, 0);
            xSemaphoreGive(xAvoidSemaphore);
          }
          
          turn_current_dir = Turn_UpdateDirection(turn_current_dir, TURN_UTURN);
          turn_target_room = 0;
          nav_state = NAV_RETURNING;
        } else if (nav_state == NAV_RETURNING && room_num == 0) {
          nav_state = NAV_IDLE;
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            Motion_Ctrl(0, 0, 0, 0);
            xSemaphoreGive(xAvoidSemaphore);
          }
        }
      }
    }
    
    MFRC522_ClearBitMask(MFRC522_REG_STATUS2, 0x08);
    MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}




// // ==================== RFID导航任务 ====================
// void CardNavTask(void *argument) {
//   uint8_t uid[5];
//   uint8_t last_uid[5] = {0};
//   uint8_t status;
//   uint8_t i;
//   uint8_t same_card;
//   uint8_t block_data[16];
//   uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//   TurnInfo_t turn_info[MAX_TURN_DATA];
//   uint8_t turn_count = 0;
//   char msg[80];
  
//   vTaskDelay(600);
  
//   Turn_Init(DIR_S, 3);
  
//   UART_Send_String("\r\n==================================\r\n");
//   UART_Send_String("     Navigation Task Started\r\n");
//   UART_Send_String("==================================\r\n");
//   sprintf(msg, "Target Room: %d\r\n", turn_target_room);
//   UART_Send_String(msg);
//   sprintf(msg, "Start Dir: %s\r\n", Turn_DirToString(turn_current_dir));
//   UART_Send_String(msg);
  
//   for(;;) {
//     // 清空UID
//     for (i = 0; i < 5; i++) uid[i] = 0;
    
//     // 第1步：寻卡
//     status = MFRC522_Request(PICC_REQALL, uid);
//     if (status != MI_OK) {
//       for (i = 0; i < 4; i++) last_uid[i] = 0;
//       vTaskDelay(pdMS_TO_TICKS(100));
//       continue;
//     }
    
//     // 第2步：防冲突
//     status = MFRC522_Anticoll(uid);
//     if (status != MI_OK) {
//       vTaskDelay(pdMS_TO_TICKS(100));
//       continue;
//     }
    
//     // 检查是否同一张卡
//     same_card = 1;
//     for (i = 0; i < 4; i++) {
//       if (uid[i] != last_uid[i]) {
//         same_card = 0;
//         break;
//       }
//     }
//     if (same_card) {
//       vTaskDelay(pdMS_TO_TICKS(100));
//       continue;
//     }
    
//     // 更新卡号记录
//     for (i = 0; i < 4; i++) last_uid[i] = uid[i];
    
//     // 第3步：选择卡片
//     uid[4] = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
//     if (MFRC522_SelectTag(uid) == 0) {
//       vTaskDelay(pdMS_TO_TICKS(100));
//       continue;
//     }
    
//     // 第4步：认证
//     status = MFRC522_Auth(PICC_AUTHENT1A, 1, key, uid);
//     if (status != MI_OK) {
//       vTaskDelay(pdMS_TO_TICKS(100));
//       continue;
//     }
    
//     // 第5步：读取Block 1
//     status = MFRC522_Read(1, block_data);
//     if (status != MI_OK) {
//       vTaskDelay(pdMS_TO_TICKS(100));
//       continue;
//     }
    
//     UART_Send_String("\nCard detected!\r\n");
//     UART_Send_UID(uid, 4);
    
//     // 保存卡片内容到LCD显示变量
//     for (i = 0; i < 16; i++) {
//       turn_card_content[i] = block_data[i];
//       if (block_data[i] == 0) break;
//     }
//     turn_card_content[i] = '\0';
    

//     // 解析卡片数据
//     if (Turn_ParseData(block_data, turn_info, &turn_count) == MI_OK) {
//       turn_action = Turn_CalculateTurn(turn_current_dir, turn_target_room, turn_info, turn_count);
//       turn_action_valid = 1;
      
//       sprintf(msg, "Card: %s\r\n", turn_card_content);
//       UART_Send_String(msg);
//       sprintf(msg, "Current Dir: %s\r\n", Turn_DirToString(turn_current_dir));
//       UART_Send_String(msg);
//       sprintf(msg, "Turn: %s\r\n", Turn_TurnToString(turn_action));
//       UART_Send_String(msg);
      
//       if (turn_action != TURN_ERROR) {
//         turn_current_dir = Turn_UpdateDirection(turn_current_dir, turn_action);
//         sprintf(msg, "New Dir: %s\r\n", Turn_DirToString(turn_current_dir));
//         UART_Send_String(msg);
//       }
      
//       if(xNavSemaphore != NULL) {
//         xSemaphoreGive(xNavSemaphore);
//       }
//     } else {
//       UART_Send_String("Parse card data failed\r\n");
//       turn_action_valid = 0;
//     }
    
//     // 清理RC522状态：关闭加密单元，清空FIFO，为读下一张卡做准备
//     MFRC522_ClearBitMask(MFRC522_REG_STATUS2, 0x08);
//     MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);
    
//     vTaskDelay(pdMS_TO_TICKS(1000));
//   }
// }

/**
 * @brief  电机测试任务：依次测试前进、后退、左移、右移、左转、右转
 */
void MotorTestTask(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  for(;;)
  {
    Motion_Ctrl(60, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    Motion_Ctrl(0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    Motion_Ctrl(-60, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    Motion_Ctrl(0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    Motion_Ctrl(0, -60, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    Motion_Ctrl(0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    Motion_Ctrl(0, 60, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    Motion_Ctrl(0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    Motion_Ctrl(0, 0, -60, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));
    Motion_Ctrl(0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    Motion_Ctrl(0, 0, 60, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));
    Motion_Ctrl(0, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

