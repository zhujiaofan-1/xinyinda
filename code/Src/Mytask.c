#include "Mytask.h"
#include "Headfile.h"
#include "Turn.h"
#include "usart.h"

SemaphoreHandle_t xAvoidSemaphore = NULL;

/**
 * @brief  初始化避障信号量，初始状态为可用
 */
void Avoidance_Semaphore_Init(void)
{
    xAvoidSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xAvoidSemaphore);  // 初始释放，允许巡线任务运行
}

// 避障状态机
enum {
    AVOID_NORMAL = 0,      // 正常巡线
    AVOID_CHECK_FRONT,     // 检测前方距离
    AVOID_CHECK_LEFT,      // 检测左边距离
    AVOID_CHECK_RIGHT,     // 检测右边距离
    AVOID_DECIDE,          // 三个方向测完，判断方向
    AVOID_TRANSLATE_OUT,   // 平移出线
    AVOID_FORWARD,         // 向前走绕过障碍物
    AVOID_TRANSLATE_BACK,  // 平移回线上
    AVOID_ABORT            // RFID转向中止避障
};

int16_t g_translate_encoder_target = 1000;  // 平移出/入线的编码器目标值
int16_t g_forward_encoder = 4000;           // 避障前进的编码器值，可通过蓝牙调参
int16_t g_obstacle_distance = 40;   // 障碍物判定距离(cm)

/**
 * @brief  避障任务，基于超声波测距的状态机实现绕障逻辑
 * @param  param FreeRTOS任务参数（未使用）
 * @retval 无
 */
void Avoidance_Task(void* param)
{
    static float distance;
    static uint8_t avoid_state = AVOID_NORMAL;
    static float front_dist = 0, left_dist = 0, right_dist = 0;
    static uint8_t obstacle_side = 0;  // 0=右侧空间大(向右平移), 1=左侧空间大(向左平移)
    static motor_encoder_t encoder;
    static int32_t avg_enc;

    while(1)
    {
        HCSR04_Trigger();  // 触发超声波测距
        vTaskDelay(pdMS_TO_TICKS(100));
        distance = HCSR04_Get_Distance();
        g_ultrasonic_distance = distance;

        // RFID转向优先：如果nav_state变为NAV_IDLE（RFID识别停车），中止避障
        if(nav_state == NAV_IDLE && avoid_state != AVOID_NORMAL)
        {
            avoid_state = AVOID_ABORT;
        }

        if(nav_state != NAV_GOING && nav_state != NAV_RETURNING) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;  // 非导航状态，跳过避障处理
        }

        switch(avoid_state)
        {
            case AVOID_NORMAL:
                if(distance < g_obstacle_distance && distance > 0)
                {
                    xSemaphoreTake(xAvoidSemaphore, portMAX_DELAY);  // 获取信号量，阻止巡线
                    if(nav_state != NAV_GOING && nav_state != NAV_RETURNING) {
                        xSemaphoreGive(xAvoidSemaphore);
                        break;
                    }
                    Motion_Ctrl(0, 0, 0, 0);  // 停车
                    Servo_SetAngle(90);  // 舵机回正，准备测前方距离
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    avoid_state = AVOID_CHECK_FRONT;
                }
                break;

            case AVOID_CHECK_FRONT:
                front_dist = distance;
                Servo_SetAngle(0);   // 舵机转左，测左方距离
                vTaskDelay(pdMS_TO_TICKS(1000));
                avoid_state = AVOID_CHECK_LEFT;
                break;

            case AVOID_CHECK_LEFT:
                left_dist = distance;
                Servo_SetAngle(180);  // 舵机转右，测右方距离
                vTaskDelay(pdMS_TO_TICKS(1000));
                avoid_state = AVOID_CHECK_RIGHT;
                break;

            case AVOID_CHECK_RIGHT:
                right_dist = distance;
                Servo_SetAngle(90);   // 舵机回正
                vTaskDelay(pdMS_TO_TICKS(1000));
                avoid_state = AVOID_DECIDE;
                break;

            case AVOID_DECIDE:
                // 三方围堵，无法绕行
                if(front_dist < g_obstacle_distance && left_dist < g_obstacle_distance && right_dist < g_obstacle_distance)
                {
                    Motion_Ctrl(0, 0, 0, 0);  // 三方围堵，无法绕行，停车
                    avoid_state = AVOID_NORMAL;
                    xSemaphoreGive(xAvoidSemaphore);
                    break;
                }

                Motor_Reset_Encoder();  // 重置编码器，用于平移距离计数

                // 选择空间更大的一侧平移
                if(left_dist > right_dist && left_dist > g_obstacle_distance)
                {
                    obstacle_side = 1;  // 左侧空间大，向左平移
                    Motion_Ctrl(0, 60, 0, 0);
                }
                else
                {
                    obstacle_side = 0;  // 右侧空间大，向右平移
                    Motion_Ctrl(0, -60, 0, 0);
                }
                avoid_state = AVOID_TRANSLATE_OUT;
                break;

            case AVOID_TRANSLATE_OUT:
                Motor_Get_Encoder(&encoder);
                avg_enc = (abs(encoder.encoder_m1) + abs(encoder.encoder_m2)
                         + abs(encoder.encoder_m3) + abs(encoder.encoder_m4)) / 4;  // 四轮平均编码器值

                if(avg_enc >= g_translate_encoder_target)  // 平移到位
                {
                    Motion_Ctrl(0, 0, 0, 0);
                    Motor_Reset_Encoder();
                    Motion_Ctrl(50, 0, 0, 0);
                    avoid_state = AVOID_FORWARD;
                }
                break;

            case AVOID_FORWARD:
                Motor_Get_Encoder(&encoder);
                avg_enc = (abs(encoder.encoder_m1) + abs(encoder.encoder_m2)
                         + abs(encoder.encoder_m3) + abs(encoder.encoder_m4)) / 4;

                if(avg_enc >= g_forward_encoder)  // 前进到位
                {
                    Motion_Ctrl(0, 0, 0, 0);
                    Motor_Reset_Encoder();

                    // 反方向平移回线
                    if(obstacle_side == 0)
                        Motion_Ctrl(0, 60, 0, 0);
                    else
                        Motion_Ctrl(0, -60, 0, 0);

                    avoid_state = AVOID_TRANSLATE_BACK;
                }
                break;

            case AVOID_TRANSLATE_BACK:
                Motor_Get_Encoder(&encoder);
                avg_enc = (abs(encoder.encoder_m1) + abs(encoder.encoder_m2)
                         + abs(encoder.encoder_m3) + abs(encoder.encoder_m4)) / 4;

                if(avg_enc >= g_translate_encoder_target)  // 平移回线到位
                {
                    Motion_Ctrl(0, 0, 0, 0);
                    front_dist = 0;
                    left_dist = 0;
                    right_dist = 0;
                    avoid_state = AVOID_NORMAL;
                    xSemaphoreGive(xAvoidSemaphore);
                }
                break;

            case AVOID_ABORT:
                Motion_Ctrl(0, 0, 0, 0);  // 停车
                Servo_SetAngle(90);        // 舵机回正
                front_dist = 0;            // 重置测距数据
                left_dist = 0;
                right_dist = 0;
                avoid_state = AVOID_NORMAL;
                xSemaphoreGive(xAvoidSemaphore);  // 释放信号量，恢复巡线
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/**
 * @brief  巡线任务，通过红外传感器检测黑线并控制小车沿线路行驶
 * @param  param FreeRTOS任务参数（未使用）
 * @retval 无
 */
//巡线任务
void irtracking_Task(void* param)
{

    while(1)
    {
        // 转向时nav_state=NAV_IDLE，巡线自动停止，避免与转向冲突
        if((nav_state == NAV_GOING || nav_state == NAV_RETURNING) &&
           xSemaphoreTake(xAvoidSemaphore, 0) == pdPASS)
        {
            if(LineWalking() != 0) {
                // 无有效黑线（不在地图上），停车
                Motion_Ctrl(0, 0, 0, 0);
            }
            xSemaphoreGive(xAvoidSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/**
 * @brief  RFID导航联动任务，读取RFID卡信息实现路口转向和房间识别
 * @param  param FreeRTOS任务参数（未使用）
 * @retval 无
 */
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
  
  Turn_Init(DIR_S, 0);
  nav_state = NAV_IDLE;      // 停车等待语音/蓝牙指令
  
  while(1)
  {
    for (i = 0; i < 5; i++) uid[i] = 0;
    
    status = MFRC522_Request(PICC_REQALL, uid);  // 寻卡
    if (status != MI_OK) {
      for (i = 0; i < 4; i++) last_uid[i] = 0;  // 无卡时重置上次卡号，允许重试
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    status = MFRC522_Anticoll(uid);  // 防冲突
    if (status != MI_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    same_card = 1;
    for (i = 0; i < 4; i++) {  // 检查是否与上次同一张卡，防止重复触发
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
    
    uid[4] = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];  // 计算校验字节
    if (MFRC522_SelectTag(uid) == 0) {  // 选卡
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    status = MFRC522_Auth(PICC_AUTHENT1A, 1, key, uid);  // 密钥认证
    if (status != MI_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    status = MFRC522_Read(1, block_data);  // 读取Block 1数据
    if (status != MI_OK) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    for (i = 0; i < 16; i++) {
      turn_card_content[i] = block_data[i];  // 保存卡片内容用于LCD显示
      if (block_data[i] == 0) break;
    }
    turn_card_content[i] = '\0';

    // 尝试解析为路口卡(N34S0W2E15格式)
    if (Turn_ParseData(block_data, turn_info, &turn_count) == MI_OK) {
      if (nav_state == NAV_IDLE) {
        nav_state = NAV_GOING;
      }

      // 保存当前导航状态，转向后恢复
      uint8_t prev_nav_state = nav_state;

      if (nav_state == NAV_RETURNING) {
        // 返航：根据当前方向计算返回起点的转向
        turn_action = Turn_CalculateReturn(turn_current_dir);
      } else if (turn_target_room == 0) {
        // 还没设置目标房间，直行通过路口
        turn_action = TURN_STRAIGHT;
      } else {
        // 前往目标房间：根据路口卡信息计算转向
        turn_action = Turn_CalculateTurn(turn_current_dir, turn_target_room, turn_info, turn_count);
      }
      turn_action_valid = 1;

        // 识别到RFID，立即停车
        nav_state = NAV_IDLE;
        vTaskDelay(pdMS_TO_TICKS(50));  // 等待巡线任务释放信号量
        Motion_Ctrl(0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // 停车等待1s

        if (turn_action != TURN_ERROR && turn_action != TURN_STRAIGHT) {  // 需要转向
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(2000)) == pdPASS) {
            uint16_t turn_delay = 0;
            motor_speed_t turn_speed;

            switch(turn_action)
            {
              case TURN_LEFT:
                turn_speed.speed_m1 = 60;
                turn_speed.speed_m2 = -60;
                turn_speed.speed_m3 = 60;
                turn_speed.speed_m4 = -60;
                turn_delay = g_turn_delay_90;
                break;
              case TURN_RIGHT:
                turn_speed.speed_m1 = -60;
                turn_speed.speed_m2 = 60;
                turn_speed.speed_m3 = -60;
                turn_speed.speed_m4 = 60;
                turn_delay = g_turn_delay_90;
                break;
              case TURN_UTURN:
                turn_speed.speed_m1 = 60;
                turn_speed.speed_m2 = -60;
                turn_speed.speed_m3 = 60;
                turn_speed.speed_m4 = -60;
                turn_delay = g_turn_delay_180;
                break;
            }

            Motor_Set_Speed(turn_speed);
            vTaskDelay(pdMS_TO_TICKS(turn_delay));  // 等待转向完成
            Motion_Ctrl(0, 0, 0, 0);
            turn_current_dir = Turn_UpdateDirection(turn_current_dir, turn_action);  // 更新当前朝向
            xSemaphoreGive(xAvoidSemaphore);
          }
        } else {
          if(turn_action == TURN_STRAIGHT) {
            turn_current_dir = Turn_UpdateDirection(turn_current_dir, turn_action);
          }
        }

        // 转向完成，等待1s再直行，恢复之前的导航状态
        vTaskDelay(pdMS_TO_TICKS(1000));
        nav_state = prev_nav_state;  // 恢复导航状态，继续巡线
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
        if (nav_state == NAV_GOING && room_num == turn_target_room) {  // 到达目标房间
          nav_state = NAV_IDLE;
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            Motion_Ctrl(0, 0, 0, 0);
            xSemaphoreGive(xAvoidSemaphore);
          }
          vTaskDelay(pdMS_TO_TICKS(5000));  // 在房间停留5秒
          
          // 到达房间后掉头
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            motor_speed_t uturn_speed;
            uturn_speed.speed_m1 = 60;
            uturn_speed.speed_m2 = -60;
            uturn_speed.speed_m3 = 60;
            uturn_speed.speed_m4 = -60;
            Motor_Set_Speed(uturn_speed);
            vTaskDelay(pdMS_TO_TICKS(g_turn_delay_180));
            Motion_Ctrl(0, 0, 0, 0);
            xSemaphoreGive(xAvoidSemaphore);
          }
          
          turn_current_dir = Turn_UpdateDirection(turn_current_dir, TURN_UTURN);
          turn_target_room = 0;  // 清除目标房间
          nav_state = NAV_RETURNING;  // 切换为返航模式
        } else if (nav_state == NAV_RETURNING && room_num == 0) {  // 返航到达起点
          nav_state = NAV_IDLE;
          if(xSemaphoreTake(xAvoidSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            Motion_Ctrl(0, 0, 0, 0);
            xSemaphoreGive(xAvoidSemaphore);
          }
        }
      }
    }
    
    MFRC522_ClearBitMask(MFRC522_REG_STATUS2, 0x08);  // 关闭加密单元
    MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);  // 清空FIFO，为读下一张卡做准备
    
    vTaskDelay(pdMS_TO_TICKS(200));
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

