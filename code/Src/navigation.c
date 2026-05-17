#include "navigation.h"
#include "rc522.h"
#include "usart.h"
#include "stdio.h"

// 方向指令字符串
char* direction_str[] = {"Straight", "Left", "Right", "U-Turn", "Unknown"};

// 当前目标房间
uint16_t target_room = 102;  // 默认目标房间

// 导航状态枚举
typedef enum {
  NAV_GO_TO_ROOM,      // 前往目标房间
  NAV_RETURN_TO_START  // 返回起点
} NavState;

// 导航状态
static NavState nav_state = NAV_GO_TO_ROOM;

// 起点卡片UID（用于检测是否到达起点）
uint8_t start_point_uid[4] = {0x11, 0x22, 0x33, 0x44};  // 默认起点卡片UID

// 起点房间号（特殊标识）
#define START_ROOM 0

// 读取路口卡片数据（Block 1）
uint8_t Nav_ReadJunctionData(uint8_t* uid, uint8_t* block_data) {
  uint8_t status;
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 默认密码
  uint8_t uid_full[5];
  uint8_t i;
  uint8_t tmp_uid[5];
  
  // 重新唤醒卡片（因为之前的Check函数调用了Halt）
  status = MFRC522_Request(PICC_REQIDL, tmp_uid);
  if (status != MI_OK) {
    return status;
  }
  
  // 重新获取UID（防冲突）
  status = MFRC522_Anticoll(tmp_uid);
  if (status != MI_OK) {
    return status;
  }
  
  // 复制UID并添加校验位（用于SelectTag）
  for (i = 0; i < 4; i++) {
    uid_full[i] = tmp_uid[i];
  }
  uid_full[4] = tmp_uid[0] ^ tmp_uid[1] ^ tmp_uid[2] ^ tmp_uid[3];
  
  // 选择卡片（必须在认证之前）
  if (MFRC522_SelectTag(uid_full) == 0) {
    return MI_ERR;
  }
  
  // 认证扇区0（Block 1属于扇区0）
  status = MFRC522_Auth(PICC_AUTHENT1A, 1, key, tmp_uid);
  if (status != MI_OK) {
    return status;
  }
  
  // 读取Block 1（16字节）
  status = MFRC522_Read(1, block_data);
  
  // 让卡片休眠
  MFRC522_Halt();
  
  return status;
}

// 根据目标房间查询转向方向
Direction Nav_FindDirection(uint8_t* block_data, uint16_t target_room) {
  // 遍历4个房间的映射（每个房间占4字节）
  // 格式: [房间号高字节][房间号低字节][方向][保留]
  for (int i = 0; i < 4; i++) {
    uint16_t room_number = (block_data[i*4] << 8) | block_data[i*4 + 1];
    
    if (room_number == target_room) {
      return (Direction)block_data[i*4 + 2];
    }
  }
  return DIR_UNKNOWN;
}

// 比较两个UID是否相等
static uint8_t UID_Compare(uint8_t* uid1, uint8_t* uid2) {
  for (int i = 0; i < 4; i++) {
    if (uid1[i] != uid2[i]) return 0;
  }
  return 1;
}

// 执行导航逻辑
void Nav_ProcessJunction(uint8_t* uid) {
  uint8_t block_data[16];
  Direction dir;
  char msg[80];
  
  // 检测是否是起点卡片
  if (UID_Compare(uid, start_point_uid)) {
    if (nav_state == NAV_RETURN_TO_START) {
      // 已到达起点
      UART_Send_String("\n=== Navigation Complete ===\r\n");
      UART_Send_String("Arrived at START POINT!\r\n");
      UART_Send_String("==========================\r\n");
      nav_state = NAV_GO_TO_ROOM;  // 重置状态，准备下一次导航
      sprintf(msg, "Ready to go to Room %d\r\n", target_room);
      UART_Send_String(msg);
    } else {
      // 从起点出发
      UART_Send_String("\n=== Starting Navigation ===\r\n");
      UART_Send_String("Starting from START POINT\r\n");
      sprintf(msg, "Destination: Room %d\r\n", target_room);
      UART_Send_String(msg);
      UART_Send_String("==========================\r\n");
    }
    return;
  }
  
  // 读取路口卡片数据
  if (Nav_ReadJunctionData(uid, block_data) != MI_OK) {
    UART_Send_String("Read card data failed\r\n");
    return;
  }
  
  // 根据当前状态确定目标
  uint16_t current_target;
  if (nav_state == NAV_GO_TO_ROOM) {
    current_target = target_room;
  } else {
    current_target = START_ROOM;  // 返回起点
  }
  
  // 查询方向
  dir = Nav_FindDirection(block_data, current_target);
  
  // 输出导航信息
  sprintf(msg, "\n=== Junction Navigation ===\r\n");
  UART_Send_String(msg);
  
  if (nav_state == NAV_GO_TO_ROOM) {
    sprintf(msg, "State: Going to Room %d\r\n", target_room);
    UART_Send_String(msg);
  } else {
    UART_Send_String("State: Returning to START POINT\r\n");
  }
  
  sprintf(msg, "Turn Direction: %s\r\n", direction_str[dir]);
  UART_Send_String(msg);
  
  UART_Send_String("==========================\r\n");
  
  // 检测是否到达目标房间（特殊处理：目标房间数据存在说明到达）
  if (nav_state == NAV_GO_TO_ROOM && dir != DIR_UNKNOWN) {
    // 检查是否到达目标房间（简化：检测到有效方向后假设到达目标区域）
    // 实际应用中可能需要单独的房间卡片
    UART_Send_String("\n=== Arrived at Destination ===\r\n");
    sprintf(msg, "Reached Room %d!\r\n", target_room);
    UART_Send_String(msg);
    UART_Send_String("Auto-return to START POINT...\r\n");
    UART_Send_String("==========================\r\n");
    nav_state = NAV_RETURN_TO_START;  // 切换到返回起点状态
  }
}