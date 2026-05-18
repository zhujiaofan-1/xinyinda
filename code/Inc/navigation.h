#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stdint.h"

// 方向定义
typedef enum {
  DIR_STRAIGHT = 0x00,  // 直行
  DIR_LEFT     = 0x01,  // 左转
  DIR_RIGHT    = 0x02,  // 右转
  DIR_U_TURN   = 0x03,  // 掉头
  DIR_UNKNOWN  = 0xFF   // 未知
} Direction;

// 导航状态定义
#define NAV_GO_TO_ROOM       0
#define NAV_RETURN_TO_START  1

// 方向指令字符串
extern char* direction_str[];

// 当前目标房间
extern uint16_t target_room;

// 导航同步变量
extern volatile uint8_t nav_direction;      // 导航方向指令
extern volatile uint8_t nav_ready;          // 导航指令就绪标志
extern SemaphoreHandle_t xNavSemaphore;     // 导航同步信号量
extern uint8_t nav_state;                   // 导航状态

// LCD显示用变量
extern volatile uint8_t lcd_uid[5];         // 最近读取的RFID卡号
extern volatile uint8_t lcd_uid_valid;      // 卡号有效标志
extern volatile uint16_t lcd_room_number;   // 读取到的房间号
extern volatile uint8_t lcd_room_valid;     // 房间号有效标志

// 读取路口卡片数据（Block 1）
uint8_t Nav_ReadJunctionData(uint8_t* uid, uint8_t* block_data);

// 读取房间门口卡片数据
uint8_t Nav_ReadRoomNumber(uint8_t* uid, uint16_t* room_number);

// 根据目标房间查询转向方向
Direction Nav_FindDirection(uint8_t* block_data, uint16_t target_room);

// 执行导航逻辑
void Nav_ProcessJunction(uint8_t* uid);

// 检查是否到达目标房间
uint8_t Nav_CheckDestination(uint8_t* uid, uint16_t target_room);

#endif