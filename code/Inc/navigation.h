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

// 方向指令字符串
extern char* direction_str[];

// 当前目标房间
extern uint16_t target_room;

// 读取路口卡片数据（Block 1）
uint8_t Nav_ReadJunctionData(uint8_t* uid, uint8_t* block_data);

// 根据目标房间查询转向方向
Direction Nav_FindDirection(uint8_t* block_data, uint16_t target_room);

// 执行导航逻辑
void Nav_ProcessJunction(uint8_t* uid);

#endif