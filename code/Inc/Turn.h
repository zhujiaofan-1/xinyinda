#ifndef __TURN_H__
#define __TURN_H__

#include "stdint.h"

// 方向定义
#define DIR_N  0x01  // 北
#define DIR_S  0x02  // 南
#define DIR_W  0x03  // 西
#define DIR_E  0x04  // 东

// 转向动作定义
#define TURN_STRAIGHT  0x00  // 直行
#define TURN_LEFT      0x01  // 左转
#define TURN_RIGHT     0x02  // 右转
#define TURN_UTURN     0x03  // 掉头
#define TURN_ERROR     0xFF  // 错误，不执行转向

// 导航状态定义
#define NAV_IDLE       0     // 空闲，在起始点等待
#define NAV_GOING      1     // 正在前往目标房间
#define NAV_RETURNING  2     // 正在返航

// 转向数据结构
typedef struct {
  uint8_t direction;    // 方向（N/S/W/E）
  uint8_t room_number;  // 房间号
} TurnInfo_t;

// 最多支持4组方向房间数据
#define MAX_TURN_DATA  8

// 全局变量声明
extern volatile uint8_t turn_current_dir;    // 小车当前方位（来自的方向）
extern volatile uint16_t turn_target_room;   // 目标房间号
extern volatile uint8_t turn_action;         // 计算出的转向动作
extern volatile uint8_t turn_action_valid;   // 转向动作有效标志
extern volatile char turn_card_content[17];  // 卡片内容字符串
extern volatile uint8_t nav_state;           // 导航状态

void Turn_GoRoom(uint16_t room);

// 初始化Turn模块（设置初始方向和目标房间）
void Turn_Init(uint8_t init_dir, uint16_t target_room);

// 转向后更新当前方向
uint8_t Turn_UpdateDirection(uint8_t current_dir, uint8_t turn_action);

// 返航导航：根据当前方向计算返回南方的转向
uint8_t Turn_CalculateReturn(uint8_t current_dir);

// 读取路口卡片Block 1数据
uint8_t Turn_ReadCardData(uint8_t* uid, uint8_t* block_data);

// 解析N1S2W3E4格式数据
uint8_t Turn_ParseData(uint8_t* block_data, TurnInfo_t* turn_info, uint8_t* count);

// 根据当前位置和目标房间计算转向动作
uint8_t Turn_CalculateTurn(uint8_t current_dir, uint8_t target_room, TurnInfo_t* turn_info, uint8_t count);

// 字符转方向编码
uint8_t Turn_CharToDir(char c);

// 方向编码转字符串
const char* Turn_DirToString(uint8_t dir);

// 转向动作转字符串
const char* Turn_TurnToString(uint8_t turn);

#endif
