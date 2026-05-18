#include "Headfile.h"
#include "Turn.h"
#include "rc522.h"
#include "usart.h"
#include "stdio.h"

// 全局变量定义
volatile uint8_t turn_current_dir = DIR_S;     // 默认从南边出发
volatile uint16_t turn_target_room = 0;        // 目标房间号
volatile uint8_t turn_action = TURN_ERROR;     // 转向动作
volatile uint8_t turn_action_valid = 0;        // 转向动作有效标志
volatile char turn_card_content[17] = {0};     // 卡片内容字符串

/**
 * @brief  初始化Turn模块
 * @param  init_dir: 小车初始方位（来自的方向）
 * @param  target_room: 目标房间号
 */
void Turn_Init(uint8_t init_dir, uint16_t target_room) {
  turn_current_dir = init_dir;
  turn_target_room = target_room;
  turn_action = TURN_ERROR;
  turn_action_valid = 0;
}

/**
 * @brief  转向后更新当前方向
 * @param  current_dir: 当前方位
 * @param  action: 执行的转向动作
 * @retval 更新后的朝向方位
 */
uint8_t Turn_UpdateDirection(uint8_t current_dir, uint8_t action) {
  switch (action) {
    case TURN_STRAIGHT:
      return current_dir;
    case TURN_LEFT:
      // 左转：逆时针 N→W→S→E→N
      // 例如：从南边左转，新朝向方位为西边
      switch (current_dir) {
        case DIR_N: return DIR_W;
        case DIR_W: return DIR_S;
        case DIR_S: return DIR_E;
        case DIR_E: return DIR_N;
        default: return current_dir;
      }
    case TURN_RIGHT:
      // 右转：顺时针 N→E→S→W→N
      switch (current_dir) {
        case DIR_N: return DIR_E;
        case DIR_E: return DIR_S;
        case DIR_S: return DIR_W;
        case DIR_W: return DIR_N;
        default: return current_dir;
      }
    case TURN_UTURN:
      // 掉头：取反 N↔S, W↔E
      switch (current_dir) {
        case DIR_N: return DIR_S;
        case DIR_S: return DIR_N;
        case DIR_W: return DIR_E;
        case DIR_E: return DIR_W;
        default: return current_dir;
      }
    default:
      return current_dir;
  }
}

/**
 * @brief  读取路口卡片Block 1数据
 * @param  uid: 卡片UID
 * @param  block_data: 存储读取的16字节数据
 * @retval MI_OK=成功, 其他=失败
 */
uint8_t Turn_ReadCardData(uint8_t* uid, uint8_t* block_data) {
  uint8_t status;
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 默认密码
  uint8_t uid_full[5];
  uint8_t i;
  uint8_t tmp_uid[5];
  
  // 唤醒卡片（使用WUPA，可唤醒被Halt休眠的卡片）
  status = MFRC522_Request(PICC_REQALL, tmp_uid);
  if (status != MI_OK) {
    return status;
  }
  
  // 防冲突获取UID
  status = MFRC522_Anticoll(tmp_uid);
  if (status != MI_OK) {
    return status;
  }
  
  // 复制UID并添加校验位
  for (i = 0; i < 4; i++) {
    uid_full[i] = tmp_uid[i];
  }
  uid_full[4] = tmp_uid[0] ^ tmp_uid[1] ^ tmp_uid[2] ^ tmp_uid[3];
  
  // 选择卡片
  if (MFRC522_SelectTag(uid_full) == 0) {
    return MI_ERR;
  }
  
  // 认证扇区0
  status = MFRC522_Auth(PICC_AUTHENT1A, 1, key, tmp_uid);
  if (status != MI_OK) {
    return status;
  }
  
  // 读取Block 1（16字节）
  status = MFRC522_Read(1, block_data);
  
  // 卡片休眠
  MFRC522_Halt();
  
  return status;
}

/**
 * @brief  解析N34S0W2E15格式数据（每个数字为独立房间号）
 * @param  block_data: 卡片读取的16字节数据
 * @param  turn_info: 存储解析后的转向信息数组
 * @param  count: 返回解析到的数据组数
 * @retval MI_OK=成功, MI_ERR=失败
 */
uint8_t Turn_ParseData(uint8_t* block_data, TurnInfo_t* turn_info, uint8_t* count) {
  char str[17];
  uint8_t i;
  uint8_t idx = 0;
  uint8_t current_dir = 0;
  
  //转换为字符串解析
  for (i = 0; i < 16; i++) {
    str[i] = block_data[i];
    if (block_data[i] == 0) break;
  }
  str[i] = '\0';
  
  *count = 0;
  
  // 解析字符串：字母确定方向，每个数字单独作为房间号
  // 例如 N34 表示北方有房间3和房间4
  for (i = 0; str[i] != '\0' && idx < MAX_TURN_DATA; i++) {
    if ((str[i] >= 'A' && str[i] <= 'Z') || (str[i] >= 'a' && str[i] <= 'z')) {
      current_dir = Turn_CharToDir(str[i]);
    } else if (str[i] >= '0' && str[i] <= '9' && current_dir != 0) {
      turn_info[idx].direction = current_dir;
      turn_info[idx].room_number = str[i] - '0';
      idx++;
      (*count)++;
    }
  }
  
  return (*count > 0) ? MI_OK : MI_ERR;
}

/**
 * @brief  根据当前位置和目标房间计算转向动作
 * @param  current_dir: 小车当前所在方向（DIR_N/S/W/E）
 * @param  target_room: 目标房间号
 * @param  turn_info: 解析后的转向信息数组
 * @param  count: 数据组数
 * @retval 转向动作（TURN_STRAIGHT/LEFT/RIGHT/UTURN），0xFF=未找到
 */
uint8_t Turn_CalculateTurn(uint8_t current_dir, uint8_t target_room, TurnInfo_t* turn_info, uint8_t count) {
  uint8_t i;
  uint8_t target_dir = 0;
  
  // 验证当前方向是否合法
  if (current_dir != DIR_N && current_dir != DIR_S &&
      current_dir != DIR_W && current_dir != DIR_E) {
    return TURN_ERROR;
  }
  
  // 验证数据组数
  if (count == 0 || turn_info == 0) {
    return TURN_ERROR;
  }
  
  // 查找目标房间所在方向
  for (i = 0; i < count; i++) {
    if (turn_info[i].room_number == target_room) {
      target_dir = turn_info[i].direction;
      break;
    }
  }
  
  // 未找到目标房间
  if (target_dir == 0) {
    return TURN_ERROR;
  }
  
  // 同一方向（目标在来路=背后），掉头
  if (current_dir == target_dir) {
    return TURN_UTURN;
  }
  
  // 相对方向（目标在前方），直行
  if ((current_dir == DIR_N && target_dir == DIR_S) ||
      (current_dir == DIR_S && target_dir == DIR_N) ||
      (current_dir == DIR_W && target_dir == DIR_E) ||
      (current_dir == DIR_E && target_dir == DIR_W)) {
    return TURN_STRAIGHT;
  }
  
  // 顺时针方向，左转（N→E→S→W→N）
  if ((current_dir == DIR_N && target_dir == DIR_E) ||
      (current_dir == DIR_E && target_dir == DIR_S) ||
      (current_dir == DIR_S && target_dir == DIR_W) ||
      (current_dir == DIR_W && target_dir == DIR_N)) {
    return TURN_LEFT;
  }
  
  // 其他情况，右转
  return TURN_RIGHT;
}

/**
 * @brief  字符转方向编码
 * @param  c: 方向字符（N/S/W/E）
 * @retval 方向编码（DIR_N/S/W/E）
 */
uint8_t Turn_CharToDir(char c) {
  switch (c) {
    case 'N': case 'n': return DIR_N;
    case 'S': case 's': return DIR_S;
    case 'W': case 'w': return DIR_W;
    case 'E': case 'e': return DIR_E;
    default: return 0;
  }
}

/**
 * @brief  方向编码转字符串
 * @param  dir: 方向编码
 * @retval 方向字符串
 */
const char* Turn_DirToString(uint8_t dir) {
  switch (dir) {
    case DIR_N: return "North";
    case DIR_S: return "South";
    case DIR_W: return "West";
    case DIR_E: return "East";
    default:    return "Unknown";
  }
}

/**
 * @brief  转向动作转字符串
 * @param  turn: 转向动作编码
 * @retval 转向动作字符串
 */
const char* Turn_TurnToString(uint8_t turn) {
  switch (turn) {
    case TURN_STRAIGHT: return "Straight";
    case TURN_LEFT:     return "Left";
    case TURN_RIGHT:    return "Right";
    case TURN_UTURN:    return "U-Turn";
    case TURN_ERROR:    return "Error";
    default:            return "Unknown";
  }
}
