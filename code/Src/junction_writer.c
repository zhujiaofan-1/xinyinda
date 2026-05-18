#include "junction_writer.h"
#include "rc522.h"
#include "usart.h"
#include "stdio.h"

// 将数据写入卡片Block 1
uint8_t WriteJunctionBlock(uint8_t* uid, uint8_t* block_data) {
  uint8_t status;
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 默认密码
  uint8_t uid_full[5];
  uint8_t i;
  uint8_t tmp_uid[5];
  
  // 重新唤醒卡片（使用WUPA，可唤醒被Halt休眠的卡片）
  status = MFRC522_Request(PICC_REQALL, tmp_uid);
  if (status != MI_OK) {
    UART_Send_String("Request failed!\r\n");
    return status;
  }
  
  // 重新获取UID（防冲突）
  status = MFRC522_Anticoll(tmp_uid);
  if (status != MI_OK) {
    UART_Send_String("Anticoll failed!\r\n");
    return status;
  }
  
  // 复制UID并添加校验位（用于SelectTag）
  for (i = 0; i < 4; i++) {
    uid_full[i] = tmp_uid[i];
  }
  uid_full[4] = tmp_uid[0] ^ tmp_uid[1] ^ tmp_uid[2] ^ tmp_uid[3];
  
  // 选择卡片（必须在认证之前）
  if (MFRC522_SelectTag(uid_full) == 0) {
    UART_Send_String("Select tag failed!\r\n");
    return MI_ERR;
  }
  
  // 认证扇区0
  status = MFRC522_Auth(PICC_AUTHENT1A, 1, key, tmp_uid);
  if (status != MI_OK) {
    UART_Send_String("Auth failed!\r\n");
    return status;
  }
  
  // 写入Block 1
  status = MFRC522_Write(1, block_data);
  if (status != MI_OK) {
    UART_Send_String("Write failed!\r\n");
    return status;
  }
  
  // 让卡片休眠
  MFRC522_Halt();
  
  UART_Send_String("Write success!\r\n");
  return MI_OK;
}

// 写入示例数据：路口A
uint8_t WriteJunctionA(uint8_t* uid) {
  uint8_t block_data[16];
  
  // Room 101: Left
  block_data[0] = (101 >> 8) & 0xFF;
  block_data[1] = 101 & 0xFF;
  block_data[2] = 0x01;  // Left
  block_data[3] = 0x00;
  
  // Room 102: Straight
  block_data[4] = (102 >> 8) & 0xFF;
  block_data[5] = 102 & 0xFF;
  block_data[6] = 0x00;  // Straight
  block_data[7] = 0x00;
  
  // Room 103: Right
  block_data[8] = (103 >> 8) & 0xFF;
  block_data[9] = 103 & 0xFF;
  block_data[10] = 0x02;  // Right
  block_data[11] = 0x00;
  
  // START_ROOM (0): Direction back to start point
  block_data[12] = 0x00;  // Room 0 (START_ROOM)
  block_data[13] = 0x00;
  block_data[14] = 0x03;  // U-Turn (direction back to start)
  block_data[15] = 0x00;
  
  UART_Send_String("Writing Junction A data...\r\n");
  return WriteJunctionBlock(uid, block_data);
}

// 写入示例数据：路口B
uint8_t WriteJunctionB(uint8_t* uid) {
  uint8_t block_data[16];
  
  // Room 101: U-Turn
  block_data[0] = (101 >> 8) & 0xFF;
  block_data[1] = 101 & 0xFF;
  block_data[2] = 0x03;  // U-Turn
  block_data[3] = 0x00;
  
  // Room 102: Left
  block_data[4] = (102 >> 8) & 0xFF;
  block_data[5] = 102 & 0xFF;
  block_data[6] = 0x01;  // Left
  block_data[7] = 0x00;
  
  // Room 103: Straight
  block_data[8] = (103 >> 8) & 0xFF;
  block_data[9] = 103 & 0xFF;
  block_data[10] = 0x00;  // Straight
  block_data[11] = 0x00;
  
  // START_ROOM (0): Direction back to start point
  block_data[12] = 0x00;  // Room 0 (START_ROOM)
  block_data[13] = 0x00;
  block_data[14] = 0x01;  // Left (direction back to start)
  block_data[15] = 0x00;
  
  UART_Send_String("Writing Junction B data...\r\n");
  return WriteJunctionBlock(uid, block_data);
}

// 写入示例数据：路口C
uint8_t WriteJunctionC(uint8_t* uid) {
  uint8_t block_data[16];
  
  // Room 101: Right
  block_data[0] = (101 >> 8) & 0xFF;
  block_data[1] = 101 & 0xFF;
  block_data[2] = 0x02;  // Right
  block_data[3] = 0x00;
  
  // Room 102: U-Turn
  block_data[4] = (102 >> 8) & 0xFF;
  block_data[5] = 102 & 0xFF;
  block_data[6] = 0x03;  // U-Turn
  block_data[7] = 0x00;
  
  // Room 103: Left
  block_data[8] = (103 >> 8) & 0xFF;
  block_data[9] = 103 & 0xFF;
  block_data[10] = 0x01;  // Left
  block_data[11] = 0x00;
  
  // START_ROOM (0): Direction back to start point
  block_data[12] = 0x00;  // Room 0 (START_ROOM)
  block_data[13] = 0x00;
  block_data[14] = 0x02;  // Right (direction back to start)
  block_data[15] = 0x00;
  
  UART_Send_String("Writing Junction C data...\r\n");
  return WriteJunctionBlock(uid, block_data);
}