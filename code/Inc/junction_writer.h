#ifndef JUNCTION_WRITER_H
#define JUNCTION_WRITER_H

#include "stdint.h"

// 将数据写入卡片Block 1
uint8_t WriteJunctionBlock(uint8_t* uid, uint8_t* block_data);

// 写入示例数据：路口A
uint8_t WriteJunctionA(uint8_t* uid);

// 写入示例数据：路口B
uint8_t WriteJunctionB(uint8_t* uid);

// 写入示例数据：路口C
uint8_t WriteJunctionC(uint8_t* uid);

#endif