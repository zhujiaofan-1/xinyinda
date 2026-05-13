#ifndef _DEALY_H_
#define _DEALY_H_

#include "stdint.h"

void InitDelay(uint8_t SYSCLK);
void DelayMs(uint16_t nms);
void DelayUs(uint32_t nus);

#endif
