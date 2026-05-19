#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include <stdio.h>

extern char Blue_Serial_RxPacket[];
extern uint8_t Blue_Serial_RxFlag;

void Blue_Serial_Init(void);
void Blue_Serial_SendByte(uint8_t Byte);
void Blue_Serial_SendArray(uint8_t *Array, uint16_t Length);
void Blue_Serial_SendString(char *String);
void Blue_Serial_SendNumber(uint32_t Number, uint8_t Length);
void Blue_Serial_Printf(char *format, ...);

#endif
