#include "stdint.h"
// Mifare RC522 RFID Card reader 13.56 MHz
// STM32F103 RFID RC522 SPI3 / UART / USB / Keil HAL

// MFRC522		STM32F103		DESCRIPTION
// CS (SDA)		PA4					SPI3_NSS	Chip select for SPI
// SCK				PB3					SPI3_SCK	Serial Clock for SPI
// MOSI			PB5					SPI3_MOSI	Master In Slave Out for SPI
// MISO			PB4					SPI3_MISO	Master Out Slave In for SPI
// IRQ				-						Irq
// GND				GND					Ground
// RST				3.3V				Reset pin (3.3V)
// VCC				3.3V				3.3V power

// SPI CS define
#define cs_reset() 					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define cs_set() 						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

// Status enumeration, Used with most functions
#define MI_OK													0
#define MI_NOTAGERR										1
#define MI_ERR												2

// MFRC522 Commands
#define PCD_IDLE											0x00
#define PCD_AUTHENT										0x0E
#define PCD_RECEIVE										0x08
#define PCD_TRANSMIT									0x04
#define PCD_TRANSCEIVE								0x0C
#define PCD_RESETPHASE								0x0F
#define PCD_CALCCRC										0x03

// Mifare_One card command word
#define PICC_REQIDL										0x26
#define PICC_REQALL										0x52
#define PICC_ANTICOLL									0x93
#define PICC_SElECTTAG								0x93
#define PICC_AUTHENT1A								0x60
#define PICC_AUTHENT1B								0x61
#define PICC_READ											0x30
#define PICC_WRITE										0xA0
#define PICC_DECREMENT								0xC0
#define PICC_INCREMENT								0xC1
#define PICC_RESTORE									0xC2
#define PICC_TRANSFER									0xB0
#define PICC_HALT											0x50

// MFRC522 Registers
// Page 0: Command and Status
#define MFRC522_REG_RESERVED00				0x00
#define MFRC522_REG_COMMAND						0x01
#define MFRC522_REG_COMM_IE_N					0x02
#define MFRC522_REG_DIV1_EN						0x03
#define MFRC522_REG_COMM_IRQ					0x04
#define MFRC522_REG_DIV_IRQ						0x05
#define MFRC522_REG_ERROR							0x06
#define MFRC522_REG_STATUS1						0x07
#define MFRC522_REG_STATUS2						0x08
#define MFRC522_REG_FIFO_DATA					0x09
#define MFRC522_REG_FIFO_LEVEL				0x0A
#define MFRC522_REG_WATER_LEVEL				0x0B
#define MFRC522_REG_CONTROL						0x0C
#define MFRC522_REG_BIT_FRAMING				0x0D
#define MFRC522_REG_COLL							0x0E
#define MFRC522_REG_RESERVED01				0x0F
// Page 1: Command
#define MFRC522_REG_RESERVED10				0x10
#define MFRC522_REG_MODE							0x11
#define MFRC522_REG_TX_MODE						0x12
#define MFRC522_REG_RX_MODE						0x13
#define MFRC522_REG_TX_CONTROL				0x14
#define MFRC522_REG_TX_AUTO						0x15
#define MFRC522_REG_TX_SELL						0x16
#define MFRC522_REG_RX_SELL						0x17
#define MFRC522_REG_RX_THRESHOLD			0x18
#define MFRC522_REG_DEMOD							0x19
#define MFRC522_REG_RESERVED11				0x1A
#define MFRC522_REG_RESERVED12				0x1B
#define MFRC522_REG_MIFARE						0x1C
#define MFRC522_REG_RESERVED13				0x1D
#define MFRC522_REG_RESERVED14				0x1E
#define MFRC522_REG_SERIALSPEED				0x1F
// Page 2: CFG
#define MFRC522_REG_RESERVED20				0x20
#define MFRC522_REG_CRC_RESULT_M			0x21
#define MFRC522_REG_CRC_RESULT_L			0x22
#define MFRC522_REG_RESERVED21				0x23
#define MFRC522_REG_MOD_WIDTH					0x24
#define MFRC522_REG_RESERVED22				0x25
#define MFRC522_REG_RF_CFG						0x26
#define MFRC522_REG_GS_N							0x27
#define MFRC522_REG_CWGS_PREG					0x28
#define MFRC522_REG__MODGS_PREG				0x29
#define MFRC522_REG_T_MODE						0x2A
#define MFRC522_REG_T_PRESCALER				0x2B
#define MFRC522_REG_T_RELOAD_H				0x2C
#define MFRC522_REG_T_RELOAD_L				0x2D
#define MFRC522_REG_T_COUNTER_VALUE_H	0x2E
#define MFRC522_REG_T_COUNTER_VALUE_L	0x2F
// Page 3: TestRegister
#define MFRC522_REG_RESERVED30				0x30
#define MFRC522_REG_TEST_SEL1					0x31
#define MFRC522_REG_TEST_SEL2					0x32
#define MFRC522_REG_TEST_PIN_EN				0x33
#define MFRC522_REG_TEST_PIN_VALUE		0x34
#define MFRC522_REG_TEST_BUS					0x35
#define MFRC522_REG_AUTO_TEST					0x36
#define MFRC522_REG_VERSION						0x37
#define MFRC522_REG_ANALOG_TEST				0x38
#define MFRC522_REG_TEST_ADC1					0x39
#define MFRC522_REG_TEST_ADC2					0x3A
#define MFRC522_REG_TEST_ADC0					0x3B
#define MFRC522_REG_RESERVED31				0x3C
#define MFRC522_REG_RESERVED32				0x3D
#define MFRC522_REG_RESERVED33				0x3E
#define MFRC522_REG_RESERVED34				0x3F

#define MFRC522_DUMMY									0x00
#define MFRC522_MAX_LEN								16

// Function declarations
uint8_t SPI1SendByte(uint8_t data);
void SPI1_WriteReg(uint8_t address, uint8_t value);
uint8_t SPI1_ReadReg(uint8_t address);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalculateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);
