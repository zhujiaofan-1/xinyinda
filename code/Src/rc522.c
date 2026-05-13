// Mifare RC522 RFID Card reader 13.56 MHz
// STM32F103 RFID RC522 SPI1 / UART / USB / Keil HAL / 2017 vk.com/zz555


// MFRC522		STM32F103		DESCRIPTION
// CS (SDA)		PA4					SPI1_NSS	Chip select for SPI
// SCK				PA5					SPI1_SCK	Serial Clock for SPI
// MOSI				PA7 				SPI1_MOSI	Master In Slave Out for SPI
// MISO				PA6					SPI1_MISO	Master Out Slave In for SPI
// IRQ				-						Irq
// GND				GND					Ground
// RST				3.3V				Reset pin (3.3V)
// VCC				3.3V				3.3V power

#include "Headfile.h"
#include "rc522.h"
#include "spi.h"
#include "stdint.h"

extern SPI_HandleTypeDef hspi1;

/**
 * @brief  SPI发送1字节数据
 * @param  data: 要发送的数据
 * @retval 接收到的数据
 */
uint8_t SPI1SendByte(uint8_t data) {
	unsigned char writeCommand[1];
	unsigned char readValue[1];
	
	writeCommand[0] = data;
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&writeCommand, (uint8_t*)&readValue, 1, 10);
	return readValue[0];

}

/**
 * @brief  SPI写寄存器
 * @param  address: 寄存器地址
 * @param  value: 要写入的值
 * @retval 无
 */
void SPI1_WriteReg(uint8_t address, uint8_t value) {
	cs_reset();
	SPI1SendByte(address);
	SPI1SendByte(value);
	cs_set();
}

/**
 * @brief  SPI读寄存器
 * @param  address: 寄存器地址
 * @retval 寄存器值
 */
uint8_t SPI1_ReadReg(uint8_t address) {
	uint8_t	val;

	cs_reset();
	SPI1SendByte(address);
	val = SPI1SendByte(0x00);
	cs_set();
	return val;
}

/**
 * @brief  RC522写寄存器函数
 * @param  addr: 寄存器地址
 * @param  val: 要写入的值
 * @retval 无
 */
void MFRC522_WriteRegister(uint8_t addr, uint8_t val) {
	addr = (addr << 1) & 0x7E;
  SPI1_WriteReg(addr, val);
}

/**
 * @brief  RC522读寄存器函数
 * @param  addr: 寄存器地址
 * @retval 寄存器值
 */
uint8_t MFRC522_ReadRegister(uint8_t addr) {
	uint8_t val;

	addr = ((addr << 1) & 0x7E) | 0x80;
	val = SPI1_ReadReg(addr);
	return val;	
}

/**
 * @brief  检测是否有卡片
 * @param  id: 卡片ID缓冲区
 * @retval MI_OK: 检测到卡片, MI_ERR: 未检测到卡片
 */
uint8_t MFRC522_Check(uint8_t* id) {
	uint8_t status;
	status = MFRC522_Request(PICC_REQIDL, id);
	if (status == MI_OK) status = MFRC522_Anticoll(id);
	MFRC522_Halt();
	return status;
}

/**
 * @brief  比较两张卡片ID是否相同
 * @param  CardID: 当前卡片ID
 * @param  CompareID: 要比较的卡片ID
 * @retval MI_OK: 相同, MI_ERR: 不同
 */
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) return MI_ERR;
	}
	return MI_OK;
}

/**
 * @brief  设置寄存器位掩码
 * @param  reg: 寄存器地址
 * @param  mask: 要设置的掩码
 * @retval 无
 */
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
	MFRC522_WriteRegister(reg, MFRC522_ReadRegister(reg) | mask);
}

/**
 * @brief  清除寄存器位掩码
 * @param  reg: 寄存器地址
 * @param  mask: 要清除的掩码
 * @retval 无
 */
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask){
	MFRC522_WriteRegister(reg, MFRC522_ReadRegister(reg) & (~mask));
}


/*
函数名: 寻卡
					reqMode: 寻卡方式
					0x52   = 寻感应区内所有符合14443A标准的卡
					0x26   = 寻未进入休眠状态的卡
					TagType: 卡片类型代码
					0x4400 = Mifare_UltraLight
					0x0400 = Mifare_One(S50)
					0x0200 = Mifare_One(S70)
					0x0800 = Mifare_Pro(X)
					0x4403 = Mifare_DESFire
*/
/**
 * @brief  寻卡
 * @param  reqMode: 寻卡方式，0x26=寻未休眠卡，0x52=寻所有卡
 * @param  TagType: 卡片类型代码缓冲区
 * @retval MI_OK: 寻卡成功, MI_ERR: 寻卡失败
 */
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType) {
	uint8_t status;  
	uint16_t backBits;

	MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);
	TagType[0] = reqMode;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
	if ((status != MI_OK) || (backBits != 0x10)) status = MI_ERR;
	return status;
}

/*
函数功能: 通过RC522和ISO14443卡通讯
          command: RC522命令字
          sendData: 通过RC522发送到卡片的数据
          sendLen: 发送数据的字节长度
          backData: 接收到的卡片返回数据
          backLen: 返回数据的位长度
*/
/**
 * @brief  通过RC522和ISO14443卡通讯
 * @param  command: RC522命令字
 * @param  sendData: 发送到卡片的数据
 * @param  sendLen: 发送数据的字节长度
 * @param  backData: 接收到的卡片返回数据
 * @param  backLen: 返回数据的位长度
 * @retval MI_OK: 通讯成功, MI_ERR: 通讯失败
 */
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {
	uint8_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
		break;
	}

	MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	MFRC522_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
	MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE);

	for (i = 0; i < sendLen; i++) MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, sendData[i]);

	MFRC522_WriteRegister(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) MFRC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);

	i = 2000;
	do {
		n = MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	MFRC522_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);

	if (i != 0)  {
		if (!(MFRC522_ReadRegister(MFRC522_REG_ERROR) & 0x1B)) {
			status = MI_OK;
			if (n & irqEn & 0x01) status = MI_NOTAGERR;
			if (command == PCD_TRANSCEIVE) {
				n = MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
				lastBits = MFRC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;
				if (lastBits) *backLen = (n-1)*8+lastBits; else *backLen = n*8;
				if (n == 0) n = 1;
				if (n > MFRC522_MAX_LEN) n = MFRC522_MAX_LEN;
				for (i = 0; i < n; i++) backData[i] = MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);
			}
		} else status = MI_ERR;
	}
	return status;
}

/**
 * @brief  防冲突处理，获取卡片序列号
 * @param  serNum: 卡片序列号缓冲区，5字节
 * @retval MI_OK: 成功, MI_ERR: 失败
 */
uint8_t MFRC522_Anticoll(uint8_t* serNum) {
	uint8_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);
	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
	if (status == MI_OK) {
		for (i = 0; i < 4; i++) serNumCheck ^= serNum[i];
		if (serNumCheck != serNum[i]) status = MI_ERR;
	}
	return status;
} 

/**
 * @brief  计算CRC校验值
 * @param  pIndata: 输入数据指针
 * @param  len: 数据长度
 * @param  pOutData: CRC结果缓冲区，2字节
 * @retval 无
 */
void MFRC522_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);
	MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	for (i = 0; i < len; i++) MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata+i));
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	i = 0xFF;
	do {
		n = MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));

	pOutData[0] = MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}

/**
 * @brief  选择卡片
 * @param  serNum: 卡片序列号，5字节
 * @retval 卡片容量大小，0表示失败
 */
uint8_t MFRC522_SelectTag(uint8_t* serNum) {
	uint8_t i;
	uint8_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9]; 

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) buffer[i+2] = *(serNum+i);
	MFRC522_CalculateCRC(buffer, 7, &buffer[7]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
	if ((status == MI_OK) && (recvBits == 0x18)) size = buffer[0]; else size = 0;
	return size;
}

/*
	验证卡片密码
	authMode: 密码验证模式
	0x60 = 验证A密钥
	0x61 = 验证B密钥 
	BlockAddr: 块地址
	Sectorkey: 扇区密码
	serNum: 卡片序列号，4字节
*/
/**
 * @brief  验证卡片密码
 * @param  authMode: 密码验证模式，0x60=A密钥，0x61=B密钥
 * @param  BlockAddr: 块地址
 * @param  Sectorkey: 扇区密码，6字节
 * @param  serNum: 卡片序列号，4字节
 * @retval MI_OK: 验证成功, MI_ERR: 验证失败
 */
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
	uint8_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12]; 

	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) buff[i+2] = *(Sectorkey+i);
	for (i=0; i<4; i++) buff[i+8] = *(serNum+i);
	status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);
	if ((status != MI_OK) || (!(MFRC522_ReadRegister(MFRC522_REG_STATUS2) & 0x08))) status = MI_ERR;
	return status;
}

/**
 * @brief  读取M1卡一个块数据
 * @param  blockAddr: 块地址
 * @param  recvData: 读取到的卡片数据缓冲区，16字节
 * @retval MI_OK: 读取成功, MI_ERR: 读取失败
 */
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData) {
	uint8_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	MFRC522_CalculateCRC(recvData,2, &recvData[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
	if ((status != MI_OK) || (unLen != 0x90)) status = MI_ERR;
	return status;
}

/**
 * @brief  写数据到M1卡指定块
 * @param  blockAddr: 块地址
 * @param  writeData: 要写入的数据，16字节
 * @retval MI_OK: 写入成功, MI_ERR: 写入失败
 */
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData) {
	uint8_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18]; 

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	MFRC522_CalculateCRC(buff, 2, &buff[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);
	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) status = MI_ERR;
	if (status == MI_OK) {
		for (i = 0; i < 16; i++) buff[i] = *(writeData+i);
		MFRC522_CalculateCRC(buff, 16, &buff[16]);
		status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) status = MI_ERR;
	}
	return status;
}

/**
 * @brief  RC522初始化，配置ISO14443_A协议参数
 * @param  无
 * @retval 无
 */
void MFRC522_Init(void) {
	MFRC522_Reset();
	MFRC522_WriteRegister(MFRC522_REG_T_MODE, 0x8D);
	MFRC522_WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
	MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_L, 30);
	MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_H, 0);
	MFRC522_WriteRegister(MFRC522_REG_RF_CFG, 0xF7);
	MFRC522_WriteRegister(MFRC522_REG_RX_THRESHOLD, 0x84);
	MFRC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);

	MFRC522_WriteRegister(MFRC522_REG_MODE, 0x3D);
	MFRC522_AntennaOn();
}

/**
 * @brief  RC522复位
 * @param  无
 * @retval 无
 */
void MFRC522_Reset(void) {
	MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

/**
 * @brief  RC522开启天线
 * @param  无
 * @retval 无
 */
void MFRC522_AntennaOn(void) {
	uint8_t temp;

	temp = MFRC522_ReadRegister(MFRC522_REG_TX_CONTROL);
	if (!(temp & 0x03)) MFRC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}

/**
 * @brief  RC522关闭天线
 * @param  无
 * @retval 无
 */
void MFRC522_AntennaOff(void) {
	MFRC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}

/**
 * @brief  命令卡片进入休眠状态
 * @param  无
 * @retval 无
 */
void MFRC522_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4]; 

	buff[0] = PICC_HALT;
	buff[1] = 0;
	MFRC522_CalculateCRC(buff, 2, &buff[2]);
	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}


void MFRC522_Task(void* param)
{
    uint8_t CardID[8];
    if (MFRC522_Check(CardID) == MI_OK) {
        printf("Card: %02X%02X%02X%02X\n", CardID[0],CardID[1],CardID[2],CardID[3]);
    }
}