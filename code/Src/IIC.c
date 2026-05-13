#include "Headfile.h"
#include "IIC.h"

void I2C_SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin=IIC_IO_SDA;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	// 开漏输出模式：I2C总线要求SDA和SCL都是开漏输出，支持多主设备仲裁
	// 开漏输出只能拉低电平，高电平由外部上拉电阻提供
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_OD;
	// 上拉电阻确保总线空闲时为高电平，符合I2C协议规范
	GPIO_InitStructure.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOX,&GPIO_InitStructure);
}

void I2C_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin=IIC_IO_SDA;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	// 输入模式：用于读取从设备的应答信号(ACK/NACK)
	// 在发送完8位数据后，需要切换SDA为输入模式来读取从设备的应答
	GPIO_InitStructure.Mode=GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOX,&GPIO_InitStructure);
}

void IIC_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitStructure.Pin = IIC_IO_SDA |IIC_IO_SCL;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOX, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOX,IIC_IO_SDA|IIC_IO_SCL, GPIO_PIN_SET);
}

void IIC_start()
{
	I2C_SDA_OUT();  // 确保SDA为输出模式
	// I2C起始条件：在SCL为高电平时，SDA由高变低
	// 这个信号告诉所有从设备主机准备开始通信
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);
	DelayUs(5);  // 延时确保信号稳定，满足I2C时序要求
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_RESET);
	DelayUs(5);
	// 拉低SCL，准备数据传输（I2C只在SCL低电平时改变数据）
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
}

void IIC_stop()
{
	I2C_SDA_OUT();  // 确保SDA为输出模式
	// I2C停止条件：在SCL为高电平时，SDA由低变高
	// 这个信号告诉从设备本次通信结束，释放总线
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_RESET);
	DelayUs(5);
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_SET);
	DelayUs(5);	
}

void IIC_ack()
{
	// 应答信号(ACK)：主机在接收完一个字节后向从设备发送低电平
	// 告诉从设备"我已成功接收数据，请继续发送下一字节"
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
	I2C_SDA_OUT();
  HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_RESET);  // SDA拉低表示ACK
  DelayUs(5);
  HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);    // SCL高电平期间从设备检测ACK
  DelayUs(5);
  HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
}

void IIC_noack()
{
	// 非应答信号(NACK)：主机在接收完最后一个字节后向从设备发送高电平
	// 告诉从设备"我已接收完毕，请停止发送数据"
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
	I2C_SDA_OUT();
  HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_SET);    // SDA拉高表示NACK
  DelayUs(5);
  HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);    // SCL高电平期间从设备检测NACK
  DelayUs(5);
  HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
}

uint8_t IIC_wait_ack()
{
	uint8_t tempTime=0;
	// 切换到输入模式，准备读取从设备的应答信号
	I2C_SDA_IN();
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_SET);  // 释放SDA线，由从设备控制
	DelayUs(2);
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);  // 拉高SCL，从设备在此期间拉低SDA表示ACK
	DelayUs(2);

	// 等待从设备拉低SDA（ACK信号）
	// 如果从设备正常接收数据，会在SCL高电平期间将SDA拉低
	while(HAL_GPIO_ReadPin(GPIOX, IIC_IO_SDA))
	{
		tempTime++;
		// 超时保护：300次循环约600us，说明从设备无响应或总线故障
		if(tempTime>300)
		{
			IIC_stop();  // 发送停止信号释放总线
			return 1;    // 返回1表示等待ACK失败
		}
	}
	// 检测到ACK信号，拉低SCL准备下一次数据传输
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
	return 0;  // 返回0表示等待ACK成功
}

void IIC_send_byte(uint8_t txd)
{
	uint8_t i=0;
	I2C_SDA_OUT();  // 确保SDA为输出模式
	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);  // 拉低SCL，准备发送数据
	// 逐位发送8位数据，从最高位(MSB)开始发送
	for(i=0;i<8;i++)
	{
		// 判断当前最高位是1还是0，设置SDA电平
		if(txd&0x80)
			HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOX, IIC_IO_SDA, GPIO_PIN_RESET);
		txd<<=1;  // 左移一位，准备发送下一位
		DelayUs(5);
		// I2C协议：SCL高电平期间数据必须保持稳定，从设备在此期间读取SDA
		HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);
		DelayUs(5);
		// 拉低SCL，允许主机改变SDA数据
		HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);
		DelayUs(2);
	}
}

uint8_t IIC_read_byte(uint8_t ack)
{
	uint8_t i=0,receive=0;
	I2C_SDA_IN();  // 切换到输入模式，准备读取从设备数据
  for(i=0;i<8;i++)
  {
   	HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_RESET);  // 拉低SCL，从设备准备数据
		DelayUs(5);
		// 拉高SCL，从设备在此期间将数据放到SDA线上
		HAL_GPIO_WritePin(GPIOX, IIC_IO_SCL, GPIO_PIN_SET);
		DelayUs(5);
		receive<<=1;  // 左移一位，为新数据腾出位置
		// 读取SDA电平，如果是1则最低位加1
		if(HAL_GPIO_ReadPin(GPIOX, IIC_IO_SDA))
		   receive++;
		DelayUs(2);
  }

  // 读取完8位数据后，向从设备发送ACK或NACK
  // ack=1表示发送ACK（继续接收下一字节），ack=0表示发送NACK（停止接收）
  if(!ack)
	  IIC_noack();
	else
		IIC_ack();

	return receive;
}

// 读数据（不动）
// 功能：从I2C从设备指定寄存器读取多个字节数据
// 流程：START → 发送设备地址(写) → 发送寄存器地址 → START → 发送设备地址(读) → 读取数据 → STOP
uint8_t I2C_Read_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	// 第一步：发送起始信号
	IIC_start();
	// 第二步：发送7位设备地址+写标志位(0)，通知从设备准备接收寄存器地址
	// CAM_DEFAULT_I2C_ADDRESS << 1 将7位地址左移，最低位0表示写操作
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);
	if(IIC_wait_ack() == 1)  // 等待从设备ACK，失败则停止并返回错误
	{
		IIC_stop();
		return 1;
	}
	// 第三步：发送要读取的寄存器地址
	IIC_send_byte(Reg);
	if(IIC_wait_ack() == 1)
	{
		IIC_stop();
		return 1;
	}
	// 第四步：发送重复起始信号(Repeated START)，切换为读操作
	// 重复START不需要先发送STOP，直接开始新的通信
	IIC_start();
	// 第五步：发送7位设备地址+读标志位(1)，通知从设备准备发送数据
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1);
	if(IIC_wait_ack() == 1)
	{
		IIC_stop();
		return 1;
	}
	// 第六步：循环读取指定长度的数据
	for(i=0;i<Len;i++)
	{
		// 如果不是最后一个字节，发送ACK表示继续接收
		// 如果是最后一个字节，发送NACK表示接收完毕
		if(i != Len-1)
			Buf[i] = IIC_read_byte(1);  // 1表示发送ACK
		else
			Buf[i] = IIC_read_byte(0);  // 0表示发送NACK
	}
	// 第七步：发送停止信号，释放I2C总线
	IIC_stop();
	return 0;  // 返回0表示读取成功
}

// 写数据（不动）
int8_t I2C_Write_Len(int8_t Reg,int8_t *Buf,int8_t Len)
{
	uint8_t i;
	IIC_start();
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);
	if(IIC_wait_ack() == 1)
	{
		IIC_stop();
		return 1;
	}
	IIC_send_byte(Reg);
	if(IIC_wait_ack() == 1)
	{
		IIC_stop();
		return 1;
	}
	for(i =0;i<Len;i++)
	{
		IIC_send_byte(Buf[i]);
		if(IIC_wait_ack() == 1)
		{
			IIC_stop();
			return 1;
		}
	}
	IIC_stop();
	return 0;
}

// ==================== IIC2 模块(服务巡线模块) ====================

void I2C2_SDA_OUT(void)
{
  GPIO_InitTypeDef IIC2_GPIO_InitStructure;
	IIC2_GPIO_InitStructure.Pin=IIC2_IO_SDA;
	IIC2_GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	IIC2_GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_OD;
	IIC2_GPIO_InitStructure.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOX2,&IIC2_GPIO_InitStructure);
}

void I2C2_SDA_IN(void)
{
	GPIO_InitTypeDef IIC2_GPIO_InitStructure;
	IIC2_GPIO_InitStructure.Pin=IIC2_IO_SDA;
	IIC2_GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	IIC2_GPIO_InitStructure.Mode=GPIO_MODE_INPUT;
	IIC2_GPIO_InitStructure.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOX2,&IIC2_GPIO_InitStructure);
}

void IIC2_init()
{
	GPIO_InitTypeDef IIC2_GPIO_InitStructure;
	__HAL_RCC_GPIOC_CLK_ENABLE();
	IIC2_GPIO_InitStructure.Pin = IIC2_IO_SDA |IIC2_IO_SCL;
	IIC2_GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	IIC2_GPIO_InitStructure.Pull = GPIO_PULLUP;
	IIC2_GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOX2, &IIC2_GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOX2,IIC2_IO_SDA|IIC2_IO_SCL, GPIO_PIN_SET);
}

void IIC2_start()
{
	I2C2_SDA_OUT();
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
	DelayUs(25);  // 加长延时
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_RESET);
	DelayUs(25);
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
}

void IIC2_stop()
{
	I2C2_SDA_OUT();
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_RESET);
	DelayUs(25);
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_SET);
	DelayUs(25);	
}

void IIC2_ack()
{
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
	I2C2_SDA_OUT();
  HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_RESET);
  DelayUs(25);
  HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
  DelayUs(25);
  HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
}

void IIC2_noack()
{
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
	I2C2_SDA_OUT();
  HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_SET);
  DelayUs(25);	
  HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
  DelayUs(25);	
  HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
}

uint8_t IIC2_wait_ack()
{
	uint8_t tempTime=0;
	I2C2_SDA_IN();
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_SET);
	DelayUs(25);
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
	DelayUs(25);

	while(HAL_GPIO_ReadPin(GPIOX2, IIC2_IO_SDA))
	{
		tempTime++;
		if(tempTime>150)  // 超时加长，更稳
		{
			IIC2_stop();
			return 1;
		}
	}
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
	return 0;
}

void IIC2_send_byte(uint8_t txd)
{
	uint8_t i=0;
	I2C2_SDA_OUT();
	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
	for(i=0;i<8;i++)
	{
		if(txd&0x80)
			HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SDA, GPIO_PIN_RESET);
		txd<<=1;
		DelayUs(25);
		HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
		DelayUs(25);
		HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
		DelayUs(25);	
	}
}

uint8_t IIC2_read_byte(uint8_t ack)
{
	uint8_t i=0,receive=0;
	I2C2_SDA_IN();
  for(i=0;i<8;i++)
  {
   	HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_RESET);
		DelayUs(25);
		HAL_GPIO_WritePin(GPIOX2, IIC2_IO_SCL, GPIO_PIN_SET);
		DelayUs(25); 
		DelayUs(25);
		receive<<=1;
		if(HAL_GPIO_ReadPin(GPIOX2, IIC2_IO_SDA))
		   receive++;
		DelayUs(25);
  }

  if(!ack)
	  IIC2_noack();
	else
		IIC2_ack();

	return receive;
}

// 读数据
uint8_t I2C2_Read_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	IIC2_start();
	IIC2_send_byte((IR_DEV_ADDR << 1) | 0);
	if(IIC2_wait_ack() == 1)
	{
		IIC2_stop();
		return 1;
	}
	IIC2_send_byte(Reg);
	if(IIC2_wait_ack() == 1)
	{
		IIC2_stop();
		return 1;
	}
	IIC2_start();
	IIC2_send_byte((IR_DEV_ADDR << 1) | 1);
	if(IIC2_wait_ack() == 1)
	{
		IIC2_stop();
		return 1;
	}
	for(i=0;i<Len;i++)
	{
		if(i != Len-1)
			Buf[i] = IIC2_read_byte(1);
		else
			Buf[i] = IIC2_read_byte(0);
	}
	IIC2_stop();
	return 0;
}

// 写数据
int8_t I2C2_Write_Len(int8_t Reg,int8_t *Buf,int8_t Len)
{
	uint8_t i;
	IIC2_start();
	IIC2_send_byte((IR_DEV_ADDR << 1) | 0);
	if(IIC2_wait_ack() == 1)
	{
		IIC2_stop();
		return 1;
	}
	IIC2_send_byte(Reg);
	if(IIC2_wait_ack() == 1)
	{
		IIC2_stop();
		return 1;
	}
	for(i =0;i<Len;i++)
	{
		IIC2_send_byte(Buf[i]);
		if(IIC2_wait_ack() == 1)
		{
			IIC2_stop();
			return 1;
		}
	}
	IIC2_stop();
	return 0;
}
