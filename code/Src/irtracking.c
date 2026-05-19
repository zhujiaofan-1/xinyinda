#include "irtracking.h"
#include "Headfile.h"
#include "Turn.h"

int16_t g_turn_encoder_90 = 20;
int16_t g_turn_encoder_180 = 40;
int16_t g_turn_encoder_straight = 50;

/**
 * @brief  读取红外巡线模块8个传感器状态
 * @param  s1-s8: 8个传感器状态指针，0表示黑线，1表示白底
 * @retval 无
 */
void irtacking_Read(uint8_t *s1,uint8_t *s2,uint8_t *s3,uint8_t *s4,uint8_t *s5,uint8_t *s6,uint8_t *s7,uint8_t *s8)
{
    uint8_t irtacking_data = 0x00;
    // 通过I2C2读取巡线模块传感器状态寄存器（1字节）
    // 每个bit对应一个传感器：0表示黑线，1表示白底
    I2C2_Read_Len(IR_READ_ADDR, &irtacking_data, 1);
	
	
    // 将8位数据逐位提取到对应的传感器变量中
    // s1对应最高位(bit7)，s8对应最低位(bit0)
    *s1 = (irtacking_data >> 7) & 0x01;
    *s2 = (irtacking_data >> 6) & 0x01;
    *s3 = (irtacking_data >> 5) & 0x01;
    *s4 = (irtacking_data >> 4) & 0x01;
    *s5 = (irtacking_data >> 3) & 0x01;
    *s6 = (irtacking_data >> 2) & 0x01;
    *s7 = (irtacking_data >> 1) & 0x01;
    *s8 = (irtacking_data >> 0) & 0x01;
}

//巡线探头的处理
// static void track_deal_four(u8 *s1,u8 *s2,u8 *s3,u8 *s4,u8 *s5,u8 *s6,u8 *s7,u8 *s8)
// {
// 	*s1 = IN_X1;
// 	*s2 = IN_X2;
// 	*s3 = IN_X3;
// 	*s4 = IN_X4;
	
// 	*s5 = IN_X5;
// 	*s6 = IN_X6;
// 	*s7 = IN_X7;
// 	*s8 = IN_X8;
// }



/*************************PID*用PID进行巡黑线********************************/

//pid_t pid_IRR;
////增量PID
//#define IRR_PID_KP       (0) //这可能是接近最好的参数
//#define IRR_PID_KI       (0)
//#define IRR_PID_KD       (0)//kp固定情况下 不能往上调


#define IRTrack_Trun_KP (8)   // 比例系数，决定响应速度
#define IRTrack_Trun_KI (0)   // 积分系数，当前未使用
#define IRTrack_Trun_KD (2)   // 微分系数，抑制震荡，提高稳定性

int pid_output_IRR = 0;

#define IRR_SPEED 			  80   // 巡线速度（范围-100~+100）
#define IRTrack_Minddle    0    // 中间的目标值

/**
 * @brief  位置式PID计算巡线偏差
 * @param  actual_value: 当前传感器偏差值
 * @retval PID输出值
 */
float APP_ELE_PID_Calc(int8_t actual_value)
{

	float IRTrackTurn = 0;
	int8_t error;
	static int8_t error_last=0;  // 保存上一次的误差，用于微分计算
	static float IRTrack_Integral;// 积分项累加器
	
	// 计算当前偏差：实际传感器值与中间目标值的差值
	// IRTrack_Minddle=0表示理想状态是传感器居中
	error=actual_value-IRTrack_Minddle;
	
	// 累加误差到积分项，用于消除稳态误差
	IRTrack_Integral +=error;
	
	// 位置式PID公式：输出 = Kp×误差 + Ki×积分 + Kd×(当前误差-上次误差)
	// Kp=450: 比例系数，决定响应速度
	// Ki=0: 积分系数，当前未使用
	// Kd=30: 微分系数，抑制震荡，提高稳定性
	IRTrackTurn=error*IRTrack_Trun_KP
							+IRTrack_Trun_KI*IRTrack_Integral
							+(error - error_last)*IRTrack_Trun_KD;
	return IRTrackTurn;
}

/**
 * @brief  巡线控制函数，根据传感器状态计算PID输出
 * @param  无
 * @retval 无
 */
//x1-x8 从左往右数
uint8_t LineWalking(void)
{
	static int8_t err = 0;
	static uint8_t x1,x2,x3,x4,x5,x6,x7,x8;
	irtacking_Read(&x1,&x2,&x3,&x4,&x5,&x6,&x7,&x8);

	// 优先判断特殊路况
	// 情况1：x1、x3、x4、x5、x8都在黑线上，说明车身居中，偏差为0
  if(x1 == 0 &&  x3 == 0 && x4 == 0 && x5 == 0 && x8 == 0 )
	{
		err = 0;
	}
	// 情况2：左直角转弯 - x1或x2在黑线且x8在白底，说明左侧有黑线
	else if((x1 == 0 || x2 == 0 ) && x8 == 1)
	{
		err = -15;  // 负偏差表示需要向左转
	}
	// 情况3：右直角转弯 - x7或x8在黑线且x1在白底，说明右侧有黑线
	else if((x7 == 0 ||  x8 == 0) && x1 == 1) 
	{
		err = 15;  // 正偏差表示需要向右转
	}
	
	// 以下是直线巡线判断，根据传感器组合计算偏差值
	// 偏差值范围：-3到+3，负值表示偏左，正值表示偏右

	// x4在黑线（偏左一点）
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1)
	{
		err = -1;
	}
	// x3和x4在黑线（偏左较多）
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1)
	{
		err = -2;
	}
	// x3在黑线（偏左）
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1)
	{
		err = -2;
	}
	
	// x2和x3在黑线（偏左很多）
	else if(x1 == 1 && x2 == 0  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1)
	{
		err = -3;
	}

	// 右侧传感器判断（对称逻辑）
	// x5在黑线（偏右一点）
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 1  && x7 == 1 && x8 == 1)
	{
		err = 1;
	} 
	// x5和x6在黑线（偏右较多）
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 0  && x7 == 1 && x8 == 1)
	{
		err = 2;
	}
	// x6在黑线（偏右）
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 1 && x8 == 1)
	{
		err = 2;
	}
	// x6和x7在黑线（偏右很多）
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 0 && x8 == 1)
	{
		err = 3;
	}
	
	// x4和x5都在黑线，说明车身居中
	else if(x1 == 1 &&x2 == 1 &&x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1&& x8 == 1)
	{
		err = 0;
	}
	
	// 其他情况保持上一个状态，维持当前转向
	
	// 调用PID计算函数，将偏差值转换为电机控制量
	pid_output_IRR = (int)(APP_ELE_PID_Calc(err));
	
    // 电机控制：根据PID输出和巡线速度控制左右轮差速
	Motion_Ctrl(IRR_SPEED, 0, pid_output_IRR, 0);

	return 0;
}
