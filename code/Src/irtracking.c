#include "irtracking.h"
#include "Headfile.h"
#include "Turn.h"

int16_t g_turn_delay_90 = 700;     // 90度转向延时(ms)
int16_t g_turn_delay_180 = 1400;   // 180度转向延时(ms)

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


#define IRTrack_Trun_KP (3)   // 比例系数，决定响应速度
#define IRTrack_Trun_KI (0)   // 积分系数，当前未使用
#define IRTrack_Trun_KD (3)   // 微分系数，抑制震荡，提高稳定性

int16_t g_ir_speed = 20;       // 巡线速度（范围-100~+100）
int16_t g_ir_pid_kp = 3;       // 巡线PID比例系数
int16_t g_ir_pid_ki = 0;       // 巡线PID积分系数
int16_t g_ir_pid_kd = 3;       // 巡线PID微分系数

int pid_output_IRR = 0;

#define IRTrack_Minddle    0    // 中间的目标值

// 丢线缓冲相关变量
static int8_t g_last_err = 0;           // 最后一次有效偏差
static uint16_t g_lost_line_timeout = 0; // 丢线超时计数器
#define LOST_LINE_MAX_TICKS  5         // 丢线后最大保持次数(约500ms @ 10ms周期)

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
	IRTrackTurn=error*g_ir_pid_kp
							+g_ir_pid_ki*IRTrack_Integral
							+(error - error_last)*g_ir_pid_kd;
	return IRTrackTurn;
}

/**
 * @brief  巡线控制函数，根据传感器状态计算PID输出
 * @param  无
 * @retval 0-正常巡线中  1-脱离黑线已停车
 */
//x1-x8 从左往右数
uint8_t LineWalking(void)
{
	static int8_t err = 0;
	static uint8_t x1,x2,x3,x4,x5,x6,x7,x8;
	uint8_t black_count = 0;  // 黑线传感器计数
	irtacking_Read(&x1,&x2,&x3,&x4,&x5,&x6,&x7,&x8);

	// 统计检测到黑线的传感器数量
	if(x1 == 0) black_count++;
	if(x2 == 0) black_count++;
	if(x3 == 0) black_count++;
	if(x4 == 0) black_count++;
	if(x5 == 0) black_count++;
	if(x6 == 0) black_count++;
	if(x7 == 0) black_count++;
	if(x8 == 0) black_count++;

	// 黑线传感器数量异常：全白(0)或全黑(8)或过多(>=6)，说明不在有效黑线上
	if(black_count == 0 || black_count >= 6)
	{
		// 丢线后保持最后一次偏差值继续转向，给转弯留出缓冲时间
		if(black_count == 0)
		{
			// 全白丢线：保持最后偏差继续转向
			if(g_lost_line_timeout < LOST_LINE_MAX_TICKS)
			{
				g_lost_line_timeout++;
				err = g_last_err;  // 保持最后一次偏差
			}
			else
			{
				// 超时仍未找到线，停车
				Motion_Ctrl(0, 0, 0, 0);
				err = 0;
				return 1;
			}
		}
		else
		{
			// 黑线过多，直接停车
			Motion_Ctrl(0, 0, 0, 0);
			err = 0;
			return 1;
		}
	}
	else
	{
		// 正常巡线中，重置丢线计数器
		g_lost_line_timeout = 0;
	}

	// 巡线偏差判断，传感器从左到右：x1 x2 x3 x4 x5 x6 x7 x8
	// 0=黑线, 1=白底, 负值=偏左需左转, 正值=偏右需右转

	// 居中：x4和x5任意一个或都在黑线
	if(x4 == 0 || x5 == 0)
	{
		err = 0;
	}
	// 偏左较多：x3在黑线
	else if(x3 == 0 && x4 == 1)
	{
		err = -2;
	}
	// 偏右较多：x6在黑线
	else if(x5 == 1 && x6 == 0)
	{
		err = 2;
	}
	// 偏左很多：x2在黑线
	else if(x2 == 0 && x3 == 1)
	{
		err = -3;
	}
	// 偏右很多：x7在黑线
	else if(x6 == 1 && x7 == 0)
	{
		err = 3;
	}
	// 极左：x1在黑线，大幅左转
	else if(x1 == 0 && x2 == 1)
	{
		err = -5;
	}
	// 极右：x8在黑线，大幅右转
	else if(x7 == 1 && x8 == 0)
	{
		err = 5;
	}
	// 其他情况保持上一个err值

	// 记录最后一次有效偏差，用于丢线缓冲
	if(black_count > 0 && black_count < 6)
	{
		g_last_err = err;
	}

	// 调用PID计算函数，将偏差值转换为电机控制量
	pid_output_IRR = (int)(APP_ELE_PID_Calc(err));
	
    // 电机控制：根据PID输出和巡线速度控制左右轮差速
	Motion_Ctrl(g_ir_speed, 0, pid_output_IRR, 0);

	return 0;
}
