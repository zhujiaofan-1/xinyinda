#include "Headfile.h"
#include "LCD_Show.h"
#include "Turn.h"



/**
 * @brief  LCD显示初始化
 * @param  无
 * @retval 无
 */
static void LCD_Show_Init(void)
{
    SPI_LCD_Init();
    LCD_SetBackColor(LCD_WHITE);
    LCD_Clear();
    LCD_SetTextFont(&CH_Font24); 
    LCD_SetColor(LCD_RED);

}

/**
 * @brief  LCD显示任务
 * @param  param: 任务参数
 * @retval 无
 */
void LCD_Show_Task(void* param)
{
    
    LCD_Show_Init();
    char Text[20];

    volatile uint16_t V = 0;

    motor_encoder_t Encoder;

    while (1)
    {
        // 获取编码器数据并显示
        Motor_Get_Encoder(&Encoder);

        snprintf(Text, sizeof(Text), "Encoder = %d        ", Encoder.encoder_m1);
        LCD_DisplayText(10, 20, Text);

        // 获取电池电压并显示
        V = Motor_Get_Vol();
		snprintf(Text, sizeof(Text), "V = %dmV       ", V);
        LCD_DisplayText(10, 40, Text);

        // 显示超声波测距值
        snprintf(Text, sizeof(Text), "Distance = %.2f      ", g_ultrasonic_distance);
        LCD_DisplayText(10, 60, Text);

        // if(distance <= 5.0f && HAL_GetTick() - last_time > 1000)
        // {
        //     last_time = HAL_GetTick();
        //     switch(dir)
        //     {
        //         case 0:         //左边
        //             dir = 1;
        //             Angle = 90;
        //             break;
        //         case 1:         //中间
        //             dir = -1;
        //             Angle = 0;
        //             break;
        //         case -1:        //右边
        //             dir = 0;
        //             Angle = 180;
        //             break;

        //         default:break;
        //     }
            
            
        // }

        // Servo_SetAngle(Angle);

        // IR传感器数据由irtracking_Task独占读取，LCD不再直接访问I2C2避免冲突
        // 如需查看IR数据，可通过irtracking模块的全局变量获取
        LCD_DisplayText(10, 80, "IR:See Task     ");

        // RC522由CardNavTask独占访问，LCD不再直接读取RFID
        // 显示卡片内容（由CardNavTask写入turn_card_content）
        if(turn_card_content[0] != '\0')
        {
            char card_buf[17];
            for(int i = 0; i < 16; i++) {
                card_buf[i] = turn_card_content[i];
                if(card_buf[i] == '\0') break;
            }
            card_buf[16] = '\0';
            snprintf(Text, sizeof(Text), "Card:%s          ", card_buf);
            LCD_DisplayText(10, 100, Text);
        }
        else
        {
            LCD_DisplayText(10, 100, "No Card         ");
        }
        
        // 显示小车当前方位
        switch(turn_current_dir)
        {
            case DIR_N: LCD_DisplayText(10, 120, "Pos:North       "); break;
            case DIR_S: LCD_DisplayText(10, 120, "Pos:South       "); break;
            case DIR_W: LCD_DisplayText(10, 120, "Pos:West        "); break;
            case DIR_E: LCD_DisplayText(10, 120, "Pos:East        "); break;
            default:    LCD_DisplayText(10, 120, "Pos:Unknown     "); break;
        }
        
        // 显示目标房间
        snprintf(Text, sizeof(Text), "Target:%d        ", turn_target_room);
        LCD_DisplayText(10, 140, Text);
        
        // 显示导航状态
        switch(nav_state)
        {
            case NAV_IDLE:      LCD_DisplayText(10, 160, "State:Idle      "); break;
            case NAV_GOING:     LCD_DisplayText(10, 160, "State:Going     "); break;
            case NAV_RETURNING: LCD_DisplayText(10, 160, "State:Return    "); break;
            default:            LCD_DisplayText(10, 160, "State:Unknown   "); break;
        }
        
        // 显示转向方向
        if(turn_action_valid)
        {
            switch(turn_action)
            {
                case TURN_STRAIGHT: LCD_DisplayText(10, 180, "Dir:Straight    "); break;
                case TURN_LEFT:     LCD_DisplayText(10, 180, "Dir:Left        "); break;
                case TURN_RIGHT:    LCD_DisplayText(10, 180, "Dir:Right       "); break;
                case TURN_UTURN:    LCD_DisplayText(10, 180, "Dir:U-Turn      "); break;
                case TURN_ERROR:    LCD_DisplayText(10, 180, "Dir:Error       "); break;
                default:            LCD_DisplayText(10, 180, "Dir:Unknown     "); break;
            }
        }


        // 显示SU03T语音接收数据
        snprintf(Text, sizeof(Text), "SU03T:%02X %02X %02X    ",
                 g_su03t_recv_data[0], g_su03t_recv_data[1], g_su03t_recv_data[2]);
        LCD_DisplayText(10, 200, Text);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
}
