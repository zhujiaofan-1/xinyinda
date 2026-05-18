#include "Headfile.h"
#include "LCD_Show.h"
#include "navigation.h"
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
    Servo_Init();
    MFRC522_Init();
	volatile int i = 0;
    char Text[20];
    HAL_TIM_Base_Start(&htim3);
    
    volatile uint16_t V = 0;
    
    motor_speed_t speed;
    speed.speed_m1 = 0;
    speed.speed_m2 = 0;
    speed.speed_m3 = 0;
    speed.speed_m4 = 0;

    motor_encoder_t Encoder;

    uint16_t Angle = 90;
    int8_t dir = 1;

    uint32_t last_time = 0;
    
    while (1)
    {
        
        Motor_Set_Speed(speed);
        Motor_Get_Encoder(&Encoder);
        
        snprintf(Text, sizeof(Text), "Encoder = %d        ", Encoder.encoder_m1);
        LCD_DisplayText(10, 20, Text);

        V = Motor_Get_Vol();
		snprintf(Text, sizeof(Text), "V = %dmV       ", V);
        LCD_DisplayText(10, 40, Text);

        float distance = HCSR04_Get_Distance();
        snprintf(Text, sizeof(Text), "Distance = %.2f      ", distance);
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

        uint8_t s1,s2,s3,s4,s5,s6,s7,s8;
        irtacking_Read(&s1,&s2,&s3,&s4,&s5,&s6,&s7,&s8);
        snprintf(Text, sizeof(Text), "%d%d%d%d%d%d%d%d", s1,s2,s3,s4,s5,s6,s7,s8);
        LCD_DisplayText(10, 80, Text);

        // RC522由CardNavTask独占访问，LCD不再直接读取RFID
        // 显示卡片内容（由CardNavTask写入turn_card_content）
        if(turn_card_content[0] != '\0')
        {
            snprintf(Text, sizeof(Text), "Card:%s          ", turn_card_content);
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
        
        // 显示转向方向
        if(turn_action_valid)
        {
            switch(turn_action)
            {
                case TURN_STRAIGHT: LCD_DisplayText(10, 160, "Dir:Straight    "); break;
                case TURN_LEFT:     LCD_DisplayText(10, 160, "Dir:Left        "); break;
                case TURN_RIGHT:    LCD_DisplayText(10, 160, "Dir:Right       "); break;
                case TURN_UTURN:    LCD_DisplayText(10, 160, "Dir:U-Turn      "); break;
                case TURN_ERROR:    LCD_DisplayText(10, 160, "Dir:Error       "); break;
                default:            LCD_DisplayText(10, 160, "Dir:Unknown     "); break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
}
