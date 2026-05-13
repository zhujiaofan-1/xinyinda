#include "Headfile.h"
#include "LCD_Show.h"



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
    
    uint8_t CardID[5];
    uint8_t card_found = 0;
    
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

        if(MFRC522_Check(CardID) == MI_OK)
        {
            snprintf(Text, sizeof(Text), "RC522:%02X%02X%02X%02X", CardID[0],CardID[1],CardID[2],CardID[3]);
            LCD_DisplayText(10, 100, Text);
            card_found = 1;
        }
        else
        {
            if(card_found)
            {
                LCD_DisplayText(10, 100, "No Card         ");
                card_found = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
}
