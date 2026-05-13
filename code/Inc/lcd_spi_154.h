#ifndef __spi_lcd
#define __spi_lcd

#include "main.h"

#include "lcd_fonts.h"	// 图片锟斤拷锟街匡拷锟侥硷拷锟斤拷锟角憋拷锟诫，锟矫伙拷锟斤拷锟斤拷锟斤拷删锟斤拷
#include	"lcd_image.h"


/*----------------------------------------------- 锟斤拷锟斤拷锟斤拷 -------------------------------------------*/

#define LCD_Width     240		// LCD锟斤拷锟斤拷锟截筹拷锟斤拷
#define LCD_Height    240		// LCD锟斤拷锟斤拷锟截匡拷锟斤拷

// 锟斤拷示锟斤拷锟斤拷锟斤拷锟?
// 使锟斤拷示锟斤拷锟斤拷LCD_DisplayDirection(Direction_H) 锟斤拷锟斤拷锟斤拷幕锟斤拷锟斤拷锟斤拷示
#define	Direction_H				0					//LCD锟斤拷锟斤拷锟斤拷示
#define	Direction_H_Flip	   1					//LCD锟斤拷锟斤拷锟斤拷示,锟斤拷锟铰凤拷转
#define	Direction_V				2					//LCD锟斤拷锟斤拷锟斤拷示 
#define	Direction_V_Flip	   3					//LCD锟斤拷锟斤拷锟斤拷示,锟斤拷锟铰凤拷转 

// 锟斤拷锟矫憋拷锟斤拷锟斤拷示时锟斤拷锟斤拷位锟斤拷0锟斤拷锟角诧拷锟秸革拷
// 只锟斤拷 LCD_DisplayNumber() 锟斤拷示锟斤拷锟斤拷 锟斤拷 LCD_DisplayDecimals()锟斤拷示小锟斤拷 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫碉拷
// 使锟斤拷示锟斤拷锟斤拷 LCD_ShowNumMode(Fill_Zero) 锟斤拷锟矫讹拷锟斤拷位锟斤拷锟?锟斤拷锟斤拷锟斤拷 123 锟斤拷锟斤拷锟斤拷示为 000123
#define  Fill_Zero  0		//锟斤拷锟?
#define  Fill_Space 1		//锟斤拷锟秸革拷


/*---------------------------------------- 锟斤拷锟斤拷锟斤拷色 ------------------------------------------------------

 1. 锟斤拷锟斤拷为锟剿凤拷锟斤拷锟矫伙拷使锟矫ｏ拷锟斤拷锟斤拷锟斤拷锟?4位 RGB888锟斤拷色锟斤拷然锟斤拷锟斤拷通锟斤拷锟斤拷锟斤拷锟皆讹拷转锟斤拷锟斤拷 16位 RGB565 锟斤拷锟斤拷色
 2. 24位锟斤拷锟斤拷色锟叫ｏ拷锟接革拷位锟斤拷锟斤拷位锟街憋拷锟接?R锟斤拷G锟斤拷B  3锟斤拷锟斤拷色通锟斤拷
 3. 锟矫伙拷锟斤拷锟斤拷锟节碉拷锟斤拷锟矫碉拷色锟斤拷锟饺?4位RGB锟斤拷色锟斤拷锟劫斤拷锟斤拷色锟斤拷锟斤拷LCD_SetColor()锟斤拷LCD_SetBackColor()锟酵匡拷锟斤拷锟斤拷示锟斤拷锟斤拷应锟斤拷锟斤拷色 
 */                                                  						
#define 	LCD_WHITE       0xFFFFFF	 // 锟斤拷锟斤拷色
#define 	LCD_BLACK       0x000000    // 锟斤拷锟斤拷色
                        
#define 	LCD_BLUE        0x0000FF	 //	锟斤拷锟斤拷色
#define 	LCD_GREEN       0x00FF00    //	锟斤拷锟斤拷色
#define 	LCD_RED         0xFF0000    //	锟斤拷锟斤拷色
#define 	LCD_CYAN        0x00FFFF    //	锟斤拷锟斤拷色
#define 	LCD_MAGENTA     0xFF00FF    //	锟较猴拷色
#define 	LCD_YELLOW      0xFFFF00    //	锟斤拷色
#define 	LCD_GREY        0x2C2C2C    //	锟斤拷色
												
#define 	LIGHT_BLUE      0x8080FF    //	锟斤拷锟斤拷色
#define 	LIGHT_GREEN     0x80FF80    //	锟斤拷锟斤拷色
#define 	LIGHT_RED       0xFF8080    //	锟斤拷锟斤拷色
#define 	LIGHT_CYAN      0x80FFFF    //	锟斤拷锟斤拷锟斤拷色
#define 	LIGHT_MAGENTA   0xFF80FF    //	锟斤拷锟较猴拷色
#define 	LIGHT_YELLOW    0xFFFF80    //	锟斤拷锟斤拷色
#define 	LIGHT_GREY      0xA3A3A3    //	锟斤拷锟斤拷色
												
#define 	DARK_BLUE       0x000080    //	锟斤拷锟斤拷色
#define 	DARK_GREEN      0x008000    //	锟斤拷锟斤拷色
#define 	DARK_RED        0x800000    //	锟斤拷锟斤拷色
#define 	DARK_CYAN       0x008080    //	锟斤拷锟斤拷锟斤拷色
#define 	DARK_MAGENTA    0x800080    //	锟斤拷锟较猴拷色
#define 	DARK_YELLOW     0x808000    //	锟斤拷锟斤拷色
#define 	DARK_GREY       0x404040    //	锟斤拷锟斤拷色

/*------------------------------------------------ 锟斤拷锟斤拷锟斤拷锟斤拷 ----------------------------------------------*/

void  SPI_LCD_Init(void);      // 液锟斤拷锟斤拷锟皆硷拷SPI锟斤拷始锟斤拷   
void  LCD_Clear(void);			 // 锟斤拷锟斤拷锟斤拷锟斤拷
void  LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);	// 锟街诧拷锟斤拷锟斤拷锟斤拷锟斤拷

void  LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);	// 锟斤拷锟斤拷锟斤拷锟斤拷		
void  LCD_SetColor(uint32_t Color); 				   //	锟斤拷锟矫伙拷锟斤拷锟斤拷色
void  LCD_SetBackColor(uint32_t Color);  				//	锟斤拷锟矫憋拷锟斤拷锟斤拷色
void  LCD_SetDirection(uint8_t direction);  	      //	锟斤拷锟斤拷锟斤拷示锟斤拷锟斤拷

//>>>>>	锟斤拷示ASCII锟街凤拷
void  LCD_SetAsciiFont(pFONT *fonts);										//	锟斤拷锟斤拷ASCII锟斤拷锟斤拷
void 	LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c);				//	锟斤拷示锟斤拷锟斤拷ASCII锟街凤拷
void 	LCD_DisplayString( uint16_t x, uint16_t y, char *p);	 		//	锟斤拷示ASCII锟街凤拷锟斤拷

//>>>>>	锟斤拷示锟斤拷锟斤拷锟街凤拷锟斤拷锟斤拷锟斤拷ASCII锟斤拷
void 	LCD_SetTextFont(pFONT *fonts);										// 锟斤拷锟斤拷锟侥憋拷锟斤拷锟藉，锟斤拷锟斤拷锟斤拷锟侥猴拷ASCII锟斤拷锟斤拷
void 	LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText);		// 锟斤拷示锟斤拷锟斤拷锟斤拷锟斤拷
void 	LCD_DisplayText(uint16_t x, uint16_t y, char *pText) ;		// 锟斤拷示锟街凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥猴拷ASCII锟街凤拷

//>>>>>	锟斤拷示锟斤拷锟斤拷锟斤拷小锟斤拷
void  LCD_ShowNumMode(uint8_t mode);		// 锟斤拷锟矫憋拷锟斤拷锟斤拷示模式锟斤拷锟斤拷锟斤拷位锟斤拷锟秸革拷锟斤拷锟斤拷锟?
void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number,uint8_t len) ;					// 锟斤拷示锟斤拷锟斤拷
void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double number,uint8_t len,uint8_t decs);	// 锟斤拷示小锟斤拷

//>>>>>	2D图锟轿猴拷锟斤拷
void  LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color);   	//锟斤拷锟斤拷

void  LCD_DrawLine_V(uint16_t x, uint16_t y, uint16_t height);          // 锟斤拷锟斤拷直锟斤拷
void  LCD_DrawLine_H(uint16_t x, uint16_t y, uint16_t width);           // 锟斤拷水平锟斤拷
void  LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);	// 锟斤拷锟斤拷之锟戒画锟斤拷

void  LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//锟斤拷锟斤拷锟斤拷
void  LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r);									//锟斤拷圆
void  LCD_DrawEllipse(int x, int y, int r1, int r2);											//锟斤拷锟斤拷圆

//>>>>>	锟斤拷锟斤拷锟斤拷浜拷锟?
void  LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//锟斤拷锟斤拷锟斤拷
void  LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r);									//锟斤拷锟皆?

//>>>>>	锟斤拷锟狡碉拷色图片
void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)  ;


/*------------------------------------------------------  锟斤拷锟斤拷锟斤拷锟矫猴拷 -------------------------------------------------*/	

#define 	LCD_SCK_PIN      		 GPIO_PIN_3									// SCK锟斤拷锟脚ｏ拷 锟斤拷要锟截讹拷锟斤拷SPI3锟斤拷IO锟节革拷锟斤拷
#define 	LCD_SCK_PORT     		 GPIOB                 					// SCK锟斤拷锟斤拷锟矫碉拷锟侥端匡拷  
#define 	GPIO_LCD_SCK_CLK      __HAL_RCC_GPIOB_CLK_ENABLE()  		// SCK锟斤拷锟斤拷IO锟斤拷时锟斤拷

#define 	LCD_SDA_PIN      		 GPIO_PIN_5									// SDA锟斤拷锟脚ｏ拷 锟斤拷要锟截讹拷锟斤拷SPI3锟斤拷IO锟节革拷锟斤拷
#define 	LCD_SDA_PORT    		 GPIOB                 					// SDA锟斤拷锟斤拷锟矫碉拷锟侥端匡拷  
#define 	GPIO_LCD_SDA_CLK      __HAL_RCC_GPIOB_CLK_ENABLE()  		// SDA锟斤拷锟斤拷IO锟斤拷时锟斤拷


#define 	LCD_CS_PIN       				GPIO_PIN_11							// CS片选锟斤拷锟脚ｏ拷锟酵碉拷平锟斤拷效
#define 	LCD_CS_PORT      				GPIOD                 			// CS锟斤拷锟斤拷锟矫碉拷锟侥端匡拷 
#define 	GPIO_LCD_CS_CLK     			__HAL_RCC_GPIOD_CLK_ENABLE()	// CS锟斤拷锟斤拷IO锟斤拷时锟斤拷

#define  LCD_DC_PIN						GPIO_PIN_12				         // 锟斤拷锟斤拷指锟斤拷选锟斤拷  锟斤拷锟斤拷				
#define	LCD_DC_PORT						GPIOD									// 锟斤拷锟斤拷指锟斤拷选锟斤拷  GPIO锟剿匡拷
#define 	GPIO_LCD_DC_CLK     			__HAL_RCC_GPIOD_CLK_ENABLE()	// 锟斤拷锟斤拷指锟斤拷选锟斤拷  GPIO时锟斤拷 	

#define  LCD_Backlight_PIN				GPIO_PIN_13				         // 锟斤拷锟斤拷  锟斤拷锟斤拷				
#define	LCD_Backlight_PORT			GPIOD									// 锟斤拷锟斤拷 GPIO锟剿匡拷
#define 	GPIO_LCD_Backlight_CLK     __HAL_RCC_GPIOD_CLK_ENABLE()	// 锟斤拷锟斤拷 GPIO时锟斤拷 	


/*--------------------------------------------------------- 锟斤拷锟狡猴拷 ---------------------------------------------------*/

// 锟斤拷为片选锟斤拷锟斤拷锟斤拷要频锟斤拷锟斤拷锟斤拷锟斤拷使锟矫寄达拷锟斤拷效锟绞伙拷锟叫?
#define 	LCD_CS_H    		 	LCD_CS_PORT->BSRR  = LCD_CS_PIN							// 片选锟斤拷锟斤拷
#define 	LCD_CS_L     			LCD_CS_PORT->BSRR  = (uint32_t)LCD_CS_PIN << 16U	// 片选锟斤拷锟斤拷

#define	LCD_DC_Command		   HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET)	  		// 锟酵碉拷平锟斤拷指锟筋传锟斤拷 
#define 	LCD_DC_Data		      HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET)				// 锟竭碉拷平锟斤拷锟斤拷锟捷达拷锟斤拷

#define 	LCD_Backlight_ON      HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_SET)		// 锟竭碉拷平锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
#define 	LCD_Backlight_OFF  	 HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_RESET)	// 锟酵碉拷平锟斤拷锟截闭憋拷锟斤拷



#endif //__spi_lcd




