/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_LCD200
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_LCD200_H_
#define _NEUQ_LCD200_H_

#include "stm32f4xx_hal.h"
#include "NEUQ_CONSTS.h"


// LCD使用独立SPI3，避免覆盖板载Flash使用的SPI1配置。
#define LCD200_SPI_USE  (Hard_SPI_3)
#define LCD200_SCK_USE  (Hard_SPI_3_SCK_PB3)
#define LCD200_SDA_USE  (Hard_SPI_3_SDA_PB5)
#define LCD200_CS_USE   (PB4)
#define LCD200_DC_USE   (PD5)
#define LCD200_BL_USE   (PD3)
#define LCD200_RST_USE  (PE2)

#define LCD200_Width     240		// LCD的像素长度
#define LCD200_Height    320		// LCD的像素宽度

/*---------------------------------------- 常用颜色 ------------------------------------------------------

 1. 这里为了方便用户使用，定义的是24位 RGB888颜色，然后再通过代码自动转换成 16位 RGB565 的颜色
 2. 24位的颜色中，从高位到低位分别对应 R、G、B  3个颜色通道
 3. 用户可以在电脑用调色板获取24位RGB颜色，再将颜色输入LCD_SetColor()或LCD_SetBackColor()就可以显示出相应的颜色 
 */                                                  						
#define 	LCD_WHITE       0xFFFFFF	 // 纯白色
#define 	LCD_BLACK       0x000000    // 纯黑色
                        
#define 	LCD_BLUE        0x0000FF	 //	纯蓝色
#define 	LCD_GREEN       0x00FF00    //	纯绿色
#define 	LCD_RED         0xFF0000    //	纯红色
#define 	LCD_CYAN        0x00FFFF    //	蓝绿色
#define 	LCD_MAGENTA     0xFF00FF    //	紫红色
#define 	LCD_YELLOW      0xFFFF00    //	黄色
#define 	LCD_GREY        0x2C2C2C    //	灰色
												
#define 	LIGHT_BLUE      0x8080FF    //	亮蓝色
#define 	LIGHT_GREEN     0x80FF80    //	亮绿色
#define 	LIGHT_RED       0xFF8080    //	亮红色
#define 	LIGHT_CYAN      0x80FFFF    //	亮蓝绿色
#define 	LIGHT_MAGENTA   0xFF80FF    //	亮紫红色
#define 	LIGHT_YELLOW    0xFFFF80    //	亮黄色
#define 	LIGHT_GREY      0xA3A3A3    //	亮灰色
												
#define 	DARK_BLUE       0x000080    //	暗蓝色
#define 	DARK_GREEN      0x008000    //	暗绿色
#define 	DARK_RED        0x800000    //	暗红色
#define 	DARK_CYAN       0x008080    //	暗蓝绿色
#define 	DARK_MAGENTA    0x800080    //	暗紫红色
#define 	DARK_YELLOW     0x808000    //	暗黄色
#define 	DARK_GREY       0x404040    //	暗灰色

// 设置变量显示时多余位补0还是补空格
// 只有 LCD_DisplayNumber() 显示整数 和 LCD_DisplayDecimals()显示小数 这两个函数用到
// 使用示例： LCD_ShowNumMode(Fill_Zero) 设置多余位填充0，例如 123 可以显示为 000123
#define  Fill_Zero  0		//填充0
#define  Fill_Space 1		//填充空格

typedef enum{
    LCD200_DIR_Cross,           //横向
    LCD200_DIR_Cross_Flip,      //横向翻转
    LCD200_DIR_Parallel,        //纵向
    LCD200_DIR_Parallel_Flip,   //纵向翻转
}LCD200_DIR_ENUM;

typedef struct{
    uint16_t Background_Color;
    uint16_t Pen_Color;
    LCD200_DIR_ENUM Direction;

    uint8_t  ShowNum_Mode;		// 数字显示模式,Fill_Space 填充空格，Fill_Zero 填充零
    uint16_t Width;            // 屏幕像素长度
    uint16_t Height;           // 屏幕像素宽度	
    uint8_t  X_Offset;         // X坐标偏移，用于设置屏幕控制器的显存写入方式
    uint8_t  Y_Offset;         // Y坐标偏移，用于设置屏幕控制器的显存写入方式
}LCD200_Config_Struct;

void NEUQ_LCD200_init(void);
void NEUQ_LCD200_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void NEUQ_LCD200_SetPenColor(uint32_t Color);
void NEUQ_LCD200_SetBackColor(uint32_t Color);
void NEUQ_LCD200_SetDirection(LCD200_DIR_ENUM dir);
void NEUQ_LCD200_SetAsciiFont(pFONT *Asciifonts);
void NEUQ_LCD200_SetTextFont(pFONT *fonts);
void NEUQ_LCD200_ShowNumMode(uint8_t mode);


void NEUQ_LCD200_WriteCommand(uint8_t lcd_command);
void NEUQ_LCD200_WriteData_8bit(uint8_t lcd_data);
void NEUQ_LCD200_WriteData_16bit_in_8bit_mode(uint16_t lcd_data);
void NEUQ_LCD200_WriteBuff(uint16_t *DataBuff, uint16_t DataSize);
void NEUQ_LCD200_Clear(void);

void NEUQ_LCD200_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void NEUQ_LCD200_DrawPoint(uint16_t x,uint16_t y,uint32_t color);
void NEUQ_LCD200_DisplayChar(uint16_t x, uint16_t y,uint8_t c);
void NEUQ_LCD200_DisplayString( uint16_t x, uint16_t y, char *p);
void NEUQ_LCD200_Display_int32( uint16_t x, uint16_t y, int32_t number, uint8_t len);
void NEUQ_LCD200_DisplayFloat(uint16_t x, uint16_t y, float number, uint8_t len, uint8_t decimal);
void NEUQ_LCD200_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void NEUQ_LCD200_DrawCircle(uint16_t x, uint16_t y, uint16_t r);
void NEUQ_LCD200_DisplayPicture(uint16_t x, uint16_t y, uint16_t width, uint16_t height,uint16_t *rgb565_data);



#endif 
