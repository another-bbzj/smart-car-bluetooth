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
* 2025-09-20        张瑞        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

LCD200_Config_Struct  LCD200_Config={0};
static pFONT *LCD200_AsciiFonts;		// 英文字体，ASCII字符集
// 因为这类SPI的屏幕，每次更新显示时，需要先配置坐标区域、再写显存，
// 在显示字符时，如果是一个个点去写坐标写显存，会非常慢，
// 因此开辟一片缓冲区，先将需要显示的数据写进缓冲区，最后再批量写入显存。
// 用户可以根据实际情况去修改此处缓冲区的大小，
// 例如，用户需要显示32*32的汉字时，需要的大小为 32*32*2 = 2048 字节（每个像素点占2字节）
uint16_t  LCD200_Buff[4096];        // LCD缓冲区，16位宽（每个像素点占2字节）

//==============================================================================================
// 函数名称 : NEUQ_LCD200_WriteCommand
// 函数功能 : 向LCD200写入一字节指令
// 输入参数 : lcd_command : 要写入的指令
// 输出参数 : 无
// 返 回 值 : 无
//==============================================================================================
void NEUQ_LCD200_WriteCommand(uint8_t lcd_command){
	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);
	NEUQ_GPIO_write(LCD200_DC_USE,GPIO_Low);//	DC引脚输出低，代表写指令
	NEUQ_Hard_SPI_write_byte(LCD200_SPI_USE,LCD200_CS_USE,lcd_command);// 发送数据
	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_GPIO_write(LCD200_DC_USE,GPIO_High);//	DC引脚输出高，代表写数据		
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_WriteData_8bit
// 函数功能 : 向LCD200写入一字节数据
// 输入参数 : lcd_data : 要写入的数据
// 输出参数 : 无
// 返 回 值 : 无
//==============================================================================================
void NEUQ_LCD200_WriteData_8bit(uint8_t lcd_data){
	NEUQ_Hard_SPI_write_byte(LCD200_SPI_USE,LCD200_CS_USE,lcd_data);// 发送数据
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_WriteData_16bit
// 函数功能 : 向LCD200写入两字节数据
// 输入参数 : lcd_data : 要写入的数据
// 输出参数 : 无
// 返 回 值 : 无
//==============================================================================================
void NEUQ_LCD200_WriteData_16bit(uint16_t lcd_data){
	NEUQ_Hard_SPI_write_2byte(LCD200_SPI_USE,LCD200_CS_USE,lcd_data);// 发送数据
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_WriteData_16bit_in_8bit_mode
// 函数功能 : 向LCD200写入两字节数据，使用8位SPI模式
// 输入参数 : lcd_data : 要写入的数据
// 输出参数 : 无
// 返 回 值 : 无
//==============================================================================================
void NEUQ_LCD200_WriteData_16bit_in_8bit_mode(uint16_t lcd_data){
	NEUQ_Hard_SPI_write_byte(LCD200_SPI_USE,LCD200_CS_USE,lcd_data>>8);// 发送高字节
	NEUQ_Hard_SPI_write_byte(LCD200_SPI_USE,LCD200_CS_USE,lcd_data);// 发送低字节
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_WriteBuff
// 函数功能 : 向LCD200写入一片缓冲区数据
// 输入参数 : DataBuff : 要写入的数据缓冲区
//            DataSize : 要写入的数据长度，单位为 16位数据个数
// 输出参数 : 无
// 返 回 值 : 无
//==============================================================================================
void NEUQ_LCD200_WriteBuff(uint16_t *DataBuff, uint16_t DataSize){
	NEUQ_Hard_SPI_switch_to_16bit_mode(LCD200_SPI_USE);
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);

	NEUQ_Hard_SPI_write_16bit_data(LCD200_SPI_USE,LCD200_CS_USE,DataBuff,DataSize);
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);

	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_Hard_SPI_switch_to_8bit_mode(LCD200_SPI_USE);
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_init
// 函数功能 : 初始化LCD200驱动
// 输入参数 : 无
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_init();
//==============================================================================================
void NEUQ_LCD200_init(void){
	NEUQ_Hard_SPI_init(LCD200_SPI_USE,LCD200_SCK_USE,LCD200_SDA_USE,LCD200_CS_USE);
	NEUQ_Hard_SPI_switch_to_8bit_mode(LCD200_SPI_USE);
	printf("spi inited\n");
	NEUQ_GPIO_init(LCD200_DC_USE,GPIO_Mode_Output_PushPull,GPIO_High);
	NEUQ_GPIO_init(LCD200_BL_USE,GPIO_Mode_Output_PushPull,GPIO_Low);
	NEUQ_GPIO_init(LCD200_RST_USE,GPIO_Mode_Output_PushPull,GPIO_Low);
    HAL_Delay(5);               // 屏幕刚完成复位时（包括上电复位），需要等待5ms才能发送指令
	NEUQ_GPIO_write(LCD200_RST_USE,GPIO_High);
	HAL_Delay(5); 

	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC，开始通信

 	NEUQ_LCD200_WriteCommand(0x36);       // 显存访问控制 指令，用于设置访问显存的方式
printf("cmd write\n");
// HAL_Delay(10);
	NEUQ_LCD200_WriteData_8bit(0x00);     // 配置成 从上到下、从左到右，RGB像素格式
	printf("dat written\n");
	

	NEUQ_LCD200_WriteCommand(0x3A);			// 接口像素格式 指令，用于设置使用 12位、16位还是18位色
	NEUQ_LCD200_WriteData_8bit(0x05);     // 此处配置成 16位 像素格式

// 接下来很多都是电压设置指令，直接使用厂家给设定值
 	NEUQ_LCD200_WriteCommand(0xB2);			
	NEUQ_LCD200_WriteData_8bit(0x0C);
	NEUQ_LCD200_WriteData_8bit(0x0C); 
	NEUQ_LCD200_WriteData_8bit(0x00); 
	NEUQ_LCD200_WriteData_8bit(0x33); 
	NEUQ_LCD200_WriteData_8bit(0x33); 			

	NEUQ_LCD200_WriteCommand(0xB7);		   // 栅极电压设置指令	
	NEUQ_LCD200_WriteData_8bit(0x35);     // VGH = 13.26V，VGL = -10.43V

	NEUQ_LCD200_WriteCommand(0xBB);			// 公共电压设置指令
	NEUQ_LCD200_WriteData_8bit(0x19);     // VCOM = 1.35V

	NEUQ_LCD200_WriteCommand(0xC0);
	NEUQ_LCD200_WriteData_8bit(0x2C);

	NEUQ_LCD200_WriteCommand(0xC2);       // VDV 和 VRH 来源设置
	NEUQ_LCD200_WriteData_8bit(0x01);     // VDV 和 VRH 由用户自由配置

	NEUQ_LCD200_WriteCommand(0xC3);			// VRH电压 设置指令  
	NEUQ_LCD200_WriteData_8bit(0x12);     // VRH电压 = 4.6+( vcom+vcom offset+vdv)
				
	NEUQ_LCD200_WriteCommand(0xC4);		   // VDV电压 设置指令	
	NEUQ_LCD200_WriteData_8bit(0x20);     // VDV电压 = 0v

	NEUQ_LCD200_WriteCommand(0xC6); 		// 正常模式的帧率控制指令
	NEUQ_LCD200_WriteData_8bit(0x0F);   	// 设置屏幕控制器的刷新帧率为60帧    

	NEUQ_LCD200_WriteCommand(0xD0);			// 电源控制指令
	NEUQ_LCD200_WriteData_8bit(0xA4);     // 无效数据，固定写入0xA4
	NEUQ_LCD200_WriteData_8bit(0xA1);     // AVDD = 6.8V ，AVDD = -4.8V ，VDS = 2.3V

	NEUQ_LCD200_WriteCommand(0xE0);       // 正极电压伽马值设定
	NEUQ_LCD200_WriteData_8bit(0xD0);
	NEUQ_LCD200_WriteData_8bit(0x04);
	NEUQ_LCD200_WriteData_8bit(0x0D);
	NEUQ_LCD200_WriteData_8bit(0x11);
	NEUQ_LCD200_WriteData_8bit(0x13);
	NEUQ_LCD200_WriteData_8bit(0x2B);
	NEUQ_LCD200_WriteData_8bit(0x3F);
	NEUQ_LCD200_WriteData_8bit(0x54);
	NEUQ_LCD200_WriteData_8bit(0x4C);
	NEUQ_LCD200_WriteData_8bit(0x18);
	NEUQ_LCD200_WriteData_8bit(0x0D);
	NEUQ_LCD200_WriteData_8bit(0x0B);
	NEUQ_LCD200_WriteData_8bit(0x1F);
	NEUQ_LCD200_WriteData_8bit(0x23);

	NEUQ_LCD200_WriteCommand(0xE1);      // 负极电压伽马值设定
	NEUQ_LCD200_WriteData_8bit(0xD0);
	NEUQ_LCD200_WriteData_8bit(0x04);
	NEUQ_LCD200_WriteData_8bit(0x0C);
	NEUQ_LCD200_WriteData_8bit(0x11);
	NEUQ_LCD200_WriteData_8bit(0x13);
	NEUQ_LCD200_WriteData_8bit(0x2C);
	NEUQ_LCD200_WriteData_8bit(0x3F);
	NEUQ_LCD200_WriteData_8bit(0x44);
	NEUQ_LCD200_WriteData_8bit(0x51);
	NEUQ_LCD200_WriteData_8bit(0x2F);
	NEUQ_LCD200_WriteData_8bit(0x1F);
	NEUQ_LCD200_WriteData_8bit(0x1F);
	NEUQ_LCD200_WriteData_8bit(0x20);
	NEUQ_LCD200_WriteData_8bit(0x23);

	NEUQ_LCD200_WriteCommand(0x21);       // 打开反显，因为面板是常黑型，操作需要反过来

 // 退出休眠指令，LCD控制器在刚上电、复位时，会自动进入休眠模式 ，因此操作屏幕之前，需要退出休眠  
	NEUQ_LCD200_WriteCommand(0x11);       // 退出休眠 指令
    HAL_Delay(120);               // 需要等待120ms，让电源电压和时钟电路稳定下来

 // 打开显示指令，LCD控制器在刚上电、复位时，会自动关闭显示 
	NEUQ_LCD200_WriteCommand(0x29);       // 打开显示   	

	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);	// 片选拉高		

	printf("init done\n");

// 以下进行一些驱动的默认设置
	NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);  	      //	设置显示方向
	NEUQ_LCD200_SetBackColor(LCD_WHITE);           // 设置背景色
    NEUQ_LCD200_SetPenColor(LCD_BLACK);               // 设置画笔色  
	NEUQ_LCD200_Clear();                           // 清屏

	printf("settings done\n");

	NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);       // 设置默认字体
	NEUQ_LCD200_ShowNumMode(Fill_Space);	      	// 设置变量显示模式，多余位填充空格还是填充0

	// 全部设置完毕之后，打开背光	
	NEUQ_GPIO_write(LCD200_BL_USE,GPIO_High);  // 引脚输出高电平点亮背光
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_SetAddress
// 函数功能 : 设置LCD200的显存写入区域
// 输入参数 : x1 : 起始X坐标
//            y1 : 起始Y坐标
//            x2 : 结束X坐标
//            y2 : 结束Y坐标
// 输出参数 : 无
// 返 回 值 : 无
//==============================================================================================
void NEUQ_LCD200_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2){
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC
	
	NEUQ_LCD200_WriteCommand(0x2a);			//	列地址设置，即X坐标
	NEUQ_LCD200_WriteData_16bit_in_8bit_mode(x1+LCD200_Config.X_Offset);
	NEUQ_LCD200_WriteData_16bit_in_8bit_mode(x2+LCD200_Config.X_Offset);

	NEUQ_LCD200_WriteCommand(0x2b);			//	行地址设置，即Y坐标
	NEUQ_LCD200_WriteData_16bit_in_8bit_mode(y1+LCD200_Config.Y_Offset);
	NEUQ_LCD200_WriteData_16bit_in_8bit_mode(y2+LCD200_Config.Y_Offset);

	NEUQ_LCD200_WriteCommand(0x2c);			//	开始写入显存，即要显示的颜色数据
	
	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);	// 片选拉高	
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_SetPenColor
// 函数功能 : 设置LCD200的画笔颜色
// 输入参数 : Color : 画笔颜色，使用 0x00RRGGB
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_SetPenColor(LCD_BLACK); // 设置画笔颜色为红色
//==============================================================================================
void NEUQ_LCD200_SetPenColor(uint32_t Color){
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // 转换成 16位 的RGB565颜色
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD200_Config.Pen_Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);  // 将颜色写入全局LCD参数		
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_SetBackColor
// 函数功能 : 设置LCD200的背景颜色
// 输入参数 : Color : 背景颜色，使用 0x00RRGGB
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_SetBackColor(LCD_WHITE); // 设置背景颜色为白色
//==============================================================================================
void NEUQ_LCD200_SetBackColor(uint32_t Color){
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0; //各个颜色通道的值

	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);   // 转换成 16位 的RGB565颜色
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);

	LCD200_Config.Background_Color = (uint16_t)(Red_Value | Green_Value | Blue_Value);	// 将颜色写入全局LCD参数			   	
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_SetDirection
// 函数功能 : 设置LCD200的显示方向
// 输入参数 : dir : 显示方向
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_SetDirection(LCD200_DIR_Cross); // 设置为横屏显示
//==============================================================================================
void NEUQ_LCD200_SetDirection(LCD200_DIR_ENUM dir){
	LCD200_Config.Direction = dir;    // 写入全局LCD参数

	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC
			
   if( dir == LCD200_DIR_Cross )   // 横屏显示
   {
      NEUQ_LCD200_WriteCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
      NEUQ_LCD200_WriteData_8bit(0x70);        // 横屏显示
      LCD200_Config.X_Offset   = 0;             // 设置控制器坐标偏移量
      LCD200_Config.Y_Offset   = 0;   
      LCD200_Config.Width      = LCD200_Height;		// 重新赋值长、宽
      LCD200_Config.Height     = LCD200_Width;		
   }
   else if( dir == LCD200_DIR_Parallel )
   {
      NEUQ_LCD200_WriteCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
      NEUQ_LCD200_WriteData_8bit(0x00);        // 垂直显示
      LCD200_Config.X_Offset   = 0;              // 设置控制器坐标偏移量
      LCD200_Config.Y_Offset   = 0;     
      LCD200_Config.Width      = LCD200_Width;		// 重新赋值长、宽
      LCD200_Config.Height     = LCD200_Height;						
   }
   else if( dir == LCD200_DIR_Cross_Flip )
   {
      NEUQ_LCD200_WriteCommand(0x36);   			 // 显存访问控制 指令，用于设置访问显存的方式
      NEUQ_LCD200_WriteData_8bit(0xA0);         // 横屏显示，并上下翻转，RGB像素格式
      LCD200_Config.X_Offset   = 0;              // 设置控制器坐标偏移量
      LCD200_Config.Y_Offset   = 0;      
      LCD200_Config.Width      = LCD200_Height;		 // 重新赋值长、宽
      LCD200_Config.Height     = LCD200_Width;				
   }
   else if( dir == LCD200_DIR_Parallel_Flip )
   {
      NEUQ_LCD200_WriteCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
      NEUQ_LCD200_WriteData_8bit(0xC0);        // 垂直显示 ，并上下翻转，RGB像素格式
      LCD200_Config.X_Offset   = 0;              // 设置控制器坐标偏移量
      LCD200_Config.Y_Offset   = 0;     
      LCD200_Config.Width      = LCD200_Width;		// 重新赋值长、宽
      LCD200_Config.Height     = LCD200_Height;				
   }   
   NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
   NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);	// 片选拉高			
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_SetAsciiFont
// 函数功能 : 设置LCD200的ASCII字符集字体
// 输入参数 : Asciifonts : 字体指针
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_SetAsciiFont(&ASCII_Font16); // 设置ASCII字符集字体为 1608
//==============================================================================================
void NEUQ_LCD200_SetAsciiFont(pFONT *Asciifonts){
	LCD200_AsciiFonts = Asciifonts;
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_SetTextFont
// 函数功能 : 根据字体宽度，设置LCD200的ASCII字符集字体
// 输入参数 : fonts : 字体指针
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_SetTextFont(&ASCII_Font16); // 设置ASCII字符集字体为 1608
//==============================================================================================
void NEUQ_LCD200_SetTextFont(pFONT *fonts){
	switch(fonts->Width ){
		case 12:	LCD200_AsciiFonts = &ASCII_Font12;	break;	// 设置ASCII字符的字体为 1206
		case 16:	LCD200_AsciiFonts = &ASCII_Font16;	break;	// 设置ASCII字符的字体为 1608
		case 20:	LCD200_AsciiFonts = &ASCII_Font20;	break;	// 设置ASCII字符的字体为 2010	
		case 24:	LCD200_AsciiFonts = &ASCII_Font24;	break;	// 设置ASCII字符的字体为 2412
		case 32:	LCD200_AsciiFonts = &ASCII_Font32;	break;	// 设置ASCII字符的字体为 3216		
		default: break;
	}
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_ShowNumMode
// 函数功能 : 设置LCD200显示变量时的模式，是填充空格还是填充0
// 输入参数 : mode : 显示模式
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_ShowNumMode(Fill_Zero); // 设置变量显示模式为填充0
//==============================================================================================
void NEUQ_LCD200_ShowNumMode(uint8_t mode){
	LCD200_Config.ShowNum_Mode = mode;
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_Clear
// 函数功能 : 清屏
// 输入参数 : 无
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_Clear(); // 清屏
//==============================================================================================
void NEUQ_LCD200_Clear(void){
	NEUQ_LCD200_SetAddress(0,0,LCD200_Config.Width-1,LCD200_Config.Height-1);			//设置坐标	

	NEUQ_Hard_SPI_switch_to_16bit_mode(LCD200_SPI_USE);
	
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC，开始通信
	
	for(int i=0;i<LCD200_Config.Width*LCD200_Config.Height;i++){
		NEUQ_LCD200_WriteData_16bit(LCD200_Config.Background_Color);
	}
	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);;	// 片选拉高	
	NEUQ_Hard_SPI_switch_to_8bit_mode(LCD200_SPI_USE);
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_ClearRect
// 函数功能 : 清除指定区域
// 输入参数 : x : 起始X坐标
//            y : 起始Y坐标
//            width : 要清除的宽度
//            height : 要清除的高度
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_ClearRect(10,10,50,50); // 清除从(10,10)开始，宽50，高50的区域
//==============================================================================================
void NEUQ_LCD200_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height){
	NEUQ_LCD200_SetAddress( x, y, x+width-1, y+height-1);	// 设置坐标	
	NEUQ_Hard_SPI_switch_to_16bit_mode(LCD200_SPI_USE);
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC，开始通信
	for(int i=0;i<width*height;i++){
		NEUQ_LCD200_WriteData_16bit(LCD200_Config.Background_Color);
	}
	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);;	// 片选拉高	
	NEUQ_Hard_SPI_switch_to_8bit_mode(LCD200_SPI_USE);
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DrawPoint
// 函数功能 : 在指定位置画一个点
// 输入参数 : x : X坐标
//            y : Y坐标
//            color : 颜色，使用 0x00RRGGB
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DrawPoint(10,10,LCD_RED); // 在(10,10)位置画一个红色的点
//==============================================================================================
void NEUQ_LCD200_DrawPoint(uint16_t x,uint16_t y,uint32_t color){
	NEUQ_LCD200_SetAddress(x,y,x,y);	//	设置坐标 
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC，开始通信
	NEUQ_LCD200_WriteData_16bit_in_8bit_mode(LCD200_Config.Pen_Color);
	NEUQ_Hard_SPI_wait_idle(LCD200_SPI_USE);//	等待通信完成
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_High);;	// 片选拉高	
} 

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DisplayChar
// 函数功能 : 在指定位置显示一个ASCII字符
// 输入参数 : x : 起始X坐标
//            y : 起始Y坐标
//            c : 要显示的ASCII字符
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DisplayChar(10,10,'A'); // 在(10,10)位置显示字符 'A'
//==============================================================================================
void NEUQ_LCD200_DisplayChar(uint16_t x, uint16_t y,uint8_t c){
	uint16_t i = 0, w = 0;		// 计数变量
   	uint8_t   disChar;		//存储字符的地址

	c = c - 32; 	// 计算ASCII字符的偏移
	
	NEUQ_GPIO_write(LCD200_CS_USE,GPIO_Low);	// 片选拉低，使能IC，开始通信

	for(int index = 0; index < LCD200_AsciiFonts->Sizes; index++)	{
		disChar = LCD200_AsciiFonts->pTable[c*LCD200_AsciiFonts->Sizes + index]; //获取字符的模值
		for(int counter = 0; counter < 8; counter++){ 
			if(disChar & 0x01)	{		
            	LCD200_Buff[i] =  LCD200_Config.Pen_Color;			// 当前模值不为0时，使用画笔色绘点
			}
			else{		
            	LCD200_Buff[i] = LCD200_Config.Background_Color;		//否则使用背景色绘制点
			}
			disChar >>= 1;
			i++;
         	w++;
 			if( w == LCD200_AsciiFonts->Width ){ // 如果写入的数据达到了字符宽度，则退出当前循环
											   // 进入下一字符的写入的绘制
				w = 0;
				break;
			}        
		}	
	}		
	NEUQ_LCD200_SetAddress( x, y, x+LCD200_AsciiFonts->Width-1, y+LCD200_AsciiFonts->Height-1);	   // 设置坐标	
	NEUQ_LCD200_WriteBuff(LCD200_Buff,LCD200_AsciiFonts->Width*LCD200_AsciiFonts->Height);          // 写入显存
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DisplayString
// 函数功能 : 在指定位置显示一个字符串
// 输入参数 : x : 起始X坐标
//            y : 起始Y坐标
//            p : 要显示的字符串
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DisplayString(10,10,"NEUQ"); // 在(10,10)位置显示字符串 "NEUQ"
//==============================================================================================
void NEUQ_LCD200_DisplayString( uint16_t x, uint16_t y, char *p) {  
	while ((x < LCD200_Config.Width) && (*p != '\0')){	//判断显示坐标是否超出显示区域并且字符是否为空字符
		NEUQ_LCD200_DisplayChar( x,y,*p);
		x += LCD200_AsciiFonts->Width; //显示下一个字符
		p++;	//取下一个字符地址
	}
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_Display_int32
// 函数功能 : 在指定位置显示一个32位整数
// 输入参数 : x : 起始X坐标
//            y : 起始Y坐标
//            number : 要显示的32位整数
//            len : 显示的最大长度，超过则截断，未超过则原样显示
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_Display_int32(10,10,-12345,5); // 在(10,10)位置显示整数 -12345
//==============================================================================================
void NEUQ_LCD200_Display_int32(uint16_t x, uint16_t y, int32_t number, uint8_t len) {
    char Number_Buffer[15];        // 最终用于显示的字符串
    char Temp_Buffer[15];          // 用于初步转换整数 -> 字符串

    // 1. 先把数字转换为字符串（不做格式限制）
    sprintf(Temp_Buffer, "%d", number);

    uint8_t str_len = strlen(Temp_Buffer);

    if (str_len > len) {
        // 2. 如果超出指定长度，截断前 len 位（保留开头）
        strncpy(Number_Buffer, Temp_Buffer, len);
        Number_Buffer[len] = '\0';  // 确保字符串结尾
    } else {
        // 3. 如果没超出，就原样复制
        strcpy(Number_Buffer, Temp_Buffer);
    }

    // 4. 显示结果
    NEUQ_LCD200_DisplayString(x, y, Number_Buffer);
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DisplayFloat
// 函数功能 : 在指定位置显示一个浮点数
// 输入参数 : x : 起始X坐标
//            y : 起始Y坐标
//            number : 要显示的浮点数
//            len : 显示的最大长度，超过则截断，未超过则原样显示
//            decimal : 小数点后保留的位数
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DisplayFloat(10,10,-3.14159,6,2); // 在(10,10)位置显示浮点数 -3.14
//==============================================================================================
void NEUQ_LCD200_DisplayFloat(uint16_t x, uint16_t y, float number, uint8_t len, uint8_t decimal) {
    char raw_buf[32];
    char final_buf[32];

    // 1. 初步格式化为小数点后 decimal 位（比如 %.4f）
    snprintf(raw_buf, sizeof(raw_buf), "%.*f", decimal, number);

    // 2. 去除末尾多余的 0（不影响小数点前后的真实值）
    int len_raw = strlen(raw_buf);
    int dot_pos = -1;

    for (int i = 0; i < len_raw; i++) {
        if (raw_buf[i] == '.') {
            dot_pos = i;
            break;
        }
    }

    if (dot_pos != -1) {
        // 从最后一位向前清除多余的 '0'
        int end = len_raw - 1;
        while (end > dot_pos && raw_buf[end] == '0') {
            raw_buf[end--] = '\0';
        }
        // 如果小数点最后也没用了（比如 123.），去掉小数点
        if (raw_buf[end] == '.') {
            raw_buf[end] = '\0';
        }
    }

    // 3. 最多只保留 len 位
    if (strlen(raw_buf) > len) {
        strncpy(final_buf, raw_buf, len);
        final_buf[len] = '\0';
    } else {
        strcpy(final_buf, raw_buf);
    }

    // 4. 显示结果
    NEUQ_LCD200_DisplayString(x, y, final_buf);
}

#define ABS(X)  ((X) > 0 ? (X) : -(X))    

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DrawLine
// 函数功能 : 画线
// 输入参数 : x1 : 起始X坐标
//            y1 : 起始Y坐标
//            x2 : 结束X坐标
//            y2 : 结束Y坐标
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DrawLine(10,10,100,100); // 画一条从(10,10)到(100,100)的线
//==============================================================================================
void NEUQ_LCD200_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;

	deltax = ABS(x2 - x1);        /* The difference between the x's */
	deltay = ABS(y2 - y1);        /* The difference between the y's */
	x = x1;                       /* Start x off at the first pixel */
	y = y1;                       /* Start y off at the first pixel */

	if (x2 >= x1)                 /* The x-values are increasing */
	{
	 xinc1 = 1;
	 xinc2 = 1;
	}
	else                          /* The x-values are decreasing */
	{
	 xinc1 = -1;
	 xinc2 = -1;
	}

	if (y2 >= y1)                 /* The y-values are increasing */
	{
	 yinc1 = 1;
	 yinc2 = 1;
	}
	else                          /* The y-values are decreasing */
	{
	 yinc1 = -1;
	 yinc2 = -1;
	}

	if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	{
	 xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
	 yinc2 = 0;                  /* Don't change the y for every iteration */
	 den = deltax;
	 num = deltax / 2;
	 numadd = deltay;
	 numpixels = deltax;         /* There are more x-values than y-values */
	}
	else                          /* There is at least one y-value for every x-value */
	{
	 xinc2 = 0;                  /* Don't change the x for every iteration */
	 yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
	 den = deltay;
	 num = deltay / 2;
	 numadd = deltax;
	 numpixels = deltay;         /* There are more y-values than x-values */
	}
	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		NEUQ_LCD200_DrawPoint(x,y,LCD200_Config.Pen_Color);             /* Draw the current pixel */
	 num += numadd;              /* Increase the numerator by the top of the fraction */
	 if (num >= den)             /* Check if numerator >= denominator */
	 {
		num -= den;               /* Calculate the new numerator value */
		x += xinc1;               /* Change the x as appropriate */
		y += yinc1;               /* Change the y as appropriate */
	 }
	 x += xinc2;                 /* Change the x as appropriate */
	 y += yinc2;                 /* Change the y as appropriate */
	}  
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DrawCircle
// 函数功能 : 画圆
// 输入参数 : x : 圆心X坐标
//            y : 圆心Y坐标
//            r : 半径
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DrawCircle(100,100,50); // 画一个以(100,100)为圆心，50为半径的圆
//==============================================================================================
void NEUQ_LCD200_DrawCircle(uint16_t x, uint16_t y, uint16_t r){
	int Xadd = -r, Yadd = 0, err = 2-2*r, e2;
	do {   

		NEUQ_LCD200_DrawPoint(x-Xadd,y+Yadd,LCD200_Config.Pen_Color);
		NEUQ_LCD200_DrawPoint(x+Xadd,y+Yadd,LCD200_Config.Pen_Color);
		NEUQ_LCD200_DrawPoint(x+Xadd,y-Yadd,LCD200_Config.Pen_Color);
		NEUQ_LCD200_DrawPoint(x-Xadd,y-Yadd,LCD200_Config.Pen_Color);
		
		e2 = err;
		if (e2 <= Yadd) {
			err += ++Yadd*2+1;
			if (-Xadd == Yadd && e2 <= Xadd) e2 = 0;
		}
		if (e2 > Xadd) err += ++Xadd*2+1;
    }
    while (Xadd <= 0);   
}

//==============================================================================================
// 函数名称 : NEUQ_LCD200_DisplayPicture
// 函数功能 : 显示图片
// 输入参数 : x : 起始X坐标
//            y : 起始Y坐标
//            width : 图片宽度
//            height : 图片高度
//            *rgb565_data : 图片数据，必须是 RGB565 格式
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_LCD200_DisplayPicture(0,0,240,240,pic_data); // 在(0,0)位置显示一张240x240的图片，图片数据存储在 pic_data 数组中
//==============================================================================================
void NEUQ_LCD200_DisplayPicture(uint16_t x, uint16_t y, uint16_t width, uint16_t height,uint16_t *rgb565_data) {
    // 设置屏幕显示区域
    NEUQ_LCD200_SetAddress(x, y, x + width - 1, y + height - 1);

	NEUQ_LCD200_WriteBuff(rgb565_data,height*width);
}
