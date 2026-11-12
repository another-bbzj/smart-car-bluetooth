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
* 文件名称          NEUQ_OV5640
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_OV5640_H_
#define _NEUQ_OV5640_H_

#include "stm32f4xx_hal.h"

/****************************************************************************  
   数据引脚                        时钟和同步引脚
   PC6     ------> DCMI_D0        PB7  ------> DCMI_VSYNC
   PC7     ------> DCMI_D1        PA4  ------> DCMI_HSYNC
   PC8     ------> DCMI_D2        PA6  ------> DCMI_PIXCLK
   PE1     ------> DCMI_D3	
   PE4     ------> DCMI_D4	      SCCB 控制引脚
   PB6     ------> DCMI_D5        PC3 ------> SCCB_SCL
   PE5     ------> DCMI_D6        PC4 ------> SCCB_SDA 
   PE6     ------> DCMI_D7

   掉电控制引脚
   PC5   ------> PWDN
	
******************************************************************************/ 

void NEUQ_OV5640_init(void);


#define OV5640_SCCB_SCL_PIN (PC3)
#define OV5640_SCCB_SDA_PIN (PC4)
#define OV5640_PWDN_PIN     (PC5)

#define 	OV5640_PWDN_INIT	NEUQ_GPIO_init(OV5640_PWDN_PIN,GPIO_Mode_Output_PushPull,GPIO_Low)	
#define 	OV5640_PWDN_ON		NEUQ_GPIO_write(OV5640_PWDN_PIN,GPIO_High)	
#define 	OV5640_PWDN_OFF		NEUQ_GPIO_write(OV5640_PWDN_PIN,GPIO_Low)	


// 1. 定义OV5640实际输出的图像大小，可以根据实际的应用或者显示屏进行调整
// 2. 这两个参数不会影响帧率
// 3. 因为配置的OV5640的ISP窗口比例为4:3(1280*960)，用户设置的输出尺寸也应满足这个比例
// 4. 如果需要其它比例，需要修改初始化配置里的参数
#define OV5640_FRAME_WIDTH      (160)
#define OV5640_FRAME_HEIGHT     (120)

// 用于设置输出的格式，被 OV5640_Set_Pixformat() 引用
#define 	Pixformat_RGB565   0
#define 	Pixformat_JPEG     1
#define 	Pixformat_GRAY		 2

#define  OV5640_AF_Focusing     	2           // 正在处于自动对焦中
#define  OV5640_AF_End				1				// 自动对焦结束
#define  OV5640_Success   			0           // 通讯成功标志
#define  OV5640_Error     			-1          // 通讯错误

#define  OV5640_Enable    1
#define  OV5640_Disable   0

// OV5640的特效模式，被 OV5640_Set_Effect() 引用
#define  OV5640_Effect_Normal       0  // 正常模式
#define  OV5640_Effect_Negative     1  // 负片模式，也就是颜色全部取反
#define  OV5640_Effect_BW           2  // 黑白模式
#define  OV5640_Effect_Solarize  	3  // 正负片叠加模式

// 1. 定义要显示的画面大小，数值一定要能被4整除！！
// 2. RGB565格式下，最终会由DCMI将OV5640输出的4:3图像裁剪为适应屏幕的比例
// 3. JPG模式下，数值一定要能被8整除！！
#define 	Display_Width          160   	// 图像长度 
#define 	Display_Height         120   // 图像宽度

#define 	Display_BufferSize     Display_Width * Display_Height*2 /4   // DMA传输数据大小（32位宽）

/*------------------------------------------------------------ 常用寄存器 ------------------------------------------------*/

#define 	OV5640_ChipID_H          	0x300A  	// 芯片ID寄存器 高字节
#define 	OV5640_ChipID_L          	0x300B  	// 芯片ID寄存器 低字节

#define	OV5640_FORMAT_CONTROL		0x4300	// 设置数据接口输出的格式	
#define 	OV5640_FORMAT_CONTROL_MUX  0x501F	// 设置ISP的格式

#define	OV5640_JPEG_MODE_SELECT		0x4713	// JPEG模式选择，有1~6模式，用户可数据手册里的说明
#define	OV5640_JPEG_VFIFO_CTRL00 	0x4600	// 用于设置JPEG模式2是否固定输出宽度
#define	OV5640_JPEG_VFIFO_HSIZE_H	0x4602	// JPEG输出水平尺寸,高字节
#define	OV5640_JPEG_VFIFO_HSIZE_L	0x4603	// JPEG输出水平尺寸,低字节
#define	OV5640_JPEG_VFIFO_VSIZE_H	0x4604	// JPEG输出垂直尺寸,低字节
#define	OV5640_JPEG_VFIFO_VSIZE_L	0x4605	// JPEG输出垂直尺寸,低字节

#define 	OV5640_GroupAccess			0X3212	// 寄存器组访问
#define 	OV5640_TIMING_DVPHO_H		0x3808	// 输出水平尺寸,高字节
#define 	OV5640_TIMING_DVPHO_L		0x3809	// 输出水平尺寸,低字节
#define 	OV5640_TIMING_DVPVO_H		0x380A	// 输出垂直尺寸,高字节
#define 	OV5640_TIMING_DVPVO_L		0x380B   // 输出垂直尺寸,低字节
#define 	OV5640_TIMING_Flip			0x3820	// Bit[2:1]用于设置是否垂直翻转
#define 	OV5640_TIMING_Mirror			0x3821	// Bit[2:1]用于设置是否水平镜像

#define 	OV5640_AF_CMD_MAIN			0x3022	// AF 主命令
#define 	OV5640_AF_CMD_ACK				0x3023	// AF 命令确认
#define 	OV5640_AF_FW_STATUS			0x3029	// 对焦状态寄存器

/*------------------------------------------------------------ 函数声明 ------------------------------------------------*/

int8_t   DCMI_OV5640_Init(void);	// 初始SCCB、DCMI、DMA以及配置OV5640

void     OV5640_DMA_Transmit_Continuous(uint32_t DMA_Buffer,uint32_t DMA_BufferSize);	// 启动DMA传输，连续模式
void     OV5640_DMA_Transmit_Snapshot(uint32_t DMA_Buffer,uint32_t DMA_BufferSize);		//  启动DMA传输，快照模式，传输一帧图像后停止
void     OV5640_DCMI_Suspend(void);		// 挂起DCMI，停止捕获数据
void     OV5640_DCMI_Resume(void);		// 恢复DCMI，开始捕获数据
void     OV5640_DCMI_Stop(void);			// 禁止DCMI的DMA请求，停止DCMI捕获，禁止DCMI外设
int8_t 	OV5640_DCMI_Crop(uint16_t Displey_XSize,uint16_t Displey_YSize,uint16_t Sensor_XSize,uint16_t Sensor_YSize ); // 裁剪画面

void     OV5640_Reset(void);				//	执行软件复位
uint16_t OV5640_ReadID(void);				// 读取器件ID
void		OV5640_Config(void);				// 配置OV5640各项参数
	
void		OV5640_Set_Pixformat(uint8_t pixformat);					// 设置图像输出格式	
void 		OV5640_Set_JPEG_QuantizationScale(uint8_t scale);		// 设置JPEG格式的压缩等级,取值 0x01~0x3F
int8_t 	OV5640_Set_Framesize(uint16_t width,uint16_t height);	// 设置实际输出的图像大小
int8_t 	OV5640_Set_Horizontal_Mirror( int8_t ConfigState );	// 用于设置输出的图像是否进行水平镜像
int8_t 	OV5640_Set_Vertical_Flip( int8_t ConfigState );			//	用于设置输出的图像是否进行垂直翻转 
void 		OV5640_Set_Brightness(int8_t Brightness);					// 设置亮度
void		OV5640_Set_Contrast(int8_t Contrast);						// 设置对比度
void 		OV5640_Set_Effect(uint8_t effect_Mode);					// 用于设置特效，正常、负片等模式

int8_t 	OV5640_AF_Download_Firmware(void);		//	将自动对焦固件写入OV5640
int8_t 	OV5640_AF_QueryStatus(void);				//	对焦状态查询
void 		OV5640_AF_Trigger_Constant(void);		// 自动对焦 ，持续 触发
void 		OV5640_AF_Trigger_Single(void);			// 自动对焦 ，触发 单次 
void 		OV5640_AF_Release(void);					// 释放马达，镜头回到初始（对焦为无穷远处）位置





#endif 
