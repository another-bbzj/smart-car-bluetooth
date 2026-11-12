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
* 文件名称          NEUQ_Soft_IIC
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_SOFT_IIC_H_
#define _NEUQ_SOFT_IIC_H_

#include "stm32f4xx_hal.h"
#include "NEUQ_GPIO.h"

typedef struct{
    GPIO_Ports_ENUM SCL;
    GPIO_Ports_ENUM SDA;
    uint8_t delay_us;
}Soft_IIC_Handle_Type;


/**************************************软件模拟IIC,兼容SCCB**********************************************************/
/**********************************以下是用于配置摄像头的SCCB驱动*****************************************************/
/**********************************以下是用于配置摄像头的SCCB驱动*****************************************************/
/**********************************以下是用于配置摄像头的SCCB驱动*****************************************************/
				
void NEUQ_SCCB_init(Soft_IIC_Handle_Type *Soft_IIC_Handle);
void 		SCCB_Start(void);							// 启动IIC通信
void 		SCCB_Stop(void);							// IIC停止信号
void 		SCCB_ACK(void);							//	发送响应信号
void 		SCCB_NoACK(void);							// 发送非应答信号
uint8_t 	SCCB_WaitACK(void);						//	等待应答信号
uint8_t	    SCCB_WriteByte(uint8_t IIC_Data); 	// 写字节函数
uint8_t 	SCCB_ReadByte(uint8_t ACK_Mode);		// 读字节函数
		
uint8_t     SCCB_WriteReg (uint8_t addr,uint8_t value);     	// 对指定的寄存器(8位地址)写一字节数据，OV2640用到
uint8_t     SCCB_ReadReg (uint8_t addr);                    	// 对指定的寄存器(8位地址)读一字节数据，OV2640用到
									
uint8_t 	SCCB_WriteReg_16Bit(uint16_t addr,uint8_t value);	// 对指定的寄存器(16位地址)写一字节数据，OV5640用到									
uint8_t 	SCCB_ReadReg_16Bit (uint16_t addr);						// 对指定的寄存器(16位地址)读一字节数据，OV5640用到
uint8_t 	SCCB_WriteBuffer_16Bit(uint16_t addr,uint8_t *pData, uint32_t size);	// 对指定的寄存器(16位地址)批量写数据，OV5640 写入自动对焦固件时用到		
					


#endif 
