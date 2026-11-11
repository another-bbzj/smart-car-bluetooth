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
* 文件名称          NEUQ_CONSTS
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_CONSTS_H_
#define _NEUQ_CONSTS_H_

#include "stm32f4xx_hal.h"

// 字体相关结构定义
typedef struct _pFont
{    
	const uint8_t 		*pTable;  		//	字模数组地址
	uint16_t 			Width; 		 	//	单个字符的字模宽度
	uint16_t 			Height; 			//	单个字符的字模长度
	uint16_t 			Sizes;	 		//	单个字符的字模数据个数
	uint16_t				Table_Rows;		// 该参数只有汉字字模用到，表示二维数组的行大小
} pFONT;


/*------------------------------------ ASCII字体 ---------------------------------------------*/

extern pFONT ASCII_Font32;		// 3216 字体
extern pFONT ASCII_Font24;		// 2412 字体
extern pFONT ASCII_Font20; 	    // 2010 字体
extern pFONT ASCII_Font16; 	    // 1608 字体
extern pFONT ASCII_Font12;   	// 1206 字体

extern const uint16_t NEU_logo[57600];


extern const uint8_t OV5640_AF_Firmware[4077];
extern const uint16_t OV5640_INIT_Config[254][2];
extern const uint8_t OV5640_Effect_Config[4][4];
extern const uint8_t OV5640_Contrast_Config[7][2];
extern const uint8_t OV5640_Brightness_Config[9][2];

#endif 
