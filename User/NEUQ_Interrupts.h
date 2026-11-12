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
* 文件名称          NEUQ_Interrupts
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2025-09-27        张瑞        加入外部中断回调
********************************************************************************************************************/
#ifndef _NEUQ_INTERRUPTS_H_
#define _NEUQ_INTERRUPTS_H_
#include "stm32f4xx_hal.h"

void NEUQ_PIT_1_CALLBACK(void);
void NEUQ_PIT_2_CALLBACK(void);

void TS_UART_0_RX_CALLBACK(uint8_t dat);
void TS_UART_1_RX_CALLBACK(uint8_t dat);
void TS_UART_2_RX_CALLBACK(uint8_t dat);
void TS_UART_3_RX_CALLBACK(uint8_t dat);

void NEUQ_EXTI0_Callback(void);
void NEUQ_EXTI1_Callback(void);
void NEUQ_EXTI2_Callback(void);
void NEUQ_EXTI3_Callback(void);
void NEUQ_EXTI4_Callback(void);
void NEUQ_EXTI5_Callback(void);
void NEUQ_EXTI6_Callback(void);
void NEUQ_EXTI7_Callback(void);
void NEUQ_EXTI8_Callback(void);
void NEUQ_EXTI9_Callback(void);
void NEUQ_EXTI10_Callback(void);
void NEUQ_EXTI11_Callback(void);
void NEUQ_EXTI12_Callback(void);
void NEUQ_EXTI13_Callback(void);
void NEUQ_EXTI14_Callback(void);
void NEUQ_EXTI15_Callback(void);

#endif 
