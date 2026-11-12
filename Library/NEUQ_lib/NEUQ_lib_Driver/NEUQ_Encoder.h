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
* 文件名称          NEUQ_Encoder
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_ENCODER_H_
#define _NEUQ_ENCODER_H_

#include "stm32f4xx_hal.h"

typedef enum{ // 枚举 定时器编号
    Encoder_TIM2,
    Encoder_TIM3,
    Encoder_TIM4,
    Encoder_TIM5
}Encoder_DEV_ENUM;

typedef enum{
    Encoder_TIM2_PhaseA_PA0,    // T2定时器 A相可选范围
    Encoder_TIM2_PhaseA_PA5,    // PA5被板载Flash占用，请勿同时使用
    Encoder_TIM2_PhaseA_PA15,

    Encoder_TIM3_PhaseA_PA6,    // PA6被板载Flash占用，请勿同时使用
    Encoder_TIM3_PhaseA_PB4,    // PB4被板载LCD片选占用，请勿同时使用
    Encoder_TIM3_PhaseA_PC6,

    Encoder_TIM4_PhaseA_PB6,    // T4定时器 A相可选范围
    Encoder_TIM4_PhaseA_PD12,

    Encoder_TIM5_PhaseA_PA0,    // T5定时器 A相可选范围
}Encoder_PhaseA_ENUM;

typedef enum{
    Encoder_TIM2_PhaseB_PA1,    // T2定时器 B相可选范围
    Encoder_TIM2_PhaseB_PB3,    // PB3被板载LCD SPI3占用，请勿同时使用

    Encoder_TIM3_PhaseB_PA7,    // PA7被板载Flash占用，请勿同时使用
    Encoder_TIM3_PhaseB_PB5,    // PB5被板载LCD SPI3占用，请勿同时使用
    Encoder_TIM3_PhaseB_PC7,

    Encoder_TIM4_PhaseB_PB7,    // T4定时器 B相可选范围
    Encoder_TIM4_PhaseB_PD13,

    Encoder_TIM5_PhaseB_PA1,    // T5定时器 B相可选范围
}Encoder_PhaseB_ENUM;

void NEUQ_Encoder_init(Encoder_DEV_ENUM DEV_select,Encoder_PhaseA_ENUM PhaseA_select,Encoder_PhaseB_ENUM PhaseB_select);
int32_t NEUQ_Encoder_Read(Encoder_DEV_ENUM DEV_select);
void NEUQ_Encoder_Clear(Encoder_DEV_ENUM DEV_select);

#endif 
