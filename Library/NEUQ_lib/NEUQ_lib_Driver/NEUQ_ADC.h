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
* 文件名称          NEUQ_ADC
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05       AzureHalcyon        first version
********************************************************************************************************************/

#ifndef _NEUQ_ADC_H_
#define _NEUQ_ADC_H_

#include "stm32f4xx_hal.h"
#include "main.h"

typedef enum    // 枚举ADC通道
{
    // ADC1可选引脚 PA0-PA7(CH0-CH7), PB0-PB1(CH8-CH9), PC0-PC5(CH10-CH15)
    ADC1_CH0_PA0   = 0*16 + 0,     // ADC1 Channel 0  - PA0
    ADC1_CH1_PA1,                  // ADC1 Channel 1  - PA1
    ADC1_CH2_PA2,                  // ADC1 Channel 2  - PA2
    ADC1_CH3_PA3,                  // ADC1 Channel 3  - PA3
    ADC1_CH4_PA4,                  // ADC1 Channel 4  - PA4
    ADC1_CH5_PA5,                  // ADC1 Channel 5  - PA5，板载Flash占用，请勿同时使用
    ADC1_CH6_PA6,                  // ADC1 Channel 6  - PA6，板载Flash占用，请勿同时使用
    ADC1_CH7_PA7,                  // ADC1 Channel 7  - PA7，板载Flash占用，请勿同时使用
    ADC1_CH8_PB0,                  // ADC1 Channel 8  - PB0，板载Flash片选占用，请勿同时使用
    ADC1_CH9_PB1,                  // ADC1 Channel 9  - PB1
    ADC1_CH10_PC0,                 // ADC1 Channel 10 - PC0
    ADC1_CH11_PC1,                 // ADC1 Channel 11 - PC1
    ADC1_CH12_PC2,                 // ADC1 Channel 12 - PC2
    ADC1_CH13_PC3,                 // ADC1 Channel 13 - PC3
    ADC1_CH14_PC4,                 // ADC1 Channel 14 - PC4
    ADC1_CH15_PC5,                 // ADC1 Channel 15 - PC5

    // ADC2可选引脚 PA0-PA7(CH0-CH7), PB0-PB1(CH8-CH9), PC0-PC5(CH10-CH15)
    ADC2_CH0_PA0   = 1*16 + 0,    // ADC2 Channel 0  - PA0
    ADC2_CH1_PA1,                  // ADC2 Channel 1  - PA1
    ADC2_CH2_PA2,                  // ADC2 Channel 2  - PA2
    ADC2_CH3_PA3,                  // ADC2 Channel 3  - PA3
    ADC2_CH4_PA4,                  // ADC2 Channel 4  - PA4
    ADC2_CH5_PA5,                  // ADC2 Channel 5  - PA5，板载Flash占用，请勿同时使用
    ADC2_CH6_PA6,                  // ADC2 Channel 6  - PA6，板载Flash占用，请勿同时使用
    ADC2_CH7_PA7,                  // ADC2 Channel 7  - PA7，板载Flash占用，请勿同时使用
    ADC2_CH8_PB0,                  // ADC2 Channel 8  - PB0，板载Flash片选占用，请勿同时使用
    ADC2_CH9_PB1,                  // ADC2 Channel 9  - PB1
    ADC2_CH10_PC0,                 // ADC2 Channel 10 - PC0
    ADC2_CH11_PC1,                 // ADC2 Channel 11 - PC1
    ADC2_CH12_PC2,                 // ADC2 Channel 12 - PC2
    ADC2_CH13_PC3,                 // ADC2 Channel 13 - PC3
    ADC2_CH14_PC4,                 // ADC2 Channel 14 - PC4
    ADC2_CH15_PC5,                 // ADC2 Channel 15 - PC5

    // ADC3可选引脚 PA0-PA3(CH0-CH3), PF6-PF10(CH4-CH8), PF3(CH9), PC0-PC3(CH10-CH13)
    ADC3_CH0_PA0   = 2*16 + 0,    // ADC3 Channel 0  - PA0
    ADC3_CH1_PA1,                  // ADC3 Channel 1  - PA1
    ADC3_CH2_PA2,                  // ADC3 Channel 2  - PA2
    ADC3_CH3_PA3,                  // ADC3 Channel 3  - PA3
    ADC3_CH4_PF6,                  // ADC3 Channel 4  - PF6
    ADC3_CH5_PF7,                  // ADC3 Channel 5  - PF7
    ADC3_CH6_PF8,                  // ADC3 Channel 6  - PF8
    ADC3_CH7_PF9,                  // ADC3 Channel 7  - PF9
    ADC3_CH8_PF10,                 // ADC3 Channel 8  - PF10
    ADC3_CH9_PF3,                  // ADC3 Channel 9  - PF3
    ADC3_CH10_PC0,                 // ADC3 Channel 10 - PC0
    ADC3_CH11_PC1,                 // ADC3 Channel 11 - PC1
    ADC3_CH12_PC2,                 // ADC3 Channel 12 - PC2
    ADC3_CH13_PC3,                 // ADC3 Channel 13 - PC3
} adc_channel_enum;

// 此枚举定义不允许用户修改
typedef enum        // 枚举ADC分辨率
{
    ADC_8BIT,       // 8位分辨率
    ADC_10BIT,      // 10位分辨率
    ADC_12BIT,      // 12位分辨率
} adc_resolution_enum;

//====================================================ADC 基础函数====================================================
uint16_t adc_convert             (adc_channel_enum adc_ch);                                    // ADC转换一次
uint16_t adc_mean_filter_convert (adc_channel_enum adc_ch, uint8_t count);                    // ADC均值滤波
void     adc_init               (adc_channel_enum adc_ch, adc_resolution_enum resolution);    // ADC初始化
//====================================================ADC 基础函数====================================================

#endif
