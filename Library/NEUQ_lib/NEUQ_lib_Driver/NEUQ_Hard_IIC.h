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
* 文件名称          NEUQ_Hard_IIC
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_HARD_IIC_H_
#define _NEUQ_HARD_IIC_H_

#include "stm32f4xx_hal.h"

typedef enum{
    Hard_IIC_1,     /*SCL:PB6 SDA:PB7*/
    Hard_IIC_2,     /*SCL:PB10 SDA:PB11*/
    Hard_IIC_3,     /*SCL:PA8 SDA:PC9*/
}Hard_IIC_DEV_ENUM;

void NEUQ_Hard_IIC_init(Hard_IIC_DEV_ENUM DEV_select,uint32_t Speed);
uint8_t NEUQ_Hard_IIC_read_8bit_reg(Hard_IIC_DEV_ENUM DEV_select,uint8_t dev_addr,uint8_t reg,uint8_t *pData, uint16_t Size);
uint8_t NEUQ_Hard_IIC_write_8bit_reg(Hard_IIC_DEV_ENUM DEV_select,uint8_t dev_addr,uint8_t reg,uint8_t Data);

void I2C_Scan(I2C_HandleTypeDef *hi2c);
#endif 
