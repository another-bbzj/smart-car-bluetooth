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
* 文件名称          NEUQ_Hard_SPI
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2023-09-05        张瑞        first version
* 2026-09-05        Codex       SPI1用于板载Flash，SPI3用于板载LCD
********************************************************************************************************************/
#ifndef _NEUQ_HARD_SPI_H_
#define _NEUQ_HARD_SPI_H_

#include "stm32f4xx_hal.h"
#include "NEUQ_GPIO.h"

#define HARD_SPI_USE_HAL    (0)//快速模式，绕开hal库并持续拉低CS，适用于与单一元件通信，快速模式下必须手动操作CS

// SPI1固定连接板载Flash：PA5-SCK、PA6-MISO、PA7-MOSI、PB0-CS。
// SPI3固定连接板载LCD：PB3-SCK、PB5-MOSI、PB4-CS。
// Hard_SPI_2保留枚举兼容性，当前版本未实现。
typedef enum{
    Hard_SPI_1,
    Hard_SPI_2,
    Hard_SPI_3,
}Hard_SPI_DEV_ENUM;

typedef enum{
    Hard_SPI_1_SCK_PA5,
    Hard_SPI_1_SCK_PB3,       // 仅保留旧代码兼容；初始化时不接受此配置
    Hard_SPI_3_SCK_PB3,
}Hard_SPI_SCK_ENUM;

typedef enum{
    Hard_SPI_1_SDA_PA7,
    Hard_SPI_1_SDA_PB5,       // 仅保留旧代码兼容；初始化时不接受此配置
    Hard_SPI_3_SDA_PB5,
}Hard_SPI_SDA_ENUM;

void NEUQ_Hard_SPI_init(Hard_SPI_DEV_ENUM DEV_select,Hard_SPI_SCK_ENUM SCK_select,Hard_SPI_SDA_ENUM SDA_select,GPIO_Ports_ENUM CS_select);
void NEUQ_Hard_SPI_write_byte(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint8_t byte);
void NEUQ_Hard_SPI_write_2byte(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint16_t bytes);
void NEUQ_Hard_SPI_write_8bit_data(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint8_t *data,uint32_t len);
void NEUQ_Hard_SPI_write_16bit_data(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint16_t *data,uint32_t len);
uint8_t NEUQ_Hard_SPI_transfer_byte(Hard_SPI_DEV_ENUM DEV_select,uint8_t tx_data);
void NEUQ_Hard_SPI_transfer(Hard_SPI_DEV_ENUM DEV_select,const uint8_t *tx_data,uint8_t *rx_data,uint32_t len);
void NEUQ_Hard_SPI_switch_to_16bit_mode(Hard_SPI_DEV_ENUM DEV_select);
void NEUQ_Hard_SPI_switch_to_8bit_mode(Hard_SPI_DEV_ENUM DEV_select);
void NEUQ_Hard_SPI_wait_idle(Hard_SPI_DEV_ENUM DEV_select);


#endif
