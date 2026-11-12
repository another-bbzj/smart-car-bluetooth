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
* 文件名称          NEUQ_GPIO
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_GPIO_H_
#define _NEUQ_GPIO_H_

#include "stm32f4xx_hal.h"

typedef enum {                      // 枚举端口 此枚举定义不允许用户修改
    // 板载Flash固定占用PA5、PA6、PA7、PB0；板载LCD固定占用PB3、PB4、PB5。
    // 保留这些通用GPIO枚举以兼容现有API，但应用程序不得同时复用这些引脚。
    // Port A
    PA0 = 0 * 16,  PA1,  PA2,  PA3,  PA4,  PA5,  PA6,  PA7,
    PA8,           PA9,  PA10, PA11, PA12, PA13, PA14, PA15,

    // Port B
    PB0 = 1 * 16,  PB1,  PB2,  PB3,  PB4,  PB5,  PB6,  PB7,
    PB8,           PB9,  PB10, PB11, PB12, PB13, PB14, PB15,

    // Port C
    PC0 = 2 * 16,  PC1,  PC2,  PC3,  PC4,  PC5,  PC6,  PC7,
    PC8,           PC9,  PC10, PC11, PC12, PC13, PC14, PC15,

    // Port D
    PD0 = 3 * 16,  PD1,  PD2,  PD3,  PD4,  PD5,  PD6,  PD7,
    PD8,           PD9,  PD10, PD11, PD12, PD13, PD14, PD15,

    // Port E
    PE0 = 4 * 16,  PE1,  PE2,  PE3,  PE4,  PE5,  PE6,  PE7,
    PE8,           PE9,  PE10, PE11, PE12, PE13, PE14, PE15
} GPIO_Ports_ENUM;

typedef enum{                       // 枚举端口模式   此枚举定义不允许用户修改
    GPIO_Mode_Input_Floating,       // 定义管脚浮空输入
    GPIO_Mode_Input_PullUp,         // 定义管脚上拉输入
    GPIO_Mode_Input_PullDown,       // 定义管脚下拉输入

    GPIO_Mode_Output_PushPull,      // 定义管脚推挽输出
    GPIO_Mode_Output_OpenDrain,     // 定义管脚开漏输出
}GPIO_Mode_ENUM;

typedef enum{   // 枚举端口电平    此枚举定义不允许用户修改
    GPIO_Low,   // 定义低电平
    GPIO_High,  // 定义高电平
}GPIO_Level;

void NEUQ_GPIO_init(GPIO_Ports_ENUM Port_select, GPIO_Mode_ENUM Mode, GPIO_Level Level);
void NEUQ_GPIO_write(GPIO_Ports_ENUM Port_select, GPIO_Level Level);
GPIO_Level NEUQ_GPIO_read(GPIO_Ports_ENUM Port_select);
void NEUQ_GPIO_toggle(GPIO_Ports_ENUM Port_select);

#endif 
