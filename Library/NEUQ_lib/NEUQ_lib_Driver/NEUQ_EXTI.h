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
* 文件名称          NEUQ_EXTI
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        sq          first version
* 2025-09-27        张瑞        完善所有IO的支持
********************************************************************************************************************/
#ifndef _NEUQ_EXTI_H_
#define _NEUQ_EXTI_H_

#include "NEUQ_lib_include.h"

typedef enum{
    EXTI_Edge_Up,    // 上升沿触发
    EXTI_Edge_Down,  // 下降沿触发
    EXTI_Edge_Both   // 双边沿触发
}EXTI_Edge_TypeDef;

// PA5、PA6、PA7、PB0由板载Flash占用，PB3、PB4、PB5由板载LCD占用，
// 不得在启用对应板载设备时将这些引脚配置为外部中断。
void NEUQ_EXTI_Init(GPIO_Ports_ENUM GPIO_Pin, EXTI_Edge_TypeDef Edge_Type); // 外部中断初始化函数


#endif
