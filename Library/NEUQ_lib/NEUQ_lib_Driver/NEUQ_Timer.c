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
* 文件名称          NEUQ_Timer
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        zxz        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

TIM_HandleTypeDef TIM_13_Handle={0};

//=============================================================================
// 函数简介     初始化定时器13 用于计时
// 返回参数     void
// 使用示例     NEUQ_Timer_init();
// 备注信息     定时器13时钟频率84MHz
//=============================================================================
void NEUQ_Timer_init(void)
{
    __HAL_RCC_TIM13_CLK_ENABLE();
    TIM_13_Handle.Instance=TIM13;

    TIM_13_Handle.Init.Prescaler = 84-1;  // 84MHz / 84 = 1mhz
    TIM_13_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_13_Handle.Init.Period = 0xFFFF;  // 最大值
    TIM_13_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_13_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&TIM_13_Handle);
    HAL_TIM_Base_Start(&TIM_13_Handle);
}

//=========================================================================
// 函数简介：获取定时器13的计时值 单位微秒
// 返回参数：计时值 单位微秒
// 使用示例：uint32_t time_us = NEUQ_Get_Timer_Value_us();
// 备注信息：定时器13时钟频率84MHz
//=========================================================================
uint32_t NEUQ_Get_Timer_Value_us(void) 
{
    return TIM_13_Handle.Instance->CNT;
}

//=========================================================================
// 函数简介：获取定时器13的计时值 单位毫秒
// 返回参数：计时值 单位毫秒
// 使用示例：uint32_t time_ms = NEUQ_Get_Timer_Value_ms();
// 备注信息：定时器13时钟频率84MHz
//=========================================================================
uint32_t NEUQ_Get_Timer_Value_ms(void) 
{
    return TIM_13_Handle.Instance->CNT / 1000;  // 1ms = 1000us
}

//=========================================================================
// 函数简介：重置定时器13的计时值
// 返回参数：void
// 使用示例：NEUQ_Reset_Timer();
// 备注信息：定时器13时钟频率84MHz
//=========================================================================
void NEUQ_Reset_Timer(void) 
{
    TIM_13_Handle.Instance->CNT = 0;
}
