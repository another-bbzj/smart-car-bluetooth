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
* 文件名称          NEUQ_PIT
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

TIM_HandleTypeDef TIM_6_Handle={0};
TIM_HandleTypeDef TIM_7_Handle={0};

//=======================================================================
// 函数简介：初始化PIT，单位ms
// 参数说明：DEV_select:选择PIT通道,period:周期，单位ms
// 返回参数：无
// 使用示例：NEUQ_PIT_init_ms(PIT_CH1,1000);  // 初始化PIT1，周期1秒
// 备注信息
//=======================================================================
void NEUQ_PIT_init_ms(PIT_Channel_ENUM DEV_select,uint32_t period){//最大32000ms
    TIM_HandleTypeDef *Handle_use=NULL;

    switch(DEV_select){
        case PIT_CH1:{
            __HAL_RCC_TIM6_CLK_ENABLE();
            TIM_6_Handle.Instance=TIM6;
            Handle_use=&TIM_6_Handle;
            HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
            break;
        }
        case PIT_CH2:{
            __HAL_RCC_TIM7_CLK_ENABLE();
            TIM_7_Handle.Instance=TIM7;
            Handle_use=&TIM_7_Handle;
            HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(TIM7_IRQn);
            break;
        }
    }

    Handle_use->Init.Prescaler = 42*1000-1;  // 84MHz / (42*1000) = 2khz
    Handle_use->Init.CounterMode = TIM_COUNTERMODE_UP;
    Handle_use->Init.Period = period*2-1;
    Handle_use->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    Handle_use->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(Handle_use);
    // 注册回调
    HAL_TIM_RegisterCallback(Handle_use, HAL_TIM_PERIOD_ELAPSED_CB_ID, NEUQ_TIM_PeriodElapsedCallback);
    HAL_TIM_Base_Start_IT(Handle_use);  // 开启中断模式
}

//========================================================================
// 函数简介：初始化PIT，单位us
// 参数说明：DEV_select:选择PIT通道,period:周期，单位us
// 返回参数：无
// 使用示例：NEUQ_PIT_init_us(PIT_CH1,1000000);  // 初始化PIT1，周期1秒
// 备注信息
//========================================================================
void NEUQ_PIT_init_us(PIT_Channel_ENUM DEV_select,uint32_t period){//最大32000us
    TIM_HandleTypeDef *Handle_use=NULL;

    switch(DEV_select){
        case PIT_CH1:{
            __HAL_RCC_TIM6_CLK_ENABLE();
            TIM_6_Handle.Instance=TIM6;
            Handle_use=&TIM_6_Handle;
            HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
            break;
        }
        case PIT_CH2:{
            __HAL_RCC_TIM7_CLK_ENABLE();
            TIM_7_Handle.Instance=TIM7;
            Handle_use=&TIM_7_Handle;
            HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(TIM7_IRQn);
            break;
        }
    }

    Handle_use->Init.Prescaler = 42-1;  // 84MHz / (42) = 2mhz
    Handle_use->Init.CounterMode = TIM_COUNTERMODE_UP;
    Handle_use->Init.Period = period*2-1;
    Handle_use->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    Handle_use->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(Handle_use);
    // 注册回调
    HAL_TIM_RegisterCallback(Handle_use, HAL_TIM_PERIOD_ELAPSED_CB_ID, NEUQ_TIM_PeriodElapsedCallback);
    HAL_TIM_Base_Start_IT(Handle_use);  // 开启中断模式
}

//========================================================================
//函数功能：PIT1中断服务函数
//入口参数：无
//返回参数：无
//使用示例：无
//========================================================================
void TIM6_DAC_IRQHandler(void){
    HAL_TIM_IRQHandler(&TIM_6_Handle);  // HAL 会内部调用注册的回调函数
}
//========================================================================
//函数功能：PIT2中断服务函数
//入口参数：无
//返回参数：无
//使用示例：无
//========================================================================
void TIM7_IRQHandler(void){
    HAL_TIM_IRQHandler(&TIM_7_Handle);  // HAL 会内部调用注册的回调函数
}

//========================================================================
//函数功能：定时器中断回调函数
//入口参数：htim:定时器句柄
//返回参数：无
//使用示例：无
//========================================================================
void NEUQ_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM6){
        NEUQ_PIT_1_CALLBACK();
    }
    if (htim->Instance == TIM7){
        NEUQ_PIT_2_CALLBACK();
    }
}
