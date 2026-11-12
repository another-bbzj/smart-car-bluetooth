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
* 2026-09-27                    中断分发给第三题
********************************************************************************************************************/

/*
 * ============================================================================
 * 文件名称：NEUQ_Interrupts.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件是 NEUQ 库中断回调与蓝牙差速车业务代码之间的“分发表”：
 * NEUQ 库只提供统一的回调入口，本文件把具体中断转给对应模块处理。
 *
 *     PIT_CH1 周期中断 -> Car_PIT_Handler()（采样、控制、保护）
 *     蓝牙串口接收中断 -> Car_UART_RX_Handler()（只入 FIFO）
 *     视觉串口接收中断 -> Car_Vision_UART_RX_Handler()（只入 FIFO）
 *     外部中断         -> 本车暂未使用，保留空回调
 *
 * 串口编号由 Car_Config.h 的 CAR_UART_INDEX / CAR_VISION_UART_INDEX 决定：
 *   - 蓝牙使用 UART4（NEUQ 库索引 3）；
 *   - 视觉使用 USART3（NEUQ 库索引 2）；
 *   - 其余串口接收回调留空，避免未配置的串口误入控制逻辑。
 *
 * 函数列表
 * ----------------------------------------------------------------------------
 *   void NEUQ_PIT_1_CALLBACK(void)
 *       —— PIT_CH1 周期中断入口，转交 Car_PIT_Handler()。
 *   void NEUQ_PIT_2_CALLBACK(void)
 *       —— PIT_CH2 保留入口，当前未使用。
 *   void TS_UART_0_RX_CALLBACK(uint8_t dat)
 *       —— 串口 0 接收回调；仅当蓝牙配置为串口 0 时转交蓝牙处理。
 *   void TS_UART_1_RX_CALLBACK(uint8_t dat)
 *       —— 串口 1 接收回调；仅当蓝牙配置为串口 1 时转交蓝牙处理。
 *   void TS_UART_2_RX_CALLBACK(uint8_t dat)
 *       —— 串口 2（USART3）接收回调，固定转交视觉处理。
 *   void TS_UART_3_RX_CALLBACK(uint8_t dat)
 *       —— 串口 3（UART4）接收回调；仅当蓝牙配置为串口 3 时转交蓝牙处理。
 *   void NEUQ_EXTI0_Callback(void) ~ void NEUQ_EXTI15_Callback(void)
 *       —— 外部中断 0~15 保留入口，当前全部为空。
 *
 * 注意事项
 * ----------------------------------------------------------------------------
 *   接收回调运行在中断上下文，只允许把字节写入 FIFO，不得做协议解析、
 *   浮点运算或等待发送；真正的解析和控制都在 PIT 中断中完成。
 * ============================================================================
 */

#include "../Library/NEUQ_lib/NEUQ_lib_include.h"
#include "Car_Hardware.h"
#include "Car_Config.h"

/* ========================================================================== */
/*                              周期中断分发                                   */
/* ========================================================================== */

/*
 * 函数名称：NEUQ_PIT_1_CALLBACK
 * 函数功能：PIT_CH1 周期中断入口，标称每 5 ms 触发一次，
 *           把控制权交给 Car_PIT_Handler()。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库定时器中断自动调用，业务代码无需手动调用
 *     // 等价的直接调用：Car_PIT_Handler();
 * 备注信息：采样编码器、解析蓝牙、执行速度控制、输出 PWM 和发送遥测
 *           都在 Car_PIT_Handler() 中完成。
 */
void NEUQ_PIT_1_CALLBACK(void)
{
    Car_PIT_Handler();    /* 转交底盘控制中断处理 */
}

/*
 * 函数名称：NEUQ_PIT_2_CALLBACK
 * 函数功能：PIT_CH2 周期中断入口，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库定时器中断自动调用；当前为空实现
 * 备注信息：如果后续需要第二路控制节拍，可在此实现。
 */
void NEUQ_PIT_2_CALLBACK(void)
{
}

/* ========================================================================== */
/*                              串口接收分发                                   */
/* ========================================================================== */

/*
 * 函数名称：TS_UART_0_RX_CALLBACK
 * 函数功能：串口 0 接收回调；仅当 CAR_UART_INDEX == 0 时才作为蓝牙数据。
 * 参数说明：dat —— 收到的原始字节，来源：串口接收中断。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库串口接收中断自动调用
 *     // 当前配置 CAR_UART_INDEX == 3，本回调只执行 (void)dat;
 * 备注信息：非蓝牙配置下显式丢弃，避免误入协议解析。
 */
void TS_UART_0_RX_CALLBACK(uint8_t dat)
{
#if CAR_UART_INDEX == 0
    Car_UART_RX_Handler(dat);    /* 转交蓝牙接收 */
#else
    (void)dat;                   /* 未使用，显式丢弃 */
#endif
}

/*
 * 函数名称：TS_UART_1_RX_CALLBACK
 * 函数功能：串口 1 接收回调；仅当 CAR_UART_INDEX == 1 时才作为蓝牙数据。
 * 参数说明：dat —— 收到的原始字节，来源：串口接收中断。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库串口接收中断自动调用
 *     // 当前配置 CAR_UART_INDEX == 3，本回调只执行 (void)dat;
 * 备注信息：非蓝牙配置下显式丢弃，避免误入协议解析。
 */
void TS_UART_1_RX_CALLBACK(uint8_t dat)
{
#if CAR_UART_INDEX == 1
    Car_UART_RX_Handler(dat);    /* 转交蓝牙接收 */
#else
    (void)dat;                   /* 未使用，显式丢弃 */
#endif
}

/*
 * 函数名称：TS_UART_2_RX_CALLBACK
 * 函数功能：串口 2（USART3）接收回调，固定转交视觉处理。
 * 参数说明：dat —— 收到的原始字节，来源：串口接收中断。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库串口接收中断自动调用
 *     Car_Vision_UART_RX_Handler(dat);
 * 备注信息：本板视觉串口固定为 USART3，因此不需要编译期条件判断。
 */
void TS_UART_2_RX_CALLBACK(uint8_t dat)
{
    Car_Vision_UART_RX_Handler(dat);    /* 转交视觉接收 */
}

/*
 * 函数名称：TS_UART_3_RX_CALLBACK
 * 函数功能：串口 3（UART4）接收回调；仅当 CAR_UART_INDEX == 3 时
 *           才作为蓝牙数据。
 * 参数说明：dat —— 收到的原始字节，来源：串口接收中断。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库串口接收中断自动调用
 *     // 当前配置 CAR_UART_INDEX == 3，会执行 Car_UART_RX_Handler(dat);
 * 备注信息：本板蓝牙默认使用 UART4，即 NEUQ 库索引 3。
 */
void TS_UART_3_RX_CALLBACK(uint8_t dat)
{
#if CAR_UART_INDEX == 3
    Car_UART_RX_Handler(dat);    /* 转交蓝牙接收 */
#else
    (void)dat;                   /* 未使用，显式丢弃 */
#endif
}

/* ========================================================================== */
/*                              外部中断保留入口                               */
/* ========================================================================== */

/*
 * 函数名称：NEUQ_EXTI0_Callback
 * 函数功能：外部中断 0 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI0_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI1_Callback
 * 函数功能：外部中断 1 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI1_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI2_Callback
 * 函数功能：外部中断 2 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI2_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI3_Callback
 * 函数功能：外部中断 3 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI3_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI4_Callback
 * 函数功能：外部中断 4 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI4_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI5_Callback
 * 函数功能：外部中断 5 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI5_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI6_Callback
 * 函数功能：外部中断 6 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI6_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI7_Callback
 * 函数功能：外部中断 7 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI7_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI8_Callback
 * 函数功能：外部中断 8 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI8_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI9_Callback
 * 函数功能：外部中断 9 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI9_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI10_Callback
 * 函数功能：外部中断 10 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI10_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI11_Callback
 * 函数功能：外部中断 11 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI11_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI12_Callback
 * 函数功能：外部中断 12 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI12_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI13_Callback
 * 函数功能：外部中断 13 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI13_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI14_Callback
 * 函数功能：外部中断 14 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI14_Callback(void)
{
}

/*
 * 函数名称：NEUQ_EXTI15_Callback
 * 函数功能：外部中断 15 回调，当前保留未用。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库外部中断分发自动调用；当前为空实现
 */
void NEUQ_EXTI15_Callback(void)
{
}
