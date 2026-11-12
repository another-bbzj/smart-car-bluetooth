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
* 文件名称          NEUQ_Interrupts.h
* 功能说明          NEUQ 库中断回调的用户实现声明，是库中断与整车业务之间的分发层。
*
*                   本文件声明三类回调：
*                   · NEUQ_PIT_1/2_CALLBACK —— TIM6/TIM7 周期中断（本项目只用 TIM6 的 5ms 控制拍）
*                   · TS_UART_0~3_RX_CALLBACK —— 四路串口接收中断（蓝牙走 UART_3，视觉走 UART_2）
*                   · NEUQ_EXTI0~15_Callback —— 十六条外部中断线（本项目均未使用，保留占位）
*
*                   这些函数由 NEUQ 库的中断服务程序调用，用户只需在本文件中实现；
*                   不要在其他地方重复定义，也不要手动调用。
*
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2025-09-27        张瑞        加入外部中断回调
* 2026-11-12                            补充文件综述与中文注释
********************************************************************************************************************/
#ifndef NEUQ_INTERRUPTS_H
#define NEUQ_INTERRUPTS_H

#include "stm32f4xx_hal.h"

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    NEUQ_PIT_1_CALLBACK
 * 功能说明    TIM6 周期中断回调，本项目在此进入整车 5ms 控制拍 Car_PIT_Handler。
 * 用法        由 NEUQ_PIT_init_ms(PIT_CH1, 5) 注册后自动调用，用户不要手动调用。
 * 参数说明    无
 * 返回说明    无
 * 示例        // Car_Main 中初始化 PIT_CH1 后，每 5ms 自动进入
 *-------------------------------------------------------------------------------------------------------------------*/
void NEUQ_PIT_1_CALLBACK(void);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    NEUQ_PIT_2_CALLBACK
 * 功能说明    TIM7 周期中断回调占位，本项目未使用。
 * 用法        若后续需要第二路周期任务，可初始化 PIT_CH2 并在此实现。
 * 参数说明    无
 * 返回说明    无
 *-------------------------------------------------------------------------------------------------------------------*/
void NEUQ_PIT_2_CALLBACK(void);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    TS_UART_0_RX_CALLBACK ~ TS_UART_3_RX_CALLBACK
 * 功能说明    四路串口单字节接收回调：
 *             · UART_0/1 在本项目中不参与业务，收到字节直接丢弃；
 *             · UART_2（USART3）固定接香橙派视觉链路；
 *             · UART_3（UART4）固定接 JDY-31 蓝牙链路。
 * 用法        由 NEUQ 库的 HAL_UART_RxCpltCallback 自动分发，用户不要手动调用。
 * 参数说明    dat —— 收到的单字节
 * 返回说明    无
 * 示例        // 蓝牙数据从 UART_3（UART4）进入 TS_UART_3_RX_CALLBACK
 *-------------------------------------------------------------------------------------------------------------------*/
void TS_UART_0_RX_CALLBACK(uint8_t dat);
void TS_UART_1_RX_CALLBACK(uint8_t dat);
void TS_UART_2_RX_CALLBACK(uint8_t dat);
void TS_UART_3_RX_CALLBACK(uint8_t dat);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    NEUQ_EXTI0_Callback ~ NEUQ_EXTI15_Callback
 * 功能说明    十六条外部中断线的用户回调，本项目均未使用，保持为空。
 * 用法        经 NEUQ_EXTI_Init 注册对应引脚后自动调用；未初始化的线不会触发。
 * 参数说明    无
 * 返回说明    无
 * 备注        同一条 EXTI 线只能映射到一个端口引脚（PA0/PB0/PC0…五选一）；
 *             EXTI5~9 共用 EXTI9_5 中断向量，EXTI10~15 共用 EXTI15_10 中断向量。
 *-------------------------------------------------------------------------------------------------------------------*/
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

#endif /* NEUQ_INTERRUPTS_H */
