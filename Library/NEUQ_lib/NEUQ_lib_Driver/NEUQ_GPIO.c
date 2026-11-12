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
#include "NEUQ_lib_include.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 初始化
// 参数说明     Port_select         选择的引脚 (可选择范围由 NEUQ_GPIO.h 内 GPIO_Ports_ENUM 枚举值确定)
// 参数说明     Mode        设置为输出模式时
// 参数说明     Level         引脚初始化时设置的电平状态，输出时有效 0：低电平 1：高电平 仅在设置为输出模式时有效
// 返回参数     void
// 使用示例     NEUQ_GPIO_init(PA0, GPIO_Mode_Output_PushPull, GPIO_High); // PA0初始化为GPIO功能、输出模式、输出高电平、推挽输出
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_GPIO_init(GPIO_Ports_ENUM Port_select, GPIO_Mode_ENUM Mode, GPIO_Level Level) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 计算端口号和引脚号
    uint8_t port_index = Port_select / 16;
    uint8_t pin_index  = Port_select % 16;

    // 获取对应的 GPIO_TypeDef*
    GPIO_TypeDef* GPIOx;

    switch (port_index) {
        case 0: GPIOx = GPIOA; __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case 1: GPIOx = GPIOB; __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case 2: GPIOx = GPIOC; __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case 3: GPIOx = GPIOD; __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case 4: GPIOx = GPIOE; __HAL_RCC_GPIOE_CLK_ENABLE(); break;
        default: return; // 错误端口，直接返回
    }

    // 配置 GPIO_Pin
    GPIO_InitStruct.Pin = 1 << pin_index;

    // 配置 Mode
    if (Mode == GPIO_Mode_Input_Floating) {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    } else if (Mode == GPIO_Mode_Input_PullUp) {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    } else if (Mode == GPIO_Mode_Input_PullDown) {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    } else if (Mode == GPIO_Mode_Output_PushPull) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    } else if (Mode == GPIO_Mode_Output_OpenDrain) {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    } else {
        return; // 无效模式
    }

    // 初始化 GPIO
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    // 如果是输出模式，设置初始电平
    if (Mode == GPIO_Mode_Output_PushPull || Mode == GPIO_Mode_Output_OpenDrain) {
        HAL_GPIO_WritePin(GPIOx, 1 << pin_index, (Level == GPIO_High) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 输出设置
// 参数说明     Port_select         选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 参数说明     Level         0：低电平 1：高电平
// 返回参数     void
// 使用示例     NEUQ_GPIO_write(PA0, 1);// PA0 输出高电平
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_GPIO_write(GPIO_Ports_ENUM Port_select, GPIO_Level Level) {
    uint8_t port_index = Port_select / 16;
    uint8_t pin_index  = Port_select % 16;

    GPIO_TypeDef* GPIOx;

    switch (port_index) {
        case 0: GPIOx = GPIOA; break;
        case 1: GPIOx = GPIOB; break;
        case 2: GPIOx = GPIOC; break;
        case 3: GPIOx = GPIOD; break;
        case 4: GPIOx = GPIOE; break;
        default: return;
    }

    HAL_GPIO_WritePin(GPIOx, 1 << pin_index, (Level == GPIO_High) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 电平获取
// 参数说明     Port_select         选择的引脚 (可选择范围由 NEUQ_GPIO.h 内 GPIO_Ports_ENUM 枚举值确定)
// 返回参数     GPIO_Level       引脚当前电平
// 使用示例     uint8_t status = NEUQ_GPIO_read(PA0);// 获取PA0引脚电平
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
GPIO_Level NEUQ_GPIO_read(GPIO_Ports_ENUM Port_select) {
    uint8_t port_index = Port_select / 16;
    uint8_t pin_index  = Port_select % 16;

    GPIO_TypeDef* GPIOx;

    switch (port_index) {
        case 0: GPIOx = GPIOA; break;
        case 1: GPIOx = GPIOB; break;
        case 2: GPIOx = GPIOC; break;
        case 3: GPIOx = GPIOD; break;
        case 4: GPIOx = GPIOE; break;
        default: return GPIO_Low;
    }

    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOx, 1 << pin_index);
    return (state == GPIO_PIN_SET) ? GPIO_High : GPIO_Low;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 翻转电平
// 参数说明     Port_select         选择的引脚 (可选择范围由 NEUQ_GPIO.h 内 GPIO_Ports_ENUM 枚举值确定)
// 返回参数     void
// 使用示例     NEUQ_GPIO_toggle(PA0);// PA0引脚电平翻转
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_GPIO_toggle(GPIO_Ports_ENUM Port_select) {
    uint8_t port_index = Port_select / 16;
    uint8_t pin_index  = Port_select % 16;

    GPIO_TypeDef* GPIOx;

    switch (port_index) {
        case 0: GPIOx = GPIOA; break;
        case 1: GPIOx = GPIOB; break;
        case 2: GPIOx = GPIOC; break;
        case 3: GPIOx = GPIOD; break;
        case 4: GPIOx = GPIOE; break;
        default: return;
    }

    HAL_GPIO_TogglePin(GPIOx, 1 << pin_index);
}
