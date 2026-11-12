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
* 2025-09-20        sq          first version
* 2025-09-27        张瑞        完善所有IO的支持
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

/* EXTI handles: one per line */
EXTI_HandleTypeDef hexti0  = {0};
EXTI_HandleTypeDef hexti1  = {0};
EXTI_HandleTypeDef hexti2  = {0};
EXTI_HandleTypeDef hexti3  = {0};
EXTI_HandleTypeDef hexti4  = {0};
EXTI_HandleTypeDef hexti5  = {0};
EXTI_HandleTypeDef hexti6  = {0};
EXTI_HandleTypeDef hexti7  = {0};
EXTI_HandleTypeDef hexti8  = {0};
EXTI_HandleTypeDef hexti9  = {0};
EXTI_HandleTypeDef hexti10 = {0};
EXTI_HandleTypeDef hexti11 = {0};
EXTI_HandleTypeDef hexti12 = {0};
EXTI_HandleTypeDef hexti13 = {0};
EXTI_HandleTypeDef hexti14 = {0};
EXTI_HandleTypeDef hexti15 = {0};

/* 注意：同一 EXTI 线（0~15）只能映射到一个端口引脚（如 PA0/PB0/PC0/PD0/PE0 只能五选一） */

/* ---------------- IRQ Handlers ---------------- */
void EXTI0_IRQHandler(void)  { HAL_EXTI_IRQHandler(&hexti0);  }
void EXTI1_IRQHandler(void)  { HAL_EXTI_IRQHandler(&hexti1);  }
void EXTI2_IRQHandler(void)  { HAL_EXTI_IRQHandler(&hexti2);  }
void EXTI3_IRQHandler(void)  { HAL_EXTI_IRQHandler(&hexti3);  }
void EXTI4_IRQHandler(void)  { HAL_EXTI_IRQHandler(&hexti4);  }

/* 5~9 共用一个向量，需逐条线查/处理 */
void EXTI9_5_IRQHandler(void)
{
    HAL_EXTI_IRQHandler(&hexti5);
    HAL_EXTI_IRQHandler(&hexti6);
    HAL_EXTI_IRQHandler(&hexti7);
    HAL_EXTI_IRQHandler(&hexti8);
    HAL_EXTI_IRQHandler(&hexti9);
}

/* 10~15 共用一个向量，需逐条线查/处理 */
void EXTI15_10_IRQHandler(void)
{
    HAL_EXTI_IRQHandler(&hexti10);
    HAL_EXTI_IRQHandler(&hexti11);
    HAL_EXTI_IRQHandler(&hexti12);
    HAL_EXTI_IRQHandler(&hexti13);
    HAL_EXTI_IRQHandler(&hexti14);
    HAL_EXTI_IRQHandler(&hexti15);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     EXTI 初始化
// 参数说明     GPIO_Pin         选择的引脚 (可选择范围由 NEUQ_GPIO.h 内 GPIO_Ports_ENUM 枚举值确定)
// 参数说明     Edge_Type       中断触发方式
// 返回参数     void
// 使用示例     NEUQ_EXTI_Init(PA0, EXTI_Edge_Up); // PA0初始化为上升沿触发的外部中断
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_EXTI_Init(GPIO_Ports_ENUM GPIO_Pin, EXTI_Edge_TypeDef Edge_Type)
{
    GPIO_InitTypeDef   GPIO_InitStruct = {0};
    EXTI_ConfigTypeDef exti_config     = {0};
    GPIO_TypeDef      *GPIOx           = NULL;

    uint8_t Port_Index = GPIO_Pin / 16U;
    uint8_t Pin_Index  = GPIO_Pin % 16U;

    /* 使能 SYSCFG 时钟（EXTI 复用映射需要） */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* 选择端口并使能时钟，同时准备 EXTI GPIOSel */
    switch (Port_Index)
    {
        case 0: GPIOx = GPIOA; __HAL_RCC_GPIOA_CLK_ENABLE(); exti_config.GPIOSel = EXTI_GPIOA; break;
        case 1: GPIOx = GPIOB; __HAL_RCC_GPIOB_CLK_ENABLE(); exti_config.GPIOSel = EXTI_GPIOB; break;
        case 2: GPIOx = GPIOC; __HAL_RCC_GPIOC_CLK_ENABLE(); exti_config.GPIOSel = EXTI_GPIOC; break;
        case 3: GPIOx = GPIOD; __HAL_RCC_GPIOD_CLK_ENABLE(); exti_config.GPIOSel = EXTI_GPIOD; break;
        case 4: GPIOx = GPIOE; __HAL_RCC_GPIOE_CLK_ENABLE(); exti_config.GPIOSel = EXTI_GPIOE; break;
        default: return; /* 不支持的端口 */
    }

    /* 根据 Edge_Type 选择 GPIO 中断模式 与 EXTI 触发 */
    uint32_t gpio_mode;
    uint32_t exti_trig;
    switch (Edge_Type)
    {
        case EXTI_Edge_Up:
            gpio_mode = GPIO_MODE_IT_RISING;
            exti_trig = EXTI_TRIGGER_RISING;
            break;
        case EXTI_Edge_Down:
            gpio_mode = GPIO_MODE_IT_FALLING;
            exti_trig = EXTI_TRIGGER_FALLING;
            break;
        case EXTI_Edge_Both:
        default:
            gpio_mode = GPIO_MODE_IT_RISING_FALLING;
            exti_trig = EXTI_TRIGGER_RISING_FALLING;
            break;
    }

    /* GPIO 配置为中断引脚（默认上拉；如需下拉/浮空可在此调整） */
    GPIO_InitStruct.Pin   = (uint16_t)(1U << Pin_Index);
    GPIO_InitStruct.Mode  = gpio_mode;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    /* 通用 EXTI 配置（中断模式，按需触发） */
    exti_config.Mode    = EXTI_MODE_INTERRUPT;
    exti_config.Trigger = exti_trig;

    /* 选择对应 hexti 与中断向量 */
    IRQn_Type irqn;
    EXTI_HandleTypeDef *hexti = NULL;
    void (*wrapper)(void) = NULL;

    switch (Pin_Index)
    {
        case 0:  hexti=&hexti0;  exti_config.Line=EXTI_LINE_0;  irqn=EXTI0_IRQn;      wrapper=NEUQ_EXTI0_Callback;  break;
        case 1:  hexti=&hexti1;  exti_config.Line=EXTI_LINE_1;  irqn=EXTI1_IRQn;      wrapper=NEUQ_EXTI1_Callback;  break;
        case 2:  hexti=&hexti2;  exti_config.Line=EXTI_LINE_2;  irqn=EXTI2_IRQn;      wrapper=NEUQ_EXTI2_Callback;  break;
        case 3:  hexti=&hexti3;  exti_config.Line=EXTI_LINE_3;  irqn=EXTI3_IRQn;      wrapper=NEUQ_EXTI3_Callback;  break;
        case 4:  hexti=&hexti4;  exti_config.Line=EXTI_LINE_4;  irqn=EXTI4_IRQn;      wrapper=NEUQ_EXTI4_Callback;  break;

        case 5:  hexti=&hexti5;  exti_config.Line=EXTI_LINE_5;  irqn=EXTI9_5_IRQn;    wrapper=NEUQ_EXTI5_Callback;  break;
        case 6:  hexti=&hexti6;  exti_config.Line=EXTI_LINE_6;  irqn=EXTI9_5_IRQn;    wrapper=NEUQ_EXTI6_Callback;  break;
        case 7:  hexti=&hexti7;  exti_config.Line=EXTI_LINE_7;  irqn=EXTI9_5_IRQn;    wrapper=NEUQ_EXTI7_Callback;  break;
        case 8:  hexti=&hexti8;  exti_config.Line=EXTI_LINE_8;  irqn=EXTI9_5_IRQn;    wrapper=NEUQ_EXTI8_Callback;  break;
        case 9:  hexti=&hexti9;  exti_config.Line=EXTI_LINE_9;  irqn=EXTI9_5_IRQn;    wrapper=NEUQ_EXTI9_Callback;  break;

        case 10: hexti=&hexti10; exti_config.Line=EXTI_LINE_10; irqn=EXTI15_10_IRQn;  wrapper=NEUQ_EXTI10_Callback; break;
        case 11: hexti=&hexti11; exti_config.Line=EXTI_LINE_11; irqn=EXTI15_10_IRQn;  wrapper=NEUQ_EXTI11_Callback; break;
        case 12: hexti=&hexti12; exti_config.Line=EXTI_LINE_12; irqn=EXTI15_10_IRQn;  wrapper=NEUQ_EXTI12_Callback; break;
        case 13: hexti=&hexti13; exti_config.Line=EXTI_LINE_13; irqn=EXTI15_10_IRQn;  wrapper=NEUQ_EXTI13_Callback; break;
        case 14: hexti=&hexti14; exti_config.Line=EXTI_LINE_14; irqn=EXTI15_10_IRQn;  wrapper=NEUQ_EXTI14_Callback; break;
        case 15: hexti=&hexti15; exti_config.Line=EXTI_LINE_15; irqn=EXTI15_10_IRQn;  wrapper=NEUQ_EXTI15_Callback; break;
        default: return;
    }

    /* 绑定 handle、配置线并注册通用回调 */
    HAL_EXTI_GetHandle(hexti, exti_config.Line);
    HAL_EXTI_SetConfigLine(hexti, &exti_config);
    HAL_EXTI_RegisterCallback(hexti, HAL_EXTI_COMMON_CB_ID, wrapper);

    /* NVIC 配置 */
    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);
}
