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
* 文件名称          NEUQ_PWM
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#ifndef _NEUQ_PWM_H_
#define _NEUQ_PWM_H_

#include "stm32f4xx_hal.h"

#define PWM_MAX (10000)

typedef enum{       // 枚举PWM引脚
    TIM1_CH1_PA8    =0,TIM1_CH1_PE9    =1,
    TIM1_CH2_PA9    =2,TIM1_CH2_PE11   =3,
    TIM1_CH3_PA10   =4,TIM1_CH3_PE13   =5,
    TIM1_CH4_PA11   =6,TIM1_CH4_PE14   =7,

    TIM2_CH1_PA5    =10,TIM2_CH1_PA15   =11, // PA5 reserved by onboard Flash
    TIM2_CH2_PA1    =12,TIM2_CH2_PB3    =13, // PB3 reserved by onboard LCD
    TIM2_CH3_PA2    =14,TIM2_CH3_PB10   =15,
    TIM2_CH4_PA3    =16,TIM2_CH4_PB11   =17,

    TIM3_CH1_PA6    =20,TIM3_CH1_PC6   =21, // PA6 reserved by onboard Flash
    TIM3_CH2_PA7    =22,TIM3_CH2_PC7   =23, // PA7 reserved by onboard Flash
    TIM3_CH3_PB0    =24,TIM3_CH3_PC8   =25, // PB0 reserved by onboard Flash CS
    TIM3_CH4_PB1    =26,TIM3_CH4_PC9   =27,

    TIM4_CH1_PB6    =30,TIM4_CH1_PD12   =31,
    TIM4_CH2_PB7    =32,TIM4_CH2_PD13   =33,
    TIM4_CH3_PB8    =34,TIM4_CH3_PD14   =35,
    TIM4_CH4_PB9    =36,TIM4_CH4_PC15   =37,

    TIM8_CH1_PC6    =40,TIM8_CH1_PXXX   =41,
    TIM8_CH2_PC7    =42,TIM8_CH2_PXXX   =43,
    TIM8_CH3_PC8    =44,TIM8_CH3_PXXX   =45,
    TIM8_CH4_PC9    =46,TIM8_CH4_PXXX   =47,
}PWM_Ports_ENUM;

/**************************NEUQ_PWM模块接口声明**************************/
void NEUQ_PWM_init(PWM_Ports_ENUM PWM_Port_select,uint32_t frequency,uint32_t duty);
void NEUQ_PWM_set_duty(PWM_Ports_ENUM PWM_Port_select,uint32_t duty);
void NEUQ_PWM_enable_output(PWM_Ports_ENUM PWM_Port_select);
void NEUQ_PWM_diasble_output(PWM_Ports_ENUM PWM_Port_select);
/**************************NEUQ_PWM模块接口声明**************************/

/**************************内部调用**************************/
void PWM_Port_To_GPIO(PWM_Ports_ENUM pwm_port, GPIO_TypeDef **gpio_group, uint16_t *gpio_pin, uint32_t *channel);
/**************************内部调用**************************/


#endif 
