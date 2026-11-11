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
#include "NEUQ_lib_include.h"

TIM_HandleTypeDef TIM_1_Handle={0};
TIM_HandleTypeDef TIM_2_Handle={0};
TIM_HandleTypeDef TIM_3_Handle={0};
TIM_HandleTypeDef TIM_4_Handle={0};
TIM_HandleTypeDef TIM_8_Handle={0};

//============================================================================
// 函数简介     PWM 初始化
// 参数说明     PWM_Port_select     选择 PWM 引脚
// 参数说明     frequency           设置频率 同个通道只有最后一次设置生效
// 参数说明     duty                设置占空比
// 返回参数     void
// 使用示例：   NEUQ_PWM_init(TIM2_CH1_PA5,1000,5000);//初始化TIM2_CH1_PA5为1kHz，50%占空比
// 备注信息     PWM_DUTY_MAX宏定义在NEUQ_PWM.h  默认为10000
//============================================================================
void NEUQ_PWM_init(PWM_Ports_ENUM PWM_Port_select,uint32_t frequency,uint32_t duty){
    TIM_HandleTypeDef *Handle_use=NULL;
    GPIO_TypeDef *gpio_group=NULL;
    uint16_t gpio_pin=NULL;    
    uint32_t AF_use=0;
    uint32_t Channel_use=0xFFFFFFFF;

    /*选择TIM模块*/
    switch(((uint32_t)PWM_Port_select)/10){
        case 0:{
            __HAL_RCC_TIM1_CLK_ENABLE();
            TIM_1_Handle.Instance=TIM1;
            Handle_use=&TIM_1_Handle;
            AF_use=GPIO_AF1_TIM1;
            break;
        }
        case 1:{
            __HAL_RCC_TIM2_CLK_ENABLE();
            TIM_2_Handle.Instance=TIM2;
            Handle_use=&TIM_2_Handle;
            AF_use=GPIO_AF1_TIM2;
            break;
        }
        case 2:{
            __HAL_RCC_TIM3_CLK_ENABLE();
            TIM_3_Handle.Instance=TIM3;
            Handle_use=&TIM_3_Handle;
            AF_use=GPIO_AF2_TIM3;
            break;
        }
        case 3:{
            __HAL_RCC_TIM4_CLK_ENABLE();
            TIM_4_Handle.Instance=TIM4;
            Handle_use=&TIM_4_Handle;
            AF_use=GPIO_AF2_TIM4;
            break;
        }
        case 4:{
            __HAL_RCC_TIM8_CLK_ENABLE();
            TIM_8_Handle.Instance=TIM8;
            Handle_use=&TIM_8_Handle;
            AF_use=GPIO_AF3_TIM8;
            break;
        }
    }

    PWM_Port_To_GPIO(PWM_Port_select,&gpio_group,&gpio_pin,&Channel_use);//对PWM通道进行映射
    
    /*配置GPIO*/
    //启动GPIO时钟
    if (gpio_group == GPIOA)       __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (gpio_group == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (gpio_group == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (gpio_group == GPIOD)  __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (gpio_group == GPIOE)  __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_initure={0};
    GPIO_initure.Pin = gpio_pin;
    GPIO_initure.Mode = GPIO_MODE_AF_PP;
    GPIO_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_initure.Alternate=AF_use;
    HAL_GPIO_Init(gpio_group, &GPIO_initure);

    /*配置定时器设置*/

    uint32_t timer_clock = 84000000UL;  // 统一定时器时钟频率
    uint32_t prescaler = (timer_clock / (frequency * PWM_MAX));
    if (prescaler > 0xFFFF) prescaler = 0xFFFF;  // 限制最大值

    if(Handle_use->Instance==TIM1 || Handle_use->Instance==TIM8){//TIM1和8是挂载在APB2高速总线上的，其他TIM在APB1上
        prescaler=prescaler*2-1;//将TIM1和8时钟二分频，这样所有TIM的时钟速率一致
    }
    else{
        prescaler=prescaler-1;
    }    

    Handle_use->Init.Prescaler = (uint16_t)prescaler;//理论的最小频率约为0.128hz，最大值约为8.4khz，超过此范围会被限幅
    Handle_use->Init.Period=PWM_MAX-1;//满占空比为10000，修改为1000后可以将上述频率提高十倍
    Handle_use->Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数
    Handle_use->Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;//自动装载
    HAL_TIM_PWM_Init(Handle_use);

    if(duty>PWM_MAX){duty=PWM_MAX;}

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = duty;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式
    HAL_TIM_PWM_ConfigChannel(Handle_use, &sConfigOC, Channel_use);
    HAL_TIM_PWM_Start(Handle_use, Channel_use);
}

//============================================================================
// 函数简介     PWM占空比设置
// 参数说明     PWM_Port_select         选择 PWM 引脚
// 参数说明     duty                    设置占空比
// 返回参数     void
// 使用示例：NEUQ_PWM_set_duty(TIM2_CH1_PA5,2500);//设置TIM2_CH1_PA5为25%占空比
// 备注信息     PWM_DUTY_MAX宏定义在NEUQ_PWM.h  默认为10000
//============================================================================
void NEUQ_PWM_set_duty(PWM_Ports_ENUM PWM_Port_select,uint32_t duty){
    TIM_HandleTypeDef *Handle_use=NULL;
    GPIO_TypeDef *gpio_group=NULL;
    uint16_t gpio_pin=NULL;    
    uint32_t Channel_use=0xFFFFFFFF;

    /*选择TIM模块*/
    switch(((uint32_t)PWM_Port_select)/10){
        case 0:{
            Handle_use=&TIM_1_Handle;
            break;
        }
        case 1:{
            Handle_use=&TIM_2_Handle;
            break;
        }
        case 2:{
            Handle_use=&TIM_3_Handle;
            break;
        }
        case 3:{
            Handle_use=&TIM_4_Handle;
            break;
        }
        case 4:{
            Handle_use=&TIM_8_Handle;
            break;
        }
    }

    PWM_Port_To_GPIO(PWM_Port_select,&gpio_group,&gpio_pin,&Channel_use);//对PWM通道进行映射
    if(duty>PWM_MAX){duty=PWM_MAX;}
    __HAL_TIM_SET_COMPARE(Handle_use, Channel_use, duty);
}

//============================================================================
// 函数简介     使能PWM输出
// 参数说明     PWM_Port_select             选择 PWM 引脚
// 返回参数     void
// 使用示例     NEUQ_PWM_enable_output(TIM2_CH1_PA5);//
// 注意事项     无
//============================================================================
void NEUQ_PWM_enable_output(PWM_Ports_ENUM PWM_Port_select){
    TIM_HandleTypeDef *Handle_use=NULL;
    GPIO_TypeDef *gpio_group=NULL;
    uint16_t gpio_pin=NULL;    
    uint32_t Channel_use=0xFFFFFFFF;

    /*选择TIM模块*/
    switch(((uint32_t)PWM_Port_select)/10){
        case 0:{
            Handle_use=&TIM_1_Handle;
            break;
        }
        case 1:{
            Handle_use=&TIM_2_Handle;
            break;
        }
        case 2:{
            Handle_use=&TIM_3_Handle;
            break;
        }
        case 3:{
            Handle_use=&TIM_4_Handle;
            break;
        }
        case 4:{
            Handle_use=&TIM_8_Handle;
            break;
        }
    }

    PWM_Port_To_GPIO(PWM_Port_select,&gpio_group,&gpio_pin,&Channel_use);//对PWM通道进行映射
    HAL_TIM_PWM_Start(Handle_use, Channel_use);
}

//============================================================================
//  函数简介      关闭通道的PWM输出
//  参数说明       PWM_Port_select             选择 PWM 引脚
//  使用示例       NEUQ_PWM_diasble_output(TIM2_CH1_PA5);
//  返回参数      void
//  注意事项：无
//============================================================================
void NEUQ_PWM_diasble_output(PWM_Ports_ENUM PWM_Port_select){
    TIM_HandleTypeDef *Handle_use=NULL;
    GPIO_TypeDef *gpio_group=NULL;
    uint16_t gpio_pin=NULL;    
    uint32_t Channel_use=0xFFFFFFFF;

    /*选择TIM模块*/
    switch(((uint32_t)PWM_Port_select)/10){
        case 0:{
            Handle_use=&TIM_1_Handle;
            break;
        }
        case 1:{
            Handle_use=&TIM_2_Handle;
            break;
        }
        case 2:{
            Handle_use=&TIM_3_Handle;
            break;
        }
        case 3:{
            Handle_use=&TIM_4_Handle;
            break;
        }
        case 4:{
            Handle_use=&TIM_8_Handle;
            break;
        }
    }

    PWM_Port_To_GPIO(PWM_Port_select,&gpio_group,&gpio_pin,&Channel_use);//对PWM通道进行映射
    HAL_TIM_PWM_Stop(Handle_use, Channel_use);
}

//=============================================================================
// 函数功能：PWM端口映射到GPIO和定时器通道
// 输    入：pwm_port 选择的PWM端口
//           gpio_group 指向GPIO组的指针
//           gpio_pin 指向GPIO引脚的指针
//           channel 指向定时器通道的指针
// 使用示例：见内部调用
// 注意事项：无
//  =============================================================================
void PWM_Port_To_GPIO(PWM_Ports_ENUM pwm_port, GPIO_TypeDef **gpio_group, uint16_t *gpio_pin, uint32_t *channel){
    *gpio_group = NULL;
    *gpio_pin = 0xFFFF;
    *channel = 0xFFFFFFFF;  // 无效值

    switch (pwm_port){
        // === TIM1 ===
        case TIM1_CH1_PA8:     case TIM1_CH1_PE9:
            *gpio_group = (pwm_port == TIM1_CH1_PA8) ? GPIOA : GPIOE;
            *gpio_pin   = (pwm_port == TIM1_CH1_PA8) ? GPIO_PIN_8 : GPIO_PIN_9;
            *channel = TIM_CHANNEL_1;
            break;

        case TIM1_CH2_PA9:     case TIM1_CH2_PE11:
            *gpio_group = (pwm_port == TIM1_CH2_PA9) ? GPIOA : GPIOE;
            *gpio_pin   = (pwm_port == TIM1_CH2_PA9) ? GPIO_PIN_9 : GPIO_PIN_11;
            *channel = TIM_CHANNEL_2;
            break;

        case TIM1_CH3_PA10:    case TIM1_CH3_PE13:
            *gpio_group = (pwm_port == TIM1_CH3_PA10) ? GPIOA : GPIOE;
            *gpio_pin   = (pwm_port == TIM1_CH3_PA10) ? GPIO_PIN_10 : GPIO_PIN_13;
            *channel = TIM_CHANNEL_3;
            break;

        case TIM1_CH4_PA11:    case TIM1_CH4_PE14:
            *gpio_group = (pwm_port == TIM1_CH4_PA11) ? GPIOA : GPIOE;
            *gpio_pin   = (pwm_port == TIM1_CH4_PA11) ? GPIO_PIN_11 : GPIO_PIN_14;
            *channel = TIM_CHANNEL_4;
            break;

        // === TIM2 ===
        case TIM2_CH1_PA5:     case TIM2_CH1_PA15:
            *gpio_group = (pwm_port == TIM2_CH1_PA5) ? GPIOA : GPIOA;
            *gpio_pin   = (pwm_port == TIM2_CH1_PA5) ? GPIO_PIN_5 : GPIO_PIN_15;
            *channel = TIM_CHANNEL_1;
            break;

        case TIM2_CH2_PA1:     case TIM2_CH2_PB3:
            *gpio_group = (pwm_port == TIM2_CH2_PA1) ? GPIOA : GPIOB;
            *gpio_pin   = (pwm_port == TIM2_CH2_PA1) ? GPIO_PIN_1 : GPIO_PIN_3;
            *channel = TIM_CHANNEL_2;
            break;

        case TIM2_CH3_PA2:     case TIM2_CH3_PB10:
            *gpio_group = (pwm_port == TIM2_CH3_PA2) ? GPIOA : GPIOB;
            *gpio_pin   = (pwm_port == TIM2_CH3_PA2) ? GPIO_PIN_2 : GPIO_PIN_10;
            *channel = TIM_CHANNEL_3;
            break;

        case TIM2_CH4_PA3:     case TIM2_CH4_PB11:
            *gpio_group = (pwm_port == TIM2_CH4_PA3) ? GPIOA : GPIOB;
            *gpio_pin   = (pwm_port == TIM2_CH4_PA3) ? GPIO_PIN_3 : GPIO_PIN_11;
            *channel = TIM_CHANNEL_4;
            break;

        // === TIM3 ===
        case TIM3_CH1_PA6:     case TIM3_CH1_PC6:
            *gpio_group = (pwm_port == TIM3_CH1_PA6) ? GPIOA : GPIOC;
            *gpio_pin   = (pwm_port == TIM3_CH1_PA6) ? GPIO_PIN_6 : GPIO_PIN_6;
            *channel = TIM_CHANNEL_1;
            break;

        case TIM3_CH2_PA7:     case TIM3_CH2_PC7:
            *gpio_group = (pwm_port == TIM3_CH2_PA7) ? GPIOA : GPIOC;
            *gpio_pin   = (pwm_port == TIM3_CH2_PA7) ? GPIO_PIN_7 : GPIO_PIN_7;
            *channel = TIM_CHANNEL_2;
            break;

        case TIM3_CH3_PB0:     case TIM3_CH3_PC8:
            *gpio_group = (pwm_port == TIM3_CH3_PB0) ? GPIOB : GPIOC;
            *gpio_pin   = (pwm_port == TIM3_CH3_PB0) ? GPIO_PIN_0 : GPIO_PIN_8;
            *channel = TIM_CHANNEL_3;
            break;

        case TIM3_CH4_PB1:     case TIM3_CH4_PC9:
            *gpio_group = (pwm_port == TIM3_CH4_PB1) ? GPIOB : GPIOC;
            *gpio_pin   = (pwm_port == TIM3_CH4_PB1) ? GPIO_PIN_1 : GPIO_PIN_9;
            *channel = TIM_CHANNEL_4;
            break;

        // === TIM4 ===
        case TIM4_CH1_PB6:     case TIM4_CH1_PD12:
            *gpio_group = (pwm_port == TIM4_CH1_PB6) ? GPIOB : GPIOD;
            *gpio_pin   = (pwm_port == TIM4_CH1_PB6) ? GPIO_PIN_6 : GPIO_PIN_12;
            *channel = TIM_CHANNEL_1;
            break;

        case TIM4_CH2_PB7:     case TIM4_CH2_PD13:
            *gpio_group = (pwm_port == TIM4_CH2_PB7) ? GPIOB : GPIOD;
            *gpio_pin   = (pwm_port == TIM4_CH2_PB7) ? GPIO_PIN_7 : GPIO_PIN_13;
            *channel = TIM_CHANNEL_2;
            break;

        case TIM4_CH3_PB8:     case TIM4_CH3_PD14:
            *gpio_group = (pwm_port == TIM4_CH3_PB8) ? GPIOB : GPIOD;
            *gpio_pin   = (pwm_port == TIM4_CH3_PB8) ? GPIO_PIN_8 : GPIO_PIN_14;
            *channel = TIM_CHANNEL_3;
            break;

        case TIM4_CH4_PB9:     case TIM4_CH4_PC15:
            *gpio_group = (pwm_port == TIM4_CH4_PB9) ? GPIOB : GPIOC;
            *gpio_pin   = (pwm_port == TIM4_CH4_PB9) ? GPIO_PIN_9 : GPIO_PIN_15;
            *channel = TIM_CHANNEL_4;
            break;

        // === TIM8 ===
        case TIM8_CH1_PC6:
            *gpio_group = GPIOC; *gpio_pin = GPIO_PIN_6; *channel = TIM_CHANNEL_1; break;
        case TIM8_CH2_PC7:
            *gpio_group = GPIOC; *gpio_pin = GPIO_PIN_7; *channel = TIM_CHANNEL_2; break;
        case TIM8_CH3_PC8:
            *gpio_group = GPIOC; *gpio_pin = GPIO_PIN_8; *channel = TIM_CHANNEL_3; break;
        case TIM8_CH4_PC9:
            *gpio_group = GPIOC; *gpio_pin = GPIO_PIN_9; *channel = TIM_CHANNEL_4; break;

        default:
            // 无效映射
            *gpio_group = NULL;
            *gpio_pin   = 0xFFFF;
            *channel    = 0xFFFFFFFF;
            return;
    }

}

