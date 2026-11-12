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
* 文件名称          NEUQ_Encoder
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
* 2025-09-20        张瑞        改为四倍频，提高精度
********************************************************************************************************************/

#include "NEUQ_lib_include.h"

TIM_HandleTypeDef Encoder_TIM2_Handle={0};
TIM_HandleTypeDef Encoder_TIM3_Handle={0};
TIM_HandleTypeDef Encoder_TIM4_Handle={0};
TIM_HandleTypeDef Encoder_TIM5_Handle={0};

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      正交编码器采集初始化
//  参数说明      DEV_select       选择所使用的定时器
//  参数说明      PhaseA_select         编码器 A 相
//  参数说明      PhaseB_select         编码器 B 相
//  返回参数      void
//  使用示例      NEUQ_Encoder_init(Encoder_TIM2, Encoder_TIM2_PhaseA_PA5, Encoder_TIM2_PhaseB_PA1);// 使用T2定时器   PA5引脚为A相     PA1引脚为B相
//  备注信息      区分正交和方向编码器
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_Encoder_init(Encoder_DEV_ENUM DEV_select,Encoder_PhaseA_ENUM PhaseA_select,Encoder_PhaseB_ENUM PhaseB_select){
    TIM_HandleTypeDef *htim_use=NULL;
    switch(DEV_select){
        case Encoder_TIM2:{
            __HAL_RCC_TIM2_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
            htim_use=&Encoder_TIM2_Handle;
            htim_use->Instance=TIM2;

            GPIO_InitTypeDef GPIO_initure={0};

            switch(PhaseA_select){
                case Encoder_TIM2_PhaseA_PA0:{
                    GPIO_initure.Pin=GPIO_PIN_0;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF1_TIM2;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
                case Encoder_TIM2_PhaseA_PA5:{
                    GPIO_initure.Pin=GPIO_PIN_5;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF1_TIM2;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
                case Encoder_TIM2_PhaseA_PA15:{
                    GPIO_initure.Pin=GPIO_PIN_15;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF1_TIM2;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
            }

            switch(PhaseB_select){
                case Encoder_TIM2_PhaseB_PA1:{
                    GPIO_initure.Pin=GPIO_PIN_1;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF1_TIM2;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
                case Encoder_TIM2_PhaseB_PB3:{
                    GPIO_initure.Pin=GPIO_PIN_3;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF1_TIM2;
                    HAL_GPIO_Init(GPIOB,&GPIO_initure);
                    break;
                }
            }
            break;
        }
        
        case Encoder_TIM3:{
            __HAL_RCC_TIM3_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
            __HAL_RCC_GPIOC_CLK_ENABLE();
            htim_use=&Encoder_TIM3_Handle;
            htim_use->Instance=TIM3;
            GPIO_InitTypeDef GPIO_initure={0};

            switch(PhaseA_select){
                case Encoder_TIM3_PhaseA_PA6:{
                    GPIO_initure.Pin=GPIO_PIN_6;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_PULLUP;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM3;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
                case Encoder_TIM3_PhaseA_PB4:{
                    GPIO_initure.Pin=GPIO_PIN_4;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM3;
                    HAL_GPIO_Init(GPIOB,&GPIO_initure);
                    break;
                }
                case Encoder_TIM3_PhaseA_PC6:{
                    GPIO_initure.Pin=GPIO_PIN_6;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM3;
                    HAL_GPIO_Init(GPIOC,&GPIO_initure);
                    break;
                }
            }

            switch(PhaseB_select){
                case Encoder_TIM3_PhaseB_PA7:{
                    GPIO_initure.Pin=GPIO_PIN_7;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_PULLUP;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM3;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
                case Encoder_TIM3_PhaseB_PB5:{
                    GPIO_initure.Pin=GPIO_PIN_5;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM3;
                    HAL_GPIO_Init(GPIOB,&GPIO_initure);
                    break;
                }
                case Encoder_TIM3_PhaseB_PC7:{
                    GPIO_initure.Pin=GPIO_PIN_7;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM3;
                    HAL_GPIO_Init(GPIOC,&GPIO_initure);
                    break;
                }
            }
            break;
        }
        
        case Encoder_TIM4:{
            __HAL_RCC_TIM4_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
            __HAL_RCC_GPIOD_CLK_ENABLE();
            htim_use=&Encoder_TIM4_Handle;
            htim_use->Instance=TIM4;
            GPIO_InitTypeDef GPIO_initure={0};

            switch(PhaseA_select){
                case Encoder_TIM4_PhaseA_PB6:{
                    GPIO_initure.Pin=GPIO_PIN_6;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM4;
                    HAL_GPIO_Init(GPIOB,&GPIO_initure);
                    break;
                }
                case Encoder_TIM4_PhaseA_PD12:{
                    GPIO_initure.Pin=GPIO_PIN_12;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM4;
                    HAL_GPIO_Init(GPIOD,&GPIO_initure);
                    break;
                }
            }

            switch(PhaseB_select){
                case Encoder_TIM4_PhaseB_PB7:{
                    GPIO_initure.Pin=GPIO_PIN_7;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM4;
                    HAL_GPIO_Init(GPIOB,&GPIO_initure);
                    break;
                }
                case Encoder_TIM4_PhaseB_PD13:{
                    GPIO_initure.Pin=GPIO_PIN_13;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM4;
                    HAL_GPIO_Init(GPIOD,&GPIO_initure);
                    break;
                }
            }
            break;
        }
        
        case Encoder_TIM5:{
            __HAL_RCC_TIM5_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            htim_use=&Encoder_TIM5_Handle;
            htim_use->Instance=TIM5;
            GPIO_InitTypeDef GPIO_initure={0};

            switch(PhaseA_select){
                case Encoder_TIM5_PhaseA_PA0:{
                    GPIO_initure.Pin=GPIO_PIN_0;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM5;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
            }

            switch(PhaseB_select){
                case Encoder_TIM5_PhaseB_PA1:{
                    GPIO_initure.Pin=GPIO_PIN_1;
                    GPIO_initure.Mode=GPIO_MODE_AF_PP;
                    GPIO_initure.Pull=GPIO_NOPULL;
                    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
                    GPIO_initure.Alternate=GPIO_AF2_TIM5;
                    HAL_GPIO_Init(GPIOA,&GPIO_initure);
                    break;
                }
            }
            break;
        }
    }

    htim_use->Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    htim_use->Init.Prescaler=1-1;
    htim_use->Init.CounterMode=TIM_COUNTERMODE_UP;
    htim_use->Init.Period=0xFFFF;
    htim_use->Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;

	TIM_Encoder_InitTypeDef TIM_ENC_init={0};
	TIM_ENC_init.EncoderMode = TIM_ENCODERMODE_TI12;  // 设置为编码器模式
	TIM_ENC_init.IC1Polarity = TIM_ICPOLARITY_RISING; // 输入1极性，通常选择上升沿或下降沿
	TIM_ENC_init.IC1Selection = TIM_ICSELECTION_DIRECTTI; // 输入选择：直接TI（编码器模式）
	TIM_ENC_init.IC2Polarity = TIM_ICPOLARITY_RISING; // 输入2极性
	TIM_ENC_init.IC2Selection = TIM_ICSELECTION_DIRECTTI; // 输入选择：直接TI

    HAL_TIM_Encoder_Init(htim_use, &TIM_ENC_init);
    HAL_TIM_Encoder_Start(htim_use,TIM_CHANNEL_ALL);


    NEUQ_Encoder_Clear(DEV_select);// 清除计数
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      编码器数据采集
//  参数说明      DEV_select            选择所使用的定时器
//  返回参数      void
//  使用示例      int16 speed; speed = NEUQ_Encoder_Read(Encoder_TIM2); // 使用T2定时器
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
int32_t NEUQ_Encoder_Read(Encoder_DEV_ENUM DEV_select){
    TIM_HandleTypeDef *htim_use=NULL;
    switch(DEV_select){
        case Encoder_TIM2:{
            htim_use=&Encoder_TIM2_Handle;
            break;
        }
        
        case Encoder_TIM3:{
            htim_use=&Encoder_TIM3_Handle;
            break;
        }
        
        case Encoder_TIM4:{
            htim_use=&Encoder_TIM4_Handle;
            break;
        }
        
        case Encoder_TIM5:{
            htim_use=&Encoder_TIM5_Handle;
            break;
        }
    }

    int32_t count=(int32_t)((uint16_t)__HAL_TIM_GET_COUNTER(htim_use)-0x7FFF);
    return count;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      编码器计数清除
//  参数说明      DEV_select            选择所使用的定时器
//  返回参数      void
//  使用示例      NEUQ_Encoder_Clear(Encoder_TIM2);// 使用T2定时器
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_Encoder_Clear(Encoder_DEV_ENUM DEV_select){
    TIM_HandleTypeDef *htim_use=NULL;
    switch(DEV_select){
        case Encoder_TIM2:{
            htim_use=&Encoder_TIM2_Handle;
            break;
        }

        case Encoder_TIM3:{
            htim_use=&Encoder_TIM3_Handle;
            break;
        }

        case Encoder_TIM4:{
            htim_use=&Encoder_TIM4_Handle;
            break;
        }

        case Encoder_TIM5:{
            htim_use=&Encoder_TIM5_Handle;
            break;
        }
    }
    __HAL_TIM_SET_COUNTER(htim_use, 0x7FFF);
}
