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
* 文件名称          NEUQ_ADC
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05       AzureHalcyon        first version
* 2025-09-19       张瑞                 修复功能异常
********************************************************************************************************************/

#include "NEUQ_ADC.h"
#include "stm32f4xx_hal.h"

// 提供三个静态句柄，避免局部临时对象生命周期问题
static ADC_HandleTypeDef s_hadc1;
static ADC_HandleTypeDef s_hadc2;
static ADC_HandleTypeDef s_hadc3;

static inline uint32_t adc_channel_macro_from_num(uint8_t ch)
{
    switch (ch) {
        case 0 : return ADC_CHANNEL_0;
        case 1 : return ADC_CHANNEL_1;
        case 2 : return ADC_CHANNEL_2;
        case 3 : return ADC_CHANNEL_3;
        case 4 : return ADC_CHANNEL_4;
        case 5 : return ADC_CHANNEL_5;
        case 6 : return ADC_CHANNEL_6;
        case 7 : return ADC_CHANNEL_7;
        case 8 : return ADC_CHANNEL_8;
        case 9 : return ADC_CHANNEL_9;
        case 10: return ADC_CHANNEL_10;
        case 11: return ADC_CHANNEL_11;
        case 12: return ADC_CHANNEL_12;
        case 13: return ADC_CHANNEL_13;
        case 14: return ADC_CHANNEL_14;
        case 15: return ADC_CHANNEL_15;
        default: return 0xFFFFFFFFu; // invalid
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC 转换一次
// 参数说明     adc_ch          选择 ADC 通道 (详见 NEUQ_ADC.h 中枚举 adc_channel_enum 定义)
// 返回参数     uint16_t        转换的 ADC 值
// 使用示例     adc_convert(ADC1_CH0_PA0);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint16_t adc_convert(adc_channel_enum adc_ch)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    ADC_HandleTypeDef* hadc = NULL;
    uint8_t adc_num = adc_ch / 16;    // 0->ADC1, 1->ADC2, 2->ADC3
    uint8_t channel = adc_ch % 16;    // 0..15
    uint32_t ch_macro;
    uint32_t val32;

    // 选择ADC句柄（这些句柄请在别处 static 定义并初始化过）
    switch (adc_num) {
        case 0: hadc = &s_hadc1; break;
        case 1: hadc = &s_hadc2; break;
        case 2: hadc = &s_hadc3; break;
        default: return 0;
    }

    // 将通道号映射为 HAL 宏
    ch_macro = adc_channel_macro_from_num(channel);
    if (ch_macro == 0xFFFFFFFFu) {
        return 0;
    }

    // 配置 ADC 通道
    sConfig.Channel = ch_macro;
    sConfig.Rank = 1;

    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;


    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0;
    }

    // 启动→等待→读值→停止（带错误检查）
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) { // 10ms 超时可按需调整
        HAL_ADC_Stop(hadc);
        return 0;
    }

    val32 = HAL_ADC_GetValue(hadc);  // 返回 uint32_t
    HAL_ADC_Stop(hadc);

    return (uint16_t)val32;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC 均值滤波转换
// 参数说明     adc_ch          选择 ADC 通道 (详见 NEUQ_ADC.h 中枚举 adc_channel_enum 定义)
// 参数说明     count           均值滤波次数
// 返回参数     uint16_t        转换的 ADC 值
// 使用示例     adc_mean_filter_convert(ADC1_CH0_PA0, 5);                      // 采集5次 然后返回平均值
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint16_t adc_mean_filter_convert(adc_channel_enum adc_ch, uint8_t count)
{
    uint32_t sum = 0;
    
    if(count == 0)
        return 0;

    for(uint8_t i = 0; i < count; i++)
    {
        sum += adc_convert(adc_ch);
    }

    sum = sum / count;
    return (uint16_t)sum;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC 初始化
// 参数说明     adc_ch          选择 ADC 通道 (详见 NEUQ_ADC.h 中枚举 adc_channel_enum 定义)
// 参数说明     resolution      选择通道分辨率(如果同一个 ADC 模块初始化时设置了不同的分辨率 则最后一个初始化的分辨率生效)
// 返回参数     void
// 使用示例     adc_init(ADC1_CH0_PA0, ADC_8BIT);                              // 初始化 PA0 为 ADC1 的 channel0 输入功能 分辨率为8位
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void adc_init(adc_channel_enum adc_ch, adc_resolution_enum resolution){
    uint8_t adc_num = adc_ch / 16;    // 获取ADC模块号: 0->ADC1, 1->ADC2, 2->ADC3
    uint8_t channel = adc_ch % 16;    // 获取通道号
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ===== GPIO配置 =====
    switch(adc_ch){
        // ------------------- PA0 -------------------
        case ADC1_CH0_PA0:
        case ADC2_CH0_PA0:
        case ADC3_CH0_PA0:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_0;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA1 -------------------
        case ADC1_CH1_PA1:
        case ADC2_CH1_PA1:
        case ADC3_CH1_PA1:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_1;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA2 -------------------
        case ADC1_CH2_PA2:
        case ADC2_CH2_PA2:
        case ADC3_CH2_PA2:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_2;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA3 -------------------
        case ADC1_CH3_PA3:
        case ADC2_CH3_PA3:
        case ADC3_CH3_PA3:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_3;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA4 -------------------
        case ADC1_CH4_PA4:
        case ADC2_CH4_PA4:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_4;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA5 -------------------
        case ADC1_CH5_PA5:
        case ADC2_CH5_PA5:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_5;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA6 -------------------
        case ADC1_CH6_PA6:
        case ADC2_CH6_PA6:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_6;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PA7 -------------------
        case ADC1_CH7_PA7:
        case ADC2_CH7_PA7:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_7;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        // ------------------- PB0 -------------------
        case ADC1_CH8_PB0:
        case ADC2_CH8_PB0:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_0;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        // ------------------- PB1 -------------------
        case ADC1_CH9_PB1:
        case ADC2_CH9_PB1:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_1;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        // ------------------- PC0 -------------------
        case ADC1_CH10_PC0:
        case ADC2_CH10_PC0:
        case ADC3_CH10_PC0:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_0;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        // ------------------- PC1 -------------------
        case ADC1_CH11_PC1:
        case ADC2_CH11_PC1:
        case ADC3_CH11_PC1:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_1;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        // ------------------- PC2 -------------------
        case ADC1_CH12_PC2:
        case ADC2_CH12_PC2:
        case ADC3_CH12_PC2:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_2;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        // ------------------- PC3 -------------------
        case ADC1_CH13_PC3:
        case ADC2_CH13_PC3:
        case ADC3_CH13_PC3:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_3;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        // ------------------- PC4 -------------------
        case ADC1_CH14_PC4:
        case ADC2_CH14_PC4:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_4;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        // ------------------- PC5 -------------------
        case ADC1_CH15_PC5:
        case ADC2_CH15_PC5:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_5;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        // ------------------- PF3 -------------------
        case ADC3_CH9_PF3:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_3;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
            break;
        // ------------------- PF6 -------------------
        case ADC3_CH4_PF6:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_6;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
            break;
        // ------------------- PF7 -------------------
        case ADC3_CH5_PF7:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_7;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
            break;
        // ------------------- PF8 -------------------
        case ADC3_CH6_PF8:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_8;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
            break;
        // ------------------- PF9 -------------------
        case ADC3_CH7_PF9:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_9;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
            break;
        // ------------------- PF10 -------------------
        case ADC3_CH8_PF10:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            GPIO_InitStruct.Pin = GPIO_PIN_10;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
            break;
        default:
            // 可选的防御式处理
            // printf("adc_init(): invalid adc_ch = %d\r\n", (int)adc_ch);
            return;
    }

    // ===== 选择 hadc 指针并开启时钟 =====
    ADC_HandleTypeDef *hadc = NULL;
    switch (adc_num) {
        case 0: // ADC1
            __HAL_RCC_ADC1_CLK_ENABLE();
            hadc = &s_hadc1;
            hadc->Instance = ADC1;
            break;
        case 1: // ADC2
            __HAL_RCC_ADC2_CLK_ENABLE();
            hadc = &s_hadc2;
            hadc->Instance = ADC2;
            break;
        case 2: // ADC3
            __HAL_RCC_ADC3_CLK_ENABLE();
            hadc = &s_hadc3;
            hadc->Instance = ADC3;
            break;
        default:
            return;
    }

    // ===== 基本初始化参数 =====
    hadc->Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    // 分辨率映射
    switch (resolution) {
        case ADC_8BIT:  hadc->Init.Resolution = ADC_RESOLUTION_8B;  break;
        case ADC_10BIT: hadc->Init.Resolution = ADC_RESOLUTION_10B; break;
        case ADC_12BIT: // fallthrough
        default:          hadc->Init.Resolution = ADC_RESOLUTION_12B; break;
    }
    hadc->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc->Init.ScanConvMode          = DISABLE;
    hadc->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc->Init.ContinuousConvMode    = DISABLE;
    hadc->Init.NbrOfConversion       = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; // 某些系列需要


    if (HAL_ADC_Init(hadc) != HAL_OK) {
        Error_Handler();
    }

    // ===== 配置通道 =====
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = adc_channel_macro_from_num(channel);
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // 视信号源阻抗/采样速度需要可改
    sConfig.Offset       = 0;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}
