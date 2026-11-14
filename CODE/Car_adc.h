#ifndef CAR_ADC_H
#define CAR_ADC_H

#include <stdint.h>

#include "NEUQ_ADC.h"
#include "NEUQ_GPIO.h"

/*
 * ============================================================================
 * 文件名称：Car_adc.h
 * ============================================================================
 * 模块功能：电池电压采样与低电压报警。
 *
 *   硬件参考（Docs/22届校赛主板原理图参考）：
 *     - 主板使用分压网络测量电池电压，图中可见 200k / 20k 分压；
 *     - 分压输出经 RC 滤波后送入 MCU 的 ADC 引脚；
 *     - 具体 ADC 通道请按实际飞线/网络确认后填写 CAR_ADC_CHANNEL。
 *
 *   计算关系：
 *     Vpin   = raw / 4095 * VREF
 *     Vbat   = Vpin * (R_top + R_bottom) / R_bottom
 *
 *   用户需要填写的值
 *   ------------------------------------------------------------------------
 *     CAR_ADC_CHANNEL      实际 ADC 通道（当前先填 PA0，TBD）
 *     CAR_ADC_R_TOP_OHM    上分压电阻（原理图 200k）
 *     CAR_ADC_R_BOTTOM_OHM 下分压电阻（原理图 20k）
 *     CAR_ADC_VREF_MV      ADC 参考电压（默认 3300 mV）
 *     CAR_ADC_LOW_WARN_MV  警告阈值
 *     CAR_ADC_LOW_CUT_MV   严重低压/降速阈值
 *     CAR_ADC_HYSTERESIS_MV 回差，防止阈值附近抖动
 *     CAR_ADC_DEBOUNCE_CNT 连续采样次数确认
 *     CAR_ADC_ALARM_GPIO   可选：报警输出引脚；不定义则只用状态位
 *
 *   调用约定
 *   ------------------------------------------------------------------------
 *     Car_Adc_Init() 初始化 ADC 和可选报警引脚；
 *     Car_Adc_Task() 在主循环每圈调用，内部按 CAR_ADC_PERIOD_MS 限速；
 *     不要在 5 ms PIT 中断里调用，ADC 轮询会占用较长时间。
 * ============================================================================
 */

/* ========================================================================== */
/*                        用户按原理图填写的参数                               */
/* ========================================================================== */

/* ADC 通道：TBD。原理图分压网络具体接到 PA0/PC0 等哪个脚，需按实物确认。 */
#ifndef CAR_ADC_CHANNEL
#define CAR_ADC_CHANNEL             ADC1_CH0_PA0
#endif

/* 分压电阻：原理图标注 200k / 20k，实际以贴片丝印/万用表为准。 */
#ifndef CAR_ADC_R_TOP_OHM
#define CAR_ADC_R_TOP_OHM           200000UL
#endif
#ifndef CAR_ADC_R_BOTTOM_OHM
#define CAR_ADC_R_BOTTOM_OHM        20000UL
#endif

/* ADC 参考电压和满量程。STM32F407 的 VREF+ 通常为 3.3 V。 */
#ifndef CAR_ADC_VREF_MV
#define CAR_ADC_VREF_MV             3300U
#endif
#ifndef CAR_ADC_ADC_MAX
#define CAR_ADC_ADC_MAX             4095U
#endif

/* 采样周期：Car_Adc_Task() 内部按该周期限速，主循环可以每圈调用。 */
#ifndef CAR_ADC_PERIOD_MS
#define CAR_ADC_PERIOD_MS           50U
#endif

/* 每次 Task 先做 N 次均值，再做一阶低通；N 越大越稳、越慢。 */
#ifndef CAR_ADC_FILTER_SAMPLES
#define CAR_ADC_FILTER_SAMPLES      8U
#endif
#ifndef CAR_ADC_FILTER_DIV
#define CAR_ADC_FILTER_DIV          4U
#endif

/* 低电压阈值：按电池类型填写。示例值按 12 V 电池粗填。 */
#ifndef CAR_ADC_LOW_WARN_MV
#define CAR_ADC_LOW_WARN_MV         11000U
#endif
#ifndef CAR_ADC_LOW_CUT_MV
#define CAR_ADC_LOW_CUT_MV          10500U
#endif
#ifndef CAR_ADC_HYSTERESIS_MV
#define CAR_ADC_HYSTERESIS_MV       200U
#endif
#ifndef CAR_ADC_DEBOUNCE_CNT
#define CAR_ADC_DEBOUNCE_CNT        5U
#endif

/*
 * 可选报警输出：定义 CAR_ADC_ALARM_GPIO 后，正常时输出无效电平，
 * 警告/严重低压时输出有效电平。不定义则只维护状态位。
 * 示例：
 *     #define CAR_ADC_ALARM_GPIO        PE15
 *     #define CAR_ADC_ALARM_ACTIVE_LEVEL GPIO_High
 */
#ifndef CAR_ADC_ALARM_ACTIVE_LEVEL
#define CAR_ADC_ALARM_ACTIVE_LEVEL  GPIO_High
#endif

/* 电压状态。 */
typedef enum
{
    CAR_ADC_STATE_NORMAL = 0,   /* 正常 */
    CAR_ADC_STATE_WARN,         /* 低电压警告 */
    CAR_ADC_STATE_CUT           /* 严重低压，建议降速/联锁 */
} CarAdcState;

/**
 * @brief 初始化 ADC 和可选报警引脚，并做第一次采样。
 * @note 内部调用 adc_init(CAR_ADC_CHANNEL, ADC_12BIT)。
 */
void Car_Adc_Init(void);

/**
 * @brief 周期采样电池电压，更新滤波值和报警状态。
 * @return 1 状态发生变化；0 状态未变或未到采样周期。
 * @note 主循环每圈调用即可，内部按 CAR_ADC_PERIOD_MS 限速；
 *       不要在 5 ms 控制中断里调用。
 */
uint8_t Car_Adc_Task(void);

/**
 * @brief 立即读取一次 ADC 原始值（12 位，0~4095）。
 */
uint16_t Car_Adc_ReadRaw(void);

/**
 * @brief 立即读取一次 ADC 引脚电压，单位 mV。
 */
uint16_t Car_Adc_ReadPinMv(void);

/**
 * @brief 立即读取一次电池电压（已按分压比还原），单位 mV。
 */
uint16_t Car_Adc_ReadBatteryMv(void);

/**
 * @brief 最近一次 Task 的原始 ADC 值。
 */
uint16_t Car_Adc_GetRaw(void);

/**
 * @brief 最近一次 Task 的 ADC 引脚电压，单位 mV。
 */
uint16_t Car_Adc_GetPinMv(void);

/**
 * @brief 最近一次 Task 的电池电压，单位 mV（未滤波）。
 */
uint16_t Car_Adc_GetBatteryMv(void);

/**
 * @brief 最近一次 Task 的滤波后电池电压，单位 mV。
 */
uint16_t Car_Adc_GetFilteredMv(void);

/**
 * @brief 当前低电压状态。
 */
CarAdcState Car_Adc_GetState(void);

/**
 * @brief 是否进入警告或严重低压。
 * @return 1 警告或严重低压；0 正常。
 */
uint8_t Car_Adc_IsLow(void);

/**
 * @brief 是否进入严重低压。
 * @return 1 严重低压；0 其他。
 */
uint8_t Car_Adc_IsCut(void);

/**
 * @brief 设置低电压报警阈值（菜单/持久化参数）。
 * @param warn_mv 警告阈值，mV；cut_mv 严重低压阈值，mV。
 * @return 1 成功；0 参数非法（cut > warn 或超出 1000~30000 mV）。
 * @note 只改阈值，不清除当前报警状态；主循环上下文调用。
 */
uint8_t Car_Adc_SetThresholds(uint16_t warn_mv, uint16_t cut_mv);

/**
 * @brief 当前警告阈值，单位 mV。
 */
uint16_t Car_Adc_GetWarnThreshold(void);

/**
 * @brief 当前严重低压阈值，单位 mV。
 */
uint16_t Car_Adc_GetCutThreshold(void);

#endif
