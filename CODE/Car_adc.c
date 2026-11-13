/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */

/*
 * ============================================================================
 * 文件名称：Car_adc.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 电池电压采样与低电压报警。数据流：
 *
 *     ADC 引脚 -> adc_mean_filter_convert() -> 原始值
 *              -> 引脚电压 -> 分压还原 -> 电池电压
 *              -> 一阶低通 -> 低电压状态机（回差 + 去抖）
 *              -> 状态位 / 可选报警 GPIO
 *
 * 硬件参考
 * ----------------------------------------------------------------------------
 *   Docs/22届校赛主板原理图参考 中可见 200k / 20k 分压和 RC 滤波。
 *   具体 ADC 通道、分压阻值和报警引脚由用户在 Car_adc.h 顶部填写。
 *
 * 计算关系
 * ----------------------------------------------------------------------------
 *   Vpin = raw / 4095 * VREF
 *   Vbat = Vpin * (R_top + R_bottom) / R_bottom
 *
 * 调用上下文
 * ----------------------------------------------------------------------------
 *   Car_Adc_Init() 在主循环启动前调用一次；
 *   Car_Adc_Task() 在主循环中 50~100 ms 调用一次；
 *   adc_convert() 内部轮询转换，耗时较长，不要放进 5 ms PIT 中断。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static uint16_t Car_Adc_ConvertPinMv(uint16_t raw)
 *       —— 原始值换算为 ADC 引脚电压。
 *   static uint16_t Car_Adc_ConvertBatteryMv(uint16_t pin_mv)
 *       —— 引脚电压按分压比还原为电池电压。
 *   static uint16_t Car_Adc_Filter(uint16_t sample)
 *       —— 一阶低通滤波。
 *   static CarAdcState Car_Adc_NextState(uint16_t mv, CarAdcState current)
 *       —— 带回差的低电压状态判断。
 *   static void Car_Adc_ApplyState(CarAdcState next)
 *       —— 连续采样去抖后提交状态。
 *   static void Car_Adc_AlarmWrite(CarAdcState state)
 *       —— 可选报警 GPIO 输出。
 *   void Car_Adc_Init(void)
 *       —— 初始化 ADC/报警引脚并采样一次。
 *   uint8_t Car_Adc_Task(void)
 *       —— 周期采样、滤波和报警状态更新。
 *   uint16_t Car_Adc_ReadRaw(void)
 *       —— 立即读取一次 ADC 原始值。
 *   uint16_t Car_Adc_ReadPinMv(void)
 *       —— 立即读取一次引脚电压。
 *   uint16_t Car_Adc_ReadBatteryMv(void)
 *       —— 立即读取一次电池电压。
 *   uint16_t Car_Adc_GetRaw(void)
 *       —— 最近一次原始值。
 *   uint16_t Car_Adc_GetPinMv(void)
 *       —— 最近一次引脚电压。
 *   uint16_t Car_Adc_GetBatteryMv(void)
 *       —— 最近一次电池电压。
 *   uint16_t Car_Adc_GetFilteredMv(void)
 *       —— 最近一次滤波后电池电压。
 *   CarAdcState Car_Adc_GetState(void)
 *       —— 当前低电压状态。
 *   uint8_t Car_Adc_IsLow(void)
 *       —— 是否警告或严重低压。
 *   uint8_t Car_Adc_IsCut(void)
 *       —— 是否严重低压。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"
#include "Car_adc.h"

/* 参数合法性检查：错误配置在编译期直接暴露。 */
#if CAR_ADC_ADC_MAX == 0
#error "CAR_ADC_ADC_MAX must be non-zero"
#endif
#if CAR_ADC_R_BOTTOM_OHM == 0
#error "CAR_ADC_R_BOTTOM_OHM must be non-zero"
#endif
#if CAR_ADC_FILTER_SAMPLES == 0 || CAR_ADC_FILTER_SAMPLES > 255
#error "CAR_ADC_FILTER_SAMPLES must be in 1..255"
#endif
#if CAR_ADC_FILTER_DIV == 0
#error "CAR_ADC_FILTER_DIV must be non-zero"
#endif
#if CAR_ADC_LOW_CUT_MV > CAR_ADC_LOW_WARN_MV
#error "CAR_ADC_LOW_CUT_MV must be <= CAR_ADC_LOW_WARN_MV"
#endif
#if CAR_ADC_DEBOUNCE_CNT == 0
#error "CAR_ADC_DEBOUNCE_CNT must be non-zero"
#endif
#if CAR_ADC_PERIOD_MS == 0
#error "CAR_ADC_PERIOD_MS must be non-zero"
#endif

/* 采样与报警状态。 */
static uint16_t adc_raw;           /* 最近一次原始值 */
static uint16_t adc_pin_mv;        /* 最近一次引脚电压 */
static uint16_t adc_battery_mv;    /* 最近一次电池电压（未滤波） */
static uint16_t adc_filtered_mv;   /* 最近一次滤波后电池电压 */
static uint8_t adc_filter_ready;   /* 1 表示滤波器已有初值 */

static CarAdcState adc_state;      /* 已提交的报警状态 */
static CarAdcState adc_candidate;  /* 正在去抖的候选状态 */
static uint8_t adc_debounce;       /* 候选状态连续出现次数 */
static uint32_t adc_last_ms;       /* 上一次采样时刻 */
static uint8_t adc_started;        /* 1 表示已经完成首次采样 */

/* 提前声明：Car_Adc_ApplyState() 在定义之前调用。 */
static void Car_Adc_AlarmWrite(CarAdcState state);

/* ========================================================================== */
/*                              内部辅助函数                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_Adc_ConvertPinMv
 * 函数功能：把 12 位 ADC 原始值换算成 ADC 引脚电压。
 * 参数说明：raw —— 原始值 0~4095。
 * 返回参数：引脚电压，单位 mV；超量程时饱和到 65535。
 * 使用示例：
 *     uint16_t mv = Car_Adc_ConvertPinMv(2048);
 * 备注信息：按 CAR_ADC_VREF_MV 和 CAR_ADC_ADC_MAX 计算，带四舍五入。
 */
static uint16_t Car_Adc_ConvertPinMv(uint16_t raw)
{
    uint32_t mv = ((uint32_t)raw * CAR_ADC_VREF_MV +
                   (CAR_ADC_ADC_MAX / 2U)) / CAR_ADC_ADC_MAX;

    return (mv > 65535U) ? 65535U : (uint16_t)mv;
}

/*
 * 函数名称：Car_Adc_ConvertBatteryMv
 * 函数功能：按分压比把引脚电压还原为电池电压。
 * 参数说明：pin_mv —— ADC 引脚电压，单位 mV。
 * 返回参数：电池电压，单位 mV；超量程时饱和到 65535。
 * 使用示例：
 *     uint16_t vbat = Car_Adc_ConvertBatteryMv(1090);
 * 备注信息：Vbat = Vpin * (R_top + R_bottom) / R_bottom。
 */
static uint16_t Car_Adc_ConvertBatteryMv(uint16_t pin_mv)
{
    uint32_t ratio = CAR_ADC_R_TOP_OHM + CAR_ADC_R_BOTTOM_OHM;
    uint32_t mv = ((uint32_t)pin_mv * ratio +
                   (CAR_ADC_R_BOTTOM_OHM / 2U)) / CAR_ADC_R_BOTTOM_OHM;

    return (mv > 65535U) ? 65535U : (uint16_t)mv;
}

/*
 * 函数名称：Car_Adc_Filter
 * 函数功能：对电池电压做一阶低通滤波。
 * 参数说明：sample —— 本次未滤波的电池电压，单位 mV。
 * 返回参数：滤波后的电池电压，单位 mV。
 * 使用示例：
 *     adc_filtered_mv = Car_Adc_Filter(12000);
 * 备注信息：第一次采样直接作为初值；之后按 1/CAR_ADC_FILTER_DIV 逼近。
 */
static uint16_t Car_Adc_Filter(uint16_t sample)
{
    int32_t delta;

    if (!adc_filter_ready)
    {
        adc_filtered_mv = sample;
        adc_filter_ready = 1U;
        return adc_filtered_mv;
    }

    delta = (int32_t)sample - (int32_t)adc_filtered_mv;
    adc_filtered_mv = (uint16_t)((int32_t)adc_filtered_mv +
                                 delta / (int32_t)CAR_ADC_FILTER_DIV);
    return adc_filtered_mv;
}

/*
 * 函数名称：Car_Adc_NextState
 * 函数功能：根据滤波电压和当前状态计算下一状态，带回差。
 * 参数说明：mv      —— 滤波后的电池电压，单位 mV；
 *           current —— 当前已提交状态。
 * 返回参数：下一状态。
 * 使用示例：
 *     next = Car_Adc_NextState(adc_filtered_mv, adc_state);
 * 备注信息：从严重低压恢复时需高于 CUT + HYSTERESIS，避免阈值抖动。
 */
static CarAdcState Car_Adc_NextState(uint16_t mv, CarAdcState current)
{
    if (mv <= CAR_ADC_LOW_CUT_MV)
    {
        return CAR_ADC_STATE_CUT;
    }

    if (current == CAR_ADC_STATE_CUT)
    {
        if (mv < (uint16_t)(CAR_ADC_LOW_CUT_MV + CAR_ADC_HYSTERESIS_MV))
        {
            return CAR_ADC_STATE_CUT;
        }
        if (mv <= CAR_ADC_LOW_WARN_MV)
        {
            return CAR_ADC_STATE_WARN;
        }
        return CAR_ADC_STATE_NORMAL;
    }

    if (mv <= CAR_ADC_LOW_WARN_MV)
    {
        return CAR_ADC_STATE_WARN;
    }

    if (current == CAR_ADC_STATE_WARN)
    {
        if (mv < (uint16_t)(CAR_ADC_LOW_WARN_MV + CAR_ADC_HYSTERESIS_MV))
        {
            return CAR_ADC_STATE_WARN;
        }
        return CAR_ADC_STATE_NORMAL;
    }

    return CAR_ADC_STATE_NORMAL;
}

/*
 * 函数名称：Car_Adc_ApplyState
 * 函数功能：对候选状态做连续采样去抖，满足次数后提交并刷新报警输出。
 * 参数说明：next —— 本次计算出的候选状态。
 * 返回参数：无。
 * 使用示例：
 *     Car_Adc_ApplyState(Car_Adc_NextState(v, s));
 * 备注信息：候选状态变化时重新计数；状态未变时清空计数。
 */
static void Car_Adc_ApplyState(CarAdcState next)
{
    if (next == adc_state)
    {
        adc_candidate = adc_state;
        adc_debounce = 0U;
        return;
    }

    if (next != adc_candidate)
    {
        adc_candidate = next;
        adc_debounce = 1U;
    }
    else if (adc_debounce < 255U)
    {
        adc_debounce++;
    }

    if (adc_debounce >= CAR_ADC_DEBOUNCE_CNT)
    {
        adc_state = next;
        adc_candidate = next;
        adc_debounce = 0U;
        Car_Adc_AlarmWrite(adc_state);
    }
}

/*
 * 函数名称：Car_Adc_AlarmWrite
 * 函数功能：按状态驱动可选报警 GPIO。
 * 参数说明：state —— 已提交的报警状态。
 * 返回参数：无。
 * 使用示例：
 *     Car_Adc_AlarmWrite(CAR_ADC_STATE_WARN);
 * 备注信息：未定义 CAR_ADC_ALARM_GPIO 时为空操作，只保留状态位。
 */
static void Car_Adc_AlarmWrite(CarAdcState state)
{
#ifdef CAR_ADC_ALARM_GPIO
    GPIO_Level level;

    if (state == CAR_ADC_STATE_NORMAL)
    {
        level = (CAR_ADC_ALARM_ACTIVE_LEVEL == GPIO_High) ? GPIO_Low : GPIO_High;
    }
    else
    {
        level = CAR_ADC_ALARM_ACTIVE_LEVEL;
    }

    NEUQ_GPIO_write(CAR_ADC_ALARM_GPIO, level);
#else
    (void)state;
#endif
}

/* ========================================================================== */
/*                              对外接口                                       */
/* ========================================================================== */

/*
 * 函数名称：Car_Adc_Init
 * 函数功能：初始化 ADC 和可选报警引脚，并完成第一次采样。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     Car_Adc_Init();
 * 备注信息：内部调用 adc_init(CAR_ADC_CHANNEL, ADC_12BIT)；ADC 初始化失败会
 *           进入 NEUQ 库的 Error_Handler()。
 */
void Car_Adc_Init(void)
{
    adc_init(CAR_ADC_CHANNEL, ADC_12BIT);

    adc_raw = 0U;
    adc_pin_mv = 0U;
    adc_battery_mv = 0U;
    adc_filtered_mv = 0U;
    adc_filter_ready = 0U;
    adc_state = CAR_ADC_STATE_NORMAL;
    adc_candidate = CAR_ADC_STATE_NORMAL;
    adc_debounce = 0U;
    adc_last_ms = 0U;
    adc_started = 0U;

#ifdef CAR_ADC_ALARM_GPIO
    NEUQ_GPIO_init(CAR_ADC_ALARM_GPIO, GPIO_Mode_Output_PushPull,
                   (CAR_ADC_ALARM_ACTIVE_LEVEL == GPIO_High) ? GPIO_Low : GPIO_High);
#endif

    Car_Adc_AlarmWrite(CAR_ADC_STATE_NORMAL);
    (void)Car_Adc_Task();
}

/*
 * 函数名称：Car_Adc_Task
 * 函数功能：采样电池电压，更新滤波值和低电压状态。
 * 参数说明：无。
 * 返回参数：1 报警状态发生变化；0 未变化。
 * 使用示例：
 *     while (1) { Car_Adc_Task(); }
 * 备注信息：主循环每圈调用即可；内部按 CAR_ADC_PERIOD_MS 限速，
 *           每次连续转换 CAR_ADC_FILTER_SAMPLES 次。
 */
uint8_t Car_Adc_Task(void)
{
    uint32_t now = HAL_GetTick();
    CarAdcState previous;
    uint16_t sample;

    /* 主循环可以每圈调用，这里按 CAR_ADC_PERIOD_MS 限速。 */
    if (adc_started && (uint32_t)(now - adc_last_ms) < CAR_ADC_PERIOD_MS)
    {
        return 0U;
    }

    adc_started = 1U;
    adc_last_ms = now;
    previous = adc_state;

    sample = adc_mean_filter_convert(CAR_ADC_CHANNEL,
                                     CAR_ADC_FILTER_SAMPLES);

    adc_raw = sample;
    adc_pin_mv = Car_Adc_ConvertPinMv(sample);
    adc_battery_mv = Car_Adc_ConvertBatteryMv(adc_pin_mv);
    adc_filtered_mv = Car_Adc_Filter(adc_battery_mv);

    Car_Adc_ApplyState(Car_Adc_NextState(adc_filtered_mv, adc_state));

    return (adc_state != previous) ? 1U : 0U;
}

/*
 * 函数名称：Car_Adc_ReadRaw
 * 函数功能：立即读取一次 ADC 原始值。
 * 参数说明：无。
 * 返回参数：12 位原始值 0~4095；转换失败返回 0。
 * 使用示例：
 *     uint16_t raw = Car_Adc_ReadRaw();
 * 备注信息：会阻塞一次 ADC 转换（最长 10 ms 超时）。
 */
uint16_t Car_Adc_ReadRaw(void)
{
    return adc_convert(CAR_ADC_CHANNEL);
}

/*
 * 函数名称：Car_Adc_ReadPinMv
 * 函数功能：立即读取一次 ADC 引脚电压。
 * 参数说明：无。
 * 返回参数：引脚电压，单位 mV。
 * 使用示例：
 *     uint16_t pin_mv = Car_Adc_ReadPinMv();
 * 备注信息：无。
 */
uint16_t Car_Adc_ReadPinMv(void)
{
    return Car_Adc_ConvertPinMv(Car_Adc_ReadRaw());
}

/*
 * 函数名称：Car_Adc_ReadBatteryMv
 * 函数功能：立即读取一次电池电压（按分压比还原）。
 * 参数说明：无。
 * 返回参数：电池电压，单位 mV。
 * 使用示例：
 *     uint16_t vbat = Car_Adc_ReadBatteryMv();
 * 备注信息：无。
 */
uint16_t Car_Adc_ReadBatteryMv(void)
{
    return Car_Adc_ConvertBatteryMv(Car_Adc_ReadPinMv());
}

/*
 * 函数名称：Car_Adc_GetRaw
 * 函数功能：返回最近一次 Task 的 ADC 原始值。
 * 参数说明：无。
 * 返回参数：原始值 0~4095。
 * 使用示例：
 *     uint16_t raw = Car_Adc_GetRaw();
 * 备注信息：无。
 */
uint16_t Car_Adc_GetRaw(void)
{
    return adc_raw;
}

/*
 * 函数名称：Car_Adc_GetPinMv
 * 函数功能：返回最近一次 Task 的 ADC 引脚电压。
 * 参数说明：无。
 * 返回参数：引脚电压，单位 mV。
 * 使用示例：
 *     uint16_t mv = Car_Adc_GetPinMv();
 * 备注信息：无。
 */
uint16_t Car_Adc_GetPinMv(void)
{
    return adc_pin_mv;
}

/*
 * 函数名称：Car_Adc_GetBatteryMv
 * 函数功能：返回最近一次 Task 的电池电压（未滤波）。
 * 参数说明：无。
 * 返回参数：电池电压，单位 mV。
 * 使用示例：
 *     uint16_t vbat = Car_Adc_GetBatteryMv();
 * 备注信息：无。
 */
uint16_t Car_Adc_GetBatteryMv(void)
{
    return adc_battery_mv;
}

/*
 * 函数名称：Car_Adc_GetFilteredMv
 * 函数功能：返回最近一次 Task 的滤波后电池电压。
 * 参数说明：无。
 * 返回参数：滤波后电池电压，单位 mV。
 * 使用示例：
 *     uint16_t vbat = Car_Adc_GetFilteredMv();
 * 备注信息：报警状态基于该值判断。
 */
uint16_t Car_Adc_GetFilteredMv(void)
{
    return adc_filtered_mv;
}

/*
 * 函数名称：Car_Adc_GetState
 * 函数功能：返回当前低电压状态。
 * 参数说明：无。
 * 返回参数：CAR_ADC_STATE_NORMAL / WARN / CUT。
 * 使用示例：
 *     CarAdcState state = Car_Adc_GetState();
 * 备注信息：无。
 */
CarAdcState Car_Adc_GetState(void)
{
    return adc_state;
}

/*
 * 函数名称：Car_Adc_IsLow
 * 函数功能：判断是否进入警告或严重低压。
 * 参数说明：无。
 * 返回参数：1 警告或严重低压；0 正常。
 * 使用示例：
 *     if (Car_Adc_IsLow()) { }
 * 备注信息：无。
 */
uint8_t Car_Adc_IsLow(void)
{
    return (adc_state != CAR_ADC_STATE_NORMAL) ? 1U : 0U;
}

/*
 * 函数名称：Car_Adc_IsCut
 * 函数功能：判断是否进入严重低压。
 * 参数说明：无。
 * 返回参数：1 严重低压；0 其他。
 * 使用示例：
 *     if (Car_Adc_IsCut()) { }
 * 备注信息：无。
 */
uint8_t Car_Adc_IsCut(void)
{
    return (adc_state == CAR_ADC_STATE_CUT) ? 1U : 0U;
}
