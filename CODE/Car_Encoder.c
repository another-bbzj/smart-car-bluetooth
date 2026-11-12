/*********************************************************************************************************************
* 文件名称          Car_Encoder.c
* 功能说明          双路正交编码器的采样模块，属于"传感器"模块，基于 NEUQ 库的 Encoder 接口。
*
*                   本文件包含的函数：
*                   · CarEncoder_Init      —— 初始化 TIM4/TIM3 正交编码器并建立计数基准
*                   · CarEncoder_ReadDelta —— 读取本拍计数增量，处理 16 位回绕
*
*                   硬件映射：左轮 TIM4（PB6/PB7），右轮 TIM3（PA6/PA7）。
*                   本模块只输出"计数增量"，rpm 换算、低通滤波、堵转/方向保护都在 Car_Core。
*
* 修改记录
* 日期              作者                备注
* 2026-11-12                            从 Car_Hardware.c 拆出编码器外设模块
********************************************************************************************************************/
#include "Car_Encoder.h"
#include "Car_Config.h"
#include "NEUQ_lib_include.h"

#define CAR_ENCODER_COUNT  2U   /* 本车编码器数量：左、右各一路 */

/* 左右轮对应的定时器编号：0 左轮 TIM4，1 右轮 TIM3 */
static const Encoder_DEV_ENUM encoder_device[CAR_ENCODER_COUNT] = {
    Encoder_TIM4,
    Encoder_TIM3
};

/* 上一次读取的 16 位计数器值：用于求本拍增量，并在回绕时保持正确差值 */
static uint16_t encoder_previous[CAR_ENCODER_COUNT];

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarEncoder_Init
 * 功能说明    按四倍频正交编码器模式初始化 TIM4（左轮）和 TIM3（右轮），
 *             并把上一拍计数基准设置为当前计数器值。
 * 用法        上电初始化阶段调用一次，必须早于 CarEncoder_ReadDelta。
 * 参数说明    无
 * 返回说明    无
 * 示例        CarEncoder_Init();
 * 备注        初始化函数内部会调用 NEUQ_Encoder_Clear，把计数器置到中点 0x7FFF；
 *             这里保存的基准值也来自同一个 NEUQ_Encoder_Read 接口，两者保持一致。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarEncoder_Init(void)
{
    /* 左轮：TIM4 正交编码器，A 相 PB6、B 相 PB7 */
    NEUQ_Encoder_init(Encoder_TIM4, Encoder_TIM4_PhaseA_PB6, Encoder_TIM4_PhaseB_PB7);
    /* 右轮：TIM3 正交编码器，A 相 PA6、B 相 PA7 */
    NEUQ_Encoder_init(Encoder_TIM3, Encoder_TIM3_PhaseA_PA6, Encoder_TIM3_PhaseB_PA7);

    encoder_previous[CAR_ENCODER_LEFT]  = (uint16_t)NEUQ_Encoder_Read(encoder_device[CAR_ENCODER_LEFT]);
    encoder_previous[CAR_ENCODER_RIGHT] = (uint16_t)NEUQ_Encoder_Read(encoder_device[CAR_ENCODER_RIGHT]);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarEncoder_ReadDelta
 * 功能说明    读取指定轮编码器本控制周期的计数增量，并正确处理 16 位计数器回绕。
 * 用法        由 5ms 控制拍对左右轮各调用一次，调用频率必须与 CAR_PERIOD_MS 一致。
 * 参数说明    index —— 轮子下标：CAR_ENCODER_LEFT(0) 或 CAR_ENCODER_RIGHT(1)
 * 返回说明    本周期编码器计数增量，正负表示转向；index 越界时返回 0
 * 示例        int32_t left = CarEncoder_ReadDelta(CAR_ENCODER_LEFT);
 * 备注        1) 采用"读当前值再求差"而不是"读后清零"，避免读取与清零之间丢失编码器边沿；
 *             2) NEUQ_Encoder_Read 返回计数器相对 0x7FFF 的值，两次读数相减后常数抵消，
 *                再按 16 位补码解释即可得到正确增量；
 *             3) 符号取反由 Car_Core 按 CAR_*_ENCODER_INVERT 完成。
 *-------------------------------------------------------------------------------------------------------------------*/
int32_t CarEncoder_ReadDelta(uint8_t index)
{
    int32_t  now;                                           /* 本拍 NEUQ 编码器读数（相对 0x7FFF） */
    uint16_t now_u16;                                       /* 本拍读数截断为 16 位，便于求模差值 */
    uint16_t diff;                                          /* 本拍与上一拍的 16 位差值（模 65536） */

    if (index >= CAR_ENCODER_COUNT) {
        return 0;                                           /* 防御：只允许左右两路 */
    }

    now     = NEUQ_Encoder_Read(encoder_device[index]);     /* 读当前计数器（NEUQ 库函数） */
    now_u16 = (uint16_t)now;
    diff    = (uint16_t)(now_u16 - encoder_previous[index]); /* 16 位无符号差值，自动处理回绕 */
    encoder_previous[index] = now_u16;                      /* 保存本次读数，供下一拍使用 */

    if ((diff & 0x8000U) != 0U) {
        return (int32_t)diff - 65536;                       /* 最高位为 1：负向增量 */
    }
    return (int32_t)diff;                                   /* 最高位为 0：正向增量 */
}
