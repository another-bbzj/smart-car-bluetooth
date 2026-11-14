#ifndef CAR_PARAMS_H
#define CAR_PARAMS_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * ============================================================================
 * 文件名称：Car_params.h
 * ============================================================================
 * 模块功能：菜单调参的掉电保存。
 *
 *   存储介质：STM32F407VE 内部 Flash 最后一个 128 KiB 扇区（扇区 11），
 *   地址 0x08060000，避让板载 W25Q 与 PA5/PA6/PA7 的引脚冲突。
 *
 *   安全约定：
 *     - 只有在车辆 STOP 状态才允许 Save；
 *     - Save 会擦除整个扇区并阻塞 CPU，必须由菜单在主循环调用；
 *     - 写入前后由 Car_Debug_FlashBegin/End 暂停 PIT 并重建控制时基，
 *       避免擦除时间被误判为控制周期异常；
 *     - 数据带 magic、version、size、CRC16 和逐项范围检查，加载失败
 *       时调用方必须回退默认值。
 * ============================================================================
 */

#define CAR_PARAM_MAGIC      0x4E455551UL   /* "NEUQ" */
#define CAR_PARAM_VERSION    1U
#define CAR_PARAM_FLASH_ADDR 0x08060000UL   /* 扇区 11 起始地址 */
#define CAR_PARAM_FLASH_SECTOR FLASH_SECTOR_11

/* 持久化参数：全部为整数，避免浮点格式差异。 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int32_t  kp_x100[2];      /* 左/右 Kp × 100 */
    int32_t  ki_x100[2];      /* 左/右 Ki × 100 */
    int32_t  kd_x100[2];      /* 左/右 Kd × 100 */
    int32_t  rpm_limit;       /* rpm */
    int32_t  pwm_limit;       /* PWM 数值 */
    int32_t  accel_limit;     /* rpm/s */
    int32_t  jerk_limit;      /* rpm/s² */
    int32_t  cpr_x10;         /* 每输出轴一圈计数 × 10 */
    int32_t  batt_warn_mv;    /* 低电压警告阈值 */
    int32_t  batt_cut_mv;     /* 严重低压阈值 */
    uint16_t crc;             /* 覆盖 crc 之前的全部字节 */
    uint16_t reserved;
} CarParams;

/**
 * @brief 填充电机与电池的编译期默认值。
 */
void Car_Params_Default(CarParams *out);

/**
 * @brief 检查全部字段范围与 CRC 所需字段。
 * @return 1 合法；0 非法。
 */
uint8_t Car_Params_Validate(const CarParams *params);

/**
 * @brief 从内部 Flash 读取并校验。
 * @return 1 有效；0 空、magic/版本/CRC/范围非法。
 */
uint8_t Car_Params_Load(CarParams *out);

/**
 * @brief 擦除扇区 11 并写入参数。
 * @return 1 成功且回读一致；0 失败。
 * @note 调用前必须已经处于 STOP，并由 Car_Debug_FlashBegin/End 包住。
 */
uint8_t Car_Params_Save(const CarParams *params);

/**
 * @brief Flash 中是否已有合法参数，供菜单显示。
 * @return 1 有效；0 无效。
 */
uint8_t Car_Params_IsPresent(void);

#endif
