/*
 * ============================================================================
 * 文件名称：Car_params.c
 * ============================================================================
 * 模块功能：菜单参数的内部 Flash 持久化。
 *
 *   使用 F407VE 的扇区 11（0x08060000~0x0807FFFF），代码当前约 20 KiB，
 *   不占用该扇区。写入流程：解锁 -> 擦除 -> 按字编程 -> 回读校验 -> 上锁。
 *   擦除/编程只能由菜单在 STOP 状态调用，并配合 Car_Debug_FlashBegin/End
 *   暂停 PIT、重建控制时基，避免把擦除耗时误判为周期故障。
 * ============================================================================
 */

#include <stddef.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "Car_Config.h"
#include "Car_adc.h"
#include "Car_params.h"

/* 按字编程要求结构体大小为 4 的整数倍；不满足时编译失败。 */
typedef char car_params_size_must_be_word_aligned[
    ((sizeof(CarParams) % 4U) == 0U) ? 1 : -1];

/* CRC16/CCITT-FALSE，初值 0xFFFF，多项式 0x1021，不反射。 */
static uint16_t param_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static uint16_t param_calc_crc(const CarParams *params)
{
    return param_crc16((const uint8_t *)params,
                       (uint32_t)offsetof(CarParams, crc));
}

void Car_Params_Default(CarParams *out)
{
    unsigned i;

    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->magic = CAR_PARAM_MAGIC;
    out->version = CAR_PARAM_VERSION;
    out->size = (uint16_t)sizeof(CarParams);

    for (i = 0U; i < 2U; i++)
    {
        out->kp_x100[i] = (int32_t)(CAR_DEFAULT_KP * 100.0f + 0.5f);
        out->ki_x100[i] = (int32_t)(CAR_DEFAULT_KI * 100.0f + 0.5f);
        out->kd_x100[i] = (int32_t)(CAR_DEFAULT_KD * 100.0f + 0.5f);
    }

    out->rpm_limit = (int32_t)CAR_RPM_LIMIT;
    out->pwm_limit = (int32_t)CAR_PWM_LIMIT;
    out->accel_limit = (int32_t)CAR_ACCEL_LIMIT_RPM_S;
    out->jerk_limit = (int32_t)CAR_JERK_LIMIT_RPM_S2;
    out->cpr_x10 = (int32_t)(CAR_ENCODER_CPR * 10.0f + 0.5f);
    out->batt_warn_mv = (int32_t)CAR_ADC_LOW_WARN_MV;
    out->batt_cut_mv = (int32_t)CAR_ADC_LOW_CUT_MV;
    out->crc = param_calc_crc(out);
}

uint8_t Car_Params_Validate(const CarParams *params)
{
    unsigned i;

    if (params == NULL ||
        params->magic != CAR_PARAM_MAGIC ||
        params->version != CAR_PARAM_VERSION ||
        params->size != (uint16_t)sizeof(CarParams))
    {
        return 0U;
    }

    for (i = 0U; i < 2U; i++)
    {
        if (params->kp_x100[i] < 0 || params->kp_x100[i] > 65535 ||
            params->ki_x100[i] < 0 || params->ki_x100[i] > 65535 ||
            params->kd_x100[i] < 0 || params->kd_x100[i] > 65535)
        {
            return 0U;
        }
    }

    if (params->rpm_limit < 10 || params->rpm_limit > 300 ||
        params->pwm_limit < 100 || params->pwm_limit > 10000 ||
        params->accel_limit < 10 || params->accel_limit > 5000 ||
        params->jerk_limit < 10 || params->jerk_limit > 20000 ||
        params->cpr_x10 < 100 || params->cpr_x10 > 1000000 ||
        params->batt_warn_mv < 1000 || params->batt_warn_mv > 30000 ||
        params->batt_cut_mv < 1000 || params->batt_cut_mv > 30000 ||
        params->batt_cut_mv > params->batt_warn_mv)
    {
        return 0U;
    }

    return 1U;
}

uint8_t Car_Params_Load(CarParams *out)
{
    CarParams temp;

    if (out == NULL)
    {
        return 0U;
    }

    memcpy(&temp, (const void *)CAR_PARAM_FLASH_ADDR, sizeof(temp));

    if (!Car_Params_Validate(&temp) ||
        temp.crc != param_calc_crc(&temp))
    {
        return 0U;
    }

    *out = temp;
    return 1U;
}

uint8_t Car_Params_IsPresent(void)
{
    CarParams temp;

    return Car_Params_Load(&temp);
}

uint8_t Car_Params_Save(const CarParams *params)
{
    CarParams temp;
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;
    uint32_t address;
    uint32_t word;
    uint8_t ok = 1U;
    uint32_t i;

    if (!Car_Params_Validate(params))
    {
        return 0U;
    }

    temp = *params;
    temp.magic = CAR_PARAM_MAGIC;
    temp.version = CAR_PARAM_VERSION;
    temp.size = (uint16_t)sizeof(CarParams);
    temp.crc = param_calc_crc(&temp);

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 0U;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = CAR_PARAM_FLASH_SECTOR;
    erase.NbSectors = 1U;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        ok = 0U;
    }

    if (ok)
    {
        for (i = 0U; i < sizeof(CarParams); i += 4U)
        {
            memcpy(&word, ((const uint8_t *)&temp) + i, 4U);
            address = CAR_PARAM_FLASH_ADDR + i;
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
                                  (uint64_t)word) != HAL_OK)
            {
                ok = 0U;
                break;
            }
        }
    }

    (void)HAL_FLASH_Lock();

    if (!ok)
    {
        return 0U;
    }

    /* 回读校验，避免擦除/编程静默失败。 */
    memcpy(&temp, (const void *)CAR_PARAM_FLASH_ADDR, sizeof(temp));
    return (Car_Params_Validate(&temp) &&
            temp.crc == param_calc_crc(&temp)) ? 1U : 0U;
}
