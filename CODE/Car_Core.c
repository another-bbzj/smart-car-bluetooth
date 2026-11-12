/*
 * ============================================================================
 * 文件名称：Car_Core.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件是蓝牙差速小车的“纯算法控制核心”，不直接访问任何 GPIO、PWM、
 * 编码器或串口。它把硬件层送来的字节流和编码器增量，转换为双轮 PWM 输出：
 *
 *     蓝牙字节 -> 协议解析 -> 状态机与目标转速
 *              -> 五次曲线参考规划 -> 双轮速度 PID -> 双轮输出
 *              -> 运行保护（通信超时、堵转、方向异常、周期异常）
 *
 * 硬件采样、PWM 写入、串口收发和定时中断接入由 Car_Hardware.c 完成；
 * 对外的结构体和接口说明见 Car_Core.h。
 *
 * 控制流程（每个控制周期调用一次 Car_TickDt）
 * ----------------------------------------------------------------------------
 *   1. 检查实际周期是否在允许范围内，越界立即进入时序故障；
 *   2. 检查蓝牙半帧超时和运行通信看门狗；
 *   3. 运行状态下推进双轮共享进度的五次参考曲线；
 *   4. 用编码器增量计算实测转速并做一阶低通；
 *   5. 对每一轮执行速度 PID，得到带符号 PWM 输出；
 *   6. 检查堵转和方向异常，必要时锁存故障并停车。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static float absolute(float x)
 *       —— 求浮点数绝对值，用于速度大小判断。
 *   static float clamp(float x, float lim)
 *       —— 把浮点数限制到 [-lim, +lim]，用于目标转速限幅。
 *   static void reset_motion(Car_Core *c)
 *       —— 停止运动：清空规划器、目标、参考、输出和保护计时。
 *   void Car_Init(Car_Core *c)
 *       —— 上电初始化：清空控制对象、创建规划器和双轮 PID。
 *   void Car_Fault(Car_Core *c, uint8_t fault)
 *       —— 锁存首次故障原因、立即清零输出并丢弃未收完的蓝牙帧。
 *   uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
 *       —— 计算 CRC-8（多项式 0x07），用于校验蓝牙命令帧。
 *   static uint16_t u16(const uint8_t *p)
 *       —— 从两个字节按大端顺序还原无符号 16 位整数。
 *   static float signed_rpm(const uint8_t *p)
 *       —— 从两个字节按大端顺序还原有符号 16 位转速，单位 rpm。
 *   static void execute(Car_Core *c)
 *       —— 执行一条已经通过长度和 CRC 校验的蓝牙命令。
 *   void Car_FeedByte(Car_Core *c, uint8_t byte)
 *       —— 逐字节解析蓝牙帧：C3 3C CMD LEN DATA CRC8。
 *   void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count,
 *                   uint32_t dt_us)
 *       —— 按实际控制周期更新测速、参考、PID、输出和运行保护。
 *   void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
 *       —— 固定 CAR_PERIOD_MS 周期的兼容入口，主要用于主机仿真。
 *
 * 单位约定
 * ----------------------------------------------------------------------------
 *   目标转速、参考转速、实测转速：rpm；
 *   参考加速度：rpm/s；
 *   控制周期、各类计时：微秒（us）；
 *   PID 输出：PWM 数值，正负号表示转向。
 *
 * 上下文约定
 * ----------------------------------------------------------------------------
 *   本文件不依赖 HAL。调用者必须提供有效的非空对象指针，并保证：
 *   - Car_FeedByte 与 Car_TickDt 在同一执行上下文中串行调用；
 *   - 跨上下文读取 Car_Core 时自行保证快照一致（硬件层用临界区实现）。
 * ============================================================================
 */

#include "Car_Core.h"
#include "Car_Config.h"

#include <string.h>
#include <float.h>

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：absolute
 * 函数功能：求浮点数的绝对值。
 * 参数说明：x —— 任意有限浮点数。
 * 返回参数：x 的绝对值。
 * 使用示例：
 *     abs_speed = absolute(c->speed[i]);
 * 备注信息：仅用于速度、误差等量的数值大小判断，不改变控制方向。
 */
static float absolute(float x)
{
    return x < 0.0f ? -x : x;
}

/*
 * 函数名称：clamp
 * 函数功能：把浮点数限制到 [-lim, +lim]。
 * 参数说明：x   —— 待限幅的数值；
 *           lim —— 正数形式的绝对值上限。
 * 返回参数：限幅后的数值。
 * 使用示例：
 *     c->target[0] = clamp(rpm, CAR_RPM_LIMIT);
 * 备注信息：调用者需保证 lim 为正有限数。
 */
static float clamp(float x, float lim)
{
    if (x > lim)     /* 正向超限压到上限 */
    {
        return lim;
    }

    if (x < -lim)    /* 负向超限压到下限 */
    {
        return -lim;
    }

    return x;        /* 未超限原样返回 */
}

/* 接收的是经协议校验的快照；只消费曲率，预瞄点横向位置不是车体横向误差。 */
void Car_SetVisionObservation(Car_Core *c, const CarVisionObservation *o, uint32_t now_ms)
{
    uint32_t elapsed;
    if (!o || !o->session || !(o->valid_mask & VISION_CURVATURE_VALID) ||
        !(o->confidence >= CAR_AUTO_MIN_CONFIDENCE && o->confidence <= 1.0f) ||
        !(o->curvature_inv_m <= CAR_AUTO_MAX_CURVATURE &&
          o->curvature_inv_m >= -CAR_AUTO_MAX_CURVATURE))
    {
        c->vision_present = 0;
        return;
    }
    elapsed = now_ms - o->received_ms;
    if (elapsed >= CAR_AUTO_MAX_AGE_MS ||
        o->processing_age_ms >= CAR_AUTO_MAX_AGE_MS - elapsed)
    {
        c->vision_present = 0;
        return;
    }
    /* 相同帧只可增加估计年龄，不能通过重复调用让停止更新的相机保活。 */
    if (c->vision_session != o->session || c->vision_sequence != o->sequence)
    {
        c->vision_age_us = (elapsed + o->processing_age_ms) * 1000U;
        c->vision_session = o->session;
        c->vision_sequence = o->sequence;
        c->vision_present = 1;
        c->vision_curvature = o->curvature_inv_m;
    }
    else if ((elapsed + o->processing_age_ms) * 1000U > c->vision_age_us)
    {
        c->vision_age_us = (elapsed + o->processing_age_ms) * 1000U;
    }
}

static uint8_t auto_observation_ready(const Car_Core *c)
{
    return c->vision_present && c->vision_age_us < CAR_AUTO_MAX_AGE_MS * 1000U;
}

/* 左正曲率：左轮慢、右轮快；同比限幅保持路径曲率。 */
static uint8_t auto_targets(Car_Core *c)
{
    float turn = c->vision_curvature * c->auto_track_m * 0.5f;
    float rpm, peak, scale;
    if (!(c->auto_diameter_m > 0.0f && c->auto_diameter_m <= FLT_MAX) ||
        !(turn >= -0.8f && turn <= 0.8f)) return 0;
    rpm = c->auto_speed_m_s * 60.0f / (3.14159265358979323846f * c->auto_diameter_m);
    peak = rpm * (1.0f + absolute(turn));
    if (!(peak >= 0.0f && peak <= FLT_MAX)) return 0;
    scale = peak > CAR_AUTO_RPM_LIMIT ? CAR_AUTO_RPM_LIMIT / peak : 1.0f;
    c->target[0] = rpm * (1.0f - turn) * scale;
    c->target[1] = rpm * (1.0f + turn) * scale;
    return 1;
}

/*
 * 函数名称：reset_motion
 * 函数功能：停止运动并清除规划器、PID 和保护计时。
 * 参数说明：c —— 控制对象。
 * 返回参数：无。
 * 使用示例：
 *     reset_motion(c);            // STOP 命令、ARM 启动或故障时调用
 * 备注信息：保留实测转速 speed[]，供后续测速滤波继续使用；
 *           速度环积分、目标、参考和输出全部清零。
 */
static void reset_motion(Car_Core *c)
{
    unsigned i;                                      /* 双轮循环下标，0 左轮 / 1 右轮 */

    CarReference_Reset(&c->profile);

    for (i = 0; i < 2; i++)
    {
        c->target[i] = 0.0f;                         /* 目标转速清零 */
        c->reference[i] = 0.0f;                      /* 参考转速清零 */
        c->output[i] = 0.0f;                         /* 控制输出清零 */
        c->stall_us[i] = 0;                          /* 堵转计时清零 */
        c->direction_us[i] = 0;                      /* 方向异常计时清零 */
        c->slow_speed[i] = absolute(c->speed[i]);    /* 慢速滤波重新播种 */
        Car_Pid_reset(&c->pid[i]);                   /* 清除速度环历史与故障 */
    }
}

/* ========================================================================== */
/*                              初始化与故障                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_Init
 * 函数功能：上电初始化控制对象，创建双轮共享进度的参考规划器和左右轮
 *           两套独立的速度 PID，初始化后进入 CAR_STOP。
 * 参数说明：c —— 待初始化的控制对象。
 * 返回参数：无。
 * 使用示例：
 *     Car_Core car;
 *     Car_Init(&car);
 * 备注信息：规划器限制和 PID 默认参数均来自 Car_Config.h；
 *           PID 配置失败时直接进入故障状态。
 */
void Car_Init(Car_Core *c)
{
    unsigned i;                                    /* 双轮循环下标，0 左轮 / 1 右轮 */

    memset(c, 0, sizeof(*c));                      /* 清空全部状态与历史 */

    /* 规划器限制来源：Car_Config.h 的指令限速、加速度和加加速度上限。 */
    CarReference_Init(&c->profile,
                      CAR_RPM_LIMIT,
                      CAR_ACCEL_LIMIT_RPM_S,
                      CAR_JERK_LIMIT_RPM_S2);

    for (i = 0; i < 2; i++)
    {
        /* 默认增益和输出限幅同样来自 Car_Config.h，停车时可由蓝牙修改。 */
        Car_Pid_init(&c->pid[i],
                     CAR_DEFAULT_KP,
                     CAR_DEFAULT_KI,
                     CAR_DEFAULT_KD,
                     CAR_PWM_LIMIT);

        if (!Car_Pid_configure(&c->pid[i],
                               CAR_PID_SETPOINT_WEIGHT,
                               CAR_PID_D_TAU_S,
                               1.0f,
                               CAR_FF_KS,
                               CAR_FF_KV,
                               CAR_FF_KA))
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);    /* 配置失败立即进入故障 */
        }
    }
}

/*
 * 函数名称：Car_Fault
 * 函数功能：锁存故障并立即停车。
 * 参数说明：c     —— 控制对象；
 *           fault —— 调用者传入的 CAR_*_FAULT 故障码。
 * 返回参数：无。
 * 使用示例：
 *     if (dt_us > CAR_MAX_CONTROL_US)
 *     {
 *         Car_Fault(&car, CAR_TIMING_FAULT);
 *     }
 * 备注信息：重复调用不会覆盖第一次记录的故障原因；
 *           清零输出只代表控制量为 0，不代表车轮已经物理停止。
 */
void Car_Fault(Car_Core *c, uint8_t fault)
{
    if (c->state != CAR_FAULT)    /* 只记录第一次故障原因 */
    {
        c->fault = fault;
    }

    reset_motion(c);              /* 清空目标、参考和输出 */
    c->state = CAR_FAULT;         /* 进入故障状态 */
    c->parse_state = 0;           /* 丢弃未收完的蓝牙帧 */
}

/* ========================================================================== */
/*                              蓝牙协议解析                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_Crc8
 * 函数功能：对一个字节更新 CRC-8 校验值。
 * 参数说明：crc  —— 当前累计 CRC；首字节传 0，后续传上次返回值；
 *           byte —— 本次参与计算的字节。
 * 返回参数：更新后的 CRC-8 值。
 * 使用示例：
 *     c->crc = Car_Crc8(0, c->cmd);       // 命令字节
 *     c->crc = Car_Crc8(c->crc, length);  // 长度字节
 * 备注信息：多项式 0x07，不反射，初值 0；调用方依次送入命令、长度和负载。
 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;                           /* 位循环下标，每个字节处理 8 位 */

    crc ^= byte;                          /* 当前字节并入 CRC */

    for (i = 0; i < 8; i++)
    {
        if (crc & 0x80U)                  /* 最高位为 1 时异或多项式 */
        {
            crc = (uint8_t)((crc << 1) ^ 0x07U);
        }
        else
        {
            crc = (uint8_t)(crc << 1);    /* 否则仅左移 */
        }
    }

    return crc;
}

/*
 * 函数名称：u16
 * 函数功能：从两个字节按大端顺序还原无符号 16 位整数。
 * 参数说明：p —— 指向帧中第一个字节的指针，要求 p[0]、p[1] 可读。
 * 返回参数：高字节在前组成的 uint16_t 数值。
 * 使用示例：
 *     uint16_t kp_raw = u16(c->data + 1);
 * 备注信息：蓝牙协议约定多字节字段均为大端序。
 */
static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] * 256U + p[1]);
}

/*
 * 函数名称：signed_rpm
 * 函数功能：从两个字节按大端顺序还原有符号 16 位转速。
 * 参数说明：p —— 指向帧中第一个字节的指针，要求 p[0]、p[1] 可读。
 * 返回参数：有符号转速值，单位 rpm，负值表示反转。
 * 使用示例：
 *     c->target[0] = signed_rpm(c->data);
 * 备注信息：协议把转速存成补码大端序；最高位为 1 时减去 65536 还原负值。
 */
static float signed_rpm(const uint8_t *p)
{
    uint16_t n = u16(p);    /* 原始无符号 16 位值 */

    if (n & 0x8000U)        /* 最高位为 1 表示负数 */
    {
        return (float)((int32_t)n - 65536);
    }

    return (float)((int32_t)n);
}

/*
 * 函数名称：execute
 * 函数功能：执行一条已经通过长度和 CRC 校验的蓝牙命令。
 * 参数说明：c —— 控制对象，命令号保存在 c->cmd，负载保存在 c->data。
 * 返回参数：无；结果写入 c->result：0 成功，1 状态不允许，
 *           2 参数无效，3 命令无效。
 * 使用示例：
 *     // 仅在 CRC 校验通过后由 Car_FeedByte 调用
 *     if (byte == c->crc)
 *     {
 *         execute(c);
 *     }
 * 备注信息：frames 计数包含被状态或参数校验拒绝的命令，
 *           便于上位机区分“收到了帧”和“命令生效”。
 */
static void execute(Car_Core *c)
{
    c->frames++;                          /* 统计进入执行阶段的帧数 */
    c->result = 0;                        /* 默认结果：成功 */

    switch (c->cmd)
    {
        case 1:
            /* ARM：只有停止状态能启动；故障后必须先发送 STOP 清除故障。 */
            if (c->state != CAR_STOP)
            {
                c->result = 1;            /* 状态不允许 */
                break;
            }

            reset_motion(c);              /* 清空旧目标和历史 */
            c->mode = CAR_MODE_MANUAL;
            c->link_us = 0;               /* 重置通信看门狗 */
            c->state = CAR_RUN;           /* 进入运行状态 */
            break;

        case 2:
            /* STOP：清除故障并立即归零，不等待速度规划曲线自然结束。 */
            reset_motion(c);              /* 立即清零输出 */
            c->state = CAR_STOP;          /* 回到停车状态 */
            c->mode = CAR_MODE_MANUAL;
            c->fault = 0;                 /* 清除故障码 */
            c->link_us = 0;               /* 重置通信看门狗 */
            break;

        case 3:
            /* WHEELS：左、右目标转速；双轮都为零时立即复位运动状态。 */
            if (c->state != CAR_RUN || c->mode != CAR_MODE_MANUAL)
            {
                c->result = 1;            /* 非运行状态拒绝转速命令 */
                break;
            }

            c->target[0] = clamp(signed_rpm(c->data), CAR_RPM_LIMIT);
            c->target[1] = clamp(signed_rpm(c->data + 2), CAR_RPM_LIMIT);

            if (c->target[0] == 0.0f && c->target[1] == 0.0f)
            {
                reset_motion(c);          /* 双轮零目标立即停车 */
            }

            c->link_us = 0;               /* 合法转速命令喂通信看门狗 */
            break;

        case 4:
        {
            /* PID：仅停车时调参；线上整数除以 100 后得到增益。 */
            Car_Pid_Struct next;          /* 临时对象，校验通过后整体替换 */

            if (c->state != CAR_STOP)
            {
                c->result = 1;            /* 运行中禁止修改 PID */
                break;
            }

            if (c->data[0] > 1)           /* 轮号只能是 0 或 1 */
            {
                c->result = 2;            /* 参数无效 */
                break;
            }

            if (!Car_Pid_init(&next,
                              u16(c->data + 1) / 100.0f,
                              u16(c->data + 3) / 100.0f,
                              u16(c->data + 5) / 100.0f,
                              CAR_PWM_LIMIT) ||
                !Car_Pid_configure(&next,
                                   CAR_PID_SETPOINT_WEIGHT,
                                   CAR_PID_D_TAU_S,
                                   1.0f,
                                   CAR_FF_KS,
                                   CAR_FF_KV,
                                   CAR_FF_KA))
            {
                c->result = 2;            /* 增益或前馈配置非法 */
                break;
            }

            c->pid[c->data[0]] = next;    /* 轮号 0 左轮、1 右轮 */
            break;
        }

        case 5:
            /* HEARTBEAT：运行时刷新通信看门狗，不改变目标转速。 */
            if (c->state == CAR_RUN)
            {
                c->link_us = 0;           /* 运行中喂看门狗 */
            }
            break;

        case 6:
        {
            uint16_t diameter = u16(c->data);
            uint16_t track = u16(c->data + 2);
            uint16_t speed = u16(c->data + 4);
            if (c->state != CAR_STOP) { c->result = 1; break; }
            if (diameter < 20 || diameter > 200 || track < 50 || track > 500 ||
                speed < 20 || speed > 150) { c->result = 2; break; }
            if (!auto_observation_ready(c) || absolute(c->speed[0]) >= 5.0f ||
                absolute(c->speed[1]) >= 5.0f) { c->result = 1; break; }
            c->auto_diameter_m = diameter * 0.001f;
            c->auto_track_m = track * 0.001f;
            c->auto_speed_m_s = speed * 0.001f;
            if (!auto_targets(c)) { reset_motion(c); c->result = 2; break; }
            reset_motion(c);
            c->auto_session = c->vision_session;
            c->mode = CAR_MODE_AUTO;
            c->link_us = 0;
            c->state = CAR_RUN;
            break;
        }

        default:
            c->result = 3;                /* 未知命令 */
            break;
    }
}

/*
 * 函数名称：Car_FeedByte
 * 函数功能：逐字节解析蓝牙帧。
 * 参数说明：c    —— 控制对象；
 *           byte —— 本次收到的原始字节，来源：蓝牙串口接收 FIFO。
 * 返回参数：无。
 * 使用示例：
 *     // 控制周期内从 FIFO 取出字节后逐个送入
 *     while (NEUQ_FIFO_read_byte(&rx_fifo, &byte) == NEUQ_FIFO_OK)
 *     {
 *         Car_FeedByte(&car, byte);
 *     }
 * 备注信息：帧格式为 C3 3C CMD LEN DATA CRC8。
 *           解析状态 0/1 用于寻找帧头，2/3 读取命令和长度，4 接收负载，
 *           5 校验。异常字节如果本身是 0xC3，则保留为下一帧的首字节，
 *           避免漏掉紧邻的新帧。
 */
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    /*
     * 命令 1~5 对应的负载长度：0、0、4、7、0。
     * 下标 0 占位，保证可以直接用命令号索引；先核验长度，
     * 才能保证后续写入 c->data[7] 不会越界。
     */
    static const uint8_t lengths[7] =
    {
        255,                                         /* 下标 0：无效命令占位 */
        0,                                           /* 命令 1：ARM，无负载 */
        0,                                           /* 命令 2：STOP，无负载 */
        4,                                           /* 命令 3：WHEELS，左右各 2 字节 */
        7,                                           /* 命令 4：PID，轮号 + 3 个 uint16 */
        0,                                           /* 命令 5：HEARTBEAT，无负载 */
        6                                            /* 命令 6：AUTO_ARM，三个 uint16 */
    };

    /*
     * 此处只刷新“帧字节间隔”；运行通信看门狗要等合法 ARM/WHEELS/
     * HEARTBEAT 执行时才刷新（STOP 也会清零），因此噪声或错误帧
     * 不能维持运行状态。
     */
    c->frame_us = 0;

    switch (c->parse_state)
    {
        case 0:
            /* 状态 0：寻找第一个帧头字节 0xC3。 */
            if (byte == 0xC3)
            {
                c->parse_state = 1;
            }
            break;

        case 1:
            /* 状态 1：确认第二个帧头字节 0x3C；若又收到 0xC3 则保持状态 1。 */
            if (byte == 0x3C)
            {
                c->parse_state = 2;
            }
            else if (byte == 0xC3)
            {
                c->parse_state = 1;
            }
            else
            {
                c->parse_state = 0;
            }
            break;

        case 2:
            /* 状态 2：接收命令号，并开始累计 CRC。 */
            c->cmd = byte;                           /* 保存命令号 */
            c->crc = Car_Crc8(0, byte);              /* CRC 从命令字节开始 */
            c->parse_state = 3;
            break;

        case 3:
            /* 状态 3：接收长度；命令号或长度非法时丢弃当前帧。 */
            if (c->cmd < 1 || c->cmd > 6 || byte != lengths[c->cmd])
            {
                c->parse_state = (byte == 0xC3) ? 1 : 0;
                c->result = 3;                       /* 帧格式错误 */
                break;
            }

            c->length = byte;                        /* 期望负载长度 */
            c->index = 0;                            /* 已收负载字节数 */
            c->crc = Car_Crc8(c->crc, byte);         /* 长度字节并入 CRC */
            c->parse_state = (byte != 0) ? 4 : 5;    /* 无负载直接校验 */
            break;

        case 4:
            /* 状态 4：逐字节接收负载，收满后进入校验状态。 */
            c->data[c->index++] = byte;              /* 保存负载字节 */
            c->crc = Car_Crc8(c->crc, byte);         /* 负载字节并入 CRC */

            if (c->index == c->length)
            {
                c->parse_state = 5;
            }
            break;

        default:
            /* 状态 5：最后一个字节应为 CRC；匹配后才会执行命令。 */
            c->parse_state = 0;

            if (byte == c->crc)
            {
                execute(c);                          /* CRC 正确，执行命令 */
            }
            else
            {
                c->result = 3;                       /* CRC 错误 */

                if (byte == 0xC3)                    /* 当前字节可能是下一帧帧头 */
                {
                    c->parse_state = 1;
                }
            }
            break;
    }
}

/* ========================================================================== */
/*                              周期控制与保护                                 */
/* ========================================================================== */

/*
 * 函数名称：Car_TickDt
 * 函数功能：使用实际控制周期更新测速、参考、PID 输出和运行保护。
 * 参数说明：c           —— 控制对象；
 *           left_count  —— 左轮本周期编码器增量（有符号，前进为正）；
 *           right_count —— 右轮本周期编码器增量（有符号，前进为正）；
 *           dt_us       —— 与计数增量对应的实际时间，单位微秒。
 * 返回参数：无；双轮控制量写入 c->output[0] 和 c->output[1]。
 * 使用示例：
 *     // 在 PIT 控制中断中，取完编码器增量后调用
 *     Car_TickDt(&car, left_delta, right_delta, dt_us);
 *     output(0, car.output[0]);
 *     output(1, car.output[1]);
 * 备注信息：周期越界、通信超时、堵转或方向异常都会触发故障并停车。
 *           本函数不操作 GPIO/PWM，硬件层负责把 output 应用到电机。
 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count,
                uint32_t dt_us)
{
    int32_t counts[2];                            /* 双轮编码器增量，0 左轮 / 1 右轮 */
    unsigned i;                                   /* 双轮循环下标 */
    float dt;                                     /* 实际控制周期，单位秒 */

    dt = (float)dt_us * 0.000001f;                /* 微秒换算为秒 */

    /* 周期越界时停止控制，避免用异常采样间隔计算速度和积分。 */
    if (dt_us < CAR_MIN_CONTROL_US || dt_us > CAR_MAX_CONTROL_US)
    {
        Car_Fault(c, CAR_TIMING_FAULT);
        return;
    }

    counts[0] = left_count;                       /* 左轮增量 */
    counts[1] = right_count;                      /* 右轮增量 */

    /* 蓝牙半帧超时：长时间收不到后续字节则丢弃当前半帧。 */
    if (c->parse_state &&
        (c->frame_us += dt_us) >= CAR_FRAME_TIMEOUT_MS * 1000U)
    {
        c->parse_state = 0;                       /* 丢弃半帧，重新寻找帧头 */
    }

    /* 运行通信看门狗：运行中长时间没有有效命令则判定通信故障。 */
    if (c->state == CAR_RUN &&
        (c->link_us += dt_us) >= CAR_LINK_TIMEOUT_MS * 1000U)
    {
        Car_Fault(c, CAR_LINK_FAULT);
    }

    if (c->vision_age_us < CAR_AUTO_MAX_AGE_MS * 1000U)
    {
        c->vision_age_us += dt_us;
    }
    if (c->state == CAR_RUN && c->mode == CAR_MODE_AUTO)
    {
        if (!auto_observation_ready(c) || c->auto_session != c->vision_session ||
            !auto_targets(c))
        {
            Car_Fault(c, CAR_VISION_FAULT);
        }
    }

    /* 运行状态下推进双轮共享进度的五次参考曲线。 */
    if (c->state == CAR_RUN)
    {
        CarReference_Update(&c->profile, c->target, dt);
    }

    for (i = 0; i < 2; i++)
    {
        /*
         * 计数增量 / 每圈计数 / 周期秒数 = 转/秒，
         * 再乘以 60 换算为每分钟转数（rpm）。
         */
        float raw = (float)counts[i] * (60.0f / (CAR_ENCODER_CPR * dt));
        float abs_speed;                          /* 实测转速绝对值，rpm */

        /*
         * 一阶低通滤波：系数随实际周期变化。停车时也继续更新实测转速，
         * 便于上位机观察车轮是否真的停住。
         */
        c->speed[i] += (dt / (CAR_SPEED_FILTER_TAU_S + dt)) *
                       (raw - c->speed[i]);

        if (c->state != CAR_RUN)
        {
            c->output[i] = 0.0f;                  /* 非运行状态撤去输出 */
            continue;
        }

        c->reference[i] = c->profile.value[i];    /* 本拍参考转速 */

        /*
         * 单轮目标和参考均归零后撤去驱动、清除积分；
         * 不以 PID 主动“抱住”零速。此处清零的是控制输出，
         * 车轮是否已经停住仍以 speed 反馈判断。
         */
        if (c->reference[i] == 0.0f && c->target[i] == 0.0f)
        {
            c->output[i] = 0.0f;
            Car_Pid_reset(&c->pid[i]);
        }
        else
        {
            c->output[i] = Car_Pid_update_dt(&c->pid[i],
                                             c->reference[i],
                                             c->speed[i],
                                             c->profile.acceleration[i],
                                             dt);
        }

        /* PID 内部检测到数值异常时会锁存 fault，这里立即转入整车故障。 */
        if (c->pid[i].fault)
        {
            Car_Fault(c, CAR_CONTROLLER_FAULT);
            continue;
        }

        abs_speed = absolute(c->speed[i]);        /* 取实测转速绝对值 */

        /* 转速慢速滤波：0.095 s 时间常数，用于识别换向时是否仍在减速。 */
        c->slow_speed[i] += (dt / (0.095f + dt)) *
                            (abs_speed - c->slow_speed[i]);

        /*
         * 堵转判断：
         * 参考转速和输出都比较大、实测转速却很低，且持续超过阈值时间，
         * 才判定为堵转，避免起步瞬间或短暂过载误触发。
         */
        if (absolute(c->reference[i]) >= 40.0f &&
            absolute(c->output[i]) >= 1500.0f &&
            abs_speed < 10.0f)
        {
            c->stall_us[i] += dt_us;

            if (c->stall_us[i] >= CAR_STALL_TIMEOUT_MS * 1000U)
            {
                Car_Fault(c, CAR_STALL_FAULT);
            }
        }
        else
        {
            c->stall_us[i] = 0;                   /* 条件解除，堵转计时清零 */
        }

        /*
         * 方向异常判断：
         * 参考方向、输出方向都与实测转速方向相反，输出接近限幅且转速较高，
         * 并持续超过阈值时间，说明车轮可能被卡住或接线方向错误。
         * slow_speed 用于排除明显减速过程中的正常换向；条件中断即清除计时。
         */
        if (c->reference[i] * c->speed[i] < 0.0f &&
            c->output[i] * c->speed[i] < 0.0f &&
            absolute(c->output[i]) > 0.9f * CAR_PWM_LIMIT &&
            abs_speed > 100.0f &&
            abs_speed >= c->slow_speed[i] - 5.0f)
        {
            c->direction_us[i] += dt_us;

            if (c->direction_us[i] >= CAR_DIRECTION_TIMEOUT_MS * 1000U)
            {
                Car_Fault(c, CAR_DIRECTION_FAULT);
            }
        }
        else
        {
            c->direction_us[i] = 0;               /* 条件解除，方向计时清零 */
        }
    }
}

/*
 * 函数名称：Car_Tick
 * 函数功能：固定周期的兼容入口，供主机仿真或固定周期调用者使用。
 * 参数说明：c           —— 控制对象；
 *           left_count  —— 左轮本周期编码器增量；
 *           right_count —— 右轮本周期编码器增量。
 * 返回参数：无。
 * 使用示例：
 *     Car_Tick(&car, left_delta, right_delta);
 * 备注信息：实际硬件使用 Car_TickDt，传入 TIM5 测得的真实周期。
 */
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
{
    Car_TickDt(c, left_count, right_count, CAR_PERIOD_MS * 1000U);
}
