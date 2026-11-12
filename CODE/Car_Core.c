/*********************************************************************************************************************
* 文件名称          Car_Core.c
* 功能说明          双轮差速车控制核心的实现，属于"车辆控制与命令协议"模块，不直接操作外设。
*
*                   本文件包含的函数：
*                   · reset_motion  —— 内部工具：清除两轮目标、参考、输出和计时，并复位 PID
*                   · Car_Init      —— 初始化控制核心与左右轮默认 PID
*                   · Car_Fault     —— 进入故障状态并清空运动量
*                   · Car_Crc8      —— 蓝牙命令帧的 CRC-8 逐字节校验
*                   · execute       —— 内部工具：执行一条已通过 CRC 校验的完整命令
*                   · Car_FeedByte  —— 逐字节解析蓝牙命令帧（C3 3C CMD LEN DATA CRC8）
*                   · Car_Tick      —— 5ms 控制拍：测速、斜坡、PID、保护
*
*                   本文件只处理"控制逻辑与协议"，编码器采样、PWM 输出、串口收发分别由
*                   Car_Encoder、Car_Motor、Car_Link 负责，便于单独替换硬件。
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            first version
* 2026-11-12                            统一格式与中文注释，公共工具改用 Car_Utils
********************************************************************************************************************/
#include "Car_Core.h"
#include "Car_Config.h"
#include "Car_Utils.h"
#include <string.h>

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    reset_motion
 * 功能说明    清除两轮的运动状态：目标、斜坡参考、输出、堵转/方向计时，并将 PID 与滤波后的慢速值复位。
 *             不改变当前状态机状态（state）和故障码（fault）。
 * 用法        Car_Init、Car_Fault 以及收到"双轮零速""STOP"命令时在 Core 内部调用。
 * 参数说明    c —— 控制核心对象指针
 * 返回说明    无
 * 示例        reset_motion(&car);   // 两轮立即回到零目标、零输出
 * 备注        本函数只清软件状态，机械制动仍需驱动输出为 0 后自然完成。
 *-------------------------------------------------------------------------------------------------------------------*/
static void reset_motion(Car_Core *c)
{
    unsigned i;                                             /* 轮子下标：0 左轮，1 右轮 */

    for (i = 0U; i < 2U; i++) {
        c->target[i]       = 0.0f;                          /* 目标转速清零 */
        c->reference[i]    = 0.0f;                          /* 斜坡参考清零 */
        c->output[i]       = 0.0f;                          /* PID 输出清零 */
        c->stall_ms[i]     = 0U;                            /* 堵转计时清零 */
        c->direction_ms[i] = 0U;                            /* 方向异常计时清零 */
        c->slow_speed[i]   = Car_AbsFloat(c->speed[i]);     /* 慢速滤波值直接取当前实测绝对值 */
        Car_Pid_reset(&c->pid[i]);                          /* 位置式从零状态重新开始 */
#if CAR_PID_USE_INCREMENTAL
        Car_IncPid_reset(&c->inc_pid[i]);                   /* 增量式同时复位 */
#endif
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Init
 * 功能说明    初始化整车控制核心：整块清零，并为左右轮创建默认参数的速度环 PID（STOP 状态）。
 *             位置式始终初始化；增量式仅在 CAR_PID_USE_INCREMENTAL=1 时额外初始化。
 * 用法        上电初始化阶段调用一次，必须早于 Car_FeedByte / Car_Tick。
 * 参数说明    c —— 待初始化的 Car_Core 对象指针
 * 返回说明    无
 * 示例        static Car_Core car;
 *             Car_Init(&car);
 * 备注        PID 默认 Kp/Ki/Kd 与输出限幅来自 Car_Config.h，可在停车状态下通过协议动态改写。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Init(Car_Core *c)
{
    unsigned i;                                             /* 轮子下标：0 左轮，1 右轮 */

    memset(c, 0, sizeof(*c));                               /* 所有状态、计数、缓存先归零 */

    for (i = 0U; i < 2U; i++) {
        Car_Pid_init(&c->pid[i],                            /* 位置式：左右轮使用同一组默认参数 */
                     CAR_DEFAULT_KP,
                     CAR_DEFAULT_KI,
                     CAR_DEFAULT_KD,
                     CAR_PWM_LIMIT);
#if CAR_PID_USE_INCREMENTAL
        Car_IncPid_init(&c->inc_pid[i],                     /* 增量式：仅在选择增量模式时初始化 */
                        CAR_DEFAULT_KP,
                        CAR_DEFAULT_KI,
                        CAR_DEFAULT_KD,
                        CAR_PWM_LIMIT);
#endif
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Fault
 * 功能说明    进入故障状态：清除所有运动量、置故障码、丢弃当前解析中的残帧，两轮输出归零。
 * 用法        通信超时、堵转、方向异常、串口错误等保护条件成立时调用；故障后必须先 STOP 再 ARM。
 * 参数说明    c     —— 控制核心对象指针
 *             fault —— 故障码，取值 CAR_LINK_FAULT/CAR_STALL_FAULT/CAR_DIRECTION_FAULT/CAR_RX_FAULT
 * 返回说明    无
 * 示例        Car_Fault(&car, CAR_LINK_FAULT);   // 500ms 未收到有效续租指令，通信故障停车
 * 备注        本函数不会自动恢复；恢复流程为"STOP -> ARM"，目标从零重新开始。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Fault(Car_Core *c, uint8_t fault)
{
    reset_motion(c);                                        /* 两轮目标与输出立即归零 */
    c->state       = CAR_FAULT;                             /* 状态机进入故障态 */
    c->fault       = fault;                                 /* 记录本次故障码 */
    c->parse_state = 0U;                                    /* 丢弃可能解析到一半的残帧 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Crc8
 * 功能说明    按 CRC-8 逐字节更新校验值，多项式 0x07、初值 0，用于蓝牙命令帧校验。
 * 用法        发送端对 CMD、LEN、DATA 依次迭代；接收端解析时逐字节调用，最后与帧尾 CRC 比较。
 * 参数说明    crc  —— 上一次的 CRC 值，第一次传 0
 *             byte —— 本次参与计算的字节
 * 返回说明    更新后的 8 位 CRC
 * 示例        uint8_t crc = 0;
 *             crc = Car_Crc8(crc, 0x02);     // 对 CMD 计算，随后继续对 LEN/DATA 迭代
 * 备注        标准校验值：对 ASCII "123456789" 计算结果为 0xF4。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;                                             /* 8 个数据位的循环下标 */

    crc ^= byte;                                            /* 先把本字节异或进 CRC 低 8 位 */
    for (i = 0U; i < 8U; i++) {
        if ((crc & 0x80U) != 0U) {                          /* 最高位为 1：左移后异或多项式 */
            crc = (uint8_t)((crc << 1) ^ 0x07U);
        } else {                                            /* 最高位为 0：只左移 */
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    execute
 * 功能说明    执行一条已通过 CRC 校验的完整命令：ARM、STOP、WHEELS、PID、HEARTBEAT，
 *             并维护状态机、目标轮速、链路续租计时与命令结果码。
 * 用法        由 Car_FeedByte 在收到合法帧尾 CRC 后调用，业务层无需直接调用。
 * 参数说明    c —— 控制核心对象指针，需已填好 c->cmd、c->data、c->length
 * 返回说明    无（结果写入 c->result、c->state、c->target 等字段）
 * 示例        // 协议层解析完成后内部触发：
 *             // c->cmd = 3; 数据为左右轮大端 int16，execute(&c) 后 target 被更新
 * 备注        1) WHEELS/PID 只在特定状态下受理，否则返回 1（状态拒绝）；
 *             2) 双轮目标都为 0 时立即复位运动状态，实现软件急停；
 *             3) 未知命令返回 3，PID 轮号越界返回 2。
 *-------------------------------------------------------------------------------------------------------------------*/
static void execute(Car_Core *c)
{
    uint8_t wheel;                                          /* PID 命令里的轮号：0 左轮，1 右轮 */
    float   kp;                                             /* PID 命令解析出的比例系数 */
    float   ki;                                             /* PID 命令解析出的积分系数 */
    float   kd;                                             /* PID 命令解析出的微分系数 */

    c->frames++;                                            /* 合法帧计数加一，供遥测统计 */
    c->result = 0U;                                         /* 默认结果：成功 */

    switch (c->cmd) {
    case 1:                                                 /* ARM：停车状态下使能 */
        if (c->state != CAR_STOP) {
            c->result = 1U;                                 /* 非停车状态拒绝使能 */
            break;
        }
        reset_motion(c);                                    /* 从零目标、零积分开始 */
        c->link_ms = 0U;                                    /* 重新开始链路续租计时 */
        c->state   = CAR_RUN;                               /* 进入运行状态 */
        break;

    case 2:                                                 /* STOP：停车并清故障 */
        reset_motion(c);                                    /* 输出与运动量清零 */
        c->state   = CAR_STOP;                              /* 回到停车状态 */
        c->fault   = CAR_NO_FAULT;                          /* 清除故障码 */
        c->link_ms = 0U;                                    /* 链路计时清零 */
        break;

    case 3:                                                 /* WHEELS：左右轮目标转速 */
        if (c->state != CAR_RUN) {
            c->result = 1U;                                 /* 未使能时拒绝轮速命令 */
            break;
        }
        c->target[0] = Car_ClampFloat((float)Car_ReadI16BE(c->data), CAR_RPM_LIMIT);
        c->target[1] = Car_ClampFloat((float)Car_ReadI16BE(c->data + 2), CAR_RPM_LIMIT);
        if ((c->target[0] == 0.0f) && (c->target[1] == 0.0f)) {
            reset_motion(c);                                /* 双轮零速视为软件急停 */
        }
        c->link_ms = 0U;                                    /* 有效轮速命令续租链路 */
        break;

    case 4:                                                 /* PID：停车状态下修改某个轮的参数 */
        if (c->state != CAR_STOP) {
            c->result = 1U;                                 /* 运行中禁止改 PID */
            break;
        }
        wheel = c->data[0];                                 /* 数据第 0 字节为轮号 */
        if (wheel > 1U) {
            c->result = 2U;                                 /* 轮号只能为 0 或 1 */
            break;
        }
        kp = (float)Car_ReadU16BE(c->data + 1) / 100.0f;    /* Kp，协议放大 100 倍传输 */
        ki = (float)Car_ReadU16BE(c->data + 3) / 100.0f;    /* Ki */
        kd = (float)Car_ReadU16BE(c->data + 5) / 100.0f;    /* Kd */
        Car_Pid_init(&c->pid[wheel], kp, ki, kd, CAR_PWM_LIMIT);
#if CAR_PID_USE_INCREMENTAL
        Car_IncPid_init(&c->inc_pid[wheel], kp, ki, kd, CAR_PWM_LIMIT);
#endif
        break;

    case 5:                                                 /* HEARTBEAT：仅运行中续租链路 */
        if (c->state == CAR_RUN) {
            c->link_ms = 0U;
        }
        break;

    default:                                                /* 未知命令 */
        c->result = 3U;
        break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_FeedByte
 * 功能说明    逐字节接收蓝牙串口数据并做状态机解析：寻找 C3 3C 帧头、校验 CMD/LEN、累积 DATA、
 *             校验 CRC8；校验通过后交给 execute 执行命令。
 * 用法        在串口接收中断或主循环收帧处，每收到 1 字节调用一次；不要在多个上下文并发调用同一对象。
 * 参数说明    c    —— 控制核心对象指针
 *             byte —— 本次收到的字节
 * 返回说明    无（解析结果反映在 c->state/c->target/c->result 等字段）
 * 示例        void Car_UART_RX_Handler(uint8_t byte) {
 *                 Car_FeedByte(&car, byte);
 *             }
 * 备注        1) 帧格式：C3 3C CMD LEN DATA CRC8；
 *             2) 只接受 CMD 1~5 且 LEN 与规定长度一致，否则丢帧；
 *             3) 帧内字节间隔超过 CAR_FRAME_TIMEOUT_MS 时由 Car_Tick 丢弃残帧；
 *             4) CRC 错误的帧不会续租链路，正在运行的车会因超时停车。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    /* 各命令规定的数据长度，下标就是命令字：1/2/5 无数据，3 为 4 字节，4 为 7 字节。
     * 下标 0 不使用，填 255 使任何 CMD=0 的帧都判为非法。 */
    static const uint8_t lengths[6] = { 255U, 0U, 0U, 4U, 7U, 0U };

    c->frame_ms = 0U;                                       /* 收到任何一个字节都刷新帧间隔计时 */

    switch (c->parse_state) {
    case 0:                                                 /* 状态 0：寻找第一个帧头 0xC3 */
        if (byte == 0xC3U) {
            c->parse_state = 1U;
        }
        break;

    case 1:                                                 /* 状态 1：寻找第二个帧头 0x3C */
        if (byte == 0x3CU) {
            c->parse_state = 2U;
        } else if (byte == 0xC3U) {
            c->parse_state = 1U;                            /* 连续的 0xC3 仍可能是新帧头 */
        } else {
            c->parse_state = 0U;                            /* 其他字节丢弃，重新找帧头 */
        }
        break;

    case 2:                                                 /* 状态 2：接收 CMD */
        c->cmd         = byte;
        c->crc         = Car_Crc8(0U, byte);                /* CRC 从 CMD 开始计算 */
        c->parse_state = 3U;
        break;

    case 3:                                                 /* 状态 3：接收 LEN 并立即校验 */
        if ((c->cmd < 1U) || (c->cmd > 5U) || (byte != lengths[c->cmd])) {
            c->parse_state = (byte == 0xC3U) ? 1U : 0U;     /* 非法长度：尽量从本字节重新同步 */
            c->result      = 3U;
            break;
        }
        c->length = byte;                                   /* 保存数据长度 */
        c->index  = 0U;                                     /* 数据写入下标清零 */
        c->crc    = Car_Crc8(c->crc, byte);                 /* LEN 参与 CRC */
        c->parse_state = (byte != 0U) ? 4U : 5U;            /* 无数据则直接进入 CRC 状态 */
        break;

    case 4:                                                 /* 状态 4：逐字节接收 DATA */
        c->data[c->index] = byte;
        c->index++;
        c->crc = Car_Crc8(c->crc, byte);                    /* 每个数据字节参与 CRC */
        if (c->index == c->length) {
            c->parse_state = 5U;                            /* 数据收满，等待 CRC */
        }
        break;

    default:                                                /* 状态 5：接收帧尾 CRC */
        c->parse_state = 0U;
        if (byte == c->crc) {
            execute(c);                                     /* CRC 正确，执行命令 */
        } else {
            c->result = 3U;                                 /* CRC 错误，丢弃本帧 */
            if (byte == 0xC3U) {
                c->parse_state = 1U;                        /* 本字节可能是下一帧的帧头 */
            }
        }
        break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Tick
 * 功能说明    5ms 控制拍：处理残帧超时、链路超时保护，把编码器计数换算为 rpm 并低通滤波，
 *             执行目标斜坡、速度环 PID、零速复位，以及堵转与反馈方向异常保护。
 * 用法        由定时中断（本项目 TIM6，5ms）周期性调用，每个周期传入两轮自上一拍以来的编码器增量。
 * 参数说明    c           —— 控制核心对象指针
 *             left_count  —— 左轮本周期编码器增量（已按方向取符号）
 *             right_count —— 右轮本周期编码器增量（已按方向取符号）
 * 返回说明    无（结果写入 c->speed/c->reference/c->output，故障时进入 CAR_FAULT）
 * 示例        void Car_PIT_Handler(void) {
 *                 Car_Tick(&car, left_delta, right_delta);
 *             }
 * 备注        1) 运行中 500ms 没有有效轮速/心跳 -> 通信故障；
 *             2) 参考转速≥40rpm、输出≥1500 而实测<10rpm 持续 1.5s -> 堵转故障；
 *             3) 目标与实测反向、输出接近限幅且转速>100rpm 持续 1s -> 方向故障；
 *             4) 非运行状态输出强制为 0，但实测转速仍持续更新供回传使用。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
{
    int32_t counts[2];                                      /* 本拍编码器增量：0 左轮，1 右轮 */
    unsigned i;                                             /* 轮子下标：0 左轮，1 右轮 */

    counts[0] = left_count;
    counts[1] = right_count;

    /* 残帧超时：解析到一半但字节间隔超过 CAR_FRAME_TIMEOUT_MS，丢弃残帧重新同步。 */
    if ((c->parse_state != 0U) &&
        ((c->frame_ms += CAR_PERIOD_MS) >= CAR_FRAME_TIMEOUT_MS)) {
        c->parse_state = 0U;
    }

    /* 链路超时：运行中连续 500ms 没有有效轮速/心跳，判定通信故障并停车。 */
    if ((c->state == CAR_RUN) &&
        ((c->link_ms += CAR_PERIOD_MS) >= CAR_LINK_TIMEOUT_MS)) {
        Car_Fault(c, CAR_LINK_FAULT);
    }

    for (i = 0U; i < 2U; i++) {
        /* 编码器增量换算为 rpm：
         * 每拍计数 × (60000ms / (每转计数 × 控制周期ms)) = 每分钟转数。 */
        float raw = (float)counts[i] * (60000.0f / (CAR_ENCODER_CPR * CAR_PERIOD_MS));
        float abs_speed;                                /* 实测转速绝对值，用于堵转/方向判断 */
        float delta;                                    /* 本拍参考转速的允许增量 */

        /* 一阶低通：新值占 50%，抑制编码器量化噪声，同时保留足够带宽。 */
        c->speed[i] += 0.5f * (raw - c->speed[i]);

        if (c->state != CAR_RUN) {
            c->output[i] = 0.0f;                            /* 非运行状态强制零输出 */
            continue;
        }

        /* 目标斜坡：参考转速每个控制拍最多变化 CAR_RAMP_RPM_PER_TICK，避免阶跃冲击。 */
        delta = Car_ClampFloat(c->target[i] - c->reference[i], CAR_RAMP_RPM_PER_TICK);
        c->reference[i] += delta;

        if ((c->reference[i] == 0.0f) && (c->target[i] == 0.0f)) {
            c->output[i] = 0.0f;                            /* 目标与参考都为零：直接停机 */
            Car_Pid_reset(&c->pid[i]);                      /* 防止旧积分在下次启动时冲击 */
#if CAR_PID_USE_INCREMENTAL
            Car_IncPid_reset(&c->inc_pid[i]);               /* 增量式同步复位 */
#endif
        } else {
#if CAR_PID_USE_INCREMENTAL
            /* 增量式：返回的已经是累加并限幅后的绝对 PWM，可直接交给电机模块。 */
            c->output[i] = Car_IncPid_update(&c->inc_pid[i], c->reference[i], c->speed[i]);
#else
            /* 位置式：默认路径，保持原有控制行为。 */
            c->output[i] = Car_Pid_update(&c->pid[i], c->reference[i], c->speed[i]);
#endif
        }

        /* 慢速滤波：只用于方向异常判断，避免瞬时噪声误触发。 */
        abs_speed = Car_AbsFloat(c->speed[i]);
        c->slow_speed[i] += 0.05f * (abs_speed - c->slow_speed[i]);

        /* 堵转保护：参考转速足够、输出足够，但实测转速长期低于 10rpm。 */
        if ((Car_AbsFloat(c->reference[i]) >= 40.0f) &&
            (Car_AbsFloat(c->output[i]) >= 1500.0f) &&
            (abs_speed < 10.0f)) {
            c->stall_ms[i] += CAR_PERIOD_MS;
            if (c->stall_ms[i] >= CAR_STALL_TIMEOUT_MS) {
                Car_Fault(c, CAR_STALL_FAULT);
            }
        } else {
            c->stall_ms[i] = 0U;
        }

        /* 方向异常保护：参考与实测反向、输出与实测反向、输出接近限幅，
         * 且实测转速高于慢速滤波值，说明不是正常的减速过零，而是反馈或接线方向错误。 */
        if ((c->reference[i] * c->speed[i] < 0.0f) &&
            (c->output[i] * c->speed[i] < 0.0f) &&
            (Car_AbsFloat(c->output[i]) > (0.9f * CAR_PWM_LIMIT)) &&
            (abs_speed > 100.0f) &&
            (abs_speed >= (c->slow_speed[i] - 5.0f))) {
            c->direction_ms[i] += CAR_PERIOD_MS;
            if (c->direction_ms[i] >= CAR_DIRECTION_TIMEOUT_MS) {
                Car_Fault(c, CAR_DIRECTION_FAULT);
            }
        } else {
            c->direction_ms[i] = 0U;
        }
    }
}
