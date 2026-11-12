/*
 * ============================================================================
 * 文件名称：Car_PID.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件实现双轮速度闭环所用的两种纯算法 PID 控制器，不访问任何外设，
 * 也不直接输出 PWM。每个控制周期的调用关系如下：
 *
 *     Car_Core.c（控制周期任务）
 *         -> Car_Pid_update_dt() 或 Car_PidInc_update_dt()
 *         -> 得到带符号的 PWM 数值
 *         -> Car_Hardware.c 把该数值写入电机驱动。
 *
 * 本文件包含两种实现：
 *
 *   1. 位置式 PID（Car_Pid_*）
 *      输出 = P + I + D + 前馈。积分项在结构体中显式累加；比例项支持目标
 *      权重 beta；微分项对“测量值”求导并做一阶低通，因此目标突然变化时
 *      不会产生微分冲击；积分项自身带限幅，并采用条件积分抗饱和。
 *      当前 Car_Core.c 实际使用的就是这一种。
 *
 *   2. 增量式 PID（Car_PidInc_*）
 *      不直接计算绝对输出，而是计算本周期相对上一周期的输出增量 Δu，
 *      再累加到上一周期输出上：u[k] = u[k-1] + Δu[k]。
 *      各项定义、微分滤波、前馈、输出限幅和抗饱和策略都与位置式保持一致，
 *      便于两种算法互换、对比调试。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static uint8_t number(float x)
 *       —— 判断浮点数是否为有限值，过滤 NaN 和正负无穷。
 *   static float clip(float x, float limit)
 *       —— 把浮点数限制到 [-limit, +limit] 区间。
 *   void Car_Pid_reset(Car_Pid_Struct *p)
 *       —— 保留位置式 PID 的配置，清除动态历史与故障标志。
 *   uint8_t Car_Pid_init(Car_Pid_Struct *p, float kp, float ki, float kd,
 *                        float limit)
 *       —— 初始化位置式 PID，并校验增益和输出上限。
 *   uint8_t Car_Pid_configure(Car_Pid_Struct *p, float beta, float tau,
 *                             float fraction, float ks, float kv, float ka)
 *       —— 配置位置式 PID 的目标权重、微分滤波、积分限幅和前馈。
 *   float Car_Pid_update_dt(Car_Pid_Struct *p, float target, float measure,
 *                           float acceleration, float dt)
 *       —— 使用实际周期执行一次位置式 PID 计算。
 *   float Car_Pid_update(Car_Pid_Struct *p, float target, float measure)
 *       —— 固定 5 ms 周期的位置式 PID 兼容入口。
 *   void Car_PidInc_reset(Car_PidInc_Struct *p)
 *       —— 保留增量式 PID 的配置，清除动态历史与故障标志。
 *   uint8_t Car_PidInc_init(Car_PidInc_Struct *p, float kp, float ki,
 *                           float kd, float limit)
 *       —— 初始化增量式 PID，并校验增益和输出上限。
 *   uint8_t Car_PidInc_configure(Car_PidInc_Struct *p, float beta, float tau,
 *                                float fraction, float ks, float kv, float ka)
 *       —— 配置增量式 PID 的目标权重、微分滤波、积分限幅和前馈。
 *   float Car_PidInc_update_dt(Car_PidInc_Struct *p, float target,
 *                              float measure, float acceleration, float dt)
 *       —— 使用实际周期执行一次增量式 PID 计算。
 *   float Car_PidInc_update(Car_PidInc_Struct *p, float target, float measure)
 *       —— 固定 5 ms 周期的增量式 PID 兼容入口。
 *
 * 单位约定
 * ----------------------------------------------------------------------------
 *   目标转速 target、实测转速 measure：rpm；
 *   参考加速度 acceleration：rpm/s；
 *   控制周期 dt：秒；
 *   输出、前馈项和积分项：PWM 数值（NEUQ 库 PWM_MAX = 10000 对应 100%）。
 *
 * Ki / Kd 的离散约定
 * ----------------------------------------------------------------------------
 *   为兼容既有蓝牙调参工具，Ki、Kd 仍按 5 ms 周期标定：
 *       积分增量 = Ki * error * (dt / 0.005)
 *       微分项   = -Kd * 0.005 * d(measure) / dt
 *   实际周期不是 5 ms 时，上面两式会自动按实际周期缩放。
 *
 * 数值安全
 * ----------------------------------------------------------------------------
 *   所有输入和中间结果都会检查是否为有限值。一旦发现 NaN、正负无穷、
 *   参数越界或周期异常，控制器会锁存 fault 标志并返回 0，由 Car_Core.c
 *   据此转入故障保护状态。
 * ============================================================================
 */

#include "Car_PID.h"

#include <float.h>
#include <string.h>

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：number
 * 函数功能：判断一个浮点数是否为有限值，过滤 NaN 和正负无穷。
 * 参数说明：x —— 待检查的浮点数。
 * 返回参数：1 表示有限值；0 表示 NaN 或正负无穷。
 * 使用示例：
 *     if (!number(kp))
 *     {
 *         return 0;
 *     }
 * 备注信息：NaN 与任何数比较都为假，因此只有有限值同时满足
 *           x <= FLT_MAX 和 x >= -FLT_MAX。
 */
static uint8_t number(float x)
{
    return x <= FLT_MAX && x >= -FLT_MAX;    /* 有限值返回 1，NaN/无穷返回 0 */
}

/*
 * 函数名称：clip
 * 函数功能：把浮点数限制到 [-limit, +limit] 区间。
 * 参数说明：x     —— 待限幅的数值；
 *           limit —— 正数形式的绝对值上限。
 * 返回参数：限幅后的数值。
 * 使用示例：
 *     output = clip(output, p->output_limit);
 * 备注信息：调用者需保证 limit 为正有限数；本函数不检查 limit 本身。
 */
static float clip(float x, float limit)
{
    if (x > limit)     /* 正向超限压到上限 */
    {
        return limit;
    }

    if (x < -limit)    /* 负向超限压到下限 */
    {
        return -limit;
    }

    return x;          /* 未超限原样返回 */
}

/* ========================================================================== */
/*                            位置式 PID 实现                                 */
/* ========================================================================== */

/*
 * 函数名称：Car_Pid_reset
 * 函数功能：保留增益、输出限幅、积分限幅和前馈配置，清除积分、微分历史、
 *           最近输出和全部状态标志，使控制器回到刚初始化完成的状态。
 * 参数说明：p —— 位置式 PID 对象，调用者需保证非空且独占访问。
 * 返回参数：无。
 * 使用示例：
 *     Car_Pid_reset(&car.pid[0]);       // 复位左轮速度环
 *     Car_Pid_reset(&car.pid[1]);       // 复位右轮速度环
 * 备注信息：fault 也会被清除；如果 fault 是配置参数异常引起的，必须先修正
 *           参数，再调用本函数恢复控制。
 */
void Car_Pid_reset(Car_Pid_Struct *p)
{
    p->integral = 0.0f;        /* 积分项清零 */
    p->last_measure = 0.0f;    /* 上一拍测量值清零 */
    p->derivative = 0.0f;      /* 滤波后的测量变化率清零 */
    p->p_term = 0.0f;          /* 比例项输出清零 */
    p->d_term = 0.0f;          /* 微分项输出清零 */
    p->ff_term = 0.0f;         /* 前馈项输出清零 */
    p->last_output = 0.0f;     /* 最近一次输出清零 */
    p->started = 0;            /* 重新等待首个测量周期 */
    p->fault = 0;              /* 清除锁存故障 */
    p->saturated = 0;          /* 清除饱和标志 */
}

/*
 * 函数名称：Car_Pid_init
 * 函数功能：清空位置式 PID 对象，并初始化增益与输出上限。
 * 参数说明：p     —— 待初始化的 PID 对象；
 *           kp    —— 比例增益，范围 [0, 655.35]；
 *           ki    —— 积分增益，范围 [0, 655.35]；
 *           kd    —— 微分增益，范围 [0, 655.35]；
 *           limit —— 输出绝对值上限，范围 (0, 10000]。
 * 返回参数：1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * 使用示例：
 *     Car_Pid_Struct pid;
 *     if (!Car_Pid_init(&pid, 2.0f, 0.12f, 0.0f, 3500.0f))
 *     {
 *         // 参数非法，pid.fault 已置 1
 *     }
 * 备注信息：默认 beta = 1、微分滤波时间常数 0.02 s、积分上限等于输出上限、
 *           前馈系数全为 0。
 */
uint8_t Car_Pid_init(Car_Pid_Struct *p, float kp, float ki, float kd, float limit)
{
    memset(p, 0, sizeof(*p));       /* 先整体清零，失败路径不留旧数据 */

    /* 统一检查增益和限幅：必须为有限值、非负且在允许范围内。 */
    if (!number(kp) || !number(ki) || !number(kd) || !number(limit) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        kp > 655.35f || ki > 655.35f || kd > 655.35f ||
        limit <= 0.0f || limit > 10000.0f)
    {
        p->fault = 1;               /* 锁存配置故障 */
        return 0;
    }

    p->kp = kp;                     /* 比例增益 */
    p->ki = ki;                     /* 积分增益 */
    p->kd = kd;                     /* 微分增益 */
    p->output_limit = limit;        /* 输出限幅 */
    p->integral_limit = limit;      /* 积分限幅默认等于输出限幅 */
    p->beta = 1.0f;                 /* 默认使用完整目标 */
    p->derivative_tau_s = 0.02f;    /* 默认微分滤波 20 ms */

    return 1;
}

/*
 * 函数名称：Car_Pid_configure
 * 函数功能：在已初始化的位置式 PID 上配置目标权重、微分滤波、积分限幅
 *           和前馈系数。
 * 参数说明：p        —— 已初始化的位置式 PID 对象；
 *           beta     —— 比例项目标权重，范围 [0, 1]；
 *           tau      —— 微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波；
 *           fraction —— 积分上限比例，范围 (0, 1]，
 *                       积分上限 = fraction × output_limit；
 *           ks       —— 静摩擦前馈，范围 [0, output_limit]，单位 PWM；
 *           kv       —— 速度前馈系数，范围 [0, 10000]，单位 PWM/rpm；
 *           ka       —— 加速度前馈系数，范围 [0, 10000]，单位 PWM/(rpm/s)。
 * 返回参数：1 成功并清除动态历史及故障；0 拒绝修改，原配置和历史保持不变。
 * 使用示例：
 *     Car_Pid_configure(&pid, 1.0f, 0.02f, 1.0f, 0.0f, 0.0f, 0.0f);
 * 备注信息：所有参数都必须为有限数；校验失败时不会写入任何一个字段。
 */
uint8_t Car_Pid_configure(Car_Pid_Struct *p, float beta, float tau,
                          float fraction, float ks, float kv, float ka)
{
    /* 配置参数必须有限且在合理范围内，否则整体拒绝，保持原配置不变。 */
    if (!number(beta) || !number(tau) || !number(fraction) ||
        !number(ks) || !number(kv) || !number(ka) ||
        beta < 0.0f || beta > 1.0f ||
        tau < 0.0f || tau > 1.0f ||
        fraction <= 0.0f || fraction > 1.0f ||
        ks < 0.0f || kv < 0.0f || ka < 0.0f ||
        ks > p->output_limit || kv > 10000.0f || ka > 10000.0f ||
        p->output_limit <= 0.0f)
    {
        return 0;
    }

    p->beta = beta;                                    /* 比例项目标权重 */
    p->derivative_tau_s = tau;                         /* 微分滤波时间常数 */
    p->integral_limit = fraction * p->output_limit;    /* 积分项幅值上限 */
    p->ks = ks;                                        /* 静摩擦前馈 */
    p->kv = kv;                                        /* 速度前馈 */
    p->ka = ka;                                        /* 加速度前馈 */

    /* 配置变化后旧的历史数据已不适用，清空动态状态再恢复控制。 */
    Car_Pid_reset(p);

    return 1;
}

/*
 * 函数名称：Car_Pid_update_dt
 * 函数功能：使用实际控制周期执行一次位置式 PID 计算，输出带符号 PWM 数值。
 * 参数说明：p            —— 位置式 PID 对象；
 *           target       —— 目标转速，单位 rpm；
 *           measure      —— 实测转速，单位 rpm；
 *           acceleration —— 参考加速度，单位 rpm/s，用于 Ka 前馈；
 *           dt           —— 实际控制周期，单位秒，允许 [0.0001, 0.1]。
 * 返回参数：[-output_limit, output_limit] 内的 PWM 数值；
 *           输入异常或对象已锁存故障时返回 0。
 * 使用示例：
 *     float pwm = Car_Pid_update_dt(&car.pid[0], 100.0f, speed_rpm,
 *                                   accel_rpm_s, 0.005f);
 *     if (car.pid[0].fault) { Car_Fault(&car, CAR_CONTROLLER_FAULT); }
 * 备注信息：返回 0 也可能是正常输出（例如目标为零且无扰动），调用者必须
 *           结合 p->fault 判断是否发生故障。故障一旦锁存，后续调用直接
 *           返回 0，直到 reset()。
 */
float Car_Pid_update_dt(Car_Pid_Struct *p, float target, float measure,
                       float acceleration, float dt)
{
    float error;                                           /* 速度误差 = target - measure，rpm */
    float raw_d = 0.0f;                                    /* 未滤波的测量变化率，rpm/s */
    float d_alpha;                                         /* 微分低通系数 */
    float base;                                            /* P + D + 前馈，不含积分 */
    float candidate;                                       /* 限幅后的积分项候选值 */
    float output;                                          /* 叠加积分后的总输出 */

    if (p->fault)                                          /* 已锁存故障，保持输出为 0 */
    {
        return 0.0f;
    }

    /* 统一检查输入、周期和对象内部数值，任一异常都进入 invalid。 */
    if (!number(target) || !number(measure) || !number(acceleration) || !number(dt) ||
        dt < 0.0001f || dt > 0.1f ||
        !number(p->kp) || !number(p->ki) || !number(p->kd) || !number(p->integral) ||
        !number(p->output_limit) || p->output_limit <= 0.0f ||
        p->output_limit > 10000.0f)
    {
        goto invalid;
    }

    error = target - measure;                              /* 完整速度误差，rpm */

    /*
     * 对测量值求导，而不是对误差求导：目标阶跃时不会出现微分冲击。
     * 首个控制周期没有上一拍测量值，因此不计算差分，避免上电瞬间
     * 产生虚假的微分尖峰。
     */
    if (p->started)
    {
        raw_d = (measure - p->last_measure) / dt;
    }

    p->last_measure = measure;                             /* 保存本拍测量值 */
    p->started = 1;                                        /* 从下一拍开始可以求微分 */

    /* 一阶低通抑制编码器量化噪声；系数随实际 dt 调整。 */
    d_alpha = dt / (p->derivative_tau_s + dt);
    p->derivative += d_alpha * (raw_d - p->derivative);

    /*
     * 比例项：beta 只作用于目标值。beta = 0 时相当于纯反馈比例，
     * beta = 1 时退化为常见的 Kp × (target - measure)。
     */
    p->p_term = p->kp * (p->beta * target - measure);

    p->d_term = -p->kd * 0.005f * p->derivative;           /* 微分项，5 ms 约定 */

    p->ff_term = p->kv * target + p->ka * acceleration;    /* 速度 + 加速度前馈 */

    if (target > 1.0f)                                     /* 正转时叠加正向静摩擦补偿 */
    {
        p->ff_term += p->ks;
    }
    else if (target < -1.0f)                               /* 反转时叠加负向静摩擦补偿 */
    {
        p->ff_term -= p->ks;
    }

    base = p->p_term + p->d_term + p->ff_term;             /* 不含积分的基础输出 */

    /*
     * 先限制积分项自身的幅值，再判断总输出是否允许接纳本次积分增量。
     * 按 5 ms 离散约定，积分步长为 Ki × error × (dt / 0.005)。
     */
    candidate = clip(p->integral + p->ki * error * (dt / 0.005f),
                     p->integral_limit);

    if (!number(base) || !number(candidate))
    {
        goto invalid;
    }

    output = base + candidate;                             /* 临时总输出，用于饱和判断 */

    /*
     * 条件积分抗饱和：
     * 如果总输出已经超过正限幅且误差仍为正，说明继续积分只会把积分项
     * 推得更大，退出饱和时容易产生超调，因此本次不接收积分增量；
     * 误差反向时允许积分退出饱和。负方向同理。
     */
    if (!((output > p->output_limit && error > 0.0f) ||
          (output < -p->output_limit && error < 0.0f)))
    {
        p->integral = candidate;                           /* 接收本次积分增量 */
    }

    output = base + p->integral;                           /* 用最终积分值重新计算总输出 */

    if (!number(output))
    {
        goto invalid;
    }

    p->saturated = (output > p->output_limit || output < -p->output_limit);
    p->last_output = clip(output, p->output_limit);

    return p->last_output;

invalid:
    /*
     * 输入或中间计算异常时锁存故障并返回零，
     * 上层（Car_Core.c）检测到 fault 后会进入故障状态并停车。
     */
    p->fault = 1;                                          /* 锁存数值故障 */
    p->last_output = 0.0f;                                 /* 立即撤去输出 */

    return 0.0f;
}

/*
 * 函数名称：Car_Pid_update
 * 函数功能：固定 5 ms 控制周期的位置式 PID 兼容入口。
 * 参数说明：p       —— 位置式 PID 对象；
 *           target  —— 目标转速，单位 rpm；
 *           measure —— 实测转速，单位 rpm。
 * 返回参数：同 Car_Pid_update_dt()。
 * 使用示例：
 *     float pwm = Car_Pid_update(&pid, 100.0f, speed_rpm);
 * 备注信息：参考加速度按 0 处理，适用于没有规划器的简单调用场景。
 */
float Car_Pid_update(Car_Pid_Struct *p, float target, float measure)
{
    return Car_Pid_update_dt(p, target, measure, 0.0f, 0.005f);
}

/* ========================================================================== */
/*                            增量式 PID 实现                                 */
/* ========================================================================== */

/*
 * 函数名称：Car_PidInc_reset
 * 函数功能：保留增量式 PID 的增益与限幅配置，清除积分、微分历史、
 *           上一拍输出和全部状态标志。
 * 参数说明：p —— 增量式 PID 对象，调用者需保证非空且独占访问。
 * 返回参数：无。
 * 使用示例：
 *     Car_PidInc_reset(&pid_inc);
 * 备注信息：last_output 清零意味着下一次调用的输出从 0 开始累加；
 *           如果车辆正在运动，应先确保执行器已经撤去驱动。
 */
void Car_PidInc_reset(Car_PidInc_Struct *p)
{
    p->integral = 0.0f;        /* 积分项清零 */
    p->last_measure = 0.0f;    /* 上一拍测量值清零 */
    p->derivative = 0.0f;      /* 滤波后的测量变化率清零 */
    p->last_output = 0.0f;     /* 上一拍执行器输出清零 */
    p->unclamped_output = 0.0f; /* 未限幅递推基准清零，下次从 0 累加 */
    p->last_p_term = 0.0f;     /* 上一拍比例项清零 */
    p->last_d_term = 0.0f;     /* 上一拍微分项清零 */
    p->last_ff_term = 0.0f;    /* 上一拍前馈项清零 */
    p->p_term = 0.0f;          /* 本拍比例项清零 */
    p->d_term = 0.0f;          /* 本拍微分项清零 */
    p->ff_term = 0.0f;         /* 本拍前馈项清零 */
    p->delta = 0.0f;           /* 输出增量清零 */
    p->started = 0;            /* 重新等待首个测量周期 */
    p->fault = 0;              /* 清除锁存故障 */
    p->saturated = 0;          /* 清除饱和标志 */
}

/*
 * 函数名称：Car_PidInc_init
 * 函数功能：清空增量式 PID 对象，并初始化增益与输出上限。
 * 参数说明：p     —— 待初始化的增量式 PID 对象；
 *           kp    —— 比例增益，范围 [0, 655.35]；
 *           ki    —— 积分增益，范围 [0, 655.35]；
 *           kd    —— 微分增益，范围 [0, 655.35]；
 *           limit —— 输出绝对值上限，范围 (0, 10000]。
 * 返回参数：1 成功；0 参数无效，此时对象已清空并置 fault = 1。
 * 使用示例：
 *     Car_PidInc_Struct pid_inc;
 *     if (!Car_PidInc_init(&pid_inc, 2.0f, 0.12f, 0.0f, 3500.0f))
 *     {
 *         // 参数非法，pid_inc.fault 已置 1
 *     }
 * 备注信息：默认值与位置式一致：beta = 1、微分滤波 0.02 s、
 *           积分上限等于输出上限、前馈系数全为 0。
 */
uint8_t Car_PidInc_init(Car_PidInc_Struct *p, float kp, float ki, float kd,
                        float limit)
{
    memset(p, 0, sizeof(*p));       /* 先整体清零，失败路径不留旧数据 */

    /* 统一检查增益和限幅：必须为有限值、非负且在允许范围内。 */
    if (!number(kp) || !number(ki) || !number(kd) || !number(limit) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        kp > 655.35f || ki > 655.35f || kd > 655.35f ||
        limit <= 0.0f || limit > 10000.0f)
    {
        p->fault = 1;               /* 锁存配置故障 */
        return 0;
    }

    p->kp = kp;                     /* 比例增益 */
    p->ki = ki;                     /* 积分增益 */
    p->kd = kd;                     /* 微分增益 */
    p->output_limit = limit;        /* 输出限幅 */
    p->integral_limit = limit;      /* 积分限幅默认等于输出限幅 */
    p->beta = 1.0f;                 /* 默认使用完整目标 */
    p->derivative_tau_s = 0.02f;    /* 默认微分滤波 20 ms */

    return 1;
}

/*
 * 函数名称：Car_PidInc_configure
 * 函数功能：在已初始化的增量式 PID 上配置目标权重、微分滤波、积分限幅
 *           和前馈系数，参数含义与位置式完全一致。
 * 参数说明：p        —— 已初始化的增量式 PID 对象；
 *           beta     —— 比例项目标权重，范围 [0, 1]；
 *           tau      —— 微分滤波时间常数，范围 [0, 1] 秒，0 表示不滤波；
 *           fraction —— 积分上限比例，范围 (0, 1]；
 *           ks       —— 静摩擦前馈，单位 PWM；
 *           kv       —— 速度前馈系数，单位 PWM/rpm；
 *           ka       —— 加速度前馈系数，单位 PWM/(rpm/s)。
 * 返回参数：1 成功并清除动态历史及故障；0 拒绝修改，原配置和历史保持不变。
 * 使用示例：
 *     Car_PidInc_configure(&pid_inc, 1.0f, 0.02f, 1.0f, 0.0f, 0.0f, 0.0f);
 * 备注信息：所有参数都必须为有限数；校验失败时不会写入任何一个字段。
 */
uint8_t Car_PidInc_configure(Car_PidInc_Struct *p, float beta, float tau,
                             float fraction, float ks, float kv, float ka)
{
    /* 配置参数必须有限且在合理范围内，否则整体拒绝，保持原配置不变。 */
    if (!number(beta) || !number(tau) || !number(fraction) ||
        !number(ks) || !number(kv) || !number(ka) ||
        beta < 0.0f || beta > 1.0f ||
        tau < 0.0f || tau > 1.0f ||
        fraction <= 0.0f || fraction > 1.0f ||
        ks < 0.0f || kv < 0.0f || ka < 0.0f ||
        ks > p->output_limit || kv > 10000.0f || ka > 10000.0f ||
        p->output_limit <= 0.0f)
    {
        return 0;
    }

    p->beta = beta;                                    /* 比例项目标权重 */
    p->derivative_tau_s = tau;                         /* 微分滤波时间常数 */
    p->integral_limit = fraction * p->output_limit;    /* 积分项幅值上限 */
    p->ks = ks;                                        /* 静摩擦前馈 */
    p->kv = kv;                                        /* 速度前馈 */
    p->ka = ka;                                        /* 加速度前馈 */

    /* 配置变化后旧的历史数据已不适用，清空动态状态再恢复控制。 */
    Car_PidInc_reset(p);

    return 1;
}

/*
 * 函数名称：Car_PidInc_update_dt
 * 函数功能：使用实际控制周期执行一次增量式 PID 计算，输出带符号 PWM 数值。
 * 参数说明：p            —— 增量式 PID 对象；
 *           target       —— 目标转速，单位 rpm；
 *           measure      —— 实测转速，单位 rpm；
 *           acceleration —— 参考加速度，单位 rpm/s，用于 Ka 前馈；
 *           dt           —— 实际控制周期，单位秒，允许 [0.0001, 0.1]。
 * 返回参数：[-output_limit, output_limit] 内的 PWM 数值；
 *           输入异常或对象已锁存故障时返回 0。
 * 使用示例：
 *     float pwm = Car_PidInc_update_dt(&pid_inc, 100.0f, speed_rpm,
 *                                      accel_rpm_s, 0.005f);
 *     if (pid_inc.fault) { Car_Fault(&car, CAR_CONTROLLER_FAULT); }
 * 备注信息：增量式公式为
 *               Δu = ΔP + ΔI + ΔD + ΔFF
 *               u[k] = u[k-1] + Δu
 *           其中 ΔP、ΔD、ΔFF 分别是本周期项值与上一周期项值之差，
 *           ΔI 是最终接纳的积分变化量，u 是未限幅的累加值；
 *           执行器限幅仅作用于返回值。各项定义与位置式相同。
 */
float Car_PidInc_update_dt(Car_PidInc_Struct *p, float target, float measure,
                           float acceleration, float dt)
{
    float error;                                           /* 速度误差 = target - measure，rpm */
    float raw_d = 0.0f;                                    /* 未滤波的测量变化率，rpm/s */
    float d_alpha;                                         /* 微分低通系数 */
    float integral_candidate;                              /* 限幅后的积分项候选值 */
    float integral_delta;                                  /* 本次实际接纳的积分增量 */
    float base;                                            /* P/D/FF 总和，用于条件积分判定 */
    float delta;                                           /* 本周期输出增量 Δu */
    float candidate;                                       /* 上一拍输出 + 增量，未限幅 */
    float output;                                          /* 限幅后的最终输出 */

    if (p->fault)                                          /* 已锁存故障，保持输出为 0 */
    {
        return 0.0f;
    }

    /* 与位置式相同的输入、周期和内部数值检查。 */
    if (!number(target) || !number(measure) || !number(acceleration) || !number(dt) ||
        dt < 0.0001f || dt > 0.1f ||
        !number(p->kp) || !number(p->ki) || !number(p->kd) || !number(p->integral) ||
        !number(p->output_limit) || p->output_limit <= 0.0f ||
        p->output_limit > 10000.0f)
    {
        goto invalid;
    }

    error = target - measure;                              /* 完整速度误差，rpm */

    /* 与位置式一致：对测量值求导，首周期不计算差分，避免微分冲击。 */
    if (p->started)
    {
        raw_d = (measure - p->last_measure) / dt;
    }

    p->last_measure = measure;                             /* 保存本拍测量值 */
    p->started = 1;                                        /* 从下一拍开始可以求微分 */

    /* 一阶低通抑制编码器量化噪声，系数随实际周期调整。 */
    d_alpha = dt / (p->derivative_tau_s + dt);
    p->derivative += d_alpha * (raw_d - p->derivative);

    p->p_term = p->kp * (p->beta * target - measure);      /* 比例项绝对项值 */
    p->d_term = -p->kd * 0.005f * p->derivative;           /* 微分项绝对项值 */
    p->ff_term = p->kv * target + p->ka * acceleration;    /* 前馈项绝对项值 */

    if (target > 1.0f)                                     /* 正转时叠加正向静摩擦补偿 */
    {
        p->ff_term += p->ks;
    }
    else if (target < -1.0f)                               /* 反转时叠加负向静摩擦补偿 */
    {
        p->ff_term -= p->ks;
    }

    /*
     * 积分项仍显式累加，但增量式只把“本次变化量”计入输出增量。
     * 先限制积分项自身幅值，避免积分贡献单独过大。
     */
    integral_candidate = clip(p->integral + p->ki * error * (dt / 0.005f),
                              p->integral_limit);

    if (!number(p->p_term) || !number(p->d_term) ||
        !number(p->ff_term) || !number(integral_candidate))
    {
        goto invalid;
    }

    /* 增量式核心：使用未限幅的累加基准，保留饱和时 P/D/FF 的完整历史。 */
    integral_delta = integral_candidate - p->integral;
    base = p->p_term + p->d_term + p->ff_term;
    delta = (p->p_term - p->last_p_term)
          + integral_delta
          + (p->d_term - p->last_d_term)
          + (p->ff_term - p->last_ff_term);

    /* 与位置式使用相同的绝对项判断积分，避免递推舍入误差改变临界点决策。 */
    candidate = base + integral_candidate;

    if (!number(delta) || !number(candidate))
    {
        goto invalid;
    }

    /*
     * 条件积分抗饱和（与位置式同思路）：
     * 总输出已经顶到限幅且误差仍指向同一方向时，不接收本次积分增量，
     * 防止退出饱和后积分残留导致反向超调。
     */
    if (!((candidate > p->output_limit && error > 0.0f) ||
          (candidate < -p->output_limit && error < 0.0f)))
    {
        p->integral = integral_candidate;                  /* 接收本次积分增量 */
    }
    else
    {
        /* 拒绝积分时，输出与递推历史也必须撤去这部分增量。 */
        delta = (p->p_term - p->last_p_term)
              + (p->d_term - p->last_d_term)
              + (p->ff_term - p->last_ff_term);
    }

    candidate = p->unclamped_output + delta;
    if (!number(delta) || !number(candidate))
    {
        goto invalid;
    }

    p->saturated = (candidate > p->output_limit || candidate < -p->output_limit);
    p->unclamped_output = candidate;                        /* 保存完整递推基准 */
    output = clip(candidate, p->output_limit);              /* 仅对执行器输出限幅 */
    p->last_output = output;

    /* 保存本拍各项值，供下一周期作差得到增量。 */
    p->last_p_term = p->p_term;
    p->last_d_term = p->d_term;
    p->last_ff_term = p->ff_term;
    p->delta = delta;

    return output;

invalid:
    /* 与位置式一致：锁存故障、清零输出，由上层转入故障保护。 */
    p->fault = 1;                                          /* 锁存数值故障 */
    p->last_output = 0.0f;                                 /* 立即撤去输出 */

    return 0.0f;
}

/*
 * 函数名称：Car_PidInc_update
 * 函数功能：固定 5 ms 控制周期的增量式 PID 兼容入口。
 * 参数说明：p       —— 增量式 PID 对象；
 *           target  —— 目标转速，单位 rpm；
 *           measure —— 实测转速，单位 rpm。
 * 返回参数：同 Car_PidInc_update_dt()。
 * 使用示例：
 *     float pwm = Car_PidInc_update(&pid_inc, 100.0f, speed_rpm);
 * 备注信息：参考加速度按 0 处理，适用于没有规划器的简单调用场景。
 */
float Car_PidInc_update(Car_PidInc_Struct *p, float target, float measure)
{
    return Car_PidInc_update_dt(p, target, measure, 0.0f, 0.005f);
}
