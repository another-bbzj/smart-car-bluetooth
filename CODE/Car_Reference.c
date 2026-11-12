/*
 * ============================================================================
 * 文件名称：Car_Reference.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件实现双轮速度参考规划器。它把蓝牙下发的“阶跃”目标转速，
 * 转换为速度、加速度和加加速度都有上限的平滑参考曲线：
 *
 *     蓝牙目标 -> 限幅 -> 五次平滑曲线 -> 参考转速/参考加速度
 *
 * 控制目的：
 *   - 起步和换速时不让 PID 直接追阶跃，减小电流冲击和机械冲击；
 *   - 两个车轮共享同一条曲线进度，保持轮速比例，减少跑偏；
 *   - 参考加速度同时送给速度 PID 的 Ka 前馈。
 *
 * 三个阶段
 * ----------------------------------------------------------------------------
 *   phase = 0：保持目标值，不再移动；
 *   phase = 1：沿五次多项式 s(t) = 10t³ - 15t⁴ + 6t⁵ 从起点移动到目标；
 *   phase = 2：目标中途改变时，先用受限加加速度把当前参考加速度
 *              线性降到 0，再进入 phase = 1 重新规划。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static float absolute(float x)
 *       —— 求浮点数绝对值。
 *   static float max(float x, float y)
 *       —— 求两个浮点数中的较大值。
 *   static float clip(float x, float lim)
 *       —— 把浮点数限制到 [-lim, +lim]。
 *   void CarReference_Reset(CarReference *r)
 *       —— 保留三项限制，清空双轮参考值、加速度、目标和阶段状态。
 *   void CarReference_Init(CarReference *r, float speed, float acceleration,
 *                          float jerk)
 *       —— 清空对象并保存速度、加速度和加加速度限制。
 *   static void begin_profile(CarReference *r)
 *       —— 按当前起点和目标计算五次曲线持续时间，进入 phase = 1。
 *   void CarReference_Update(CarReference *r, const float target[2], float dt)
 *       —— 推进一个控制周期的参考曲线，结果写入 value 和 acceleration。
 *
 * 单位约定
 * ----------------------------------------------------------------------------
 *   参考转速、目标转速：rpm；
 *   参考加速度：rpm/s；
 *   加加速度限制：rpm/s²；
 *   时间 dt、elapsed、duration：秒。
 *
 * 重要说明
 * ----------------------------------------------------------------------------
 *   本模块只生成参考值，不代表车辆实际已经制动或停下。
 *   需要紧急停止时，上层应调用 CarReference_Reset() 直接清零参考值，
 *   而不是把目标设为 0 后等待曲线自然结束。
 * ============================================================================
 */

#include "Car_Reference.h"

#include <math.h>
#include <string.h>

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：absolute
 * 函数功能：求浮点数的绝对值。
 * 参数说明：x —— 任意有限浮点数。
 * 返回参数：x 的绝对值。
 * 使用示例：
 *     float distance = absolute(r->goal[0] - r->value[0]);
 * 备注信息：用于计算目标变化量和参考加速度大小。
 */
static float absolute(float x)
{
    return (x < 0.0f) ? -x : x;
}

/*
 * 函数名称：max
 * 函数功能：求两个浮点数中的较大值。
 * 参数说明：x、y —— 待比较的两个有限浮点数。
 * 返回参数：较大的那个值。
 * 使用示例：
 *     r->duration = max(0.005f, required_time);
 * 备注信息：用于取两轮变化量和两种约束时间中的较大者。
 */
static float max(float x, float y)
{
    return (x > y) ? x : y;
}

/*
 * 函数名称：clip
 * 函数功能：把浮点数限制到 [-lim, +lim]。
 * 参数说明：x   —— 待限幅的数值；
 *           lim —— 正数形式的绝对值上限。
 * 返回参数：限幅后的数值。
 * 使用示例：
 *     goal[i] = clip(target[i], r->speed_limit);
 * 备注信息：调用者需保证 lim 为正有限数。
 */
static float clip(float x, float lim)
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

/* ========================================================================== */
/*                              规划器接口                                     */
/* ========================================================================== */

/*
 * 函数名称：CarReference_Reset
 * 函数功能：保留三项限制，立即清空双轮参考值、加速度和阶段状态。
 * 参数说明：r —— 规划器对象，调用者需保证非空。
 * 返回参数：无。
 * 使用示例：
 *     CarReference_Reset(&car.profile);   // 急停时立即清零参考
 * 备注信息：不读取轮速、不驱动电机；由上层决定何时应用该复位结果。
 */
void CarReference_Reset(CarReference *r)
{
    unsigned i;                             /* 双轮循环下标，0 左轮 / 1 右轮 */

    for (i = 0; i < 2; i++)
    {
        r->value[i] = 0.0f;                 /* 参考转速清零 */
        r->acceleration[i] = 0.0f;          /* 参考加速度清零 */
        r->goal[i] = 0.0f;                  /* 目标转速清零 */
        r->start[i] = 0.0f;                 /* 阶段起点清零 */
        r->brake_acceleration[i] = 0.0f;    /* 阶段 2 初始加速度清零 */
    }

    r->phase = 0;                           /* 回到保持状态 */
    r->elapsed = 0.0f;                      /* 阶段已用时间清零 */
    r->duration = 0.0f;                     /* 阶段总时间清零 */
}

/*
 * 函数名称：CarReference_Init
 * 函数功能：清空规划器并保存速度、加速度和加加速度限制。
 * 参数说明：r             —— 规划器对象；
 *           speed         —— 参考转速上限，单位 rpm；
 *           acceleration  —— 参考加速度上限，单位 rpm/s；
 *           jerk          —— 加加速度上限，单位 rpm/s²。
 * 返回参数：无。
 * 使用示例：
 *     CarReference_Init(&car.profile, 180.0f, 200.0f, 800.0f);
 * 备注信息：本函数只保存限制，不返回参数校验结果；
 *           调用者应保证三项限制为有限正数，否则 Update 会直接返回。
 */
void CarReference_Init(CarReference *r, float speed, float acceleration,
                       float jerk)
{
    memset(r, 0, sizeof(*r));                /* 清空全部状态 */

    r->speed_limit = speed;                  /* 参考转速上限 */
    r->acceleration_limit = acceleration;    /* 参考加速度上限 */
    r->jerk_limit = jerk;                    /* 加加速度上限 */
}

/*
 * 函数名称：begin_profile
 * 函数功能：根据当前起点和目标，计算五次曲线的持续时间并进入变速阶段。
 * 参数说明：r —— 规划器对象，r->start、r->goal 已设置。
 * 返回参数：无。
 * 使用示例：
 *     // 仅由 CarReference_Update 在目标变化且无剩余加速度时调用
 *     begin_profile(r);
 * 备注信息：
 *   - 以变化量较大的车轮为基准，使两轮同时到达各自目标；
 *   - 五次平滑函数一阶导峰值为 1.875，二阶导绝对值峰值为 10/√3，
 *     分别推导“加速度约束所需时间”和“加加速度约束所需时间”，取较大者；
 *   - 持续时间至少为 5 ms，避免除零和过于陡峭的曲线。
 */
static void begin_profile(CarReference *r)
{
    float distance;               /* 两轮中较大的目标变化量，rpm */

    distance = max(absolute(r->goal[0] - r->value[0]),
                   absolute(r->goal[1] - r->value[1]));

    r->start[0] = r->value[0];    /* 左轮阶段起点 */
    r->start[1] = r->value[1];    /* 右轮阶段起点 */
    r->elapsed = 0.0f;            /* 新阶段从 0 开始计时 */

    r->duration = max(0.005f,
                      max(1.875f * distance / r->acceleration_limit,
                          sqrtf(5.773503f * distance / r->jerk_limit)));

    r->phase = 1;                 /* 进入五次曲线变速阶段 */
}

/*
 * 函数名称：CarReference_Update
 * 函数功能：推进一个控制周期的双轮参考曲线。
 * 参数说明：r      —— 规划器对象；
 *           target —— 两个目标转速（rpm），下标 0 左轮、1 右轮；
 *                     输入先检查 [-100000, 100000]，再逐轮限制到 speed_limit；
 *           dt     —— 本次推进时间，单位秒，允许 (0, 0.1]。
 * 返回参数：无；结果写入 r->value 和 r->acceleration。
 * 使用示例：
 *     float target[2] = { 100.0f, 100.0f };
 *     CarReference_Update(&car.profile, target, 0.005f);
 *     // car.profile.value[] 即为本拍参考转速
 * 备注信息：周期、目标或限制检查未通过时直接返回，保留已有规划状态。
 *           目标改变时可能先经过 phase = 2；目标为零也按曲线变化，
 *           急停应调用 CarReference_Reset()。
 */
void CarReference_Update(CarReference *r, const float target[2], float dt)
{
    unsigned i;                                                   /* 双轮循环下标 */
    float goal[2];                                                /* 限幅后的新目标，rpm */
    float peak;                                                   /* 双轮参考加速度绝对值的较大者 */
    float remaining;                                              /* 本周期剩余可推进时间，秒 */
    float step;                                                   /* 阶段 2 本次实际推进时间，秒 */
    float t0;                                                     /* 阶段 1 归一化进度，0~1 */
    float t1;                                                     /* 阶段 2 归一化进度，0~1 */
    float progress;                                               /* 五次曲线位置进度，0~1 */
    float rate;                                                   /* 五次曲线对时间的导数，1/s */

    /* 周期或限制异常时不做任何修改，防止用错误参数规划出异常曲线。 */
    if (!(dt > 0.0f && dt <= 0.1f &&
          r->speed_limit > 0.0f &&
          r->acceleration_limit > 0.0f &&
          r->jerk_limit > 0.0f))
    {
        return;
    }

    for (i = 0; i < 2; i++)
    {
        /* 目标必须在合理范围内，避免异常值参与后续除法。 */
        if (!(target[i] >= -100000.0f && target[i] <= 100000.0f))
        {
            return;
        }

        goal[i] = clip(target[i], r->speed_limit);                /* 逐轮限速 */
    }

    /* 目标发生变化时，重新决定是先“刹车”还是直接重新规划。 */
    if (goal[0] != r->goal[0] || goal[1] != r->goal[1])
    {
        r->goal[0] = goal[0];                                     /* 更新左轮目标 */
        r->goal[1] = goal[1];                                     /* 更新右轮目标 */

        peak = max(absolute(r->acceleration[0]),
                   absolute(r->acceleration[1]));

        /*
         * 变速途中改目标时，若当前还有参考加速度，
         * 先以受限加加速度把它线性降到 0，再重新规划。
         */
        if (peak > 0.00001f)
        {
            r->phase = 2;                                         /* 进入加速度归零阶段 */
            r->elapsed = 0.0f;
            r->duration = peak / r->jerk_limit;

            for (i = 0; i < 2; i++)
            {
                r->start[i] = r->value[i];                        /* 记录起点 */
                r->brake_acceleration[i] = r->acceleration[i];    /* 记录初始加速度 */
            }
        }
        else
        {
            begin_profile(r);                                     /* 无剩余加速度，直接重新规划 */
        }
    }

    /* 一个采样周期可能跨过阶段边界，剩余时间继续推进下一阶段。 */
    remaining = dt;

    if (r->phase == 2)
    {
        step = r->duration - r->elapsed;                          /* 本阶段剩余时间 */

        if (step > remaining)
        {
            step = remaining;                                     /* 本周期只推进剩余时间 */
        }

        r->elapsed += step;                                       /* 累计阶段已用时间 */
        remaining -= step;                                        /* 更新本周期剩余时间 */
        t1 = r->elapsed / r->duration;

        for (i = 0; i < 2; i++)
        {
            /*
             * 令 a(t) = a0 × (1 - t/T)，积分得到参考速度。
             * 此阶段减小的是加速度，参考速度仍可能继续沿原方向变化，
             * 不会直接跳到新目标。
             */
            r->value[i] = r->start[i] +
                          r->brake_acceleration[i] * r->duration *
                          (t1 - 0.5f * t1 * t1);
            r->acceleration[i] = r->brake_acceleration[i] * (1.0f - t1);
        }

        /* 加速度已经降到 0，按新目标开始五次曲线规划。 */
        if (r->elapsed >= r->duration)
        {
            r->acceleration[0] = 0.0f;
            r->acceleration[1] = 0.0f;
            begin_profile(r);
        }
    }

    if (r->phase == 1 && remaining > 0.0f)
    {
        r->elapsed += remaining;                                  /* 推进五次曲线阶段 */
        t0 = r->elapsed / r->duration;

        if (t0 > 1.0f)
        {
            t0 = 1.0f;                                            /* 防止浮点误差超过 1 */
        }

        /* s(t) = 10t³ - 15t⁴ + 6t⁵；rate 为对实际时间的导数。 */
        progress = t0 * t0 * t0 * (10.0f + t0 * (-15.0f + 6.0f * t0));
        rate = 30.0f * t0 * t0 * (1.0f - t0) * (1.0f - t0) / r->duration;

        for (i = 0; i < 2; i++)
        {
            r->value[i] = r->start[i] +
                          (r->goal[i] - r->start[i]) * progress;
            r->acceleration[i] = (r->goal[i] - r->start[i]) * rate;
        }

        /*
         * 终点显式赋目标并清零加速度，消除浮点累积误差；
         * 之后 phase = 0，保持该参考值不变。
         */
        if (r->elapsed >= r->duration)
        {
            r->phase = 0;                                         /* 进入保持阶段 */

            for (i = 0; i < 2; i++)
            {
                r->value[i] = r->goal[i];                         /* 精确到达目标 */
                r->acceleration[i] = 0.0f;                        /* 终点加速度归零 */
            }
        }
    }
}
