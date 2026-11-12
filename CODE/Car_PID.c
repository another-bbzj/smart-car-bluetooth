/*********************************************************************************************************************
* 文件名称          Car_PID.c
* 功能说明          双轮车速度环位置式与增量式 PID 的实现，属于"控制算法"模块，不直接操作任何外设。
*
*                   本文件包含的函数：
*                   · Car_Pid_init    —— 位置式：写入 PID 参数与输出限幅，并清空历史状态
*                   · Car_Pid_reset   —— 位置式：只清积分、上次实测值和微分启动标志
*                   · Car_Pid_update  —— 位置式：计算一拍 PID 输出，含微分先行与条件积分抗饱和
*                   · Car_IncPid_init   —— 增量式：写入参数与限幅，并清空累加输出
*                   · Car_IncPid_reset  —— 增量式：只清累加输出与历史状态
*                   · Car_IncPid_update —— 增量式：ΔPWM 累加后限幅，含饱和时冻结积分增量
*
*                   位置式与增量式共用同一套限幅工具 Car_ClampFloat；
*                   未饱和时增量式是位置式的逐拍差分形式，第一拍输出与位置式一致。
*
*                   调用关系：Car_Core 在 5ms 控制拍中按轮调用 Car_Pid_update（默认），
*                   或调用 Car_IncPid_update（CAR_PID_USE_INCREMENTAL=1）；
*                   故障、急停、目标归零时调用对应的 reset 防止旧状态造成冲击。
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            first version
* 2026-11-12                            统一格式与中文注释，限幅改用 Car_Utils
* 2026-11-12                            新增增量式 Car_IncPid_*，默认不接入控制路径
********************************************************************************************************************/
#include "Car_PID.h"
#include "Car_Utils.h"

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Pid_init
 * 功能说明    初始化一个速度环 PID 对象：写入 Kp/Ki/Kd、输出限幅，并清零积分与历史状态。
 * 用法        上电初始化或修改参数时调用；之后每个控制周期调用 Car_Pid_update。
 * 参数说明    pid          —— 待初始化的 PID 对象指针
 *             kp/ki/kd     —— 比例、积分、微分参数（本项目按 5ms 离散周期整定）
 *             output_limit —— 输出限幅绝对值（本车为 PWM 计数 3500）
 * 返回说明    无
 * 示例        Car_Pid_Struct pid;
 *             Car_Pid_init(&pid, 2.0f, 0.12f, 0.0f, 3500.0f);
 * 备注        内部会调用 Car_Pid_reset，因此重复初始化等于"换参数并清状态"。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Pid_init(Car_Pid_Struct *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp           = kp;                                  /* 比例系数：调用方按实车整定 */
    pid->ki           = ki;                                  /* 积分系数：调用方按实车整定 */
    pid->kd           = kd;                                  /* 微分系数：调用方按实车整定 */
    pid->output_limit = output_limit;                        /* 输出限幅：来自 CAR_PWM_LIMIT */
    Car_Pid_reset(pid);                                      /* 新参数从零状态开始，避免继承旧积分 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Pid_reset
 * 功能说明    清空 PID 的积分项、上一次实测值与启动标志，但保留 Kp/Ki/Kd 与输出限幅不变。
 * 用法        在启动、停止、急停、故障复位、目标归零等需要"从零开始"的时机调用。
 * 参数说明    pid —— 待复位的 PID 对象指针
 * 返回说明    无
 * 示例        Car_Pid_reset(&car.pid[0]);   // 左轮速度环重新起步，避免旧积分造成冲击
 * 备注        复位后第一次 update 不计算微分项，防止历史值缺失导致 D 项冲击。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Pid_reset(Car_Pid_Struct *pid)
{
    pid->integral     = 0.0f;                                /* 积分项清零 */
    pid->last_measure = 0.0f;                                /* 上次实测值清零，等待重新采样 */
    pid->started      = 0U;                                  /* 标记为尚未启动，本次不算微分 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Pid_update
 * 功能说明    执行一次位置式 PID 计算：P 用当前误差，I 累加并做抗饱和处理，D 用实测值变化量，
 *             最终输出限幅在 ±output_limit 内。
 * 用法        每个控制周期（本项目 5ms）对每个轮子各调用一次；目标与实测单位必须一致（rpm）。
 * 参数说明    pid     —— PID 对象指针（需先 Car_Pid_init）
 *             target  —— 目标转速，单位 rpm
 *             measure —— 实测转速，单位 rpm
 * 返回说明    限幅后的控制量（PWM 计数），正负号表示期望的转动方向
 * 示例        float pwm = Car_Pid_update(&car.pid[0], 80.0f, car.speed[0]);
 * 备注        1) 微分项使用 -Kd × (本次实测 - 上次实测)，目标突变不会产生微分冲击；
 *             2) 输出顶到限幅且误差同向时，积分最多只增加到"输出刚好顶满"，避免积分饱和；
 *             3) 目标与实测均为 0 时建议由调用方直接输出 0 并复位，而不是依赖本函数。
 *-------------------------------------------------------------------------------------------------------------------*/
float Car_Pid_update(Car_Pid_Struct *pid, float target, float measure)
{
    float error = target - measure;                          /* 当前误差 e = 目标 - 实测 */
    float p     = pid->kp * error;                           /* 比例项 P */
    float d     = 0.0f;                                      /* 微分项 D，首次调用时保持 0 */
    float i_new;                                             /* 本拍"照常累加"后的积分候选值 */
    float room;                                              /* 把输出顶到限幅时积分允许占用的余量 */

    /* 微分先行：用实测值的变化量，避免目标突变时产生微分冲击。
     * 第一次调用还没有上一次实测值，因此先跳过微分。 */
    if (pid->started != 0U) {
        d = -pid->kd * (measure - pid->last_measure);
    }
    pid->last_measure = measure;                             /* 保存本次实测值，供下一拍求微分 */
    pid->started      = 1U;                                  /* 标记微分状态已经建立 */

    /* 积分先按常规累加，并限制在 ±output_limit 内，防止积分项本身无限增长。 */
    i_new = Car_ClampFloat(pid->integral + pid->ki * error, pid->output_limit);

    /* 抗积分饱和（条件积分）：
     * 照常累加会让输出超过正限幅、而且误差为正（还想继续加大输出）时，
     * 积分最多只加到"输出刚好等于正限幅"；负方向同理。
     * 其余情况积分照常更新。误差一旦反向，积分马上就能正常减小，不会"卡住"。 */
    if ((p + i_new + d > pid->output_limit) && (error > 0.0f)) {
        room = pid->output_limit - (p + d);                  /* 让输出刚好等于正限幅的积分值 */
        if (room > pid->integral) {
            pid->integral = room;
        }
    } else if ((p + i_new + d < -pid->output_limit) && (error < 0.0f)) {
        room = -pid->output_limit - (p + d);                 /* 让输出刚好等于负限幅的积分值 */
        if (room < pid->integral) {
            pid->integral = room;
        }
    } else {
        pid->integral = i_new;
    }

    return Car_ClampFloat(p + pid->integral + d, pid->output_limit); /* 最终输出限幅 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_IncPid_init
 * 功能说明    写入增量式 PID 的 Kp/Ki/Kd 与绝对输出限幅，并清空累加输出和历史状态。
 * 用法        上电初始化或停车修改参数时调用；之后每个控制周期调用 Car_IncPid_update。
 * 参数说明    pid          —— 待初始化的增量式 PID 对象指针
 *             kp/ki/kd     —— 比例、积分、微分参数（按 5ms 离散周期整定）
 *             output_limit —— 绝对输出限幅（PWM 计数）
 * 返回说明    无
 * 示例        Car_IncPid_Struct inc_pid;
 *             Car_IncPid_init(&inc_pid, 2.0f, 0.12f, 0.0f, 3500.0f);
 * 备注        内部调用 Car_IncPid_reset，重复初始化等于"换参数并清状态"。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_IncPid_init(Car_IncPid_Struct *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp           = kp;                                  /* 比例系数：调用方按实车整定 */
    pid->ki           = ki;                                  /* 积分系数：调用方按实车整定 */
    pid->kd           = kd;                                  /* 微分系数：调用方按实车整定 */
    pid->output_limit = output_limit;                        /* 绝对输出限幅：来自 CAR_PWM_LIMIT */
    Car_IncPid_reset(pid);                                   /* 新参数从零输出、零历史开始 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_IncPid_reset
 * 功能说明    清空累加输出、误差历史、实测历史和启动标志，但保留 Kp/Ki/Kd 与输出限幅。
 * 用法        启动、停止、急停、故障复位、目标归零等需要"从零开始"的时机调用。
 * 参数说明    pid —— 待复位的增量式 PID 对象指针
 * 返回说明    无
 * 示例        Car_IncPid_reset(&inc_pid);
 * 备注        复位后第一拍重新走"P+I、D=0"的初始化分支。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_IncPid_reset(Car_IncPid_Struct *pid)
{
    pid->output        = 0.0f;                               /* 累加输出清零 */
    pid->error_prev    = 0.0f;                               /* 上一次误差清零 */
    pid->measure_prev  = 0.0f;                               /* 上一次实测值清零 */
    pid->measure_prev2 = 0.0f;                               /* 上上次实测值清零 */
    pid->started       = 0U;                                 /* 标记为尚未建立历史 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_IncPid_update
 * 功能说明    执行一拍增量式 PID：ΔPWM = Kp·Δe + Ki·e - Kd·Δ²y，累加后限幅在 ±output_limit。
 * 用法        每个控制周期对每个轮子各调用一次；返回绝对 PWM 计数，可直接交给 CarMotor_SetOutput。
 * 参数说明    pid     —— 增量式 PID 对象指针（需先 Car_IncPid_init）
 *             target  —— 目标转速（rpm）
 *             measure —— 实测转速（rpm）
 * 返回说明    限幅后的绝对控制量（PWM 计数），正负号表示期望方向
 * 示例        float pwm = Car_IncPid_update(&inc_pid, 80.0f, speed);
 * 备注        1) 微分使用实测值的二阶差分，目标阶跃不产生 D 冲击；
 *             2) 输出已经顶到限幅且误差还在同方向推时，本拍冻结积分增量，防止饱和累积；
 *             3) 未饱和时与 Car_Pid_update 的位置式结果逐拍一致，便于台架对比。
 *-------------------------------------------------------------------------------------------------------------------*/
float Car_IncPid_update(Car_IncPid_Struct *pid, float target, float measure)
{
    float error = target - measure;                          /* 当前误差 e[k] */
    float delta_p;                                           /* 比例增量 Kp·(e[k]-e[k-1]) */
    float delta_i;                                           /* 积分增量 Ki·e[k] */
    float delta_d;                                           /* 微分增量 -Kd·(y[k]-2y[k-1]+y[k-2]) */
    float delta;                                             /* 本拍总增量 */

    if (pid->started == 0U) {
        /* 第一拍：与位置式首次输出保持一致（P + I，D = 0），
         * 同时用当前误差/实测值建立历史，避免下一拍出现虚假的 P/D 冲击。 */
        pid->output        = Car_ClampFloat((pid->kp + pid->ki) * error, pid->output_limit);
        pid->error_prev    = error;
        pid->measure_prev  = measure;
        pid->measure_prev2 = measure;
        pid->started       = 1U;
        return pid->output;
    }

    delta_p = pid->kp * (error - pid->error_prev);           /* 比例项：误差的变化量 */
    delta_i = pid->ki * error;                               /* 积分项：本拍误差 */
    delta_d = -pid->kd * (measure - 2.0f * pid->measure_prev + pid->measure_prev2);

    /* 条件积分：输出已经顶到限幅、误差还在同方向推时，本拍不再累加积分增量。
     * 这样松开目标或误差反向后，输出不会被旧的积分"顶住"。 */
    if (((pid->output >= pid->output_limit) && (error > 0.0f)) ||
        ((pid->output <= -pid->output_limit) && (error < 0.0f))) {
        delta_i = 0.0f;
    }

    delta = delta_p + delta_i + delta_d;                     /* 本拍总增量 */
    pid->output = Car_ClampFloat(pid->output + delta, pid->output_limit);

    /* 历史后移：本拍值成为下一拍的"上一次"和"上上次"。 */
    pid->error_prev    = error;
    pid->measure_prev2 = pid->measure_prev;
    pid->measure_prev  = measure;

    return pid->output;
}
