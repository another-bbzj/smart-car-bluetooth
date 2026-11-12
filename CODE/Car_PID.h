#ifndef CAR_PID_H
#define CAR_PID_H
#include <stdint.h>
/* 并联速度 PID：为兼容蓝牙调参工具，Ki/Kd 沿用历史 5 ms 离散增益，
 * I += Ki*error*dt/0.005，D = -Kd*0.005*d(measure)/dt。
 * 目标/反馈单位 rpm，加速度单位 rpm/s，输出和前馈项为 PWM 数值。
 * 仅停车时配置参数；本模块为纯算法，运行状态由上层控制。 */
/* 所有接口要求 p 非空且独占访问；先 init，按需 configure，再周期调用 update。 */
typedef struct {
    /* kp/ki/kd：控制增益；output_limit：输出绝对值上限；
     * integral：已累积的积分输出；last_measure：上次实测转速（rpm）。 */
    float kp,ki,kd,output_limit,integral,last_measure;
    /* beta：比例项目标权重；derivative_tau_s：微分滤波时间常数（秒）；
     * integral_limit：积分限幅；ks/kv/ka：静摩擦、速度、加速度前馈系数。 */
    float beta,derivative_tau_s,integral_limit,ks,kv,ka;
    /* derivative：滤波后的测量变化率（rpm/s）；p_term/d_term/ff_term 为各项输出，
     * last_output 为最终限幅值，可用于遥测；积分项直接读取 integral。 */
    float derivative,p_term,d_term,ff_term,last_output;
    /* started 表示已有上次测量；fault 锁存计算异常；saturated 表示输出被限幅。 */
    uint8_t started,fault,saturated;
} Car_Pid_Struct;
/**
 * @brief 清空对象并初始化增益和输出上限。
 * @param kp 比例增益，有限数，范围 [0,655.35]。
 * @param ki 积分增益，有限数，范围 [0,655.35]。
 * @param kd 微分增益，有限数，范围 [0,655.35]。
 * @param limit 输出上限，有限数，范围 (0,10000]。
 * @return 1 成功；0 参数无效，此时对象已清空并置 fault=1。
 * @note 默认 beta=1、微分滤波 0.02 s、积分上限等于输出上限，前馈为零。
 */
uint8_t Car_Pid_init(Car_Pid_Struct *p,float kp,float ki,float kd,float limit);
/**
 * @brief 在已初始化的对象上配置目标权重、微分滤波和前馈。
 * @param beta 比例项目标权重，范围 [0,1]。
 * @param derivative_tau_s 微分滤波时间常数，范围 [0,1] 秒，0 表示不滤波。
 * @param integral_fraction (0,1]，积分上限 = 此比例 × output_limit。
 * @param ks 静摩擦前馈，范围 [0,output_limit]。
 * @param kv 速度前馈系数，范围 [0,10000]。
 * @param ka 加速度前馈系数，范围 [0,10000]；所有配置参数须为有限数。
 * @return 1 成功并清除动态历史及故障；0 拒绝且保留原配置与历史。
 */
uint8_t Car_Pid_configure(Car_Pid_Struct *p,float beta,float derivative_tau_s,
                         float integral_fraction,float ks,float kv,float ka);
/**
 * @brief 保留配置，清空积分、微分历史、最近输出和状态标志。
 * @note reset 会清除 fault；配置本身异常时须先修正参数，再恢复控制。
 */
void Car_Pid_reset(Car_Pid_Struct *p);
/**
 * @brief 计算一次速度控制输出，正负号表示驱动方向。
 * @param target 参考转速（rpm），须为有限数。
 * @param measure 实测转速（rpm），须为有限数。
 * @param target_acceleration 参考加速度（rpm/s），用于 Ka 前馈。
 * @param dt_s 实际周期（秒），允许 [0.0001,0.1]；底盘核心采用更窄的范围。
 * @return [-output_limit,output_limit] 内的 PWM 数值；故障时返回 0。
 * @note 零返回值也可能是正常输出，须结合 p->fault 判断；故障会锁存。
 */
float Car_Pid_update_dt(Car_Pid_Struct *p,float target,float measure,
                       float target_acceleration,float dt_s);
/**
 * @brief 固定 5 ms、参考加速度为 0 的兼容入口，其余语义同 update_dt。
 */
float Car_Pid_update(Car_Pid_Struct *p,float target,float measure);
#endif
