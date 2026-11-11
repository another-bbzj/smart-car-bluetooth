/*********************************************************************************************************************
* 文件名称          Car_PID
* 功能说明          双轮车：速度环位置式 PID（实现），说明见 Car_PID.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "Car_PID.h"

// 把 x 限制在 [-limit, +limit] 之间
static float limit_abs(float x, float limit)
{
    if (x > limit)  return limit;
    if (x < -limit) return -limit;
    return x;
}

void Car_Pid_init(Car_Pid_Struct *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->output_limit = output_limit;
    Car_Pid_reset(pid);
}

void Car_Pid_reset(Car_Pid_Struct *pid)
{
    pid->integral     = 0.0f;
    pid->last_measure = 0.0f;
    pid->started      = 0;
}

float Car_Pid_update(Car_Pid_Struct *pid, float target, float measure)
{
    float error = target - measure;
    float p = pid->kp * error;
    float d = 0.0f;
    float i_new;

    // 微分：用实际值的变化量（第一次调用时还没有上一次的值，先不算）
    if (pid->started) {
        d = -pid->kd * (measure - pid->last_measure);
    }
    pid->last_measure = measure;
    pid->started      = 1;

    // 积分：先算出"这一拍如果照常累加"的积分值（积分本身也不能超过输出限幅）
    i_new = limit_abs(pid->integral + pid->ki * error, pid->output_limit);

    // 抗积分饱和（条件积分）：
    // 照常累加会让输出超过正限幅、而且误差为正（还想继续加大输出）时，积分最多只加到"输出刚好等于限幅"；
    // 负方向同理。其余情况积分照常更新。误差一旦反向，积分马上就能正常减小，不会"卡住"
    if (p + i_new + d > pid->output_limit && error > 0.0f) {
        float room = pid->output_limit - (p + d);           // 让输出刚好等于正限幅的积分值
        if (room > pid->integral) {
            pid->integral = room;
        }
    }
    else if (p + i_new + d < -pid->output_limit && error < 0.0f) {
        float room = -pid->output_limit - (p + d);          // 让输出刚好等于负限幅的积分值
        if (room < pid->integral) {
            pid->integral = room;
        }
    }
    else {
        pid->integral = i_new;
    }

    return limit_abs(p + pid->integral + d, pid->output_limit);
}
