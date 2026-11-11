/*********************************************************************************************************************
* 文件名称          Car_PID
* 功能说明          双轮车：速度环位置式 PID
*
*                   误差   e = 目标转速 - 实际转速
*                   比例   P = Kp × e                         误差越大，出力越大
*                   积分   I = I + Ki × e（每个控制周期加一次）   只要还有误差就一直累加，负责消除稳态误差、抵抗负载
*                   微分   D = -Kd × (本次实际 - 上次实际)       抑制转速变化过快；用"实际值的变化"而不是"误差的变化"，
*                                                              目标突然跳变时不会产生一个很大的冲击
*                   输出   = P + I + D，限制在 ±output_limit（PWM 计数）
*
*                   抗积分饱和：输出已经顶到限幅、误差还在往同一方向推时，积分最多只长到"输出刚好顶满"为止，
*                   不再继续累加。否则电机被堵住或加速跟不上时，积分会越积越大，松开后转速会冲过头很多。
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef CAR_PID_H
#define CAR_PID_H

#include <stdint.h>

typedef struct {
    float   kp, ki, kd;             // PID 参数
    float   output_limit;           // 输出限幅（绝对值）
    float   integral;               // 积分项 I
    float   last_measure;           // 上一次的实际值（算微分用）
    uint8_t started;                // 0 表示还没有"上一次"，这一次先不算微分
} Car_Pid_Struct;

void  Car_Pid_init(Car_Pid_Struct *pid, float kp, float ki, float kd, float output_limit);
void  Car_Pid_reset(Car_Pid_Struct *pid);                                 // 清空积分和历史（参数不变），启动/停止时调用
float Car_Pid_update(Car_Pid_Struct *pid, float target, float measure);   // 每个控制周期调用一次，返回输出

#endif
