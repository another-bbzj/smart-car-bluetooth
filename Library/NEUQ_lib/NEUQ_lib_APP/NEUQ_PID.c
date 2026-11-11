/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_PID
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        zxz        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include <math.h>

pid_value yaw_anglespeed_wheel , yaw_angle_wheel , yaw_speed_wheel ;
IncrementalPID stra_speed_wheel_1,stra_speed_wheel_2;

void pid_parameters_setting(pid_value* theptr, float kp, float ki, float kd)
{
    theptr->Kp=kp;
    theptr->Ki=ki;
    theptr->Kd=kd;
    theptr->Ek=0;
    theptr->Ek1=0;
    theptr->Ek2=0;
    theptr->Ek_total=0;
    theptr->OUT=0;
}

void Ipid_parameters_setting(IncrementalPID* theptr, float kp, float ki, float kd)
{
    theptr->Kp=kp;
    theptr->Ki=ki;
    theptr->Kd=kd;
    theptr->error[0] = 0;
    theptr->error[1] = 0;
    theptr->error[2] = 0;
    theptr->max_delta_pwm = 6000;
}

float Unclipped_Positional_PID(pid_value *pid, float SetValue, float ActualValue, float Max_I)
{
    pid->Ek = SetValue - ActualValue;
    pid->Ek_total += pid->Ek;
    pid->Ek_total = fmaxf(fminf(pid->Ek_total, Max_I), -Max_I);
    float derivative = pid->Ek - pid->Ek1;
    pid->OUT = (pid->Kp * pid->Ek)
             + (pid->Ki * pid->Ek_total)
             + (pid->Kd * derivative);
    pid->Ek1 = pid->Ek;
    return pid->OUT;
}


float IncrementalPID_Update(IncrementalPID *pid, float tar_speed, float cur_speed) {

    pid->error[0] = tar_speed - cur_speed;

    float delta_pwm = pid->Kp * (pid->error[0] - pid->error[1])
                    + pid->Ki * pid->error[0]
                    + pid->Kd * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);

    if (delta_pwm > pid->max_delta_pwm) delta_pwm = pid->max_delta_pwm;
    else if (delta_pwm < -pid->max_delta_pwm) delta_pwm = -pid->max_delta_pwm;


    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    return delta_pwm; 
}


void pid_parameters_init(void)
{
    pid_parameters_setting(&yaw_anglespeed_wheel, 0,0,0);
    pid_parameters_setting(&yaw_angle_wheel, 0,0,0 );

    pid_parameters_setting(&yaw_anglespeed_wheel, 0,0,0);

    Ipid_parameters_setting(&stra_speed_wheel_1,0,0,0);
    Ipid_parameters_setting(&stra_speed_wheel_2,0,0,0);
}
