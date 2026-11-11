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
* 文件名称          NEUQ_IMU_Solve
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        zxz        first version
********************************************************************************************************************/

/*
 * 无需闭角度环
 * 偏航角度由上位机反馈，或者直接闭角速度环
 * 如有余力，可以进行串级pid闭环。。。。
 */
#include "NEUQ_lib_include.h"
#include <stdbool.h>
#include <math.h>

anglespeed_struct yaw_anglespeed ;
float yaw_anglespeed_offset = 0;
float alpha = 0.2f;

//陀螺仪零偏标定，上电静止3000次采样取平均值
void mpu_solve_zero(void)
{
    int32_t sum_z = 0;
    for(int i=0;i<3000;i++)
    {
        NEUQ_MPU6050_read_data();
        sum_z += mpu6050_gyro_z;
        NEUQ_Delay_ms(2);
    }
    yaw_anglespeed_offset = (float)sum_z/3000;
}

//计算偏航角速度，一般用于闭角速度环
float mpu_get_yaw_Z(void)
{
    NEUQ_MPU6050_read_data();
    yaw_anglespeed.z = yaw_anglespeed.z*(1-alpha) + mpu6050_gyro_z*alpha;
    return MPU6050_gyro_transition(yaw_anglespeed.z-yaw_anglespeed_offset);
}
