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
* 文件名称          NEUQ_MPU6050
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        zxz        first version
********************************************************************************************************************/
#ifndef _NEUQ_MPU6050_H_
#define _NEUQ_MPU6050_H_

#include "stm32f4xx_hal.h"

#define MPU6050_IIC_METHOD_HARD_IIC     (1)
#define MPU6050_IIC_METHOD_SOFT_IIC     (2)
#define MPU6050_IIC_METHOD      (MPU6050_IIC_METHOD_HARD_IIC)

#if MPU6050_IIC_METHOD==MPU6050_IIC_METHOD_HARD_IIC

#define MPU6050_HARD_IIC_USE    (Hard_IIC_3)
#define MPU6050_HARD_IIC_SPEED_USE    (400*1000)

#endif

#define MPU6050_ADDR         (0x68 << 1)

// MPU6050 Register Addresses
#define MPU6050_SELF_TEST_X    0x0D
#define MPU6050_SELF_TEST_Y    0x0E
#define MPU6050_SELF_TEST_Z    0x0F
#define MPU6050_SELF_TEST_A    0x10
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_FIFO_EN        0x23
#define MPU6050_INT_ENABLE     0x38
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_XOUT_L   0x3C
#define MPU6050_ACCEL_YOUT_H   0x3D
#define MPU6050_ACCEL_YOUT_L   0x3E
#define MPU6050_ACCEL_ZOUT_H   0x3F
#define MPU6050_ACCEL_ZOUT_L   0x40
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_TEMP_OUT_L     0x42
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_GYRO_XOUT_L    0x44
#define MPU6050_GYRO_YOUT_H    0x45
#define MPU6050_GYRO_YOUT_L    0x46
#define MPU6050_GYRO_ZOUT_H    0x47
#define MPU6050_GYRO_ZOUT_L    0x48
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_WHO_AM_I       0x75

// Config values (examples)
#define MPU6050_WAKEUP            0x00
#define MPU6050_DEFAULT_SMPLRT    0x00  // 1kHz / (1+7) = 125Hz
#define MPU6050_DEFAULT_CONFIG    0x03  // DLPF 44Hz
#define MPU6050_ACCEL_FS_2G       0x00
#define MPU6050_ACCEL_FS_4G       0x08
#define MPU6050_ACCEL_FS_8G       0x10
#define MPU6050_ACCEL_FS_16G      0x18
#define MPU6050_GYRO_FS_250       0x00
#define MPU6050_GYRO_FS_500       0x08
#define MPU6050_GYRO_FS_1000      0x10
#define MPU6050_GYRO_FS_2000      0x18
#define MPU6050_FIFO_ENABLE       0x00
#define MPU6050_INT_ENABLE_NONE   0x00


// 用户可选配置（初始化前设置）
#define MPU6050_ACCEL_SCALE     (MPU6050_ACCEL_FS_8G)   // 支持 2/4/8/16 (g)
#define MPU6050_GYRO_SCALE      (MPU6050_GYRO_FS_2000) // 支持 250/500/1000/2000 (°/s)


typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    float Temperature;
} MPU6050_Data;
extern int16_t mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z;            // 三轴陀螺仪数据   gyro (陀螺仪)
extern int16_t mpu6050_acc_x, mpu6050_acc_y , mpu6050_acc_z ;               // 三轴加速度计数据 acc  (accelerometer 加速度计)
extern MPU6050_Data MPU6050_Data_get;

uint8_t NEUQ_MPU6050_init(void);
void NEUQ_MPU6050_read_data(void);
float MPU6050_acc_transition (int16_t acc_value);
float MPU6050_gyro_transition(int16_t gyro_value);
#endif 
