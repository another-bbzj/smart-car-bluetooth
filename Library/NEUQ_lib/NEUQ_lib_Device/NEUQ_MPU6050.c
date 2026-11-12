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
* 2025-09-22        张瑞       修复连续reset导致从机iic死锁的问题
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

MPU6050_Data MPU6050_Data_get={0};
int16_t mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;            // 三轴陀螺仪数据   gyro (陀螺仪)
int16_t mpu6050_acc_x = 0, mpu6050_acc_y = 0, mpu6050_acc_z = 0;               // 三轴加速度计数据 acc  (accelerometer 加速度计)
int16_t temp = 0;                                                      // 温度数据 temp (temperature 温度)
//==============================================================================================
// 函数名称 : NEUQ_MPU6050_init
// 函数功能 : 初始化MPU6050传感器
// 输入参数 : 无
// 输出参数 : 无
// 返 回 值 : 1 成功 0 失败
// 使用示例 : NEUQ_MPU6050_init() // 初始化MPU6050传感器
//==============================================================================================
static inline void MPU6050_IIC_Reset(void);
uint8_t NEUQ_MPU6050_init(void){
#if MPU6050_IIC_METHOD==MPU6050_IIC_METHOD_HARD_IIC
    MPU6050_IIC_Reset(); // 强行拉高scl要求从机释放总线，避免总线死锁
    NEUQ_Delay_ms(100);
    NEUQ_Hard_IIC_init(MPU6050_HARD_IIC_USE,MPU6050_HARD_IIC_SPEED_USE);//启动iic通信
    NEUQ_Delay_ms(100);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, 0x6B, 0x80);
    NEUQ_Delay_ms(10);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_PWR_MGMT_1,0x00);//唤醒传感器
    NEUQ_Delay_ms(10);

    uint8_t data=0;
    if(NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_WHO_AM_I,&data,1)!=1){//检查传感器是否在线
        return 0;
    }

    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_SMPLRT_DIV, MPU6050_DEFAULT_SMPLRT);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_CONFIG, MPU6050_DEFAULT_CONFIG);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_SCALE);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_GYRO_CONFIG, MPU6050_GYRO_SCALE);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_FIFO_EN, MPU6050_FIFO_ENABLE);
    NEUQ_Hard_IIC_write_8bit_reg(MPU6050_HARD_IIC_USE,MPU6050_ADDR,MPU6050_INT_ENABLE, MPU6050_INT_ENABLE_NONE);

    return 1;
#endif
}

//==============================================================================================
// 函数名称 : NEUQ_MPU6050_read_data
// 函数功能 : 读取MPU6050传感器数据
// 输入参数 : 无
// 输出参数 : 无
// 返 回 值 : 无
// 使用示例 : NEUQ_MPU6050_read_data() // 读取MPU6050传感器数据，数据保存在 MPU6050_Data_get 结构体中
//==============================================================================================
void NEUQ_MPU6050_read_data(void){
    uint8_t raw[14];
#if MPU6050_IIC_METHOD==MPU6050_IIC_METHOD_HARD_IIC
    NEUQ_Hard_IIC_read_8bit_reg(MPU6050_HARD_IIC_USE, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, raw, 14);
#endif

    mpu6050_acc_x = (int16_t)(raw[0] << 8 | raw[1]);
    mpu6050_acc_y = (int16_t)(raw[2] << 8 | raw[3]);
    mpu6050_acc_z = (int16_t)(raw[4] << 8 | raw[5]);
    temp = (int16_t)(raw[6] << 8 | raw[7]);
    mpu6050_gyro_x = (int16_t)(raw[8] << 8 | raw[9]);
    mpu6050_gyro_y = (int16_t)(raw[10] << 8 | raw[11]);
    mpu6050_gyro_z = (int16_t)(raw[12] << 8 | raw[13]);

    MPU6050_Data_get.Temperature = temp / 340.0f + 36.53f;
}

//==============================================================================================
// 函数名称 : MPU6050_acc_transition
// 函数功能 : 加速度计数据转换为 g
// 输入参数 : acc_value : 原始加速度计数据
// 输出参数 : 无
// 返 回 值 : 转换后的加速度值，单位 g
// 使用示例 : float ax = MPU6050_acc_transition(mpu6050_acc_x); // 将原始加速度计数据转换为 g
//==============================================================================================
float MPU6050_acc_transition (int16_t acc_value)
{
    float accel_sensitivity;
    switch (MPU6050_ACCEL_SCALE) {
        case MPU6050_ACCEL_FS_2G:  accel_sensitivity = 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  accel_sensitivity = 8192.0f; break;
        case MPU6050_ACCEL_FS_8G:  accel_sensitivity = 4096.0f; break;
        case MPU6050_ACCEL_FS_16G: accel_sensitivity = 2048.0f; break;
        default: accel_sensitivity = 4096.0f; break; // 默认 ±8g
    }
    return acc_value / accel_sensitivity;
}

//==============================================================================================
// 函数名称 : MPU6050_gyro_transition
// 函数功能 : 陀螺仪数据转换为 °/s
// 输入参数 : gyro_value : 原始陀螺仪数据
// 输出参数 : 无
// 返 回 值 : 转换后的陀螺仪值，单位 °/s
// 使用示例 : float gx = MPU6050_gyro_transition(mpu6050_gyro_x); // 将原始陀螺仪数据转换为 °/s
//==============================================================================================
float MPU6050_gyro_transition (int16_t gyro_value)
{
    float gyro_sensitivity;
    switch (MPU6050_GYRO_SCALE) {
        case MPU6050_GYRO_FS_250:  gyro_sensitivity = 131.0f; break;
        case MPU6050_GYRO_FS_500:  gyro_sensitivity = 65.5f; break;
        case MPU6050_GYRO_FS_1000: gyro_sensitivity = 32.8f; break;
        case MPU6050_GYRO_FS_2000: gyro_sensitivity = 16.4f; break;
        default:   gyro_sensitivity = 65.5f; break; // 默认 ±500°/s
    }
    // float deg_to_rad = 3.1415926f / 180.0f;
    return gyro_value / gyro_sensitivity;
}

#define I2C_SCL_GPIO_Port   GPIOA
#define I2C_SCL_Pin         GPIO_PIN_8
#define I2C_SDA_GPIO_Port   GPIOC
#define I2C_SDA_Pin         GPIO_PIN_9
static inline void gpio_od_high(GPIO_TypeDef* port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin = pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;   // 板上有上拉则 NOPULL
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); // 释放=写高
}
static inline void gpio_od_low(GPIO_TypeDef* port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin = pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // 拉低
}

static inline uint8_t pin_read(GPIO_TypeDef* port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}
static inline void short_delay(void) { for(volatile int i=0;i<200;i++); } // ~us 级

static inline void MPU6050_IIC_Reset(void) {
    // 2) 接管引脚为开漏输出并先“释放为高”
    gpio_od_high(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
    gpio_od_high(I2C_SDA_GPIO_Port, I2C_SDA_Pin);

    // 3) 若 SDA 低，则对 SCL 打最多 9 个脉冲促其释放
    if (!pin_read(I2C_SDA_GPIO_Port, I2C_SDA_Pin)) {
        for (int i = 0; i < 9 && !pin_read(I2C_SDA_GPIO_Port, I2C_SDA_Pin); i++) {
            // SCL 释放为高，等待它真的为高（兼容从机 clock stretch）
            gpio_od_high(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
            uint32_t to = 2000;
            while (!pin_read(I2C_SCL_GPIO_Port, I2C_SCL_Pin) && to--) { /* wait */ }

            // 再拉低形成一个完整时钟
            short_delay();
            gpio_od_low(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
            short_delay();
        }
    }

    // 4) 构造 STOP：保证 SCL 高期间，让 SDA 由低 -> 高
    gpio_od_low(I2C_SDA_GPIO_Port, I2C_SDA_Pin);  // 先拉 SDA 低
    short_delay();
    gpio_od_high(I2C_SCL_GPIO_Port, I2C_SCL_Pin); // SCL 释放为高
    uint32_t to = 2000;
    while (!pin_read(I2C_SCL_GPIO_Port, I2C_SCL_Pin) && to--) { /* wait */ }
    short_delay();
    gpio_od_high(I2C_SDA_GPIO_Port, I2C_SDA_Pin); // SDA 释放为高 = STOP
    short_delay();
}
