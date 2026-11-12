/*********************************************************************************************************************
* 文件名称          Car_Motor.c
* 功能说明          双路直流电机的外设驱动，属于"执行器"模块，只依赖 NEUQ 库的 GPIO/PWM 接口。
*
*                   本文件包含的函数：
*                   · CarMotor_Init      —— 初始化左右轮方向脚与 PWM，输出初始为 0
*                   · CarMotor_SetOutput —— 按带符号的 PWM 请求设置方向与占空比，换向时插入零输出
*
*                   本模块不读取编码器、不计算 PID、不做保护判断；
*                   上层 Car_Core 决定"输出多少"，本模块只负责"如何安全地写到 H 桥"。
*
* 修改记录
* 日期              作者                备注
* 2026-11-12                            从 Car_Hardware.c 拆出电机外设模块
********************************************************************************************************************/
#include "Car_Motor.h"
#include "Car_Config.h"
#include "NEUQ_lib_include.h"

#define CAR_MOTOR_COUNT  2U     /* 本车电机数量：左、右各一路 */

/* 左右轮 PWM 通道：下标 0 左轮，1 右轮 */
static const PWM_Ports_ENUM motor_pwm[CAR_MOTOR_COUNT] = {
    CAR_LEFT_PWM,
    CAR_RIGHT_PWM
};

/* 左右轮方向 GPIO：下标 0 左轮，1 右轮 */
static const GPIO_Ports_ENUM motor_dir[CAR_MOTOR_COUNT] = {
    CAR_LEFT_DIR,
    CAR_RIGHT_DIR
};

/* 左右轮"正转"对应的方向脚电平：1 高电平，0 低电平，来自 Car_Config.h */
static const uint8_t motor_forward_level[CAR_MOTOR_COUNT] = {
    CAR_LEFT_FORWARD_HIGH,
    CAR_RIGHT_FORWARD_HIGH
};

/* 当前已生效的转向：0 正转，1 反转；用于判断是否需要换向 */
static uint8_t motor_reverse[CAR_MOTOR_COUNT];

/* 换向等待标志：1 表示本拍已先输出 0，下一拍再真正换向并输出 */
static uint8_t motor_waiting[CAR_MOTOR_COUNT];

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarMotor_Init
 * 功能说明    初始化左右轮方向脚为推挽输出（先输出正转电平），并把 PWM 占空比初始化为 0。
 * 用法        上电初始化阶段调用一次，必须早于 CarMotor_SetOutput。
 * 参数说明    无
 * 返回说明    无
 * 示例        CarMotor_Init();
 * 备注        方向脚先写正转电平，PWM 再写 0，保证初始化期间电机不会突然转动。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarMotor_Init(void)
{
    unsigned i;                                             /* 电机下标：0 左轮，1 右轮 */

    for (i = 0U; i < CAR_MOTOR_COUNT; i++) {
        NEUQ_GPIO_init(motor_dir[i],
                       GPIO_Mode_Output_PushPull,
                       (motor_forward_level[i] != 0U) ? GPIO_High : GPIO_Low);
        NEUQ_PWM_init(motor_pwm[i], CAR_PWM_HZ, 0U);
        motor_reverse[i] = 0U;                              /* 初始方向按正转处理 */
        motor_waiting[i] = 0U;                              /* 没有待处理的换向 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarMotor_SetOutput
 * 功能说明    把一路带符号的 PWM 请求写到硬件：处理方向脚与占空比，换向时先插入一拍零输出。
 * 用法        由 5ms 控制拍在 Car_Tick 之后对左右轮各调用一次。
 * 参数说明    index —— 轮子下标：CAR_MOTOR_LEFT(0) 或 CAR_MOTOR_RIGHT(1)
 *             value —— 速度环输出（PWM 计数），正负表示方向，0 表示停止
 * 返回说明    无
 * 示例        CarMotor_SetOutput(CAR_MOTOR_LEFT, car.output[0]);
 * 备注        1) value == 0 时立即写 0 并清除换向等待标志；
 *             2) 方向需要改变时，先写 0 并置等待标志，下一拍再真正换向并输出；
 *             3) 方向极性由 CAR_LEFT_FORWARD_HIGH / CAR_RIGHT_FORWARD_HIGH 决定；
 *             4) index 越界时直接返回，避免数组越界。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarMotor_SetOutput(uint8_t index, float value)
{
    uint8_t next_reverse;                                   /* 本次请求的方向：0 正转，1 反转 */
    float   magnitude;                                      /* PWM 占空比的绝对值 */
    uint32_t duty;                                          /* 取整后的 PWM 计数 */
    uint8_t  level;                                         /* 方向脚最终要输出的电平 */

    if (index >= CAR_MOTOR_COUNT) {
        return;                                             /* 防御：只允许左右两路 */
    }

    next_reverse = (value < 0.0f) ? 1U : 0U;

    if (value == 0.0f) {
        NEUQ_PWM_set_duty(motor_pwm[index], 0U);            /* 零请求：立即关断输出 */
        motor_waiting[index] = 0U;
        return;
    }

    if (next_reverse != motor_reverse[index]) {
        if (motor_waiting[index] == 0U) {
            NEUQ_PWM_set_duty(motor_pwm[index], 0U);        /* 换向第一拍：先输出 0 */
            motor_waiting[index] = 1U;
            return;                                         /* 下一拍再真正换向 */
        }
        /* 换向第二拍：先改方向脚，再在函数末尾输出占空比。
         * 正转时输出 CAR_*_FORWARD_HIGH，反转时输出它的反相电平。 */
        if (next_reverse != 0U) {
            level = (motor_forward_level[index] != 0U) ? GPIO_Low : GPIO_High;
        } else {
            level = (motor_forward_level[index] != 0U) ? GPIO_High : GPIO_Low;
        }
        NEUQ_GPIO_write(motor_dir[index], (GPIO_Level)level);
        motor_reverse[index] = next_reverse;
    }

    motor_waiting[index] = 0U;

    magnitude = (value < 0.0f) ? -value : value;            /* 占空比只取大小，方向由方向脚决定 */
    duty = (uint32_t)(magnitude + 0.5f);                    /* 四舍五入到整数 PWM 计数 */
    NEUQ_PWM_set_duty(motor_pwm[index], duty);
}
