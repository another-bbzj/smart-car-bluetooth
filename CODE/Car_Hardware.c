/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */
/*********************************************************************************************************************
* 文件名称          Car_Hardware.c
* 功能说明          整车板级编排：负责各外设模块的初始化顺序、5ms 控制拍的执行顺序和 VOFA+ 遥测打包。
*                   本文件不直接操作 GPIO/PWM/定时器寄存器，具体外设分别由以下模块实现：
*                   · Car_Motor    —— 电机方向与 PWM 输出
*                   · Car_Encoder  —— 编码器采样
*                   · Car_Link     —— 蓝牙/视觉串口接收与快照
*                   · Car_Core     —— 状态机、命令协议、速度环与保护
*                   · My_Vofa      —— VOFA+ JustFloat 遥测发送
*
*                   本文件包含的函数：
*                   · Car_PIT_Handler —— 5ms 控制拍：采样、解析、控制、输出、视觉、遥测
*                   · Car_Main        —— 上电初始化并进入 VOFA+ 发送主循环
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            first version
* 2026-11-12                            拆分外设模块，本文件只保留板级编排与遥测
********************************************************************************************************************/
#include "Car_Hardware.h"
#include "Car_Config.h"
#include "Car_Core.h"
#include "Car_Encoder.h"
#include "Car_Motor.h"
#include "My_Vofa.h"
#include "NEUQ_lib_include.h"

/* 整车控制核心：保存状态机、两轮目标、实测转速、PID 和保护计时 */
static Car_Core car;

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_PIT_Handler
 * 功能说明    5ms 定时中断主流程：读取编码器增量、处理串口故障与命令解析、执行速度控制与电机输出，
 *             批量处理视觉数据，并按 100ms 周期通过 VOFA+ JustFloat 回传 16 通道遥测。
 * 用法        由库的 TIM6 定时中断回调 NEUQ_PIT_1_CALLBACK 调用，必须严格保持 5ms 周期。
 * 参数说明    无
 * 返回说明    无
 * 示例        void NEUQ_PIT_1_CALLBACK(void) { Car_PIT_Handler(); }
 * 备注        1) 执行顺序固定为"采样 -> 保护/解析 -> 控制 -> 输出 -> 视觉 -> 遥测"，
 *                保证电机输出始终基于本拍最新数据；
 *             2) 串口故障时清空蓝牙 FIFO 并锁定 CAR_RX_FAULT；
 *             3) 视觉处理放在电机输出之后，且每拍最多处理 64 字节，避免拖长控制中断；
 *             4) 遥测 20 拍（100ms）发一帧，9600 波特率下占用约 680 字节/秒。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_PIT_Handler(void)
{
    static unsigned telemetry_ticks;                        /* 遥测分频计数：每 20 拍（100ms）发送一帧 */
    int32_t left_count;                                     /* 左轮本拍编码器增量（已按配置取符号） */
    int32_t right_count;                                    /* 右轮本拍编码器增量（已按配置取符号） */
    float   channels[CAR_TELEMETRY_CHANNELS];               /* VOFA+ 遥测通道数组，顺序见下方赋值 */

    /* 1. 采样：读取两轮编码器增量，并按配置统一正方向。 */
    left_count  = CarEncoder_ReadDelta(CAR_ENCODER_LEFT);
    right_count = CarEncoder_ReadDelta(CAR_ENCODER_RIGHT);
    if (CAR_LEFT_ENCODER_INVERT != 0) {
        left_count = -left_count;
    }
    if (CAR_RIGHT_ENCODER_INVERT != 0) {
        right_count = -right_count;
    }

    /* 2. 保护/解析：先处理蓝牙接收故障或命令，再进入控制计算。 */
    CarLink_ProcessBluetooth(&car);

    /* 3. 控制：测速换算、目标斜坡、速度环与保护判断。 */
    Car_Tick(&car, left_count, right_count);

    /* 4. 输出：把本拍速度环结果写到左右电机。 */
    CarMotor_SetOutput(CAR_MOTOR_LEFT, car.output[0]);
    CarMotor_SetOutput(CAR_MOTOR_RIGHT, car.output[1]);

    /* 5. 视觉：放在电机输出之后处理，限制每拍字节数，避免影响控制实时性；
     *    只有停车状态才允许视觉端发送 HELLO 建立新会话。 */
    CarLink_ProcessVision((car.state == CAR_STOP) ? 1U : 0U);

    /* 6. 遥测：每 20 拍（100ms）打包一次 16 通道数据。 */
    telemetry_ticks++;
    if (telemetry_ticks >= CAR_TELEMETRY_TICKS) {
        telemetry_ticks = 0U;

        channels[0]  = (float)car.state;                    /* 通道 0：状态机状态 0/1/2 */
        channels[1]  = (float)car.fault;                    /* 通道 1：故障码 */
        channels[2]  = car.reference[0];                    /* 通道 2：左轮斜坡后参考转速（rpm） */
        channels[3]  = car.speed[0];                        /* 通道 3：左轮实测转速（rpm） */
        channels[4]  = car.reference[1];                    /* 通道 4：右轮斜坡后参考转速（rpm） */
        channels[5]  = car.speed[1];                        /* 通道 5：右轮实测转速（rpm） */
        channels[6]  = car.output[0] / 100.0f;              /* 通道 6：左轮 PWM 输出（缩小 100 倍显示） */
        channels[7]  = car.output[1] / 100.0f;              /* 通道 7：右轮 PWM 输出（缩小 100 倍显示） */
        channels[8]  = (float)car.result;                   /* 通道 8：最近一条命令结果码 */
        channels[9]  = (float)car.frames;                   /* 通道 9：累计合法命令帧数 */
        channels[10] = car.pid[0].kp;                       /* 通道 10：左轮 Kp */
        channels[11] = car.pid[0].ki;                       /* 通道 11：左轮 Ki */
        channels[12] = car.pid[0].kd;                       /* 通道 12：左轮 Kd */
        channels[13] = car.pid[1].kp;                       /* 通道 13：右轮 Kp */
        channels[14] = car.pid[1].ki;                       /* 通道 14：右轮 Ki */
        channels[15] = car.pid[1].kd;                       /* 通道 15：右轮 Kd */

        My_Vofa_push(channels, CAR_TELEMETRY_CHANNELS);     /* 整帧放入发送缓冲，9600 下约 680 字节/秒 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Main
 * 功能说明    整车初始化并进入主循环：初始化控制核心与通信链路、Flash 片选、电机、编码器、
 *             两路串口及中断、5ms 定时器，然后循环执行 VOFA+ 数据发送。
 * 用法        由 User_Main 调用一次，之后不再返回；必须先确认接线和配置再上电。
 * 参数说明    无
 * 返回说明    无（正常情况下不返回）
 * 示例        void User_Main(void) { Car_Main(); }
 * 备注        1) 初始化顺序有依赖：先建控制核心和接收 FIFO，再配置引脚与外设；
 *             2) Flash 片选初始化为高，避免与 PA5/6/7 复用冲突；
 *             3) 上电后处于 STOP，必须收到合法 ARM 且再收到轮速命令才会运动；
 *             4) 主循环只做 My_Vofa_poll，实际控制全部在 5ms 中断中完成。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Main(void)
{
    /* 1. 软件状态：控制核心与通信链路（视觉上下文、两条接收 FIFO）。 */
    Car_Init(&car);
    CarLink_Init();

    /* 2. 板载 Flash 片选：先拉高，避免后面复用 PA5/PA6/PA7 时误选 Flash。 */
    NEUQ_GPIO_init(CAR_FLASH_CS, GPIO_Mode_Output_PushPull, GPIO_High);

    /* 3. 执行器与传感器：电机方向/PWM、两路正交编码器。 */
    CarMotor_Init();
    CarEncoder_Init();

    /* 4. 蓝牙串口：UART4（库 UART_3），并开启单字节接收中断。 */
    NEUQ_Uart_init((UART_DEV_ENUM)CAR_UART_INDEX, CAR_UART_BAUD);
    My_Vofa_init(CAR_UART_INDEX);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_UART_INDEX,
                                  CAR_BLUETOOTH_RX_PRIORITY);

    /* 5. 视觉串口：USART3（库 UART_2），与蓝牙链路独立。 */
    NEUQ_Uart_init((UART_DEV_ENUM)CAR_VISION_UART_INDEX, CAR_VISION_UART_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_VISION_UART_INDEX,
                                  CAR_VISION_RX_PRIORITY);

    /* 6. 5ms 定时器：启动后由 NEUQ_PIT_1_CALLBACK 周期进入 Car_PIT_Handler。 */
    NEUQ_PIT_init_ms(PIT_CH1, CAR_PERIOD_MS);

    /* 7. 主循环：只负责把遥测缓冲慢慢发到串口，实时控制都在定时中断中完成。 */
    while (1) {
        My_Vofa_poll();
    }
}
