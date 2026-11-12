#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*********************************************************************************************************************
* 文件名称          Car_Config.h
* 功能说明          整车硬件与控制的集中配置：串口编号/波特率、电机 PWM 与方向引脚、编码器参数、
*                   控制周期、保护阈值、PID 默认参数以及通信缓冲参数。
*
*                   本文件只放"可调常量"，不放变量定义和函数实现，便于换板或换电机时集中修改。
*                   修改任何引脚或极性后，必须重新架空核对方向与测速符号。
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            整理为集中配置
* 2026-11-12                            补充中文注释，拆分通信/驱动参数
********************************************************************************************************************/

/* ==================================== 蓝牙与视觉串口 ==================================== */
/* 参考原理图：驱动通道 1 接左轮，通道 2 接右轮。
 * 首次上电必须架空车轮，核对实际轮位与转向后再落地运行。 */
#define CAR_UART_INDEX            3U                    /* 蓝牙串口编号：库 UART_3 对应 STM32 UART4（PC10/PC11） */
#define CAR_UART_BAUD             UART_BAUD_9600        /* 蓝牙波特率：JDY-31 出厂默认 9600，8N1 */

/* 香橙派视觉链路：USART3，PB10 为 TX、PB11 为 RX，与蓝牙链路互相独立。
 * 该路只接收视觉观测，不允许通过它启动电机或修改 PWM。 */
#define CAR_VISION_UART_INDEX     2U                    /* 视觉串口编号：库 UART_2 对应 STM32 USART3 */
#define CAR_VISION_UART_BAUD      UART_BAUD_115200      /* 视觉波特率：与香橙派发送端保持一致 */

#if CAR_UART_INDEX != 3 || CAR_VISION_UART_INDEX != 2
#error 当前板级适配只支持蓝牙走 UART4（库 UART_3）、视觉走 USART3（库 UART_2）
#endif

/* 串口接收中断优先级：数值越小优先级越高。视觉低于蓝牙，保证蓝牙停车命令优先得到处理。 */
#define CAR_BLUETOOTH_RX_PRIORITY 3U                    /* 蓝牙 UART4 接收中断优先级 */
#define CAR_VISION_RX_PRIORITY    4U                    /* 视觉 USART3 接收中断优先级 */

/* ==================================== 电机与驱动 ==================================== */
#define CAR_MOTOR_NOLOAD_RPM      1000.0f               /* 电机输出轴空载转速（rpm），仅作参数参考，不是命令限幅 */
#define CAR_FLASH_CS              PB0                    /* 板载 W25Q Flash 片选：先拉高，避免与 PA5/6/7 复用冲突 */
#define CAR_LEFT_PWM              TIM8_CH1_PC6           /* 左轮 PWM：TIM8_CH1，对应 PC6 */
#define CAR_LEFT_DIR              PC7                    /* 左轮方向：普通 GPIO，对应 PC7 */
#define CAR_RIGHT_PWM             TIM2_CH3_PA2           /* 右轮 PWM：TIM2_CH3，对应 PA2 */
#define CAR_RIGHT_DIR             PA5                    /* 右轮方向：普通 GPIO，对应 PA5 */
#define CAR_LEFT_FORWARD_HIGH     1                      /* 左轮正转时方向脚电平：1 高电平，0 低电平 */
#define CAR_RIGHT_FORWARD_HIGH    1                      /* 右轮正转时方向脚电平：1 高电平，0 低电平 */
#define CAR_LEFT_ENCODER_INVERT   1                      /* 左轮编码器符号取反：1 取反，0 保持原始方向 */
#define CAR_RIGHT_ENCODER_INVERT  1                      /* 右轮编码器符号取反：1 取反，0 保持原始方向 */

#define CAR_PWM_HZ                8000U                  /* 电机 PWM 频率（Hz），需在驱动板允许范围内 */
#define CAR_PWM_LIMIT             3500.0f                /* PWM 输出限幅（计数）：PID 与手推动作都不得超过 */
#define CAR_RPM_LIMIT             180.0f                 /* 轮速目标限幅（rpm）：低速调试值，不是电机额定能力 */
#define CAR_RAMP_RPM_PER_TICK     1.0f                   /* 每个 5ms 控制拍允许的参考转速增量（rpm/拍） */

/* ==================================== 控制周期与保护 ==================================== */
#define CAR_PERIOD_MS             5U                     /* 控制周期（ms）：TIM6 中断节拍，所有"每拍"参数都基于它 */
#define CAR_TELEMETRY_TICKS       20U                    /* 遥测分频：20 拍 × 5ms = 100ms 发送一帧 VOFA+ 数据 */
#define CAR_TELEMETRY_CHANNELS    16U                    /* VOFA+ 每帧通道数：与 My_Vofa 的最大通道数一致 */
#define CAR_BLUETOOTH_DRAIN_MAX   32U                    /* 每个控制拍最多解析的蓝牙字节数，避免拖长中断 */
#define CAR_VISION_DRAIN_MAX      64U                    /* 每个控制拍最多解析的视觉字节数，放在电机输出之后执行 */

#define CAR_LINK_TIMEOUT_MS       500U                   /* 链路续租超时：运行中 500ms 无有效命令则通信故障停车 */
#define CAR_FRAME_TIMEOUT_MS      100U                   /* 单帧字节间隔超时：超过 100ms 未收完整帧则丢弃残帧 */
#define CAR_STALL_TIMEOUT_MS      1500U                  /* 堵转判定时间：输出足够但实测转速过低持续 1.5s 则故障 */
#define CAR_DIRECTION_TIMEOUT_MS  1000U                  /* 方向异常判定时间：目标与实测反向持续 1s 则故障 */

/* ==================================== 速度环 PID 默认参数 ==================================== */
/* PID 结构选择：0 = 位置式 Car_Pid_*（默认，保持原有控制行为）；
 * 1 = 增量式 Car_IncPid_*（与位置式共用同一组参数和输出限幅）。
 * 选择增量式时，Car_Core 会额外分配并初始化 inc_pid[2]；
 * 两套实现的参数和限幅一致，但切换后必须重新台架验证动态、堵转和换向。 */
#ifndef CAR_PID_USE_INCREMENTAL
#define CAR_PID_USE_INCREMENTAL   0
#endif

#define CAR_DEFAULT_KP            2.0f                   /* 默认比例系数，可在停车状态下通过命令在线修改 */
#define CAR_DEFAULT_KI            0.12f                  /* 默认积分系数，按 5ms 控制周期整定 */
#define CAR_DEFAULT_KD            0.0f                   /* 默认微分系数，初始不使用微分 */

/* ==================================== 编码器参数 ==================================== */
/* 初始台架假设：11 线霍尔编码器、四倍频、减速比 10，即输出轴每转 440 计数。
 * JGB37-520 型号与 1000rpm 空载转速不能反推出真实 PPR 和减速比；
 * 换电机后必须实测输出轴一圈的计数并修改本宏，否则测速与里程都不准确。 */
#define CAR_ENCODER_CPR           (11.0f * 4.0f * 10.0f)

#endif /* CAR_CONFIG_H */
