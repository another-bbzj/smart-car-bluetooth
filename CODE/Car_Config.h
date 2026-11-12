#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* Reference schematics: channel 1 is assigned LEFT, channel 2 RIGHT.
 * Verify physical wheel placement and signs with the chassis lifted. */

/* ---------- 串口 ---------- */
#define CAR_UART_INDEX          3          /* 蓝牙遥控串口编号：UART4（NEUQ_Uart 按 0 起索引） */
#define CAR_UART_BAUD           UART_BAUD_9600    /* 蓝牙串口波特率：9600 */
/* Orange Pi vision: USART3, PB10 TX / PB11 RX, separate from Bluetooth. */
#define CAR_VISION_UART_INDEX   2          /* 视觉串口编号：USART3（PB10 TX / PB11 RX），与蓝牙独立 */
#define CAR_VISION_UART_BAUD    UART_BAUD_115200  /* 视觉串口波特率：115200 */
#if CAR_UART_INDEX != 3 || CAR_VISION_UART_INDEX != 2
#error This board adapter currently supports Bluetooth UART4 and vision USART3 only
#endif

/* ---------- 电机与引脚 ---------- */
#define CAR_MOTOR_NOLOAD_RPM    1000.0f    /* 电机输出轴空载转速 rpm，仅用于换算，不是指令限幅 */
#define CAR_FLASH_CS            PB0        /* 板载 W25Q Flash 片选脚；复用 PA5/6/7 前需拉高释放 */
#define CAR_LEFT_PWM            TIM8_CH1_PC6   /* 左轮 PWM 输出：TIM8 通道 1（PC6） */
#define CAR_LEFT_DIR            PC7        /* 左轮方向控制脚 */
#define CAR_RIGHT_PWM           TIM2_CH3_PA2   /* 右轮 PWM 输出：TIM2 通道 3（PA2） */
#define CAR_RIGHT_DIR           PA5        /* 右轮方向控制脚 */
#define CAR_LEFT_FORWARD_HIGH   1          /* 左轮方向脚电平与转向关系：1 = 高电平正转，0 = 低电平正转 */
#define CAR_RIGHT_FORWARD_HIGH  1          /* 右轮方向脚电平与转向关系：1 = 高电平正转，0 = 低电平正转 */
#define CAR_LEFT_ENCODER_INVERT 1          /* 左编码器计数取反：使正转时计数增大（1=取反，0=不取反） */
#define CAR_RIGHT_ENCODER_INVERT 1         /* 右编码器计数取反：使正转时计数增大（1=取反，0=不取反） */

/* ---------- 时序 ---------- */
#define CAR_PWM_HZ              8000U      /* PWM 频率 8 kHz */
#define CAR_PERIOD_MS           5U         /* 控制周期 5 ms（PIT 定时中断） */
#define CAR_TELEMETRY_TICKS     20U        /* 每 20 个控制周期上报一次遥测（即 100 ms） */

/* ---------- 编码器与限幅 ---------- */
/* Initial bench assumption only. Verify counts per OUTPUT shaft revolution;
 * JGB37-520 model and 1000rpm alone do not establish PPR or gear ratio. */
#define CAR_ENCODER_CPR         (11.0f * 4.0f * 10.0f)  /* 编码器每转计数：11 线 × 4 倍频 × 10 减速比 = 440 */
#define CAR_RPM_LIMIT           180.0f     /* 目标转速限幅 rpm（速度指令绝对值上限） */
#define CAR_PWM_LIMIT           3500.0f    /* PID 输出 / PWM 占空比限幅（PWM 计数值上限）；NEUQ 库 PWM_MAX=10000 对应 100%，3500 即最大 35% 占空比 */
#define CAR_ACCEL_LIMIT_RPM_S   200.0f     /* 参考轨迹加速度上限 rpm/s */
#define CAR_JERK_LIMIT_RPM_S2   800.0f     /* 参考轨迹加加速度（jerk）上限 rpm/s² */
#define CAR_MIN_CONTROL_US      1000U      /* 控制周期允许的最小间隔 1 ms，越界判定时故障 */
#define CAR_MAX_CONTROL_US      15000U     /* 控制周期允许的最大间隔 15 ms，越界判定时故障 */
#define CAR_SPEED_FILTER_TAU_S  0.005f     /* 速度一阶低通滤波时间常数 5 ms；值越大越平滑，但反馈延迟越大 */

/* ---------- 前馈与 PID ---------- */
/* PWM-count feedforward coefficients require identification on this chassis. */
#define CAR_FF_KS               0.0f       /* 前馈静态项（克服静摩擦，PWM 计数）；0 = 关闭 */
#define CAR_FF_KV               0.0f       /* 前馈速度项系数（PWM 计数/rpm）；0 = 关闭 */
#define CAR_FF_KA               0.0f       /* 前馈加速度项系数（PWM 计数/(rpm/s)）；0 = 关闭 */
#define CAR_PID_D_TAU_S         0.020f     /* 微分项低通滤波时间常数 20 ms，抑制噪声放大 */
#define CAR_PID_SETPOINT_WEIGHT 1.0f       /* P 项设定值权重 beta：1 = 常规 PID，0 = 设定值不计入 P */
#define CAR_DEFAULT_KP          2.0f       /* 默认比例增益 */
#define CAR_DEFAULT_KI          0.12f      /* 默认积分增益（沿用 5 ms 离散单位，具体换算见 Car_PID.h） */
#define CAR_DEFAULT_KD          0.0f       /* 默认微分增益（沿用 5 ms 离散单位，具体换算见 Car_PID.h） */

/* ---------- 超时与故障检测 ---------- */
#define CAR_LINK_TIMEOUT_MS     500U       /* 遥控链路超时：运行中 500 ms 未收到有效指令则判失联故障 */
#define CAR_FRAME_TIMEOUT_MS    100U       /* 半帧接收超时：100 ms 内帧未收完则丢弃该帧 */
#define CAR_STALL_TIMEOUT_MS    1500U      /* 堵转判定：输出饱和但转速过低持续 1.5 s 则判堵转故障 */
#define CAR_DIRECTION_TIMEOUT_MS 1000U     /* 方向异常：目标与转速、输出与转速均反向持续 1 s 则判故障 */
#endif
