/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */

/*
 * ============================================================================
 * 文件名称：Car_Hardware.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件是蓝牙差速小车的“硬件适配层”，负责把 STM32F407 的外设与
 * Car_Core.c 中的纯算法控制核心连接起来：
 *
 *     编码器 -> 计数增量 -> Car_TickDt() -> 双轮 PWM/方向引脚
 *     蓝牙串口 -> 接收 FIFO -> Car_FeedByte() -> 状态机
 *     香橙派视觉串口 -> 接收 FIFO -> CarVision_Feed() -> 视觉观测缓存
 *     VOFA+ 遥测 <- My_Vofa_push() <- 固定周期采样的控制量
 *
 * 本文件掌管的外设：
 *   - TIM8_CH1 / TIM2_CH3：左右轮 PWM；
 *   - PC7 / PA5：左右轮方向引脚（PA5 与 Flash 片选复用，初始化时先拉高 CS）；
 *   - TIM4 / TIM3：左右轮正交编码器；
 *   - UART4：蓝牙 JDY-31，NEUQ 库设备索引 3；
 *   - USART3：香橙派视觉串口，NEUQ 库设备索引 2；
 *   - TIM5：1 MHz 自由运行计数器，用于测量真实控制周期；
 *   - PIT_CH1：5 ms 控制节拍，实际周期由 TIM5 修正。
 *
 * 执行上下文
 * ----------------------------------------------------------------------------
 *   串口接收中断：只把字节写入 FIFO，不做解析；
 *   PIT 控制中断：取编码器、解析蓝牙、更新控制、输出 PWM、解析视觉；
 *   主循环：只负责把遥测 FIFO 中的数据发到串口。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static uint8_t init_control_clock(void)
 *       —— 把 TIM5 配置成 1 MHz 自由运行计数器，为周期测量提供微秒时基。
 *   static void output(unsigned i, float value)
 *       —— 把带符号 PWM 数值写到指定轮的 PWM 和方向引脚。
 *   static int32_t delta_count(unsigned i, TIM_HandleTypeDef *timer)
 *       —— 读取编码器计数器并计算本周期有符号增量，支持 16 位回绕。
 *   void Car_UART_RX_Handler(uint8_t byte)
 *       —— 蓝牙串口单字节接收回调，只写入接收 FIFO。
 *   void Car_Vision_UART_RX_Handler(uint8_t byte)
 *       —— 视觉串口单字节接收回调，只写入视觉 FIFO。
 *   uint8_t Car_GetVisionObservation(CarVisionObservation *out)
 *       —— 在临界区内复制道路观测快照。
 *   uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out)
 *       —— 在临界区内复制数独子块快照。
 *   void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 *       —— 串口错误回调：标记对应通道故障并重新挂接单字节接收。
 *   void Car_PIT_Handler(void)
 *       —— 5 ms 控制中断入口：取数、解析、控制、输出、视觉和遥测。
 *   void Car_Main(void)
 *       —— 底盘入口：初始化所有外设和共享对象，启动 PIT，进入遥测主循环。
 *
 * 单位约定
 * ----------------------------------------------------------------------------
 *   编码器增量：计数（有符号）；控制周期：微秒；
 *   PWM 数值：NEUQ 库 PWM_MAX = 10000 对应 100% 占空比；
 *   遥测输出百分比：PWM 数值 / 100。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"
#include "Car_Config.h"
#include "Car_Core.h"
#include "Car_Hardware.h"
#include "My_Vofa.h"

/* NEUQ 库内部接口：声明在此以便直接为指定串口打开单字节接收中断。 */
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM dev, uint8_t priority);

/* 蓝牙 UART4 的 HAL 句柄和单字节接收缓存，来源：NEUQ 库串口初始化。 */
extern UART_HandleTypeDef UART_3_handle;
extern uint8_t UART_3_RX_dat;

/* 视觉 USART3 的 HAL 句柄和单字节接收缓存，来源：NEUQ 库串口初始化。 */
extern UART_HandleTypeDef UART_2_handle;
extern uint8_t UART_2_RX_dat;

/* 左右轮编码器定时器句柄，来源：NEUQ_Encoder_init 初始化。 */
extern TIM_HandleTypeDef Encoder_TIM3_Handle;
extern TIM_HandleTypeDef Encoder_TIM4_Handle;

/* ========================================================================== */
/*                             模块内部静态变量                                */
/* ========================================================================== */

/* 整车控制对象：协议状态、目标、参考、实测、PID 和保护计时都保存在这里。 */
static Car_Core car;

/* 蓝牙接收 FIFO 及其缓冲区：中断写入，PIT 中断读取。 */
static NEUQ_FIFO_t rx_fifo;
static uint8_t rx_buffer[128];

/* 蓝牙接收异常标志：FIFO 溢出或串口错误时置 1，由下次 PIT 处理。 */
static volatile uint8_t rx_fault;

/* 香橙派视觉协议解析器：只维护观测和分块缓存，不参与电机控制。 */
static CarVision vision;

/* 视觉接收 FIFO 及其缓冲区：中断写入，PIT 中断读取。 */
static NEUQ_FIFO_t vision_fifo;
static uint8_t vision_buffer[512];

/* 视觉接收异常标志：FIFO 溢出或串口错误时置 1，由下次 PIT 使缓存失效。 */
static volatile uint8_t vision_rx_fault;

/* 编码器上次采样值：由 PIT 中断维护，用于计算 16 位计数器增量。 */
static uint16_t previous[2];

/* TIM5 微秒控制时钟：提供实际周期测量，避免中断抖动影响速度计算。 */
static TIM_HandleTypeDef control_clock;

/* 上一次控制周期结束时的 TIM5 计数值，单位微秒。 */
static uint32_t last_control_us;

/* TIM5 初始化结果：1 表示控制时钟可用，0 表示测周期失败。 */
static uint8_t control_clock_ready;

/* reverse[i]：当前方向对应的逻辑方向，1 表示反向；
 * waiting[i]：已经撤去 PWM、正在等待下一次调用切换方向引脚。 */
static uint8_t reverse[2];
static uint8_t waiting[2];

/* 左右轮 PWM 通道，来源：Car_Config.h 的引脚配置；下标 0 左、1 右。 */
static const PWM_Ports_ENUM pwm[2] =
{
    CAR_LEFT_PWM,
    CAR_RIGHT_PWM
};

/* 左右轮方向引脚，来源：Car_Config.h；下标 0 左、1 右。 */
static const GPIO_Ports_ENUM dir_pin[2] =
{
    CAR_LEFT_DIR,
    CAR_RIGHT_DIR
};

/* 各轮“前进”对应的方向引脚电平，来源：Car_Config.h；下标 0 左、1 右。 */
static const uint8_t forward_high[2] =
{
    CAR_LEFT_FORWARD_HIGH,
    CAR_RIGHT_FORWARD_HIGH
};

/* ========================================================================== */
/*                              内部工具函数                                   */
/* ========================================================================== */

/*
 * 函数名称：init_control_clock
 * 函数功能：把 TIM5 配置成 1 MHz 自由运行计数器，为控制周期测量提供
 *           微秒级时基。
 * 参数说明：无。
 * 返回参数：1 成功；0 时钟频率无法整除 1 MHz 或 HAL 初始化失败。
 * 使用示例：
 *     control_clock_ready = init_control_clock();
 *     last_control_us = __HAL_TIM_GET_COUNTER(&control_clock);
 * 备注信息：TIM5 不占用编码器引脚，只用于测量控制周期的真实长度。
 *           计数器周期为 32 位最大值，约 71 分钟自然回绕一次；
 *           无符号减法允许跨越一次回绕。
 */
static uint8_t init_control_clock(void)
{
    RCC_ClkInitTypeDef clocks;                                    /* 当前系统时钟配置 */
    uint32_t latency;                                             /* Flash 等待周期，本函数不使用 */
    uint32_t timer_hz;                                            /* TIM5 输入时钟频率，Hz */

    HAL_RCC_GetClockConfig(&clocks, &latency);

    timer_hz = HAL_RCC_GetPCLK1Freq();                            /* APB1 时钟频率 */

    /* APB1 分频不为 1 时，定时器时钟为 PCLK1 的两倍。 */
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timer_hz *= 2U;
    }

    /* 预分频只能取整数，因此要求定时器时钟能被 1 MHz 整除。 */
    if (timer_hz < 1000000U || (timer_hz % 1000000U) != 0U)
    {
        return 0;
    }

    __HAL_RCC_TIM5_CLK_ENABLE();                                  /* 打开 TIM5 时钟 */

    control_clock.Instance = TIM5;                                /* 使用 TIM5 */
    control_clock.Init.Prescaler = timer_hz / 1000000U - 1U;      /* 分频到 1 MHz */
    control_clock.Init.Period = 0xFFFFFFFFU;                      /* 32 位自由运行 */
    control_clock.Init.CounterMode = TIM_COUNTERMODE_UP;          /* 向上计数 */
    control_clock.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    /* 不分频 */
    control_clock.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&control_clock) != HAL_OK)
    {
        return 0;
    }

    return (HAL_TIM_Base_Start(&control_clock) == HAL_OK) ? 1U : 0U;
}

/*
 * 函数名称：output
 * 函数功能：把带符号 PWM 数值应用到指定车轮。
 * 参数说明：i     —— 轮号，0 左轮、1 右轮；
 *           value —— 带符号 PWM 数值，正负号表示方向，绝对值表示占空比。
 * 返回参数：无。
 * 使用示例：
 *     output(0, car.output[0]);   // 左轮
 *     output(1, car.output[1]);   // 右轮
 * 备注信息：改变方向前先把 PWM 置零，等下一次输出调用再切换方向引脚，
 *           形成一个控制周期的换向间隔，避免 H 桥上下管直通。
 */
static void output(unsigned i, float value)
{
    uint8_t next;                            /* 期望方向，1 表示反向 */

    next = (value < 0.0f) ? 1U : 0U;

    /* 零输出：撤去 PWM、结束等待状态，方向保持不变。 */
    if (value == 0.0f)
    {
        NEUQ_PWM_set_duty(pwm[i], 0);        /* 占空比归零 */
        waiting[i] = 0;                      /* 退出换向等待 */
        return;
    }

    /* 方向需要改变时，先撤去 PWM，等下一次调用再翻转方向引脚。 */
    if (next != reverse[i])
    {
        if (!waiting[i])
        {
            NEUQ_PWM_set_duty(pwm[i], 0);    /* 先撤去 PWM */
            waiting[i] = 1;                  /* 进入换向等待 */
            return;
        }

        /* forward_high 为 1 表示高电平前进；next 为 1 时取反。 */
        if (next)
        {
            NEUQ_GPIO_write(dir_pin[i],
                            forward_high[i] ? GPIO_Low : GPIO_High);
        }
        else
        {
            NEUQ_GPIO_write(dir_pin[i],
                            forward_high[i] ? GPIO_High : GPIO_Low);
        }

        reverse[i] = next;                   /* 记录新方向 */
    }

    waiting[i] = 0;                          /* 换向完成，退出等待 */

    /* 四舍五入到整数占空比；PWM 数值范围由 NEUQ 库限制。 */
    NEUQ_PWM_set_duty(pwm[i],
                      (uint32_t)((value < 0.0f ? -value : value) + 0.5f));
}

/*
 * 函数名称：delta_count
 * 函数功能：读取编码器计数器并计算本周期有符号增量。
 * 参数说明：i     —— 轮号，0 左轮、1 右轮；
 *           timer —— 对应的编码器定时器句柄。
 * 返回参数：本周期计数增量，正负号表示转向。
 * 使用示例：
 *     l = delta_count(0, &Encoder_TIM4_Handle);
 *     r = delta_count(1, &Encoder_TIM3_Handle);
 * 备注信息：编码器计数器不清零，使用 16 位模差处理回绕，
 *           避免“读后清零”丢失两次读取之间的脉冲。
 *           前提是相邻采样间的实际增量绝对值小于 32768。
 */
static int32_t delta_count(unsigned i, TIM_HandleTypeDef *timer)
{
    uint16_t now;     /* 本次读取的 16 位计数值 */
    uint16_t diff;    /* 两次读取的 16 位模差 */

    now = (uint16_t)__HAL_TIM_GET_COUNTER(timer);
    diff = (uint16_t)(now - previous[i]);
    previous[i] = now;

    /* 最高位为 1 表示模差对应负数，减去 65536 还原有符号增量。 */
    if (diff & 0x8000U)
    {
        return (int32_t)diff - 65536;
    }

    return (int32_t)diff;
}

/* ========================================================================== */
/*                              串口接收接口                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_UART_RX_Handler
 * 函数功能：蓝牙串口单字节接收回调，只把字节写入接收 FIFO。
 * 参数说明：byte —— 刚收到的原始字节，来源：UART4 接收中断。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库串口接收回调自动触发
 *     Car_UART_RX_Handler(dat);
 * 备注信息：回调中不做任何解析；FIFO 写满时只置故障标志，
 *           由下次 PIT 中断统一停车并清空队列。
 */
void Car_UART_RX_Handler(uint8_t byte)
{
    if (NEUQ_FIFO_write_byte(&rx_fifo, byte) != NEUQ_FIFO_OK)
    {
        rx_fault = 1;    /* 标记蓝牙接收异常 */
    }
}

/*
 * 函数名称：Car_Vision_UART_RX_Handler
 * 函数功能：视觉串口单字节接收回调，只把字节写入视觉 FIFO。
 * 参数说明：byte —— 刚收到的原始字节，来源：USART3 接收中断。
 * 返回参数：无。
 * 使用示例：
 *     // 由 NEUQ 库串口接收回调自动触发
 *     Car_Vision_UART_RX_Handler(dat);
 * 备注信息：视觉数据不参与实时控制，FIFO 写满只标记异常，
 *           由 PIT 使视觉缓存失效，不会导致整车停车。
 */
void Car_Vision_UART_RX_Handler(uint8_t byte)
{
    if (NEUQ_FIFO_write_byte(&vision_fifo, byte) != NEUQ_FIFO_OK)
    {
        vision_rx_fault = 1;    /* 标记视觉接收异常 */
    }
}

/*
 * 函数名称：Car_GetVisionObservation
 * 函数功能：在临界区内复制道路观测快照。
 * 参数说明：out —— 接收副本的对象，不能为空。
 * 返回参数：1 表示有可用快照；0 表示无数据、已过期或参数无效。
 * 使用示例：
 *     CarVisionObservation obs;
 *     if (Car_GetVisionObservation(&obs) && (obs.valid_mask & VISION_LATERAL_VALID))
 *     {
 *         float lateral = obs.lateral_m;
 *     }
 * 备注信息：视觉解析在 PIT 中断中进行，本函数短暂屏蔽中断以复制一致快照，
 *           结束后恢复调用前的中断屏蔽状态。
 *           成功后仍需检查 valid_mask 和 confidence，不能直接当作行驶指令。
 */
uint8_t Car_GetVisionObservation(CarVisionObservation *out)
{
    uint32_t saved;             /* 调用前的中断屏蔽状态 */
    uint8_t result;             /* 视觉层返回结果 */

    saved = __get_PRIMASK();    /* 保存中断屏蔽状态 */
    __disable_irq();            /* 进入临界区 */

    result = CarVision_GetObservation(&vision, HAL_GetTick(), out);

    __set_PRIMASK(saved);       /* 恢复原中断屏蔽状态 */

    return result;
}

/*
 * 函数名称：Car_GetVisionTile
 * 函数功能：在临界区内复制当前会话的数独子块快照。
 * 参数说明：index —— 子块编号：0 左上、1 右上、2 左下、3 右下；
 *           out   —— 接收副本的对象，不能为空。
 * 返回参数：1 成功；0 参数无效或数据不可用，失败时不改写 out。
 * 使用示例：
 *     CarVisionTile tile;
 *     if (Car_GetVisionTile(0, &tile))
 *     {
 *         // tile.cells[0] ~ tile.cells[8] 可用
 *     }
 * 备注信息：与观测快照一样在临界区内复制，避免读到解析到一半的数据。
 */
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out)
{
    uint32_t saved;             /* 调用前的中断屏蔽状态 */
    uint8_t result;             /* 视觉层返回结果 */

    saved = __get_PRIMASK();    /* 保存中断屏蔽状态 */
    __disable_irq();            /* 进入临界区 */

    result = CarVision_GetTile(&vision, index, HAL_GetTick(), out);

    __set_PRIMASK(saved);       /* 恢复原中断屏蔽状态 */

    return result;
}

/*
 * 函数名称：HAL_UART_ErrorCallback
 * 函数功能：HAL 串口错误回调，标记对应通道并重新挂接单字节接收。
 * 参数说明：huart —— 发生错误的串口句柄。
 * 返回参数：无。
 * 使用示例：
 *     // HAL 在串口错误时自动调用，业务代码无需手动调用
 *     // UART4 错误 -> rx_fault = 1；USART3 错误 -> vision_rx_fault = 1
 * 备注信息：蓝牙异常交由 PIT 停车处理，视觉异常只使缓存失效。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        rx_fault = 1;           /* 蓝牙通道异常 */
        HAL_UART_Receive_IT(&UART_3_handle, &UART_3_RX_dat, 1);
    }
    else if (huart->Instance == USART3)
    {
        vision_rx_fault = 1;    /* 视觉通道异常 */
        HAL_UART_Receive_IT(&UART_2_handle, &UART_2_RX_dat, 1);
    }
}

/* ========================================================================== */
/*                              周期控制中断                                   */
/* ========================================================================== */

/*
 * 函数名称：Car_PIT_Handler
 * 函数功能：PIT_CH1 控制中断入口，标称每 5 ms 执行一次，完成编码器采样、
 *           蓝牙解析、双轮控制、PWM 输出、视觉解析和遥测入队。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     void NEUQ_PIT_1_CALLBACK(void)
 *     {
 *         Car_PIT_Handler();
 *     }
 * 备注信息：实际周期由 TIM5 测得，控制与保护都使用实测周期。
 *           执行顺序固定为：故障处理 -> 编码器 -> 蓝牙解析 -> 控制输出
 *           -> 视觉解析 -> 遥测入队，确保电机输出不受视觉解析耗时影响。
 */
void Car_PIT_Handler(void)
{
    static unsigned telemetry;                   /* 遥测分频计数 */
    unsigned n;                                  /* 本周期已处理的字节数 */
    uint8_t byte;                                /* 从 FIFO 读出的临时字节 */
    uint32_t sample_us;                          /* 本次控制时刻，微秒 */
    uint32_t dt_us;                              /* 实际控制周期，微秒 */
    int32_t l;                                   /* 左轮本周期编码器增量 */
    int32_t r;                                   /* 右轮本周期编码器增量 */

    sample_us = control_clock_ready
              ? __HAL_TIM_GET_COUNTER(&control_clock)
              : 0U;

    /* 无符号减法允许微秒计数器跨越一次自然回绕。 */
    dt_us = sample_us - last_control_us;

    l = delta_count(0, &Encoder_TIM4_Handle);    /* 左轮计数增量 */
    r = delta_count(1, &Encoder_TIM3_Handle);    /* 右轮计数增量 */

    /* 编码器安装方向与车轮前进方向不一致时，在配置中取反。 */
    if (CAR_LEFT_ENCODER_INVERT)
    {
        l = -l;
    }

    if (CAR_RIGHT_ENCODER_INVERT)
    {
        r = -r;
    }

    last_control_us = sample_us;                 /* 保存本次时刻，供下周期求差 */

    /*
     * 蓝牙字节可能丢失，此时不再信任残留队列：
     * 先故障停车，再清空待解析数据。
     */
#if !CAR_COMMAND_SOURCE_PI
    if (rx_fault)
    {
        Car_Fault(&car, CAR_RX_FAULT);           /* 接收异常立即停车 */

        while (NEUQ_FIFO_read_byte(&rx_fifo, &byte) == NEUQ_FIFO_OK)
        {
        }

        rx_fault = 0;                            /* 清除接收异常标志 */
    }
    else
    {
        /* 每周期最多处理 32 字节蓝牙数据，限制通信解析占用的中断时间。 */
        for (n = 0;
             n < 32 && NEUQ_FIFO_read_byte(&rx_fifo, &byte) == NEUQ_FIFO_OK;
             n++)
        {
            Car_FeedByte(&car, byte);            /* 逐字节送入协议解析 */
        }
    }
#else
    /* USART3 is the only motion-command source in the competition build. */
    for (n = 0; n < 32 &&
         NEUQ_FIFO_read_byte(&rx_fifo, &byte) == NEUQ_FIFO_OK; n++) {}
    rx_fault = 0;
    if (vision_rx_fault)
    {
        Car_Fault(&car, CAR_RX_FAULT);
        while (NEUQ_FIFO_read_byte(&vision_fifo, &byte) == NEUQ_FIFO_OK) {}
        vision_rx_fault = 0;
    }
    else
    {
        for (n = 0; n < 64 &&
             NEUQ_FIFO_read_byte(&vision_fifo, &byte) == NEUQ_FIFO_OK; n++)
            Car_FeedByte(&car, byte);
    }
#endif

    /* 使用实测周期更新控制器，并把双轮输出应用到电机。 */
    Car_TickDt(&car, l, r, dt_us);
    output(0, car.output[0]);                    /* 应用左轮输出 */
    output(1, car.output[1]);                    /* 应用右轮输出 */

    /*
     * 更新电机后再处理视觉，每周期最多解析 64 字节；视觉解析器只更新观测。
     * 蓝牙接收异常触发停车，视觉接收异常仅使视觉缓存失效。
     */
#if !CAR_COMMAND_SOURCE_PI
    if (vision_rx_fault)
    {
        CarVision_Invalidate(&vision);           /* 视觉异常使缓存失效 */

        for (n = 0;
             n < sizeof(vision_buffer) &&
             NEUQ_FIFO_read_byte(&vision_fifo, &byte) == NEUQ_FIFO_OK;
             n++)
        {
        }

        vision_rx_fault = 0;                     /* 清除视觉异常标志 */
    }
    else
    {
        uint32_t now = HAL_GetTick();            /* 当前毫秒时间 */

        for (n = 0;
             n < 64 && NEUQ_FIFO_read_byte(&vision_fifo, &byte) == NEUQ_FIFO_OK;
             n++)
        {
            /* 只有停车状态才允许接受新的 HELLO 会话。 */
            CarVision_Feed(&vision, byte, now, car.state == CAR_STOP);
        }
    }
#endif

    /* 每 CAR_TELEMETRY_TICKS 个控制周期发送一次 16 通道遥测。 */
    if (++telemetry >= CAR_TELEMETRY_TICKS)
    {
        float channels[16];                      /* 遥测通道数组 */

        telemetry = 0;                           /* 遥测分频计数清零 */

        /*
         * 遥测顺序：
         *   0 状态，1 故障码，2/3 左轮参考/实测，4/5 右轮参考/实测，
         *   6/7 左右轮输出百分比，8 命令结果，9 帧数，
         *   10~12 左轮 Kp/Ki/Kd，13~15 右轮 Kp/Ki/Kd。
         */
        channels[0] = (float)car.state;
        channels[1] = (float)car.fault;
        channels[2] = car.reference[0];
        channels[3] = car.speed[0];
        channels[4] = car.reference[1];
        channels[5] = car.speed[1];
        channels[6] = car.output[0] / 100.0f;
        channels[7] = car.output[1] / 100.0f;
        channels[8] = (float)car.result;
        channels[9] = (float)car.frames;
        channels[10] = car.pid[0].kp;
        channels[11] = car.pid[0].ki;
        channels[12] = car.pid[0].kd;
        channels[13] = car.pid[1].kp;
        channels[14] = car.pid[1].ki;
        channels[15] = car.pid[1].kd;

        /*
         * 默认 10 Hz × (16 × 4 + 4) 字节 = 680 B/s，
         * 小于 9600 / 10 = 960 B/s，不会阻塞主循环。
         */
        My_Vofa_push(channels, 16);
    }
}

/* ========================================================================== */
/*                                底盘入口                                     */
/* ========================================================================== */

/*
 * 函数名称：Car_Main
 * 函数功能：底盘入口，初始化全部外设和共享对象后进入遥测主循环。
 * 参数说明：无。
 * 返回参数：正常不会返回。
 * 使用示例：
 *     void User_Main(void)
 *     {
 *         Car_Main();             // 初始化后由 PIT 中断驱动控制
 *     }
 * 备注信息：初始化顺序有依赖：
 *           1. 先建立控制对象和 FIFO；
 *           2. 先拉高 Flash 片选，再复用 PA5/PA6/PA7；
 *           3. 配置方向、PWM、编码器和串口；
 *           4. 建立编码器基准、启动 TIM5；
 *           5. 最后启动 PIT，避免中断访问未初始化对象。
 */
void Car_Main(void)
{
    unsigned i;                                    /* 双轮循环下标 */

    Car_Init(&car);                                /* 初始化控制对象 */
    CarVision_Init(&vision);                       /* 初始化视觉解析器 */

    NEUQ_FIFO_init(&vision_fifo, vision_buffer, sizeof(vision_buffer));

    /* 先取消板载 Flash 片选，再将共用的 PA5/PA6/PA7 用于方向和编码器。 */
    NEUQ_GPIO_init(CAR_FLASH_CS, GPIO_Mode_Output_PushPull, GPIO_High);

    NEUQ_FIFO_init(&rx_fifo, rx_buffer, sizeof(rx_buffer));

    for (i = 0; i < 2; i++)
    {
        /* 方向引脚初始电平按“前进”配置，PWM 初始占空比为 0。 */
        NEUQ_GPIO_init(dir_pin[i],
                       GPIO_Mode_Output_PushPull,
                       forward_high[i] ? GPIO_High : GPIO_Low);
        NEUQ_PWM_init(pwm[i], CAR_PWM_HZ, 0);
    }

    NEUQ_Encoder_init(Encoder_TIM4,
                      Encoder_TIM4_PhaseA_PB6,
                      Encoder_TIM4_PhaseB_PB7);
    NEUQ_Encoder_init(Encoder_TIM3,
                      Encoder_TIM3_PhaseA_PA6,
                      Encoder_TIM3_PhaseB_PA7);

    /* 在启动周期中断前建立编码器基准，首周期只计算此后产生的脉冲。 */
    previous[0] = (uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM4_Handle);
    previous[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM3_Handle);

    NEUQ_Uart_init((UART_DEV_ENUM)CAR_UART_INDEX, CAR_UART_BAUD);
    My_Vofa_init(CAR_UART_INDEX);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_UART_INDEX, 3);

    NEUQ_Uart_init((UART_DEV_ENUM)CAR_VISION_UART_INDEX, CAR_VISION_UART_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_VISION_UART_INDEX, 4);

    control_clock_ready = init_control_clock();    /* 启动微秒时基 */
    last_control_us = control_clock_ready
                    ? __HAL_TIM_GET_COUNTER(&control_clock)
                    : 0U;

    /*
     * 所有共享对象和外设就绪后再启动控制。
     * 如果 TIM5 初始化失败，dt 将保持 0，Car_TickDt 会触发时序故障。
     */
    NEUQ_PIT_init_ms(PIT_CH1, CAR_PERIOD_MS);

    /* 控制由 PIT 中断驱动，主循环只持续推进遥测串口发送。 */
    while (1)
    {
        My_Vofa_poll();                            /* 发送已排队的遥测数据 */
    }
}
