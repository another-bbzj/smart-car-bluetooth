/*********************************************************************************************************************
* 文件名称          My_Vofa.c
* 功能说明          VOFA+ JustFloat 遥测发送（实现），接口说明见 My_Vofa.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/

/*
 * ============================================================================
 * 文件名称：My_Vofa.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 本文件实现 VOFA+ 上位机使用的 JustFloat 协议发送缓冲：
 *
 *     PIT 控制中断 -> My_Vofa_push() -> 发送 FIFO -> My_Vofa_poll()（主循环）
 *                   -> 蓝牙串口 -> VOFA+ 波形显示
 *
 * 设计目的：
 *   - 中断只负责把整帧数据放进 FIFO，不在中断里等待串口发送；
 *   - 主循环每次只发送“调用这一刻”FIFO 中已有的字节，
 *     避免数据持续到来导致主循环卡死在发送函数里；
 *   - FIFO 空间不足时整帧丢弃，绝不发送半帧，保证上位机不会错位解析。
 *
 * JustFloat 帧格式：
 *     N 个 float（小端） + 帧尾 00 00 80 7F
 *
 * 函数列表
 * ----------------------------------------------------------------------------
 *   void My_Vofa_init(uint8_t uart_dev)
 *       —— 保存要使用的串口，并初始化发送 FIFO。
 *   uint8_t My_Vofa_push(const float *data, uint8_t channels)
 *       —— 把一帧 N 通道数据整体放入发送 FIFO（可在中断中调用）。
 *   void My_Vofa_poll(void)
 *       —— 把 FIFO 中当前已有的字节发到串口（只在主循环调用）。
 *
 * 上下文约定
 * ----------------------------------------------------------------------------
 *   My_Vofa_push：生产者，运行在 PIT 控制中断；
 *   My_Vofa_poll：消费者，运行在主循环；
 *   NEUQ_FIFO 内部已处理单生产者/单消费者并发。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"
#include "My_Vofa.h"

/* JustFloat 协议帧尾：VOFA+ 用它判断一帧浮点数据结束。 */
static const uint8_t s_tail[4] =
{
    0x00,
    0x00,
    0x80,
    0x7F
};

/* 发送缓冲区：由 NEUQ_FIFO 管理，中断写入、主循环读取。 */
static uint8_t s_buffer[MY_VOFA_FIFO_SIZE];

/* 发送 FIFO 对象，来源：My_Vofa_init 初始化。 */
static NEUQ_FIFO_t s_fifo;

/* 遥测串口编号，来源：My_Vofa_init 参数；默认 UART_0。 */
static UART_DEV_ENUM s_uart = UART_0;

/*
 * 函数名称：My_Vofa_init
 * 函数功能：初始化 VOFA+ 发送模块，保存遥测串口并建立发送 FIFO。
 * 参数说明：uart_dev —— NEUQ 库串口编号，必须与蓝牙串口一致。
 * 返回参数：无。
 * 使用示例：
 *     NEUQ_Uart_init((UART_DEV_ENUM)CAR_UART_INDEX, CAR_UART_BAUD);
 *     My_Vofa_init(CAR_UART_INDEX);
 * 备注信息：只初始化软件 FIFO，串口本身由 Car_Hardware.c 初始化。
 */
void My_Vofa_init(uint8_t uart_dev)
{
    s_uart = (UART_DEV_ENUM)uart_dev;    /* 保存遥测串口编号 */
    NEUQ_FIFO_init(&s_fifo, s_buffer, sizeof(s_buffer));
}

/*
 * 函数名称：My_Vofa_push
 * 函数功能：把一帧多通道数据整体放入发送 FIFO（定时中断中调用）。
 * 参数说明：data     —— 要发送的数据，每个 float 对应 VOFA+ 的一个通道；
 *           channels —— 通道数，范围 1 ~ MY_VOFA_MAX_CHANNELS。
 * 返回参数：1 整帧放入；0 参数非法或剩余空间不足，整帧丢弃。
 * 使用示例：
 *     float channels[2] = { speed_rpm, output_pwm };
 *     My_Vofa_push(channels, 2);
 * 备注信息：STM32 和电脑都是小端，float 直接按内存拷贝即为 JustFloat
 *           要求的字节顺序；写入使用 write_all，保证不会只写半帧。
 */
uint8_t My_Vofa_push(const float *data, uint8_t channels)
{
    uint8_t frame[MY_VOFA_MAX_CHANNELS * 4U + 4U];     /* 临时整帧缓冲 */
    uint32_t length;                                   /* 当前已填充字节数 */

    length = (uint32_t)channels * 4U;                  /* N 个 float 的字节数 */

    if (channels == 0U || channels > MY_VOFA_MAX_CHANNELS)
    {
        return 0;                                      /* 通道数非法，整帧丢弃 */
    }

    /* 先拷贝 N 个 float，再追加 JustFloat 帧尾。 */
    memcpy(frame, data, length);                       /* 拷贝通道数据 */
    memcpy(frame + length, s_tail, sizeof(s_tail));    /* 追加帧尾 */
    length += sizeof(s_tail);                          /* 更新总长度 */

    /* write_all：空间足够才整帧写入，空间不足时一个字节都不写。 */
    return (NEUQ_FIFO_write_all(&s_fifo, frame, length) == NEUQ_FIFO_OK)
         ? 1U
         : 0U;
}

/*
 * 函数名称：My_Vofa_poll
 * 函数功能：把发送 FIFO 中当前已有的数据发到串口（主循环中调用）。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     while (1)
 *     {
 *         My_Vofa_poll();
 *     }
 * 备注信息：只发送调用这一刻 FIFO 中已有的字节；发送期间中断新放入的帧
 *           留到下一次调用，避免数据持续到来导致主循环无法退出。
 */
void My_Vofa_poll(void)
{
    uint32_t count;                            /* 本次准备发送的字节数 */
    uint8_t byte;                              /* 从 FIFO 读出的临时字节 */

    count = NEUQ_FIFO_get_used(&s_fifo);

    while (count > 0U &&
           NEUQ_FIFO_read_byte(&s_fifo, &byte) == NEUQ_FIFO_OK)
    {
        NEUQ_Uart_write_byte(s_uart, byte);    /* 发送一个字节 */
        count--;                               /* 剩余字节数减一 */
    }
}
