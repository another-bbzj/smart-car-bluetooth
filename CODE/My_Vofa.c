/*********************************************************************************************************************
* 文件名称          My_Vofa
* 功能说明          VOFA+ JustFloat 发送（实现），说明见 My_Vofa.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "My_Vofa.h"

static const uint8_t s_tail[4] = {0x00, 0x00, 0x80, 0x7F};    // JustFloat 帧尾

// 发送缓冲使用库里的 NEUQ_FIFO：一个生产者（中断）+ 一个消费者（主循环），库内部已处理好并发
static uint8_t       s_buffer[MY_VOFA_FIFO_SIZE];
static NEUQ_FIFO_t   s_fifo;
static UART_DEV_ENUM s_uart = UART_0;

void My_Vofa_init(uint8_t uart_dev)
{
    s_uart = (UART_DEV_ENUM)uart_dev;
    NEUQ_FIFO_init(&s_fifo, s_buffer, sizeof(s_buffer));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把一帧数据放进发送缓冲（定时中断中调用）
// 参数说明     data        要发送的数据，每个 float 对应 VOFA+ 的一个通道
// 参数说明     channels    通道数 1 ~ MY_VOFA_MAX_CHANNELS
// 返回参数     1 整帧放入  0 缓冲剩余空间不够，整帧丢弃
// 备注信息     STM32 和电脑都是小端，float 直接按内存拷贝就是 JustFloat 要求的字节顺序
//-------------------------------------------------------------------------------------------------------------------
uint8_t My_Vofa_push(const float *data, uint8_t channels)
{
    uint8_t  frame[MY_VOFA_MAX_CHANNELS * 4U + 4U];
    uint32_t length = (uint32_t)channels * 4U;

    if (channels == 0U || channels > MY_VOFA_MAX_CHANNELS) {
        return 0;
    }
    memcpy(frame, data, length);                        // N 个 float
    memcpy(frame + length, s_tail, sizeof(s_tail));     // 帧尾
    length += sizeof(s_tail);

    // write_all：空间够才整帧写入，不够就一个字节都不写
    return (NEUQ_FIFO_write_all(&s_fifo, frame, length) == NEUQ_FIFO_OK) ? 1U : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把缓冲里的数据发到串口（主循环中调用）
// 备注信息     只发"调用这一刻"缓冲里已有的字节；发送期间中断新放进来的帧留到下一次，
//              避免数据一直在来导致主循环卡在这里出不去
//-------------------------------------------------------------------------------------------------------------------
void My_Vofa_poll(void)
{
    uint32_t count = NEUQ_FIFO_get_used(&s_fifo);
    uint8_t  byte;

    while (count > 0U && NEUQ_FIFO_read_byte(&s_fifo, &byte) == NEUQ_FIFO_OK) {
        NEUQ_Uart_write_byte(s_uart, byte);             // 库函数：等上一个字节发完再发这个
        count--;
    }
}
