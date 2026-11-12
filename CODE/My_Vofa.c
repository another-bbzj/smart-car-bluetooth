/*********************************************************************************************************************
* 文件名称          My_Vofa.c
* 功能说明          VOFA+ JustFloat 遥测发送的实现，属于"调试/遥测"模块，不参与车辆控制。
*
*                   本文件包含的函数：
*                   · My_Vofa_init —— 绑定输出串口并初始化发送 FIFO
*                   · My_Vofa_push —— 在中断中把一整帧 float + 帧尾放入发送缓冲
*                   · My_Vofa_poll —— 在主循环中把缓冲字节逐个写串口
*
*                   数据流：5ms 控制拍（每 20 拍）打包 16 通道 -> My_Vofa_push 入队
*                   -> 主循环 My_Vofa_poll 逐字节发送 -> 上位机 VOFA+ 显示波形。
*                   缓冲不足时整帧丢弃，宁可丢一帧遥测也不发送半帧。
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            first version
* 2026-11-12                            统一格式与中文注释，修正 TXE 说明
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include "My_Vofa.h"

/* JustFloat 帧尾：4 个固定字节，VOFA+ 以它识别一帧结束 */
static const uint8_t s_tail[4] = { 0x00U, 0x00U, 0x80U, 0x7FU };

/* 发送缓冲使用 NEUQ_FIFO：一个生产者（控制中断）+ 一个消费者（主循环），库内部已处理并发 */
static uint8_t       s_buffer[MY_VOFA_FIFO_SIZE];           /* 发送 FIFO 的底层存储 */
static NEUQ_FIFO_t   s_fifo;                                /* 发送 FIFO 管理结构 */
static UART_DEV_ENUM s_uart = UART_0;                       /* 实际输出串口，默认 UART_0，由 init 改写 */

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    My_Vofa_init
 * 功能说明    初始化 VOFA+ 发送模块：绑定输出串口，并初始化基于 NEUQ_FIFO 的软件发送缓冲。
 * 用法        在目标串口已经完成 NEUQ_Uart_init 之后调用一次。
 * 参数说明    uart_dev —— 发送所用串口编号（UART_0 ~ UART_3），本车为 UART_3（UART4）
 * 返回说明    无
 * 示例        NEUQ_Uart_init((UART_DEV_ENUM)CAR_UART_INDEX, CAR_UART_BAUD);
 *             My_Vofa_init(CAR_UART_INDEX);
 * 备注        发送与接收共用同一个蓝牙串口：上行是 VOFA+ 遥测，下行是 C3 3C 命令帧。
 *-------------------------------------------------------------------------------------------------------------------*/
void My_Vofa_init(uint8_t uart_dev)
{
    s_uart = (UART_DEV_ENUM)uart_dev;                       /* 保存输出串口编号 */
    NEUQ_FIFO_init(&s_fifo, s_buffer, sizeof(s_buffer));    /* 绑定底层数组并清空读写指针 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    My_Vofa_push
 * 功能说明    把一帧 JustFloat 数据（N 个小端 float + 帧尾 00 00 80 7F）整体放入发送缓冲；
 *             缓冲剩余空间不足时整帧丢弃，保证上位机不会收到半帧而错位。
 * 用法        在定时中断等实时上下文中调用（本项目每 100ms 一次），只做内存拷贝，不阻塞等待串口。
 * 参数说明    data     —— 通道数据数组，每个 float 对应 VOFA+ 的一个通道
 *             channels —— 通道数，范围 1 ~ MY_VOFA_MAX_CHANNELS（16）
 * 返回说明    1 表示整帧成功入队；0 表示通道数非法或缓冲空间不足，整帧已丢弃
 * 示例        float ch[16] = { ... };
 *             My_Vofa_push(ch, 16);   // 16 通道，一帧 68 字节
 * 备注        STM32 与 PC 均为小端，float 直接按内存拷贝即为 JustFloat 要求的字节序；
 *             本函数不发送数据，真正发送由主循环的 My_Vofa_poll 完成。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t My_Vofa_push(const float *data, uint8_t channels)
{
    uint8_t  frame[MY_VOFA_MAX_CHANNELS * 4U + 4U];         /* 栈上的整帧缓冲：最多 16×4 + 4 字节 */
    uint32_t length = (uint32_t)channels * 4U;              /* 本次数据的字节数（不含帧尾） */

    if ((channels == 0U) || (channels > MY_VOFA_MAX_CHANNELS)) {
        return 0U;                                          /* 通道数非法：整帧丢弃 */
    }

    memcpy(frame, data, length);                            /* 先拷贝 N 个 float */
    memcpy(frame + length, s_tail, sizeof(s_tail));         /* 再追加 JustFloat 帧尾 */
    length += sizeof(s_tail);                               /* 加上帧尾后的总长度 */

    /* write_all：空间够才整帧写入，不够时一个字节都不写，避免半帧。 */
    return (NEUQ_FIFO_write_all(&s_fifo, frame, length) == NEUQ_FIFO_OK) ? 1U : 0U;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    My_Vofa_poll
 * 功能说明    把发送缓冲中"调用这一刻"已有的字节逐个写入串口，发送完即返回。
 * 用法        在主循环中反复调用（本项目 Car_Main 的 while(1) 只调用它）。
 * 参数说明    无
 * 返回说明    无
 * 示例        while (1) { My_Vofa_poll(); }
 * 备注        1) 只发送进入函数时缓冲中的字节数，中断期间新入队的数据留到下一次，
 *                避免持续有数据时主循环卡在发送里；
 *             2) NEUQ_Uart_write_byte 会等待发送数据寄存器空（TXE），9600 波特率下约 1ms/字节；
 *             3) 缓冲满时 push 整帧丢弃，因此主循环应及时轮询，避免遥测丢失。
 *-------------------------------------------------------------------------------------------------------------------*/
void My_Vofa_poll(void)
{
    uint32_t count = NEUQ_FIFO_get_used(&s_fifo);           /* 进入函数时缓冲中的字节数（快照） */
    uint8_t  byte;                                          /* 当前要发送的字节 */

    while ((count > 0U) && (NEUQ_FIFO_read_byte(&s_fifo, &byte) == NEUQ_FIFO_OK)) {
        NEUQ_Uart_write_byte(s_uart, byte);                 /* 阻塞等待 TXE 后写入一个字节 */
        count--;
    }
}
