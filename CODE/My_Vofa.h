/*********************************************************************************************************************
* 文件名称          My_Vofa
* 功能说明          向上位机 VOFA+ 发送波形数据（JustFloat 协议）
*
*                   JustFloat 帧格式：N 个 float（每个 4 字节，小端）+ 帧尾 00 00 80 7F
*                   VOFA+ 收到后把第 1 个 float 画成通道 I0，第 2 个画成 I1，依此类推
*
*                   为什么分成 push 和 poll 两个函数：
*                   115200 波特率下发一个字节要约 87 us，一帧十几到几十字节要 1 ~ 4 ms。
*                   如果在定时中断里直接发，中断会被拖长很多。所以：
*                   · 定时中断里调用 My_Vofa_push()：只把一帧数据放进软件缓冲（FIFO），几微秒就返回
*                   · 主循环里调用 My_Vofa_poll()：慢慢把缓冲里的字节发到串口
*                   缓冲放不下时整帧丢弃，不会出现"半帧"，VOFA+ 不会解析错位
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
********************************************************************************************************************/
#ifndef _MY_VOFA_H_
#define _MY_VOFA_H_

#include <stdint.h>

#define MY_VOFA_MAX_CHANNELS       (16U)       // 一帧最多几个通道
#define MY_VOFA_FIFO_SIZE          (2048U)     // 发送缓冲大小（字节），主循环忙于刷屏、写 Flash 时先存在这里

void    My_Vofa_init(uint8_t uart_dev);                        // uart_dev 取 UART_0 ~ UART_3，串口需要已经初始化
uint8_t My_Vofa_push(const float *data, uint8_t channels);     // 中断中调用：放入一帧，1 成功 0 缓冲满被丢弃
void    My_Vofa_poll(void);                                    // 主循环中调用：把缓冲里的数据发出去

#endif
