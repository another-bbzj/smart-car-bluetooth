/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
* 本开源库使用 GPL3.0 开源许可证协议，修改时必须保留作者的版权声明
*
* 文件名称          NEUQ_FIFO
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2026-09-06        沈泉               增加软件FIFO驱动
********************************************************************************************************************/
#include "NEUQ_FIFO.h"

#include <stddef.h>
#include "stm32f4xx_hal.h"

//==============================================================================================
// 函数名称 : fifo_valid
// 函数功能 : 检查FIFO结构体、存储数组和双周期索引是否有效
// 返回值   : NEUQ_FIFO_OK表示有效，NEUQ_FIFO_ERROR表示无效
//==============================================================================================
static uint8_t fifo_valid(NEUQ_FIFO_t *fifo)
{
    uint32_t cycle;
    if (fifo == NULL || fifo->buffer == NULL || fifo->size == 0U ||
        fifo->size > 0x7FFFFFFFU) {
        return NEUQ_FIFO_ERROR;
    }
    cycle = fifo->size * 2U;
    return (fifo->head < cycle && fifo->tail < cycle) ?
           NEUQ_FIFO_OK : NEUQ_FIFO_ERROR;
}

//==============================================================================================
// 函数名称 : fifo_move
// 函数功能 : 按双周期环形方式移动FIFO索引，避免head与tail相等时混淆满和空
// 返回值   : 移动后的索引
//==============================================================================================
static uint32_t fifo_move(uint32_t index, uint32_t step, uint32_t size)
{
    uint32_t cycle = size * 2U;
    if (step >= cycle - index) {
        return step - (cycle - index);
    }
    return index + step;
}

//==============================================================================================
// 函数名称 : fifo_phys
// 函数功能 : 将双周期索引转换为存储数组中的实际下标
// 返回值   : buffer数组下标
//==============================================================================================
static uint32_t fifo_phys(uint32_t index, uint32_t size)
{
    return (index >= size) ? (index - size) : index;
}

//==============================================================================================
// 函数名称 : fifo_used
// 函数功能 : 计算FIFO当前已使用的字节数
// 返回值   : 已使用字节数
//==============================================================================================
static uint32_t fifo_used(NEUQ_FIFO_t *fifo)
{
    uint32_t cycle = fifo->size * 2U;
    /* Snapshot each volatile index once: a higher-priority consumer may move
     * tail across wrap between the comparison and the subtraction. */
    uint32_t head = fifo->head;
    uint32_t tail = fifo->tail;
    return (head >= tail) ? (head - tail) : (cycle - tail + head);
}

//==============================================================================================
// 函数名称 : fifo_min
// 函数功能 : 返回两个无符号整数中的较小值
// 返回值   : 较小值
//==============================================================================================
static uint32_t fifo_min(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_init
// 函数功能 : 初始化FIFO管理结构体和读写索引
//==============================================================================================
void NEUQ_FIFO_init(NEUQ_FIFO_t *fifo, uint8_t *buffer, uint32_t size)
{
    if (fifo == NULL) {
        return;
    }
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->head = 0U;
    fifo->tail = 0U;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_clear
// 函数功能 : 清零读写索引，清空FIFO中的逻辑数据
//==============================================================================================
void NEUQ_FIFO_clear(NEUQ_FIFO_t *fifo)
{
    if (fifo != NULL) {
        fifo->head = 0U;
        fifo->tail = 0U;
    }
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_write_byte
// 函数功能 : 写入一个字节；写入完成后再发布head，适合中断生产者使用
//==============================================================================================
uint8_t NEUQ_FIFO_write_byte(NEUQ_FIFO_t *fifo, uint8_t data)
{
    uint32_t head;
    if (fifo_valid(fifo) == NEUQ_FIFO_ERROR || fifo_used(fifo) >= fifo->size) {
        return NEUQ_FIFO_ERROR;
    }
    head = fifo->head;
    fifo->buffer[fifo_phys(head, fifo->size)] = data;
    __DMB();
    fifo->head = fifo_move(head, 1U, fifo->size);
    return NEUQ_FIFO_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_read_byte
// 函数功能 : 读取并移除一个字节；读取完成后再释放tail对应空间
//==============================================================================================
uint8_t NEUQ_FIFO_read_byte(NEUQ_FIFO_t *fifo, uint8_t *data)
{
    uint32_t tail;
    if (data == NULL || fifo_valid(fifo) == NEUQ_FIFO_ERROR || fifo_used(fifo) == 0U) {
        return NEUQ_FIFO_ERROR;
    }
    tail = fifo->tail;
    *data = fifo->buffer[fifo_phys(tail, fifo->size)];
    __DMB();
    fifo->tail = fifo_move(tail, 1U, fifo->size);
    return NEUQ_FIFO_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_peek_byte
// 函数功能 : 查看队首字节，不移动tail
//==============================================================================================
uint8_t NEUQ_FIFO_peek_byte(NEUQ_FIFO_t *fifo, uint8_t *data)
{
    if (data == NULL || fifo_valid(fifo) == NEUQ_FIFO_ERROR || fifo_used(fifo) == 0U) {
        return NEUQ_FIFO_ERROR;
    }
    *data = fifo->buffer[fifo_phys(fifo->tail, fifo->size)];
    return NEUQ_FIFO_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_write
// 函数功能 : 批量写入数据，FIFO空间不足时停止并返回实际写入长度
//==============================================================================================
uint32_t NEUQ_FIFO_write(NEUQ_FIFO_t *fifo, const uint8_t *data, uint32_t length)
{
    uint32_t count = 0U;
    if (data == NULL || length == 0U || fifo_valid(fifo) == NEUQ_FIFO_ERROR) {
        return 0U;
    }
    while (count < length && NEUQ_FIFO_write_byte(fifo, data[count]) == NEUQ_FIFO_OK) {
        count++;
    }
    return count;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_read
// 函数功能 : 批量读取数据，FIFO数据不足时停止并返回实际读取长度
//==============================================================================================
uint32_t NEUQ_FIFO_read(NEUQ_FIFO_t *fifo, uint8_t *data, uint32_t length)
{
    uint32_t count = 0U;
    if (data == NULL || length == 0U || fifo_valid(fifo) == NEUQ_FIFO_ERROR) {
        return 0U;
    }
    while (count < length && NEUQ_FIFO_read_byte(fifo, &data[count]) == NEUQ_FIFO_OK) {
        count++;
    }
    return count;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_peek
// 函数功能 : 复制FIFO中的数据但不移动tail，支持跨环形边界查看
//==============================================================================================
uint32_t NEUQ_FIFO_peek(NEUQ_FIFO_t *fifo, uint8_t *data, uint32_t length)
{
    uint32_t index;
    uint32_t count = 0U;
    if (data == NULL || length == 0U || fifo_valid(fifo) == NEUQ_FIFO_ERROR) {
        return 0U;
    }
    length = fifo_min(length, fifo_used(fifo));
    index = fifo->tail;
    while (count < length) {
        data[count++] = fifo->buffer[fifo_phys(index, fifo->size)];
        index = fifo_move(index, 1U, fifo->size);
    }
    return count;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_write_all
// 函数功能 : 仅在空间足够时完整写入数据
//==============================================================================================
uint8_t NEUQ_FIFO_write_all(NEUQ_FIFO_t *fifo, const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U || fifo_valid(fifo) == NEUQ_FIFO_ERROR ||
        NEUQ_FIFO_get_free(fifo) < length) {
        return NEUQ_FIFO_ERROR;
    }
    return (NEUQ_FIFO_write(fifo, data, length) == length) ?
           NEUQ_FIFO_OK : NEUQ_FIFO_ERROR;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_read_all
// 函数功能 : 仅在数据足够时完整读取数据
//==============================================================================================
uint8_t NEUQ_FIFO_read_all(NEUQ_FIFO_t *fifo, uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U || fifo_valid(fifo) == NEUQ_FIFO_ERROR ||
        NEUQ_FIFO_get_used(fifo) < length) {
        return NEUQ_FIFO_ERROR;
    }
    return (NEUQ_FIFO_read(fifo, data, length) == length) ?
           NEUQ_FIFO_OK : NEUQ_FIFO_ERROR;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_drop
// 函数功能 : 移动tail以丢弃最早数据，常与DMA读块接口配合使用
//==============================================================================================
uint32_t NEUQ_FIFO_drop(NEUQ_FIFO_t *fifo, uint32_t length)
{
    if (length == 0U || fifo_valid(fifo) == NEUQ_FIFO_ERROR) {
        return 0U;
    }
    length = fifo_min(length, fifo_used(fifo));
    __DMB();
    fifo->tail = fifo_move(fifo->tail, length, fifo->size);
    return length;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_get_used
// 函数功能 : 获取FIFO当前已使用空间
//==============================================================================================
uint32_t NEUQ_FIFO_get_used(NEUQ_FIFO_t *fifo)
{
    return (fifo_valid(fifo) == NEUQ_FIFO_OK) ? fifo_used(fifo) : 0U;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_get_free
// 函数功能 : 获取FIFO当前剩余空间
//==============================================================================================
uint32_t NEUQ_FIFO_get_free(NEUQ_FIFO_t *fifo)
{
    return (fifo_valid(fifo) == NEUQ_FIFO_OK) ? fifo->size - fifo_used(fifo) : 0U;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_get_size
// 函数功能 : 获取FIFO总容量
//==============================================================================================
uint32_t NEUQ_FIFO_get_size(NEUQ_FIFO_t *fifo)
{
    return (fifo_valid(fifo) == NEUQ_FIFO_OK) ? fifo->size : 0U;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_is_empty / NEUQ_FIFO_is_full
// 函数功能 : 判断FIFO是否为空或已满
//==============================================================================================
uint8_t NEUQ_FIFO_is_empty(NEUQ_FIFO_t *fifo)
{
    return (fifo_valid(fifo) == NEUQ_FIFO_OK && fifo->head == fifo->tail) ? 1U : 0U;
}

uint8_t NEUQ_FIFO_is_full(NEUQ_FIFO_t *fifo)
{
    return (fifo_valid(fifo) == NEUQ_FIFO_OK && fifo_used(fifo) == fifo->size) ? 1U : 0U;
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_get_read_block
// 函数功能 : 返回tail开始的连续可读区域，适合交给DMA或直接处理
//==============================================================================================
uint32_t NEUQ_FIFO_get_read_block(NEUQ_FIFO_t *fifo, uint8_t **data)
{
    uint32_t used;
    uint32_t tail;
    if (data == NULL) {
        return 0U;
    }
    *data = NULL;
    if (fifo_valid(fifo) == NEUQ_FIFO_ERROR || (used = fifo_used(fifo)) == 0U) {
        return 0U;
    }
    tail = fifo_phys(fifo->tail, fifo->size);
    *data = &fifo->buffer[tail];
    return fifo_min(used, fifo->size - tail);
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_get_write_block
// 函数功能 : 返回head开始的连续可写区域，适合DMA直接写入
//==============================================================================================
uint32_t NEUQ_FIFO_get_write_block(NEUQ_FIFO_t *fifo, uint8_t **data)
{
    uint32_t free_length;
    uint32_t head;
    if (data == NULL) {
        return 0U;
    }
    *data = NULL;
    if (fifo_valid(fifo) == NEUQ_FIFO_ERROR ||
        (free_length = fifo->size - fifo_used(fifo)) == 0U) {
        return 0U;
    }
    head = fifo_phys(fifo->head, fifo->size);
    *data = &fifo->buffer[head];
    return fifo_min(free_length, fifo->size - head);
}

//==============================================================================================
// 函数名称 : NEUQ_FIFO_commit_write
// 函数功能 : 发布外部已经写入的数据，长度限制为本次连续可写区域长度
//==============================================================================================
uint32_t NEUQ_FIFO_commit_write(NEUQ_FIFO_t *fifo, uint32_t length)
{
    uint8_t *data;
    uint32_t contiguous = NEUQ_FIFO_get_write_block(fifo, &data);
    if (length > contiguous) {
        length = contiguous;
    }
    if (length != 0U) {
        __DMB();
        fifo->head = fifo_move(fifo->head, length, fifo->size);
    }
    return length;
}
