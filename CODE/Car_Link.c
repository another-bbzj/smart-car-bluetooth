/*********************************************************************************************************************
* 文件名称          Car_Link.c
* 功能说明          蓝牙链路与视觉链路的接收缓冲、错误恢复和视觉快照接口，属于"通信链路"模块。
*
*                   本文件包含的函数：
*                   · CarLink_Init              —— 初始化视觉上下文与两条接收 FIFO
*                   · Car_UART_RX_Handler       —— 蓝牙字节入队（UART4 中断调用）
*                   · Car_Vision_UART_RX_Handler—— 视觉字节入队（USART3 中断调用）
*                   · Car_GetVisionObservation  —— 关中断读取视觉观测快照
*                   · Car_GetVisionTile         —— 关中断读取数独子块快照
*                   · CarLink_ProcessBluetooth  —— 控制拍内有界解析蓝牙命令
*                   · CarLink_ProcessVision     —— 控制拍内有界解析视觉数据
*                   · HAL_UART_ErrorCallback    —— HAL 串口错误回调重写：置故障标志并重新开接收
*
*                   设计要点：中断只入队，解析放在 5ms 控制拍；两条链路使用各自独立的 FIFO，
*                   视觉数据不能启动电机，也不能影响蓝牙命令的受理。
*
* 修改记录
* 日期              作者                备注
* 2026-11-12                            从 Car_Hardware.c 拆出串口链路模块
********************************************************************************************************************/
#include "Car_Link.h"
#include "Car_Config.h"
#include "NEUQ_lib_include.h"

#define CAR_BLUETOOTH_FIFO_SIZE  128U   /* 蓝牙接收 FIFO 容量（字节），可缓存约 0.5s 的 9600 数据 */
#define CAR_VISION_FIFO_SIZE     512U   /* 视觉接收 FIFO 容量（字节），可缓存多帧 115200 数据 */

/* 蓝牙接收 FIFO 与其底层存储：一个生产者（中断）加一个消费者（5ms 控制拍） */
static NEUQ_FIFO_t bluetooth_fifo;
static uint8_t     bluetooth_buffer[CAR_BLUETOOTH_FIFO_SIZE];

/* 蓝牙接收故障标志：1 表示 FIFO 溢出或串口错误，由控制拍统一处理 */
static volatile uint8_t bluetooth_rx_fault;

/* 视觉协议解析上下文：保存会话、序号、观测、四份数独子块和统计计数 */
static CarVision vision;

/* 视觉接收 FIFO 与其底层存储：与蓝牙链路完全独立 */
static NEUQ_FIFO_t vision_fifo;
static uint8_t     vision_buffer[CAR_VISION_FIFO_SIZE];

/* 视觉接收故障标志：1 表示 FIFO 溢出或串口错误，由控制拍作废视觉数据 */
static volatile uint8_t vision_rx_fault;

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarLink_Init
 * 功能说明    初始化视觉协议上下文、蓝牙接收 FIFO 和视觉接收 FIFO。
 * 用法        上电初始化阶段调用一次，必须早于任何接收回调。
 * 参数说明    无
 * 返回说明    无
 * 示例        CarLink_Init();
 * 备注        本函数只清软件状态，不配置串口外设。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarLink_Init(void)
{
    CarVision_Init(&vision);                                /* 视觉上下文清零：无会话、无观测 */
    NEUQ_FIFO_init(&vision_fifo, vision_buffer, sizeof(vision_buffer));
    NEUQ_FIFO_init(&bluetooth_fifo, bluetooth_buffer, sizeof(bluetooth_buffer));
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_UART_RX_Handler
 * 功能说明    蓝牙串口接收回调：把收到的单字节写入接收 FIFO；写入失败（溢出）时置位接收故障标志。
 * 用法        在 UART4 接收中断（库回调 TS_UART_3_RX_CALLBACK）中调用，只做入队，不做解析。
 * 参数说明    byte —— 收到的字节
 * 返回说明    无
 * 示例        void TS_UART_3_RX_CALLBACK(uint8_t dat) { Car_UART_RX_Handler(dat); }
 * 备注        中断上下文与控制拍共享 FIFO，NEUQ_FIFO 内部已按单生产者/单消费者设计。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_UART_RX_Handler(uint8_t byte)
{
    if (NEUQ_FIFO_write_byte(&bluetooth_fifo, byte) != NEUQ_FIFO_OK) {
        bluetooth_rx_fault = 1U;                            /* 溢出：交给控制拍锁定故障并清空 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_Vision_UART_RX_Handler
 * 功能说明    视觉串口接收回调：把香橙派发来的单字节写入独立视觉 FIFO；
 *             写入失败时置位视觉接收故障标志。
 * 用法        在 USART3 接收中断（库回调 TS_UART_2_RX_CALLBACK）中调用。
 * 参数说明    byte —— 收到的字节
 * 返回说明    无
 * 示例        void TS_UART_2_RX_CALLBACK(uint8_t dat) { Car_Vision_UART_RX_Handler(dat); }
 * 备注        视觉链路与蓝牙链路完全独立；视觉数据不能启动电机。
 *-------------------------------------------------------------------------------------------------------------------*/
void Car_Vision_UART_RX_Handler(uint8_t byte)
{
    if (NEUQ_FIFO_write_byte(&vision_fifo, byte) != NEUQ_FIFO_OK) {
        vision_rx_fault = 1U;                               /* 溢出：交给控制拍作废视觉数据 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_GetVisionObservation
 * 功能说明    在关中断临界区内读取一份视觉道路观测快照，保证读取期间数据不会被接收路径改写。
 * 用法        任务/控制层需要视觉观测时调用；返回 0 表示无数据或已过期。
 * 参数说明    out —— 输出观测结构指针
 * 返回说明    1 成功；0 无有效观测或参数为空
 * 示例        CarVisionObservation obs;
 *             if (Car_GetVisionObservation(&obs)) { use(obs.lateral_m); }
 * 备注        临界区很短（仅一次结构体拷贝），但仍建议不要在中断内调用。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t Car_GetVisionObservation(CarVisionObservation *out)
{
    uint32_t saved  = __get_PRIMASK();                      /* 保存当前中断屏蔽状态 */
    uint8_t  result;                                        /* 快照读取结果 */

    __disable_irq();                                        /* 进入临界区，防止视觉接收路径改写缓存 */
    result = CarVision_GetObservation(&vision, HAL_GetTick(), out);
    __set_PRIMASK(saved);                                   /* 恢复原来的中断屏蔽状态 */

    return result;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    Car_GetVisionTile
 * 功能说明    在关中断临界区内读取指定编号的数独子块快照。
 * 用法        需要拼合数独题目时按编号 0~3 调用；返回 0 表示该子块尚未收到。
 * 参数说明    index —— 子块编号 0~3
 *             out   —— 输出子块结构指针
 * 返回说明    1 成功；0 编号非法、无数据或参数为空
 * 示例        CarVisionTile tile;
 *             if (Car_GetVisionTile(0, &tile)) { show(tile.cells); }
 * 备注        子块在本会话内长期有效，是否集齐四块由上层判断。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out)
{
    uint32_t saved  = __get_PRIMASK();                      /* 保存当前中断屏蔽状态 */
    uint8_t  result;                                        /* 快照读取结果 */

    __disable_irq();                                        /* 进入临界区，防止视觉接收路径改写缓存 */
    result = CarVision_GetTile(&vision, index, HAL_GetTick(), out);
    __set_PRIMASK(saved);                                   /* 恢复原来的中断屏蔽状态 */

    return result;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarLink_ProcessBluetooth
 * 功能说明    在控制拍中处理蓝牙接收数据：有接收故障时锁定 CAR_RX_FAULT 并清空 FIFO；
 *             否则最多解析 CAR_BLUETOOTH_DRAIN_MAX 个字节。
 * 用法        由 5ms 控制拍在采样编码器之后、Car_Tick 之前调用。
 * 参数说明    core —— 整车控制核心对象指针
 * 返回说明    无
 * 示例        CarLink_ProcessBluetooth(&car);
 * 备注        单拍解析字节数有上限；故障时本拍不再执行任何命令，保证停车优先。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarLink_ProcessBluetooth(Car_Core *core)
{
    uint8_t  byte;                                          /* 当前取出的蓝牙字节 */
    unsigned n;                                             /* 本拍已处理的字节数 */

    if (bluetooth_rx_fault != 0U) {
        Car_Fault(core, CAR_RX_FAULT);                      /* 接收异常：锁定故障并停车 */
        while (NEUQ_FIFO_read_byte(&bluetooth_fifo, &byte) == NEUQ_FIFO_OK) {
            /* 清空 FIFO 中的残留字节 */
        }
        bluetooth_rx_fault = 0U;                            /* 故障只处理一次 */
        return;
    }

    for (n = 0U; (n < CAR_BLUETOOTH_DRAIN_MAX) &&
                 (NEUQ_FIFO_read_byte(&bluetooth_fifo, &byte) == NEUQ_FIFO_OK); n++) {
        Car_FeedByte(core, byte);                           /* 逐字节交给命令解析状态机 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarLink_ProcessVision
 * 功能说明    在控制拍中处理视觉接收数据：有接收故障时作废视觉数据并清空 FIFO；
 *             否则最多解析 CAR_VISION_DRAIN_MAX 个字节。
 * 用法        由 5ms 控制拍在电机输出之后调用；只有整车停车时才允许绑定新的视觉会话。
 * 参数说明    allow_hello —— 非 0 表示允许处理 HELLO（停车状态传 1，运行状态传 0）
 * 返回说明    无
 * 示例        CarLink_ProcessVision(car.state == CAR_STOP);
 * 备注        视觉处理放在电机输出之后，保证即使视觉解析耗时也不会延迟本拍电机更新。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarLink_ProcessVision(uint8_t allow_hello)
{
    uint8_t  byte;                                          /* 当前取出的视觉字节 */
    unsigned n;                                             /* 本拍已处理的字节数 */
    uint32_t now;                                           /* 本拍毫秒时基，整批字节共用 */

    if (vision_rx_fault != 0U) {
        CarVision_Invalidate(&vision);                      /* 接收异常：旧观测与子块作废 */
        for (n = 0U; (n < sizeof(vision_buffer)) &&
                     (NEUQ_FIFO_read_byte(&vision_fifo, &byte) == NEUQ_FIFO_OK); n++) {
            /* 清空 FIFO 中的残留字节 */
        }
        vision_rx_fault = 0U;                               /* 故障只处理一次 */
        return;
    }

    now = HAL_GetTick();                                    /* 读取一次时钟，本拍所有字节共用 */
    for (n = 0U; (n < CAR_VISION_DRAIN_MAX) &&
                 (NEUQ_FIFO_read_byte(&vision_fifo, &byte) == NEUQ_FIFO_OK); n++) {
        CarVision_Feed(&vision, byte, now, allow_hello);    /* 流式拆帧并更新观测/子块 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    HAL_UART_ErrorCallback
 * 功能说明    HAL 串口错误回调：识别蓝牙 UART4 或视觉 USART3 的溢出/帧错误，置位对应故障标志，
 *             并调用 NEUQ 库重新开启单字节中断接收，让链路能够继续工作。
 * 用法        HAL 库在串口错误时自动调用，无需用户注册；属于弱函数重写。
 * 参数说明    huart —— 发生错误的串口句柄
 * 返回说明    无
 * 示例        // 由 HAL 自动调用，例如 UART4 溢出后：
 *             // bluetooth_rx_fault = 1，控制拍会锁定 CAR_RX_FAULT 并清空缓冲
 * 备注        蓝牙侧错误会在下一个 5ms 控制拍触发 CAR_RX_FAULT 保护停车；
 *             视觉侧错误只作废视觉数据，不影响车辆运动状态。
 *-------------------------------------------------------------------------------------------------------------------*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        bluetooth_rx_fault = 1U;                            /* 蓝牙链路故障标志 */
        NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_UART_INDEX,
                                      CAR_BLUETOOTH_RX_PRIORITY);
    } else if (huart->Instance == USART3) {
        vision_rx_fault = 1U;                               /* 视觉链路故障标志 */
        NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_VISION_UART_INDEX,
                                      CAR_VISION_RX_PRIORITY);
    } else {
        /* 其他串口未在本项目使用，不处理 */
    }
}
