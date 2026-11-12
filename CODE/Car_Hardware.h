#ifndef CAR_HARDWARE_H
#define CAR_HARDWARE_H
#include <stdint.h>
#include "Car_Vision.h"
/**
 * @brief 底盘入口：初始化控制器、FIFO、GPIO、PWM、编码器及串口，最后启动 PIT。
 * @note 由 User_Main 调用一次；随后永久循环发送遥测，正常不会返回。
 */
void Car_Main(void);
/**
 * @brief PIT_CH1 回调入口：取编码器增量、解析命令、更新并输出控制量。
 * @note 初始化完成后由定时中断调用，不应由主循环额外调用或重入。
 *       视觉解析和遥测入队在电机更新之后执行。
 */
void Car_PIT_Handler(void);
/**
 * @brief 蓝牙单字节接收回调，只写接收 FIFO。
 * @param byte 刚收到的原始字节；FIFO 写入失败会标记故障，由下次 PIT 处理。
 */
void Car_UART_RX_Handler(uint8_t byte);
/**
 * @brief 视觉单字节接收回调，只入队；溢出由下次 PIT 使视觉缓存失效。
 */
void Car_Vision_UART_RX_Handler(uint8_t byte);
/* 通过临界区读取快照。观测缺失或过期返回 0，字段还须检查 valid_mask；
 * 数独块在当前会话内保留，接口不按观测的 200 ms 时效自动淘汰。 */
/**
 * @brief 复制道路观测快照，调用时短暂屏蔽中断。
 * @param out 接收副本的对象；NULL 会返回 0。
 * @return 1 有可用快照；0 无数据或超时，失败时不改写 out。
 * @note 成功后仍需检查 valid_mask 和 confidence，不应直接当作行驶指令。
 */
uint8_t Car_GetVisionObservation(CarVisionObservation *out);
/**
 * @brief 复制当前会话的数独子块快照，调用时短暂屏蔽中断。
 * @param index 0 左上、1 右上、2 左下、3 右下；out 为非空输出对象。
 * @return 1 成功；0 参数无效或数据不可用，失败时不改写 out。
 */
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out);
#endif
