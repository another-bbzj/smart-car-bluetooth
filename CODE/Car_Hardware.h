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

/* ========================================================================== */
/*                          板载菜单调试接口                                   */
/* ========================================================================== */

/* 控制快照：全部字段在临界区内一次拷贝，供主循环菜单显示。 */
typedef struct
{
    uint8_t  state;              /* CAR_STOP / CAR_RUN / CAR_FAULT */
    uint8_t  fault;              /* 故障码 */
    uint8_t  mode;               /* CAR_MODE_MANUAL / CAR_MODE_AUTO */
    uint8_t  result;             /* 最近命令结果 */
    uint8_t  vision_present;     /* 1 表示有通过几何检查的视觉观测 */
    uint16_t vision_sequence;
    uint32_t vision_session;
    uint32_t vision_age_ms;
    float    vision_curvature;

    float target[2];
    float reference[2];
    float speed[2];
    float output[2];
    float kp[2];
    float ki[2];
    float kd[2];

    float rpm_limit;
    float pwm_limit;
    float accel_limit;
    float jerk_limit;
    float cpr;
} CarDebugSnapshot;

/**
 * @brief 复制一份控制快照，调用时短暂屏蔽中断。
 * @param out 非空输出；失败时不改写。
 * @return 1 成功；0 参数无效。
 */
uint8_t Car_Debug_GetSnapshot(CarDebugSnapshot *out);

/**
 * @brief 停车时设置单轮 PID，参数校验与蓝牙 CMD4 相同。
 * @return 1 成功；0 非停车、轮号或参数非法。
 */
uint8_t Car_Debug_SetPid(uint8_t wheel, float kp, float ki, float kd);

/**
 * @brief 停车时设置限速/PWM/加速度/加加速度。
 * @return 1 成功；0 非停车或参数越界。
 */
uint8_t Car_Debug_SetLimits(float rpm_limit, float pwm_limit,
                            float accel_limit, float jerk_limit);

/**
 * @brief 停车时设置编码器 CPR。
 * @return 1 成功；0 非停车或参数非法。
 */
uint8_t Car_Debug_SetCpr(float cpr);

/**
 * @brief 菜单 STOP：等价于蓝牙 CMD2，清软件故障并回到停车。
 * @note 不会消除硬件故障源，也不会保证车轮已物理停止。
 */
void Car_Debug_Stop(void);

/**
 * @brief 进入 Flash 擦写临界流程：仅允许 STOP，暂停 PIT 中断。
 * @return 1 允许擦写；0 非 STOP，调用方不得写 Flash。
 * @note 必须与 Car_Debug_FlashEnd() 成对调用；期间 SysTick 仍可运行。
 */
uint8_t Car_Debug_FlashBegin(void);

/**
 * @brief 结束 Flash 擦写：重建控制时基、清 PIT 挂起标志并恢复中断。
 */
void Car_Debug_FlashEnd(void);

#endif
