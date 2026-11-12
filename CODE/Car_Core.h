#ifndef CAR_CORE_H
#define CAR_CORE_H
#include <stdint.h>
#include "Car_PID.h"
#include "Car_Reference.h"
/* 状态转换：初始化 -> STOP；STOP + ARM -> RUN；任意状态 + STOP -> STOP。
 * 故障 -> FAULT；FAULT 必须先 STOP 清故障，再 ARM 启动，ARM 不会恢复旧目标。
 * 启动后还需 WHEELS 给定转速；STOP/RUN/FAULT 的枚举值同时用于遥测。 */
enum { CAR_STOP, CAR_RUN, CAR_FAULT };
/* 故障码按声明顺序为 0~6：无故障、运行通信超时、堵转、方向异常、
 * 蓝牙接收异常、控制周期异常、控制器异常；FAULT 状态保留首次故障原因。 */
enum { CAR_NO_FAULT, CAR_LINK_FAULT, CAR_STALL_FAULT,
       CAR_DIRECTION_FAULT, CAR_RX_FAULT, CAR_TIMING_FAULT, CAR_CONTROLLER_FAULT };
/* 蓝牙命令：1 启动，2 停止/清故障，3 双轮转速（两个大端 int16），
 * 4 设置 PID（轮号 0/1 + 大端 uint16 的 Kp/Ki/Kd，增益放大 100 倍），5 心跳。
 * 帧格式：C3 3C CMD LEN DATA CRC8（多项式 0x07，初值 0，覆盖 CMD 至 DATA）。
 * result：0 成功，1 状态不允许，2 参数无效，3 帧或命令无效。 */
/* 算法状态不依赖 HAL。调用者提供有效的非空对象指针；所有修改串行执行。
 * 硬件层由 PIT 中断统一修改本对象，跨上下文读取时需自行保证快照一致性。 */
typedef struct {
    Car_Pid_Struct pid[2];
    /* 双轮下标：0 左、1 右。目标/规划参考/反馈单位 rpm；输出为带符号 PWM 数值。 */
    float target[2], reference[2], speed[2], output[2];
    /* 单位均为微秒：运行通信超时、双轮堵转、双轮方向异常、帧字节间隔。 */
    uint32_t link_us, stall_us[2], direction_us[2], frame_us;
    CarReference profile; /* 两轮共享进度的参考规划器，生成 reference 与参考加速度。 */
    /* 转速绝对值的慢速滤波结果，用于识别换向时是否仍在减速。 */
    float slow_speed[2];
    uint32_t frames; /* 进入命令执行阶段的帧数；状态或参数拒绝也计数。 */
    /* state/fault 分别对应上方状态与故障枚举；result 是最近命令/帧处理结果。 */
    uint8_t state, fault, result;
    /* 蓝牙解析上下文：命令、期望负载长度、已收字节数、累计 CRC 和最大 7 字节负载。 */
    uint8_t parse_state, cmd, length, index, crc, data[7];
} Car_Core;
/**
 * @brief 清空控制对象并装载 Car_Config.h 中的默认参数。
 * @param c 待初始化对象；原有目标、测速、协议状态及统计都会清零。
 * @note 正常进入 STOP；控制器配置失败时进入 FAULT。
 */
void Car_Init(Car_Core *c);
/**
 * @brief 对一个字节更新 CRC-8，返回新的累计值。
 * @param crc 首字节传 0，其余字节传上一次返回值；byte 为当前字节。
 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte);
/**
 * @brief 输入一个蓝牙字节；只有完整合法帧才会执行命令。
 * @note 无返回值，通过 c->result、state、frames 等观察结果。
 *       此函数可能修改目标和状态，应与 Car_TickDt 在同一上下文调用。
 */
void Car_FeedByte(Car_Core *c, uint8_t byte);
/**
 * @brief 使用 CAR_PERIOD_MS 固定周期更新，供仿真或固定周期调用者使用。
 * @param left_count 左轮本周期的有符号计数增量，不能传累计计数。
 * @param right_count 右轮本周期的有符号计数增量。
 */
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count);
/**
 * @brief 使用实际周期更新测速、控制输出和保护状态。
 * @param left_count 左输出轴编码器增量，前进为正。
 * @param right_count 右输出轴编码器增量，前进为正。
 * @param dt_us 与计数增量对应的时间跨度，单位微秒。
 * @note 周期须在配置的闭区间内，否则进入 CAR_TIMING_FAULT。
 *       结果写入 c->output；本函数不操作 GPIO/PWM，硬件层负责应用输出。
 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count, uint32_t dt_us);
/**
 * @brief 锁存故障并清零运动目标、参考值及输出；重复调用不覆盖首个故障码。
 * @param fault 调用者传入非零的 CAR_*_FAULT 故障码。
 * @note 保留实测转速；清零输出不代表轮子已经物理停止。
 */
void Car_Fault(Car_Core *c, uint8_t fault);
#endif
