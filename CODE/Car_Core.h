#ifndef CAR_CORE_H
#define CAR_CORE_H

#include <stdint.h>

#include "Car_PID.h"
#include "Car_Reference.h"
#include "Car_Vision.h"

/*
 * ============================================================================
 * 文件名称：Car_Core.h
 * ============================================================================
 * 模块功能：声明蓝牙差速车的纯算法控制核心 Car_Core 及其对外接口。
 *           实现见 Car_Core.c；硬件适配见 Car_Hardware.c。
 * ============================================================================
 */

/*
 * 整车状态。
 *   状态转换：初始化 -> STOP；STOP + ARM -> RUN；任意状态 + STOP -> STOP；
 *   检测到异常 -> FAULT；FAULT 必须先 STOP 清除故障，再 ARM 启动。
 *   启动后还需要 WHEELS 命令给定转速。
 *   枚举值同时用于遥测，因此顺序不能随意修改。
 */
enum
{
    CAR_STOP,  /* 0：停车，允许修改 PID 参数 */
    CAR_RUN,   /* 1：运行，接受转速与心跳命令 */
    CAR_FAULT  /* 2：故障，输出已清零，需 STOP 清除 */
};

/*
 * 故障码。
 *   FAULT 状态保留第一次触发的原因；
 *   枚举值同时用于遥测，因此顺序不能随意修改。
 */
enum
{
    CAR_NO_FAULT,        /* 0：无故障 */
    CAR_LINK_FAULT,      /* 1：运行通信超时 */
    CAR_STALL_FAULT,     /* 2：堵转 */
    CAR_DIRECTION_FAULT, /* 3：转向异常 */
    CAR_RX_FAULT,        /* 4：蓝牙接收异常 */
    CAR_TIMING_FAULT,    /* 5：控制周期异常 */
    CAR_CONTROLLER_FAULT, /* 6：控制器配置或数值异常 */
    CAR_VISION_FAULT      /* 7：自主模式视觉无效、过期或会话变化 */
};

enum { CAR_MODE_MANUAL, CAR_MODE_AUTO };

/*
 * 蓝牙协议摘要。
 *
 *   帧格式：C3 3C CMD LEN DATA CRC8
 *   CRC8：多项式 0x07，初值 0，覆盖 CMD 至 DATA。
 *
 *   命令：
 *     1 ARM     ：启动，无负载；
 *     2 STOP    ：停止/清故障，无负载；
 *     3 WHEELS  ：双轮目标转速，两个大端 int16；
 *     4 PID     ：设置 PID，轮号 0/1 + 三个大端 uint16（Kp/Ki/Kd，放大 100 倍）；
 *     5 HEARTBEAT：心跳，无负载；
 *     6 AUTO_ARM：停车时启动低速视觉模式，直径 mm/轮距 mm/速度 mm/s 三个大端 uint16。
 *
 *   result：0 成功，1 状态不允许，2 参数无效，3 帧或命令无效。
 */
typedef struct
{
    /* ---------- 双轮控制状态 ---------- */
    Car_Pid_Struct pid[2];   /* 左右轮速度 PID，下标 0 左、1 右 */
    float target[2];         /* 目标转速，单位 rpm，下标 0 左、1 右 */
    float reference[2];      /* 规划器输出的参考转速，单位 rpm，下标 0 左、1 右 */
    float speed[2];          /* 滤波后的实测转速，单位 rpm，下标 0 左、1 右 */
    float output[2];         /* 带符号 PWM 输出，正负号表示方向，下标 0 左、1 右 */

    /* ---------- 计时与保护，单位均为微秒 ---------- */
    uint32_t link_us;        /* 运行通信看门狗计时 */
    uint32_t stall_us[2];    /* 左右轮堵转计时 */
    uint32_t direction_us[2];/* 左右轮方向异常计时 */
    uint32_t frame_us;       /* 蓝牙帧字节间隔计时 */

    /* 双轮共享进度的五次参考规划器，生成 reference 和参考加速度。 */
    CarReference profile;

    /* 转速绝对值的慢速滤波结果，用于识别换向时是否仍在减速。 */
    float slow_speed[2];

    /* 进入命令执行阶段的帧数；被状态或参数拒绝的命令也计数。 */
    uint32_t frames;

    /* ---------- 状态与协议字段 ---------- */
    uint8_t state;           /* 整车状态，取值见上方 CAR_STOP/CAR_RUN/CAR_FAULT */
    uint8_t fault;           /* 故障码，取值见上方 CAR_*_FAULT */
    uint8_t result;          /* 最近一条命令或帧的处理结果 */
    uint8_t mode;            /* MANUAL 或 AUTO；STOP 后回到 MANUAL */
    uint8_t vision_present;  /* 通过几何/有效位检查，年龄还需单独检查 */
    uint16_t vision_sequence;
    uint32_t vision_session;
    uint32_t auto_session;   /* 启动时固定会话，中途不允许更换 */
    uint32_t vision_age_us;  /* 重复读取同一帧不得刷新年龄 */
    float vision_curvature;  /* 1/m，左转为正 */
    float auto_diameter_m;
    float auto_track_m;
    float auto_speed_m_s;

    /* 蓝牙解析上下文。 */
    uint8_t parse_state;     /* 解析状态机当前状态 0~5 */
    uint8_t cmd;             /* 当前帧的命令号 */
    uint8_t length;          /* 当前帧期望的负载长度 */
    uint8_t index;           /* 已接收的负载字节数 */
    uint8_t crc;             /* 当前帧累计 CRC-8 */
    uint8_t data[7];         /* 负载缓存，最大 7 字节，对应 PID 命令 */
} Car_Core;

/**
 * @brief 清空控制对象并装载 Car_Config.h 中的默认参数。
 * @param c 待初始化对象；原有目标、测速、协议状态及统计都会清零。
 * @note 正常进入 STOP；控制器配置失败时进入 FAULT。
 */
void Car_Init(Car_Core *c);

/** 由控制上下文传入已由 CarVision 校验的快照；NULL 表示无有效观测。
 * now_ms 与快照 received_ms 使用相同时钟。不使能、不直接写输出。
 * 重读相同 session/sequence 不刷新年龄；自主故障须先 STOP 再重新使能。 */
void Car_SetVisionObservation(Car_Core *c, const CarVisionObservation *observation,
                              uint32_t now_ms);

/**
 * @brief 对一个字节更新 CRC-8，返回新的累计值。
 * @param crc  首字节传 0，其余字节传上一次返回值。
 * @param byte 当前参与计算的字节。
 * @return 更新后的 CRC-8 值。
 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte);

/**
 * @brief 输入一个蓝牙字节；只有完整合法帧才会执行命令。
 * @param c    控制对象。
 * @param byte 刚收到的原始字节。
 * @note 无返回值，通过 c->result、state、frames 等观察结果。
 *       此函数可能修改目标和状态，应与 Car_TickDt 在同一上下文调用。
 */
void Car_FeedByte(Car_Core *c, uint8_t byte);

/**
 * @brief 使用 CAR_PERIOD_MS 固定周期更新，供仿真或固定周期调用者使用。
 * @param c           控制对象。
 * @param left_count  左轮本周期的有符号计数增量，不能传累计计数。
 * @param right_count 右轮本周期的有符号计数增量。
 */
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count);

/**
 * @brief 使用实际周期更新测速、控制输出和保护状态。
 * @param c           控制对象。
 * @param left_count  左输出轴编码器增量，前进为正。
 * @param right_count 右输出轴编码器增量，前进为正。
 * @param dt_us       与计数增量对应的时间跨度，单位微秒。
 * @note 周期须在配置的闭区间内，否则进入 CAR_TIMING_FAULT。
 *       结果写入 c->output；本函数不操作 GPIO/PWM，硬件层负责应用输出。
 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count,
                uint32_t dt_us);

/**
 * @brief 锁存故障并清零运动目标、参考值及输出；重复调用不覆盖首个故障码。
 * @param c     控制对象。
 * @param fault 调用者传入非零的 CAR_*_FAULT 故障码。
 * @note 保留实测转速；清零输出不代表轮子已经物理停止。
 */
void Car_Fault(Car_Core *c, uint8_t fault);

#endif
