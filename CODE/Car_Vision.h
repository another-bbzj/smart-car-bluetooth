#ifndef CAR_VISION_H
#define CAR_VISION_H

/*********************************************************************************************************************
* 文件名称          Car_Vision.h
* 功能说明          香橙派 -> STM32F407 的视觉观测接收接口（v1）。
*
*                   本接口只接收"识别结果"和道路观测，不接收原始图像；
*                   任何视觉消息都不能启动电机或直接写 PWM，电机只能由 Car_Core 控制。
*
*                   线协议（所有多字节整数均为大端）：
*                   A5 5A | 版本:u8 | 类型:u8 | 长度:u16 | 序号:u16 | 会话:u32 | 处理时延:u16
*                         | 负载 | CRC16-CCITT-FALSE（覆盖"版本"到"负载"末尾）
*
*                   · HELLO       = 0，长度 0、时延 0：建立/切换会话，仅允许在停车状态绑定
*                   · TRACK       = 1，长度 16：道路几何与元素观测
*                   · SUDOKU_TILE = 2，长度 10：3×3 数独子块
*
*                   新鲜度：接收时延与视觉端处理时延之和超过 CAR_VISION_MAX_AGE_MS 即视为过期。
*                   本文件只做协议解析与缓存，不参与车辆运动决策。
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            first version
* 2026-11-12                            英文注释改为中文，补充字段说明
********************************************************************************************************************/
#include <stdint.h>

#define CAR_VISION_MAX_PAYLOAD   32U                    /* 单帧最大负载字节数（含 TRACK 的 16 字节） */
#define CAR_VISION_MAX_FRAME     (16U + CAR_VISION_MAX_PAYLOAD) /* 单帧最大总长：14 字节头 + 负载 + 2 字节 CRC */
#define CAR_VISION_FRAME_GAP_MS  50U                    /* 帧内字节间隔超过 50ms 即丢弃残帧 */
#define CAR_VISION_MAX_AGE_MS    200U                   /* 观测最大允许年龄：接收时延 + 处理时延 */

/* TRACK 观测的有效字段位：某位为 0 表示该字段本次不可用，不能把 0 当作"测量值为 0" */
enum {
    VISION_LATERAL_VALID   = 1,     /* bit0：横向偏差有效 */
    VISION_HEADING_VALID   = 2,     /* bit1：航向偏差有效 */
    VISION_CURVATURE_VALID = 4,     /* bit2：曲率有效 */
    VISION_DISTANCE_VALID  = 8      /* bit3：元素距离有效 */
};

/* 视觉元素类型：只是"候选观测"，不是控制命令 */
enum {
    VISION_ELEMENT_NONE = 0,        /* 未识别到元素 */
    VISION_ELEMENT_RING,            /* 环岛 */
    VISION_ELEMENT_CROSS,           /* 十字路口 */
    VISION_ELEMENT_GAP,             /* 断路 */
    VISION_ELEMENT_GREEN,           /* 绿色障碍 */
    VISION_ELEMENT_YELLOW,          /* 黄色障碍 */
    VISION_ELEMENT_RED,             /* 红色障碍 */
    VISION_ELEMENT_FINISH,          /* 终点候选 */
    VISION_ELEMENT_TAG              /* AprilTag / Tag36h11 */
};

/* 一帧道路与元素观测（TYPE=1，负载 16 字节） */
typedef struct {
    uint32_t received_ms;           /* 本机收到该帧的毫秒时基（HAL_GetTick），用于新鲜度判断 */
    uint32_t session;               /* 视觉端会话号：每次上电/新比赛由香橙派生成的非零值 */
    uint16_t sequence;              /* 会话内 16 位递增序号，用于拒绝重复/倒退帧 */
    uint16_t processing_age_ms;     /* 视觉端从采集到发送前的处理时延（ms） */
    uint16_t valid_mask;            /* 有效字段位，见 VISION_*_VALID */
    float lateral_m;                /* 横向偏差（m），左正右负 */
    float heading_rad;              /* 航向偏差（rad），逆时针为正 */
    float curvature_inv_m;          /* 道路曲率（1/m） */
    float distance_m;               /* 元素或目标距离（m） */
    float confidence;               /* 置信度 0.0 ~ 1.0，由协议中的 0~1000 换算 */
    uint8_t element;                /* 元素类型，见 VISION_ELEMENT_* */
    uint8_t tag_id;                 /* Tag36h11 编号 0~7，255 表示未识别 */
} CarVisionObservation;

/* 一份 3×3 数独子块的识别结果（TYPE=2，负载 10 字节） */
typedef struct {
    uint32_t received_ms;           /* 本机收到该子块的毫秒时基 */
    uint32_t session;               /* 所属视觉会话号 */
    uint16_t sequence;              /* 该帧的会话内序号 */
    uint8_t  cells[9];              /* 9 个格子，按行排列：0 空白，1~6 为识别出的数字 */
    uint8_t  valid;                 /* 1 表示该子块已收到且有效 */
} CarVisionTile;

/* 视觉接收上下文：缓冲区、会话状态、统计计数和最近一次解析结果 */
typedef struct {
    uint8_t  buffer[CAR_VISION_MAX_FRAME]; /* 流式拆帧缓冲，存放尚未处理完的字节 */
    uint8_t  used;                  /* 缓冲中已有字节数 */
    uint8_t  session_valid;         /* 1 表示已经绑定会话，可以接收观测/子块 */
    uint8_t  have_sequence;         /* 1 表示 last_sequence 有效，需要做递增检查 */
    uint8_t  have_observation;      /* 1 表示 observation 中有一份可用观测 */
    uint16_t last_sequence;         /* 最近一次接收的序号，用于判断重复/倒退 */
    uint32_t last_byte_ms;          /* 上一个字节的到达时刻，用于帧间隔超时 */
    uint32_t session;               /* 当前绑定的会话号 */
    uint32_t accepted_frames;       /* 统计：通过校验并接受的帧数 */
    uint32_t bad_frames;            /* 统计：长度或 CRC 错误的帧数 */
    uint32_t rejected_frames;       /* 统计：格式正确但业务规则拒绝的帧数 */
    CarVisionObservation observation; /* 最近一次有效观测 */
    CarVisionTile tiles[4];         /* 四份数独子块缓存：0 左上，1 右上，2 左下，3 右下 */
} CarVision;

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Init
 * 功能说明    清空视觉接收上下文：缓冲、会话状态、统计计数、观测和四份子块全部归零。
 * 用法        上电初始化时调用一次，必须早于 CarVision_Feed。
 * 参数说明    v —— 视觉接收上下文指针
 * 返回说明    无
 * 示例        static CarVision vision;
 *             CarVision_Init(&vision);
 * 备注        初始化后没有有效会话，必须先在停车状态下收到 HELLO 才会接收观测数据。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarVision_Init(CarVision *v);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Invalidate
 * 功能说明    作废当前视觉数据：清空未处理字节、观测有效标志和四份数独子块；
 *             保留会话绑定、序号状态和统计计数。
 * 用法        串口错误、缓冲区溢出、需要丢弃旧数据时调用；同一会话可继续接收新的有效帧。
 * 参数说明    v —— 视觉接收上下文指针
 * 返回说明    无
 * 示例        CarVision_Invalidate(&vision);
 * 备注        1) 本函数不改变 accepted_frames / bad_frames / rejected_frames 等诊断计数；
 *             2) 会话绑定仍然有效，因此不会强制要求视觉端重新发送 HELLO。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarVision_Invalidate(CarVision *v);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Crc16
 * 功能说明    按 CRC-16/CCITT-FALSE（多项式 0x1021、初值 0xFFFF）逐字节更新校验值。
 * 用法        视觉帧接收时对 version 至 payload 逐字节迭代，再与帧尾 CRC 比较。
 * 参数说明    crc  —— 上一次 CRC，第一字节传 0xFFFF
 *             byte —— 本次参与计算的字节
 * 返回说明    更新后的 16 位 CRC
 * 示例        uint16_t crc = 0xFFFF;
 *             for (i = 2; i < total - 2; i++) crc = CarVision_Crc16(crc, buf[i]);
 * 备注        标准校验值：对 ASCII "123456789" 计算结果为 0x29B1。
 *-------------------------------------------------------------------------------------------------------------------*/
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Feed
 * 功能说明    逐字节接收视觉串口数据，按 A5 5A 帧头、长度、CRC16 做流式拆帧，
 *             整帧合法后解析并更新观测/子块缓存。
 * 用法        在视觉串口接收路径中每收到 1 字节调用一次；必须在单一上下文调用。
 * 参数说明    v           —— 视觉接收上下文指针
 *             byte        —— 本次收到的字节
 *             now_ms      —— 当前毫秒时基（HAL_GetTick）
 *             allow_hello —— 非 0 允许绑定新会话（仅在整车 STOP 时传 1）
 * 返回说明    无
 * 示例        CarVision_Feed(&vision, byte, HAL_GetTick(), car.state == CAR_STOP);
 * 备注        1) 长度超过 CAR_VISION_MAX_PAYLOAD 视为坏帧；
 *             2) 字节间隔超过 CAR_VISION_FRAME_GAP_MS 时丢弃半帧；
 *             3) 本接口只接收，不能启动电机或写 PWM。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_GetObservation
 * 功能说明    读取最近一帧道路观测的快照，并检查传输时延与视觉端处理时延之和是否仍在新鲜度窗口内。
 * 用法        控制/任务层需要道路偏差、航向、曲率、元素或距离时调用；返回 0 表示无可用观测。
 * 参数说明    v      —— 视觉接收上下文指针（只读）
 *             now_ms —— 当前毫秒时基
 *             out    —— 输出观测结构，成功时被完整写入
 * 返回说明    1 表示数据新鲜可用；0 表示从未收到、已失效或已过期
 * 示例        CarVisionObservation obs;
 *             if (CarVision_GetObservation(&vision, HAL_GetTick(), &obs)) {
 *                 // 再检查 obs.valid_mask 和 confidence
 *             }
 * 备注        1) 本函数不修改接收上下文，可在快照临界区内调用；
 *             2) valid_mask 为 0 表示链路正常但当前没有检测到道路，不等于通信失败；
 *             3) 判断使用无符号时间差，可安全跨越 32 位毫秒计数器回绕。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out);

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_GetTile
 * 功能说明    读取指定编号的数独子块快照（同一会话内一旦收到即长期保留）。
 * 用法        需要拼合四份数独子块时按 0~3 依次调用，由上层判断是否已集齐。
 * 参数说明    v      —— 视觉接收上下文指针（只读）
 *             index  —— 子块编号 0~3
 *             now_ms —— 当前毫秒时基，用于识别已回绕/异常的旧数据
 *             out    —— 输出子块结构
 * 返回说明    1 表示该子块有效；0 表示编号非法、尚未收到或数据异常
 * 示例        CarVisionTile tile;
 *             if (CarVision_GetTile(&vision, 2, HAL_GetTick(), &tile)) { ... }
 * 备注        子块没有单独的过期时间，会话切换或 Invalidate 时统一失效。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out);

#endif /* CAR_VISION_H */
