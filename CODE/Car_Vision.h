#ifndef CAR_VISION_H
#define CAR_VISION_H
#include <stdint.h>

/* 香橙派 -> F407 的独立视觉链路，所有线上多字节整数均为大端。
 * A5 5A | version:u8 type:u8 length:u16 seq:u16 session:u32 age_ms:u16
 * | payload | CRC16-CCITT-FALSE（覆盖 version 至 payload）。
 * HELLO=0（负载 0 字节、age 为 0），TRACK=1（16 字节），SUDOKU_TILE=2（10 字节）。
 * 仅停车时允许 HELLO 绑定非零会话；本接口中的消息不能启动电机或写入 PWM。 */
/* 固定开销 16 字节 = 帧头 2 + 元数据 12 + CRC 2，最大完整帧 48 字节。
 * 字节间隔超过 50 ms 丢弃半帧；道路观测年龄超过 200 ms 判为过期。 */
#define CAR_VISION_MAX_PAYLOAD 32U
#define CAR_VISION_MAX_FRAME   (16U+CAR_VISION_MAX_PAYLOAD)
#define CAR_VISION_FRAME_GAP_MS 50U
#define CAR_VISION_MAX_AGE_MS   200U
/* valid_mask 为位掩码，可按位组合；只有置位的几何量才允许供业务层使用。 */
enum { VISION_LATERAL_VALID=1, VISION_HEADING_VALID=2,
       VISION_CURVATURE_VALID=4, VISION_DISTANCE_VALID=8 };
/* 元素值按顺序为 0~8：无、环岛、十字、断路、绿、黄、红、终点候选、Tag。 */
enum { VISION_ELEMENT_NONE, VISION_ELEMENT_RING, VISION_ELEMENT_CROSS,
       VISION_ELEMENT_GAP, VISION_ELEMENT_GREEN, VISION_ELEMENT_YELLOW,
       VISION_ELEMENT_RED, VISION_ELEMENT_FINISH, VISION_ELEMENT_TAG };
typedef struct {
    uint32_t received_ms, session;
    /* received_ms 是解析时传入的本机时间，processing_age_ms 是发送端报告的数据年龄；
     * valid_mask 的各位决定横向偏差、航向、曲率、距离字段是否有效。 */
    uint16_t sequence, processing_age_ms, valid_mask;
    /* lateral_m：横向误差（米，左正）；heading_rad：航向误差（弧度，逆时针正）；
     * curvature_inv_m：曲率（1/米）；distance_m：元素距离（米）；confidence：[0,1]。
     * received_ms 不代表相机曝光或 UART 首字节到达时间。 */
    float lateral_m, heading_rad, curvature_inv_m, distance_m, confidence;
    uint8_t element, tag_id; /* 元素枚举；Tag 编号 0~7，255 表示未识别。 */
} CarVisionObservation;
typedef struct {
    uint32_t received_ms, session;
    uint16_t sequence;
    /* cells 按行排列九格：0 空白，1~6 为数字；valid 表示本会话缓存是否可用。 */
    uint8_t cells[9], valid;
} CarVisionTile;
typedef struct {
    /* buffer/used：尚未消费的串口字节及其数量；其余三个标志分别表示
     * 已绑定会话、已有有效业务序号、已有道路观测（仍需另外检查时效）。 */
    uint8_t buffer[CAR_VISION_MAX_FRAME], used, session_valid, have_sequence, have_observation;
    uint16_t last_sequence; /* 道路与数独消息共享的最近业务序号，HELLO 不推进它。 */
    /* accepted_frames：已接受帧（含 HELLO）；bad_frames：长度越界/CRC 错误；
     * rejected_frames：CRC 正确但版本、会话、序号或业务字段不合法。 */
    uint32_t last_byte_ms, session, accepted_frames, bad_frames, rejected_frames;
    CarVisionObservation observation;
    CarVisionTile tiles[4]; /* 按左上、右上、左下、右下缓存，同一索引的新数据覆盖旧值。 */
} CarVision;
/**
 * @brief 清空解析器、会话、统计和全部缓存；v 须为非空对象。
 * @note 后续须先接收停车状态下的合法 HELLO，才会接受业务观测。
 */
void CarVision_Init(CarVision *v);
/**
 * @brief 接收异常后作废半帧、道路观测和四个数独块。
 * @note 保留会话、最近序号和统计；缓存内容可能仍在内存中，但有效标志已清除。
 */
void CarVision_Invalidate(CarVision *v);
/**
 * @brief 更新一个字节的 CRC16-CCITT-FALSE，返回新的累计值。
 * @param crc 首字节传 0xFFFF，之后传上次结果；byte 为待计算字节。
 * @note 多项式 0x1021，不反射，最终不异或；线上的 CRC 高字节在前。
 */
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte);
/* 状态修改须在同一执行上下文完成；跨中断读取应通过硬件层快照接口。 */
/**
 * @brief 输入一个视觉字节，完成同步、校验和缓存更新。
 * @param v 非空解析器；byte 为接收字节；now_ms 为本机当前毫秒时间。
 * @param allow_hello 非零允许 HELLO，硬件层仅在 CAR_STOP 时传入非零值。
 * @note 无返回值，可读统计观察结果；不直接执行电机动作或发送 ACK。
 */
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello);
/* 返回有效快照不代表识别到道路：新鲜数据也可能 valid_mask=0。 */
/**
 * @brief 复制通过时效检查的最新道路观测；v 非空，now_ms 为当前毫秒时间。
 * @return 1 成功；0 表示 out 为 NULL、无观测或过期，out 保持原值。
 * @note 年龄按发送端处理时间加本地解析后时间计算，不含无法测量的排队延迟。
 */
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out);
/* 数独块在本会话内保留，由调用者判断四块拼图是否收齐。 */
/**
 * @brief 复制索引 0~3 的数独块，不使用道路观测的 200 ms 时效规则。
 * @return 1 成功；0 表示参数无效、缓存失效或时间差达到半个 32 位计时范围。
 * @note v 须非空；失败不修改 out。更新会话或接收异常会使旧块失效。
 */
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out);
#endif
