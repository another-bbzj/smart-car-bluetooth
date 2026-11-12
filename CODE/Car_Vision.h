#ifndef CAR_VISION_H
#define CAR_VISION_H

#include <stdint.h>

/*
 * ============================================================================
 * 文件名称：Car_Vision.h
 * ============================================================================
 * 模块功能：声明香橙派 -> STM32F407 的视觉数据接收协议和缓存结构。
 *
 * 线上帧格式（所有多字节整数均为大端）：
 *     A5 5A | version:u8 | type:u8 | length:u16 | seq:u16 | session:u32
 *           | age_ms:u16 | payload | CRC16-CCITT-FALSE
 *
 *   - CRC 覆盖 version 至 payload；
 *   - HELLO = 0：负载 0 字节、age 为 0；
 *   - TRACK = 1：负载 16 字节；
 *   - SUDOKU_TILE = 2：负载 10 字节；
 *   - 仅停车时允许 HELLO 绑定非零会话；
 *   - 本接口中的消息不能启动电机或写入 PWM。
 *
 * 固定开销 16 字节 = 帧头 2 + 元数据 12 + CRC 2，最大完整帧 48 字节；
 * 字节间隔超过 50 ms 丢弃半帧；道路观测年龄超过 200 ms 判为过期。
 * ============================================================================
 */

#define CAR_VISION_MAX_PAYLOAD   32U
#define CAR_VISION_MAX_FRAME     (16U + CAR_VISION_MAX_PAYLOAD)
#define CAR_VISION_FRAME_GAP_MS  50U
#define CAR_VISION_MAX_AGE_MS    200U

/*
 * 道路观测有效位掩码，可按位组合；
 * 只有置位的几何量才允许供业务层使用。
 */
enum
{
    VISION_LATERAL_VALID = 1,   /* 横向偏差有效 */
    VISION_HEADING_VALID = 2,   /* 航向角有效 */
    VISION_CURVATURE_VALID = 4, /* 曲率有效 */
    VISION_DISTANCE_VALID = 8   /* 距离有效 */
};

/*
 * 道路元素类型，按顺序为 0~8。
 */
enum
{
    VISION_ELEMENT_NONE,   /* 0：无元素 */
    VISION_ELEMENT_RING,   /* 1：环岛 */
    VISION_ELEMENT_CROSS,  /* 2：十字 */
    VISION_ELEMENT_GAP,    /* 3：断路 */
    VISION_ELEMENT_GREEN,  /* 4：绿灯 */
    VISION_ELEMENT_YELLOW, /* 5：黄灯 */
    VISION_ELEMENT_RED,    /* 6：红灯 */
    VISION_ELEMENT_FINISH, /* 7：终点候选 */
    VISION_ELEMENT_TAG     /* 8：Tag */
};

/*
 * 道路观测快照。
 */
typedef struct
{
    uint32_t received_ms;        /* 本机解析该帧的时间，单位毫秒，不是曝光时间 */
    uint32_t session;            /* 会话编号，来源：HELLO 帧 */
    uint16_t sequence;           /* 业务帧序号，来源：帧头，16 位回绕 */
    uint16_t processing_age_ms;  /* 发送端报告的处理年龄，单位毫秒 */
    uint16_t valid_mask;         /* 有效位掩码，取值见 VISION_*_VALID */

    /* 以下几何量仅在 valid_mask 对应位为 1 时有效。 */
    float lateral_m;             /* 横向误差，单位米，左正 */
    float heading_rad;           /* 航向误差，单位弧度，逆时针正 */
    float curvature_inv_m;       /* 曲率，单位 1/米 */
    float distance_m;            /* 元素距离，单位米 */
    float confidence;            /* 置信度，范围 0~1，由线上千分数换算 */

    uint8_t element;             /* 元素枚举，取值见 VISION_ELEMENT_* */
    uint8_t tag_id;              /* Tag 编号 0~7，255 表示未识别 */
} CarVisionObservation;

/*
 * 数独子块快照。
 */
typedef struct
{
    uint32_t received_ms; /* 本机解析该帧的时间，单位毫秒 */
    uint32_t session;     /* 会话编号，来源：HELLO 帧 */
    uint16_t sequence;    /* 业务帧序号，来源：帧头 */
    uint8_t cells[9];     /* 3×3 格值，按行排列：0 空白，1~6 为数字 */
    uint8_t valid;        /* 1 表示本会话缓存可用 */
} CarVisionTile;

/*
 * 视觉解析器状态。
 */
typedef struct
{
    /* buffer/used：尚未消费的串口字节及其数量。 */
    uint8_t buffer[CAR_VISION_MAX_FRAME];
    uint8_t used;

    /* 会话与序号状态。 */
    uint8_t session_valid;     /* 1 表示已绑定会话 */
    uint8_t have_sequence;     /* 1 表示已有可比较的业务序号 */
    uint8_t have_observation;  /* 1 表示已有道路观测，仍需另外检查时效 */

    /* 道路与数独消息共享的最近业务序号；HELLO 不推进它。 */
    uint16_t last_sequence;

    /* 统计与时间。 */
    uint32_t last_byte_ms;     /* 最近一次收到字节的时间，单位毫秒 */
    uint32_t session;          /* 当前绑定的会话编号 */
    uint32_t accepted_frames;  /* 已接受帧数，含 HELLO */
    uint32_t bad_frames;       /* 长度越界或 CRC 错误的帧数 */
    uint32_t rejected_frames;  /* CRC 正确但业务字段不合法的帧数 */

    CarVisionObservation observation; /* 最近一份有效道路观测 */

    /* 四个数独块，按左上、右上、左下、右下缓存；
     * 同一索引的新数据覆盖旧值。 */
    CarVisionTile tiles[4];
} CarVision;

/**
 * @brief 清空解析器、会话、统计和全部缓存。
 * @param v 解析器对象，须非空。
 * @note 后续须先接收停车状态下的合法 HELLO，才会接受业务观测。
 */
void CarVision_Init(CarVision *v);

/**
 * @brief 接收异常后作废半帧、道路观测和四个数独块。
 * @param v 解析器对象。
 * @note 保留会话、最近序号和统计；缓存内容可能仍在内存中，
 *       但有效标志已清除。
 */
void CarVision_Invalidate(CarVision *v);

/**
 * @brief 更新一个字节的 CRC16-CCITT-FALSE，返回新的累计值。
 * @param crc  首字节传 0xFFFF，之后传上次结果。
 * @param byte 待计算字节。
 * @return 更新后的 CRC-16 值。
 * @note 多项式 0x1021，不反射，最终不异或；线上的 CRC 高字节在前。
 */
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte);

/**
 * @brief 输入一个视觉字节，完成同步、校验和缓存更新。
 * @param v           非空解析器。
 * @param byte        接收字节。
 * @param now_ms      本机当前毫秒时间。
 * @param allow_hello 非零允许 HELLO；硬件层仅在 CAR_STOP 时传入非零值。
 * @note 无返回值，可读统计观察结果；不直接执行电机动作或发送 ACK。
 *       状态修改须在同一执行上下文完成；跨中断读取应通过硬件层快照接口。
 */
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms,
                    uint8_t allow_hello);

/**
 * @brief 复制通过时效检查的最新道路观测。
 * @param v      非空解析器。
 * @param now_ms 当前毫秒时间。
 * @param out    接收副本的对象。
 * @return 1 成功；0 表示 out 为 NULL、无观测或过期，out 保持原值。
 * @note 年龄按发送端处理时间加本地解析后时间计算，
 *       不含无法测量的排队延迟。
 *       返回有效快照不代表识别到道路：新鲜数据也可能 valid_mask = 0。
 */
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms,
                                 CarVisionObservation *out);

/**
 * @brief 复制索引 0~3 的数独块，不使用道路观测的 200 ms 时效规则。
 * @param v      非空解析器。
 * @param index  子块编号 0~3。
 * @param now_ms 当前毫秒时间。
 * @param out    接收副本的对象。
 * @return 1 成功；0 表示参数无效、缓存失效或时间差达到半个 32 位计时范围。
 * @note 失败不修改 out；更新会话或接收异常会使旧块失效。
 *       数独块在本会话内保留，由调用者判断四块拼图是否收齐。
 */
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms,
                          CarVisionTile *out);

#endif
