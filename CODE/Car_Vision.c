/*********************************************************************************************************************
* 文件名称          Car_Vision.c
* 功能说明          香橙派视觉观测协议的接收与解析，属于"视觉通信协议"模块，不直接操作电机。
*
*                   本文件包含的函数：
*                   · CarVision_Crc16          —— CRC-16/CCITT-FALSE 逐字节校验
*                   · CarVision_Init           —— 清空视觉接收上下文
*                   · CarVision_Invalidate     —— 作废观测与子块（保留会话绑定和统计计数）
*                   · discard                  —— 内部工具：从接收缓冲头部丢弃 n 字节
*                   · accept                   —— 内部工具：处理一条 CRC 正确的完整视觉帧
*                   · CarVision_Feed           —— 逐字节流式拆帧（A5 5A 帧头 + 长度 + CRC16）
*                   · CarVision_GetObservation —— 读取带新鲜度检查的观测快照
*                   · CarVision_GetTile        —— 读取指定编号的数独子块快照
*
*                   数据流：USART3 中断 -> Car_Link 的字节 FIFO -> 5ms 控制拍批量调用 CarVision_Feed
*                   -> 解析结果缓存 -> Car_GetVisionObservation / Car_GetVisionTile 取快照。
*
* 修改记录
* 日期              作者                备注
* 2026-09-27                            first version
* 2026-11-12                            英文注释改为中文，公共工具改用 Car_Utils，修正失效说明
********************************************************************************************************************/
#include "Car_Vision.h"
#include "Car_Utils.h"
#include <string.h>

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
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte)
{
    unsigned i;                                             /* 8 个数据位的循环下标 */

    crc ^= (uint16_t)byte << 8;                             /* 本字节移到高 8 位后异或进 CRC */
    for (i = 0U; i < 8U; i++) {
        if ((crc & 0x8000U) != 0U) {                        /* 最高位为 1：左移后异或多项式 */
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {                                            /* 最高位为 0：只左移 */
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Init
 * 功能说明    初始化视觉接收上下文：清零接收缓冲、会话状态、统计计数和已缓存的观测/子块。
 * 用法        上电初始化时调用一次，必须早于 CarVision_Feed。
 * 参数说明    v —— 视觉接收上下文指针
 * 返回说明    无
 * 示例        static CarVision vision;
 *             CarVision_Init(&vision);
 * 备注        初始化后没有有效会话，必须先在停车状态下收到 HELLO 才会接收观测数据。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarVision_Init(CarVision *v)
{
    memset(v, 0, sizeof(*v));                               /* 所有字段归零，等价于"无会话、无数据" */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Invalidate
 * 功能说明    作废当前视觉数据：清空未处理字节、观测有效标志和四份数独子块；
 *             保留会话绑定、序号状态和统计计数。
 * 用法        串口错误、缓冲区溢出、需要丢弃旧数据时调用；同一会话可继续接收新的有效帧。
 * 参数说明    v —— 视觉接收上下文指针
 * 返回说明    无
 * 示例        CarVision_Invalidate(&vision);   // 串口异常后丢弃旧观测，等待后续有效帧
 * 备注        1) 本函数不改变 accepted_frames / bad_frames / rejected_frames 等诊断计数；
 *             2) 会话绑定仍然有效，因此不会强制要求视觉端重新发送 HELLO。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarVision_Invalidate(CarVision *v)
{
    unsigned i;                                             /* 数独子块下标 0~3 */

    v->used             = 0U;                               /* 丢弃尚未拆完的字节 */
    v->have_observation = 0U;                               /* 旧观测立即失效 */
    for (i = 0U; i < 4U; i++) {
        v->tiles[i].valid = 0U;                             /* 四份数独子块全部作废 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    discard
 * 功能说明    从接收缓冲区头部丢弃 n 个字节，并把剩余字节前移。
 * 用法        Car_Vision.c 内部在帧头不匹配、CRC 错误、长度非法或整帧处理完成后调用。
 * 参数说明    v —— 视觉接收上下文指针
 *             n —— 要丢弃的字节数，不能大于 v->used
 * 返回说明    无
 * 示例        discard(&vision, 1);        // 丢一个非法字节继续找帧头
 *             discard(&vision, total);   // 整帧处理完，移出缓冲
 * 备注        使用 memmove 处理重叠区域；调用方需保证 n <= v->used。
 *-------------------------------------------------------------------------------------------------------------------*/
static void discard(CarVision *v, unsigned n)
{
    v->used = (uint8_t)(v->used - n);                       /* 先减少有效字节数 */
    if (v->used != 0U) {
        memmove(v->buffer, v->buffer + n, v->used);         /* 剩余字节整体前移到缓冲头部 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    accept
 * 功能说明    处理一条已通过 CRC 校验的完整视觉帧：校验版本/会话/序号/新鲜度，解析 TRACK 观测
 *             或 SUDOKU_TILE 子块，更新统计计数；HELLO 只在允许时绑定新会话。
 * 用法        由 CarVision_Feed 在整帧校验通过后调用，业务层无需直接调用。
 * 参数说明    v           —— 视觉接收上下文指针
 *             now         —— 当前毫秒时基（HAL_GetTick）
 *             allow_hello —— 非 0 表示当前允许绑定新会话（仅停车状态传入 1）
 * 返回说明    无（结果写入 v->observation / v->tiles / 统计字段）
 * 示例        // 内部调用：CRC 通过且长度合法后
 *             // accept(&vision, HAL_GetTick(), car.state == CAR_STOP);
 * 备注        1) 会话号必须非 0，且与已绑定会话一致；
 *             2) 序号必须递增（允许 16 位回绕），重复或倒退帧被拒绝；
 *             3) 处理时延 age 超过 CAR_VISION_MAX_AGE_MS 的帧被拒绝；
 *             4) 字段越界（置信度、航向、曲率、元素编号等）整帧拒绝，避免脏数据进入控制层。
 *-------------------------------------------------------------------------------------------------------------------*/
static void accept(CarVision *v, uint32_t now, uint8_t allow_hello)
{
    const uint8_t *f = v->buffer;                           /* 整帧起始地址（A5 5A ...） */
    const uint8_t *p = f + 14;                              /* 负载起点：固定 14 字节帧头之后 */
    uint8_t  type      = f[3];                              /* 帧类型：0 HELLO，1 TRACK，2 SUDOKU_TILE */
    uint16_t len       = Car_ReadU16BE(f + 4);              /* 负载长度（大端） */
    uint16_t seq       = Car_ReadU16BE(f + 6);              /* 会话内序号（大端） */
    uint16_t age       = Car_ReadU16BE(f + 12);             /* 视觉端处理时延（大端，ms） */
    uint32_t session   = Car_ReadU32BE(f + 8);              /* 视觉会话号（大端） */
    uint16_t delta;                                         /* 当前序号与上一序号的差值 */
    unsigned i;                                             /* 数独格子/子块下标 */

    /* 版本必须为 1，会话号必须非 0；否则整帧丢弃。 */
    if ((f[2] != 1U) || (session == 0U)) {
        v->rejected_frames++;
        return;
    }

    /* HELLO：长度 0、时延 0，用于建立或切换会话。 */
    if ((type == 0U) && (len == 0U) && (age == 0U)) {
        if (allow_hello == 0U) {
            v->rejected_frames++;                           /* 行驶中禁止重新握手 */
            return;
        }
        if ((v->session_valid == 0U) || (v->session != session)) {
            v->have_observation = 0U;                       /* 新会话：旧观测作废 */
            v->have_sequence    = 0U;                       /* 新会话：序号重新开始 */
            for (i = 0U; i < 4U; i++) {
                v->tiles[i].valid = 0U;                     /* 新会话：旧数独子块作废 */
            }
            v->session       = session;                     /* 绑定新会话号 */
            v->session_valid = 1U;
        }
        v->accepted_frames++;
        return;
    }

    /* 非 HELLO 帧：必须已绑定会话、会话号一致，且处理时延未过期。 */
    if ((v->session_valid == 0U) || (session != v->session) || (age > CAR_VISION_MAX_AGE_MS)) {
        v->rejected_frames++;
        return;
    }

    /* 序号必须递增：重复帧或倒退帧拒绝；允许 16 位自然回绕。 */
    delta = (uint16_t)(seq - v->last_sequence);
    if ((v->have_sequence != 0U) && ((delta == 0U) || (delta >= 0x8000U))) {
        v->rejected_frames++;
        return;
    }

    if ((type == 1U) && (len == 16U)) {
        /* TRACK 观测：16 字节负载，先做范围检查，再换算物理量。 */
        CarVisionObservation o;                             /* 临时观测，校验全部通过后才覆盖缓存 */
        int32_t  lateral   = Car_ReadI16BE(p + 4);          /* 横向偏差，单位 mm，左正 */
        int32_t  heading   = Car_ReadI16BE(p + 6);          /* 航向偏差，单位 mrad，逆时针正 */
        int32_t  curvature = Car_ReadI16BE(p + 8);          /* 曲率，单位 0.001/m */
        uint16_t flags      = Car_ReadU16BE(p);             /* 有效字段位 */
        uint16_t confidence = Car_ReadU16BE(p + 2);         /* 置信度 0~1000 */

        if ((flags > 15U) || (confidence > 1000U) ||
            (lateral < -1000) || (lateral > 1000) ||
            (heading < -3142) || (heading > 3142) ||
            (curvature < -20000) || (curvature > 20000) ||
            (p[10] > VISION_ELEMENT_TAG) ||
            ((p[11] > 7U) && (p[11] != 255U)) ||
            (Car_ReadU16BE(p + 14) != 0U)) {
            v->rejected_frames++;                           /* 任一字段越界：整帧拒绝 */
            return;
        }

        memset(&o, 0, sizeof(o));
        o.received_ms       = now;                          /* 本机接收时刻 */
        o.session           = session;                      /* 所属会话 */
        o.sequence          = seq;                          /* 帧序号 */
        o.processing_age_ms = age;                          /* 视觉端处理时延 */
        o.valid_mask        = flags;                        /* 哪些字段有效 */
        o.confidence        = (float)confidence / 1000.0f; /* 0~1000 换算为 0.0~1.0 */
        o.lateral_m         = (float)lateral / 1000.0f;     /* mm 换算为 m */
        o.heading_rad       = (float)heading / 1000.0f;     /* mrad 换算为 rad */
        o.curvature_inv_m   = (float)curvature / 1000.0f;   /* 0.001/m 换算为 1/m */
        o.distance_m        = (float)Car_ReadU16BE(p + 12) / 1000.0f; /* mm 换算为 m */
        o.element           = p[10];                        /* 元素类型 */
        o.tag_id            = p[11];                        /* Tag 编号 */
        v->observation      = o;                            /* 覆盖最近一次有效观测 */
        v->have_observation = 1U;
    } else if ((type == 2U) && (len == 10U)) {
        /* SUDOKU_TILE 子块：第 0 字节为子块编号，后 9 字节为格子值。 */
        CarVisionTile tile;                                 /* 临时子块，校验通过后才写入缓存 */

        if (p[0] > 3U) {
            v->rejected_frames++;                           /* 子块编号只能为 0~3 */
            return;
        }
        for (i = 1U; i < 10U; i++) {
            if (p[i] > 6U) {
                v->rejected_frames++;                       /* 每格只能是 0~6 */
                return;
            }
        }
        memset(&tile, 0, sizeof(tile));
        tile.received_ms = now;                             /* 本机接收时刻 */
        tile.session     = session;                         /* 所属会话 */
        tile.sequence    = seq;                             /* 帧序号 */
        tile.valid       = 1U;                              /* 标记该子块有效 */
        memcpy(tile.cells, p + 1, 9U);                      /* 拷贝 9 个格子的识别结果 */
        v->tiles[p[0]]   = tile;                            /* 按编号写入对应缓存 */
    } else {
        v->rejected_frames++;                               /* 类型或长度不匹配 */
        return;
    }

    v->last_sequence = seq;                                 /* 记录最新序号，供下一帧递增检查 */
    v->have_sequence = 1U;
    v->accepted_frames++;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_Feed
 * 功能说明    逐字节接收视觉串口数据，按 A5 5A 帧头、长度、CRC16 做流式拆帧，整帧合法后转 accept 处理；
 *             同时处理帧间间隔超时与接收缓冲上限。
 * 用法        在视觉串口接收路径中每收到 1 字节调用一次（本项目在 5ms 定时拍内批量喂入），
 *             必须在单一上下文调用，读侧通过 Car_GetVisionObservation 取快照。
 * 参数说明    v           —— 视觉接收上下文指针
 *             byte        —— 本次收到的字节
 *             now_ms      —— 当前毫秒时基（HAL_GetTick）
 *             allow_hello —— 非 0 允许绑定新会话（仅在整车 STOP 时传 1）
 * 返回说明    无
 * 示例        CarVision_Feed(&vision, byte, HAL_GetTick(), car.state == CAR_STOP);
 * 备注        1) 帧格式：A5 5A | 版本 | 类型 | 长度 | 序号 | 会话 | 处理时延 | 负载 | CRC16；
 *             2) 长度超过 CAR_VISION_MAX_PAYLOAD 视为坏帧；
 *             3) 字节间隔超过 CAR_VISION_FRAME_GAP_MS 时丢弃半帧；
 *             4) 本接口只接收，不能启动电机或写 PWM。
 *-------------------------------------------------------------------------------------------------------------------*/
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello)
{
    unsigned i;                                             /* CRC 计算的字节下标 */
    unsigned total;                                         /* 当前帧的总长度（头 + 负载 + CRC） */
    uint16_t len;                                           /* 当前帧的负载长度 */
    uint16_t crc;                                           /* CRC 计算中间值/最终值 */

    /* 帧间超时：上一帧没接收完且间隔超过 50ms，丢弃残帧。 */
    if ((v->used != 0U) && ((uint32_t)(now_ms - v->last_byte_ms) > CAR_VISION_FRAME_GAP_MS)) {
        v->used = 0U;
    }
    v->last_byte_ms = now_ms;                               /* 刷新最近字节到达时刻 */

    /* 缓冲已满仍未拆出完整帧：丢弃最老的一个字节，保证新字节有位置。 */
    if (v->used >= CAR_VISION_MAX_FRAME) {
        discard(v, 1U);
    }
    v->buffer[v->used] = byte;                              /* 新字节追加到缓冲尾部 */
    v->used++;

    while (v->used != 0U) {
        if (v->buffer[0] != 0xA5U) {                        /* 第一帧头不匹配：丢一个字节重新找 */
            discard(v, 1U);
            continue;
        }
        if (v->used < 2U) {
            return;                                         /* 还不够判断第二个帧头 */
        }
        if (v->buffer[1] != 0x5AU) {                        /* 第二帧头不匹配：丢一个字节重新找 */
            discard(v, 1U);
            continue;
        }
        if (v->used < 6U) {
            return;                                         /* 还不够读取长度字段 */
        }

        len = Car_ReadU16BE(v->buffer + 4);                 /* 帧头第 4~5 字节为负载长度 */
        if (len > CAR_VISION_MAX_PAYLOAD) {
            v->bad_frames++;                                /* 长度超限：坏帧 */
            discard(v, 1U);
            continue;
        }

        total = 16U + len;                                  /* 14 字节头 + 负载 + 2 字节 CRC */
        if (v->used < total) {
            return;                                         /* 整帧还没收完，等待后续字节 */
        }

        crc = 0xFFFFU;                                      /* CRC 初值 */
        for (i = 2U; i < (total - 2U); i++) {               /* 覆盖版本到负载末尾 */
            crc = CarVision_Crc16(crc, v->buffer[i]);
        }
        if (crc != Car_ReadU16BE(v->buffer + total - 2U)) {
            v->bad_frames++;                                /* CRC 不匹配：坏帧 */
            discard(v, 1U);
            continue;
        }

        accept(v, now_ms, allow_hello);                     /* 整帧合法：交给业务解析 */
        discard(v, total);                                  /* 从缓冲移除整帧 */
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数名称    CarVision_GetObservation
 * 功能说明    读取最近一帧道路观测的快照，并检查传输时延与视觉端处理时延之和是否仍在新鲜度窗口内。
 * 用法        控制/任务层需要道路偏差、航向、曲率、元素或距离时调用；返回 0 表示无可用观测。
 * 参数说明    v      —— 视觉接收上下文指针（只读）
 *             now_ms —— 当前毫秒时基
 *             out    —— 输出观测结构，成功时被完整写入
 * 返回说明    1 表示数据新鲜可用；0 表示从未收到、已失效或已过期
 * 示例        CarVisionObservation obs;
 *             if (CarVision_GetObservation(&vision, HAL_GetTick(), &obs)) { ... }
 * 备注        1) 本函数不修改接收上下文，可在快照临界区内调用；
 *             2) valid_mask 为 0 表示链路正常但当前没有检测到道路，不等于通信失败；
 *             3) 判断使用无符号时间差，可安全跨越 32 位毫秒计数器回绕。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out)
{
    uint32_t age;                                           /* 从本机接收到现在经过的时间（ms） */

    if ((out == NULL) || (v->have_observation == 0U)) {
        return 0U;                                          /* 没有缓存观测 */
    }

    age = (uint32_t)(now_ms - v->observation.received_ms);
    if ((age > CAR_VISION_MAX_AGE_MS) ||
        (age > (CAR_VISION_MAX_AGE_MS - v->observation.processing_age_ms))) {
        return 0U;                                          /* 接收时延 + 处理时延已超出新鲜度窗口 */
    }

    *out = v->observation;                                  /* 拷贝一致快照给调用方 */
    return 1U;
}

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
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out)
{
    if ((out == NULL) || (index > 3U) || (v->tiles[index].valid == 0U) ||
        ((uint32_t)(now_ms - v->tiles[index].received_ms) >= 0x80000000U)) {
        return 0U;
    }

    *out = v->tiles[index];                                 /* 拷贝一致快照给调用方 */
    return 1U;
}
