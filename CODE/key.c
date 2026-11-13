/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */

/*
 * ============================================================================
 * 文件名称：key.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 三键非阻塞扫描：去抖、单击/双击判定、长按与自动重复。
 * 只依赖 NEUQ_GPIO 和 HAL_GetTick()，不访问定时器中断，也不阻塞主循环。
 *
 * 状态机
 * ----------------------------------------------------------------------------
 *   采样值 raw 与去抖值 stable 不同并持续 KEY_DEBOUNCE_MS 后，才接受跳变：
 *     stable 0 -> 1：产生 PRESS，记录按下时刻；
 *     stable 1 -> 0：产生 RELEASE，并根据长按/双击状态产生点击事件。
 *
 * 双击判定
 * ----------------------------------------------------------------------------
 *   第一次松开后进入“等待第二次点击”状态，click_count = 1；
 *   若窗口内再次按下，press 时把 click_count 提升为 2；
 *   第二次松开仍在窗口内则产生 DOUBLE_CLICK。
 *   若第二次按下后变成 LONG_PRESS，本次手势只上报长按，第一次单击作废；
 *   若第二次按下拖到窗口之外才松开，也不产生点击事件。
 *   窗口到期且按键已松开，才补发第一次的 SINGLE_CLICK。
 *
 * 事件队列
 * ----------------------------------------------------------------------------
 *   每个按键一个深度为 KEY_EVENT_QUEUE_SIZE 的环形队列；
 *   队列满时丢弃最旧事件并累加 drop_count，保证最新手势不丢失。
 *   Key_Scan() 与 Key_PollEvent() 必须在同一执行上下文，不需要关中断。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static uint8_t key_read_raw(Key_Id id)
 *       —— 读取指定按键的原始电平。
 *   static void key_push_event(Key_State *state, Key_Event event)
 *       —— 把事件写入环形队列，队列满时丢弃最旧事件。
 *   static void key_on_press(Key_State *state, uint32_t now_ms)
 *       —— 稳定按下处理：上报 PRESS，记录长按与双击判定的起点。
 *   static void key_on_release(Key_State *state, uint32_t now_ms)
 *       —— 稳定松开处理：区分长按、双击和单击。
 *   static void key_scan_one(Key_Id id, uint32_t now_ms)
 *       —— 推进单个按键的去抖、长按和双击状态机。
 *   void Key_Init(void)
 *       —— 初始化三个按键引脚和内部状态。
 *   void Key_Scan(uint32_t now_ms)
 *       —— 周期采样全部按键并产生事件。
 *   uint8_t Key_PollEvent(Key_Id id, Key_Event *event)
 *       —— 从指定按键的事件队列取出一个事件。
 *   Key_Event Key_GetEvent(Key_Id id)
 *       —— 取事件；空队列返回 KEY_EVENT_NONE。
 *   uint8_t Key_IsDown(Key_Id id)
 *       —— 查询去抖后的按下状态。
 *   uint32_t Key_GetDropCount(Key_Id id)
 *       —— 查询因队列满而丢弃的事件数。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"
#include "key.h"

/* ========================================================================== */
/*                              按键对象与引脚表                               */
/* ========================================================================== */

/* 按键对象：保存去抖、点击和事件队列状态。 */
typedef struct
{
    GPIO_Ports_ENUM pin;        /* 引脚编号，来源：key.h */

    uint8_t raw;                /* 最近一次采样值，1 表示按下 */
    uint8_t stable;             /* 去抖后的稳定状态，1 表示按下 */
    uint8_t last_raw;           /* 上一次采样值，用于检测输入变化 */
    uint32_t change_ms;         /* 最近一次 raw 变化的时刻 */

    uint32_t press_ms;          /* 稳定按下的时刻 */
    uint32_t last_click_ms;     /* 最近一次单击松开的时刻 */
    uint8_t click_count;        /* 0 无待定点击，1 等待双击，2 双击候选 */
    uint8_t long_reported;      /* 1 表示本次按住已经上报过长按 */
    uint32_t repeat_ms;         /* 上一次长按/重复事件的时刻 */

    uint8_t queue[KEY_EVENT_QUEUE_SIZE]; /* 事件环形队列 */
    uint8_t queue_head;         /* 写入位置 */
    uint8_t queue_tail;         /* 读取位置 */
    uint32_t drop_count;        /* 队列满丢弃的事件数 */
} Key_State;

/* 三个按键对象的静态实例。 */
static Key_State keys[KEY_COUNT];

/* 引脚映射，下标与 Key_Id 一致。 */
static const GPIO_Ports_ENUM key_pins[KEY_COUNT] =
{
    KEY_UP_PIN,
    KEY_DOWN_PIN,
    KEY_OK_PIN
};

/* ========================================================================== */
/*                              内部状态机函数                                 */
/* ========================================================================== */

/*
 * 函数名称：key_read_raw
 * 函数功能：读取指定按键的原始电平。
 * 参数说明：id —— 按键编号，调用者必须保证在 0 ~ KEY_COUNT-1 范围内。
 * 返回参数：1 按下；0 松开。
 * 使用示例：
 *     uint8_t down = key_read_raw(KEY_OK);
 * 备注信息：不做去抖；原始值由 key_scan_one() 送入去抖状态机。
 */
static uint8_t key_read_raw(Key_Id id)
{
    return (NEUQ_GPIO_read(keys[id].pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;
}

/*
 * 函数名称：key_push_event
 * 函数功能：把一个事件写入指定按键的环形队列。
 * 参数说明：state —— 按键对象；
 *           event —— 待写入的事件。
 * 返回参数：无。
 * 使用示例：
 *     key_push_event(state, KEY_EVENT_PRESS);
 * 备注信息：队列满时推进读指针，丢弃最旧事件并累加 drop_count；
 *           本函数只在 Key_Scan() 的同一上下文调用，不需要临界区。
 */
static void key_push_event(Key_State *state, Key_Event event)
{
    uint8_t next = (uint8_t)((state->queue_head + 1U) % KEY_EVENT_QUEUE_SIZE);

    if (next == state->queue_tail)
    {
        /* 队列已满：推进读指针，丢掉最旧的事件。 */
        state->queue_tail = (uint8_t)((state->queue_tail + 1U) % KEY_EVENT_QUEUE_SIZE);
        state->drop_count++;
    }

    state->queue[state->queue_head] = (uint8_t)event;
    state->queue_head = next;
}

/*
 * 函数名称：key_on_press
 * 函数功能：处理去抖后的稳定按下：上报 PRESS，重置长按与重复计时。
 * 参数说明：state  —— 按键对象；
 *           now_ms —— 本次扫描的毫秒时基。
 * 返回参数：无。
 * 使用示例：
 *     key_on_press(state, HAL_GetTick());
 * 备注信息：双击开启时，若上一次单击仍在等待窗口，把 click_count 置 2，
 *           表示“双击候选”，最终结果在松开时确认。
 */
static void key_on_press(Key_State *state, uint32_t now_ms)
{
    uint8_t click_count = state->click_count;

    key_push_event(state, KEY_EVENT_PRESS);
    state->press_ms = now_ms;
    state->repeat_ms = now_ms;
    state->long_reported = 0U;

#if KEY_ENABLE_DOUBLE_CLICK
    /* 双击窗口内的第二次按下：先记为候选双击，等松开时确认。 */
    if (click_count == 1U)
    {
        state->click_count = 2U;
    }
#else
    (void)click_count;
    state->click_count = 0U;
#endif
}

/*
 * 函数名称：key_on_release
 * 函数功能：处理去抖后的稳定松开，并根据手势产生长按/双击/单击事件。
 * 参数说明：state  —— 按键对象；
 *           now_ms —— 本次扫描的毫秒时基。
 * 返回参数：无。
 * 使用示例：
 *     key_on_release(state, HAL_GetTick());
 * 备注信息：长按松手不产生点击；双击开启时，第一次单击要等窗口到期
 *           才由 key_scan_one() 补发 SINGLE_CLICK。
 */
static void key_on_release(Key_State *state, uint32_t now_ms)
{
    key_push_event(state, KEY_EVENT_RELEASE);

    if (state->long_reported)
    {
        /* 长按手势不再产生点击事件。 */
        state->click_count = 0U;
        return;
    }

#if KEY_ENABLE_DOUBLE_CLICK
    if (state->click_count >= 2U)
    {
        /* 第二次点击在窗口内松开：确认双击。 */
        if ((uint32_t)(now_ms - state->last_click_ms) <= KEY_DOUBLE_CLICK_MS)
        {
            key_push_event(state, KEY_EVENT_DOUBLE_CLICK);
        }
        state->click_count = 0U;
    }
    else
    {
        /* 第一次点击：等待双击窗口，暂不产生单击事件。 */
        state->click_count = 1U;
        state->last_click_ms = now_ms;
    }
#else
    key_push_event(state, KEY_EVENT_SINGLE_CLICK);
    state->click_count = 0U;
#endif
}

/*
 * 函数名称：key_scan_one
 * 函数功能：采样一个按键并推进去抖、长按、重复和双击超时状态。
 * 参数说明：id     —— 按键编号；
 *           now_ms —— 本次扫描的毫秒时基，允许 32 位回绕。
 * 返回参数：无。
 * 使用示例：
 *     key_scan_one(KEY_UP, HAL_GetTick());
 * 备注信息：调用周期建议 5~10 ms；周期过长会漏掉短按。
 */
static void key_scan_one(Key_Id id, uint32_t now_ms)
{
    Key_State *state = &keys[id];
    uint8_t raw = key_read_raw(id);

    state->raw = raw;

    /* 输入变化就重新开始去抖计时；抖动期间不会接受新的稳定值。 */
    if (raw != state->last_raw)
    {
        state->last_raw = raw;
        state->change_ms = now_ms;
    }

    if ((uint32_t)(now_ms - state->change_ms) >= KEY_DEBOUNCE_MS &&
        state->stable != raw)
    {
        state->stable = raw;
        if (raw)
        {
            key_on_press(state, now_ms);
        }
        else
        {
            key_on_release(state, now_ms);
        }
    }

    if (state->stable)
    {
        if (!state->long_reported &&
            (uint32_t)(now_ms - state->press_ms) >= KEY_LONG_PRESS_MS)
        {
            key_push_event(state, KEY_EVENT_LONG_PRESS);
            state->long_reported = 1U;
            state->repeat_ms = now_ms;
        }
        else if (state->long_reported &&
                 (uint32_t)(now_ms - state->repeat_ms) >= KEY_REPEAT_MS)
        {
            key_push_event(state, KEY_EVENT_REPEAT);
            state->repeat_ms = now_ms;
        }
    }

#if KEY_ENABLE_DOUBLE_CLICK
    /* 双击窗口到期且按键已松开：补发单击。 */
    if (state->click_count == 1U && !state->stable &&
        (uint32_t)(now_ms - state->last_click_ms) > KEY_DOUBLE_CLICK_MS)
    {
        key_push_event(state, KEY_EVENT_SINGLE_CLICK);
        state->click_count = 0U;
    }
#endif
}

/* ========================================================================== */
/*                              对外接口                                       */
/* ========================================================================== */

/*
 * 函数名称：Key_Init
 * 函数功能：初始化三个按键引脚为内部上拉输入，并清空事件与计时状态。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     Key_Init();
 * 备注信息：在主循环启动前调用一次；重复调用会清空所有待处理事件。
 */
void Key_Init(void)
{
    uint32_t now = HAL_GetTick();
    unsigned i;

    memset(keys, 0, sizeof(keys));

    for (i = 0; i < KEY_COUNT; i++)
    {
        keys[i].pin = key_pins[i];

        /* 内部上拉；按键另一端接 GND，按下读到低电平。 */
        NEUQ_GPIO_init(keys[i].pin, GPIO_Mode_Input_PullUp, GPIO_High);

        keys[i].raw = key_read_raw((Key_Id)i);
        keys[i].last_raw = keys[i].raw;
        keys[i].stable = keys[i].raw;
        keys[i].change_ms = now;
        keys[i].press_ms = now;
        keys[i].repeat_ms = now;
    }
}

/*
 * 函数名称：Key_Scan
 * 函数功能：采样全部按键并推进各自的状态机，产生的事件进入队列。
 * 参数说明：now_ms —— 当前毫秒时基，通常传 HAL_GetTick()；允许 32 位回绕。
 * 返回参数：无。
 * 使用示例：
 *     while (1)
 *     {
 *         Key_Scan(HAL_GetTick());
 *         // 处理其他任务
 *     }
 * 备注信息：必须周期调用（建议 5~10 ms），且与 Key_PollEvent() 同一上下文。
 */
void Key_Scan(uint32_t now_ms)
{
    unsigned i;

    for (i = 0; i < KEY_COUNT; i++)
    {
        key_scan_one((Key_Id)i, now_ms);
    }
}

/*
 * 函数名称：Key_PollEvent
 * 函数功能：从指定按键的事件队列取出一个事件。
 * 参数说明：id    —— 按键编号；
 *           event —— 非空输出，成功时写入事件。
 * 返回参数：1 取到事件；0 队列为空或参数无效。
 * 使用示例：
 *     Key_Event event;
 *     while (Key_PollEvent(KEY_OK, &event))
 *     {
 *         // 处理 event
 *     }
 * 备注信息：取出后事件从队列移除；队列为空不会改写 event。
 */
uint8_t Key_PollEvent(Key_Id id, Key_Event *event)
{
    Key_State *state;

    if ((unsigned)id >= KEY_COUNT || event == 0)
    {
        return 0U;
    }

    state = &keys[id];

    if (state->queue_tail == state->queue_head)
    {
        return 0U;
    }

    *event = (Key_Event)state->queue[state->queue_tail];
    state->queue_tail = (uint8_t)((state->queue_tail + 1U) % KEY_EVENT_QUEUE_SIZE);
    return 1U;
}

/*
 * 函数名称：Key_GetEvent
 * 函数功能：取出一个事件；队列为空时返回 KEY_EVENT_NONE。
 * 参数说明：id —— 按键编号。
 * 返回参数：取到的事件；空队列或参数无效返回 KEY_EVENT_NONE。
 * 使用示例：
 *     Key_Event event = Key_GetEvent(KEY_DOWN);
 * 备注信息：只取一个事件；需要一次处理完队列时使用 Key_PollEvent()。
 */
Key_Event Key_GetEvent(Key_Id id)
{
    Key_Event event = KEY_EVENT_NONE;

    (void)Key_PollEvent(id, &event);
    return event;
}

/*
 * 函数名称：Key_IsDown
 * 函数功能：查询去抖后的按下状态。
 * 参数说明：id —— 按键编号。
 * 返回参数：1 按下；0 松开或参数无效。
 * 使用示例：
 *     if (Key_IsDown(KEY_OK))
 *     {
 *         // 按下处理
 *     }
 * 备注信息：读取的是稳定状态，不产生事件。
 */
uint8_t Key_IsDown(Key_Id id)
{
    if ((unsigned)id >= KEY_COUNT)
    {
        return 0U;
    }

    return keys[id].stable;
}

/*
 * 函数名称：Key_GetDropCount
 * 函数功能：查询指定按键因事件队列满而丢弃的事件数。
 * 参数说明：id —— 按键编号。
 * 返回参数：累计丢弃数；参数无效返回 0。
 * 使用示例：
 *     uint32_t lost = Key_GetDropCount(KEY_UP);
 * 备注信息：该值持续增大说明 Key_Scan() 调用过慢或主循环处理事件不及时。
 */
uint32_t Key_GetDropCount(Key_Id id)
{
    if ((unsigned)id >= KEY_COUNT)
    {
        return 0U;
    }

    return keys[id].drop_count;
}
