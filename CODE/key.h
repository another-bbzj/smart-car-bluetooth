#ifndef KEY_H
#define KEY_H

#include <stdint.h>

#include "NEUQ_GPIO.h"

/*
 * ============================================================================
 * 文件名称：key.h
 * ============================================================================
 * 模块功能：三键（上/下/确认）非阻塞扫描驱动，供 Car_menu 使用。
 *
 *   事件模型
 *   ------------------------------------------------------------------------
 *   按下沿立即产生 PRESS，松开沿产生 RELEASE；
 *   一次完整点击在双击窗口结束后产生 SINGLE_CLICK；
 *   窗口内完成第二次点击产生 DOUBLE_CLICK；
 *   按住超过长按门限产生 LONG_PRESS，之后按重复周期产生 REPEAT。
 *
 *   双击与单击的取舍
 *   ------------------------------------------------------------------------
 *   开启双击后，SINGLE_CLICK 必须等待 KEY_DOUBLE_CLICK_MS 才能确定，
 *   因此菜单响应会有一个双击窗口的延迟。若不需要双击，把
 *   KEY_ENABLE_DOUBLE_CLICK 设为 0，松开按键立即产生 SINGLE_CLICK。
 *
 *   调用约定
 *   ------------------------------------------------------------------------
 *   - Key_Init() 在主循环启动前调用一次；
 *   - Key_Scan(now_ms) 必须由同一上下文周期调用，建议 5~10 ms；
 *   - 事件队列只在 Key_Scan() 内写入，Key_PollEvent() 读取，不需要临界区；
 *   - 本模块不阻塞、不使用 HAL_Delay()，时间基准为 HAL_GetTick()。
 *
 *   引脚约定
 *   ------------------------------------------------------------------------
 *   默认 PB12/PB13/PB14，按键另一端接 GND，内部上拉，按下为低电平。
 *   这三个引脚在本工程中未被 PWM、编码器、串口、Flash 或 LCD 占用。
 *   若实际硬件不同，可在编译选项中覆盖下面的宏，不要直接修改库文件。
 * ============================================================================
 */

/* 按键引脚：上、下、确认。 */
#ifndef KEY_UP_PIN
#define KEY_UP_PIN              PB12
#endif
#ifndef KEY_DOWN_PIN
#define KEY_DOWN_PIN            PB13
#endif
#ifndef KEY_OK_PIN
#define KEY_OK_PIN              PB14
#endif

/* 按下时的有效电平：按键接地时为 GPIO_Low。 */
#ifndef KEY_ACTIVE_LEVEL
#define KEY_ACTIVE_LEVEL        GPIO_Low
#endif

/* 时间参数，单位毫秒。 */
#ifndef KEY_DEBOUNCE_MS
#define KEY_DEBOUNCE_MS         20U
#endif
#ifndef KEY_DOUBLE_CLICK_MS
#define KEY_DOUBLE_CLICK_MS     300U
#endif
#ifndef KEY_LONG_PRESS_MS
#define KEY_LONG_PRESS_MS       800U
#endif
#ifndef KEY_REPEAT_MS
#define KEY_REPEAT_MS           120U
#endif

/* 双击开关：0 关闭，1 开启。 */
#ifndef KEY_ENABLE_DOUBLE_CLICK
#define KEY_ENABLE_DOUBLE_CLICK 1
#endif

/* 每个按键的事件队列深度，必须大于 1。 */
#ifndef KEY_EVENT_QUEUE_SIZE
#define KEY_EVENT_QUEUE_SIZE    8U
#endif

/* 按键编号。 */
typedef enum
{
    KEY_UP = 0,     /* 上翻 */
    KEY_DOWN,       /* 下翻 */
    KEY_OK,         /* 确认 */
    KEY_COUNT       /* 按键数量，仅用于内部数组 */
} Key_Id;

/* 按键事件。 */
typedef enum
{
    KEY_EVENT_NONE = 0,     /* 无事件 */
    KEY_EVENT_PRESS,        /* 按下沿，立即产生 */
    KEY_EVENT_RELEASE,      /* 松开沿，立即产生 */
    KEY_EVENT_SINGLE_CLICK, /* 单击，双击窗口结束后产生 */
    KEY_EVENT_DOUBLE_CLICK, /* 双击，第二次松开时产生 */
    KEY_EVENT_LONG_PRESS,   /* 长按门限到达 */
    KEY_EVENT_REPEAT        /* 长按后的自动重复 */
} Key_Event;

/**
 * @brief 初始化三个按键引脚和内部的去抖/事件状态。
 * @note 在主循环启动前调用一次；重复调用会清空所有待处理事件。
 */
void Key_Init(void);

/**
 * @brief 采样全部按键并推进状态机，产生的事件进入每个按键的队列。
 * @param now_ms 当前毫秒时基，通常传 HAL_GetTick()；允许 32 位回绕。
 * @note 必须周期性调用（建议 5~10 ms）。调用过慢会漏掉短按。
 */
void Key_Scan(uint32_t now_ms);

/**
 * @brief 从指定按键的事件队列取出一个事件。
 * @param id    按键编号。
 * @param event 非空输出；成功时写入事件。
 * @return 1 取到事件；0 队列为空或参数无效。
 */
uint8_t Key_PollEvent(Key_Id id, Key_Event *event);

/**
 * @brief 取出一个事件，空队列返回 KEY_EVENT_NONE。
 * @param id 按键编号。
 */
Key_Event Key_GetEvent(Key_Id id);

/**
 * @brief 查询去抖后的按下状态。
 * @param id 按键编号。
 * @return 1 按下；0 松开或参数无效。
 */
uint8_t Key_IsDown(Key_Id id);

/**
 * @brief 查询该按键因队列满而丢弃的事件数，用于诊断扫描过慢。
 * @param id 按键编号。
 */
uint32_t Key_GetDropCount(Key_Id id);

#endif
