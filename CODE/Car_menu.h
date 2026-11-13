#ifndef CAR_MENU_H
#define CAR_MENU_H

#include <stdint.h>

#include "key.h"

/*
 * ============================================================================
 * 文件名称：Car_menu.h
 * ============================================================================
 * 模块功能：NEUQ 智能车校赛链表式目录（Linked-list Index）。
 *
 *   结构参考提交的菜单框架：
 *     - 用 MenuID 枚举统一编号，MENU_NONE = -1，MAIN = 0，MENU_COUNT 收尾；
 *     - MenuConfig 只描述“每个页面/节点的静态配置”；
 *     - MenuNode 在运行时由 Car_Menu_BuildLink() 按 parent_id 建链；
 *     - 同级节点使用环形双向链表，UP/DOWN 到头自动回绕；
 *     - page 表示当前页面，selected 永远是 page 的子节点；
 *     - editing = 0 浏览，editing = 1 编辑。
 *
 *   一级目录固定四类，不增加第五类：
 *       数独、香橙派、赛题元素PID、电池电压显示。
 *   MAIN 只是导航根页面，不属于一级分类。
 *
 *   显示说明
 *   ------------------------------------------------------------------------
 *   LCD200 默认 LCD200_DIR_Cross 是 320x240 横屏；原 17 行竖屏布局超出
 *   240 像素高度，属于显示 bug，已暂时移除渲染代码。菜单的建链、浏览和
 *   编辑逻辑不依赖 LCD，可先单独调试；显示层按横屏重新设计后再恢复。
 *
 *   调用约定
 *   ------------------------------------------------------------------------
 *   Car_Menu_Init() 初始化参数、建链、自检并初始化按键；
 *   Car_Menu_Poll() 在主循环周期调用，内部调用 Key_Scan(HAL_GetTick())；
 *   Car_Menu_Update() 可供主机测试直接注入按键事件。
 * ============================================================================
 */

/* 成绩无效标记，用于 160 cm 断路等失败即终止的节点。 */
#define CAR_MENU_SCORE_INVALID  (-32768)

/* 节点类型。 */
typedef enum
{
    CAR_MENU_TYPE_BASIC = 0,    /* 基本任务 */
    CAR_MENU_TYPE_SCORE,        /* 得分元素 */
    CAR_MENU_TYPE_PENALTY,      /* 罚时元素 */
    CAR_MENU_TYPE_RULE,         /* 规则约束 */
    CAR_MENU_TYPE_HARDWARE,     /* 硬件约束 */
    CAR_MENU_TYPE_TBD,          /* 现场TBD */
    CAR_MENU_TYPE_FLOW          /* 子流程/PID页 */
} CarMenuNodeType;

/* 是否可主动屏蔽。路障节点在第一圈必须为 CAR_MENU_SKIP_NO。 */
typedef enum
{
    CAR_MENU_SKIP_NO = 0,       /* 不可屏蔽 */
    CAR_MENU_SKIP_YES,          /* 可屏蔽 */
    CAR_MENU_SKIP_PARTIAL       /* 部分可屏蔽/按现场策略 */
} CarMenuSkip;

/* 适用圈数。 */
typedef enum
{
    CAR_MENU_LAP_FIRST = 0,     /* 第一圈 */
    CAR_MENU_LAP_SECOND,        /* 第二圈 */
    CAR_MENU_LAP_BOTH           /* 全程 */
} CarMenuLap;

/* 节点状态。 */
typedef enum
{
    CAR_MENU_STATUS_CONFIRMED = 0, /* 已确认 */
    CAR_MENU_STATUS_SITE,          /* 待现场确认 */
    CAR_MENU_STATUS_TBD            /* 未确定/不得臆造 */
} CarMenuStatus;

/* 卡尔曼相关程度。 */
typedef enum
{
    CAR_MENU_KALMAN_NO = 0,     /* 不使用 */
    CAR_MENU_KALMAN_YES,        /* 使用 */
    CAR_MENU_KALMAN_INDIRECT    /* 间接使用/可选 */
} CarMenuKalman;

/*
 * 菜单页面编号。
 * 与提交文件一致：MENU_NONE = -1，MAIN = 0，最后一个枚举 MENU_COUNT。
 * 枚举顺序同时决定同级节点的初始排列顺序（建链时按 i 递增追加）。
 */
typedef enum
{
    MENU_NONE = -1,             /* 空链接/无父节点 */
    MAIN = 0,                   /* 导航根页面 */

    CAT_SUDOKU,                 /* 一级目录：数独 */
    CAT_ORANGEPI,               /* 一级目录：香橙派 */
    CAT_PID,                    /* 一级目录：赛题元素PID */
    CAT_BATTERY,                /* 一级目录：电池电压显示 */

    S01, S02, S03, S04, S05, S06, S07, S08, S09,

    O01, O02, O03, O04, O05, O06, O07, O08,

    N01, N02, N03, N04, N05, N06, N07, N08, N09, N10,
    N11, N12, N13, N14, N15, N16, N17,

    P01, P02, P03, P04, P05, P06, P07, P08, P09, P10, P11, P12, P13,

    B01, B02, B03, B04, B05, B06,

    MENU_COUNT                  /* 节点总数，不含 MENU_NONE */
} MenuID;

/*
 * 可在菜单里编辑的参数。
 * 参考提交文件的做法：MenuConfig.value 指向这里的 int32_t 字段，
 * 浏览时按 OK 进入编辑，UP/DOWN 按 step 调整，OK 提交，BACK 或双击 OK 取消。
 */
typedef struct
{
    int32_t pid_kp;         /* 巡线 P，放大 100 倍 */
    int32_t pid_ki;         /* 速度 I，放大 100 倍 */
    int32_t pid_kd;         /* 微分，放大 100 倍 */
    int32_t kalman_q;       /* 过程噪声 Q，放大 1000 倍 */
    int32_t kalman_r;       /* 观测噪声 R，放大 1000 倍 */
    int32_t lap_mode;       /* 0 第一圈，1 第二圈 */
    int32_t speed_limit;    /* 第二圈限速，rpm */
    int32_t ring_speed;     /* 环岛速度，rpm */
    int32_t cone_offset;    /* 锥桶绕行偏移，mm */
    int32_t red_stop_s;     /* 红障第二圈停车时间，s */
    int32_t gap_speed;      /* 160cm 断路通过速度，rpm */
    int32_t battery_low;    /* 低电压阈值，mV */
} CarMenuParams;

extern CarMenuParams car_menu_params;

/* 页面/节点的静态配置。 */
typedef struct
{
    const char *name;       /* ASCII 显示名 */
    MenuID parent_id;       /* 父节点编号；MENU_NONE 表示根 */
    int32_t *value;         /* 可编辑参数地址；NULL 表示纯页面 */
    int32_t min;            /* 编辑下限 */
    int32_t max;            /* 编辑上限 */
    int32_t step;           /* 每次调整步长 */
    CarMenuNodeType type;
    CarMenuSkip skippable;
    CarMenuLap lap;
    int16_t score_s;        /* 正罚时、负奖励、CAR_MENU_SCORE_INVALID */
    CarMenuKalman kalman;
    CarMenuStatus status;
    MenuID pid_link;        /* 跨目录链接，仅作元数据 */
} MenuConfig;

/**
 * @brief 初始化参数、按 parent_id 建链、自检并初始化按键。
 * @note 不初始化 LCD；当前显示层已暂时移除。
 */
void Car_Menu_Init(void);

/**
 * @brief 主循环周期任务：扫描按键并处理浏览/编辑事件。
 * @note 建议每 5~10 ms 调用一次；不要和 Key_Scan() 的其他调用者并发。
 */
void Car_Menu_Poll(void);

/**
 * @brief 显示入口。当前为空实现，显示层待按 320x240 横屏重新设计。
 */
void Car_Menu_Render(void);

/**
 * @brief 注入一个按键事件，推进浏览/编辑状态机。
 * @param key   按键编号。
 * @param event 按键事件，只使用单击/双击/长按/重复。
 * @return 1 状态改变；0 忽略。
 * @note 供 Car_Menu_Poll() 和主机测试调用。
 */
uint8_t Car_Menu_Update(Key_Id key, Key_Event event);

/**
 * @brief 按 MenuTable[i].parent_id 构建运行时链表。
 * @return 1 成功；0 存在 parent_id 非法或自引用。
 * @note 同级节点使用环形双向链表；child 指向第一个子节点。
 */
uint8_t Car_Menu_BuildLink(void);

/**
 * @brief 检查建链是否闭合。
 * @return 1 通过；0 存在错误。
 * @note 检查父指针、环形同级链、根节点唯一性、parent 链可达 MAIN、
 *       可编辑参数的 min/max/step 以及 pid_link 目标。
 */
uint8_t Car_Menu_ValidateLinks(void);

/**
 * @brief 最近一次建链自检的错误数。
 */
uint16_t Car_Menu_LinkErrorCount(void);

/**
 * @brief 当前页面编号（page）。
 */
MenuID Car_Menu_GetPageId(void);

/**
 * @brief 当前选中项编号（selected）。
 */
MenuID Car_Menu_GetSelectedId(void);

/**
 * @brief 是否处于编辑模式。
 */
uint8_t Car_Menu_IsEditing(void);

/**
 * @brief 当前编辑值；非编辑模式下为 0。
 */
int32_t Car_Menu_GetEditValue(void);

/**
 * @brief 直接设置编辑值，用于测试或外部恢复；会按 min/max 限幅。
 */
void Car_Menu_SetEditValue(int32_t value);

/**
 * @brief 跳转到指定节点：叶子节点进入其父页面并选中自己，页面节点进入自身。
 * @return 1 成功；0 编号无效。
 */
uint8_t Car_Menu_Goto(MenuID id);

/**
 * @brief 取节点静态配置；编号无效返回 NULL。
 */
const MenuConfig *Car_Menu_GetConfig(MenuID id);

/**
 * @brief 节点总数（MENU_COUNT）。
 */
int16_t Car_Menu_NodeCount(void);

#endif
