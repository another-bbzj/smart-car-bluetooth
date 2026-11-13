#ifndef CAR_MENU_H
#define CAR_MENU_H

#include <stdint.h>

/*
 * ============================================================================
 * 文件名称：Car_menu.h
 * ============================================================================
 * 模块功能：NEUQ 智能车校赛链表式目录（Linked-list Index）的页面骨架与
 *           菜单导航接口。
 *
 *   一级目录固定四类，不增加第五类：
 *       1. 数独
 *       2. 香橙派
 *       3. 赛题元素PID
 *       4. 电池电压显示
 *
 *   链表组织
 *   ------------------------------------------------------------------------
 *   节点存放在只读数组 car_menu_nodes[] 中，节点之间用 16 位索引连接，
 *   0（CAR_MENU_INDEX_NONE）表示空。每个节点同时具备：
 *       prev / next       同级默认前后继，构成双向链表；
 *       branch_to / merge_to  事件分支入口与汇合点；
 *       parent / child    页面层级；
 *       pid_link          跨一级目录的双向链接。
 *
 *   导航约定（按键见 key.h）
 *   ------------------------------------------------------------------------
 *       UP/DOWN 单击或长按重复   同级 prev / next，到头回绕；
 *       UP/DOWN 双击             切换一级目录；
 *       UP/DOWN 长按             回到本一级目录入口；
 *       OK 单击                  进入 child，否则跳到 pid_link；
 *       OK 双击                  返回上一个进入点（内部返回栈）；
 *       OK 长按                  切换第一圈/第二圈显示状态。
 *
 *   显示约定
 *   ------------------------------------------------------------------------
 *   LCD200 只有 ASCII 字库，因此页面同时保存中文 name（文档/调试用）和
 *   ASCII label（屏幕显示用）。中文名不直接上屏，避免出现乱码。
 *
 *   集成说明
 *   ------------------------------------------------------------------------
 *   这两个新文件加入 Keil 工程后，在主循环中周期调用 Car_Menu_Poll()，
 *   并在启动时调用一次 Car_Menu_Init()。Car_menu 不修改底盘控制状态；
 *   PID/卡尔曼参数写回、圈次切换等执行动作仍由上层控制模块完成。
 * ============================================================================
 */

/* 屏幕显示开关：0 时菜单仍可导航，但不初始化/不绘制 LCD。 */
#ifndef CAR_MENU_USE_LCD
#define CAR_MENU_USE_LCD        1
#endif

/* 链表空索引。数组下标 0 保留为哑节点，因此 0 可以安全表示“无”。 */
#define CAR_MENU_INDEX_NONE     0

/* 成绩无效标记，用于 160 cm 断路等失败即终止的节点。 */
#define CAR_MENU_SCORE_INVALID  (-32768)

/* 返回栈深度。 */
#ifndef CAR_MENU_STACK_DEPTH
#define CAR_MENU_STACK_DEPTH    8U
#endif

/* 一级分类，顺序固定，不得增加第五类。 */
typedef enum
{
    CAR_MENU_CAT_SUDOKU = 0,    /* 数独 */
    CAR_MENU_CAT_ORANGEPI,      /* 香橙派 */
    CAR_MENU_CAT_PID,           /* 赛题元素PID */
    CAR_MENU_CAT_BATTERY,       /* 电池电压显示 */
    CAR_MENU_CAT_COUNT
} CarMenuCategory;

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

/* 触发条件。 */
typedef enum
{
    CAR_MENU_TRIG_TOPOLOGY = 0, /* 拓扑到达 */
    CAR_MENU_TRIG_VISION,       /* 视觉检测 */
    CAR_MENU_TRIG_COLOR,        /* 颜色识别 */
    CAR_MENU_TRIG_TAG,          /* Tag 识别 */
    CAR_MENU_TRIG_SITE,         /* 现场公布 */
    CAR_MENU_TRIG_TIMER,        /* 计时触发 */
    CAR_MENU_TRIG_VOLTAGE,      /* 电压阈值 */
    CAR_MENU_TRIG_MANUAL        /* 人工/调试 */
} CarMenuTrigger;

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
 * 链表节点。
 * 所有链接字段都是 car_menu_nodes[] 的下标；CAR_MENU_INDEX_NONE 表示空。
 * 省略的字段由 C99 指定初始化器清零，因此默认就是“无链接”。
 */
typedef struct CarMenuNode
{
    const char *id;             /* 节点ID，如 "N01" */
    const char *name;           /* 中文名称，供文档和调试查看 */
    const char *label;          /* ASCII 短标签，供 LCD200 显示 */

    CarMenuCategory category;   /* 一级分类 */
    CarMenuNodeType type;       /* 节点类型 */
    CarMenuSkip skippable;      /* 是否可屏蔽 */
    CarMenuLap lap;             /* 适用圈数 */
    CarMenuTrigger trigger;     /* 触发条件 */
    CarMenuStatus status;       /* 已确认/现场/TBD */
    CarMenuKalman kalman;       /* 卡尔曼相关程度 */
    int16_t score_s;            /* 正数罚时、负数奖励；INVALID 表示成绩无效 */

    int16_t prev;               /* 同级前驱 */
    int16_t next;               /* 同级后继 */
    int16_t branch_to;          /* 事件分支入口（如路障颜色分支） */
    int16_t merge_to;           /* 分支汇合点 */
    int16_t parent;             /* 所属上级页面 */
    int16_t child;              /* 进入的子页面/子链表 */
    int16_t pid_link;           /* 跨目录双向链接目标 */

    uint8_t pid_page;           /* PID 页编号 1~13，0 表示非 PID 页 */
} CarMenuNode;

/* 菜单运行状态。 */
typedef struct CarMenu
{
    int16_t current;            /* 当前节点索引 */
    int16_t stack[CAR_MENU_STACK_DEPTH]; /* 返回栈 */
    uint8_t depth;              /* 返回栈深度 */
    uint8_t category;           /* 当前一级分类 */
    CarMenuLap lap;             /* 屏幕显示的第一圈/第二圈状态 */
    uint8_t dirty;              /* 1 表示需要重绘 */
} CarMenu;

/**
 * @brief 初始化按键和 LCD，定位到主链入口并绘制首屏。
 * @note 会调用 Key_Init() 和 NEUQ_LCD200_init()，应在主循环启动前调用一次。
 *       HAL_Delay() 会在此期间执行，不能放在中断里。
 */
void Car_Menu_Init(void);

/**
 * @brief 周期任务：扫描按键、处理事件、按需重绘。
 * @note 建议主循环每 5~10 ms 调用一次；不得在中断中调用。
 */
void Car_Menu_Poll(void);

/**
 * @brief 强制重绘当前页面。
 * @note 未启用 LCD（CAR_MENU_USE_LCD=0）时为空操作。
 */
void Car_Menu_Render(void);

/**
 * @brief 跳转到指定节点。
 * @param index 节点索引，必须大于 0 且小于节点总数。
 * @return 1 成功；0 索引无效。
 */
uint8_t Car_Menu_Goto(int16_t index);

/**
 * @brief 按节点ID跳转，例如 "P10"。
 * @param id 以 '\0' 结尾的节点ID；NULL 或未找到返回 0。
 * @return 1 成功；0 未找到。
 */
uint8_t Car_Menu_GotoId(const char *id);

/**
 * @brief 当前节点索引。
 */
int16_t Car_Menu_CurrentIndex(void);

/**
 * @brief 当前节点只读指针；索引异常时返回入口节点。
 */
const CarMenuNode *Car_Menu_CurrentNode(void);

/**
 * @brief 按索引取只读节点指针。
 * @return 索引有效返回节点；否则返回 NULL。
 */
const CarMenuNode *Car_Menu_GetNode(int16_t index);

/**
 * @brief 节点总数（不含保留的空索引）。
 */
int16_t Car_Menu_NodeCount(void);

/**
 * @brief 设置屏幕显示的圈次状态。
 * @note 只影响菜单显示，不改变底盘控制；圈次执行由上层状态机负责。
 */
void Car_Menu_SetLap(CarMenuLap lap);

/**
 * @brief 读取屏幕显示的圈次状态。
 */
CarMenuLap Car_Menu_GetLap(void);

#endif
