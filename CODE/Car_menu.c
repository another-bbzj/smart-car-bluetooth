/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */

/*
 * ============================================================================
 * 文件名称：Car_menu.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 链表式目录的建链、浏览和编辑核心。结构参考提交文件：
 *
 *   1. MenuTable[] 只描述静态配置：name、parent_id、value、min/max/step；
 *   2. Car_Menu_BuildLink() 在运行时把 nodes[] 按 parent_id 建成
 *      父子关系和环形同级双向链表；
 *   3. page 是当前页面，selected 永远是 page 的子节点；
 *   4. editing = 0 浏览，editing = 1 编辑；
 *   5. UP/DOWN 在环形同级链表里移动，OK 进入/编辑，双击 OK 返回，
 *      长按 OK 回到 MAIN。
 *
 *   一级目录固定四类：数独、香橙派、赛题元素PID、电池电压显示。
 *   MAIN 只是导航根页面，不属于第五类。
 *
 * 显示说明
 * ----------------------------------------------------------------------------
 *   LCD200 默认 Cross 横屏是 320x240；原 17 行竖屏布局超过 240 像素高度，
 *   属于显示 bug，已暂时移除渲染代码。本文件的建链/浏览/编辑不依赖 LCD。
 *
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static void menu_reset_params(void)
 *       —— 恢复菜单参数的默认值。
 *   uint8_t Car_Menu_BuildLink(void)
 *       —— 按 MenuTable[].parent_id 建立父子关系和环形同级链。
 *   uint8_t Car_Menu_ValidateLinks(void)
 *       —— 检查建链是否闭合。
 *   uint16_t Car_Menu_LinkErrorCount(void)
 *       —— 最近一次建链自检的错误数。
 *   void Car_Menu_Init(void)
 *       —— 初始化参数、建链、自检并初始化按键。
 *   uint8_t Car_Menu_Update(Key_Id key, Key_Event event)
 *       —— 浏览/编辑状态机。
 *   void Car_Menu_Poll(void)
 *       —— 主循环周期任务。
 *   void Car_Menu_Render(void)
 *       —— 显示入口（暂时空实现）。
 *   uint8_t Car_Menu_Goto(MenuID id)
 *       —— 跳转到指定节点。
 *   MenuID Car_Menu_GetPageId(void)
 *       —— 当前页面编号。
 *   MenuID Car_Menu_GetSelectedId(void)
 *       —— 当前选中项编号。
 *   uint8_t Car_Menu_IsEditing(void)
 *       —— 是否处于编辑模式。
 *   int32_t Car_Menu_GetEditValue(void)
 *       —— 当前编辑值。
 *   void Car_Menu_SetEditValue(int32_t value)
 *       —— 设置编辑值并按范围限幅。
 *   const MenuConfig *Car_Menu_GetConfig(MenuID id)
 *       —— 取节点静态配置。
 *   int16_t Car_Menu_NodeCount(void)
 *       —— 节点总数。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"
#include "Car_menu.h"
#include "key.h"

/* ========================================================================== */
/*                              可编辑参数                                     */
/* ========================================================================== */

CarMenuParams car_menu_params;

/* ========================================================================== */
/*                              静态配置表                                     */
/* ========================================================================== */

#define MENU_ITEM(name_, parent_, value_, min_, max_, step_, type_, skip_, lap_, score_, kalman_, status_, link_) \
    { (name_), (parent_), (value_), (min_), (max_), (step_), (type_), (skip_), (lap_), (score_), (kalman_), (status_), (link_) }

/*
 * 建链数据源：
 *   parent_id 决定父子关系；
 *   MENU_COUNT 的枚举顺序决定同级节点的初始排列顺序；
 *   value 非空表示该页面可以直接编辑参数。
 *
 * 主链层级：
 *   MAIN -> CAT_* -> 竞赛节点；
 *   N07 -> N08/N09/N10 构成路障颜色分支子链；
 *   CAT_PID 的同级顺序为 N01..N07、N11..N17、P01..P13。
 */
static const MenuConfig MenuTable[MENU_COUNT] =
{
    [MAIN] = MENU_ITEM("MAIN", MENU_NONE, NULL, 0, 0, 0,
                       CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                       0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),

    [CAT_SUDOKU] = MENU_ITEM("SUDOKU", MAIN, NULL, 0, 0, 0,
                             CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                             0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [CAT_ORANGEPI] = MENU_ITEM("ORANGE PI", MAIN, NULL, 0, 0, 0,
                               CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                               0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [CAT_PID] = MENU_ITEM("RACE PID", MAIN, NULL, 0, 0, 0,
                          CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                          0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [CAT_BATTERY] = MENU_ITEM("BATTERY", MAIN, NULL, 0, 0, 0,
                              CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                              0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),

    /* ---------------- 一级目录：数独 ---------------- */
    [S01] = MENU_ITEM("SUDOKU OV", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_SCORE, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S02] = MENU_ITEM("TILE CAPTURE", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S03] = MENU_ITEM("TAG CHECK", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S04] = MENU_ITEM("STITCH", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S05] = MENU_ITEM("SOLVE", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S06] = MENU_ITEM("DISPLAY", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S07] = MENU_ITEM("REWARD -25S", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_SCORE, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [S08] = MENU_ITEM("MASK NO REWARD", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P10),
    [S09] = MENU_ITEM("SUDOKU PID", CAT_SUDOKU, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),

    /* ---------------- 一级目录：香橙派 ---------------- */
    [O01] = MENU_ITEM("BOARD LIMIT", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_HARDWARE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [O02] = MENU_ITEM("CAMERA", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_HARDWARE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [O03] = MENU_ITEM("PIPELINE", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [O04] = MENU_ITEM("SCHEDULER", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [O05] = MENU_ITEM("DEPLOY", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [O06] = MENU_ITEM("DATA FLOW", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P12),
    [O07] = MENU_ITEM("LOG", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [O08] = MENU_ITEM("SITE TBD", CAT_ORANGEPI, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_TBD, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_TBD, MENU_NONE),

    /* ---------------- 一级目录：赛题元素PID ---------------- */
    [N01] = MENU_ITEM("START", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [N02] = MENU_ITEM("LAP1 TIME", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P12),
    [N03] = MENU_ITEM("TRACK", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_YES, CAR_MENU_STATUS_CONFIRMED, P01),
    [N04] = MENU_ITEM("CROSS", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P04),
    [N05] = MENU_ITEM("RING", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      45, CAR_MENU_KALMAN_YES, CAR_MENU_STATUS_CONFIRMED, P03),
    [N06] = MENU_ITEM("160CM GAP", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      CAR_MENU_SCORE_INVALID, CAR_MENU_KALMAN_INDIRECT,
                      CAR_MENU_STATUS_CONFIRMED, P06),
    [N07] = MENU_ITEM("BLOCK DETECT", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P13),
    [N08] = MENU_ITEM("GREEN PASS", N07, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      10, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P07),
    [N09] = MENU_ITEM("YELLOW DETOUR", N07, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_PENALTY, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      10, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P08),
    [N10] = MENU_ITEM("RED L1 PASS", N07, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_PENALTY, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      10, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P09),
    [N11] = MENU_ITEM("LAP1 END", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P12),
    [N12] = MENU_ITEM("LAP2 LOW", CAT_PID, &car_menu_params.speed_limit, 0, 500, 5,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P12),
    [N13] = MENU_ITEM("RED L2 STOP5", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_PENALTY, CAR_MENU_SKIP_NO, CAR_MENU_LAP_SECOND,
                      10, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, P09),
    [N14] = MENU_ITEM("SUDOKU", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_SCORE, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      -25, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P10),
    [N15] = MENU_ITEM("BLIND BOX", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_PENALTY, CAR_MENU_SKIP_PARTIAL, CAR_MENU_LAP_SECOND,
                      20, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_SITE, P11),
    [N16] = MENU_ITEM("REAL STOP", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_BASIC, CAR_MENU_SKIP_NO, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_SITE, P11),
    [N17] = MENU_ITEM("SCORE", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),

    /* ---------------- 一级目录：赛题元素PID 的 PID 页 ---------------- */
    [P01] = MENU_ITEM("PID STEER", CAT_PID, &car_menu_params.pid_kp, 0, 65535, 1,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_YES, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P02] = MENU_ITEM("PID SPEED", CAT_PID, &car_menu_params.pid_ki, 0, 65535, 1,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P03] = MENU_ITEM("PID RING", CAT_PID, &car_menu_params.ring_speed, 0, 500, 5,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_YES, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P04] = MENU_ITEM("PID CROSS", CAT_PID, &car_menu_params.kalman_q, 0, 100000, 10,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P05] = MENU_ITEM("PID CONE", CAT_PID, &car_menu_params.cone_offset, 0, 500, 5,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      15, CAR_MENU_KALMAN_YES, CAR_MENU_STATUS_TBD, MENU_NONE),
    [P06] = MENU_ITEM("PID GAP", CAT_PID, &car_menu_params.gap_speed, 0, 500, 5,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P07] = MENU_ITEM("PID GREEN", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P08] = MENU_ITEM("PID YELLOW", CAT_PID, &car_menu_params.kalman_r, 0, 100000, 10,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_FIRST,
                      0, CAR_MENU_KALMAN_YES, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P09] = MENU_ITEM("PID RED STOP", CAT_PID, &car_menu_params.red_stop_s, 1, 10, 1,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P10] = MENU_ITEM("PID SUDOKU", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_YES, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P11] = MENU_ITEM("PID BLIND", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_PARTIAL, CAR_MENU_LAP_SECOND,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_SITE, MENU_NONE),
    [P12] = MENU_ITEM("PID STATE", CAT_PID, &car_menu_params.lap_mode, 0, 1, 1,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [P13] = MENU_ITEM("PID DISPATCH", CAT_PID, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),

    /* ---------------- 一级目录：电池电压显示 ---------------- */
    [B01] = MENU_ITEM("DIVIDER", CAT_BATTERY, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_HARDWARE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [B02] = MENU_ITEM("ADC FILTER", CAT_BATTERY, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [B03] = MENU_ITEM("VOLT DISPLAY", CAT_BATTERY, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, MENU_NONE),
    [B04] = MENU_ITEM("LOW VOLT SAFE", CAT_BATTERY, &car_menu_params.battery_low,
                      5000, 12600, 100,
                      CAR_MENU_TYPE_RULE, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P12),
    [B05] = MENU_ITEM("SAFETY LINK", CAT_BATTERY, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_FLOW, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_INDIRECT, CAR_MENU_STATUS_CONFIRMED, P12),
    [B06] = MENU_ITEM("SITE TBD", CAT_BATTERY, NULL, 0, 0, 0,
                      CAR_MENU_TYPE_TBD, CAR_MENU_SKIP_NO, CAR_MENU_LAP_BOTH,
                      0, CAR_MENU_KALMAN_NO, CAR_MENU_STATUS_TBD, MENU_NONE)
};

/* ========================================================================== */
/*                              运行时节点                                     */
/* ========================================================================== */

typedef struct MenuNode
{
    const MenuConfig *info;  /* 节点静态配置 */
    struct MenuNode *parent; /* 父节点 */
    struct MenuNode *child;  /* 第一个子节点 */
    struct MenuNode *prev;   /* 同级前驱（环形） */
    struct MenuNode *next;   /* 同级后继（环形） */
} MenuNode;

static MenuNode nodes[MENU_COUNT]; /* 固定内存，不申请堆 */
static MenuNode *page;              /* 当前页面 */
static MenuNode *selected;          /* 当前选中项，永远是 page 的子节点 */
static uint8_t editing;             /* 0 浏览，1 编辑 */
static int32_t edit_value;          /* 编辑中的临时值 */
static uint16_t menu_link_errors;   /* 建链自检错误数 */

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/*
 * 函数名称：menu_reset_params
 * 函数功能：恢复菜单可编辑参数的默认值。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     menu_reset_params();
 * 备注信息：这些值只是菜单侧的初始值；控制层是否采用由上层决定。
 */
static void menu_reset_params(void)
{
    car_menu_params.pid_kp = 200;       /* 2.00 */
    car_menu_params.pid_ki = 12;        /* 0.12 */
    car_menu_params.pid_kd = 0;
    car_menu_params.kalman_q = 100;     /* 0.100 */
    car_menu_params.kalman_r = 1000;    /* 1.000 */
    car_menu_params.lap_mode = 0;       /* 第一圈 */
    car_menu_params.speed_limit = 180;  /* rpm */
    car_menu_params.ring_speed = 120;   /* rpm */
    car_menu_params.cone_offset = 150;  /* mm */
    car_menu_params.red_stop_s = 5;     /* s */
    car_menu_params.gap_speed = 160;    /* rpm */
    car_menu_params.battery_low = 6600; /* mV */
}

/* ========================================================================== */
/*                              建链与自检                                     */
/* ========================================================================== */

/*
 * 函数名称：Car_Menu_BuildLink
 * 函数功能：按 MenuTable[].parent_id 建立父子关系和环形同级双向链表。
 * 参数说明：无。
 * 返回参数：1 成功；0 parent_id 非法或自引用。
 * 使用示例：
 *     Car_Menu_BuildLink();
 * 备注信息：节点内存固定为 nodes[MENU_COUNT]，不需要 malloc；
 *           同级链首尾相接，UP/DOWN 到头自动回绕。
 */
uint8_t Car_Menu_BuildLink(void)
{
    uint16_t errors = 0U;
    int i;

    for (i = 0; i < MENU_COUNT; i++)
    {
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = NULL;
        nodes[i].next = NULL;
    }

    for (i = 0; i < MENU_COUNT; i++)
    {
        MenuID parent_id = MenuTable[i].parent_id;
        MenuNode *node;
        MenuNode *parent;
        MenuNode *first;
        MenuNode *last;

        if (parent_id == MENU_NONE)
        {
            continue;
        }

        if (parent_id < 0 || parent_id >= MENU_COUNT || (int)parent_id == i)
        {
            errors++;
            continue;
        }

        node = &nodes[i];
        parent = &nodes[parent_id];
        node->parent = parent;

        first = parent->child;
        if (first == NULL)
        {
            parent->child = node;
            node->prev = node;
            node->next = node;
        }
        else
        {
            last = first->prev;
            node->prev = last;
            last->next = node;
            node->next = first;
            first->prev = node;
        }
    }

    menu_link_errors = errors;
    return (errors == 0U) ? 1U : 0U;
}

/*
 * 函数名称：Car_Menu_ValidateLinks
 * 函数功能：检查建链是否闭合、同级链是否环形、父链是否可达 MAIN。
 * 参数说明：无。
 * 返回参数：1 通过；0 存在错误。
 * 使用示例：
 *     if (!Car_Menu_ValidateLinks())
 *     {
 *         // 建链错误
 *     }
 * 备注信息：只读检查，不修改 nodes[]；结果写入 menu_link_errors。
 */
uint8_t Car_Menu_ValidateLinks(void)
{
    uint16_t errors = 0U;
    int i;
    int root_count = 0;

    for (i = 0; i < MENU_COUNT; i++)
    {
        const MenuConfig *config = &MenuTable[i];

        if (nodes[i].info != config)
        {
            errors++;
        }

        if (config->parent_id == MENU_NONE)
        {
            root_count++;
            if (nodes[i].parent != NULL)
            {
                errors++;
            }
        }
        else if (config->parent_id < 0 || config->parent_id >= MENU_COUNT)
        {
            errors++;
        }
        else if (nodes[i].parent != &nodes[config->parent_id])
        {
            errors++;
        }

        if (config->value != NULL)
        {
            if (config->min > config->max || config->step <= 0)
            {
                errors++;
            }
        }

        if (config->pid_link != MENU_NONE &&
            (config->pid_link < 0 || config->pid_link >= MENU_COUNT))
        {
            errors++;
        }
    }

    if (root_count != 1 || nodes[MAIN].parent != NULL)
    {
        errors++;
    }

    for (i = 0; i < MENU_COUNT; i++)
    {
        MenuNode *first = nodes[i].child;

        if (first != NULL)
        {
            MenuNode *current = first;
            uint16_t count = 0U;

            do
            {
                if (current->parent != &nodes[i] ||
                    current->next == NULL || current->next->prev != current ||
                    current->prev == NULL || current->prev->next != current)
                {
                    errors++;
                    break;
                }

                current = current->next;
                count++;
            } while (current != first && count < (uint16_t)MENU_COUNT);

            if (current != first || count == 0U)
            {
                errors++;
            }
        }
    }

    for (i = 1; i < MENU_COUNT; i++)
    {
        const MenuNode *current = &nodes[i];
        uint16_t depth = 0U;

        while (current->parent != NULL)
        {
            current = current->parent;
            depth++;
            if (depth > (uint16_t)MENU_COUNT)
            {
                errors++;
                break;
            }
        }

        if (current != &nodes[MAIN])
        {
            errors++;
        }
    }

    menu_link_errors = errors;
    return (errors == 0U) ? 1U : 0U;
}

/*
 * 函数名称：Car_Menu_LinkErrorCount
 * 函数功能：返回最近一次建链自检发现的错误数量。
 * 参数说明：无。
 * 返回参数：错误数；0 表示通过。
 * 使用示例：
 *     uint16_t errors = Car_Menu_LinkErrorCount();
 * 备注信息：每次 Car_Menu_BuildLink() 或 Car_Menu_ValidateLinks() 后更新。
 */
uint16_t Car_Menu_LinkErrorCount(void)
{
    return menu_link_errors;
}

/* ========================================================================== */
/*                              浏览与编辑                                     */
/* ========================================================================== */

/*
 * 函数名称：Car_Menu_Update
 * 函数功能：根据按键事件推进浏览或编辑状态机。
 * 参数说明：key   —— 按键编号（KEY_UP/KEY_DOWN/KEY_OK）；
 *           event —— 按键事件。
 * 返回参数：1 状态改变；0 忽略。
 * 使用示例：
 *     Car_Menu_Update(KEY_OK, KEY_EVENT_SINGLE_CLICK);
 * 备注信息：浏览模式下 UP/DOWN 移动，OK 进入/编辑，双击 OK 返回，
 *           长按 OK 回到 MAIN；编辑模式下 UP/DOWN 调整，OK 提交，
 *           双击 OK 取消。
 */
uint8_t Car_Menu_Update(Key_Id key, Key_Event event)
{
    if (page == NULL || selected == NULL)
    {
        return 0U;
    }

    if (editing)
    {
        const MenuConfig *item = selected->info;
        int32_t next;

        if (item == NULL || item->value == NULL)
        {
            editing = 0U;
            return 1U;
        }

        switch (key)
        {
            case KEY_UP:
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT || event == KEY_EVENT_LONG_PRESS)
                {
                    next = edit_value + item->step;
                    if (next > item->max)
                    {
                        next = item->max;
                    }
                    if (next != edit_value)
                    {
                        edit_value = next;
                        return 1U;
                    }
                }
                break;

            case KEY_DOWN:
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT || event == KEY_EVENT_LONG_PRESS)
                {
                    next = edit_value - item->step;
                    if (next < item->min)
                    {
                        next = item->min;
                    }
                    if (next != edit_value)
                    {
                        edit_value = next;
                        return 1U;
                    }
                }
                break;

            case KEY_OK:
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_LONG_PRESS)
                {
                    *item->value = edit_value;
                    editing = 0U;
                    return 1U;
                }
                if (event == KEY_EVENT_DOUBLE_CLICK)
                {
                    editing = 0U;
                    return 1U;
                }
                break;

            default:
                break;
        }

        return 0U;
    }

    switch (key)
    {
        case KEY_UP:
            if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT || event == KEY_EVENT_LONG_PRESS)
            {
                selected = selected->prev;
                return 1U;
            }
            break;

        case KEY_DOWN:
            if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT || event == KEY_EVENT_LONG_PRESS)
            {
                selected = selected->next;
                return 1U;
            }
            break;

        case KEY_OK:
            if (event == KEY_EVENT_SINGLE_CLICK)
            {
                if (selected->child != NULL)
                {
                    page = selected;
                    selected = page->child;
                    return 1U;
                }

                if (selected->info != NULL && selected->info->value != NULL)
                {
                    edit_value = *selected->info->value;
                    if (edit_value < selected->info->min)
                    {
                        edit_value = selected->info->min;
                    }
                    if (edit_value > selected->info->max)
                    {
                        edit_value = selected->info->max;
                    }
                    editing = 1U;
                    return 1U;
                }

                return 0U;
            }

            if (event == KEY_EVENT_DOUBLE_CLICK)
            {
                if (page->parent != NULL)
                {
                    selected = page;
                    page = page->parent;
                    return 1U;
                }
                return 0U;
            }

            if (event == KEY_EVENT_LONG_PRESS)
            {
                page = &nodes[MAIN];
                selected = page->child;
                editing = 0U;
                return 1U;
            }
            break;

        default:
            break;
    }

    return 0U;
}

/* ========================================================================== */
/*                              对外接口                                       */
/* ========================================================================== */

/*
 * 函数名称：Car_Menu_Init
 * 函数功能：初始化参数、建链、自检并初始化按键。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     Car_Menu_Init();
 * 备注信息：不初始化 LCD；当前显示层已暂时移除。
 */
void Car_Menu_Init(void)
{
    menu_reset_params();
    (void)Car_Menu_BuildLink();
    (void)Car_Menu_ValidateLinks();

    page = &nodes[MAIN];
    selected = page->child;
    editing = 0U;
    edit_value = 0;

    Key_Init();
}

/*
 * 函数名称：Car_Menu_Poll
 * 函数功能：扫描按键并处理全部待处理事件。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     while (1) { Car_Menu_Poll(); }
 * 备注信息：建议每 5~10 ms 调用一次；显示层暂时为空。
 */
void Car_Menu_Poll(void)
{
    unsigned i;
    Key_Event event;

    Key_Scan(HAL_GetTick());

    for (i = 0U; i < KEY_COUNT; i++)
    {
        while (Key_PollEvent((Key_Id)i, &event))
        {
            (void)Car_Menu_Update((Key_Id)i, event);
        }
    }
}

/*
 * 函数名称：Car_Menu_Render
 * 函数功能：显示入口；当前为空实现。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     Car_Menu_Render();
 * 备注信息：LCD200 默认 Cross 横屏为 320x240，原竖屏 17 行布局超出高度，
 *           属于显示 bug，已暂时删除渲染代码；待按横屏重新设计后恢复。
 */
void Car_Menu_Render(void)
{
    /* 暂时不显示，避免越界写屏。 */
}

/*
 * 函数名称：Car_Menu_Goto
 * 函数功能：跳转到指定节点。
 * 参数说明：id —— 目标节点编号。
 * 返回参数：1 成功；0 编号无效。
 * 使用示例：
 *     Car_Menu_Goto(P10);
 * 备注信息：叶子节点进入其父页面并选中自己；页面节点进入自身。
 */
uint8_t Car_Menu_Goto(MenuID id)
{
    MenuNode *node;

    if (id < 0 || id >= MENU_COUNT)
    {
        return 0U;
    }

    node = &nodes[id];

    if (node->parent != NULL)
    {
        page = node->parent;
        selected = node;
    }
    else
    {
        page = node;
        selected = node->child;
    }

    editing = 0U;
    return 1U;
}

/*
 * 函数名称：Car_Menu_GetPageId
 * 函数功能：返回当前页面编号。
 * 参数说明：无。
 * 返回参数：页面编号；未初始化返回 MENU_NONE。
 * 使用示例：
 *     MenuID page_id = Car_Menu_GetPageId();
 * 备注信息：通过指针差计算，nodes[] 是固定数组。
 */
MenuID Car_Menu_GetPageId(void)
{
    return (page != NULL) ? (MenuID)(page - nodes) : MENU_NONE;
}

/*
 * 函数名称：Car_Menu_GetSelectedId
 * 函数功能：返回当前选中项编号。
 * 参数说明：无。
 * 返回参数：选中项编号；未初始化返回 MENU_NONE。
 * 使用示例：
 *     MenuID selected_id = Car_Menu_GetSelectedId();
 * 备注信息：selected 永远是 page 的子节点。
 */
MenuID Car_Menu_GetSelectedId(void)
{
    return (selected != NULL) ? (MenuID)(selected - nodes) : MENU_NONE;
}

/*
 * 函数名称：Car_Menu_IsEditing
 * 函数功能：返回当前是否处于编辑模式。
 * 参数说明：无。
 * 返回参数：1 编辑；0 浏览。
 * 使用示例：
 *     if (Car_Menu_IsEditing()) { }
 * 备注信息：无。
 */
uint8_t Car_Menu_IsEditing(void)
{
    return editing;
}

/*
 * 函数名称：Car_Menu_GetEditValue
 * 函数功能：返回编辑中的临时值。
 * 参数说明：无。
 * 返回参数：编辑值；非编辑模式返回 0。
 * 使用示例：
 *     int32_t value = Car_Menu_GetEditValue();
 * 备注信息：无。
 */
int32_t Car_Menu_GetEditValue(void)
{
    return (editing != 0U) ? edit_value : 0;
}

/*
 * 函数名称：Car_Menu_SetEditValue
 * 函数功能：直接设置编辑值，并按当前选中项的范围限幅。
 * 参数说明：value —— 新值。
 * 返回参数：无。
 * 使用示例：
 *     Car_Menu_SetEditValue(200);
 * 备注信息：用于测试或外部恢复；提交仍由 Car_Menu_Update() 完成。
 */
void Car_Menu_SetEditValue(int32_t value)
{
    edit_value = value;

    if (editing != 0U && selected != NULL && selected->info != NULL &&
        selected->info->value != NULL)
    {
        if (edit_value < selected->info->min)
        {
            edit_value = selected->info->min;
        }
        if (edit_value > selected->info->max)
        {
            edit_value = selected->info->max;
        }
    }
}

/*
 * 函数名称：Car_Menu_GetConfig
 * 函数功能：按编号取节点的静态配置。
 * 参数说明：id —— 节点编号。
 * 返回参数：配置指针；编号无效返回 NULL。
 * 使用示例：
 *     const MenuConfig *cfg = Car_Menu_GetConfig(P10);
 * 备注信息：返回只读常量表指针，调用者不得写入。
 */
const MenuConfig *Car_Menu_GetConfig(MenuID id)
{
    if (id < 0 || id >= MENU_COUNT)
    {
        return NULL;
    }

    return &MenuTable[id];
}

/*
 * 函数名称：Car_Menu_NodeCount
 * 函数功能：返回节点总数（不含 MENU_NONE）。
 * 参数说明：无。
 * 返回参数：MENU_COUNT。
 * 使用示例：
 *     for (i = 0; i < Car_Menu_NodeCount(); i++) { }
 * 备注信息：当前为 58 个：MAIN + 4 个目录 + 53 个竞赛节点。
 */
int16_t Car_Menu_NodeCount(void)
{
    return (int16_t)MENU_COUNT;
}
