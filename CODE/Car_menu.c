/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */

/*
 * ============================================================================
 * 文件名称：Car_menu.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 校赛知识库的链表式目录与 LCD 菜单骨架。一级目录固定四类：
 *   数独、香橙派、赛题元素PID、电池电压显示。
 *
 * 节点组织
 * ----------------------------------------------------------------------------
 *   car_menu_nodes[] 是只读节点表，下标 0 保留为空索引（CAR_MENU_INDEX_NONE）。
 *   prev/next 组成同级双向链表；child 进入子链表；parent 指向所属页面；
 *   branch_to/merge_to 表示事件分支入口和汇合点；pid_link 表示跨目录链接。
 *
 *   主链 N01~N17 按“发车→第一圈→基本元素→路障分支→第一圈结束→
 *   第二圈低速→红障停车→数独→盲盒→真停车点→结算”组织。
 *   N07 的 branch_to 指向路障颜色分支 N08/N09/N10，三者 merge_to = N11。
 *   路障位置不写死：N07 由颜色识别事件触发，分支结束后汇合回主链。
 *
 * 显示与交互
 * ----------------------------------------------------------------------------
 *   LCD200 只有 ASCII 字体，节点同时保存中文 name 与 ASCII label；
 *   屏幕显示 label/ID/枚举状态，中文 name 留给文档和调试查看。
 *   按键事件由 key.c 提供，导航约定见 Car_menu.h。
 *
 * 执行上下文
 * ----------------------------------------------------------------------------
 *   Car_Menu_Init() 在主循环启动前调用一次；
 *   Car_Menu_Poll() 在主循环中每 5~10 ms 调用；
 *   本模块不修改电机、PID 或圈次执行状态，只维护菜单显示与导航。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"

#include "Car_menu.h"
#include "key.h"

/*
 * 节点索引。
 * 0 保留为空索引，因此指定初始化器里省略的链接字段默认就是“无”。
 * 编号与《NEUQ智能车校赛链表式目录》保持一致。
 */
enum
{
    N01 = 1, N02, N03, N04, N05, N06, N07, N08, N09, N10,
    N11, N12, N13, N14, N15, N16, N17,

    S01, S02, S03, S04, S05, S06, S07, S08, S09,

    O01, O02, O03, O04, O05, O06, O07, O08,

    P01, P02, P03, P04, P05, P06, P07, P08, P09, P10, P11, P12, P13,

    B01, B02, B03, B04, B05, B06,

    CAR_MENU_NODE_COUNT
};

/* 四个一级目录的入口节点，顺序与 CarMenuCategory 一致。 */
static const int16_t category_head[CAR_MENU_CAT_COUNT] =
{
    S01,    /* 数独 */
    O01,    /* 香橙派 */
    N01,    /* 赛题元素PID：主链入口 */
    B01     /* 电池电压显示 */
};

/*
 * 链表节点表。
 * 只写非默认字段：C99 指定初始化器会把未写字段清零，即“无链接”。
 * 需要修改链接时只改 prev/next/branch_to/merge_to/parent/child/pid_link。
 */
static const CarMenuNode car_menu_nodes[CAR_MENU_NODE_COUNT] =
{
    /* ---------------- 主链：发车到结算 ---------------- */
    [N01] = {
        .id = "N01", .name = "发车启动", .label = "START",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .next = N02
    },
    [N02] = {
        .id = "N02", .name = "第一圈竞速计时", .label = "LAP1 TIME",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = N01, .next = N03, .pid_link = P12
    },
    [N03] = {
        .id = "N03", .name = "直道/弯道巡线", .label = "TRACK",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TOPOLOGY, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_YES, .score_s = 0,
        .prev = N02, .next = N04, .pid_link = P01
    },
    [N04] = {
        .id = "N04", .name = "十字路口通行", .label = "CROSS",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TOPOLOGY, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = N03, .next = N05, .pid_link = P04
    },
    [N05] = {
        .id = "N05", .name = "环岛进入/出环", .label = "RING",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TOPOLOGY, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_YES, .score_s = 45,
        .prev = N04, .next = N06, .pid_link = P03
    },
    [N06] = {
        .id = "N06", .name = "160cm断路通过", .label = "160CM GAP",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TOPOLOGY, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = CAR_MENU_SCORE_INVALID,
        .prev = N05, .next = N07, .pid_link = P06
    },
    [N07] = {
        .id = "N07", .name = "三色路障事件检测", .label = "BLOCK DETECT",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = N06, .next = N11, .branch_to = N08, .merge_to = N11,
        .child = N08, .pid_link = P13
    },
    [N08] = {
        .id = "N08", .name = "绿色路障直通", .label = "GREEN PASS",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 10,
        .next = N09, .merge_to = N11, .parent = N07, .pid_link = P07
    },
    [N09] = {
        .id = "N09", .name = "黄色路障绕行", .label = "YELLOW DETOUR",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_PENALTY,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 10,
        .prev = N08, .next = N10, .merge_to = N11, .parent = N07, .pid_link = P08
    },
    [N10] = {
        .id = "N10", .name = "红色路障第一圈忽略", .label = "RED L1 PASS",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_PENALTY,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 10,
        .prev = N09, .merge_to = N11, .parent = N07, .pid_link = P09
    },
    [N11] = {
        .id = "N11", .name = "第一圈计时结束", .label = "LAP1 END",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = N07, .next = N12, .pid_link = P12
    },
    [N12] = {
        .id = "N12", .name = "第二圈低速开始", .label = "LAP2 LOW",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = N11, .next = N13, .pid_link = P12
    },
    [N13] = {
        .id = "N13", .name = "红色路障第二圈停车5s", .label = "RED L2 STOP5",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_PENALTY,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 10,
        .prev = N12, .next = N14, .pid_link = P09
    },
    [N14] = {
        .id = "N14", .name = "数独采集/拼接/显示", .label = "SUDOKU",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_SCORE,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = N13, .next = N15, .pid_link = P10
    },
    [N15] = {
        .id = "N15", .name = "软件盲盒", .label = "BLIND BOX",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_PENALTY,
        .skippable = CAR_MENU_SKIP_PARTIAL, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_SITE, .status = CAR_MENU_STATUS_SITE,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 20,
        .prev = N14, .next = N16, .pid_link = P11
    },
    [N16] = {
        .id = "N16", .name = "真正停车点", .label = "REAL STOP",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_BASIC,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_SITE, .status = CAR_MENU_STATUS_SITE,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = N15, .next = N17, .pid_link = P11
    },
    [N17] = {
        .id = "N17", .name = "结算/成绩判定", .label = "SCORE",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = N16, .next = P01
    },

    /* ---------------- 一级目录：数独 ---------------- */
    [S01] = {
        .id = "S01", .name = "数独任务总览", .label = "SUDOKU OV",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_SCORE,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .next = S02, .pid_link = P10
    },
    [S02] = {
        .id = "S02", .name = "四份子块采集", .label = "TILE CAPTURE",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = S01, .next = S03, .pid_link = P10
    },
    [S03] = {
        .id = "S03", .name = "Tag36h11 0~7定位与校验", .label = "TAG CHECK",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = S02, .next = S04, .pid_link = P10
    },
    [S04] = {
        .id = "S04", .name = "四子块拼接", .label = "STITCH",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = S03, .next = S05, .pid_link = P10
    },
    [S05] = {
        .id = "S05", .name = "6x6数独求解补全", .label = "SOLVE",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = S04, .next = S06, .pid_link = P10
    },
    [S06] = {
        .id = "S06", .name = "屏幕显示完整答案", .label = "DISPLAY",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = S05, .next = S07, .pid_link = P10
    },
    [S07] = {
        .id = "S07", .name = "数独完成-25s奖励判定", .label = "REWARD -25S",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_SCORE,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = -25,
        .prev = S06, .next = S08, .pid_link = P10
    },
    [S08] = {
        .id = "S08", .name = "数独主动屏蔽与无奖励", .label = "MASK NO REWARD",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = S07, .next = S09, .pid_link = P10
    },
    [S09] = {
        .id = "S09", .name = "数独视觉流程PID链接", .label = "SUDOKU PID",
        .category = CAR_MENU_CAT_SUDOKU, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = S08, .pid_link = P10
    },

    /* ---------------- 一级目录：香橙派 ---------------- */
    [O01] = {
        .id = "O01", .name = "香橙派与核心板型号约束", .label = "BOARD LIMIT",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_HARDWARE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .next = O02
    },
    [O02] = {
        .id = "O02", .name = "摄像头选型与连接", .label = "CAMERA",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_HARDWARE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = O01, .next = O03
    },
    [O03] = {
        .id = "O03", .name = "图像采集管线", .label = "PIPELINE",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = O02, .next = O04, .pid_link = P10
    },
    [O04] = {
        .id = "O04", .name = "算力分配与任务调度", .label = "SCHEDULER",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = O03, .next = O05
    },
    [O05] = {
        .id = "O05", .name = "代码部署与现场调试", .label = "DEPLOY",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = O04, .next = O06
    },
    [O06] = {
        .id = "O06", .name = "与PID/卡尔曼数据流", .label = "DATA FLOW",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = O05, .next = O07, .pid_link = P12
    },
    [O07] = {
        .id = "O07", .name = "日志与故障排查", .label = "LOG",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = O06, .next = O08
    },
    [O08] = {
        .id = "O08", .name = "香橙派现场TBD", .label = "SITE TBD",
        .category = CAR_MENU_CAT_ORANGEPI, .type = CAR_MENU_TYPE_TBD,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_SITE, .status = CAR_MENU_STATUS_TBD,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = O07
    },

    /* ---------------- 一级目录：赛题元素PID ---------------- */
    [P01] = {
        .id = "P01", .name = "巡线转向控制页", .label = "PID STEER",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_YES, .score_s = 0,
        .prev = N17, .next = P02, .pid_page = 1
    },
    [P02] = {
        .id = "P02", .name = "速度闭环控制页", .label = "PID SPEED",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = P01, .next = P03, .pid_page = 2
    },
    [P03] = {
        .id = "P03", .name = "环岛进入/出环控制页", .label = "PID RING",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_YES, .score_s = 0,
        .prev = P02, .next = P04, .pid_page = 3
    },
    [P04] = {
        .id = "P04", .name = "十字路口通行控制页", .label = "PID CROSS",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = P03, .next = P05, .pid_page = 4
    },
    [P05] = {
        .id = "P05", .name = "锥桶内侧避撞控制页", .label = "PID CONE",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_TBD,
        .kalman = CAR_MENU_KALMAN_YES, .score_s = 15,
        .prev = P04, .next = P06, .pid_page = 5
    },
    [P06] = {
        .id = "P06", .name = "160cm断路通过控制页", .label = "PID GAP",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VISION, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = P05, .next = P07, .pid_page = 6
    },
    [P07] = {
        .id = "P07", .name = "绿色路障直通页", .label = "PID GREEN",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = P06, .next = P08, .pid_page = 7
    },
    [P08] = {
        .id = "P08", .name = "黄色路障绕行页", .label = "PID YELLOW",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_FIRST,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_YES, .score_s = 0,
        .prev = P07, .next = P09, .pid_page = 8
    },
    [P09] = {
        .id = "P09", .name = "红色路障第二圈停车5s页", .label = "PID RED STOP",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = P08, .next = P10, .pid_page = 9
    },
    [P10] = {
        .id = "P10", .name = "数独采集/拼接/显示流程页", .label = "PID SUDOKU",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_YES, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_TAG, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = P09, .next = P11, .pid_page = 10
    },
    [P11] = {
        .id = "P11", .name = "软件盲盒与真正停车点页", .label = "PID BLIND",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_PARTIAL, .lap = CAR_MENU_LAP_SECOND,
        .trigger = CAR_MENU_TRIG_SITE, .status = CAR_MENU_STATUS_SITE,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = P10, .next = P12, .pid_page = 11
    },
    [P12] = {
        .id = "P12", .name = "第一圈/第二圈状态切换与安全联锁页", .label = "PID STATE",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_TIMER, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = P11, .next = P13, .pid_page = 12
    },
    [P13] = {
        .id = "P13", .name = "路障检测与颜色分支调度页", .label = "PID DISPATCH",
        .category = CAR_MENU_CAT_PID, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_COLOR, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = P12, .pid_page = 13
    },

    /* ---------------- 一级目录：电池电压显示 ---------------- */
    [B01] = {
        .id = "B01", .name = "电压采集与分压电路", .label = "DIVIDER",
        .category = CAR_MENU_CAT_BATTERY, .type = CAR_MENU_TYPE_HARDWARE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_MANUAL, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .next = B02
    },
    [B02] = {
        .id = "B02", .name = "ADC采样与滤波", .label = "ADC FILTER",
        .category = CAR_MENU_CAT_BATTERY, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VOLTAGE, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = B01, .next = B03
    },
    [B03] = {
        .id = "B03", .name = "屏幕电压显示", .label = "VOLT DISPLAY",
        .category = CAR_MENU_CAT_BATTERY, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VOLTAGE, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = B02, .next = B04
    },
    [B04] = {
        .id = "B04", .name = "低电压报警/降速/安全联锁", .label = "LOW VOLT SAFE",
        .category = CAR_MENU_CAT_BATTERY, .type = CAR_MENU_TYPE_RULE,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VOLTAGE, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = B03, .next = B05, .pid_link = P12
    },
    [B05] = {
        .id = "B05", .name = "与赛题元素PID安全联锁链接", .label = "SAFETY LINK",
        .category = CAR_MENU_CAT_BATTERY, .type = CAR_MENU_TYPE_FLOW,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_VOLTAGE, .status = CAR_MENU_STATUS_CONFIRMED,
        .kalman = CAR_MENU_KALMAN_INDIRECT, .score_s = 0,
        .prev = B04, .next = B06, .pid_link = P12
    },
    [B06] = {
        .id = "B06", .name = "电池电压现场TBD", .label = "SITE TBD",
        .category = CAR_MENU_CAT_BATTERY, .type = CAR_MENU_TYPE_TBD,
        .skippable = CAR_MENU_SKIP_NO, .lap = CAR_MENU_LAP_BOTH,
        .trigger = CAR_MENU_TRIG_SITE, .status = CAR_MENU_STATUS_TBD,
        .kalman = CAR_MENU_KALMAN_NO, .score_s = 0,
        .prev = B05
    }
};

/* 菜单单例。 */
static CarMenu menu;

#if CAR_MENU_USE_LCD
/* LCD 显示用的枚举文本。 */
static const char *const category_text[CAR_MENU_CAT_COUNT] =
{
    "SUDOKU", "ORANGE PI", "RACE PID", "BATTERY"
};

static const char *const type_text[CAR_MENU_TYPE_FLOW + 1] =
{
    "BASIC", "SCORE", "PENALTY", "RULE", "HARDWARE", "TBD", "FLOW"
};

static const char *const skip_text[CAR_MENU_SKIP_PARTIAL + 1] =
{
    "NO", "YES", "PART"
};

static const char *const lap_text[CAR_MENU_LAP_BOTH + 1] =
{
    "LAP1", "LAP2", "BOTH"
};

static const char *const trigger_text[CAR_MENU_TRIG_MANUAL + 1] =
{
    "TOPO", "VISION", "COLOR", "TAG", "SITE", "TIMER", "VOLT", "MANUAL"
};

static const char *const status_text[CAR_MENU_STATUS_TBD + 1] =
{
    "CONFIRMED", "SITE", "TBD"
};

static const char *const kalman_text[CAR_MENU_KALMAN_INDIRECT + 1] =
{
    "NO", "YES", "INDIRECT"
};
#endif

/* -------------------------------------------------------------------------- */
/*                              内部辅助函数                                   */
/* -------------------------------------------------------------------------- */

#if CAR_MENU_USE_LCD
/* 按枚举值取文本，越界时返回 "?"，避免坏索引读越界。 */
static const char *menu_enum_text(const char *const *table, unsigned count,
                                  unsigned value)
{
    return (value < count) ? table[value] : "?";
}

/* 取节点ID文本；空索引返回 "-"。 */
static const char *menu_index_id(int16_t index, char *buffer, size_t size)
{
    if (index > CAR_MENU_INDEX_NONE && index < CAR_MENU_NODE_COUNT)
    {
        snprintf(buffer, size, "%s", car_menu_nodes[index].id);
    }
    else
    {
        snprintf(buffer, size, "-");
    }

    return buffer;
}

/* 把罚时/奖励格式化成 ASCII。 */
static void menu_score_text(int16_t score, char *buffer, size_t size)
{
    if (score == CAR_MENU_SCORE_INVALID)
    {
        snprintf(buffer, size, "INVALID");
    }
    else if (score < 0)
    {
        snprintf(buffer, size, "%dS", (int)score);
    }
    else if (score > 0)
    {
        snprintf(buffer, size, "+%dS", (int)score);
    }
    else
    {
        snprintf(buffer, size, "0S");
    }
}

/* 把 PID 页编号格式化成 "PID-xx"。 */
static void menu_pid_text(uint8_t page, char *buffer, size_t size)
{
    if (page == 0U)
    {
        snprintf(buffer, size, "-");
    }
    else
    {
        snprintf(buffer, size, "PID-%02u", (unsigned)page);
    }
}
#endif  /* CAR_MENU_USE_LCD */

/* 返回栈压入：满时丢弃最旧的一项。 */
static void menu_push(int16_t index)
{
    if (menu.depth >= CAR_MENU_STACK_DEPTH)
    {
        memmove(&menu.stack[0], &menu.stack[1],
                (CAR_MENU_STACK_DEPTH - 1U) * sizeof(menu.stack[0]));
        menu.depth = (uint8_t)(CAR_MENU_STACK_DEPTH - 1U);
    }

    menu.stack[menu.depth] = index;
    menu.depth++;
}

/* 同级移动，到头回绕；子链表也按自己的 prev/next 回绕。 */
static void menu_move(int8_t direction)
{
    const CarMenuNode *node = Car_Menu_CurrentNode();
    int16_t target = (direction < 0) ? node->prev : node->next;

    if (target == CAR_MENU_INDEX_NONE)
    {
        target = menu.current;

        if (direction < 0)
        {
            /* 在表头继续向上：绕到表尾。 */
            while (car_menu_nodes[target].next != CAR_MENU_INDEX_NONE)
            {
                target = car_menu_nodes[target].next;
            }
        }
        else
        {
            /* 在表尾继续向下：绕到表头。 */
            while (car_menu_nodes[target].prev != CAR_MENU_INDEX_NONE)
            {
                target = car_menu_nodes[target].prev;
            }
        }
    }

    if (target != menu.current)
    {
        (void)Car_Menu_Goto(target);
    }
}

/* 切换一级目录：direction = -1 上一类，+1 下一类。 */
static void menu_switch_category(int8_t direction)
{
    int category = (int)menu.category + CAR_MENU_CAT_COUNT + direction;

    category %= CAR_MENU_CAT_COUNT;
    menu.depth = 0;
    (void)Car_Menu_Goto(category_head[category]);
}

/* 回到当前一级目录入口。 */
static void menu_home(void)
{
    menu.depth = 0;
    (void)Car_Menu_Goto(category_head[menu.category]);
}

/* 进入 child；没有 child 时跳到 pid_link。 */
static void menu_enter(void)
{
    const CarMenuNode *node = Car_Menu_CurrentNode();
    int16_t target = node->child;

    if (target == CAR_MENU_INDEX_NONE)
    {
        target = node->pid_link;
    }

    if (target != CAR_MENU_INDEX_NONE)
    {
        menu_push(menu.current);
        (void)Car_Menu_Goto(target);
    }
}

/* 返回上一个进入点；返回栈为空时回到一级目录入口。 */
static void menu_back(void)
{
    if (menu.depth > 0U)
    {
        menu.depth--;
        (void)Car_Menu_Goto(menu.stack[menu.depth]);
    }
    else
    {
        (void)Car_Menu_Goto(category_head[menu.category]);
    }
}

/* 只切换屏幕显示的圈次，不改变底盘执行状态。 */
static void menu_toggle_lap(void)
{
    menu.lap = (menu.lap == CAR_MENU_LAP_FIRST)
             ? CAR_MENU_LAP_SECOND
             : CAR_MENU_LAP_FIRST;
    menu.dirty = 1U;
}

/* 分发一个按键事件。 */
static void menu_dispatch_key(Key_Id id, Key_Event event)
{
    switch (id)
    {
        case KEY_UP:
            switch (event)
            {
                case KEY_EVENT_SINGLE_CLICK:
                case KEY_EVENT_REPEAT:
                    menu_move(-1);
                    break;

                case KEY_EVENT_DOUBLE_CLICK:
                    menu_switch_category(-1);
                    break;

                case KEY_EVENT_LONG_PRESS:
                    menu_home();
                    break;

                default:
                    break;
            }
            break;

        case KEY_DOWN:
            switch (event)
            {
                case KEY_EVENT_SINGLE_CLICK:
                case KEY_EVENT_REPEAT:
                    menu_move(1);
                    break;

                case KEY_EVENT_DOUBLE_CLICK:
                    menu_switch_category(1);
                    break;

                case KEY_EVENT_LONG_PRESS:
                    menu_home();
                    break;

                default:
                    break;
            }
            break;

        case KEY_OK:
            switch (event)
            {
                case KEY_EVENT_SINGLE_CLICK:
                    menu_enter();
                    break;

                case KEY_EVENT_DOUBLE_CLICK:
                    menu_back();
                    break;

                case KEY_EVENT_LONG_PRESS:
                    menu_toggle_lap();
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

/* 取出并处理所有按键事件。 */
static void menu_handle_keys(void)
{
    unsigned i;
    Key_Event event;

    for (i = 0; i < KEY_COUNT; i++)
    {
        while (Key_PollEvent((Key_Id)i, &event))
        {
            menu_dispatch_key((Key_Id)i, event);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                              对外接口                                       */
/* -------------------------------------------------------------------------- */

void Car_Menu_Init(void)
{
    Key_Init();

#if CAR_MENU_USE_LCD
    NEUQ_LCD200_init();
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font16);
    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
    NEUQ_LCD200_Clear();
#endif

    menu.current = N01;
    menu.depth = 0;
    menu.category = (uint8_t)car_menu_nodes[N01].category;
    menu.lap = CAR_MENU_LAP_FIRST;
    menu.dirty = 0;

    Car_Menu_Render();
}

void Car_Menu_Poll(void)
{
    Key_Scan(HAL_GetTick());
    menu_handle_keys();

#if CAR_MENU_USE_LCD
    if (menu.dirty)
    {
        Car_Menu_Render();
        menu.dirty = 0;
    }
#endif
}

void Car_Menu_Render(void)
{
#if CAR_MENU_USE_LCD
    const CarMenuNode *node = Car_Menu_CurrentNode();
    char line[40];
    char score[12];
    char pid[12];
    char prev_id[8];
    char next_id[8];
    char branch_id[8];
    char merge_id[8];
    char parent_id[8];
    char child_id[8];
    char link_id[8];
    uint16_t y = 0U;

    NEUQ_LCD200_SetBackColor(LCD_BLACK);
    NEUQ_LCD200_Clear();

    /* 标题行使用不同颜色，便于快速定位。 */
    NEUQ_LCD200_SetPenColor(LCD_CYAN);
    NEUQ_LCD200_DisplayString(4U, y, (char *)"NEUQ CAR LINK MENU");
    y += 18U;

    NEUQ_LCD200_SetPenColor(LCD_WHITE);

    snprintf(line, sizeof(line), "CAT %u/4: %s",
             (unsigned)node->category + 1U,
             menu_enum_text(category_text, CAR_MENU_CAT_COUNT,
                            (unsigned)node->category));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "NODE: %s %s", node->id, node->label);
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "TYPE: %s",
             menu_enum_text(type_text, CAR_MENU_TYPE_FLOW + 1,
                            (unsigned)node->type));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "SKIP: %s  LAP: %s",
             menu_enum_text(skip_text, CAR_MENU_SKIP_PARTIAL + 1,
                            (unsigned)node->skippable),
             menu_enum_text(lap_text, CAR_MENU_LAP_BOTH + 1,
                            (unsigned)node->lap));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    menu_score_text(node->score_s, score, sizeof(score));
    snprintf(line, sizeof(line), "SCORE: %s  KAL: %s", score,
             menu_enum_text(kalman_text, CAR_MENU_KALMAN_INDIRECT + 1,
                            (unsigned)node->kalman));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    menu_pid_text(node->pid_page, pid, sizeof(pid));
    snprintf(line, sizeof(line), "PID PAGE: %s", pid);
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "STATUS: %s",
             menu_enum_text(status_text, CAR_MENU_STATUS_TBD + 1,
                            (unsigned)node->status));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "TRIGGER: %s",
             menu_enum_text(trigger_text, CAR_MENU_TRIG_MANUAL + 1,
                            (unsigned)node->trigger));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "PREV %s  NEXT %s",
             menu_index_id(node->prev, prev_id, sizeof(prev_id)),
             menu_index_id(node->next, next_id, sizeof(next_id)));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "BRANCH %s  MERGE %s",
             menu_index_id(node->branch_to, branch_id, sizeof(branch_id)),
             menu_index_id(node->merge_to, merge_id, sizeof(merge_id)));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "PARENT %s  CHILD %s",
             menu_index_id(node->parent, parent_id, sizeof(parent_id)),
             menu_index_id(node->child, child_id, sizeof(child_id)));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "LINK %s",
             menu_index_id(node->pid_link, link_id, sizeof(link_id)));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    snprintf(line, sizeof(line), "MENU LAP: %s",
             menu_enum_text(lap_text, CAR_MENU_LAP_BOTH + 1,
                            (unsigned)menu.lap));
    NEUQ_LCD200_DisplayString(4U, y, line);
    y += 18U;

    NEUQ_LCD200_SetPenColor(LCD_GREY);
    NEUQ_LCD200_DisplayString(4U, y, (char *)"OK=ENTER 2xOK=BACK");
    y += 18U;
    NEUQ_LCD200_DisplayString(4U, y, (char *)"HOLD OK=LAP 2xUP=CAT");
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
#else
    menu.dirty = 0;
#endif
}

uint8_t Car_Menu_Goto(int16_t index)
{
    if (index <= CAR_MENU_INDEX_NONE || index >= CAR_MENU_NODE_COUNT)
    {
        return 0U;
    }

    menu.current = index;
    menu.category = (uint8_t)car_menu_nodes[index].category;
    menu.dirty = 1U;
    return 1U;
}

uint8_t Car_Menu_GotoId(const char *id)
{
    int16_t i;

    if (id == 0)
    {
        return 0U;
    }

    for (i = 1; i < CAR_MENU_NODE_COUNT; i++)
    {
        if (strcmp(car_menu_nodes[i].id, id) == 0)
        {
            return Car_Menu_Goto(i);
        }
    }

    return 0U;
}

int16_t Car_Menu_CurrentIndex(void)
{
    return menu.current;
}

const CarMenuNode *Car_Menu_CurrentNode(void)
{
    if (menu.current <= CAR_MENU_INDEX_NONE ||
        menu.current >= CAR_MENU_NODE_COUNT)
    {
        return &car_menu_nodes[N01];
    }

    return &car_menu_nodes[menu.current];
}

const CarMenuNode *Car_Menu_GetNode(int16_t index)
{
    if (index <= CAR_MENU_INDEX_NONE || index >= CAR_MENU_NODE_COUNT)
    {
        return 0;
    }

    return &car_menu_nodes[index];
}

int16_t Car_Menu_NodeCount(void)
{
    return (int16_t)(CAR_MENU_NODE_COUNT - 1);
}

void Car_Menu_SetLap(CarMenuLap lap)
{
    if (lap > CAR_MENU_LAP_BOTH)
    {
        return;
    }

    menu.lap = lap;
    menu.dirty = 1U;
}

CarMenuLap Car_Menu_GetLap(void)
{
    return menu.lap;
}
