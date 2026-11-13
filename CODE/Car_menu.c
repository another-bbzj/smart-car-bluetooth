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
 * 函数列表（按文件内出现顺序）
 * ----------------------------------------------------------------------------
 *   static const char *menu_enum_text(const char *const *table, unsigned count,
 *                                     unsigned value)
 *       —— 按枚举值取 LCD 文本，越界返回 "?"。
 *   static const char *menu_index_id(int16_t index, char *buffer, size_t size)
 *       —— 把节点索引转换成 ID 文本。
 *   static void menu_score_text(int16_t score, char *buffer, size_t size)
 *       —— 把罚时/奖励格式化成 ASCII。
 *   static void menu_pid_text(uint8_t page, char *buffer, size_t size)
 *       —— 把 PID 页编号格式化成 "PID-xx"。
 *   static void menu_push(int16_t index)
 *       —— 把当前节点压入返回栈。
 *   static void menu_move(int8_t direction)
 *       —— 同级前后移动，到头回绕。
 *   static void menu_switch_category(int8_t direction)
 *       —— 切换一级目录。
 *   static void menu_home(void)
 *       —— 回到当前一级目录入口。
 *   static void menu_enter(void)
 *       —— 进入 child，否则跳到 pid_link。
 *   static void menu_back(void)
 *       —— 返回上一个进入点。
 *   static void menu_toggle_lap(void)
 *       —— 切换屏幕显示的第一圈/第二圈。
 *   static void menu_dispatch_key(Key_Id id, Key_Event event)
 *       —— 分发一个按键事件。
 *   static void menu_handle_keys(void)
 *       —— 取出并处理全部按键事件。
 *   void Car_Menu_Init(void)
 *       —— 初始化按键和 LCD，定位到主链入口并绘制首屏。
 *   void Car_Menu_Poll(void)
 *       —— 周期扫描按键、处理事件、按需重绘。
 *   void Car_Menu_Render(void)
 *       —— 绘制当前节点页面。
 *   uint8_t Car_Menu_Goto(int16_t index)
 *       —— 按索引跳转。
 *   uint8_t Car_Menu_GotoId(const char *id)
 *       —— 按节点 ID 跳转。
 *   int16_t Car_Menu_CurrentIndex(void)
 *       —— 当前节点索引。
 *   const CarMenuNode *Car_Menu_CurrentNode(void)
 *       —— 当前节点只读指针。
 *   const CarMenuNode *Car_Menu_GetNode(int16_t index)
 *       —— 按索引取节点只读指针。
 *   int16_t Car_Menu_NodeCount(void)
 *       —— 节点总数（不含空索引）。
 *   static uint8_t menu_valid_index(int16_t index)
 *       —— 判断节点索引是否有效。
 *   static uint8_t menu_in_child_list(int16_t parent, int16_t node)
 *       —— 判断节点是否位于父节点的 child 子链表。
 *   uint8_t Car_Menu_ValidateLinks(void)
 *       —— 检查 next/prev、parent/child、branch/merge、pid_link 和
 *          pid_page 的建链一致性。
 *   uint16_t Car_Menu_LinkErrorCount(void)
 *       —— 返回最近一次建链自检的错误数。
 *   void Car_Menu_SetLap(CarMenuLap lap)
 *       —— 设置屏幕显示的圈次。
 *   CarMenuLap Car_Menu_GetLap(void)
 *       —— 读取屏幕显示的圈次。
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

/* ========================================================================== */
/*                          建链表（静态链接）                                 */
/* ========================================================================== */

/*
 * 建链方式
 * ----------------------------------------------------------------------------
 * 本模块不申请堆内存，也不在运行时修补指针。链表的“建链”发生在这张只读
 * 节点表的指定初始化器里：
 *
 *   - 数组下标 0 保留为 CAR_MENU_INDEX_NONE，所有省略的链接字段自然为 0；
 *   - prev/next 建立同级双向链；
 *   - parent/child 建立页面层级：child 是子链表的入口；
 *   - branch_to/merge_to 建立事件分支与汇合点；
 *   - pid_link 建立跨一级目录的双向链接；
 *   - pid_page 给 P01~P13 编号，供 PID 页索引使用。
 *
 * 主链建链顺序
 * ----------------------------------------------------------------------------
 *   N01→N02→…→N06→N07；N07.next=N11 表示模板主链跳过颜色分支；
 *   N07.child=N08 挂出路障子链 N08→N09→N10，三者 merge_to=N11；
 *   N11→N12→…→N17；N17.next=P01；P01→P02→…→P13。
 *   四条一级目录链分别以 S01、O01、N01、B01 为入口。
 *
 * 建链自检
 * ----------------------------------------------------------------------------
 *   Car_Menu_ValidateLinks() 会检查 next/prev 互逆、parent/child 包含关系、
 *   branch_to/merge_to/pid_link 目标、pid_page 唯一性和四个目录入口。
 *   修改本表后应先跑一次自检，再上电联调。
 *
 * 修改约定
 * ----------------------------------------------------------------------------
 *   只写非默认字段；C99 指定初始化器会把未写字段清零，即“无链接”。
 *   需要修改链接时只改 prev/next/branch_to/merge_to/parent/child/pid_link。
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

/* ========================================================================== */
/*                           菜单单例与 LCD 文本表                             */
/* ========================================================================== */

/* 菜单单例。 */
static CarMenu menu;

/* 最近一次建链自检发现的错误数，供调试和页面显示使用。 */
static uint16_t menu_link_errors;

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
/*
 * 函数名称：menu_enum_text
 * 函数功能：按枚举值从只读文本表中取字符串，避免坏索引读越界。
 * 参数说明：table —— 文本表指针；
 *           count —— 文本表项数；
 *           value —— 待查的枚举值。
 * 返回参数：对应文本；越界返回 "?"。
 * 使用示例：
 *     const char *text = menu_enum_text(type_text, CAR_MENU_TYPE_FLOW + 1,
 *                                       node->type);
 * 备注信息：仅在本文件的绘制函数中使用。
 */
static const char *menu_enum_text(const char *const *table, unsigned count,
                                  unsigned value)
{
    return (value < count) ? table[value] : "?";
}

/*
 * 函数名称：menu_index_id
 * 函数功能：把节点索引转换成节点 ID 文本，供 LCD 显示链接字段。
 * 参数说明：index  —— 节点索引，0 表示空；
 *           buffer —— 输出缓冲区；
 *           size   —— 缓冲区字节数。
 * 返回参数：buffer 指针。
 * 使用示例：
 *     char text[8];
 *     menu_index_id(node->next, text, sizeof(text));
 * 备注信息：空索引或越界时写入 "-"，保证调用者总能拿到可显示文本。
 */
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

/*
 * 函数名称：menu_score_text
 * 函数功能：把节点的罚时/奖励数值格式化成 LCD 可显示的 ASCII 文本。
 * 参数说明：score  —— 分数，正数罚时、负数奖励、CAR_MENU_SCORE_INVALID 无效；
 *           buffer —— 输出缓冲区；
 *           size   —— 缓冲区字节数。
 * 返回参数：无。
 * 使用示例：
 *     char text[12];
 *     menu_score_text(node->score_s, text, sizeof(text));
 * 备注信息：输出 "+10S"、"-25S"、"0S" 或 "INVALID"。
 */
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

/*
 * 函数名称：menu_pid_text
 * 函数功能：把 PID 页编号格式化成 "PID-xx" 文本。
 * 参数说明：page   —— PID 页编号 1~13，0 表示非 PID 页；
 *           buffer —— 输出缓冲区；
 *           size   —— 缓冲区字节数。
 * 返回参数：无。
 * 使用示例：
 *     char text[12];
 *     menu_pid_text(node->pid_page, text, sizeof(text));
 * 备注信息：page = 0 时输出 "-"。
 */
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

/* ========================================================================== */
/*                              菜单导航状态机                                 */
/* ========================================================================== */

/*
 * 函数名称：menu_push
 * 函数功能：把当前节点索引压入返回栈，供 menu_back() 使用。
 * 参数说明：index —— 待压入的节点索引。
 * 返回参数：无。
 * 使用示例：
 *     menu_push(menu.current);
 * 备注信息：栈满时丢弃最旧的一项，保证最近一次进入点一定可返回。
 */
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

/*
 * 函数名称：menu_move
 * 函数功能：在当前同级链表中前后移动，到达表头/表尾时回绕。
 * 参数说明：direction —— 小于 0 向前（prev），大于 0 向后（next）。
 * 返回参数：无。
 * 使用示例：
 *     menu_move(-1);  // 上一项
 *     menu_move(1);   // 下一项
 * 备注信息：子链表（如路障颜色分支）同样使用自身的 prev/next 回绕。
 */
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

/*
 * 函数名称：menu_switch_category
 * 函数功能：按固定顺序切换一级目录，并跳到该类入口节点。
 * 参数说明：direction —— -1 上一类，+1 下一类。
 * 返回参数：无。
 * 使用示例：
 *     menu_switch_category(1);  // 数独 -> 香橙派 -> 赛题元素PID -> 电池
 * 备注信息：切换时清空返回栈；四个一级目录按 CarMenuCategory 顺序回绕。
 */
static void menu_switch_category(int8_t direction)
{
    int category = (int)menu.category + CAR_MENU_CAT_COUNT + direction;

    category %= CAR_MENU_CAT_COUNT;
    menu.depth = 0;
    (void)Car_Menu_Goto(category_head[category]);
}

/*
 * 函数名称：menu_home
 * 函数功能：清空返回栈并回到当前一级目录的入口节点。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     menu_home();  // 长按 UP/DOWN 时调用
 * 备注信息：入口节点由 category_head[menu.category] 决定。
 */
static void menu_home(void)
{
    menu.depth = 0;
    (void)Car_Menu_Goto(category_head[menu.category]);
}

/*
 * 函数名称：menu_enter
 * 函数功能：进入当前节点的子链表；没有 child 时跳到跨目录 pid_link。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     menu_enter();  // OK 单击时调用
 * 备注信息：跳转前把当前节点压入返回栈；两者都没有时保持原页面。
 */
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

/*
 * 函数名称：menu_back
 * 函数功能：返回上一个进入点；返回栈为空时回到当前一级目录入口。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     menu_back();  // OK 双击时调用
 * 备注信息：返回操作不再压栈，避免来回跳转时栈无限增长。
 */
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

/*
 * 函数名称：menu_toggle_lap
 * 函数功能：在第一圈/第二圈显示状态之间切换。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     menu_toggle_lap();  // 长按 OK 时调用
 * 备注信息：只改变菜单显示和 Car_Menu_GetLap() 的返回值，不改变底盘
 *           执行状态；圈次执行仍由上层状态机负责。
 */
static void menu_toggle_lap(void)
{
    menu.lap = (menu.lap == CAR_MENU_LAP_FIRST)
             ? CAR_MENU_LAP_SECOND
             : CAR_MENU_LAP_FIRST;
    menu.dirty = 1U;
}

/*
 * 函数名称：menu_dispatch_key
 * 函数功能：把一个按键事件映射为菜单动作。
 * 参数说明：id    —— 按键编号（UP/DOWN/OK）；
 *           event —— 按键事件（单击/双击/长按/重复）。
 * 返回参数：无。
 * 使用示例：
 *     menu_dispatch_key(KEY_OK, KEY_EVENT_DOUBLE_CLICK);
 * 备注信息：导航约定见 Car_menu.h；PRESS/RELEASE 不产生菜单动作。
 */
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

/*
 * 函数名称：menu_handle_keys
 * 函数功能：依次取出三个按键的全部待处理事件并分发。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     menu_handle_keys();  // Car_Menu_Poll() 内部调用
 * 备注信息：使用 while 把队列排空，保证一次轮询处理完积压事件；
 *           事件队列为空后退出，不会阻塞。
 */
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

/*
 * 函数名称：Car_Menu_Init
 * 函数功能：初始化按键和 LCD，定位到主链入口 N01 并绘制首屏。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     Car_Menu_Init();
 * 备注信息：内部调用 Key_Init() 和 NEUQ_LCD200_init()，含 HAL_Delay()，
 *           只能在主循环启动前调用，不能放在中断里。
 */
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
    menu.link_fault = Car_Menu_ValidateLinks() ? 0U : 1U;

    Car_Menu_Render();
}

/*
 * 函数名称：Car_Menu_Poll
 * 函数功能：周期扫描按键、处理按键事件，并在需要时重绘页面。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     while (1)
 *     {
 *         Car_Menu_Poll();
 *         // 其他主循环任务
 *     }
 * 备注信息：建议每 5~10 ms 调用一次；不得在中断中调用，也不得与
 *           Key_Scan() 的其他调用者并发。
 */
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

/*
 * 函数名称：Car_Menu_Render
 * 函数功能：把当前节点的 ID、类型、屏蔽/圈次、得分、链接和状态绘制到 LCD。
 * 参数说明：无。
 * 返回参数：无。
 * 使用示例：
 *     Car_Menu_Render();
 * 备注信息：CAR_MENU_USE_LCD = 0 时为空操作；中文 name 不上屏，屏幕使用
 *           ASCII label 和枚举文本，避免 LCD200 字库缺字。
 */
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
    y += 18U;
    NEUQ_LCD200_SetPenColor(menu.link_fault ? LCD_RED : LCD_GREEN);
    snprintf(line, sizeof(line), "LINKCHECK: %s",
             menu.link_fault ? "ERR" : "OK");
    NEUQ_LCD200_DisplayString(4U, y, line);
    NEUQ_LCD200_SetPenColor(LCD_WHITE);
#else
    menu.dirty = 0;
#endif
}

/*
 * 函数名称：Car_Menu_Goto
 * 函数功能：跳转到指定索引的节点，并同步当前一级分类。
 * 参数说明：index —— 节点索引，必须大于 0 且小于节点总数。
 * 返回参数：1 成功；0 索引无效。
 * 使用示例：
 *     Car_Menu_Goto(P10);  // 跳到数独流程页
 * 备注信息：只改变菜单状态并置 dirty，不执行任何控制动作。
 */
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

/*
 * 函数名称：Car_Menu_GotoId
 * 函数功能：按节点 ID（如 "P10"）查找并跳转。
 * 参数说明：id —— 以 '\0' 结尾的节点 ID 字符串。
 * 返回参数：1 成功；0 参数为空或未找到。
 * 使用示例：
 *     Car_Menu_GotoId("N07");
 * 备注信息：线性查找 53 个节点，仅在初始化或调试跳转时使用。
 */
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

/*
 * 函数名称：Car_Menu_CurrentIndex
 * 函数功能：返回当前节点索引。
 * 参数说明：无。
 * 返回参数：当前节点索引。
 * 使用示例：
 *     int16_t index = Car_Menu_CurrentIndex();
 * 备注信息：索引 0 表示空；正常使用时始终指向有效节点。
 */
int16_t Car_Menu_CurrentIndex(void)
{
    return menu.current;
}

/*
 * 函数名称：Car_Menu_CurrentNode
 * 函数功能：返回当前节点的只读指针。
 * 参数说明：无。
 * 返回参数：当前节点指针；索引异常时返回入口节点 N01。
 * 使用示例：
 *     const CarMenuNode *node = Car_Menu_CurrentNode();
 * 备注信息：返回值指向只读常量表，调用者不得写入。
 */
const CarMenuNode *Car_Menu_CurrentNode(void)
{
    if (menu.current <= CAR_MENU_INDEX_NONE ||
        menu.current >= CAR_MENU_NODE_COUNT)
    {
        return &car_menu_nodes[N01];
    }

    return &car_menu_nodes[menu.current];
}

/*
 * 函数名称：Car_Menu_GetNode
 * 函数功能：按索引返回节点的只读指针。
 * 参数说明：index —— 节点索引。
 * 返回参数：节点指针；索引无效返回 NULL。
 * 使用示例：
 *     const CarMenuNode *node = Car_Menu_GetNode(P10);
 * 备注信息：返回值指向只读常量表，调用者不得写入。
 */
const CarMenuNode *Car_Menu_GetNode(int16_t index)
{
    if (index <= CAR_MENU_INDEX_NONE || index >= CAR_MENU_NODE_COUNT)
    {
        return 0;
    }

    return &car_menu_nodes[index];
}

/*
 * 函数名称：Car_Menu_NodeCount
 * 函数功能：返回节点总数（不含保留的空索引 0）。
 * 参数说明：无。
 * 返回参数：节点总数。
 * 使用示例：
 *     for (i = 1; i <= Car_Menu_NodeCount(); i++) { }
 * 备注信息：当前节点表为 53 个节点。
 */
int16_t Car_Menu_NodeCount(void)
{
    return (int16_t)(CAR_MENU_NODE_COUNT - 1);
}

/*
 * 函数名称：menu_valid_index
 * 函数功能：判断节点索引是否落在有效范围（1 ~ 节点总数-1）。
 * 参数说明：index —— 待检查的索引。
 * 返回参数：1 有效；0 无效。
 * 使用示例：
 *     if (menu_valid_index(node->next)) { }
 * 备注信息：索引 0 是保留的空索引，不算有效节点。
 */
static uint8_t menu_valid_index(int16_t index)
{
    return (index > CAR_MENU_INDEX_NONE && index < CAR_MENU_NODE_COUNT)
         ? 1U : 0U;
}

/*
 * 函数名称：menu_in_child_list
 * 函数功能：判断节点是否位于指定父节点的 child 子链表中。
 * 参数说明：parent —— 父节点索引；
 *           node   —— 待查找的节点索引。
 * 返回参数：1 在子链表中；0 不在或参数无效。
 * 使用示例：
 *     if (!menu_in_child_list(node->parent, i)) { error++; }
 * 备注信息：沿 child 入口的 next 指针遍历，带节点总数上限防止坏链死循环。
 */
static uint8_t menu_in_child_list(int16_t parent, int16_t node)
{
    int16_t current;
    uint16_t guard = 0U;

    if (!menu_valid_index(parent) || !menu_valid_index(node))
    {
        return 0U;
    }

    current = car_menu_nodes[parent].child;

    while (current != CAR_MENU_INDEX_NONE && guard < CAR_MENU_NODE_COUNT)
    {
        if (!menu_valid_index(current))
        {
            return 0U;
        }

        if (current == node)
        {
            return 1U;
        }

        current = car_menu_nodes[current].next;
        guard++;
    }

    return 0U;
}

/*
 * 函数名称：Car_Menu_ValidateLinks
 * 函数功能：检查静态节点表的建链是否闭合，结果写入 menu_link_errors。
 * 参数说明：无。
 * 返回参数：1 全部通过；0 存在建链错误。
 * 使用示例：
 *     if (!Car_Menu_ValidateLinks())
 *     {
 *         // 建链表有错，禁止进入比赛流程
 *     }
 * 备注信息：检查 next/prev 互逆、parent/child 包含关系、branch_to/
 *           merge_to/pid_link 目标、pid_page 唯一性和四个目录入口。
 *           只读遍历，不修改节点表；可在上电初始化和主机测试中调用。
 */
uint8_t Car_Menu_ValidateLinks(void)
{
    uint16_t errors = 0U;
    int16_t i;
    int16_t j;
    uint8_t category;
    uint8_t page;

    for (i = 1; i < CAR_MENU_NODE_COUNT; i++)
    {
        const CarMenuNode *node = &car_menu_nodes[i];

        if (node->id == 0 || node->name == 0 || node->label == 0)
        {
            errors++;
        }
        if ((unsigned)node->category >= CAR_MENU_CAT_COUNT)
        {
            errors++;
        }
        if ((unsigned)node->type > CAR_MENU_TYPE_FLOW)
        {
            errors++;
        }
        if ((unsigned)node->skippable > CAR_MENU_SKIP_PARTIAL)
        {
            errors++;
        }
        if ((unsigned)node->lap > CAR_MENU_LAP_BOTH)
        {
            errors++;
        }
        if ((unsigned)node->trigger > CAR_MENU_TRIG_MANUAL)
        {
            errors++;
        }
        if ((unsigned)node->status > CAR_MENU_STATUS_TBD)
        {
            errors++;
        }
        if ((unsigned)node->kalman > CAR_MENU_KALMAN_INDIRECT)
        {
            errors++;
        }
        if (node->pid_page > 13U)
        {
            errors++;
        }

        /* next/prev 必须互为逆；否则遍历会跳错页或跳过节点。 */
        if (node->next != CAR_MENU_INDEX_NONE &&
            (!menu_valid_index(node->next) ||
             car_menu_nodes[node->next].prev != i))
        {
            errors++;
        }
        if (node->prev != CAR_MENU_INDEX_NONE &&
            (!menu_valid_index(node->prev) ||
             car_menu_nodes[node->prev].next != i))
        {
            errors++;
        }

        /* 事件分支与跨目录链接的目标必须在节点表内。 */
        if (node->branch_to != CAR_MENU_INDEX_NONE &&
            !menu_valid_index(node->branch_to))
        {
            errors++;
        }
        if (node->merge_to != CAR_MENU_INDEX_NONE &&
            !menu_valid_index(node->merge_to))
        {
            errors++;
        }
        if (node->pid_link != CAR_MENU_INDEX_NONE &&
            !menu_valid_index(node->pid_link))
        {
            errors++;
        }

        /* child 是子链表入口；parent 必须能在该子链表中找到当前节点。 */
        if (node->child != CAR_MENU_INDEX_NONE &&
            (!menu_valid_index(node->child) ||
             car_menu_nodes[node->child].parent != i))
        {
            errors++;
        }
        if (node->parent != CAR_MENU_INDEX_NONE &&
            !menu_in_child_list(node->parent, i))
        {
            errors++;
        }

        /* 节点 ID 必须唯一，便于 Car_Menu_GotoId() 稳定跳转。 */
        if (node->id != 0)
        {
            for (j = (int16_t)(i + 1); j < CAR_MENU_NODE_COUNT; j++)
            {
                if (car_menu_nodes[j].id != 0 &&
                    strcmp(node->id, car_menu_nodes[j].id) == 0)
                {
                    errors++;
                }
            }
        }
    }

    /* P01~P13 的 pid_page 必须 1~13 各出现一次。 */
    for (page = 1U; page <= 13U; page++)
    {
        uint16_t matches = 0U;

        for (i = 1; i < CAR_MENU_NODE_COUNT; i++)
        {
            if (car_menu_nodes[i].pid_page == page)
            {
                matches++;
            }
        }

        if (matches != 1U)
        {
            errors++;
        }
    }

    /* 四个一级目录入口必须存在，且分类与数组顺序一致。 */
    for (category = 0U; category < CAR_MENU_CAT_COUNT; category++)
    {
        int16_t head = category_head[category];

        if (!menu_valid_index(head) ||
            (unsigned)car_menu_nodes[head].category != category)
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
 * 返回参数：错误数；0 表示建链通过。
 * 使用示例：
 *     uint16_t errors = Car_Menu_LinkErrorCount();
 * 备注信息：每次调用 Car_Menu_ValidateLinks() 都会重新计数。
 */
uint16_t Car_Menu_LinkErrorCount(void)
{
    return menu_link_errors;
}

/*
 * 函数名称：Car_Menu_SetLap
 * 函数功能：设置菜单显示的第一圈/第二圈状态。
 * 参数说明：lap —— CAR_MENU_LAP_FIRST / SECOND / BOTH。
 * 返回参数：无。
 * 使用示例：
 *     Car_Menu_SetLap(CAR_MENU_LAP_SECOND);
 * 备注信息：仅影响显示；圈次执行由上层状态机负责。参数越界时忽略。
 */
void Car_Menu_SetLap(CarMenuLap lap)
{
    if (lap > CAR_MENU_LAP_BOTH)
    {
        return;
    }

    menu.lap = lap;
    menu.dirty = 1U;
}

/*
 * 函数名称：Car_Menu_GetLap
 * 函数功能：读取菜单显示的第一圈/第二圈状态。
 * 参数说明：无。
 * 返回参数：当前显示圈次。
 * 使用示例：
 *     CarMenuLap lap = Car_Menu_GetLap();
 * 备注信息：该值不代表底盘实际圈次，只代表菜单状态。
 */
CarMenuLap Car_Menu_GetLap(void)
{
    return menu.lap;
}
