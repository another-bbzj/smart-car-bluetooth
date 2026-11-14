#ifndef CAR_MENU_H
#define CAR_MENU_H

#include <stdint.h>

#include "key.h"

/*
 * ============================================================================
 * 文件名称：Car_menu.h
 * ============================================================================
 * 模块功能：现场调试用链表式菜单 + LCD200 显示。
 *
 *   - MenuID 枚举决定节点编号，MenuConfig 只描述静态配置；
 *   - Car_Menu_BuildLink() 按 parent_id 建父子关系和环形同级双向链表；
 *   - page 是当前页面，selected 永远是 page 的子节点；
 *   - editing = 0 浏览，editing = 1 编辑；
 *   - 参数只允许在车辆 STOP 时修改，修改后立即写入控制量；
 *   - SAVE/LOAD/DEFAULTS 使用内部 Flash 扇区 11，不碰板载 W25Q。
 *
 *   显示布局（LCD200 横屏 320x240，ASCII_Font24，12x24 像素）：
 *     第 0 行：目录路径，固定
 *     第 1 行：状态/故障/模式/电压/提示，固定
 *     第 2~9 行：8 个列表项；选中项到达倒数第 3 行（第 7 行）后开始滚动。
 * ============================================================================
 */

/* 参数显示格式。 */
typedef enum
{
    CAR_MENU_FMT_INT = 0,   /* 整数 */
    CAR_MENU_FMT_X100,      /* 放大 100 倍，显示两位小数 */
    CAR_MENU_FMT_X10,       /* 放大 10 倍，显示一位小数 */
    CAR_MENU_FMT_MV         /* 毫伏整数 */
} CarMenuFormat;

/* 动作节点。动作只在长按 OK 时执行，避免误触。 */
typedef enum
{
    CAR_MENU_ACT_NONE = 0,
    CAR_MENU_ACT_STOP,      /* 停车/清故障 */
    CAR_MENU_ACT_SAVE,      /* 写内部 Flash */
    CAR_MENU_ACT_LOAD,      /* 从内部 Flash 读取并应用 */
    CAR_MENU_ACT_DEFAULTS   /* 恢复编译期默认值 */
} CarMenuAction;

/*
 * 菜单页面编号。
 * 同级节点的显示顺序由枚举顺序决定（建链时按 i 递增追加）。
 */
typedef enum
{
    MENU_NONE = -1,
    MAIN = 0,

    CAT_STATUS,                 /* 一级：状态 */
    CAT_TUNE,                   /* 一级：调参 */
    CAT_ACTIONS,                /* 一级：动作 */
    CAT_INFO,                   /* 一级：信息 */

    ST_STATE, ST_FAULT, ST_MODE,
    ST_SPD_L, ST_SPD_R, ST_REF_L, ST_REF_R, ST_OUT_L, ST_OUT_R,
    ST_VISION, ST_BATT,

    TU_PID_L, TU_PID_R, TU_LIMITS, TU_ENC, TU_BATT,

    PL_KP, PL_KI, PL_KD,
    PR_KP, PR_KI, PR_KD,

    LM_RPM, LM_PWM, LM_ACC, LM_JERK,
    EN_CPR,
    BT_WARN, BT_CUT,

    AC_STOP, AC_SAVE, AC_LOAD, AC_DEF,

    IN_FW, IN_NODES, IN_LINK, IN_FLASH,

    MENU_COUNT
} MenuID;

/* 菜单参数，全部为整数，写入控制量时再换算。 */
typedef struct
{
    int32_t kp_x100[2];     /* 左/右 Kp × 100 */
    int32_t ki_x100[2];
    int32_t kd_x100[2];
    int32_t rpm_limit;      /* rpm */
    int32_t pwm_limit;      /* PWM 数值 */
    int32_t accel_limit;    /* rpm/s */
    int32_t jerk_limit;     /* rpm/s² */
    int32_t cpr_x10;        /* 每圈计数 × 10 */
    int32_t batt_warn_mv;
    int32_t batt_cut_mv;
} CarMenuParams;

extern CarMenuParams car_menu_params;

/* 页面/节点的静态配置。 */
typedef struct
{
    const char *name;       /* ASCII 显示名，建议不超过 12 字符 */
    MenuID parent_id;       /* 父节点；MENU_NONE 表示根 */
    int32_t *value;         /* 可编辑参数地址；NULL 表示页面/只读/动作 */
    int32_t min;
    int32_t max;
    int32_t step;
    CarMenuFormat format;
    CarMenuAction action;
} MenuConfig;

/**
 * @brief 初始化参数、建链、自检、按键和 LCD200，并尝试从 Flash 加载参数。
 * @note 必须在 PIT 启动前调用；LCD 初始化含 120 ms 延时，不进入控制中断。
 */
void Car_Menu_Init(void);

/**
 * @brief 主循环周期任务：扫描按键、处理事件、按需刷新 LCD。
 * @note 建议每 5~10 ms 调用；显示刷新只在主循环，不在 PIT 中断。
 */
void Car_Menu_Poll(void);

/**
 * @brief 注入一个按键事件，推进浏览/编辑状态机。
 * @return 1 状态改变；0 忽略。
 */
uint8_t Car_Menu_Update(Key_Id key, Key_Event event);

/**
 * @brief 显示入口；由 Car_Menu_Poll 按需调用。
 */
void Car_Menu_Render(void);

/**
 * @brief 按 MenuTable[i].parent_id 构建运行时链表。
 */
uint8_t Car_Menu_BuildLink(void);

/**
 * @brief 检查建链闭合、参数范围和动作节点合法性。
 */
uint8_t Car_Menu_ValidateLinks(void);

/**
 * @brief 最近一次建链自检的错误数。
 */
uint16_t Car_Menu_LinkErrorCount(void);

/**
 * @brief 跳转：叶子进入父页面并选中自己，页面进入自身。
 */
uint8_t Car_Menu_Goto(MenuID id);

MenuID Car_Menu_GetPageId(void);
MenuID Car_Menu_GetSelectedId(void);
uint8_t Car_Menu_IsEditing(void);
int32_t Car_Menu_GetEditValue(void);
void Car_Menu_SetEditValue(int32_t value);
const MenuConfig *Car_Menu_GetConfig(MenuID id);
int16_t Car_Menu_NodeCount(void);

/**
 * @brief 当前列表滚动偏移，供主机测试/诊断。
 */
int16_t Car_Menu_GetScrollOffset(void);

/**
 * @brief 把当前菜单参数写入控制量与 ADC 阈值（仅 STOP）。
 * @return 1 成功；0 非 STOP 或参数非法。
 */
uint8_t Car_Menu_ApplyParams(void);

/**
 * @brief 保存当前参数到内部 Flash（内部检查 STOP）。
 * @return 1 成功；0 非 STOP、校验失败或擦写失败。
 */
uint8_t Car_Menu_SaveParams(void);

/**
 * @brief 从内部 Flash 读取并应用参数。
 * @return 1 成功；0 无有效数据、非 STOP 或应用失败。
 */
uint8_t Car_Menu_LoadParams(void);

#endif
