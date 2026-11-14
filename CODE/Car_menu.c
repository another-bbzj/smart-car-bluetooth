/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */

/*
 * ============================================================================
 * 文件名称：Car_menu.c
 * ============================================================================
 * 模块功能：现场调试链式菜单、按键浏览/编辑、LCD200 显示和 Flash 持久化。
 *
 *   显示：LCD200 横屏 320x240，ASCII_Font24（12x24），共 10 行。
 *     行 0：目录路径（固定）
 *     行 1：状态/故障/模式/电压/提示（固定）
 *     行 2~9：列表 8 行；选中项到倒数第 3 行（行 7）后开始滚动。
 *
 *   安全：参数只在 STOP 时应用；动作节点必须长按 OK；Flash 擦写前后
 *         由 Car_Debug_FlashBegin/End 暂停 PIT 并重建控制时基。
 * ============================================================================
 */

#include "NEUQ_lib_include.h"
#include "Car_Config.h"
#include "Car_Core.h"
#include "Car_Hardware.h"
#include "Car_adc.h"
#include "Car_params.h"
#include "Car_menu.h"
#include "key.h"

#define MENU_TEXT_LEN       26U
#define MENU_ROW_PX         24U
#define MENU_SCREEN_W       320U
#define MENU_SCREEN_H       240U
#define MENU_LIST_ROWS      8U
#define MENU_LIST_FIRST_ROW 2U
#define MENU_SCROLL_ROW     5U   /* 列表第 6 行 = 屏幕第 8 行 = 倒数第 3 行 */

typedef struct MenuNode
{
    MenuID id;
    const MenuConfig *info;
    struct MenuNode *parent;
    struct MenuNode *child;
    struct MenuNode *prev;
    struct MenuNode *next;
} MenuNode;

CarMenuParams car_menu_params;

#define MENU_ITEM(name_, parent_, value_, min_, max_, step_, fmt_, act_) \
    { (name_), (parent_), (value_), (min_), (max_), (step_), (fmt_), (act_) }
#define MENU_PAGE(name_, parent_) \
    MENU_ITEM((name_), (parent_), NULL, 0, 0, 0, CAR_MENU_FMT_INT, CAR_MENU_ACT_NONE)
#define MENU_PARAM(name_, parent_, ptr_, min_, max_, step_, fmt_) \
    MENU_ITEM((name_), (parent_), (ptr_), (min_), (max_), (step_), (fmt_), CAR_MENU_ACT_NONE)
#define MENU_ACTION(name_, parent_, act_) \
    MENU_ITEM((name_), (parent_), NULL, 0, 0, 0, CAR_MENU_FMT_INT, (act_))

/*
 * 静态配置表。parent_id 决定层级，枚举顺序决定同级显示顺序。
 * 参数地址指向 car_menu_params；修改后由 Car_Menu_ApplyParams 写入控制量。
 */
static const MenuConfig MenuTable[MENU_COUNT] =
{
    [MAIN] = MENU_PAGE("MAIN", MENU_NONE),

    [CAT_STATUS] = MENU_PAGE("STATUS", MAIN),
    [CAT_TUNE] = MENU_PAGE("TUNE", MAIN),
    [CAT_ACTIONS] = MENU_PAGE("ACTIONS", MAIN),
    [CAT_INFO] = MENU_PAGE("INFO", MAIN),

    [ST_STATE] = MENU_PAGE("STATE", CAT_STATUS),
    [ST_FAULT] = MENU_PAGE("FAULT", CAT_STATUS),
    [ST_MODE] = MENU_PAGE("MODE", CAT_STATUS),
    [ST_SPD_L] = MENU_PAGE("L SPEED", CAT_STATUS),
    [ST_SPD_R] = MENU_PAGE("R SPEED", CAT_STATUS),
    [ST_REF_L] = MENU_PAGE("L REF", CAT_STATUS),
    [ST_REF_R] = MENU_PAGE("R REF", CAT_STATUS),
    [ST_OUT_L] = MENU_PAGE("L OUT", CAT_STATUS),
    [ST_OUT_R] = MENU_PAGE("R OUT", CAT_STATUS),
    [ST_VISION] = MENU_PAGE("VISION", CAT_STATUS),
    [ST_BATT] = MENU_PAGE("BATTERY", CAT_STATUS),

    [TU_PID_L] = MENU_PAGE("PID LEFT", CAT_TUNE),
    [TU_PID_R] = MENU_PAGE("PID RIGHT", CAT_TUNE),
    [TU_LIMITS] = MENU_PAGE("LIMITS", CAT_TUNE),
    [TU_ENC] = MENU_PAGE("ENCODER", CAT_TUNE),
    [TU_BATT] = MENU_PAGE("BAT THRESH", CAT_TUNE),

    [PL_KP] = MENU_PARAM("L KP", TU_PID_L, &car_menu_params.kp_x100[0],
                         0, 65535, 1, CAR_MENU_FMT_X100),
    [PL_KI] = MENU_PARAM("L KI", TU_PID_L, &car_menu_params.ki_x100[0],
                         0, 65535, 1, CAR_MENU_FMT_X100),
    [PL_KD] = MENU_PARAM("L KD", TU_PID_L, &car_menu_params.kd_x100[0],
                         0, 65535, 1, CAR_MENU_FMT_X100),
    [PR_KP] = MENU_PARAM("R KP", TU_PID_R, &car_menu_params.kp_x100[1],
                         0, 65535, 1, CAR_MENU_FMT_X100),
    [PR_KI] = MENU_PARAM("R KI", TU_PID_R, &car_menu_params.ki_x100[1],
                         0, 65535, 1, CAR_MENU_FMT_X100),
    [PR_KD] = MENU_PARAM("R KD", TU_PID_R, &car_menu_params.kd_x100[1],
                         0, 65535, 1, CAR_MENU_FMT_X100),

    [LM_RPM] = MENU_PARAM("RPM LIMIT", TU_LIMITS, &car_menu_params.rpm_limit,
                          10, 300, 5, CAR_MENU_FMT_INT),
    [LM_PWM] = MENU_PARAM("PWM LIMIT", TU_LIMITS, &car_menu_params.pwm_limit,
                          100, 10000, 50, CAR_MENU_FMT_INT),
    [LM_ACC] = MENU_PARAM("ACCEL", TU_LIMITS, &car_menu_params.accel_limit,
                          10, 5000, 10, CAR_MENU_FMT_INT),
    [LM_JERK] = MENU_PARAM("JERK", TU_LIMITS, &car_menu_params.jerk_limit,
                           10, 20000, 50, CAR_MENU_FMT_INT),

    [EN_CPR] = MENU_PARAM("CPR X10", TU_ENC, &car_menu_params.cpr_x10,
                          100, 1000000, 10, CAR_MENU_FMT_X10),

    [BT_WARN] = MENU_PARAM("WARN MV", TU_BATT, &car_menu_params.batt_warn_mv,
                           1000, 30000, 100, CAR_MENU_FMT_MV),
    [BT_CUT] = MENU_PARAM("CUT MV", TU_BATT, &car_menu_params.batt_cut_mv,
                          1000, 30000, 100, CAR_MENU_FMT_MV),

    [AC_STOP] = MENU_ACTION("STOP/CLEAR", CAT_ACTIONS, CAR_MENU_ACT_STOP),
    [AC_SAVE] = MENU_ACTION("SAVE FLASH", CAT_ACTIONS, CAR_MENU_ACT_SAVE),
    [AC_LOAD] = MENU_ACTION("LOAD FLASH", CAT_ACTIONS, CAR_MENU_ACT_LOAD),
    [AC_DEF] = MENU_ACTION("DEFAULTS", CAT_ACTIONS, CAR_MENU_ACT_DEFAULTS),

    [IN_FW] = MENU_PAGE("FW VERSION", CAT_INFO),
    [IN_NODES] = MENU_PAGE("NODE COUNT", CAT_INFO),
    [IN_LINK] = MENU_PAGE("LINK ERR", CAT_INFO),
    [IN_FLASH] = MENU_PAGE("FLASH", CAT_INFO)
};

static MenuNode nodes[MENU_COUNT];
static MenuNode *page;
static MenuNode *selected;
static uint8_t editing;
static int32_t edit_value;
static int16_t scroll_offset;
static uint16_t link_errors;
static uint8_t menu_dirty;
static uint8_t flash_present;
static uint32_t message_until;
static char menu_message[MENU_TEXT_LEN + 1U];
static char shown[10][MENU_TEXT_LEN + 1U];

/* ========================================================================== */
/*                              内部工具                                       */
/* ========================================================================== */

static void menu_copy(char *dst, const char *src, unsigned n)
{
    unsigned i = 0U;

    if (n == 0U)
    {
        return;
    }

    while (i + 1U < n && src[i] != '\0')
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void menu_message_set(const char *text)
{
    menu_copy(menu_message, text, sizeof(menu_message));
    message_until = HAL_GetTick() + 1500U;
    menu_dirty = 1U;
}

static uint8_t menu_message_active(void)
{
    return ((int32_t)(HAL_GetTick() - message_until) < 0) ? 1U : 0U;
}

static int32_t menu_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static void menu_format_value(const MenuConfig *cfg, int32_t value,
                              char *out, unsigned n)
{
    int32_t whole;
    int32_t frac;

    switch (cfg->format)
    {
        case CAR_MENU_FMT_X100:
            whole = value / 100;
            frac = menu_abs32(value % 100);
            (void)snprintf(out, n, "%d.%02d", (int)whole, (int)frac);
            break;

        case CAR_MENU_FMT_X10:
            whole = value / 10;
            frac = menu_abs32(value % 10);
            (void)snprintf(out, n, "%d.%d", (int)whole, (int)frac);
            break;

        default:
            (void)snprintf(out, n, "%d", (int)value);
            break;
    }
}

static const char *menu_state_name(uint8_t state)
{
    switch (state)
    {
        case CAR_RUN: return "RUN";
        case CAR_FAULT: return "FAULT";
        default: return "STOP";
    }
}

static const char *menu_mode_name(uint8_t mode)
{
    return (mode == CAR_MODE_AUTO) ? "AUTO" : "MANUAL";
}

static const char *menu_fault_name(uint8_t fault)
{
    switch (fault)
    {
        case 0: return "NONE";
        case 1: return "LINK";
        case 2: return "STALL";
        case 3: return "DIR";
        case 4: return "RX";
        case 5: return "TIMING";
        case 6: return "CTRL";
        case 7: return "VISION";
        default: return "?";
    }
}

static void menu_status_value(MenuID id, const CarDebugSnapshot *s,
                              char *out, unsigned n)
{
    uint16_t mv;
    CarAdcState adc_state;

    switch (id)
    {
        case ST_STATE:
            menu_copy(out, menu_state_name(s->state), n);
            break;
        case ST_FAULT:
            (void)snprintf(out, n, "%u %s", (unsigned)s->fault,
                           menu_fault_name(s->fault));
            break;
        case ST_MODE:
            menu_copy(out, menu_mode_name(s->mode), n);
            break;
        case ST_SPD_L:
            (void)snprintf(out, n, "%d.%d RPM", (int)(s->speed[0]),
                           (int)(menu_abs32((int32_t)(s->speed[0] * 10.0f)) % 10));
            break;
        case ST_SPD_R:
            (void)snprintf(out, n, "%d.%d RPM", (int)(s->speed[1]),
                           (int)(menu_abs32((int32_t)(s->speed[1] * 10.0f)) % 10));
            break;
        case ST_REF_L:
            (void)snprintf(out, n, "%d.%d RPM", (int)(s->reference[0]),
                           (int)(menu_abs32((int32_t)(s->reference[0] * 10.0f)) % 10));
            break;
        case ST_REF_R:
            (void)snprintf(out, n, "%d.%d RPM", (int)(s->reference[1]),
                           (int)(menu_abs32((int32_t)(s->reference[1] * 10.0f)) % 10));
            break;
        case ST_OUT_L:
            (void)snprintf(out, n, "%d.%d %%", (int)(s->output[0] / 100.0f),
                           (int)(menu_abs32((int32_t)(s->output[0] / 10.0f)) % 10));
            break;
        case ST_OUT_R:
            (void)snprintf(out, n, "%d.%d %%", (int)(s->output[1] / 100.0f),
                           (int)(menu_abs32((int32_t)(s->output[1] / 10.0f)) % 10));
            break;
        case ST_VISION:
            (void)snprintf(out, n, "%s %uMS Q%u",
                           s->vision_present ? "ON" : "OFF",
                           (unsigned)s->vision_age_ms,
                           (unsigned)s->vision_sequence);
            break;
        case ST_BATT:
            mv = Car_Adc_GetFilteredMv();
            adc_state = Car_Adc_GetState();
            (void)snprintf(out, n, "%u.%uV %s", (unsigned)(mv / 1000U),
                           (unsigned)((mv % 1000U) / 100U),
                           (adc_state == CAR_ADC_STATE_CUT) ? "CUT" :
                           (adc_state == CAR_ADC_STATE_WARN) ? "WARN" : "OK");
            break;
        default:
            out[0] = '\0';
            break;
    }
}

static void menu_info_value(MenuID id, char *out, unsigned n)
{
    switch (id)
    {
        case IN_FW:
            menu_copy(out, "MENU 1.0", n);
            break;
        case IN_NODES:
            (void)snprintf(out, n, "%u", (unsigned)MENU_COUNT);
            break;
        case IN_LINK:
            (void)snprintf(out, n, "%u", (unsigned)link_errors);
            break;
        case IN_FLASH:
            menu_copy(out, flash_present ? "SAVED" : "EMPTY", n);
            break;
        default:
            out[0] = '\0';
            break;
    }
}

/* ========================================================================== */
/*                              建链与自检                                     */
/* ========================================================================== */

uint8_t Car_Menu_BuildLink(void)
{
    int i;

    for (i = 0; i < (int)MENU_COUNT; i++)
    {
        nodes[i].id = (MenuID)i;
        nodes[i].info = &MenuTable[i];
        nodes[i].parent = NULL;
        nodes[i].child = NULL;
        nodes[i].prev = &nodes[i];
        nodes[i].next = &nodes[i];
    }

    for (i = 1; i < (int)MENU_COUNT; i++)
    {
        MenuID parent_id = MenuTable[i].parent_id;
        MenuNode *node = &nodes[i];
        MenuNode *parent;
        MenuNode *last;

        if (parent_id <= MENU_NONE || parent_id >= MENU_COUNT ||
            parent_id == (MenuID)i)
        {
            return 0U;
        }

        parent = &nodes[parent_id];
        node->parent = parent;

        if (parent->child == NULL)
        {
            parent->child = node;
            node->prev = node;
            node->next = node;
        }
        else
        {
            last = parent->child->prev;
            last->next = node;
            node->prev = last;
            node->next = parent->child;
            parent->child->prev = node;
        }
    }

    return 1U;
}

static uint8_t menu_node_reaches_main(const MenuNode *node)
{
    unsigned depth = 0U;

    while (node != NULL && node->id != MAIN && depth < MENU_COUNT)
    {
        node = node->parent;
        depth++;
    }

    return (node != NULL && node->id == MAIN) ? 1U : 0U;
}

uint8_t Car_Menu_ValidateLinks(void)
{
    unsigned i;
    unsigned errors = 0U;

    if (nodes[MAIN].parent != NULL)
    {
        errors++;
    }

    for (i = 1U; i < MENU_COUNT; i++)
    {
        const MenuConfig *cfg = nodes[i].info;

        if (nodes[i].parent == NULL || nodes[i].parent->id != cfg->parent_id)
        {
            errors++;
        }
        if (nodes[i].next == NULL || nodes[i].prev == NULL ||
            nodes[i].next->prev != &nodes[i] ||
            nodes[i].prev->next != &nodes[i])
        {
            errors++;
        }
        if (!menu_node_reaches_main(&nodes[i]))
        {
            errors++;
        }
        if (cfg->value != NULL)
        {
            if (cfg->min > cfg->max || cfg->step <= 0 ||
                cfg->action != CAR_MENU_ACT_NONE)
            {
                errors++;
            }
        }
        if (cfg->action != CAR_MENU_ACT_NONE && cfg->value != NULL)
        {
            errors++;
        }
    }

    link_errors = (uint16_t)errors;
    return (errors == 0U) ? 1U : 0U;
}

uint16_t Car_Menu_LinkErrorCount(void)
{
    return link_errors;
}

/* ========================================================================== */
/*                              参数应用                                       */
/* ========================================================================== */

static void menu_params_default(void)
{
    unsigned i;

    for (i = 0U; i < 2U; i++)
    {
        car_menu_params.kp_x100[i] = (int32_t)(CAR_DEFAULT_KP * 100.0f + 0.5f);
        car_menu_params.ki_x100[i] = (int32_t)(CAR_DEFAULT_KI * 100.0f + 0.5f);
        car_menu_params.kd_x100[i] = (int32_t)(CAR_DEFAULT_KD * 100.0f + 0.5f);
    }

    car_menu_params.rpm_limit = (int32_t)CAR_RPM_LIMIT;
    car_menu_params.pwm_limit = (int32_t)CAR_PWM_LIMIT;
    car_menu_params.accel_limit = (int32_t)CAR_ACCEL_LIMIT_RPM_S;
    car_menu_params.jerk_limit = (int32_t)CAR_JERK_LIMIT_RPM_S2;
    car_menu_params.cpr_x10 = (int32_t)(CAR_ENCODER_CPR * 10.0f + 0.5f);
    car_menu_params.batt_warn_mv = (int32_t)CAR_ADC_LOW_WARN_MV;
    car_menu_params.batt_cut_mv = (int32_t)CAR_ADC_LOW_CUT_MV;
}

uint8_t Car_Menu_ApplyParams(void)
{
    unsigned i;

    if (!Car_Debug_SetLimits((float)car_menu_params.rpm_limit,
                             (float)car_menu_params.pwm_limit,
                             (float)car_menu_params.accel_limit,
                             (float)car_menu_params.jerk_limit))
    {
        return 0U;
    }

    for (i = 0U; i < 2U; i++)
    {
        if (!Car_Debug_SetPid((uint8_t)i,
                              (float)car_menu_params.kp_x100[i] / 100.0f,
                              (float)car_menu_params.ki_x100[i] / 100.0f,
                              (float)car_menu_params.kd_x100[i] / 100.0f))
        {
            return 0U;
        }
    }

    if (!Car_Debug_SetCpr((float)car_menu_params.cpr_x10 / 10.0f))
    {
        return 0U;
    }

    if (!Car_Adc_SetThresholds((uint16_t)car_menu_params.batt_warn_mv,
                               (uint16_t)car_menu_params.batt_cut_mv))
    {
        return 0U;
    }

    return 1U;
}

static void menu_params_from_flash(const CarParams *p)
{
    unsigned i;

    for (i = 0U; i < 2U; i++)
    {
        car_menu_params.kp_x100[i] = p->kp_x100[i];
        car_menu_params.ki_x100[i] = p->ki_x100[i];
        car_menu_params.kd_x100[i] = p->kd_x100[i];
    }

    car_menu_params.rpm_limit = p->rpm_limit;
    car_menu_params.pwm_limit = p->pwm_limit;
    car_menu_params.accel_limit = p->accel_limit;
    car_menu_params.jerk_limit = p->jerk_limit;
    car_menu_params.cpr_x10 = p->cpr_x10;
    car_menu_params.batt_warn_mv = p->batt_warn_mv;
    car_menu_params.batt_cut_mv = p->batt_cut_mv;
}

static void menu_params_to_flash(CarParams *p)
{
    unsigned i;

    Car_Params_Default(p);

    for (i = 0U; i < 2U; i++)
    {
        p->kp_x100[i] = car_menu_params.kp_x100[i];
        p->ki_x100[i] = car_menu_params.ki_x100[i];
        p->kd_x100[i] = car_menu_params.kd_x100[i];
    }

    p->rpm_limit = car_menu_params.rpm_limit;
    p->pwm_limit = car_menu_params.pwm_limit;
    p->accel_limit = car_menu_params.accel_limit;
    p->jerk_limit = car_menu_params.jerk_limit;
    p->cpr_x10 = car_menu_params.cpr_x10;
    p->batt_warn_mv = car_menu_params.batt_warn_mv;
    p->batt_cut_mv = car_menu_params.batt_cut_mv;
}

uint8_t Car_Menu_SaveParams(void)
{
    CarParams params;
    uint8_t ok;

    menu_params_to_flash(&params);

    if (!Car_Debug_FlashBegin())
    {
        return 0U;
    }

    ok = Car_Params_Save(&params);
    Car_Debug_FlashEnd();

    flash_present = Car_Params_IsPresent();
    return ok;
}

uint8_t Car_Menu_LoadParams(void)
{
    CarParams params;
    CarMenuParams saved = car_menu_params;

    if (!Car_Params_Load(&params))
    {
        flash_present = 0U;
        return 0U;
    }

    menu_params_from_flash(&params);
    if (!Car_Menu_ApplyParams())
    {
        car_menu_params = saved;
        (void)Car_Menu_ApplyParams();
        return 0U;
    }

    flash_present = 1U;
    return 1U;
}

/* ========================================================================== */
/*                              浏览/编辑状态机                                 */
/* ========================================================================== */

static int16_t menu_child_count(const MenuNode *parent)
{
    int16_t count = 0;
    MenuNode *n;

    if (parent == NULL || parent->child == NULL)
    {
        return 0;
    }

    n = parent->child;
    do
    {
        count++;
        n = n->next;
    } while (n != parent->child && count < (int16_t)MENU_COUNT);

    return count;
}

static int16_t menu_child_index(const MenuNode *parent, const MenuNode *node)
{
    int16_t index = 0;
    MenuNode *n;

    if (parent == NULL || parent->child == NULL || node == NULL)
    {
        return 0;
    }

    n = parent->child;
    do
    {
        if (n == node)
        {
            return index;
        }
        index++;
        n = n->next;
    } while (n != parent->child && index < (int16_t)MENU_COUNT);

    return 0;
}

static MenuNode *menu_child_at(const MenuNode *parent, int16_t index)
{
    int16_t i = 0;
    MenuNode *n;

    if (parent == NULL || parent->child == NULL || index < 0)
    {
        return NULL;
    }

    n = parent->child;
    do
    {
        if (i == index)
        {
            return n;
        }
        i++;
        n = n->next;
    } while (n != parent->child && i < (int16_t)MENU_COUNT);

    return NULL;
}

static void menu_scroll_adjust(void)
{
    int16_t index = menu_child_index(page, selected);
    int16_t count = menu_child_count(page);
    int16_t max_offset = (count > (int16_t)MENU_LIST_ROWS)
                       ? (int16_t)(count - (int16_t)MENU_LIST_ROWS) : 0;

    if (index < scroll_offset)
    {
        scroll_offset = index;
    }
    if (index > (int16_t)(scroll_offset + (int16_t)MENU_SCROLL_ROW))
    {
        scroll_offset = (int16_t)(index - (int16_t)MENU_SCROLL_ROW);
    }
    if (scroll_offset < 0)
    {
        scroll_offset = 0;
    }
    if (scroll_offset > max_offset)
    {
        scroll_offset = max_offset;
    }
}

static uint8_t menu_can_edit(void)
{
    CarDebugSnapshot snapshot;

    if (!Car_Debug_GetSnapshot(&snapshot))
    {
        return 0U;
    }

    return (snapshot.state == CAR_STOP) ? 1U : 0U;
}

static void menu_action_run(CarMenuAction action)
{
    switch (action)
    {
        case CAR_MENU_ACT_STOP:
            Car_Debug_Stop();
            menu_message_set("STOPPED");
            break;

        case CAR_MENU_ACT_SAVE:
            menu_message_set(Car_Menu_SaveParams() ? "SAVED" : "SAVE FAIL");
            break;

        case CAR_MENU_ACT_LOAD:
            menu_message_set(Car_Menu_LoadParams() ? "LOADED" : "LOAD FAIL");
            break;

        case CAR_MENU_ACT_DEFAULTS:
            menu_params_default();
            menu_message_set(Car_Menu_ApplyParams() ? "DEFAULTS" : "STOP FIRST");
            flash_present = Car_Params_IsPresent();
            break;

        default:
            break;
    }
}

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
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT ||
                    event == KEY_EVENT_LONG_PRESS)
                {
                    next = edit_value + item->step;
                    if (next > item->max)
                    {
                        next = item->max;
                    }
                    if (next != edit_value)
                    {
                        edit_value = next;
                        menu_dirty = 1U;
                        return 1U;
                    }
                }
                break;

            case KEY_DOWN:
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT ||
                    event == KEY_EVENT_LONG_PRESS)
                {
                    next = edit_value - item->step;
                    if (next < item->min)
                    {
                        next = item->min;
                    }
                    if (next != edit_value)
                    {
                        edit_value = next;
                        menu_dirty = 1U;
                        return 1U;
                    }
                }
                break;

            case KEY_OK:
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_LONG_PRESS)
                {
                    int32_t old = *item->value;

                    *item->value = edit_value;
                    editing = 0U;
                    if (!Car_Menu_ApplyParams())
                    {
                        *item->value = old;
                        (void)Car_Menu_ApplyParams();
                        menu_message_set("STOP/PARAM FAIL");
                    }
                    else
                    {
                        menu_message_set("APPLIED");
                    }
                    menu_dirty = 1U;
                    return 1U;
                }
                if (event == KEY_EVENT_DOUBLE_CLICK)
                {
                    editing = 0U;
                    menu_dirty = 1U;
                    return 1U;
                }
                break;

            case KEY_BACK:
                if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_DOUBLE_CLICK ||
                    event == KEY_EVENT_LONG_PRESS)
                {
                    editing = 0U;
                    menu_dirty = 1U;
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
            if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT ||
                event == KEY_EVENT_LONG_PRESS)
            {
                selected = selected->prev;
                menu_scroll_adjust();
                menu_dirty = 1U;
                return 1U;
            }
            break;

        case KEY_DOWN:
            if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_REPEAT ||
                event == KEY_EVENT_LONG_PRESS)
            {
                selected = selected->next;
                menu_scroll_adjust();
                menu_dirty = 1U;
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
                    scroll_offset = 0;
                    menu_dirty = 1U;
                    return 1U;
                }

                if (selected->info != NULL &&
                    selected->info->action != CAR_MENU_ACT_NONE)
                {
                    menu_message_set("HOLD OK");
                    return 1U;
                }

                if (selected->info != NULL && selected->info->value != NULL)
                {
                    if (!menu_can_edit())
                    {
                        menu_message_set("STOP FIRST");
                        return 1U;
                    }
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
                    menu_dirty = 1U;
                    return 1U;
                }
                return 0U;
            }

            if (event == KEY_EVENT_LONG_PRESS)
            {
                if (selected->info != NULL &&
                    selected->info->action != CAR_MENU_ACT_NONE)
                {
                    menu_action_run(selected->info->action);
                    menu_dirty = 1U;
                    return 1U;
                }

                page = &nodes[MAIN];
                selected = page->child;
                scroll_offset = 0;
                editing = 0U;
                menu_dirty = 1U;
                return 1U;
            }

            if (event == KEY_EVENT_DOUBLE_CLICK)
            {
                if (page->parent != NULL)
                {
                    selected = page;
                    page = page->parent;
                    menu_scroll_adjust();
                    menu_dirty = 1U;
                    return 1U;
                }
                return 0U;
            }
            break;

        case KEY_BACK:
            if (event == KEY_EVENT_SINGLE_CLICK || event == KEY_EVENT_DOUBLE_CLICK)
            {
                if (page->parent != NULL)
                {
                    selected = page;
                    page = page->parent;
                    menu_scroll_adjust();
                    menu_dirty = 1U;
                    return 1U;
                }
                return 0U;
            }

            if (event == KEY_EVENT_LONG_PRESS)
            {
                page = &nodes[MAIN];
                selected = page->child;
                scroll_offset = 0;
                editing = 0U;
                menu_dirty = 1U;
                return 1U;
            }
            break;

        default:
            break;
    }

    return 0U;
}

/* ========================================================================== */
/*                              显示                                           */
/* ========================================================================== */

static void menu_draw_row(unsigned row, const char *text, uint8_t highlight)
{
    if (row >= 10U || text == NULL)
    {
        return;
    }

    if (strcmp(shown[row], text) == 0)
    {
        return;
    }

    NEUQ_LCD200_SetBackColor(highlight ? LCD_WHITE : LCD_BLACK);
    NEUQ_LCD200_SetPenColor(highlight ? LCD_BLACK : LCD_WHITE);
    NEUQ_LCD200_ClearRect(0, (uint16_t)(row * MENU_ROW_PX),
                          MENU_SCREEN_W, MENU_ROW_PX);
    NEUQ_LCD200_DisplayString(0, (uint16_t)(row * MENU_ROW_PX),
                              (char *)text);
    menu_copy(shown[row], text, MENU_TEXT_LEN + 1U);
}

static void menu_invalidate_display(void)
{
    memset(shown, 0, sizeof(shown));
}

static void menu_append(char *out, unsigned n, const char *src)
{
    unsigned used = (unsigned)strlen(out);

    if (used + 1U >= n)
    {
        return;
    }

    menu_copy(out + used, src, n - used);
}

static void menu_build_path(char *out, unsigned n)
{
    const MenuNode *stack[10];
    unsigned depth = 0U;
    unsigned i;
    const MenuNode *node = page;

    while (node != NULL && depth < 10U)
    {
        stack[depth++] = node;
        node = node->parent;
    }

    out[0] = '\0';
    for (i = depth; i > 0U; i--)
    {
        if (out[0] != '\0')
        {
            menu_append(out, n, ">");
        }
        menu_append(out, n, stack[i - 1U]->info->name);
    }
}

static void menu_build_status(char *out, unsigned n, const CarDebugSnapshot *s)
{
    uint16_t mv = Car_Adc_GetFilteredMv();

    if (menu_message_active())
    {
        menu_copy(out, menu_message, n);
        return;
    }

    (void)snprintf(out, n, "%s F%u %s %u.%uV",
                   menu_state_name(s->state), (unsigned)s->fault,
                   menu_mode_name(s->mode),
                   (unsigned)(mv / 1000U), (unsigned)((mv % 1000U) / 100U));
}

static void menu_build_body(const MenuNode *node, const CarDebugSnapshot *s,
                            char *out, unsigned n)
{
    const MenuConfig *cfg = node->info;
    char value[MENU_TEXT_LEN + 1U];

    if (cfg == NULL)
    {
        out[0] = '\0';
        return;
    }

    value[0] = '\0';

    if (cfg->value != NULL)
    {
        int32_t v = (editing && node == selected) ? edit_value : *cfg->value;
        menu_format_value(cfg, v, value, sizeof(value));
        if (editing && node == selected)
        {
            (void)snprintf(out, n, "%s: [%s]", cfg->name, value);
        }
        else
        {
            (void)snprintf(out, n, "%s: %s", cfg->name, value);
        }
    }
    else if (cfg->action != CAR_MENU_ACT_NONE)
    {
        (void)snprintf(out, n, "%s [HOLD OK]", cfg->name);
    }
    else if (node->child != NULL)
    {
        menu_copy(out, cfg->name, n);
    }
    else if (node->id >= ST_STATE && node->id <= ST_BATT)
    {
        menu_status_value(node->id, s, value, sizeof(value));
        (void)snprintf(out, n, "%s: %s", cfg->name, value);
    }
    else
    {
        menu_info_value(node->id, value, sizeof(value));
        (void)snprintf(out, n, "%s: %s", cfg->name, value);
    }
}

void Car_Menu_Render(void)
{
    CarDebugSnapshot snapshot;
    char text[MENU_TEXT_LEN + 1U];
    char body[MENU_TEXT_LEN + 1U];
    unsigned row;

    if (!Car_Debug_GetSnapshot(&snapshot))
    {
        return;
    }

    menu_build_path(text, sizeof(text));
    menu_draw_row(0U, text, 0U);

    menu_build_status(text, sizeof(text), &snapshot);
    menu_draw_row(1U, text, 0U);

    for (row = 0U; row < MENU_LIST_ROWS; row++)
    {
        int16_t index = (int16_t)(scroll_offset + (int16_t)row);
        MenuNode *node = menu_child_at(page, index);

        if (node == NULL)
        {
            text[0] = '\0';
            menu_draw_row(MENU_LIST_FIRST_ROW + row, text, 0U);
            continue;
        }

        menu_build_body(node, &snapshot, body, sizeof(body));
        if (node == selected)
        {
            (void)snprintf(text, sizeof(text), ">%s", body);
        }
        else
        {
            (void)snprintf(text, sizeof(text), " %s", body);
        }
        menu_draw_row(MENU_LIST_FIRST_ROW + row, text, node == selected);
    }

    menu_dirty = 0U;
}

/* ========================================================================== */
/*                              对外接口                                       */
/* ========================================================================== */

void Car_Menu_Init(void)
{
    static uint8_t initialized;

    menu_params_default();
    (void)Car_Menu_BuildLink();
    (void)Car_Menu_ValidateLinks();

    page = &nodes[MAIN];
    selected = page->child;
    editing = 0U;
    edit_value = 0;
    scroll_offset = 0;
    menu_dirty = 1U;
    menu_message[0] = '\0';
    message_until = 0U;

    Key_Init();

    if (!initialized)
    {
        /* LCD 初始化内部使用 printf；先初始化调试串口，避免 TXE 等待死锁。 */
        NEUQ_Debug_init();
        NEUQ_LCD200_init();
        NEUQ_LCD200_SetDirection(LCD200_DIR_Cross);
        NEUQ_LCD200_SetBackColor(LCD_BLACK);
        NEUQ_LCD200_SetPenColor(LCD_WHITE);
        NEUQ_LCD200_SetAsciiFont(&ASCII_Font24);
        NEUQ_LCD200_Clear();
        menu_invalidate_display();
        initialized = 1U;
    }

    /* 上电先应用默认值；Flash 有合法数据时覆盖。 */
    (void)Car_Menu_ApplyParams();
    flash_present = 0U;
    {
        CarParams params;

        if (Car_Params_Load(&params))
        {
            menu_params_from_flash(&params);
            if (!Car_Menu_ApplyParams())
            {
                menu_params_default();
                (void)Car_Menu_ApplyParams();
            }
            else
            {
                flash_present = 1U;
            }
        }
    }
}

void Car_Menu_Poll(void)
{
    static uint32_t last_status_ms;
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

    if ((uint32_t)(HAL_GetTick() - last_status_ms) >= 200U)
    {
        last_status_ms = HAL_GetTick();
        menu_dirty = 1U;
    }

    if (menu_dirty)
    {
        Car_Menu_Render();
    }
}

uint8_t Car_Menu_Goto(MenuID id)
{
    MenuNode *node;

    if (id <= MENU_NONE || id >= MENU_COUNT)
    {
        return 0U;
    }

    node = &nodes[id];
    if (node->child != NULL)
    {
        page = node;
        selected = page->child;
    }
    else if (node->parent != NULL)
    {
        page = node->parent;
        selected = node;
    }
    else
    {
        page = node;
        selected = page->child;
    }

    editing = 0U;
    scroll_offset = 0;
    menu_scroll_adjust();
    menu_dirty = 1U;
    return 1U;
}

MenuID Car_Menu_GetPageId(void)
{
    return (page != NULL) ? page->id : MENU_NONE;
}

MenuID Car_Menu_GetSelectedId(void)
{
    return (selected != NULL) ? selected->id : MENU_NONE;
}

uint8_t Car_Menu_IsEditing(void)
{
    return editing;
}

int32_t Car_Menu_GetEditValue(void)
{
    return editing ? edit_value : 0;
}

void Car_Menu_SetEditValue(int32_t value)
{
    const MenuConfig *cfg = (selected != NULL) ? selected->info : NULL;

    if (cfg != NULL && cfg->value != NULL)
    {
        if (value < cfg->min)
        {
            value = cfg->min;
        }
        if (value > cfg->max)
        {
            value = cfg->max;
        }
    }

    edit_value = value;
    menu_dirty = 1U;
}

const MenuConfig *Car_Menu_GetConfig(MenuID id)
{
    if (id <= MENU_NONE || id >= MENU_COUNT)
    {
        return NULL;
    }

    return &MenuTable[id];
}

int16_t Car_Menu_NodeCount(void)
{
    return (int16_t)MENU_COUNT;
}

int16_t Car_Menu_GetScrollOffset(void)
{
    return scroll_offset;
}
