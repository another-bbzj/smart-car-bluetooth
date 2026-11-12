#ifndef CAR_REFERENCE_H
#define CAR_REFERENCE_H

/*
 * ============================================================================
 * 文件名称：Car_Reference.h
 * ============================================================================
 * 模块功能：声明双轮速度参考规划器。
 *
 *   规划器把阶跃目标转换为速度、加速度、加加速度均受限的五次曲线；
 *   两轮共用同一条曲线进度，从静止起步时保持轮速比例。
 *   目标变化时先把参考加速度平滑降为零，再重新规划。
 *
 *   本模块只规划参考值，不代表车辆实际制动；
 *   紧急停止应直接调用 CarReference_Reset() 清零。
 * ============================================================================
 */

typedef struct
{
    /* ---------- 双轮参考状态，下标 0 左轮、1 右轮 ---------- */
    float value[2];             /* 当前参考转速，单位 rpm */
    float acceleration[2];      /* 参考转速的变化率，单位 rpm/s */
    float goal[2];              /* 限幅后的目标转速，单位 rpm */
    float start[2];             /* 当前阶段起点的参考转速，单位 rpm */
    float brake_acceleration[2];/* 阶段 2 开始时的参考加速度，单位 rpm/s，
                                   不是制动 PWM */

    /* ---------- 限制参数（来源：CarReference_Init 参数） ---------- */
    float speed_limit;          /* 参考转速绝对值上限，单位 rpm */
    float acceleration_limit;   /* 参考加速度绝对值上限，单位 rpm/s */
    float jerk_limit;           /* 加加速度绝对值上限，单位 rpm/s² */

    /* ---------- 阶段计时 ---------- */
    float elapsed;              /* 当前阶段已用时间，单位秒 */
    float duration;             /* 当前阶段总时间，单位秒 */

    /* 阶段：
     *   0 保持目标；
     *   1 沿五次曲线变速；
     *   2 将参考加速度线性降为零。 */
    unsigned phase;
} CarReference;

/**
 * @brief 清零规划状态并保存速度、加速度和加加速度限制。
 * @param r            非空且由调用者独占访问的对象。
 * @param speed_limit        参考转速上限，单位 rpm。
 * @param acceleration_limit 参考加速度上限，单位 rpm/s。
 * @param jerk_limit         加加速度上限，单位 rpm/s²。
 * @note 三项限制应为有限正数；本函数只保存限制，不返回参数校验结果。
 */
void CarReference_Init(CarReference *r, float speed_limit,
                       float acceleration_limit, float jerk_limit);

/**
 * @brief 保留三项限制，将双轮参考值、加速度和阶段状态立即清零。
 * @param r 规划器对象。
 * @note 不读取轮速、不驱动电机；由上层决定何时应用该复位结果。
 */
void CarReference_Reset(CarReference *r);

/**
 * @brief 推进双轮参考曲线，结果写入 r->value 与 r->acceleration。
 * @param r      规划器对象。
 * @param target 指向两个目标转速（rpm）的数组，下标 0 左轮、1 右轮；
 *               输入先检查 [-100000, 100000]，再逐轮限制到 speed_limit；
 *               函数不修改输入数组。
 * @param dt_s   本次推进时间（秒），范围 (0, 0.1]。
 * @note 周期、目标检查未通过或限制非正时直接返回，保留已有规划状态。
 *       目标改变时可能先经过阶段 2；目标为零也按曲线变化，
 *       急停应调用 CarReference_Reset()。
 */
void CarReference_Update(CarReference *r, const float target[2], float dt_s);

#endif
