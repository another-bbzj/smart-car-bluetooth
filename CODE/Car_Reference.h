#ifndef CAR_REFERENCE_H
#define CAR_REFERENCE_H
/* 双轮参考转速规划，单位 rpm。两轮共用曲线进度，从静止起步时保持轮速比例。
 * 目标变化时先把参考加速度平滑降为零，再构造限加速度/限加加速度的五次曲线。
 * 此处只规划参考值，不代表车辆实际制动；紧急停止直接调用 Reset 清零。 */
typedef struct {
    /* 下标 0/1 为左/右轮。value：当前参考转速；acceleration：其变化率；
     * goal：限幅后的目标；start：当前阶段起点转速；
     * brake_acceleration：阶段 2 开始时的参考加速度，不是制动 PWM。 */
    float value[2],acceleration[2],goal[2],start[2],brake_acceleration[2];
    /* 三项限制分别为 rpm、rpm/s、rpm/s²；elapsed/duration 为阶段已用/总时间（秒）。 */
    float speed_limit,acceleration_limit,jerk_limit,elapsed,duration;
    /* phase：0 保持目标，1 沿五次曲线变速，2 将参考加速度线性降为零。
     * elapsed/duration 单位秒；acceleration_limit 为 rpm/s，jerk_limit 为 rpm/s²。 */
    unsigned phase;
} CarReference;
/**
 * @brief 清零规划状态并保存速度、加速度和加加速度限制。
 * @param r 非空且由调用者独占访问的对象；三项限制应为有限正数。
 * @note 本函数只保存限制，不返回参数校验结果。
 */
void CarReference_Init(CarReference *r,float speed_limit,float acceleration_limit,float jerk_limit);
/**
 * @brief 保留三项限制，将双轮参考值、加速度和阶段状态立即清零。
 * @note 不读取轮速、不驱动电机；由上层决定何时应用该复位结果。
 */
void CarReference_Reset(CarReference *r);
/**
 * @brief 推进双轮参考曲线，结果写入 r->value 与 r->acceleration。
 * @param target 指向两个目标转速（rpm），输入先检查 [-100000,100000]，
 *               再逐轮限制到 speed_limit；函数不修改输入数组。
 * @param dt_s 本次推进时间（秒），范围 (0,0.1]。
 * @note 周期、目标检查未通过或限制非正时直接返回，保留已有规划状态。
 *       目标改变时可能先经过阶段 2；目标为零也按曲线变化，急停应调用 Reset。
 */
void CarReference_Update(CarReference *r,const float target[2],float dt_s);
#endif
