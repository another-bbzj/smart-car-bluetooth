/* 底盘纯算法核心：蓝牙协议 -> 状态/目标 -> 双轮参考 -> 速度 PID -> 输出。
 * 硬件采样和 PWM 写入由 Car_Hardware.c 完成；公共接口说明见 Car_Core.h。 */
#include "Car_Core.h"
#include "Car_Config.h"
#include <string.h>

static float absolute(float x) { return x < 0 ? -x : x; }
static float clamp(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
/* 停止运动并清除规划器、PID 和保护计时；保留实测转速，供后续测速滤波使用。 */
static void reset_motion(Car_Core *c)
{
    unsigned i;
    CarReference_Reset(&c->profile);
    for(i=0;i<2;i++) {
        c->target[i]=c->reference[i]=c->output[i]=0;
        c->stall_us[i]=c->direction_us[i]=0;
        c->slow_speed[i]=absolute(c->speed[i]);
        Car_Pid_reset(&c->pid[i]);
    }
}
/* 上电进入 CAR_STOP，建立双轮速度规划器及两套独立的速度 PID。 */
void Car_Init(Car_Core *c)
{
    unsigned i;
    memset(c,0,sizeof(*c));
    CarReference_Init(&c->profile,CAR_RPM_LIMIT,CAR_ACCEL_LIMIT_RPM_S,CAR_JERK_LIMIT_RPM_S2);
    for(i=0;i<2;i++) {
        Car_Pid_init(&c->pid[i],CAR_DEFAULT_KP,CAR_DEFAULT_KI,CAR_DEFAULT_KD,CAR_PWM_LIMIT);
        if(!Car_Pid_configure(&c->pid[i],CAR_PID_SETPOINT_WEIGHT,CAR_PID_D_TAU_S,1,CAR_FF_KS,CAR_FF_KV,CAR_FF_KA))
            Car_Fault(c,CAR_CONTROLLER_FAULT);
    }
}
/* 保留首次故障原因，立即清零双轮控制输出，并丢弃尚未收齐的蓝牙帧。 */
void Car_Fault(Car_Core *c, uint8_t fault)
{
    if(c->state!=CAR_FAULT)c->fault=fault;
    reset_motion(c); c->state=CAR_FAULT;
    c->parse_state=0;
}
/* CRC-8：多项式 0x07；调用方以 0 初始化，依次送入命令、长度及负载。 */
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;
    crc^=byte;
    for(i=0;i<8;i++) crc=(uint8_t)((crc&0x80U) ? (crc<<1)^0x07U : crc<<1);
    return crc;
}
static uint16_t u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]*256U+p[1]); }
/* 蓝牙轮速为大端有符号 16 位整数，显式还原负值，单位 rpm。 */
static float signed_rpm(const uint8_t *p)
{
    uint16_t n=u16(p);
    return (float)((n&0x8000U) ? (int32_t)n-65536 : (int32_t)n);
}
/* 仅执行长度和 CRC 已通过检查的命令；frames 包含被状态/参数校验拒绝的命令。 */
static void execute(Car_Core *c)
{
    c->frames++;
    c->result=0;
    switch(c->cmd) {
    case 1: /* ARM：只有停止状态能启动，故障后须先发送 STOP。 */
        if(c->state!=CAR_STOP) { c->result=1; break; }
        reset_motion(c); c->link_us=0; c->state=CAR_RUN; break;
    case 2: /* STOP：清除故障并立即归零，不等待速度规划曲线结束。 */
        reset_motion(c); c->state=CAR_STOP; c->fault=0; c->link_us=0; break;
    case 3: /* WHEELS：左、右目标转速；双轮都为零时立即复位运动状态。 */
        if(c->state!=CAR_RUN) { c->result=1; break; }
        c->target[0]=clamp(signed_rpm(c->data),CAR_RPM_LIMIT);
        c->target[1]=clamp(signed_rpm(c->data+2),CAR_RPM_LIMIT);
        if(c->target[0]==0 && c->target[1]==0) reset_motion(c);
        c->link_us=0; break;
    case 4: /* PID：仅停车时调参；线上整数除以 100 后得到增益。 */
        if(c->state!=CAR_STOP) { c->result=1; break; }
        if(c->data[0]>1) { c->result=2; break; }
        {
            /* 先校验临时控制器，全部成功后再替换，避免部分参数更新。 */
            Car_Pid_Struct next;
            if(!Car_Pid_init(&next,u16(c->data+1)/100.0f,u16(c->data+3)/100.0f,
                             u16(c->data+5)/100.0f,CAR_PWM_LIMIT) ||
               !Car_Pid_configure(&next,CAR_PID_SETPOINT_WEIGHT,CAR_PID_D_TAU_S,1,
                                   CAR_FF_KS,CAR_FF_KV,CAR_FF_KA)) { c->result=2;break; }
            c->pid[c->data[0]]=next;
        }
        break;
    case 5: /* HEARTBEAT：运行时刷新通信看门狗，不改变目标转速。 */
        if(c->state==CAR_RUN) c->link_us=0;
        break;
    default: c->result=3; break;
    }
}
/* 逐字节解析 C3 3C CMD LEN DATA CRC：状态 0/1 找帧头，2/3 读命令和长度，
 * 4 收负载，5 校验。异常字节若是 C3，则保留为下一帧的首字节。 */
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    /* 命令 1~5 的负载长度：0、0、4、7、0；先核验长度，保证 data[7] 不越界。 */
    static const uint8_t lengths[6]={255,0,0,4,7,0};
    /* 此处只刷新帧字节间隔；运行通信看门狗要等合法 ARM/WHEELS/HEARTBEAT
     * 执行时才刷新（STOP 也会清零），噪声或错误帧不能维持运行状态。 */
    c->frame_us=0;
    switch(c->parse_state) {
    case 0: if(byte==0xc3) c->parse_state=1; break;
    case 1: c->parse_state=byte==0x3c ? 2 : (byte==0xc3 ? 1 : 0); break;
    case 2:
        c->cmd=byte; c->crc=Car_Crc8(0,byte); c->parse_state=3; break;
    case 3:
        if(c->cmd<1 || c->cmd>5 || byte!=lengths[c->cmd]) {
            c->parse_state=byte==0xc3 ? 1 : 0; c->result=3; break;
        }
        c->length=byte; c->index=0; c->crc=Car_Crc8(c->crc,byte);
        c->parse_state=byte ? 4 : 5; break;
    case 4:
        c->data[c->index++]=byte; c->crc=Car_Crc8(c->crc,byte);
        if(c->index==c->length) c->parse_state=5;
        break;
    default:
        c->parse_state=0;
        if(byte==c->crc) execute(c);
        else { c->result=3; if(byte==0xc3) c->parse_state=1; }
        break;
    }
}
/* 每次传入本周期编码器增量与实测周期（微秒）；下标 0 为左轮，1 为右轮。
 * 按通信保护、参考规划、测速滤波、PID 和运动保护的顺序更新。 */
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count, uint32_t dt_us)
{
    int32_t counts[2];
    unsigned i;
    float dt=dt_us*0.000001f;
    /* 周期越界时停止控制，避免用异常采样间隔计算速度及积分。 */
    if(dt_us<CAR_MIN_CONTROL_US || dt_us>CAR_MAX_CONTROL_US) { Car_Fault(c,CAR_TIMING_FAULT);return; }
    counts[0]=left_count; counts[1]=right_count;
    if(c->parse_state && (c->frame_us+=dt_us)>=CAR_FRAME_TIMEOUT_MS*1000U) c->parse_state=0;
    if(c->state==CAR_RUN && (c->link_us+=dt_us)>=CAR_LINK_TIMEOUT_MS*1000U) Car_Fault(c,CAR_LINK_FAULT);
    if(c->state==CAR_RUN)CarReference_Update(&c->profile,c->target,dt);
    for(i=0;i<2;i++) {
        /* 计数增量 / 每圈计数 / 周期秒数得到转/秒，再乘 60 换算为 rpm。 */
        float raw=(float)counts[i]*(60.0f/(CAR_ENCODER_CPR*dt));
        float abs_speed;
        /* 一阶低通系数随实际周期变化；停车时仍继续更新实测转速。 */
        c->speed[i]+=(dt/(CAR_SPEED_FILTER_TAU_S+dt))*(raw-c->speed[i]);
        if(c->state!=CAR_RUN) { c->output[i]=0; continue; }
        c->reference[i]=c->profile.value[i];
        /* 单轮目标和参考均归零后撤去驱动、清除积分；不以 PID 主动保持零速。
         * 此处清零的是控制输出，车轮是否已停仍以 speed 反馈判断。 */
        if(c->reference[i]==0 && c->target[i]==0) {
            c->output[i]=0; Car_Pid_reset(&c->pid[i]);
        } else c->output[i]=Car_Pid_update_dt(&c->pid[i],c->reference[i],c->speed[i],c->profile.acceleration[i],dt);
        if(c->pid[i].fault) { Car_Fault(c,CAR_CONTROLLER_FAULT);continue; }
        abs_speed=absolute(c->speed[i]);
        c->slow_speed[i]+=(dt/(0.095f+dt))*(abs_speed-c->slow_speed[i]);
        /* 参考转速和输出均较大、实测转速却很低，持续达到阈值才判定堵转。 */
        if(absolute(c->reference[i])>=40 && absolute(c->output[i])>=1500 && abs_speed<10) {
            c->stall_us[i]+=dt_us;
            if(c->stall_us[i]>=CAR_STALL_TIMEOUT_MS*1000U) Car_Fault(c,CAR_STALL_FAULT);
        } else c->stall_us[i]=0;
        /* 输出接近限幅且持续反向高速运动时检查方向异常；慢速滤波值用于
         * 排除明显减速中的正常换向过程，条件中断即清除计时。 */
        if(c->reference[i]*c->speed[i]<0 && c->output[i]*c->speed[i]<0 &&
           absolute(c->output[i])>0.9f*CAR_PWM_LIMIT && abs_speed>100 &&
           abs_speed>=c->slow_speed[i]-5) {
            c->direction_us[i]+=dt_us;
            if(c->direction_us[i]>=CAR_DIRECTION_TIMEOUT_MS*1000U) Car_Fault(c,CAR_DIRECTION_FAULT);
        } else c->direction_us[i]=0;
    }
}

/* 主机仿真的固定周期兼容入口；硬件使用带实测周期的 Car_TickDt。 */
void Car_Tick(Car_Core *c,int32_t left_count,int32_t right_count)
{ Car_TickDt(c,left_count,right_count,CAR_PERIOD_MS*1000U); }
