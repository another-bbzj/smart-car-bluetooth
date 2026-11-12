/* 双轮共享时间轴的五次速度规划；阶段定义与接口前提见 Car_Reference.h。 */
#include "Car_Reference.h"
#include <math.h>
#include <string.h>

static float absolute(float x) { return x<0?-x:x; }
static float max(float x,float y) { return x>y?x:y; }
static float clip(float x,float lim) { return x>lim?lim:(x<-lim?-lim:x); }
void CarReference_Reset(CarReference *r)
{
    unsigned i;
    for(i=0;i<2;i++)r->value[i]=r->acceleration[i]=r->goal[i]=r->start[i]=r->brake_acceleration[i]=0;
    r->phase=0;r->elapsed=r->duration=0;
}
void CarReference_Init(CarReference *r,float speed,float acceleration,float jerk)
{
    memset(r,0,sizeof(*r));r->speed_limit=speed;r->acceleration_limit=acceleration;r->jerk_limit=jerk;
}
/* 按变化量较大的车轮确定共同持续时间，使两轮同时到达目标。 */
static void begin_profile(CarReference *r)
{
    float distance=max(absolute(r->goal[0]-r->value[0]),absolute(r->goal[1]-r->value[1]));
    r->start[0]=r->value[0];r->start[1]=r->value[1];r->elapsed=0;
    /* 五次平滑函数一阶导峰值 1.875、二阶导绝对值峰值 10/√3；
     * 分别推导加速度与加加速度约束所需时间，取较大者，且至少为 5 ms。 */
    r->duration=max(0.005f,max(1.875f*distance/r->acceleration_limit,
                             sqrtf(5.773503f*distance/r->jerk_limit)));
    r->phase=1;
}
void CarReference_Update(CarReference *r,const float target[2],float dt)
{
    unsigned i;float goal[2],peak,remaining,step,t0,t1,progress,rate;
    if(!(dt>0 && dt<=0.1f && r->speed_limit>0 && r->acceleration_limit>0 && r->jerk_limit>0))return;
    for(i=0;i<2;i++) {
        if(!(target[i]>=-100000 && target[i]<=100000))return;
        goal[i]=clip(target[i],r->speed_limit);
    }
    if(goal[0]!=r->goal[0] || goal[1]!=r->goal[1]) {
        r->goal[0]=goal[0];r->goal[1]=goal[1];
        peak=max(absolute(r->acceleration[0]),absolute(r->acceleration[1]));
        /* 变速途中改目标时，先以受限加加速度消除当前参考加速度。 */
        if(peak>0.00001f) {
            r->phase=2;r->elapsed=0;r->duration=peak/r->jerk_limit;
            for(i=0;i<2;i++) { r->start[i]=r->value[i];r->brake_acceleration[i]=r->acceleration[i]; }
        } else begin_profile(r);
    }
    /* 一个采样周期可能跨过阶段边界，剩余时间继续推进下一阶段。 */
    remaining=dt;
    if(r->phase==2) {
        step=r->duration-r->elapsed;if(step>remaining)step=remaining;
        r->elapsed+=step;remaining-=step;t1=r->elapsed/r->duration;
        for(i=0;i<2;i++) {
            /* 将 a(t)=a0*(1-t/T) 积分得到参考速度；此阶段减小的是加速度，
             * 参考速度仍可能继续沿原方向变化，不会直接跳到新目标。 */
            r->value[i]=r->start[i]+r->brake_acceleration[i]*r->duration*(t1-0.5f*t1*t1);
            r->acceleration[i]=r->brake_acceleration[i]*(1-t1);
        }
        if(r->elapsed>=r->duration) {
            r->acceleration[0]=r->acceleration[1]=0;begin_profile(r);
        }
    }
    if(r->phase==1 && remaining>0) {
        r->elapsed+=remaining;
        t0=r->elapsed/r->duration;if(t0>1)t0=1;
        /* s(t)=10t³-15t⁴+6t⁵；rate 为对实际时间的导数，供加速度前馈使用。 */
        progress=t0*t0*t0*(10+t0*(-15+6*t0));
        rate=30*t0*t0*(1-t0)*(1-t0)/r->duration;
        for(i=0;i<2;i++) {
            r->value[i]=r->start[i]+(r->goal[i]-r->start[i])*progress;
            r->acceleration[i]=(r->goal[i]-r->start[i])*rate;
        }
        /* 终点显式赋目标并清零加速度，消除浮点累积误差，后续保持该参考值。 */
        if(r->elapsed>=r->duration) {
            r->phase=0;
            for(i=0;i<2;i++) { r->value[i]=r->goal[i];r->acceleration[i]=0; }
        }
    }
}
