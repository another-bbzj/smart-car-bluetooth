#include "Car_Core.h"
#include "Car_Config.h"
#include <string.h>

static float absolute(float x) { return x < 0 ? -x : x; }
static float clamp(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
static void reset_motion(Car_Core *c)
{
    unsigned i;
    for(i=0;i<2;i++) {
        c->target[i]=c->reference[i]=c->output[i]=0;
        c->stall_ms[i]=c->direction_ms[i]=0;
        c->slow_speed[i]=absolute(c->speed[i]);
        Car_Pid_reset(&c->pid[i]);
    }
}
void Car_Init(Car_Core *c)
{
    unsigned i;
    memset(c,0,sizeof(*c));
    for(i=0;i<2;i++) Car_Pid_init(&c->pid[i],CAR_DEFAULT_KP,CAR_DEFAULT_KI,CAR_DEFAULT_KD,CAR_PWM_LIMIT);
}
void Car_Fault(Car_Core *c, uint8_t fault)
{
    reset_motion(c); c->state=CAR_FAULT; c->fault=fault;
    c->parse_state=0;
}
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;
    crc^=byte;
    for(i=0;i<8;i++) crc=(uint8_t)((crc&0x80U) ? (crc<<1)^0x07U : crc<<1);
    return crc;
}
static uint16_t u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]*256U+p[1]); }
static float signed_rpm(const uint8_t *p)
{
    uint16_t n=u16(p);
    return (float)((n&0x8000U) ? (int32_t)n-65536 : (int32_t)n);
}
static void execute(Car_Core *c)
{
    c->frames++;
    c->result=0;
    switch(c->cmd) {
    case 1:
        if(c->state!=CAR_STOP) { c->result=1; break; }
        reset_motion(c); c->link_ms=0; c->state=CAR_RUN; break;
    case 2:
        reset_motion(c); c->state=CAR_STOP; c->fault=0; c->link_ms=0; break;
    case 3:
        if(c->state!=CAR_RUN) { c->result=1; break; }
        c->target[0]=clamp(signed_rpm(c->data),CAR_RPM_LIMIT);
        c->target[1]=clamp(signed_rpm(c->data+2),CAR_RPM_LIMIT);
        if(c->target[0]==0 && c->target[1]==0) reset_motion(c);
        c->link_ms=0; break;
    case 4:
        if(c->state!=CAR_STOP) { c->result=1; break; }
        if(c->data[0]>1) { c->result=2; break; }
        Car_Pid_init(&c->pid[c->data[0]],u16(c->data+1)/100.0f,
                    u16(c->data+3)/100.0f,u16(c->data+5)/100.0f,CAR_PWM_LIMIT);
        break;
    case 5:
        if(c->state==CAR_RUN) c->link_ms=0;
        break;
    default: c->result=3; break;
    }
}
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    static const uint8_t lengths[6]={255,0,0,4,7,0};
    c->frame_ms=0;
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
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count)
{
    int32_t counts[2];
    unsigned i;
    counts[0]=left_count; counts[1]=right_count;
    if(c->parse_state && (c->frame_ms+=CAR_PERIOD_MS)>=CAR_FRAME_TIMEOUT_MS) c->parse_state=0;
    if(c->state==CAR_RUN && (c->link_ms+=CAR_PERIOD_MS)>=CAR_LINK_TIMEOUT_MS) Car_Fault(c,CAR_LINK_FAULT);
    for(i=0;i<2;i++) {
        float raw=(float)counts[i]*(60000.0f/(CAR_ENCODER_CPR*CAR_PERIOD_MS));
        float abs_speed, delta;
        c->speed[i]+=0.5f*(raw-c->speed[i]);
        if(c->state!=CAR_RUN) { c->output[i]=0; continue; }
        delta=clamp(c->target[i]-c->reference[i],CAR_RAMP_RPM_PER_TICK);
        c->reference[i]+=delta;
        if(c->reference[i]==0 && c->target[i]==0) {
            c->output[i]=0; Car_Pid_reset(&c->pid[i]);
        } else c->output[i]=Car_Pid_update(&c->pid[i],c->reference[i],c->speed[i]);
        abs_speed=absolute(c->speed[i]);
        c->slow_speed[i]+=0.05f*(abs_speed-c->slow_speed[i]);
        if(absolute(c->reference[i])>=40 && absolute(c->output[i])>=1500 && abs_speed<10) {
            c->stall_ms[i]+=CAR_PERIOD_MS;
            if(c->stall_ms[i]>=CAR_STALL_TIMEOUT_MS) Car_Fault(c,CAR_STALL_FAULT);
        } else c->stall_ms[i]=0;
        if(c->reference[i]*c->speed[i]<0 && c->output[i]*c->speed[i]<0 &&
           absolute(c->output[i])>0.9f*CAR_PWM_LIMIT && abs_speed>100 &&
           abs_speed>=c->slow_speed[i]-5) {
            c->direction_ms[i]+=CAR_PERIOD_MS;
            if(c->direction_ms[i]>=CAR_DIRECTION_TIMEOUT_MS) Car_Fault(c,CAR_DIRECTION_FAULT);
        } else c->direction_ms[i]=0;
    }
}
