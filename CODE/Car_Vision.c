#include "Car_Vision.h"
#include <string.h>

static uint16_t u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]*256U+p[1]); }
static uint32_t u32(const uint8_t *p) { return ((uint32_t)u16(p)<<16)|u16(p+2); }
static int32_t i16(const uint8_t *p)
{ uint16_t n=u16(p);return (n&0x8000U) ? (int32_t)n-65536 : (int32_t)n; }
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte)
{
    unsigned i;crc^=(uint16_t)byte<<8;
    for(i=0;i<8;i++)crc=(uint16_t)((crc&0x8000U) ? (crc<<1)^0x1021U : crc<<1);
    return crc;
}
void CarVision_Init(CarVision *v) { memset(v,0,sizeof(*v)); }
void CarVision_Invalidate(CarVision *v)
{
    unsigned i;
    v->used=0;v->have_observation=0;
    for(i=0;i<4;i++)v->tiles[i].valid=0;
}
static void discard(CarVision *v, unsigned n)
{
    v->used=(uint8_t)(v->used-n);
    if(v->used)memmove(v->buffer,v->buffer+n,v->used);
}
static void accept(CarVision *v, uint32_t now, uint8_t allow_hello)
{
    const uint8_t *f=v->buffer,*p=f+14;
    uint8_t type=f[3];
    uint16_t len=u16(f+4),seq=u16(f+6),age=u16(f+12),delta;
    uint32_t session=u32(f+8);
    unsigned i;
    if(f[2]!=1 || session==0) { v->rejected_frames++;return; }
    if(type==0 && len==0 && age==0) {
        if(!allow_hello) { v->rejected_frames++;return; }
        if(!v->session_valid || v->session!=session) {
            v->have_observation=0;v->have_sequence=0;
            for(i=0;i<4;i++)v->tiles[i].valid=0;
            v->session=session;v->session_valid=1;
        }
        v->accepted_frames++;return;
    }
    if(!v->session_valid || session!=v->session || age>CAR_VISION_MAX_AGE_MS) {
        v->rejected_frames++;return;
    }
    delta=(uint16_t)(seq-v->last_sequence);
    if(v->have_sequence && (delta==0 || delta>=0x8000U)) { v->rejected_frames++;return; }
    if(type==1 && len==16) {
        CarVisionObservation o;
        int32_t lateral=i16(p+4),heading=i16(p+6),curvature=i16(p+8);
        uint16_t flags=u16(p),confidence=u16(p+2);
        if(flags>15 || confidence>1000 || lateral < -1000 || lateral > 1000 ||
           heading < -3142 || heading > 3142 || curvature < -20000 || curvature > 20000 ||
           p[10]>VISION_ELEMENT_TAG || (p[11]>7 && p[11]!=255) || u16(p+14)!=0) {
            v->rejected_frames++;return;
        }
        memset(&o,0,sizeof(o));
        o.received_ms=now;o.session=session;o.sequence=seq;o.processing_age_ms=age;
        o.valid_mask=flags;o.confidence=confidence/1000.0f;
        o.lateral_m=lateral/1000.0f;o.heading_rad=heading/1000.0f;
        o.curvature_inv_m=curvature/1000.0f;o.distance_m=u16(p+12)/1000.0f;
        o.element=p[10];o.tag_id=p[11];
        v->observation=o;v->have_observation=1;
    } else if(type==2 && len==10) {
        CarVisionTile tile;
        if(p[0]>3) { v->rejected_frames++;return; }
        for(i=1;i<10;i++)if(p[i]>6) { v->rejected_frames++;return; }
        memset(&tile,0,sizeof(tile));tile.received_ms=now;tile.session=session;
        tile.sequence=seq;tile.valid=1;memcpy(tile.cells,p+1,9);
        v->tiles[p[0]]=tile;
    } else { v->rejected_frames++;return; }
    v->last_sequence=seq;v->have_sequence=1;v->accepted_frames++;
}
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello)
{
    unsigned i,total;uint16_t len,crc;
    if(v->used && (uint32_t)(now_ms-v->last_byte_ms)>CAR_VISION_FRAME_GAP_MS) v->used=0;
    v->last_byte_ms=now_ms;
    if(v->used>=CAR_VISION_MAX_FRAME)discard(v,1);
    v->buffer[v->used++]=byte;
    while(v->used) {
        if(v->buffer[0]!=0xa5) { discard(v,1);continue; }
        if(v->used<2)return;
        if(v->buffer[1]!=0x5a) { discard(v,1);continue; }
        if(v->used<6)return;
        len=u16(v->buffer+4);
        if(len>CAR_VISION_MAX_PAYLOAD) { v->bad_frames++;discard(v,1);continue; }
        total=16U+len;
        if(v->used<total)return;
        crc=0xffff;
        for(i=2;i<total-2;i++)crc=CarVision_Crc16(crc,v->buffer[i]);
        if(crc!=u16(v->buffer+total-2)) { v->bad_frames++;discard(v,1);continue; }
        accept(v,now_ms,allow_hello);discard(v,total);
    }
}
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out)
{
    uint32_t age;
    if(!out || !v->have_observation)return 0;
    age=(uint32_t)(now_ms-v->observation.received_ms);
    if(age>CAR_VISION_MAX_AGE_MS ||
       age>CAR_VISION_MAX_AGE_MS-v->observation.processing_age_ms)return 0;
    *out=v->observation;return 1;
}
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out)
{
    if(!out || index>3 || !v->tiles[index].valid ||
       (uint32_t)(now_ms-v->tiles[index].received_ms)>=0x80000000U)return 0;
    *out=v->tiles[index];return 1;
}
