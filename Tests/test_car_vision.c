#include "Car_Vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned make_frame(uint8_t *f, uint8_t type, uint16_t seq, uint32_t session,
                           uint16_t age, const uint8_t *data, unsigned len)
{
    unsigned i; uint16_t crc=0xffff;
    f[0]=0xa5;f[1]=0x5a;f[2]=1;f[3]=type;f[4]=0;f[5]=(uint8_t)len;
    f[6]=(uint8_t)(seq>>8);f[7]=(uint8_t)seq;
    f[8]=(uint8_t)(session>>24);f[9]=(uint8_t)(session>>16);
    f[10]=(uint8_t)(session>>8);f[11]=(uint8_t)session;
    f[12]=(uint8_t)(age>>8);f[13]=(uint8_t)age;
    if(len)memcpy(f+14,data,len);
    for(i=2;i<14+len;i++)crc=CarVision_Crc16(crc,f[i]);
    f[14+len]=(uint8_t)(crc>>8);f[15+len]=(uint8_t)crc;
    return len+16;
}
static void feed(CarVision *v, const uint8_t *f, unsigned n, uint32_t t, uint8_t stopped)
{unsigned i;for(i=0;i<n;i++)CarVision_Feed(v,f[i],t,stopped);}
int main(void)
{
    CarVision v; CarVisionObservation o; CarVisionTile tile;
    uint8_t f[64],data[16]={0,15,3,132,0,50,0,100,0,20,6,255,0,80,0,0};
    unsigned n; uint16_t crc=0xffff; const uint8_t *s=(const uint8_t *)"123456789";
    while(*s)crc=CarVision_Crc16(crc,*s++);
    assert(crc==0x29b1);
    CarVision_Init(&v);
    n=make_frame(f,1,1,123,0,data,16);feed(&v,f,n,10,1);
    assert(!CarVision_GetObservation(&v,10,&o)); /* Explicit HELLO required. */
    n=make_frame(f,0,0,123,0,0,0);feed(&v,f,n,20,0);
    assert(!v.session_valid); /* No session takeover while driving. */
    feed(&v,f,n,20,1);assert(v.session_valid);
    n=make_frame(f,1,65535,123,30,data,16);feed(&v,f,n,100,0);
    assert(CarVision_GetObservation(&v,100,&o));
    assert(o.lateral_m>0.049f && o.lateral_m<0.051f && o.element==6);
    assert(!CarVision_GetObservation(&v,271,&o)); /* 30ms processing + 171ms transport age. */
    n=make_frame(f,1,0,123,0,data,16);feed(&v,f,n,280,0); /* Sequence wrap. */
    assert(CarVision_GetObservation(&v,280,&o) && o.sequence==0);
    feed(&v,f,n,350,0);assert(CarVision_GetObservation(&v,450,&o));
    assert(!CarVision_GetObservation(&v,481,&o)); /* Duplicate does not refresh age. */
    n=make_frame(f,1,65534,123,0,data,16);feed(&v,f,n,490,0);
    assert(!CarVision_GetObservation(&v,490,&o)); /* Old frame rejected. */
    n=make_frame(f,1,1,123,0,data,16);f[n-1]^=1;feed(&v,f,n,500,0);
    assert(v.bad_frames>0 && !CarVision_GetObservation(&v,500,&o));
    n=make_frame(f,1,1,123,0,data,16);feed(&v,f,9,510,0);
    feed(&v,f,n,600,0);assert(CarVision_GetObservation(&v,600,&o)); /* Fragment timeout. */
    data[0]=0;data[1]=0;
    n=make_frame(f,1,2,123,0,data,16);feed(&v,f,n,650,0);
    assert(CarVision_GetObservation(&v,650,&o) && o.valid_mask==0); /* Alive, no detection. */
    data[2]=0xff;data[3]=0xff;
    n=make_frame(f,1,3,123,0,data,16);feed(&v,f,n,660,0);
    assert(CarVision_GetObservation(&v,660,&o) && o.sequence==2); /* Invalid confidence ignored. */
    n=make_frame(f,0,0,456,0,0,0);feed(&v,f,n,700,0);
    assert(v.session==123);
    feed(&v,f,n,700,1);assert(v.session==456 && !CarVision_GetObservation(&v,700,&o));
    {uint8_t cells[10]={2,1,2,3,4,5,6,0,0,0};
     n=make_frame(f,2,1,456,0,cells,10);feed(&v,f,n,710,0);
     assert(CarVision_GetTile(&v,2,710,&tile) && tile.cells[5]==6);}
    data[2]=0;data[3]=0;
    n=make_frame(f,1,2,456,0,data,16);feed(&v,f,n,0xfffffff0U,0);
    assert(CarVision_GetObservation(&v,0x10U,&o));
    assert(!CarVision_GetObservation(&v,0x100U,&o));
    CarVision_Invalidate(&v);assert(!CarVision_GetObservation(&v,0x100U,&o));
    puts("vision interface: all checks passed");
    return 0;
}
