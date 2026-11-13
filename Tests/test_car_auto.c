#include "Car_Core.h"
#include "Car_Vision.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void command(Car_Core *c, unsigned char cmd, const unsigned char *data, unsigned char len)
{
    unsigned char frame[16] = {0xc3, 0x3c}, crc = 0, i;
    frame[2] = cmd; frame[3] = len;
    if (len) memcpy(frame + 4, data, len);
    for (i = 2; i < 4 + len; i++) crc = Car_Crc8(crc, frame[i]);
    frame[4 + len] = crc;
    for (i = 0; i < 5 + len; i++) Car_FeedByte(c, frame[i]);
}
static void arm(Car_Core *c, unsigned diameter, unsigned track, unsigned speed)
{
    unsigned char p[6] = {diameter >> 8, diameter, track >> 8, track, speed >> 8, speed};
    command(c, 6, p, 6);
}
static CarVisionObservation observation(float curvature, unsigned seq)
{
    CarVisionObservation o;
    memset(&o, 0, sizeof(o));
    o.session = 123; o.sequence = (uint16_t)seq;
    o.valid_mask = VISION_CURVATURE_VALID; o.confidence = 1;
    o.curvature_inv_m = curvature;
    return o;
}
int main(void)
{
    Car_Core c;
    CarVisionObservation o = observation(0, 1);
    unsigned i;
    Car_Init(&c);
    arm(&c, 60, 150, 80);
    assert(c.state == CAR_STOP && c.result != 0);
    Car_SetVisionObservation(&c, &o, 0);
    arm(&c, 0, 150, 80);
    assert(c.state == CAR_STOP && c.result == 2);
    arm(&c, 60, 150, 151);
    assert(c.state == CAR_STOP && c.result == 2);
    arm(&c, 60, 150, 80);
    assert(c.state == CAR_RUN && c.mode == CAR_MODE_AUTO && c.result == 0);
    Car_Tick(&c, 0, 0);
    assert(fabsf(c.target[0] - 25.46479f) < 0.01f && c.target[0] == c.target[1]);
    o = observation(2, 2); Car_SetVisionObservation(&c, &o, 0); Car_Tick(&c, 0, 0);
    assert(fabsf(c.target[0] - 21.64507f) < 0.01f);
    assert(fabsf(c.target[1] - 29.28451f) < 0.01f);
    o = observation(-2, 3); Car_SetVisionObservation(&c, &o, 0); Car_Tick(&c, 0, 0);
    assert(c.target[0] > c.target[1]);
    { unsigned char wheels[4] = {0, 180, 0, 180};
      float old = c.target[0]; command(&c, 3, wheels, 4);
      assert(c.result == 1 && c.target[0] == old); }
    /* Re-reading the identical sample cannot restart its freshness timer. */
    for (i = 0; i < 21; i++) { Car_SetVisionObservation(&c, &o, 0); command(&c, 5, 0, 0); Car_Tick(&c, 0, 0); }
    assert(c.state == CAR_FAULT && c.fault == CAR_VISION_FAULT && c.output[0] == 0);
    o = observation(0, 4); Car_SetVisionObservation(&c, &o, 0);
    arm(&c, 60, 150, 80); assert(c.state == CAR_FAULT);
    command(&c, 2, 0, 0); arm(&c, 60, 150, 80); assert(c.state == CAR_RUN);
    o = observation(0, 5); o.valid_mask = 0; Car_SetVisionObservation(&c, &o, 0); Car_Tick(&c, 0, 0);
    assert(c.state == CAR_FAULT && c.fault == CAR_VISION_FAULT && c.target[1] == 0);
    command(&c, 2, 0, 0); command(&c, 1, 0, 0);
    assert(c.state == CAR_RUN && c.mode == CAR_MODE_MANUAL);
    Car_SetVisionObservation(&c, 0, 0); Car_Tick(&c, 0, 0);
    assert(c.state == CAR_RUN); /* Vision failure does not disable manual mode. */
    command(&c, 2, 0, 0);
    o = observation(NAN, 6); Car_SetVisionObservation(&c, &o, 0); arm(&c, 60, 150, 80);
    assert(c.state == CAR_STOP);
    o = observation(0, 7); o.processing_age_ms = 100; Car_SetVisionObservation(&c, &o, 0);
    arm(&c, 60, 150, 80); assert(c.state == CAR_STOP);
    o = observation(2, 8); Car_SetVisionObservation(&c, &o, 0); arm(&c, 20, 150, 150); Car_Tick(&c, 0, 0);
    assert(c.target[1] <= 60.001f && fabsf(c.target[0]/c.target[1] - 0.85f/1.15f) < 0.001f);
    o = observation(0, 9); o.session = 124; Car_SetVisionObservation(&c, &o, 0); Car_Tick(&c, 0, 0);
    assert(c.state == CAR_FAULT && c.fault == CAR_VISION_FAULT);
    /* Fresh vision never substitutes for the supervised Bluetooth heartbeat. */
    Car_Init(&c); o = observation(0, 1); Car_SetVisionObservation(&c, &o, 0); arm(&c, 60, 150, 80);
    for (i = 0; i < 100; i++) { o.sequence++; Car_SetVisionObservation(&c, &o, 0); Car_Tick(&c, 0, 0); }
    assert(c.state == CAR_FAULT && c.fault == CAR_LINK_FAULT);
    /* Physical wheel motion blocks AUTO_ARM, even in STOP. */
    Car_Init(&c); o = observation(0, 1); Car_SetVisionObservation(&c, &o, 0);
    c.speed[0] = 5; arm(&c, 60, 150, 80); assert(c.state == CAR_STOP);
    c.speed[0] = 0; c.speed[1] = -5; arm(&c, 60, 150, 80); assert(c.state == CAR_STOP);
    c.speed[1] = 0; arm(&c, 60, 150, 80); Car_Tick(&c, 0, 0);
    command(&c, 2, 0, 0);
    assert(c.state == CAR_STOP && c.mode == CAR_MODE_MANUAL && c.target[0] == 0 && c.output[1] == 0);
    /* Wide axle plus tight turn would require reversing: refuse it. */
    o = observation(8, 2); Car_SetVisionObservation(&c, &o, 0); arm(&c, 60, 500, 80);
    assert(c.state == CAR_STOP && c.result == 2);
    o = observation(0, 3); o.confidence = .79f; Car_SetVisionObservation(&c, &o, 0);
    arm(&c, 60, 150, 80); assert(c.state == CAR_STOP);
    /* Millisecond counter rollover preserves true age and the strict 100ms gate. */
    o = observation(0, 4); o.received_ms = 0xfffffff0U; o.processing_age_ms = 67;
    Car_SetVisionObservation(&c, &o, 0x10U); arm(&c, 60, 150, 80);
    assert(c.state == CAR_RUN); Car_Tick(&c, 0, 0);
    assert(c.state == CAR_FAULT && c.fault == CAR_VISION_FAULT);
    puts("low-speed autonomy: all checks passed");
    return 0;
}
