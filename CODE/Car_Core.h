#ifndef CAR_CORE_H
#define CAR_CORE_H
#include <stdint.h>
#include "Car_PID.h"
enum { CAR_STOP, CAR_RUN, CAR_FAULT };
enum { CAR_NO_FAULT, CAR_LINK_FAULT, CAR_STALL_FAULT,
       CAR_DIRECTION_FAULT, CAR_RX_FAULT };
/* Commands: 1 ARM, 2 STOP, 3 WHEELS(i16be x2),
 * 4 PID(wheel 0/1, u16be kp/ki/kd scaled100), 5 HEARTBEAT.
 * Frame: C3 3C CMD LEN DATA CRC8(poly07, init0, CMD through DATA).
 * Results: 0 OK, 1 state rejected, 2 value invalid, 3 frame invalid. */
typedef struct {
    Car_Pid_Struct pid[2];
    float target[2], reference[2], speed[2], output[2];
    uint16_t link_ms, stall_ms[2], direction_ms[2], frame_ms;
    float slow_speed[2];
    uint32_t frames;
    uint8_t state, fault, result;
    uint8_t parse_state, cmd, length, index, crc, data[7];
} Car_Core;
void Car_Init(Car_Core *c);
uint8_t Car_Crc8(uint8_t crc, uint8_t byte);
void Car_FeedByte(Car_Core *c, uint8_t byte);
void Car_Tick(Car_Core *c, int32_t left_count, int32_t right_count);
void Car_Fault(Car_Core *c, uint8_t fault);
#endif
