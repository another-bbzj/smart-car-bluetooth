/* Consume actual standalone-adapter output using the production F407 parser/core.
 * Fixture: five straight-road frames, five blue gap frames, five road frames. */
#define _CRT_SECURE_NO_WARNINGS
#include "Car_Core.h"
#include <assert.h>
#include <stdio.h>

static void arm(Car_Core *car)
{
    unsigned char bytes[] = {0xc3, 0x3c, 6, 6, 0, 60, 0, 150, 0, 80, 0};
    unsigned i;
    for (i = 2; i < 10; ++i) bytes[10] = Car_Crc8(bytes[10], bytes[i]);
    for (i = 0; i < sizeof(bytes); ++i) Car_FeedByte(car, bytes[i]);
}

int main(int argc, char **argv)
{
    FILE *input;
    CarVision vision;
    Car_Core car;
    CarVisionObservation observation;
    unsigned seen = 0, valid = 0, started = 0, faulted = 0;
    int byte;
    assert(argc == 2);
    input = fopen(argv[1], "rb"); assert(input);
    CarVision_Init(&vision); Car_Init(&car);
    while ((byte = fgetc(input)) != EOF) {
        uint32_t before = vision.accepted_frames;
        CarVision_Feed(&vision, (uint8_t)byte, seen * 10, car.state == CAR_STOP);
        if (vision.accepted_frames == before || !vision.have_observation) continue;
        assert(CarVision_GetObservation(&vision, seen * 10, &observation));
        assert(observation.sequence == seen + 1);
        Car_SetVisionObservation(&car, &observation, seen * 10);
        if (observation.valid_mask & VISION_CURVATURE_VALID) {
            valid++;
            if (!started) { assert(seen < 5); arm(&car); assert(car.state == CAR_RUN); started = 1; }
        }
        if (seen >= 5 && seen < 10) assert(observation.valid_mask == 0);
        Car_Tick(&car, 0, 0);
        if (seen >= 5) {
            assert(car.state == CAR_FAULT && car.fault == CAR_VISION_FAULT);
            assert(car.output[0] == 0 && car.output[1] == 0);
            faulted++;
        }
        seen++;
    }
    assert(!ferror(input)); fclose(input);
    assert(seen == 15 && valid >= 2 && valid <= 10 && started && faulted == 10);
    assert(vision.bad_frames == 0 && vision.rejected_frames == 0);
    puts("adapter replay -> firmware: gap stops, recovered vision never auto-rearms");
    return 0;
}
