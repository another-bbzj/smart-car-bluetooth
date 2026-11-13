#include "Car_Core.h"
#include <assert.h>
#include <stdio.h>

static void send_command(Car_Core *c, unsigned char cmd, const unsigned char *data,
                         unsigned char size)
{
    unsigned char crc = Car_Crc8(Car_Crc8(0, cmd), size);
    unsigned i;
    Car_FeedByte(c, 0xc3); Car_FeedByte(c, 0x3c);
    Car_FeedByte(c, cmd); Car_FeedByte(c, size);
    for (i = 0; i < size; ++i) { crc = Car_Crc8(crc, data[i]); Car_FeedByte(c, data[i]); }
    Car_FeedByte(c, crc);
}

int main(void)
{
    Car_Core c;
    unsigned i;
    const unsigned char wheels[4] = {0, 50, 0, 50};
    Car_Init(&c);
    send_command(&c, 1, 0, 0);
    send_command(&c, 3, wheels, 4);
    for (i = 0; i < 100; ++i) {
        if (i % 10 == 0) send_command(&c, 5, 0, 0);
        Car_Tick(&c, 0, 0);
    }
    assert(c.state == CAR_FAULT && c.fault == CAR_LINK_FAULT);
    assert(c.target[0] == 0 && c.output[0] == 0 && c.output[1] == 0);
    send_command(&c, 2, 0, 0);
    send_command(&c, 1, 0, 0);
    for (i = 0; i < 80; ++i) {
        if (i % 10 == 0) send_command(&c, 3, wheels, 4);
        Car_Tick(&c, 0, 0);
    }
    assert(c.state == CAR_RUN);
    send_command(&c, 2, 0, 0);
    assert(c.state == CAR_STOP && c.target[0] == 0 && c.output[0] == 0);
    puts("Pi wheel command deadline: checks passed");
    return 0;
}
