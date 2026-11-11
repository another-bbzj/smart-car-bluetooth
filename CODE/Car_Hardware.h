#ifndef CAR_HARDWARE_H
#define CAR_HARDWARE_H
#include <stdint.h>
#include "Car_Vision.h"
void Car_Main(void);
void Car_PIT_Handler(void);
void Car_UART_RX_Handler(uint8_t byte);
void Car_Vision_UART_RX_Handler(uint8_t byte);
/* Thread-safe snapshots; return 0 if absent/stale. Observation fields have validity bits. */
uint8_t Car_GetVisionObservation(CarVisionObservation *out);
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out);
#endif
