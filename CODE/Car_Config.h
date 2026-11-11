#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* Reference schematics: channel 1 is assigned LEFT, channel 2 RIGHT.
 * Verify physical wheel placement and signs with the chassis lifted. */
#define CAR_UART_INDEX          3
#define CAR_UART_BAUD           UART_BAUD_9600
/* Orange Pi vision: USART3, PB10 TX / PB11 RX, separate from Bluetooth. */
#define CAR_VISION_UART_INDEX   2
#define CAR_VISION_UART_BAUD    UART_BAUD_115200
#if CAR_UART_INDEX != 3 || CAR_VISION_UART_INDEX != 2
#error This board adapter currently supports Bluetooth UART4 and vision USART3 only
#endif
#define CAR_MOTOR_NOLOAD_RPM    1000.0f /* User-specified output-shaft no-load speed, not a command limit. */
#define CAR_FLASH_CS            PB0  /* V2 coreboard W25Q CS; deassert before reusing PA5/6/7 */
#define CAR_LEFT_PWM            TIM8_CH1_PC6
#define CAR_LEFT_DIR            PC7
#define CAR_RIGHT_PWM           TIM2_CH3_PA2
#define CAR_RIGHT_DIR           PA5
#define CAR_LEFT_FORWARD_HIGH   1
#define CAR_RIGHT_FORWARD_HIGH  1
#define CAR_LEFT_ENCODER_INVERT 1
#define CAR_RIGHT_ENCODER_INVERT 1
#define CAR_PWM_HZ              8000U
#define CAR_PERIOD_MS           5U
#define CAR_TELEMETRY_TICKS     20U
/* Initial bench assumption only. Verify counts per OUTPUT shaft revolution;
 * JGB37-520 model and 1000rpm alone do not establish PPR or gear ratio. */
#define CAR_ENCODER_CPR         (11.0f * 4.0f * 10.0f)
#define CAR_RPM_LIMIT           180.0f
#define CAR_PWM_LIMIT           3500.0f
#define CAR_RAMP_RPM_PER_TICK    1.0f
#define CAR_LINK_TIMEOUT_MS     500U
#define CAR_FRAME_TIMEOUT_MS    100U
#define CAR_STALL_TIMEOUT_MS    1500U
#define CAR_DIRECTION_TIMEOUT_MS 1000U
#define CAR_DEFAULT_KP         2.0f
#define CAR_DEFAULT_KI         0.12f
#define CAR_DEFAULT_KD         0.0f
#endif
