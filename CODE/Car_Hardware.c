/* NEUQ SmartCar derivative; original NEUQ library Copyright (c) 2025
 * NEUQ SmartCar Lab, GPL-3.0-or-later. */
#include "NEUQ_lib_include.h"
#include "Car_Config.h"
#include "Car_Core.h"
#include "Car_Hardware.h"
#include "My_Vofa.h"

void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM dev, uint8_t priority);
extern UART_HandleTypeDef UART_3_handle;
extern uint8_t UART_3_RX_dat;
extern UART_HandleTypeDef UART_2_handle;
extern uint8_t UART_2_RX_dat;
extern TIM_HandleTypeDef Encoder_TIM3_Handle, Encoder_TIM4_Handle;
static Car_Core car;
static NEUQ_FIFO_t rx_fifo;
static uint8_t rx_buffer[128];
static volatile uint8_t rx_fault;
static CarVision vision;
static NEUQ_FIFO_t vision_fifo;
static uint8_t vision_buffer[512];
static volatile uint8_t vision_rx_fault;
static uint16_t previous[2];
static uint8_t reverse[2], waiting[2];
static const PWM_Ports_ENUM pwm[2]={CAR_LEFT_PWM,CAR_RIGHT_PWM};
static const GPIO_Ports_ENUM dir_pin[2]={CAR_LEFT_DIR,CAR_RIGHT_DIR};
static const uint8_t forward_high[2]={CAR_LEFT_FORWARD_HIGH,CAR_RIGHT_FORWARD_HIGH};

static void output(unsigned i, float value)
{
    uint8_t next=value<0;
    if(value==0) { NEUQ_PWM_set_duty(pwm[i],0); waiting[i]=0; return; }
    if(next!=reverse[i]) {
        if(!waiting[i]) { NEUQ_PWM_set_duty(pwm[i],0); waiting[i]=1; return; }
        NEUQ_GPIO_write(dir_pin[i],(next ? !forward_high[i] : forward_high[i]) ? GPIO_High : GPIO_Low);
        reverse[i]=next;
    }
    waiting[i]=0;
    NEUQ_PWM_set_duty(pwm[i],(uint32_t)((value<0 ? -value : value)+0.5f));
}
/* Free-running counters avoid losing encoder edges between read and clear. */
static int32_t delta_count(unsigned i, TIM_HandleTypeDef *timer)
{
    uint16_t now=(uint16_t)__HAL_TIM_GET_COUNTER(timer);
    uint16_t diff=(uint16_t)(now-previous[i]);
    previous[i]=now;
    return (diff&0x8000U) ? (int32_t)diff-65536 : (int32_t)diff;
}
void Car_UART_RX_Handler(uint8_t byte)
{
    if(NEUQ_FIFO_write_byte(&rx_fifo,byte)!=NEUQ_FIFO_OK) rx_fault=1;
}
void Car_Vision_UART_RX_Handler(uint8_t byte)
{
    if(NEUQ_FIFO_write_byte(&vision_fifo,byte)!=NEUQ_FIFO_OK)vision_rx_fault=1;
}
uint8_t Car_GetVisionObservation(CarVisionObservation *out)
{
    uint32_t saved=__get_PRIMASK();uint8_t result;
    __disable_irq();result=CarVision_GetObservation(&vision,HAL_GetTick(),out);
    __set_PRIMASK(saved);return result;
}
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out)
{
    uint32_t saved=__get_PRIMASK();uint8_t result;
    __disable_irq();result=CarVision_GetTile(&vision,index,HAL_GetTick(),out);
    __set_PRIMASK(saved);return result;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==UART4) {
        rx_fault=1;
        HAL_UART_Receive_IT(&UART_3_handle,&UART_3_RX_dat,1);
    } else if(huart->Instance==USART3) {
        vision_rx_fault=1;
        HAL_UART_Receive_IT(&UART_2_handle,&UART_2_RX_dat,1);
    }
}
void Car_PIT_Handler(void)
{
    static unsigned telemetry;
    unsigned n;
    uint8_t byte;
    int32_t l=delta_count(0,&Encoder_TIM4_Handle);
    int32_t r=delta_count(1,&Encoder_TIM3_Handle);
    if(CAR_LEFT_ENCODER_INVERT) l=-l;
    if(CAR_RIGHT_ENCODER_INVERT) r=-r;
    if(rx_fault) {
        Car_Fault(&car,CAR_RX_FAULT);
        while(NEUQ_FIFO_read_byte(&rx_fifo,&byte)==NEUQ_FIFO_OK) {}
        rx_fault=0;
    } else {
        for(n=0;n<32 && NEUQ_FIFO_read_byte(&rx_fifo,&byte)==NEUQ_FIFO_OK;n++) Car_FeedByte(&car,byte);
    }
    Car_Tick(&car,l,r);
    output(0,car.output[0]); output(1,car.output[1]);
    /* Bounded work after the motor update; the parser cannot write motor state. */
    if(vision_rx_fault) {
        CarVision_Invalidate(&vision);
        for(n=0;n<sizeof(vision_buffer) && NEUQ_FIFO_read_byte(&vision_fifo,&byte)==NEUQ_FIFO_OK;n++) {}
        vision_rx_fault=0;
    } else {
        uint32_t now=HAL_GetTick();
        for(n=0;n<64 && NEUQ_FIFO_read_byte(&vision_fifo,&byte)==NEUQ_FIFO_OK;n++)
            CarVision_Feed(&vision,byte,now,car.state==CAR_STOP);
    }
    if(++telemetry>=CAR_TELEMETRY_TICKS) {
        float channels[16];
        telemetry=0;
        channels[0]=(float)car.state; channels[1]=(float)car.fault;
        channels[2]=car.reference[0]; channels[3]=car.speed[0];
        channels[4]=car.reference[1]; channels[5]=car.speed[1];
        channels[6]=car.output[0]/100.0f; channels[7]=car.output[1]/100.0f;
        channels[8]=(float)car.result; channels[9]=(float)car.frames;
        channels[10]=car.pid[0].kp; channels[11]=car.pid[0].ki; channels[12]=car.pid[0].kd;
        channels[13]=car.pid[1].kp; channels[14]=car.pid[1].ki; channels[15]=car.pid[1].kd;
        My_Vofa_push(channels,16); /* 680 bytes/s, fits 9600 baud 8N1. */
    }
}
void Car_Main(void)
{
    unsigned i;
    Car_Init(&car);
    CarVision_Init(&vision);
    NEUQ_FIFO_init(&vision_fifo,vision_buffer,sizeof(vision_buffer));
    NEUQ_GPIO_init(CAR_FLASH_CS,GPIO_Mode_Output_PushPull,GPIO_High);
    NEUQ_FIFO_init(&rx_fifo,rx_buffer,sizeof(rx_buffer));
    for(i=0;i<2;i++) {
        NEUQ_GPIO_init(dir_pin[i],GPIO_Mode_Output_PushPull,forward_high[i] ? GPIO_High : GPIO_Low);
        NEUQ_PWM_init(pwm[i],CAR_PWM_HZ,0);
    }
    NEUQ_Encoder_init(Encoder_TIM4,Encoder_TIM4_PhaseA_PB6,Encoder_TIM4_PhaseB_PB7);
    NEUQ_Encoder_init(Encoder_TIM3,Encoder_TIM3_PhaseA_PA6,Encoder_TIM3_PhaseB_PA7);
    previous[0]=(uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM4_Handle);
    previous[1]=(uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM3_Handle);
    NEUQ_Uart_init((UART_DEV_ENUM)CAR_UART_INDEX,CAR_UART_BAUD);
    My_Vofa_init(CAR_UART_INDEX);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_UART_INDEX,3);
    NEUQ_Uart_init((UART_DEV_ENUM)CAR_VISION_UART_INDEX,CAR_VISION_UART_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_VISION_UART_INDEX,4);
    NEUQ_PIT_init_ms(PIT_CH1,CAR_PERIOD_MS);
    while(1) My_Vofa_poll();
}
