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
/* 串口中断只入 FIFO；PIT 中断负责解析与控制，主循环负责发送遥测。 */
static Car_Core car;
static NEUQ_FIFO_t rx_fifo;
static uint8_t rx_buffer[128];
static volatile uint8_t rx_fault;
static CarVision vision;
static NEUQ_FIFO_t vision_fifo;
static uint8_t vision_buffer[512];
static volatile uint8_t vision_rx_fault;
/* 编码器上次采样值及控制时钟状态；由周期控制中断维护。 */
static uint16_t previous[2];
static TIM_HandleTypeDef control_clock;
static uint32_t last_control_us;
static uint8_t control_clock_ready;
/* reverse 保存当前方向；waiting 表示已撤去 PWM、正在等待下一次换向调用。 */
static uint8_t reverse[2], waiting[2];
static const PWM_Ports_ENUM pwm[2]={CAR_LEFT_PWM,CAR_RIGHT_PWM};
static const GPIO_Ports_ENUM dir_pin[2]={CAR_LEFT_DIR,CAR_RIGHT_DIR};
static const uint8_t forward_high[2]={CAR_LEFT_FORWARD_HIGH,CAR_RIGHT_FORWARD_HIGH};

/* TIM5 作为 1 MHz 自由运行时钟提供微秒时间，不占用编码器引脚。 */
static uint8_t init_control_clock(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency,timer_hz;
    HAL_RCC_GetClockConfig(&clocks,&latency);
    timer_hz=HAL_RCC_GetPCLK1Freq();
    /* APB1 分频不为 1 时，定时器时钟为 PCLK1 的两倍。 */
    if(clocks.APB1CLKDivider!=RCC_HCLK_DIV1)timer_hz*=2U;
    if(timer_hz<1000000U || timer_hz%1000000U)return 0;
    __HAL_RCC_TIM5_CLK_ENABLE();
    control_clock.Instance=TIM5;
    control_clock.Init.Prescaler=timer_hz/1000000U-1U;
    control_clock.Init.Period=0xffffffffU;
    control_clock.Init.CounterMode=TIM_COUNTERMODE_UP;
    control_clock.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    control_clock.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    return HAL_TIM_Base_Init(&control_clock)==HAL_OK && HAL_TIM_Base_Start(&control_clock)==HAL_OK;
}

/* 正负号控制方向，绝对值控制 PWM；改变方向前先将 PWM 置零，
 * 等下一次输出调用再切换方向引脚，形成一个控制周期的换向间隔。 */
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
/* 编码器不清零，使用 16 位模差处理计数器回绕，避免读后清零丢失脉冲。
 * 前提是相邻采样间的实际增量绝对值小于 32768。 */
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
/* 暂时屏蔽中断以复制一致快照，并恢复调用前的中断屏蔽状态。 */
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
/* 串口出错后标记对应通道并重新挂接单字节接收，交由 PIT 处理失效数据。 */
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
/* 标称每 5 ms 执行；实际 dt 来自 TIM5，控制与保护均使用实测周期。 */
void Car_PIT_Handler(void)
{
    static unsigned telemetry;
    unsigned n;
    uint8_t byte;
    uint32_t sample_us=control_clock_ready ? __HAL_TIM_GET_COUNTER(&control_clock) : 0;
    /* 无符号减法允许微秒计数器跨越一次自然回绕。 */
    uint32_t dt_us=sample_us-last_control_us;
    int32_t l=delta_count(0,&Encoder_TIM4_Handle);
    int32_t r=delta_count(1,&Encoder_TIM3_Handle);
    if(CAR_LEFT_ENCODER_INVERT) l=-l;
    if(CAR_RIGHT_ENCODER_INVERT) r=-r;
    last_control_us=sample_us;
    /* 蓝牙字节可能丢失后不再信任残留队列：先故障停车，再清空待解析数据。 */
    if(rx_fault) {
        Car_Fault(&car,CAR_RX_FAULT);
        while(NEUQ_FIFO_read_byte(&rx_fifo,&byte)==NEUQ_FIFO_OK) {}
        rx_fault=0;
    } else {
        /* 每周期最多处理 32 字节蓝牙数据，限制通信解析占用的中断时间。 */
        for(n=0;n<32 && NEUQ_FIFO_read_byte(&rx_fifo,&byte)==NEUQ_FIFO_OK;n++) Car_FeedByte(&car,byte);
    }
    Car_TickDt(&car,l,r,dt_us);
    output(0,car.output[0]); output(1,car.output[1]);
    /* 更新电机后再处理视觉，每周期最多解析 64 字节；视觉解析器只更新观测。
     * 蓝牙接收异常触发停车，视觉接收异常仅使视觉缓存失效。 */
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
        /* 遥测顺序：状态/故障，左右参考与实测转速，左右输出百分比，
         * 命令结果/帧数，左轮 Kp/Ki/Kd，右轮 Kp/Ki/Kd。 */
        channels[0]=(float)car.state; channels[1]=(float)car.fault;
        channels[2]=car.reference[0]; channels[3]=car.speed[0];
        channels[4]=car.reference[1]; channels[5]=car.speed[1];
        channels[6]=car.output[0]/100.0f; channels[7]=car.output[1]/100.0f;
        channels[8]=(float)car.result; channels[9]=(float)car.frames;
        channels[10]=car.pid[0].kp; channels[11]=car.pid[0].ki; channels[12]=car.pid[0].kd;
        channels[13]=car.pid[1].kp; channels[14]=car.pid[1].ki; channels[15]=car.pid[1].kd;
        My_Vofa_push(channels,16); /* 默认 10 Hz × (16×4 + 4) 字节 = 680 B/s，小于 9600/10 = 960 B/s。 */
    }
}
void Car_Main(void)
{
    unsigned i;
    Car_Init(&car);
    CarVision_Init(&vision);
    NEUQ_FIFO_init(&vision_fifo,vision_buffer,sizeof(vision_buffer));
    /* 先取消板载 Flash 片选，再将共用的 PA5/PA6/PA7 用于方向和编码器。 */
    NEUQ_GPIO_init(CAR_FLASH_CS,GPIO_Mode_Output_PushPull,GPIO_High);
    NEUQ_FIFO_init(&rx_fifo,rx_buffer,sizeof(rx_buffer));
    for(i=0;i<2;i++) {
        NEUQ_GPIO_init(dir_pin[i],GPIO_Mode_Output_PushPull,forward_high[i] ? GPIO_High : GPIO_Low);
        NEUQ_PWM_init(pwm[i],CAR_PWM_HZ,0);
    }
    NEUQ_Encoder_init(Encoder_TIM4,Encoder_TIM4_PhaseA_PB6,Encoder_TIM4_PhaseB_PB7);
    NEUQ_Encoder_init(Encoder_TIM3,Encoder_TIM3_PhaseA_PA6,Encoder_TIM3_PhaseB_PA7);
    /* 在启动周期中断前建立编码器基准，首周期只计算此后产生的脉冲。 */
    previous[0]=(uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM4_Handle);
    previous[1]=(uint16_t)__HAL_TIM_GET_COUNTER(&Encoder_TIM3_Handle);
    NEUQ_Uart_init((UART_DEV_ENUM)CAR_UART_INDEX,CAR_UART_BAUD);
    My_Vofa_init(CAR_UART_INDEX);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_UART_INDEX,3);
    NEUQ_Uart_init((UART_DEV_ENUM)CAR_VISION_UART_INDEX,CAR_VISION_UART_BAUD);
    NEUQ_Uart_Enable_RX_Interrupt((UART_DEV_ENUM)CAR_VISION_UART_INDEX,4);
    control_clock_ready=init_control_clock();
    last_control_us=control_clock_ready ? __HAL_TIM_GET_COUNTER(&control_clock) : 0;
    /* 所有共享对象和外设就绪后再启动控制；时钟失败将使 dt=0 并触发时序故障。 */
    NEUQ_PIT_init_ms(PIT_CH1,CAR_PERIOD_MS);
    /* 控制由 PIT 中断驱动，主循环持续推进遥测串口发送。 */
    while(1) My_Vofa_poll();
}
