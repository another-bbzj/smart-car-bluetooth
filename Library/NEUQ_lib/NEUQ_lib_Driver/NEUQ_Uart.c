/*********************************************************************************************************************
* NEUQ Library 即（东北大学秦皇岛分校 开源库）是一个基于官方 HAL 库的第三方开源库
* Copyright (c) 2025 NEUQ SmartCar Lab
*
* 本文件是 NEUQ Library 的一部分
*
* NEUQ Library 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 
* 欢迎各位使用并传播本程序 但修改内容时必须保留作者的版权声明（即本声明）
*
* 文件名称          NEUQ_Uart
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2025-09-05        张瑞        first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

UART_HandleTypeDef UART_0_handle={0};
UART_HandleTypeDef UART_1_handle={0};
UART_HandleTypeDef UART_2_handle={0};
UART_HandleTypeDef UART_3_handle={0};

uint8_t UART_0_RX_dat=0;
uint8_t UART_1_RX_dat=0;
uint8_t UART_2_RX_dat=0;
uint8_t UART_3_RX_dat=0;

//============================================================================
// 函数功能：初始化串口
// 入口参数：选择串口号，选择波特率
// 使用示例：NEUQ_Uart_init(UART_0,UART_BAUD_115200);
// 返回  值：无
//============================================================================
void NEUQ_Uart_init(UART_DEV_ENUM DEV_select,UART_BAUD_ENUM BAUD_select){
    UART_HandleTypeDef *Handle_use=NULL;
    switch(DEV_select){
        case UART_0:{
            __HAL_RCC_USART1_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_9|GPIO_PIN_10;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF7_USART1;
            HAL_GPIO_Init(GPIOA,&GPIO_initure);

            UART_0_handle.Instance=USART1;
            Handle_use=&UART_0_handle;
            break;
        }
        case UART_1:{
            __HAL_RCC_USART2_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_2|GPIO_PIN_3;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF7_USART2;
            HAL_GPIO_Init(GPIOA,&GPIO_initure);

            UART_1_handle.Instance=USART2;
            Handle_use=&UART_1_handle;
            break;
        }
        case UART_2:{
            __HAL_RCC_USART3_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_10|GPIO_PIN_11;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF7_USART3;
            HAL_GPIO_Init(GPIOB,&GPIO_initure);

            UART_2_handle.Instance=USART3;
            Handle_use=&UART_2_handle;
            break;
        }
        case UART_3:{
            __HAL_RCC_UART4_CLK_ENABLE();
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_10|GPIO_PIN_11;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF8_UART4;
            HAL_GPIO_Init(GPIOC,&GPIO_initure);

            UART_3_handle.Instance=UART4;
            Handle_use=&UART_3_handle;
            break;
        }
    }

    switch(BAUD_select){
        case UART_BAUD_9600:{
            Handle_use->Init.BaudRate=9600;
            break;
        }        
        case UART_BAUD_115200:{
            Handle_use->Init.BaudRate=115200;
            break;
        }
        case UART_BAUD_230400:{
            Handle_use->Init.BaudRate=230400;
            break;
        }
        case UART_BAUD_460800:{
            Handle_use->Init.BaudRate=460800;
            break;
        }
        case UART_BAUD_1000000:{
            Handle_use->Init.BaudRate=1000000;
            break;
        }
        case UART_BAUD_2000000:{
            Handle_use->Init.BaudRate=2000000;
            break;
        }
    }

    Handle_use->Init.WordLength=UART_WORDLENGTH_8B;
    Handle_use->Init.StopBits=UART_STOPBITS_1;
    Handle_use->Init.Parity=UART_PARITY_NONE;
    Handle_use->Init.Mode=UART_MODE_TX_RX;
    Handle_use->Init.HwFlowCtl=UART_HWCONTROL_NONE;
    Handle_use->Init.OverSampling=UART_OVERSAMPLING_16;

    HAL_UART_Init(Handle_use);
}

//============================================================================
// 函数功能：发送一个字节
// 入口参数：选择串口号，发送的字节
// 使用示例：NEUQ_Uart_write_byte(UART_0,'A');
// 返回  值：无
//============================================================================
void NEUQ_Uart_write_byte(UART_DEV_ENUM DEV_select,uint8_t byte){
    switch(DEV_select){
        case UART_0:{
            while (!(USART1->SR & USART_SR_TXE)); // 等待发送缓冲区空
            USART1->DR = byte;                    // 写入数据寄存器
            break;
        }
        case UART_1:{
            while (!(USART2->SR & USART_SR_TXE)); // 等待发送缓冲区空
            USART2->DR = byte;                    // 写入数据寄存器
            break;
        }
        case UART_2:{
            while (!(USART3->SR & USART_SR_TXE)); // 等待发送缓冲区空
            USART3->DR = byte;                    // 写入数据寄存器
            break;
        }
        case UART_3:{
            while (!(UART4->SR & USART_SR_TXE)); // 等待发送缓冲区空
            UART4->DR = byte;                    // 写入数据寄存器
            break;
        }
    }
}

//============================================================================
// 函数功能：发送字符串
// 入口参数：选择串口号，发送的字符串
// 使用示例：NEUQ_Uart_write_string(UART_0,"Hello World");
// 返回  值：无
//============================================================================
void NEUQ_Uart_write_string(UART_DEV_ENUM DEV_select,uint8_t *str){
    int ptr=0;
    while(str[ptr]!='\0'){
        NEUQ_Uart_write_byte(DEV_select,str[ptr++]);
    }
}


//============================================================================
// 函数功能：发送数据
// 入口参数：选择串口号，发送的数据指针，发送的数据长度
// 使用示例：NEUQ_Uart_write_data(UART_0,buf,10);
// 返回  值：无
//============================================================================
void NEUQ_Uart_write_data(UART_DEV_ENUM DEV_select,uint8_t *pdata,uint32_t len){
    for(int i=0;i<len;i++){
        NEUQ_Uart_write_byte(DEV_select,pdata[i]);
    }
}

//============================================================================
// 函数功能：使能串口接收中断
// 入口参数：选择串口号，中断优先级
// 使用示例：NEUQ_Uart_Enable_RX_Interrupt(UART_0,5);
// 返回  值：无
void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select,uint8_t priority){
    switch(DEV_select){
        case UART_0:{
            HAL_NVIC_SetPriority(USART1_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(USART1_IRQn);         
            HAL_UART_Receive_IT(&UART_0_handle,&UART_0_RX_dat,1);
            break;
        }
        case UART_1:{
            HAL_NVIC_SetPriority(USART2_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(USART2_IRQn);    
            HAL_UART_Receive_IT(&UART_1_handle,&UART_1_RX_dat,1);
            break;
        }
        case UART_2:{
            HAL_NVIC_SetPriority(USART3_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(USART3_IRQn);    
            HAL_UART_Receive_IT(&UART_2_handle,&UART_2_RX_dat,1);
            break;
        }
        case UART_3:{
            HAL_NVIC_SetPriority(UART4_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(UART4_IRQn);    
            HAL_UART_Receive_IT(&UART_3_handle,&UART_3_RX_dat,1);
            break;
        }
    }
}

//============================================================================
// 函数功能：使能串口发送中断
// 入口参数：选择串口号，中断优先级
// 使用示例：NEUQ_Uart_Enable_TX_Interrupt(UART_0,5);
// 返回  值：无
//============================================================================
void NEUQ_Uart_Enable_TX_Interrupt(UART_DEV_ENUM DEV_select,uint8_t priority){
    switch(DEV_select){
        case UART_0:{
            HAL_NVIC_SetPriority(USART1_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(USART1_IRQn);         
            __HAL_UART_ENABLE_IT(&UART_0_handle,UART_IT_TC);
            break;
        }
        case UART_1:{
            HAL_NVIC_SetPriority(USART2_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(USART2_IRQn);    
            HAL_UART_Receive_IT(&UART_1_handle,&UART_1_RX_dat,1);
            break;
        }
        case UART_2:{
            HAL_NVIC_SetPriority(USART3_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(USART3_IRQn);    
            HAL_UART_Receive_IT(&UART_2_handle,&UART_2_RX_dat,1);
            break;
        }
        case UART_3:{
            HAL_NVIC_SetPriority(UART4_IRQn, priority, 0);// 设置中断优先级
            HAL_NVIC_EnableIRQ(UART4_IRQn);    
            HAL_UART_Receive_IT(&UART_3_handle,&UART_3_RX_dat,1);
            break;
        }
    }
}

// 中断服务函数
void USART1_IRQHandler(void){HAL_UART_IRQHandler(&UART_0_handle);}
void USART2_IRQHandler(void){HAL_UART_IRQHandler(&UART_1_handle);}
void USART3_IRQHandler(void){HAL_UART_IRQHandler(&UART_2_handle);}
void UART4_IRQHandler(void){HAL_UART_IRQHandler(&UART_3_handle);}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART1){
        TS_UART_0_RX_CALLBACK(UART_0_RX_dat);
        HAL_UART_Receive_IT(&UART_0_handle,&UART_0_RX_dat,1);
    }
    else if(huart->Instance==USART2){
        TS_UART_1_RX_CALLBACK(UART_1_RX_dat);
        HAL_UART_Receive_IT(&UART_1_handle,&UART_1_RX_dat,1);
    }
    else if(huart->Instance==USART3){
        TS_UART_2_RX_CALLBACK(UART_2_RX_dat);
        HAL_UART_Receive_IT(&UART_2_handle,&UART_2_RX_dat,1);
    }
    else if(huart->Instance==UART4){
        TS_UART_3_RX_CALLBACK(UART_3_RX_dat);
        HAL_UART_Receive_IT(&UART_3_handle,&UART_3_RX_dat,1);
    }
}
