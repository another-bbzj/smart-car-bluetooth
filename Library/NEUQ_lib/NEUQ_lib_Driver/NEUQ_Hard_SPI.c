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
* 文件名称          NEUQ_Hard_SPI
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2023-09-05        张瑞        first version
* 2026-09-05        Codex       SPI1用于板载Flash，SPI3用于板载LCD
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

SPI_HandleTypeDef Hard_SPI_1_Handle={0};
SPI_HandleTypeDef Hard_SPI_2_Handle={0};
SPI_HandleTypeDef Hard_SPI_3_Handle={0};

static SPI_HandleTypeDef *NEUQ_Hard_SPI_get_handle(Hard_SPI_DEV_ENUM DEV_select)
{
    switch(DEV_select){
        case Hard_SPI_1: return &Hard_SPI_1_Handle;
        case Hard_SPI_3: return &Hard_SPI_3_Handle;
        default: return NULL;
    }
}

//============================================================================
// 函数功能：初始化硬件SPI
// 入口参数：DEV_select:选择使用的SPI设备
//          SCK_select:选择SCK引脚
//          SDA_select:选择SDA引脚
//          CS_select:选择CS引脚
// 返回参数：无
// 使用示例：NEUQ_Hard_SPI_init(Hard_SPI_1,Hard_SPI_1_SCK_PA5,Hard_SPI_1_SDA_PA7,PB0);
//============================================================================
void NEUQ_Hard_SPI_init(Hard_SPI_DEV_ENUM DEV_select,Hard_SPI_SCK_ENUM SCK_select,Hard_SPI_SDA_ENUM SDA_select,GPIO_Ports_ENUM CS_select){
    SPI_HandleTypeDef *Handle_use=NULL;
    GPIO_InitTypeDef GPIO_initure={0};

    switch(DEV_select){
        case Hard_SPI_1:{
            if(SCK_select!=Hard_SPI_1_SCK_PA5 || SDA_select!=Hard_SPI_1_SDA_PA7){
                return;
            }

            __HAL_RCC_GPIOA_CLK_ENABLE();
            __HAL_RCC_SPI1_CLK_ENABLE();

            GPIO_initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF5_SPI1;
            HAL_GPIO_Init(GPIOA,&GPIO_initure);

            Hard_SPI_1_Handle.Instance=SPI1;
            Handle_use=&Hard_SPI_1_Handle;
            break;
        }
        case Hard_SPI_3:{
            if(SCK_select!=Hard_SPI_3_SCK_PB3 || SDA_select!=Hard_SPI_3_SDA_PB5){
                return;
            }

            __HAL_RCC_GPIOB_CLK_ENABLE();
            __HAL_RCC_SPI3_CLK_ENABLE();

            GPIO_initure.Pin=GPIO_PIN_3|GPIO_PIN_5;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF6_SPI3;
            HAL_GPIO_Init(GPIOB,&GPIO_initure);

            Hard_SPI_3_Handle.Instance=SPI3;
            Handle_use=&Hard_SPI_3_Handle;
            break;
        }
        default:
            return;
    }

    Handle_use->Init.Mode=SPI_MODE_MASTER;                            //SPI：主机模式
    Handle_use->Init.Direction=(DEV_select==Hard_SPI_1) ? SPI_DIRECTION_2LINES : SPI_DIRECTION_1LINE;
    Handle_use->Init.DataSize=SPI_DATASIZE_8BIT;                      //数据宽度：8bit
    Handle_use->Init.CLKPolarity=SPI_POLARITY_LOW;                    //CLK空闲为低
    Handle_use->Init.CLKPhase=SPI_PHASE_1EDGE;                        //数据在第一个边沿有效
    Handle_use->Init.NSS=SPI_NSS_SOFT;                                //软件片选
    // Flash先使用较保守的10.5 MHz，降低读ID正常但擦写数据不稳定的概率；LCD仍保持21 MHz
    Handle_use->Init.BaudRatePrescaler=(DEV_select==Hard_SPI_1) ?
                                        SPI_BAUDRATEPRESCALER_8 : SPI_BAUDRATEPRESCALER_2;
    Handle_use->Init.FirstBit=SPI_FIRSTBIT_MSB;                       //大端序
    Handle_use->Init.TIMode=SPI_TIMODE_DISABLE;                       //禁止TI模式，使用摩托罗拉协议
    Handle_use->Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;       //禁止CRC校验
    Handle_use->Init.CRCPolynomial=1;                                 //CRC校验，无效

    printf("spi init res:%d\n",HAL_SPI_Init(Handle_use));
    

    NEUQ_GPIO_init(CS_select,GPIO_Mode_Output_PushPull,GPIO_High);    //初始化片选引脚并拉高

    __HAL_SPI_ENABLE(Handle_use);   // 使能SPI
    if(DEV_select==Hard_SPI_3){
        SPI_1LINE_TX(Handle_use);   // LCD使用单线发送模式
    }
}

//============================================================================
// 函数功能：SPI写入一个字节
// 入口参数：DEV_select:选择使用的SPI设备
//          CS_select:选择CS引脚
//          byte:要写入的字节
// 返回参数：无
// 使用示例：NEUQ_Hard_SPI_write_byte(Hard_SPI_3,PB4,0xFF);
//============================================================================
void NEUQ_Hard_SPI_write_byte(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint8_t byte){
    SPI_HandleTypeDef *Handle_use=NULL;
    switch(DEV_select){
        case Hard_SPI_1:{
                Handle_use=&Hard_SPI_1_Handle;
            break;
        }
        case Hard_SPI_3:{
                Handle_use=&Hard_SPI_3_Handle;
            break;
        }
    }

    if(Handle_use==NULL || Handle_use->Instance==NULL){return;}
    if(DEV_select==Hard_SPI_1){
        (void)NEUQ_Hard_SPI_transfer_byte(DEV_select,byte);
        return;
    }

    #if HARD_SPI_USE_HAL
    NEUQ_GPIO_write(CS_select,GPIO_Low);
    HAL_SPI_Transmit(Handle_use,&byte,1,100);
    NEUQ_GPIO_write(CS_select,GPIO_High);

    #else

    // printf("start write byte\n");
    while (!(Handle_use->Instance->SR & SPI_SR_TXE));    // 等待发送缓冲区空
    // printf("tx fifo empty\n");
    Handle_use->Instance->DR = byte;                     // 写入数据寄存器，开始发送
    // printf("byte written\n");
    while (Handle_use->Instance->SR & SPI_SR_BSY);      // 等待发送完成
    // printf("send complete\n");

    #endif
}

//============================================================================
// 函数功能：SPI写入两个字节
// 入口参数：DEV_select:选择使用的SPI设备
//          CS_select:选择CS引脚
//          bytes:要写入的两个字节
// 返回参数：无
// 使用示例：NEUQ_Hard_SPI_write_2byte(Hard_SPI_3,PB4,0xFFFF);
//============================================================================
void NEUQ_Hard_SPI_write_2byte(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint16_t bytes){
    SPI_HandleTypeDef *Handle_use=NULL;
    volatile uint32_t dummy;
    switch(DEV_select){
        case Hard_SPI_1:{
                Handle_use=&Hard_SPI_1_Handle;
            break;
        }
        case Hard_SPI_3:{
                Handle_use=&Hard_SPI_3_Handle;
            break;
        }
    }

    if(Handle_use==NULL || Handle_use->Instance==NULL){return;}

    #if HARD_SPI_USE_HAL
    NEUQ_GPIO_write(CS_select,GPIO_Low);
    HAL_SPI_Transmit(Handle_use,(uint8_t *)&bytes,1,100);
    NEUQ_GPIO_write(CS_select,GPIO_High);

    #else

    while (!(Handle_use->Instance->SR & SPI_SR_TXE));    // 等待发送缓冲区空
    Handle_use->Instance->DR = bytes;                     // 写入数据寄存器，开始发送
    if(DEV_select==Hard_SPI_1){
        while (!(Handle_use->Instance->SR & SPI_SR_RXNE));
        dummy=Handle_use->Instance->DR;
        (void)dummy;
    }
    while (Handle_use->Instance->SR & SPI_SR_BSY);      // 等待发送完成

    #endif
}

//============================================================================
// 函数功能：SPI写入多字节8位数据
// 入口参数：DEV_select:选择使用的SPI设备
//          CS_select:选择CS引脚
//          data:要写入的数据
//          len:数据长度
// 返回参数：无
// 使用示例：uint8_t data[3]={0x01,0x02,0x03};
//          NEUQ_Hard_SPI_write_8bit_data(Hard_SPI_3,PB4,data,3);
//============================================================================
void NEUQ_Hard_SPI_write_8bit_data(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint8_t *data,uint32_t len){
    SPI_HandleTypeDef *Handle_use=NULL;
    switch(DEV_select){
        case Hard_SPI_1:{
                Handle_use=&Hard_SPI_1_Handle;
            break;
        }
        case Hard_SPI_3:{
                Handle_use=&Hard_SPI_3_Handle;
            break;
        }
    }

    if(Handle_use==NULL || Handle_use->Instance==NULL || data==NULL){return;}
    if(DEV_select==Hard_SPI_1){
        NEUQ_Hard_SPI_transfer(DEV_select,data,NULL,len);
        return;
    }

    #if HARD_SPI_USE_HAL
    NEUQ_GPIO_write(CS_select,GPIO_Low);
    #endif        

    for(int i=0;i<len;i++){
        #if HARD_SPI_USE_HAL

        HAL_SPI_Transmit(Handle_use,data+i,1,100);

        #else

        while (!(Handle_use->Instance->SR & SPI_SR_TXE));    // 等待发送缓冲区空
        Handle_use->Instance->DR = *(data+i);                     // 写入数据寄存器，开始发送
        while (Handle_use->Instance->SR & SPI_SR_BSY);      // 等待发送完成

        #endif        
    }

    #if HARD_SPI_USE_HAL
    NEUQ_GPIO_write(CS_select,GPIO_High);
    #endif     

}

//============================================================================
// 函数功能：SPI写入多字节16位数据
// 入口参数：DEV_select:选择使用的SPI设备
//          CS_select:选择CS引脚
//          data:要写入的数据
//          len:数据长度
// 返回参数：无
// 使用示例：uint16_t data[3]={0x0102,0x0304,0x0506};
//          NEUQ_Hard_SPI_write_16bit_data(Hard_SPI_3,PB4,data,3);
//============================================================================
void NEUQ_Hard_SPI_write_16bit_data(Hard_SPI_DEV_ENUM DEV_select,GPIO_Ports_ENUM CS_select,uint16_t *data,uint32_t len){
    SPI_HandleTypeDef *Handle_use=NULL;
    volatile uint32_t dummy;
    switch(DEV_select){
        case Hard_SPI_1:{
                Handle_use=&Hard_SPI_1_Handle;
            break;
        }
        case Hard_SPI_3:{
                Handle_use=&Hard_SPI_3_Handle;
            break;
        }
    }

    if(Handle_use==NULL || Handle_use->Instance==NULL || data==NULL){return;}

    #if HARD_SPI_USE_HAL
    NEUQ_GPIO_write(CS_select,GPIO_Low);
    #endif        

    for(int i=0;i<len;i++){
        #if HARD_SPI_USE_HAL

        HAL_SPI_Transmit(Handle_use,data+i,1,100);

        #else

        while (!(Handle_use->Instance->SR & SPI_SR_TXE));    // 等待发送缓冲区空
        Handle_use->Instance->DR = *(data+i);                     // 写入数据寄存器，开始发送
        if(DEV_select==Hard_SPI_1){
            while (!(Handle_use->Instance->SR & SPI_SR_RXNE));
            dummy=Handle_use->Instance->DR;
            (void)dummy;
        }
        while (Handle_use->Instance->SR & SPI_SR_BSY);      // 等待发送完成

        #endif        
    }

    #if HARD_SPI_USE_HAL
    NEUQ_GPIO_write(CS_select,GPIO_High);
    #endif     

}

//============================================================================
// 函数功能：SPI收发一个字节，主要供板载Flash使用
// 注意事项：本函数不控制片选，调用者必须在完整事务前后控制CS
//============================================================================
uint8_t NEUQ_Hard_SPI_transfer_byte(Hard_SPI_DEV_ENUM DEV_select,uint8_t tx_data){
    SPI_HandleTypeDef *Handle_use=NEUQ_Hard_SPI_get_handle(DEV_select);
    uint8_t rx_data=0xFF;

    if(Handle_use==NULL || Handle_use->Instance==NULL || DEV_select!=Hard_SPI_1){
        return 0xFF;
    }

    while (!(Handle_use->Instance->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&Handle_use->Instance->DR=tx_data;
    while (!(Handle_use->Instance->SR & SPI_SR_RXNE));
    rx_data=*(__IO uint8_t *)&Handle_use->Instance->DR;
    while (Handle_use->Instance->SR & SPI_SR_BSY);
    return rx_data;
}

//============================================================================
// 函数功能：SPI连续收发，tx_data或rx_data允许为NULL
// 注意事项：本函数不控制片选，调用者必须在完整事务前后控制CS
//============================================================================
void NEUQ_Hard_SPI_transfer(Hard_SPI_DEV_ENUM DEV_select,const uint8_t *tx_data,uint8_t *rx_data,uint32_t len){
    uint32_t i;
    uint8_t rx_byte;

    for(i=0;i<len;i++){
        rx_byte=NEUQ_Hard_SPI_transfer_byte(DEV_select,tx_data==NULL ? 0xFF : tx_data[i]);
        if(rx_data!=NULL){
            rx_data[i]=rx_byte;
        }
    }
}

//============================================================================
// 函数功能：切换到16位数据模式
// 入口参数：DEV_select:选择使用的SPI设备
// 返回参数：无
// 使用示例：NEUQ_Hard_SPI_switch_to_16bit_mode(Hard_SPI_1);
//============================================================================
void NEUQ_Hard_SPI_switch_to_16bit_mode(Hard_SPI_DEV_ENUM DEV_select){
    SPI_HandleTypeDef *Handle_use=NEUQ_Hard_SPI_get_handle(DEV_select);

    if(Handle_use==NULL || Handle_use->Instance==NULL){
        return;
    }

    Handle_use->Instance->CR1 &= ~SPI_CR1_SPE;
    Handle_use->Instance->CR1 |= SPI_CR1_DFF;   // 16位
    Handle_use->Instance->CR1 |= SPI_CR1_SPE;
}

//============================================================================
// 函数功能：切换到8位数据模式
// 入口参数：DEV_select:选择使用的SPI设备
// 返回参数：无
// 使用示例：NEUQ_Hard_SPI_switch_to_8bit_mode(Hard_SPI_1);
//============================================================================
void NEUQ_Hard_SPI_switch_to_8bit_mode(Hard_SPI_DEV_ENUM DEV_select){
    SPI_HandleTypeDef *Handle_use=NEUQ_Hard_SPI_get_handle(DEV_select);

    if(Handle_use==NULL || Handle_use->Instance==NULL){
        return;
    }

    Handle_use->Instance->CR1 &= ~SPI_CR1_SPE;
    Handle_use->Instance->CR1 &= ~SPI_CR1_DFF;  // 8位
    Handle_use->Instance->CR1 |= SPI_CR1_SPE;
}

//============================================================================
// 函数功能：等待SPI空闲
// 入口参数：DEV_select:选择使用的SPI设备
// 返回参数：无
// 使用示例：NEUQ_Hard_SPI_wait_idle(Hard_SPI_1);
//============================================================================
void NEUQ_Hard_SPI_wait_idle(Hard_SPI_DEV_ENUM DEV_select){
    SPI_HandleTypeDef *Handle_use=NEUQ_Hard_SPI_get_handle(DEV_select);

    if(Handle_use==NULL || Handle_use->Instance==NULL){
        return;
    }

    while (Handle_use->Instance->SR & SPI_SR_BSY);      // 等待发送完成
}
