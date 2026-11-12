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
* 文件名称          NEUQ_Hard_IIC
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

I2C_HandleTypeDef Hard_IIC_1_Handle={0};
I2C_HandleTypeDef Hard_IIC_2_Handle={0};
I2C_HandleTypeDef Hard_IIC_3_Handle={0};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     硬件IIC 接口初始化 
// 参数说明     DEV_select        硬件IIC 外设号
// 参数说明     Speed             硬件IIC 总线速率，单位Hz，最大400KHz
// 返回参数     void
// 使用示例     NEUQ_Hard_IIC_init(Hard_IIC_1, 200000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NEUQ_Hard_IIC_init(Hard_IIC_DEV_ENUM DEV_select,uint32_t Speed){
    I2C_HandleTypeDef *Handle_use=NULL;
    switch(DEV_select){
        case Hard_IIC_1:{
            __HAL_RCC_I2C1_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_6;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_NOPULL;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF4_I2C1;
            HAL_GPIO_Init(GPIOB,&GPIO_initure);
            GPIO_initure.Pin=GPIO_PIN_7;
            GPIO_initure.Mode=GPIO_MODE_AF_OD;
            HAL_GPIO_Init(GPIOB,&GPIO_initure);

            Handle_use=&Hard_IIC_1_Handle;
            Hard_IIC_1_Handle.Instance=I2C1;

            break;
        }

        case Hard_IIC_2:{
            __HAL_RCC_I2C2_CLK_ENABLE();
            __HAL_RCC_GPIOB_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_10;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_NOPULL;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF4_I2C1;
            HAL_GPIO_Init(GPIOB,&GPIO_initure);
            GPIO_initure.Pin=GPIO_PIN_11;
            GPIO_initure.Mode=GPIO_MODE_AF_OD;
            HAL_GPIO_Init(GPIOB,&GPIO_initure);

            Handle_use=&Hard_IIC_2_Handle;
            Hard_IIC_2_Handle.Instance=I2C2;

            break;
        }

        case Hard_IIC_3:{
            __HAL_RCC_I2C3_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            __HAL_RCC_GPIOC_CLK_ENABLE();
            GPIO_InitTypeDef GPIO_initure={0};

            GPIO_initure.Pin=GPIO_PIN_8;
            GPIO_initure.Mode=GPIO_MODE_AF_PP;
            GPIO_initure.Pull=GPIO_PULLUP;
            GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
            GPIO_initure.Alternate=GPIO_AF4_I2C1;
            HAL_GPIO_Init(GPIOA,&GPIO_initure);
            GPIO_initure.Pin=GPIO_PIN_9;
            GPIO_initure.Mode=GPIO_MODE_AF_OD;
            HAL_GPIO_Init(GPIOC,&GPIO_initure);

            Handle_use=&Hard_IIC_3_Handle;
            Hard_IIC_3_Handle.Instance=I2C3;

            break;
        }
    }

    if(Speed>400000){Speed=400000;}

    Handle_use->Init.ClockSpeed=Speed;
    Handle_use->Init.DutyCycle=I2C_DUTYCYCLE_16_9;
    Handle_use->Init.OwnAddress1=0;
    Handle_use->Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    Handle_use->Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    Handle_use->Init.OwnAddress2=0;
    Handle_use->Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    Handle_use->Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;

    printf("iic init res:%d\n",HAL_I2C_Init(Handle_use));
    

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     硬件 IIC 接口从传感器寄存器读取 8bit 数据
// 参数说明     DEV_select      硬件IIC 外设号
// 参数说明     dev_addr        I2C 从机设备地址
// 参数说明     reg             寄存器地址 (8 位)
// 参数说明     Size            要读取的字节数
// 参数说明     uint8_t         函数执行结果
// 参数说明                     - 1 : 读取成功
// 参数说明                     - 0 : 读取失败（会通过 printf 打印错误码）
// 使用示例     NEUQ_Hard_IIC_read_8bit_reg(Hard_IIC_1, (0x68 << 1), 0x75, &data, 1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t NEUQ_Hard_IIC_read_8bit_reg(Hard_IIC_DEV_ENUM DEV_select,uint8_t dev_addr,uint8_t reg,uint8_t *pData, uint16_t Size){
    I2C_HandleTypeDef *Handle_use=NULL;
    switch(DEV_select){
        case Hard_IIC_1:{
            Handle_use=&Hard_IIC_1_Handle;
            break;
        }
        case Hard_IIC_2:{
            Handle_use=&Hard_IIC_2_Handle;
            break;
        }
        case Hard_IIC_3:{
            Handle_use=&Hard_IIC_3_Handle;
            break;
        }
    }
    
    uint8_t res=HAL_I2C_Mem_Read(Handle_use,dev_addr,reg,I2C_MEMADD_SIZE_8BIT,pData,Size,100);

    if(res==HAL_OK){
        return 1;
    }
    else{
        printf("read err:%d\n",res);
        return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     硬件 IIC 接口向传感器寄存器写入 8bit 数据
// 参数说明     DEV_select      硬件IIC 外设号
// 参数说明     dev_addr        I2C 从机设备地址
// 参数说明     reg             寄存器地址 (8 位)
// 参数说明     Data            要写入的数据
// 参数说明     uint8_t         函数执行结果
// 参数说明                     - 1 : 写入成功
// 参数说明                     - 0 : 写入失败（会通过 printf 打印错误码）
// 使用示例     NEUQ_Hard_IIC_write_8bit_reg(Hard_IIC_1, (0x68 << 1), 0x75, 0x00);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t NEUQ_Hard_IIC_write_8bit_reg(Hard_IIC_DEV_ENUM DEV_select,uint8_t dev_addr,uint8_t reg,uint8_t Data){
    I2C_HandleTypeDef *Handle_use=NULL;
    switch(DEV_select){
        case Hard_IIC_1:{
            Handle_use=&Hard_IIC_1_Handle;
            break;
        }
        case Hard_IIC_2:{
            Handle_use=&Hard_IIC_2_Handle;
            break;
        }
        case Hard_IIC_3:{
            Handle_use=&Hard_IIC_3_Handle;
            break;
        }
    }
    
    uint8_t res=HAL_I2C_Mem_Write(Handle_use,dev_addr,reg,I2C_MEMADD_SIZE_8BIT,&Data,1,100);

    if(res==HAL_OK){
        return 1;
    }
    else{
        printf("write err:%d\n",res);
        return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     I2C 扫描函数，扫描总线上所有可能的设备地址，并打印结果
// 参数说明     hi2c            I2C 外设句柄指针
// 返回参数     void
// 使用示例     I2C_Scan(&hi2c1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    printf("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 10) == HAL_OK) {
            printf("I2C device found at address 0x%02X\r\n", addr);
        }
    }
    printf("scan done\n");
}
