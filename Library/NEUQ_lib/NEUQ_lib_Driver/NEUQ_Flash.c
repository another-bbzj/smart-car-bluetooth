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
* 文件名称          NEUQ_Flash
* 组织名称          东北大学秦皇岛分校 智能车实验室
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK-ARM v5.38
* 适用平台          STM32F407VE
*
* 修改记录
* 日期              作者                备注
* 2026-09-02        沈泉            first version
********************************************************************************************************************/
#include "NEUQ_lib_include.h"

// 板载W25Q固定使用SPI1：PA5-SCK、PA6-MISO、PA7-MOSI、PB0-CS
#define NEUQ_FLASH_SPI                (Hard_SPI_1)
#define NEUQ_FLASH_SCK                (Hard_SPI_1_SCK_PA5)
#define NEUQ_FLASH_MOSI               (Hard_SPI_1_SDA_PA7)
#define NEUQ_FLASH_CS                 (PB0)

#define NEUQ_FLASH_CMD_WRITE_ENABLE   (0x06U)
#define NEUQ_FLASH_CMD_READ_STATUS_1  (0x05U)
#define NEUQ_FLASH_CMD_READ_STATUS_2  (0x35U)
#define NEUQ_FLASH_CMD_WRITE_STATUS_1 (0x01U)
#define NEUQ_FLASH_CMD_WRITE_STATUS_2 (0x31U)
#define NEUQ_FLASH_CMD_READ_DATA      (0x03U)
#define NEUQ_FLASH_CMD_PAGE_PROGRAM   (0x02U)
#define NEUQ_FLASH_CMD_SECTOR_ERASE   (0x20U)
#define NEUQ_FLASH_CMD_JEDEC_ID       (0x9FU)

#define NEUQ_FLASH_STATUS_BUSY        (0x01U)
#define NEUQ_FLASH_STATUS_WEL         (0x02U)
#define NEUQ_FLASH_STATUS1_PROTECT    (0x7CU)
#define NEUQ_FLASH_STATUS2_CMP        (0x40U)
#define NEUQ_FLASH_PROGRAM_TIMEOUT_MS (100U)
#define NEUQ_FLASH_ERASE_TIMEOUT_MS   (3000U)

// 保存初始化时识别出的实际容量；为0表示尚未初始化或芯片型号不受支持
static uint32_t NEUQ_Flash_capacity=0;

// 写入校验和跨扇区保存使用的内部缓冲区，避免占用函数栈空间
static uint8_t NEUQ_Flash_verify_buffer[NEUQ_FLASH_PAGE_SIZE];
static uint8_t NEUQ_Flash_sector_buffer[NEUQ_FLASH_SECTOR_SIZE];

//==============================================================================================
// 函数名称 : NEUQ_Flash_select
// 函数功能 : 拉低Flash片选，开始一次完整的Flash命令事务
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : 无
// 使用示例 : 内部使用，用户无需关注
//==============================================================================================
static void NEUQ_Flash_select(void)
{
    NEUQ_GPIO_write(NEUQ_FLASH_CS,GPIO_Low);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_deselect
// 函数功能 : 拉高Flash片选，结束本次Flash命令事务
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : 无
// 使用示例 : 内部使用，用户无需关注
//==============================================================================================
static void NEUQ_Flash_deselect(void)
{
    NEUQ_GPIO_write(NEUQ_FLASH_CS,GPIO_High);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_send_address
// 函数功能 : 按高字节在前的顺序发送W25Q的24位地址
// 输入参数 : address : 要发送的Flash地址
// 输出参数 : 无
// 返回值   : 无
// 使用示例 : 内部使用，用户无需关注
//==============================================================================================
static void NEUQ_Flash_send_address(uint32_t address)
{
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,(uint8_t)(address>>16));
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,(uint8_t)(address>>8));
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,(uint8_t)address);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_wait_ready
// 函数功能 : 轮询状态寄存器BUSY位，等待Flash完成内部操作
// 输入参数 : timeout_ms : 最大等待时间，单位为毫秒
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示Flash空闲，NEUQ_FLASH_ERROR表示等待超时
// 使用示例 : 内部使用，用户无需关注
//==============================================================================================
static uint8_t NEUQ_Flash_wait_ready(uint32_t timeout_ms)
{
    uint32_t start_tick=HAL_GetTick();

    while((NEUQ_Flash_read_status()&NEUQ_FLASH_STATUS_BUSY)!=0U){
        if((HAL_GetTick()-start_tick)>=timeout_ms){
            return NEUQ_FLASH_ERROR;
        }
    }
    return NEUQ_FLASH_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_write_enable
// 函数功能 : 发送写使能命令，并确认状态寄存器WEL位置1
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示写使能成功，NEUQ_FLASH_ERROR表示失败
// 使用示例 : 内部使用，用户无需关注
//==============================================================================================
static uint8_t NEUQ_Flash_write_enable(void)
{
    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_WRITE_ENABLE);
    NEUQ_Flash_deselect();

    return ((NEUQ_Flash_read_status()&NEUQ_FLASH_STATUS_WEL)!=0U) ? NEUQ_FLASH_OK : NEUQ_FLASH_ERROR;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_write_status1
// 函数功能 : 写入状态寄存器1，用于清除SRP0、SEC、TB和BP2~BP0保护位
// 输入参数 : status : 要写入状态寄存器1的值
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示写入成功，NEUQ_FLASH_ERROR表示失败
// 使用示例 : NEUQ_Flash_write_status1(0x00);
//==============================================================================================
static uint8_t NEUQ_Flash_write_status1(uint8_t status)
{
    if(NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS)==NEUQ_FLASH_ERROR ||
       NEUQ_Flash_write_enable()==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_WRITE_STATUS_1);
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,status);
    NEUQ_Flash_deselect();
    return NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_write_status2
// 函数功能 : 写入状态寄存器2，主要用于清除新版芯片的CMP保护位
// 输入参数 : status : 要写入状态寄存器2的值
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示写入成功，NEUQ_FLASH_ERROR表示失败
// 使用示例 : NEUQ_Flash_write_status2(0x00);
//==============================================================================================
static uint8_t NEUQ_Flash_write_status2(uint8_t status)
{
    if(NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS)==NEUQ_FLASH_ERROR ||
       NEUQ_Flash_write_enable()==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_WRITE_STATUS_2);
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,status);
    NEUQ_Flash_deselect();
    return NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS);
}


//==============================================================================================
// 函数名称 : NEUQ_Flash_read_id
// 函数功能 : 读取Flash的24位JEDEC ID
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : 返回24位JEDEC ID，例如W25Q16为0xEF4015
// 使用示例 : uint32_t id=NEUQ_Flash_read_id();
//==============================================================================================
uint32_t NEUQ_Flash_read_id(void)
{
    uint32_t jedec_id;

    // 0x9F返回厂商号、存储器类型和容量编号，共3字节
    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_JEDEC_ID);
    jedec_id=((uint32_t)NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,0xFFU))<<16;
    jedec_id|=((uint32_t)NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,0xFFU))<<8;
    jedec_id|=NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,0xFFU);
    NEUQ_Flash_deselect();

    return jedec_id;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_get_capacity
// 函数功能 : 获取初始化时识别出的Flash容量
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : Flash容量，单位为字节；未初始化或识别失败时返回0
// 使用示例 : uint32_t size=NEUQ_Flash_get_capacity();
//==============================================================================================
uint32_t NEUQ_Flash_get_capacity(void)
{
    return NEUQ_Flash_capacity;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_get_sector_count
// 函数功能 : 获取当前Flash的扇区总数
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : 扇区总数；W25Q16返回512，W25Q32返回1024，未初始化时返回0
// 使用示例 : uint16_t count=NEUQ_Flash_get_sector_count();
//==============================================================================================
uint16_t NEUQ_Flash_get_sector_count(void)
{
    if(NEUQ_Flash_capacity==0U){
        return 0U;
    }
    return (uint16_t)(NEUQ_Flash_capacity/NEUQ_FLASH_SECTOR_SIZE);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_get_sector_address
// 函数功能 : 根据扇区编号计算该扇区的起始地址
// 输入参数 : sector_id : 扇区编号，W25Q16有效范围为0~511
// 输出参数 : 无
// 返回值   : 扇区起始地址；扇区编号无效或Flash未初始化时返回NEUQ_FLASH_INVALID_ADDRESS
// 使用示例 : uint32_t address=NEUQ_Flash_get_sector_address(511);
//==============================================================================================
uint32_t NEUQ_Flash_get_sector_address(uint16_t sector_id)
{
    if(NEUQ_Flash_capacity==0U || sector_id>=NEUQ_Flash_get_sector_count()){
        return NEUQ_FLASH_INVALID_ADDRESS;
    }
    return ((uint32_t)sector_id)*NEUQ_FLASH_SECTOR_SIZE;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_read_status
// 函数功能 : 读取Flash状态寄存器1
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : 状态寄存器1的8位原始值
// 使用示例 : uint8_t status=NEUQ_Flash_read_status();
//==============================================================================================
uint8_t NEUQ_Flash_read_status(void)
{
    uint8_t status;

    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_READ_STATUS_1);
    status=NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,0xFFU);
    NEUQ_Flash_deselect();
    return status;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_read_status2
// 函数功能 : 读取Flash状态寄存器2
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : 状态寄存器2的8位原始值
// 使用示例 : uint8_t status2=NEUQ_Flash_read_status2();
//==============================================================================================
uint8_t NEUQ_Flash_read_status2(void)
{
    uint8_t status;

    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_READ_STATUS_2);
    status=NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,0xFFU);
    NEUQ_Flash_deselect();
    return status;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_clear_protection
// 函数功能 : 清除Flash的区域保护位和CMP保护范围反转位
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示解锁成功，NEUQ_FLASH_ERROR表示解锁失败
// 使用示例 : NEUQ_Flash_clear_protection();
//==============================================================================================
uint8_t NEUQ_Flash_clear_protection(void)
{
    uint8_t status1;
    uint8_t status2;

    if(NEUQ_Flash_capacity==0U ||
       NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS)==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    status1=NEUQ_Flash_read_status();
    status2=NEUQ_Flash_read_status2();

    // 已经没有区域保护时不写状态寄存器，避免不必要的非易失性写入
    if((status1&NEUQ_FLASH_STATUS1_PROTECT)==0U &&
       (status2&NEUQ_FLASH_STATUS2_CMP)==0U){
        return NEUQ_FLASH_OK;
    }

    if(NEUQ_Flash_write_status1(0x00U)==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    // 老版本W25Q用0x01写一个字节时会同时清除SR2；新版仍有CMP时再使用0x31
    status2=NEUQ_Flash_read_status2();
    if((status2&NEUQ_FLASH_STATUS2_CMP)!=0U &&
       NEUQ_Flash_write_status2((uint8_t)(status2&(~NEUQ_FLASH_STATUS2_CMP)))==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    status1=NEUQ_Flash_read_status();
    status2=NEUQ_Flash_read_status2();
    return ((status1&NEUQ_FLASH_STATUS1_PROTECT)==0U &&
            (status2&NEUQ_FLASH_STATUS2_CMP)==0U) ? NEUQ_FLASH_OK : NEUQ_FLASH_ERROR;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_read
// 函数功能 : 从指定地址连续读取Flash数据
// 输入参数 :  address : 起始地址
//            buffer  : 接收数据的缓冲区
//            length  : 读取字节数
// 输出参数 : buffer中写入读取到的数据
// 返回值   : NEUQ_FLASH_OK表示读取成功，NEUQ_FLASH_ERROR表示参数错误或等待超时
// 使用示例 : uint8_t data[16]; NEUQ_Flash_read(0x001000,data,16);
//==============================================================================================
uint8_t NEUQ_Flash_read(uint32_t address,uint8_t *buffer,uint32_t length)
{
    // 同时检查初始化状态、空指针和访问范围，避免地址加法溢出
    if(buffer==NULL || length==0U || NEUQ_Flash_capacity==0U ||
       address>=NEUQ_Flash_capacity || length>(NEUQ_Flash_capacity-address)){
        return NEUQ_FLASH_ERROR;
    }
    if(NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS)==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_READ_DATA);
    NEUQ_Flash_send_address(address);
    NEUQ_Hard_SPI_transfer(NEUQ_FLASH_SPI,NULL,buffer,length);
    NEUQ_Flash_deselect();
    return NEUQ_FLASH_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_page_program
// 函数功能 : 向当前页写入数据，单次写入不能跨越256字节页边界
// 输入参数 : address : 写入起始地址
//            buffer  : 待写入数据的缓冲区
//            length  : 写入字节数，范围为1到256
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示写入成功，NEUQ_FLASH_ERROR表示参数错误、未擦除或写入失败
// 使用示例 : uint8_t data[4]={1,2,3,4}; NEUQ_Flash_page_program(0x001000,data,4);
//==============================================================================================
uint8_t NEUQ_Flash_page_program(uint32_t address,const uint8_t *buffer,uint16_t length)
{
    uint16_t page_offset=(uint16_t)(address&(NEUQ_FLASH_PAGE_SIZE-1U));

    if(buffer==NULL || length==0U || length>NEUQ_FLASH_PAGE_SIZE ||
       ((uint32_t)page_offset+length)>NEUQ_FLASH_PAGE_SIZE ||
       NEUQ_Flash_capacity==0U || address>=NEUQ_Flash_capacity ||
       length>(NEUQ_Flash_capacity-address)){
        return NEUQ_FLASH_ERROR;
    }
    if(NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS)==NEUQ_FLASH_ERROR ||
       NEUQ_Flash_write_enable()==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    // 写使能后发送页编程命令、24位地址和待写数据
    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_PAGE_PROGRAM);
    NEUQ_Flash_send_address(address);
    NEUQ_Hard_SPI_transfer(NEUQ_FLASH_SPI,buffer,NULL,length);
    NEUQ_Flash_deselect();

    return NEUQ_Flash_wait_ready(NEUQ_FLASH_PROGRAM_TIMEOUT_MS);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_write
// 函数功能 : 连续写入Flash数据，并自动按页边界拆分写入
// 输入参数 : address : 写入起始地址
//            buffer  : 待写入数据的缓冲区
//            length  : 写入字节数
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示写入成功，NEUQ_FLASH_ERROR表示参数错误或写入失败
// 使用示例 : uint8_t data[300]; NEUQ_Flash_write(0x001000,data,300);
//==============================================================================================
uint8_t NEUQ_Flash_write(uint32_t address,const uint8_t *buffer,uint32_t length)
{
    uint32_t current_address=address;
    uint32_t remaining=length;
    uint16_t write_length;

    if(buffer==NULL || length==0U || NEUQ_Flash_capacity==0U ||
       address>=NEUQ_Flash_capacity || length>(NEUQ_Flash_capacity-address)){
        return NEUQ_FLASH_ERROR;
    }

    while(remaining>0U){
        // 本次最多写到当前页末尾，下一轮再继续写下一页
        write_length=(uint16_t)(NEUQ_FLASH_PAGE_SIZE-(current_address&(NEUQ_FLASH_PAGE_SIZE-1U)));
        if(write_length>remaining){
            write_length=(uint16_t)remaining;
        }
        if(NEUQ_Flash_page_program(current_address,buffer,write_length)==NEUQ_FLASH_ERROR){
            return NEUQ_FLASH_ERROR;
        }
        // 每页写入完成后立即回读，确保写入内容与源数据一致
        if(NEUQ_Flash_read(current_address,NEUQ_Flash_verify_buffer,write_length)==NEUQ_FLASH_ERROR ||
           memcmp(NEUQ_Flash_verify_buffer,buffer,write_length)!=0){
            return NEUQ_FLASH_ERROR;
        }
        current_address+=write_length;
        buffer+=write_length;
        remaining-=write_length;
    }
    return NEUQ_FLASH_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_save
// 函数功能 : 安全保存数据，保留目标扇区内其他数据并自动完成擦除、写入和校验
// 输入参数 : address : 数据保存起始地址
//            buffer  : 待保存数据的缓冲区
//            length  : 保存字节数，可以跨越页和扇区
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示保存成功，NEUQ_FLASH_ERROR表示参数错误或保存失败
// 使用示例 : uint32_t value=123; NEUQ_Flash_save(0x1FF000,(uint8_t *)&value,4);
//==============================================================================================
uint8_t NEUQ_Flash_save(uint32_t address,const uint8_t *buffer,uint32_t length)
{
    uint32_t current_address=address;
    uint32_t remaining=length;
    uint32_t sector_start;
    uint32_t sector_offset;
    uint32_t save_length;

    if(buffer==NULL || length==0U || NEUQ_Flash_capacity==0U ||
       address>=NEUQ_Flash_capacity || length>(NEUQ_Flash_capacity-address)){
        return NEUQ_FLASH_ERROR;
    }

    while(remaining>0U){
        // 计算当前数据所在扇区，并确定本轮最多保存到该扇区末尾
        sector_start=current_address&~(NEUQ_FLASH_SECTOR_SIZE-1UL);
        sector_offset=current_address-sector_start;
        save_length=NEUQ_FLASH_SECTOR_SIZE-sector_offset;
        if(save_length>remaining){
            save_length=remaining;
        }

        // 先完整读出扇区，避免擦除时丢失同一扇区内的其他数据
        if(NEUQ_Flash_read(sector_start,NEUQ_Flash_sector_buffer,NEUQ_FLASH_SECTOR_SIZE)==NEUQ_FLASH_ERROR){
            return NEUQ_FLASH_ERROR;
        }
        memcpy(NEUQ_Flash_sector_buffer+sector_offset,buffer,save_length);

        // 擦除整个扇区后重写，NEUQ_Flash_write()会对每一页进行回读校验
        if(NEUQ_Flash_sector_erase(sector_start)==NEUQ_FLASH_ERROR ||
           NEUQ_Flash_write(sector_start,NEUQ_Flash_sector_buffer,NEUQ_FLASH_SECTOR_SIZE)==NEUQ_FLASH_ERROR){
            return NEUQ_FLASH_ERROR;
        }

        current_address+=save_length;
        buffer+=save_length;
        remaining-=save_length;
    }

    return NEUQ_FLASH_OK;
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_sector_erase
// 函数功能 : 擦除指定地址所在的4 KiB扇区
// 输入参数 : address : 扇区内任意地址，函数会自动对齐到扇区起始地址
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示擦除成功，NEUQ_FLASH_ERROR表示参数错误或擦除失败
// 使用示例 : NEUQ_Flash_sector_erase(0x001000);
//==============================================================================================
uint8_t NEUQ_Flash_sector_erase(uint32_t address)
{
    if(NEUQ_Flash_capacity==0U || address>=NEUQ_Flash_capacity){
        return NEUQ_FLASH_ERROR;
    }
    if(NEUQ_Flash_wait_ready(NEUQ_FLASH_ERASE_TIMEOUT_MS)==NEUQ_FLASH_ERROR ||
       NEUQ_Flash_write_enable()==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }

    // 对齐到address所在扇区的起始地址，再擦除完整4 KiB扇区
    address&=~(NEUQ_FLASH_SECTOR_SIZE-1UL);
    NEUQ_Flash_select();
    NEUQ_Hard_SPI_transfer_byte(NEUQ_FLASH_SPI,NEUQ_FLASH_CMD_SECTOR_ERASE);
    NEUQ_Flash_send_address(address);
    NEUQ_Flash_deselect();

    return NEUQ_Flash_wait_ready(NEUQ_FLASH_ERASE_TIMEOUT_MS);
}


//==============================================================================================
// 函数名称 : NEUQ_Flash_init
// 函数功能 : 初始化板载Flash，并通过JEDEC ID识别芯片容量
// 输入参数 : 无
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示初始化成功，NEUQ_FLASH_ERROR表示芯片型号不支持或通信失败
// 使用示例 : NEUQ_Flash_init();
//==============================================================================================
uint8_t NEUQ_Flash_init(void)
{
    uint32_t jedec_id;

    NEUQ_Hard_SPI_init(NEUQ_FLASH_SPI,NEUQ_FLASH_SCK,NEUQ_FLASH_MOSI,NEUQ_FLASH_CS);
    NEUQ_Flash_deselect();
    jedec_id=NEUQ_Flash_read_id();

    if(jedec_id==NEUQ_FLASH_JEDEC_ID_W25Q16){
        NEUQ_Flash_capacity=NEUQ_FLASH_W25Q16_SIZE;
    }
    else if(jedec_id==NEUQ_FLASH_JEDEC_ID_W25Q32){
        NEUQ_Flash_capacity=NEUQ_FLASH_W25Q32_SIZE;
    }
    else{
        NEUQ_Flash_capacity=0;
        return NEUQ_FLASH_ERROR;
    }

    // 初始化阶段自动解除保护并再次读取状态确认，应用层无需单独调用解锁函数
    return (NEUQ_Flash_clear_protection()==NEUQ_FLASH_OK) ?
           NEUQ_FLASH_OK : NEUQ_FLASH_ERROR;
}


//==============================================================================================
// 函数名称 : NEUQ_Flash_read_sector
// 函数功能 : 从指定扇区的第0字节开始读取数据
// 输入参数 : sector_id : 扇区编号，W25Q16有效范围为0~511
//            buffer    : 接收数据的缓冲区
//            length    : 读取字节数，最大不能超过一个扇区大小4096字节
// 输出参数 : buffer中写入读取到的数据
// 返回值   : NEUQ_FLASH_OK表示成功，NEUQ_FLASH_ERROR表示失败
// 使用示例 : uint32_t value; NEUQ_Flash_read_sector(511,(uint8_t *)&value,4);
//==============================================================================================
uint8_t NEUQ_Flash_read_sector(uint16_t sector_id,uint8_t *buffer,uint16_t length)
{
    uint32_t address=NEUQ_Flash_get_sector_address(sector_id);

    if(address==NEUQ_FLASH_INVALID_ADDRESS || length==0U || length>NEUQ_FLASH_SECTOR_SIZE){
        return NEUQ_FLASH_ERROR;
    }
    return NEUQ_Flash_read(address,buffer,length);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_write_sector
// 函数功能 : 擦除指定扇区后，从该扇区第0字节开始写入数据并回读校验
// 输入参数 : sector_id : 扇区编号，W25Q16有效范围为0~511
//            buffer    : 待写入数据的缓冲区
//            length    : 写入字节数，最大不能超过一个扇区大小4096字节
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示写入并校验成功，NEUQ_FLASH_ERROR表示失败
// 使用示例 : uint32_t value=123; NEUQ_Flash_write_sector(511,(uint8_t *)&value,4);
//==============================================================================================
uint8_t NEUQ_Flash_write_sector(uint16_t sector_id,const uint8_t *buffer,uint16_t length)
{
    uint32_t address=NEUQ_Flash_get_sector_address(sector_id);

    if(address==NEUQ_FLASH_INVALID_ADDRESS || buffer==NULL ||
       length==0U || length>NEUQ_FLASH_SECTOR_SIZE){
        return NEUQ_FLASH_ERROR;
    }

    // 扇区写入接口默认先擦除整个扇区，再从扇区起始地址写入用户数组
    if(NEUQ_Flash_sector_erase(address)==NEUQ_FLASH_ERROR){
        return NEUQ_FLASH_ERROR;
    }
    return NEUQ_Flash_write(address,buffer,length);
}

//==============================================================================================
// 函数名称 : NEUQ_Flash_erase_sector
// 函数功能 : 按扇区编号擦除一个完整4 KiB扇区
// 输入参数 : sector_id : 扇区编号，W25Q16有效范围为0~511
// 输出参数 : 无
// 返回值   : NEUQ_FLASH_OK表示成功，NEUQ_FLASH_ERROR表示失败
// 使用示例 : NEUQ_Flash_erase_sector(511);
//==============================================================================================
uint8_t NEUQ_Flash_erase_sector(uint16_t sector_id)
{
    uint32_t address=NEUQ_Flash_get_sector_address(sector_id);

    if(address==NEUQ_FLASH_INVALID_ADDRESS){
        return NEUQ_FLASH_ERROR;
    }
    return NEUQ_Flash_sector_erase(address);
}

