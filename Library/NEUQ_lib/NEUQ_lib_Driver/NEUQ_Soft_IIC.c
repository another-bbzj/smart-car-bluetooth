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
* 文件名称          NEUQ_Soft_IIC
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






/**************************************软件模拟IIC,兼容SCCB**********************************************************/
/**********************************以下是用于配置摄像头的SCCB驱动*****************************************************/
/**********************************以下是用于配置摄像头的SCCB驱动*****************************************************/
/**********************************以下是用于配置摄像头的SCCB驱动*****************************************************/

Soft_IIC_Handle_Type *Soft_IIC_Handle_g={0};
void NEUQ_SCCB_init(Soft_IIC_Handle_Type *Soft_IIC_Handle){
    Soft_IIC_Handle_g=Soft_IIC_Handle;
    NEUQ_GPIO_init(Soft_IIC_Handle->SCL,GPIO_Mode_Output_PushPull,GPIO_Low);
    NEUQ_GPIO_init(Soft_IIC_Handle->SDA,GPIO_Mode_Output_OpenDrain,GPIO_Low);
}

#define SCCB_SCL(a)	if (a){NEUQ_GPIO_write(Soft_IIC_Handle_g->SCL,GPIO_High);}else{NEUQ_GPIO_write(Soft_IIC_Handle_g->SCL,GPIO_Low);}
#define SCCB_SDA(a)	if (a){NEUQ_GPIO_write(Soft_IIC_Handle_g->SDA,GPIO_High);}else{NEUQ_GPIO_write(Soft_IIC_Handle_g->SDA,GPIO_Low);}
#define SCCB_Delay(i) NEUQ_Delay_us(i)
#define SCCB_DelayVaule (10)

#define ACK_OK  	1  			// 响应正常
#define ACK_ERR 	0				// 响应错误

#define OV2640_DEVICE_ADDRESS     0x60    // OV2640地址
#define OV5640_DEVICE_ADDRESS     0X78		// OV5640地址
/*****************************************************************************************
*	函 数 名: SCCB_Start
*	入口参数: 无
*	返 回 值: 无
*	函数功能: IIC起始信号
*	说    明: 在SCL处于高电平期间，SDA由高到低跳变为起始信号
******************************************************************************************/

void SCCB_Start(void)
{
	SCCB_SDA(1);		
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SDA(0);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
}

/*****************************************************************************************
*	函 数 名: SCCB_Stop
*	入口参数: 无
*	返 回 值: 无
*	函数功能: IIC停止信号
*	说    明: 在SCL处于高电平期间，SDA由低到高跳变为起始信号
******************************************************************************************/

void SCCB_Stop(void)
{
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(0);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(1);
	SCCB_Delay(SCCB_DelayVaule);
}

/*****************************************************************************************
*	函 数 名: SCCB_ACK
*	入口参数: 无
*	返 回 值: 无
*	函数功能: IIC应答信号
*	说    明: 在SCL为高电平期间，SDA引脚输出为低电平，产生应答信号
******************************************************************************************/

void SCCB_ACK(void)
{
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(0);
	SCCB_Delay(SCCB_DelayVaule);	
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SCL(0);		// SCL输出低时，SDA应立即拉高，释放总线
	SCCB_SDA(1);		
	
	SCCB_Delay(SCCB_DelayVaule);

}

/*****************************************************************************************
*	函 数 名: SCCB_NoACK
*	入口参数: 无
*	返 回 值: 无
*	函数功能: IIC非应答信号
*	说    明: 在SCL为高电平期间，若SDA引脚为高电平，产生非应答信号
******************************************************************************************/

void SCCB_NoACK(void)
{
	SCCB_SCL(0);	
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SDA(1);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);
	
	SCCB_SCL(0);
	SCCB_Delay(SCCB_DelayVaule);
}

/*****************************************************************************************
*	函 数 名: SCCB_WaitACK
*	入口参数: 无
*	返 回 值: 无
*	函数功能: 等待接收设备发出应答信号
*	说    明: 在SCL为高电平期间，若检测到SDA引脚为低电平，则接收设备响应正常
******************************************************************************************/

uint8_t SCCB_WaitACK(void)
{
	SCCB_SDA(1);
	SCCB_Delay(SCCB_DelayVaule);
	SCCB_SCL(1);
	SCCB_Delay(SCCB_DelayVaule);	
	
	if(NEUQ_GPIO_read(Soft_IIC_Handle_g->SDA) != 0) //判断设备是否有做出响应		
	{
		SCCB_SCL(0);	
		SCCB_Delay( SCCB_DelayVaule );		
		return ACK_ERR;	//无应答
	}
	else
	{
		SCCB_SCL(0);	
		SCCB_Delay( SCCB_DelayVaule );		
		return ACK_OK;	//应答正常
	}
}

/*****************************************************************************************
*	函 数 名:	SCCB_WriteByte
*	入口参数:	IIC_Data - 要写入的8位数据
*	返 回 值:	ACK_OK  - 设备响应正常
*          	   ACK_ERR - 设备响应错误
*	函数功能:	写一字节数据
*	说    明:   高位在前
******************************************************************************************/

uint8_t SCCB_WriteByte(uint8_t IIC_Data)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		SCCB_SDA(IIC_Data & 0x80);
		
		SCCB_Delay( SCCB_DelayVaule );
		SCCB_SCL(1);
		SCCB_Delay( SCCB_DelayVaule );
		SCCB_SCL(0);		
		if(i == 7)
		{
			SCCB_SDA(1);			
		}
		IIC_Data <<= 1;
	}

	return SCCB_WaitACK(); //等待设备响应
}

/*****************************************************************************************
*	函 数 名:	SCCB_ReadByte
*	入口参数:	ACK_Mode - 响应模式，输入1则发出应答信号，输入0发出非应答信号
*	返 回 值:	ACK_OK  - 设备响应正常
*          	   ACK_ERR - 设备响应错误
*	函数功能:   读一字节数据
*	说    明:   1.高位在前
*				   2.应在主机接收最后一字节数据时发送非应答信号
******************************************************************************************/

uint8_t SCCB_ReadByte(uint8_t ACK_Mode)
{
	uint8_t IIC_Data = 0;
	uint8_t i = 0;
	
	for (i = 0; i < 8; i++)
	{
		IIC_Data <<= 1;
		
		SCCB_SCL(1);
		SCCB_Delay( SCCB_DelayVaule );
		IIC_Data |= (NEUQ_GPIO_read(Soft_IIC_Handle_g->SDA) & 0x01);	
		SCCB_SCL(0);
		SCCB_Delay( SCCB_DelayVaule );
	}
	
	if ( ACK_Mode == 1 )				//	应答信号
		SCCB_ACK();
	else
		SCCB_NoACK();		 	// 非应答信号
	
	return IIC_Data; 
}


/*************************************************************************************************************************************
*	函 数 名:	SCCB_WriteHandle
*
*	入口参数:	addr - 要进行操作的寄存器(8位地址)
*
*	返 回 值:	SUCCESS - 操作成功，ERROR	  - 操作失败
*					
*	函数功能:	对指定的寄存器(8位地址)执行写操作，OV2640用到
************************************************************************************************************************************/

uint8_t SCCB_WriteHandle (uint8_t addr)
{
	uint8_t status;		// 状态标志位

	SCCB_Start();	// 启动IIC通信
	if( SCCB_WriteByte(OV2640_DEVICE_ADDRESS) == ACK_OK ) //写数据指令
	{
		if( SCCB_WriteByte((uint8_t)(addr)) != ACK_OK )
		{
			status = ERROR;	// 操作失败
		}			
	}
	status = SUCCESS;	// 操作成功
	return status;	
}

/*************************************************************************************************************************************
*	函 数 名:	SCCB_WriteReg
*
*	入口参数:	addr - 要写入的寄存器(8位地址)，value - 要写入的数据
*					
*	返 回 值:	SUCCESS - 操作成功， ERROR	  - 操作失败
*					
*	函数功能:	对指定的寄存器(8位地址)写一字节数据，OV2640用到
************************************************************************************************************************************/

uint8_t SCCB_WriteReg (uint8_t addr,uint8_t value)
{
	uint8_t status;
	
	SCCB_Start(); //启动IIC通讯

	if( SCCB_WriteHandle(addr) == SUCCESS)	//写入要操作的寄存器
	{
		if (SCCB_WriteByte(value) != ACK_OK) //写数据
		{
			status = ERROR;						
		}
	}	
	SCCB_Stop(); // 停止通讯
	
	status = SUCCESS;	// 写入成功
	return status;
}
/*************************************************************************************************************************************
*	函 数 名:	SCCB_ReadReg
*
*	入口参数:	addr - 要读取的寄存器(8位地址)
*					
*	返 回 值:	读到的数据
*					
*	函数功能:	对指定的寄存器(8位地址)读取一字节数据，OV2640用到
************************************************************************************************************************************/

uint8_t SCCB_ReadReg (uint8_t addr)
{
   uint8_t value = 0;

	SCCB_Start();		// 启动IIC通信

	if( SCCB_WriteHandle(addr) == SUCCESS) //写入要操作的寄存器
	{
      SCCB_Stop();	// 停止IIC通信
		SCCB_Start(); //重新启动IIC通讯

		if (SCCB_WriteByte(OV2640_DEVICE_ADDRESS|0X01) == ACK_OK)	// 发送读命令
		{	
			value = SCCB_ReadByte(0);	// 读到最后一个数据时发送 非应答信号
		}					
		SCCB_Stop();	// 停止IIC通信

	}

	return value;	
}

/*************************************************************************************************************************************
*	函 数 名:	SCCB_WriteHandle_16Bit
*
*	入口参数:	addr - 要进行操作的寄存器(16位地址)
*
*	返 回 值:	SUCCESS - 操作成功，ERROR - 操作失败
*					
*	函数功能:	对指定的寄存器(16位地址)执行写操作，OV5640用到
************************************************************************************************************************************/

uint8_t SCCB_WriteHandle_16Bit (uint16_t addr)
{
	uint8_t status;		// 状态标志位

	SCCB_Start();	// 启动IIC通信
	if( SCCB_WriteByte(OV5640_DEVICE_ADDRESS) == ACK_OK ) //写数据指令
	{
		if( SCCB_WriteByte((uint8_t)(addr >> 8)) == ACK_OK ) //写入16位地址
		{
			if( SCCB_WriteByte((uint8_t)(addr)) != ACK_OK )
			{
				status = ERROR;	// 操作失败
			}			
		}		
	}
	status = SUCCESS;	// 操作成功
	return status;	
}

/*************************************************************************************************************************************
*	函 数 名:	SCCB_WriteReg_16Bit
*
*	入口参数:	addr - 要写入的寄存器(16位地址)  value - 要写入的数据
*					
*	返 回 值:	SUCCESS - 操作成功，ERROR	  - 操作失败
*					
*	函数功能:	对指定的寄存器(16位地址)写一字节数据，OV5640用到
************************************************************************************************************************************/

uint8_t SCCB_WriteReg_16Bit(uint16_t addr,uint8_t value)
{
	uint8_t status;
	
	SCCB_Start(); //启动IIC通讯

	if( SCCB_WriteHandle_16Bit(addr) == SUCCESS)	//写入要操作的寄存器
	{
		if (SCCB_WriteByte(value) != ACK_OK) //写数据
		{
			status = ERROR;						
		}
	}	
	SCCB_Stop(); // 停止通讯
	
	status = SUCCESS;	// 写入成功
	return status;
}

/*************************************************************************************************************************************
*	函 数 名:	SCCB_ReadReg_16Bit
*
*	入口参数:	addr - 要读取的寄存器(16位地址)
*					
*	返 回 值:	读到的数据
*					
*	函数功能:	对指定的寄存器(16位地址)读取一字节数据，OV5640用到
************************************************************************************************************************************/

uint8_t SCCB_ReadReg_16Bit (uint16_t addr)
{
   uint8_t value = 0;

	SCCB_Start();		// 启动IIC通信

	if( SCCB_WriteHandle_16Bit(addr) == SUCCESS) //写入要操作的寄存器
	{
      SCCB_Stop();	// 停止IIC通信
		SCCB_Start(); //重新启动IIC通讯

		if (SCCB_WriteByte(OV5640_DEVICE_ADDRESS|0X01) == ACK_OK)	// 发送读命令
		{	
			value = SCCB_ReadByte(0);	// 读到最后一个数据时发送 非应答信号
		}					
		SCCB_Stop();	// 停止IIC通信

	}

	return value;	
}

/*************************************************************************************************************************************
*	函 数 名:	SCCB_WriteBuffer_16Bit
*
*	入口参数:	addr - 要写入的寄存器(16位地址)  *pData - 数据区   size - 要传输数据的大小
*					
*	返 回 值:	SUCCESS - 操作成功，ERROR	  - 操作失败
*					
*	函数功能:	对指定的寄存器(16位地址)批量写数据，OV5640 写入自动对焦固件时用到
************************************************************************************************************************************/

uint8_t SCCB_WriteBuffer_16Bit(uint16_t addr,uint8_t *pData, uint32_t size)
{
	uint8_t status;	
	uint32_t i;
	
	SCCB_Start(); //启动IIC通讯

	if( SCCB_WriteHandle_16Bit(addr) == SUCCESS)	//写入要操作的寄存器
	{
		for(i=0;i<size;i++)
		{
			SCCB_WriteByte(*pData);//写数据			
			pData++;
		}
	}	
	SCCB_Stop(); // 停止通讯
	
	status = SUCCESS;	// 写入成功
	return status;
}
/********************************************************************************************/
