/*
 * MCP4651Driver.c
 *
 *  Created on: 2018年11月19日
 *      Author: Administrator
 */
#include <OpticalDriver/MCP4651Driver.h>
#include "stm32f4xx.h"
#include "Tracer/Trace.h"

//********MCP4651命令码定义*********
#define MCP4651_TCON_CONNECT_ALL 0x1FF

#define MCP4651_WRITE_TCONRegister 		0x40	  //TCON寄存器地址0x04<<4 | 写命令码0x00

#define MCP4651_READ_TCONRegister 		0x4C	  //TCON寄存器地址0x04<<4 | 读命令码0x0C

#define MCP4651_WRITE_WIPER0 	0x00      //最低位用于存放D8的值  内存地址0x00<<4 | 写命令码0x00

#define MCP4651_WRITE_WIPER1 	0x10	  //最低位用于存放D8的值	内存地址0x01<<4 | 写命令码0x00

#define MCP4651_READ_WIPER0 	0x0C	  //最低位用于存放D8的值  内存地址0x00<<4 | 读命令码0x0C

#define MCP4651_READ_WIPER1 	0x1C	  //最低位用于存放D8的值 内存地址0x01<<4  | 读命令码0x0C

#define MCP4651_READ_RESERED_RAM 	0x5C  //最低位用于存放D8的值 内存地址0x05<<4  | 读命令码0x0C   默认保存值0x1F7

#define MCP4651_RESET			0x1FF


void MCP4651_SCLHigh(MCP4651Driver* MCP4651)
{
    GPIO_SetBits(MCP4651->portSCL, MCP4651->pinSCL);
}

void MCP4651_SCLLow(MCP4651Driver* MCP4651)
{
    GPIO_ResetBits(MCP4651->portSCL, MCP4651->pinSCL);
}

void MCP4651_SDAHigh(MCP4651Driver* MCP4651)
{
    GPIO_SetBits(MCP4651->portSDA, MCP4651->pinSDA);
}

//void MCP4651_HVC1Low(MCP4651Driver* MCP4651)
//{
//	GPIO_ResetBits(MCP4651->portHVC, MCP4651->pinHVC1);
//}

//void MCP4651_HVC2Low(MCP4651Driver* MCP4651)
//{
//	GPIO_ResetBits(MCP4651->portHVC, MCP4651->pinHVC2);
//}

//void MCP4651_HVC1High(MCP4651Driver* MCP4651)
//{
//	GPIO_SetBits(MCP4651->portHVC, MCP4651->pinHVC1);
//}

//void MCP4651_HVC2High(MCP4651Driver* MCP4651)
//{
//	GPIO_SetBits(MCP4651->portHVC, MCP4651->pinHVC2);
//}

void MCP4651_SDALow(MCP4651Driver* MCP4651)
{
    GPIO_ResetBits(MCP4651->portSDA, MCP4651->pinSDA);
}

Uint8 MCP4651_SDARead(MCP4651Driver* MCP4651)
{
    return GPIO_ReadInputDataBit(MCP4651->portSDA, MCP4651->pinSDA);
}

void MCP4651_SDAIn(MCP4651Driver* MCP4651)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = MCP4651->pinSDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(MCP4651->portSDA, &GPIO_InitStructure);
}

void MCP4651_SDAOut(MCP4651Driver* MCP4651)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = MCP4651->pinSDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(MCP4651->portSDA, &GPIO_InitStructure);
}


void MCP4651_Start(MCP4651Driver* MCP4651)
{
    MCP4651_SDAOut(MCP4651);

    //SCL、SDA拉高
    MCP4651_SDAHigh(MCP4651);
    MCP4651_SCLHigh(MCP4651);
    System_DelayUs(50);

    //SDA拉低
    MCP4651_SDALow(MCP4651);
    System_DelayUs(50);

    //SCL拉低
    MCP4651_SCLLow(MCP4651);
}

void MCP4651_Stop(MCP4651Driver* MCP4651)
{
    MCP4651_SDAOut(MCP4651);

    //SCL、SDA拉低
    MCP4651_SCLLow(MCP4651);
    MCP4651_SDALow(MCP4651);
    System_DelayUs(50);

    //SCL、SDA拉高
    MCP4651_SCLHigh(MCP4651);
    System_DelayUs(50);
    MCP4651_SDAHigh(MCP4651);
}

Bool MCP4651_WaitAck(MCP4651Driver* MCP4651)
{
    Uint16 errTimes = 0;
    MCP4651_SDAIn(MCP4651);

    MCP4651_SDAHigh(MCP4651);
    System_DelayUs(20);
    MCP4651_SCLHigh(MCP4651);
    System_DelayUs(20);

    while(MCP4651_SDARead(MCP4651))
    {
        errTimes++;
        if(errTimes>=800)
        {
            MCP4651_Stop(MCP4651);
            return FALSE;  //接收应答失败
        }
    }

    MCP4651_SCLLow(MCP4651);
    return TRUE;   //接收应答成功
}

void MCP4651_SendAck(MCP4651Driver* MCP4651)
{
    MCP4651_SCLLow(MCP4651);

    MCP4651_SDAOut(MCP4651);

    MCP4651_SDALow(MCP4651);
    System_DelayUs(2);
    MCP4651_SCLHigh(MCP4651);
    System_DelayUs(7);
    MCP4651_SCLLow(MCP4651);
}

void MCP4651_SendNoAck(MCP4651Driver* MCP4651)
{
    MCP4651_SCLLow(MCP4651);

    MCP4651_SDAOut(MCP4651);

    MCP4651_SDAHigh(MCP4651);
    System_DelayUs(2);
    MCP4651_SCLHigh(MCP4651);
    System_DelayUs(7);
    MCP4651_SCLLow(MCP4651);
}
Bool MCP4651_Reset(MCP4651Driver* MCP4651)
{
//	Uint16 byte = MCP4651_RESET;
	Uint8 i;

	MCP4651_Start(MCP4651);

	MCP4651_SDAOut(MCP4651);
	MCP4651_SCLLow(MCP4651);
	System_DelayUs(1);
	GPIO_SetBits(GPIOE, GPIO_Pin_10);
	for(i = 0; i < 8; i++)
	{
		MCP4651_SDAHigh(MCP4651);
//        TRACE_INFO("\n 1");

		System_DelayUs(50);

		MCP4651_SCLHigh(MCP4651);
		System_DelayUs(40);

		MCP4651_SCLLow(MCP4651);
		System_DelayUs(10);
	}
//	if(MCP4651_WaitAck(MCP4651))
//	{
//		TRACE_ERROR("\n reset fail :ack");
//		return FALSE;
//	}

	MCP4651_Start(MCP4651);
	MCP4651_Stop(MCP4651);
	TRACE_INFO("\n [reset success]");

	return TRUE;


}

void MCP4651_SendByte(MCP4651Driver* MCP4651, Uint8 byte)
{
    Uint8 i;
    MCP4651_SDAOut(MCP4651);

    MCP4651_SCLLow(MCP4651);
    System_DelayUs(5);
    for(i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        {
            MCP4651_SDAHigh(MCP4651);
        }
        else
        {
            MCP4651_SDALow(MCP4651);
        }
        byte <<= 1;
        System_DelayUs(50);

        MCP4651_SCLHigh(MCP4651);
        System_DelayUs(40);

        MCP4651_SCLLow(MCP4651);
        System_DelayUs(10);
    }
}

Uint8 MCP4651_ReadByte(MCP4651Driver* MCP4651)
{
    Uint8 i,receive = 0;
    MCP4651_SDAIn(MCP4651);

    for(i = 0; i < 8; i++)
    {
        MCP4651_SCLLow(MCP4651);
        System_DelayUs(4);
        MCP4651_SCLHigh(MCP4651);

        receive<<=1;
        if(MCP4651_SDARead(MCP4651))
        {
            receive++;
        }
        System_DelayUs(2);
    }

    return receive;
}

void MCP4651_Init(MCP4651Driver* MCP4651)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(MCP4651->rccSCL | MCP4651->rccSDA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = MCP4651->pinSCL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MCP4651->portSCL, &GPIO_InitStructure);
    MCP4651_SCLHigh(MCP4651);


    GPIO_InitStructure.GPIO_Pin = MCP4651->pinSDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MCP4651->portSDA, &GPIO_InitStructure);
    MCP4651_SDAHigh(MCP4651);

//	GPIO_InitStructure.GPIO_Pin = MCP4651->pinHVC1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(MCP4651->portHVC, &GPIO_InitStructure);
//	MCP4651_HVC1High(MCP4651);
////	MCP4651_HVC1Low(MCP4651);
//
//
//	GPIO_InitStructure.GPIO_Pin = MCP4651->pinHVC2;
//	GPIO_Init(MCP4651->portHVC, &GPIO_InitStructure);
//	MCP4651_HVC2High(MCP4651);  //地址位
//	MCP4651_HVC2Low(MCP4651);


}

Bool MCP4651_WriteOnce(MCP4651Driver* MCP4651, Uint8 addr, Uint16 word, Uint8 writeIndex)
{
	Uint8 addr_wirte = addr<<1;
    Uint8 msb = (Uint8)((word>>8)&0xFF);
    Uint8 lsb = (Uint8)(word&0xFF);

    //TRACE_INFO("\n MCP4651[%d], addr:%x, write cmd:%x, value: %d", writeIndex, addr, addr_wirte, word);

    if(MCP4651_MAX_VALUE & word)
    {
    	TRACE_ERROR("\n MCP4651 set max as 0x100");
    	msb |= MCP4651_MAX_VALUE>>8;
    	lsb = 0;
    }

    switch(writeIndex)
    {
	case MCP4651_LED_REF:
		msb |= MCP4651_WRITE_WIPER0;
		break;
	case MCP4651_LED_MEA:
		msb |= MCP4651_WRITE_WIPER1;
		break;
//	case MCP4651_LED_METER1:
//		msb |= MCP4651_WRITE_WIPER1;
//		break;
//	case MCP4651_LED_METER2:
//		msb |= MCP4651_WRITE_WIPER0;
//		break;
	default:
		TRACE_ERROR("\n MCP4651 index error : out of range");
		return FALSE;
		break;
    }

    MCP4651_Start(MCP4651);
    MCP4651_SendByte(MCP4651, addr_wirte);
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n MCP4651 write word send addr fail : nack");
        return FALSE;
    }
    MCP4651_SendByte(MCP4651, msb);
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n MCP4651 write word msb:%X fail : nack", msb);
        return FALSE;
    }

    MCP4651_SendByte(MCP4651, lsb);
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n MCP4651 write word lsb:%X fail : nack", lsb);
        return FALSE;
    }

    MCP4651_Stop(MCP4651);


    return TRUE;
}

Uint16 MCP4651_ReadOnce(MCP4651Driver* MCP4651, Uint8 addr, Uint8 readIndex)
{
	Uint8 addrWrite = (addr<<1);
    Uint8 addrRead = (addr<<1)+1;
    Uint8 readCmd = 0;
    Uint16 readValue = 0;
//    TRACE_INFO("\n MCP4651 addr:%x, read cmd:%x \n", addr, addr_read);

    switch(readIndex)
   {
	case MCP4651_LED_REF:
		readCmd |= MCP4651_READ_WIPER0;
		break;
	case MCP4651_LED_MEA:
		readCmd |= MCP4651_READ_WIPER1;
		break;
//	case MCP4651_LED_METER1:
//		readCmd |= MCP4651_READ_WIPER1;
//		break;
//	case MCP4651_LED_METER2:
//		readCmd |= MCP4651_READ_WIPER0;
//		break;
	default:
		TRACE_ERROR("\n MCP4651 index error : out of range");
		return FALSE;
		break;
   }

    MCP4651_Start(MCP4651);
    MCP4651_SendByte(MCP4651, addrWrite); //写设备地址
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n MCP4651 read word send addr fail : nack");
        return readValue;
    }
    MCP4651_SendByte(MCP4651, readCmd); //写寄存器地址
	if(!MCP4651_WaitAck(MCP4651))
	{
		TRACE_ERROR("\n MCP4651 read word send readCmd fail : nack");
		return readValue;
	}

	MCP4651_Start(MCP4651);	//重新发起开始信号
	MCP4651_SendByte(MCP4651, addrRead); //读设备地址
	if(!MCP4651_WaitAck(MCP4651))
	{
		TRACE_ERROR("\n MCP4651 read word send addr fail : nack");
		return readValue;
	}
    readValue += MCP4651_ReadByte(MCP4651);  //读上一次写寄存器地址数据
    MCP4651_SendAck(MCP4651);  //发送应答ACK
    readValue<<=8;
    readValue +=  MCP4651_ReadByte(MCP4651);
    MCP4651_SendNoAck(MCP4651);  //发送NACK

    MCP4651_Stop(MCP4651);

    return readValue;
}


Bool MCP4651_WriteRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint16 value, Uint8 writeIndex)
{
	Uint8 retry = 0;
//	TRACE_INFO("\n MCP4651:%d addr %d value %d writeIndex %d ",&MCP4651, addr,value,writeIndex);
	if(MCP4651_WriteOnce(MCP4651, addr, value, writeIndex))
	{
		TRACE_INFO("\n MCP4651 write rdac  success");
		return TRUE;
	}

	TRACE_INFO("\n MCP4651 write rdac fail");
	return FALSE;
}

Uint16 MCP4651_ReadRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint8 readIndex)
{
    Uint16 result = 0;

    result = MCP4651_ReadOnce(MCP4651, addr, readIndex);

    TRACE_INFO("\n MCP4651 read rdac: %d", result);
    return result;
}


Bool AD5175_WriteOnce(MCP4651Driver* MCP4651, Uint8 addr, Uint16 word)
{
	Uint8 addr_wirte = addr<<1;
    Uint8 msb = (Uint8)((word>>8)&0xFF);
    Uint8 lsb = (Uint8)(word&0xFF);

    //TRACE_INFO("\n AD5175, addr:%x, write cmd:%x, value: %d", addr, addr_wirte, word);


    MCP4651_Start(MCP4651);
    MCP4651_SendByte(MCP4651, addr_wirte);
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n AD5175 write word send addr fail : nack");
        return FALSE;
    }
    MCP4651_SendByte(MCP4651, msb);
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n AD5175 write word msb:%X fail : nack", msb);
        return FALSE;
    }

    MCP4651_SendByte(MCP4651, lsb);
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n AD5175 write word lsb:%X fail : nack", lsb);
        return FALSE;
    }

    MCP4651_Stop(MCP4651);


    return TRUE;
}

Bool AD5175_WriteRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint16 value, Uint8 writeIndex)
{
	Uint8 retry = 0;
	//TRACE_INFO("\n AD5175:%d addr %d value %d writeIndex %d ",&MCP4651, addr,value,writeIndex);
	if(!AD5175_WriteOnce(MCP4651, addr, 0x1c02))
	{
		TRACE_INFO("\n AD5175 write rdac  Fails");
		return TRUE;
	}
	value &= 0x03ff;
	value |= 0x0400;
	if(AD5175_WriteOnce(MCP4651, addr, value))
	{
		TRACE_DEBUG("\n AD5175 write rdac: %d success", value);
		return TRUE;
	}

	TRACE_INFO("\n AD5175 write rdac fail");
	return FALSE;
}

Uint16 AD5175_ReadOnce(MCP4651Driver* MCP4651, Uint8 addr)
{
	Uint8 addrWrite = (addr<<1);
    Uint8 addrRead = (addr<<1)+1;
    Uint16 readCmd = 0x0800;
    Uint16 readValue = 0;
    //TRACE_INFO("\n AD5175 addr:%x, read cmd:%x \n", addr, readCmd);

    MCP4651_Start(MCP4651);
    MCP4651_SendByte(MCP4651, addrWrite); //写设备地址
    if(!MCP4651_WaitAck(MCP4651))
    {
        TRACE_ERROR("\n AD5175 read word send addr fail : nack");
        return readValue;
    }
    MCP4651_SendByte(MCP4651, ((readCmd>>8)&0xff)); //写寄存器地址
	if(!MCP4651_WaitAck(MCP4651))
	{
		TRACE_ERROR("\n AD5175 read word send readCmd fail : nack");
		return readValue;
	}
	MCP4651_SendByte(MCP4651, (readCmd&0xff)); //写寄存器地址
	if(!MCP4651_WaitAck(MCP4651))
	{
		TRACE_ERROR("\n AD5175 read word send readCmd fail : nack");
		return readValue;
	}

	MCP4651_Start(MCP4651);	//重新发起开始信号
	MCP4651_SendByte(MCP4651, addrRead); //读设备地址
	if(!MCP4651_WaitAck(MCP4651))
	{
		TRACE_ERROR("\n AD5175 read word send addr fail : nack");
		return readValue;
	}
    readValue += MCP4651_ReadByte(MCP4651);  //读上一次写寄存器地址数据
    MCP4651_SendAck(MCP4651);  //发送应答ACK
    readValue<<=8;
    readValue +=  MCP4651_ReadByte(MCP4651);
    MCP4651_SendNoAck(MCP4651);  //发送NACK

    MCP4651_Stop(MCP4651);

    return readValue;
}

Uint16 AD5175_ReadRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint8 readIndex)
{
	Uint16 value = AD5175_ReadOnce(MCP4651, addr);
	TRACE_DEBUG("\n AD5175 read RDAC : %d", value);
	return value;
}

