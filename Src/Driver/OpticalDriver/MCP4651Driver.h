/*
 * MCP4651Driver.h
 *
 *  Created on: 2018年11月19日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_MCP4651DRIVER_H_
#define SRC_DRIVER_OPTICALDRIVER_MCP4651DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "Driver/System.h"

#define MCP4651_MAX_VALUE    0x100
#define MCP4651_MIN_VALUE    0x0
#define AD5175_MAX_VALUM	0x03ff
#define AD5175_MIN_VALUM	0x0000
#define MCP4651_ADDR_0   0x2c	//请酌情注意：当前代码配置拉高或拉低，地址A0脚都当1使用才可通信成功
#define MCP4651_ADDR_1   0x2F   //需酌情注意：当前代码配置拉高或拉低，地址A0脚都当1使用才可通信成功
#define AD5175_ADDR_1	0x2c	//定量点1
#define AD5175_ADDR_2	0x2F	//定量点2

#define MCP4651_MAX_CONTROLLER   2
#define MCP4651_LED_REF     0
#define MCP4651_LED_MEA     1
#define MCP4651_LED_METER1      2
#define MCP4651_LED_METER2      3

typedef struct
{
    GPIO_TypeDef *portSCL;
    Uint16 pinSCL;
    uint32_t rccSCL;
    GPIO_TypeDef *portSDA;
    Uint16 pinSDA;
    uint32_t rccSDA;
	//GPIO_TypeDef *portHVC;
	//Uint16 pinHVC1;
//	Uint16 pinHVC2;
//	uint32_t rccHVC;
	Uint8 index;

} MCP4651Driver;

/**
 * @brief MCP4651驱动
 * @details 使用I2C方式实现数字电位器MCP4651读写控制
 */
void MCP4651_Init(MCP4651Driver* MCP4651);

Uint16 MCP4651_ReadRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint8 readIndex);
Bool MCP4651_WriteRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint16 value, Uint8 writeIndex);
Uint16 AD5175_ReadRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint8 readIndex);
Bool AD5175_WriteRDAC(MCP4651Driver* MCP4651, Uint8 addr, Uint16 value, Uint8 writeIndex);
Bool MCP4651_Reset(MCP4651Driver* MCP4651);


#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_OPTICALDRIVER_MCP4651DRIVER_H_ */
