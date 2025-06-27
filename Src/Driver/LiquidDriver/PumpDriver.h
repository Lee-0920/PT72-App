/*
 * PumpDriver.h
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */

#ifndef SRC_PUMPDRIVER_H_
#define SRC_PUMPDRIVER_H_
#include "stm32f4xx.h"
#include "Common/Types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
//    GPIO_TypeDef *portEnable;
//    Uint16 pinEnable;
//    uint32_t rccEnable;
    GPIO_TypeDef *portDir;
    Uint16 pinDir;
    uint32_t rccDir;
    GPIO_TypeDef *portClock;
    Uint16 pinClock;
    uint32_t rccClock;
    GPIO_TypeDef *portReset;
    Uint16 pinReset;
    uint32_t rccReset;
    BitAction forwardLevel;
    GPIO_TypeDef *portDiag;
    Uint16 pinDiag;
    uint32_t rccDiag;
} PumpDriver;

typedef enum
{
    COUNTERCLOCKWISE, //逆时针
    CLOCKWISE, //顺时针
    MAX_DIRECTION
} Direction;


void PumpDriver_Init(PumpDriver *pumpDriver);
void PumpDriver_Enable(PumpDriver *pumpDriver);
void PumpDriver_Disable(PumpDriver *pumpDriver);
void PumpDriver_SetDirection(PumpDriver *pumpDriver, Direction dir);
void PumpDriver_PullHigh(PumpDriver *pumpDriver);
void PumpDriver_PullLow(PumpDriver *pumpDriver);
void PumpDriver_SetForwardLevel(PumpDriver *pumpDriver, BitAction bitVal);
void PumpDriver_Reset(PumpDriver *pumpDriver);

#ifdef __cplusplus
}
#endif

#endif /* SRC_PUMPDRIVER_H_ */
