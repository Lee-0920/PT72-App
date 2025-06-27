/*
 * MerterPDDriver.h
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_METERADC_H_
#define SRC_DRIVER_METERADC_H_

#include "stm32F4xx.h"
#include "Tracer/Trace.h"

void MerterADC_Init(void);
void MeterADC_Stop(void);
void MeterADC_Start(void);
void MerterADC_SetBufferSize(uint16_t ndtr);
void MeterADC_GetData(uint8_t index, uint16_t *data, uint16_t num);
void MeterADC_PrintfInfo(void);
uint16_t MeterADC_AirADFilter(uint8_t index, uint32_t filterLow, uint32_t filterHigh);
uint16_t MeterADC_GetCurAD(uint8_t index);
uint16_t MeterADC_GetCurWare(uint8_t index);
#endif /* SRC_DRIVER_METERADC_H_ */
