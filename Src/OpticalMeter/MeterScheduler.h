/*
 * MeterScheduler.h
 *
 *  Created on: 2016年6月23日
 *      Author: Administrator
 */

#ifndef SRC_OPTICALMETER_METERSCHEDULER_H_
#define SRC_OPTICALMETER_METERSCHEDULER_H_

typedef void (*MeterHandle)(void);

void MeterScheduler_Init(void);
void MeterScheduler_RegisterTask(MeterHandle handle);
void MeterScheduler_RegisterTimer(MeterHandle meterCheck, MeterHandle meterADPeriod);
void MeterScheduler_RegisterMeterCheck(MeterHandle meterCheck);
void MeterScheduler_MeterSuspend(void);
void MeterScheduler_MeterResume(void);
void MeterScheduler_StartMeterCheckTimer(void);
void MeterScheduler_StopMeterCheckTimer(void);
void MeterScheduler_SetMeterADReportPeriod(float report);
#endif /* SRC_OPTICALMETER_METERSCHEDULER_H_ */
