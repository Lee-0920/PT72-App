/*
 * SpinValveDriver.h
 *
 *  Created on: 2019年9月11日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_LIQUIDCONTROLLER_SPINVALVEDRIVER_H_
#define SRC_DRIVER_LIQUIDCONTROLLER_SPINVALVEDRIVER_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

typedef enum
{
    Normal  = 0x00,                 ///< 状态正常
    FrameError = 0x01,           ///< 帧错误
    ParamError = 0x02,           ///< 参数错误
    SensorError = 0x03,          ///< 光耦错误
    MotorBusy = 0x04,            ///< 电机忙
	MotorLocked = 0x05,		 	 ///< 电机堵转
	UnknownPoint = 0x06,		 ///< 位置位置
	CmdRefuse = 0x07,			 ///< 指令被拒绝
    TaskSuspend = 0xFE,          ///< 任务挂起
    UnknowError = 0xFF,          ///< 未知错误
}SpinValveResp;

void SpinValveDriver_Init(void);
Bool SpinValveDriver_WaitResp(SpinValveResp* resp, Uint8* data, Uint16 timeoutMS);
Bool SpinValveDriver_SendCmd(Uint8 cmd, Uint8* paramData, Uint8 paramLen);

#endif /* SRC_DRIVER_LIQUIDCONTROLLER_SPINVALVEDRIVER_H_ */
