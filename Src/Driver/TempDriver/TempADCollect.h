/**
 * @file
 * @brief 消解温度采集驱动头文件。
 * @details 提供采集消解温度功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#ifndef SRC_DRIVER_TEMPDRIVER_TEMPADCOLLECT_H_
#define SRC_DRIVER_TEMPDRIVER_TEMPADCOLLECT_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Common/Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void TempADCollect_Init(void);
Uint16 TempADCollect_GetAD(Uint8 index);
#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_TEMPDRIVER_TEMPADCOLLECT_H_ */
