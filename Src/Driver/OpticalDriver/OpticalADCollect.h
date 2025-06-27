/**
 * @file
 * @brief 光学信号AD采集驱动头文件。
 * @details 提供光学信号AD采集功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#ifndef SRC_DRIVER_OPTICALADCOLLECT_H_
#define SRC_DRIVER_OPTICALCOLLECT_H_

#include "stm32f4xx.h"
#include "Common/Types.h"
#ifdef __cplusplus
extern "C" {
#endif

// 极性
#define AD7791_MODE_UNIPOLAR   (1 << 2)     // 单极性
#define AD7791_MODE_BIPOLAR    0x00         // 双极性

void OpticalADCollect_Init(void);
uint32_t OpticalADCollect_GetAD(Uint8 channel);
void OpticalADCollect_ChangePolar(unsigned char polar);
void OpticalADColletc_Gain_Set(int Gin);
#ifdef __cplusplus
}
#endif


#endif /* SRC_DRIVER_OPTICALADCOLLECT_H_ */
