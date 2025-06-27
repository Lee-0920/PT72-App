/**
 * @file
 * @brief 光学信号AD通道选择驱动头文件。
 * @details 提供光学信号AD通道选择口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#include <OpticalDriver/OpticalChannel.h>
#include "stm32f4xx.h"
#include "Common/Types.h"

// **************** 引脚定义 ****************
#define REF_CTL_PIN      GPIO_Pin_11
#define REF_CTL_PORT     GPIOE
#define REF_CTL_RCC      RCC_AHB1Periph_GPIOE
#define REF_CTL_HIGH()   GPIO_SetBits(REF_CTL_PORT, REF_CTL_PIN)
#define REF_CTL_LOW()    GPIO_ResetBits(REF_CTL_PORT, REF_CTL_PIN)

#define MEAS_CTL_PIN     GPIO_Pin_12
#define MEAS_CTL_PORT    GPIOE
#define MEAS_CTL_RCC     RCC_AHB1Periph_GPIOE
#define MEAS_CTL_HIGH()  GPIO_SetBits(MEAS_CTL_PORT, MEAS_CTL_PIN)
#define MEAS_CTL_LOW()   GPIO_ResetBits(MEAS_CTL_PORT, MEAS_CTL_PIN)

/**
 * @brief 光学采集通道初始化
 * @param
 */
void OpticalChannel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(REF_CTL_RCC|MEAS_CTL_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = REF_CTL_PIN;
    GPIO_Init(REF_CTL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MEAS_CTL_PIN;
    GPIO_Init(MEAS_CTL_PORT, &GPIO_InitStructure);


    // 初始化
    REF_CTL_HIGH();
    MEAS_CTL_LOW();
}

/**
 * @brief 光学采集通道选择
 * @param index
 */
bool OpticalChannel_Select(Uint8 index)
{
    switch(index)
    {
    case 1: //参考
    	REF_CTL_HIGH();
    	MEAS_CTL_LOW();
        break;

    case 2: //测量
    	REF_CTL_LOW();
        MEAS_CTL_HIGH();
        break;
    default:
        return FALSE;
    }
    return TRUE;
}

