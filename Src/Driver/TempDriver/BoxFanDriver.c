
/*
 * BoxFanDriver.c
 *
 *  Created on: 2016年11月09日
 *      Author: Liang
 */
#include <TempDriver/BoxFanDriver.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"

// 机箱风扇 //VALVE_19
#define BOXFAN_PIN                         GPIO_Pin_8
#define BOXFAN_PORT                        GPIOA
#define BOXFAN_RCC_CONFIG                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)

static Bool s_isBoxFanOpen = FALSE;

void BoxFanDriver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    BOXFAN_RCC_CONFIG;

    GPIO_InitStructure.GPIO_Pin=BOXFAN_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(BOXFAN_PORT,&GPIO_InitStructure);
    GPIO_ResetBits(BOXFAN_PORT, BOXFAN_PIN);
//    GPIO_SetBits(BOXFAN_PORT, BOXFAN_PIN);
}

/**
 * @brief not use
 * @param
 */
static void BoxFanDriver_Open(void)
{
	if (!s_isBoxFanOpen)
	{
		s_isBoxFanOpen = TRUE;
		GPIO_SetBits(BOXFAN_PORT, BOXFAN_PIN);
	}
}
/**
 * @brief not use
 * @param
 */
static void BoxFanDriver_Close(void)
{
	if (s_isBoxFanOpen)
	{
	    s_isBoxFanOpen = FALSE;
	    GPIO_ResetBits(BOXFAN_PORT, BOXFAN_PIN);
	}
}
/**
 * @brief 设置风扇转速
 * @param
 */
void BoxFanDriver_SetOutput(float level)
{

    if (level <= 1 && level >= 0)
    {
        if(level != 0)
        {
            BoxFanDriver_Open();

        }
        else
        {
            BoxFanDriver_Close();
        }
        TRACE_DEBUG("\n outsidefan level %d %%", (uint32_t )(level * 100));
    }
    else
    {
        TRACE_ERROR("\n OutSideFan level error!");
    }
}

Bool BoxFanDriver_IsOpen(void)
{
    return s_isBoxFanOpen;
}
