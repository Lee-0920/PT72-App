/*
 * ThermostatDeviceMap.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

/*
 * ThermostatDeviceMap.c
 *
 *  Created on: 2017年11月16日
 *      Author: LIANG
 */

#include "ThermostatDeviceMap.h"
#include "Driver/System.h"
#include <string.h>
#include "Tracer/Trace.h"
#include "Driver/HardwareType.h"

// 加热丝输出使能引脚配置
#define DCENABLE_PIN                         GPIO_Pin_15
#define DCENABLE_PORT                        GPIOA
#define DCENABLE_RCC_CONFIG                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)

//加热丝输出使能初始化(V2.0使用)
static void ThermostatDeviceMap_DCEnable();

//加热丝输出使能控制(V2.0使用)
static void ThermostatDeviceMap_DCSetStatus(Bool status);

//加热丝输出
static Bool ThermostatDeviceMap_SetOutputWay1(ThermostatDeviceDriver *deviceDriver, float level);

//风扇输出
static Bool ThermostatDeviceMap_SetOutputWay2(ThermostatDeviceDriver *deviceDriver, float level);

void ThermostatDeviceMap_Init(ThermostatDevice* device)
{
    //加热设备
    //消解加热丝1 //TEMP_CTRL
	if (Hardware_v20 == HardwareType_GetValue())
	{
		device[0].maxDutyCycle = 0.72;
		ThermostatDeviceMap_DCEnable();
	}
	else
	{
		device[0].maxDutyCycle = 0.25;
	}
    device[0].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay1;
    device[0].deviceDriver.mode = THERMOSTATDEVICEDRIVER_PWM;
    device[0].deviceDriver.port = GPIOC;
    device[0].deviceDriver.pin = GPIO_Pin_8;
    device[0].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOC;
    device[0].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource8;
    device[0].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM8;
    device[0].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB2PeriphClockCmd;
    device[0].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB2Periph_TIM8;
    device[0].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 179;
    device[0].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
    device[0].deviceDriver.modeConfig.PWMConfig.timerChannel = 3;
    device[0].deviceDriver.modeConfig.PWMConfig.timer = TIM8;
    device[0].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
    device[0].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;//在向上计数模式下，TIMx_CNT < TIMx_CCR1时，通道1为无效电平
    ThermostatDevice_Init(&device[0]);

    //制冷设备
    //消解风扇1 //FAN_CTRL
    device[1].maxDutyCycle = 1;
    device[1].setOutputWayFunc = ThermostatDeviceMap_SetOutputWay2;
    device[1].deviceDriver.mode = THERMOSTATDEVICEDRIVER_PWM;
    device[1].deviceDriver.port = GPIOC;
    device[1].deviceDriver.pin = GPIO_Pin_9;
    device[1].deviceDriver.gpioRcc = RCC_AHB1Periph_GPIOC;
    device[1].deviceDriver.modeConfig.PWMConfig.pinSource = GPIO_PinSource9;
    device[1].deviceDriver.modeConfig.PWMConfig.goipAF = GPIO_AF_TIM8;
    device[1].deviceDriver.modeConfig.PWMConfig.timerRccInitFunction = RCC_APB2PeriphClockCmd;
    device[1].deviceDriver.modeConfig.PWMConfig.timerRcc = RCC_APB2Periph_TIM8;
    device[1].deviceDriver.modeConfig.PWMConfig.timerPrescaler = 179;
    device[1].deviceDriver.modeConfig.PWMConfig.timerPeriod = 49999;
    device[1].deviceDriver.modeConfig.PWMConfig.timerChannel = 4;
    device[1].deviceDriver.modeConfig.PWMConfig.timer = TIM8;
    device[1].deviceDriver.modeConfig.PWMConfig.timerOCPolarity = TIM_OCPolarity_Low;
    device[1].deviceDriver.modeConfig.PWMConfig.timerOCMode = TIM_OCMode_PWM2;
    ThermostatDevice_Init(&device[1]);
}

static Bool ThermostatDeviceMap_SetOutputWay1(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 1");
	if (Hardware_v20 == HardwareType_GetValue())
	{
		if(level)
		{
			ThermostatDeviceMap_DCSetStatus(TRUE);
		}
		else
		{
			ThermostatDeviceMap_DCSetStatus(FALSE);
		}
	}
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

static Bool ThermostatDeviceMap_SetOutputWay2(ThermostatDeviceDriver *deviceDriver, float level)
{
    TRACE_CODE("\n Output way 2");
    if (0 != level)
    {
    	//TRACE_INFO("\n Output way 2");
        level = 0.5 * level + 0.5;
        if (level < 0.75)
        {
            ThermostatDeviceDriver_SetOutput(deviceDriver, 1);
            System_Delay(200);
        }
    }
    return ThermostatDeviceDriver_SetOutput(deviceDriver, level);
}

char* ThermostatDeviceMap_GetName(Uint8 index)
{
    static char name[35] = "";
    switch(index)
    {
    case MEASUREMODULE_HEATER1:
        strcpy(name, "MeasureModuleHeater1");
        break;
    case MEASUREMODULE_FAN1:
        strcpy(name, "MeasureModuleFan1");
        break;
    default:
        strcpy(name, "NULL");
        break;
    }
    return name;
}

static void ThermostatDeviceMap_DCEnable()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    DCENABLE_RCC_CONFIG;

    GPIO_InitStructure.GPIO_Pin=DCENABLE_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(DCENABLE_PORT,&GPIO_InitStructure);
    GPIO_ResetBits(DCENABLE_PORT, DCENABLE_PIN);
}

static void ThermostatDeviceMap_DCSetStatus(Bool status)
{
	if (status && !GPIO_ReadOutputDataBit(DCENABLE_PORT, DCENABLE_PIN))
	{
		TRACE_DEBUG("DC10V Set Enable Open");
		GPIO_SetBits(DCENABLE_PORT, DCENABLE_PIN);
	}
	else if (!status && GPIO_ReadOutputDataBit(DCENABLE_PORT, DCENABLE_PIN))
	{
		TRACE_DEBUG("DC10V Set Enable Close");
		//GPIO_ResetBits(DCENABLE_PORT, DCENABLE_PIN);
		GPIO_SetBits(DCENABLE_PORT, DCENABLE_PIN);
	}
}

