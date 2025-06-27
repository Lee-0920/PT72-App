/*
 * StaticADControl.c
 *
 *  Created on: 2018年11月21日
 *      Author: Administrator
 */
#include "FreeRTOS.h"
#include "task.h"
#include "SystemConfig.h"
#include "Driver/McuFlash.h"
#include "DncpStack/DncpStack.h"
#include "Tracer/Trace.h"
#include "StaticADControl.h"
#include "OpticalMeter/Meter.h"
#include "OpticalControl.h"
#include <math.h>
#include <OpticalDriver/MCP4651Driver.h>
//#include <OpticalDriver/OpticalLed.h>
#include <OpticalDriver/OpticalXonen.h>

typedef enum
{
    ADCTL_IDLE,
    ADCTL_BUSY
} ADCtrlStatus;

typedef enum
{
    POSITIVE_CORRELATION,    //正相关-电阻越大 AD越大
    NEGATIVE_CORRELATION    //负相关-电阻越大 AD越小
}ADCtrlCorrelation;

typedef struct
{
    MCP4651Driver* MCP4651;
    Uint16  defaultValue;
    Uint16 extDefaultValue;
    Uint8   addr;
    Uint8 writeIndex;   ///数字电位器写寄存器索引
}ADController;

typedef struct
{
    ADController adController[AD_CONTROLLER_NUM];

    StaticADControlResult adCtrlResult;
    Bool adCtrlSendEvent;
    ADCtrlStatus adCtrlStatus;
    Uint8 currentIndex;   ///数字电位器器件索引
    Uint32 targetAD;
    Uint32 currentAD;
    Uint32 lastAD;
    Uint16 currentValue;
    Uint16 lastValue;
    Uint16 minValue;
    Uint16 maxValue;
    Bool adCtrlOver;
    MCP4651Driver meaureMCP4651;
    //MCP4651Driver meterMCP4651;
    Bool isValid;

    ADCtrlCorrelation correlation;


    Bool isExtLED;  ///是否为附加的LED,需使用附加的参数存储区

    TaskHandle_t staticADControlHandle;
}StaticADController;

static StaticADController g_staticADController;

static MCP4651Driver meaureMCP4651;
static MCP4651Driver meterMCP4651;
static float s_staticMultiple = 5;

static void StaticADControl_ADHandleTask(void *argument);
static void StaticADControl_MCP4651InitHandleTask(void *argument);
static Uint32 StaticADControl_GetCurrentAD(Uint8 index);

void StaticADControl_Init(void)
{
    StaticADControl_InitDriver();
    StaticADControl_InitParam();
//    StaticADControl_InitSetting();

    xTaskCreate(StaticADControl_ADHandleTask, "StaticADControlHandle",
            STATIC_AD_CONTROL_STK_SIZE, &g_staticADController,
            STATIC_AD_CONTROL_TASK_PRIO, &g_staticADController.staticADControlHandle);
    xTaskCreate(StaticADControl_MCP4651InitHandleTask, "StaticADInitHandle",
    		STATIC_AD_MCPINIT_STK_SIZE, NULL,
			STATIC_AD_MCPINIT_TASK_PRIO, NULL);
//    TRACE_INFO("\n static ad controller init over");
}

void StaticADControl_InitDriver(void)
{
//测量系统
    meaureMCP4651.pinSCL = GPIO_Pin_10;
    meaureMCP4651.portSCL = GPIOE;
    meaureMCP4651.rccSCL  = RCC_AHB1Periph_GPIOE;
    meaureMCP4651.pinSDA = GPIO_Pin_9;
    meaureMCP4651.portSDA = GPIOE;
    meaureMCP4651.rccSDA  = RCC_AHB1Periph_GPIOE;

    MCP4651_Init(&meaureMCP4651);

    g_staticADController.adController[LED_REF].MCP4651 = &meaureMCP4651;
    g_staticADController.adController[LED_REF].addr = MCP4651_ADDR_0;
    g_staticADController.adController[LED_REF].writeIndex = LED_REF;

    g_staticADController.adController[LED_MEA].MCP4651 = &meaureMCP4651;
    g_staticADController.adController[LED_MEA].addr = MCP4651_ADDR_1;
    g_staticADController.adController[LED_MEA].writeIndex = LED_MEA;

//定量系统
    meterMCP4651.pinSCL = GPIO_Pin_13;
    meterMCP4651.portSCL = GPIOE;
    meterMCP4651.rccSCL  = RCC_AHB1Periph_GPIOE;
    meterMCP4651.pinSDA = GPIO_Pin_14;
    meterMCP4651.portSDA = GPIOE;
    meterMCP4651.rccSDA  = RCC_AHB1Periph_GPIOE;

    MCP4651_Init(&meterMCP4651);

	g_staticADController.adController[LED_METER1].MCP4651 = &meterMCP4651;  //使用同一组I2C通信
	g_staticADController.adController[LED_METER1].addr = AD5175_ADDR_1;
	g_staticADController.adController[LED_METER1].MCP4651->index = LED_METER1;

	g_staticADController.adController[LED_METER2].MCP4651 = &meterMCP4651;
	g_staticADController.adController[LED_METER2].addr = AD5175_ADDR_2;
	g_staticADController.adController[LED_METER2].MCP4651->index = LED_METER2;
}

void StaticADControl_InitParam(void)
{
    Uint8 buffer[MCP4651_CONTROL_SIGN_FLASH_LEN] = { 0 };
    Uint32 flashFactorySign = 0;
    //Uint8 param[AD_CONTROLLER_NUM*2] = {0};
    Uint8 param[4*2] = {0};
//    Uint32 extFlashFactorySign = 0;
//    Uint8 extParam[AD_CONTROLLER_NUM*2] = {0};
    Uint8 i;

    //数字电位器参数初始化
    McuFlash_Read(MCP4651_CONTROL_SIGN_FLASH_BASE_ADDR, MCP4651_CONTROL_SIGN_FLASH_LEN, buffer);  //读取出厂标志位
    memcpy(&flashFactorySign, buffer, MCP4651_CONTROL_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign)       //表示已经过出厂设置
    {
        for(i = 0; i < AD_CONTROLLER_NUM; i++)
        {
            g_staticADController.adController[i].defaultValue = StaticADControl_GetDefaultValue(i, FALSE);
            System_NonOSDelay(100);

        }
//        TRACE_ERROR("\n MCP4651 out of factory.");
    }
    else
    {
        //未出厂,使用设备默认值
        for(i = 0; i < AD_CONTROLLER_NUM; i++)
        {
            //g_staticADController.adController[i].defaultValue = MCP4651_ReadRDAC(g_staticADController.adController[i].MCP4651, g_staticADController.adController[i].addr);
        	if (i < LED_METER1)
        	{
                g_staticADController.adController[i].defaultValue = MCP4651_DEFAULTVALUE;
        	}
        	else
        	{
                g_staticADController.adController[i].defaultValue = MCP4651_DEFAULTVALUE_METER;
        	}
            memcpy(&param[i*2], &g_staticADController.adController[i].defaultValue, sizeof(Uint16));

        }
        //保存设备默认值至Flash
        McuFlash_Write(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR, MCP4651_CONTROL_PARAM_FLASH_LEN_TOTAL, param);
        //写入出厂标志
        flashFactorySign = FLASH_FACTORY_SIGN;
        memcpy(buffer, &flashFactorySign, MCP4651_CONTROL_SIGN_FLASH_LEN);

        McuFlash_Write(MCP4651_CONTROL_SIGN_FLASH_BASE_ADDR, MCP4651_CONTROL_SIGN_FLASH_LEN, buffer);
    }
}

void StaticADControl_InitSetting(void)
{
	Uint8 i = 0;
    g_staticADController.correlation = POSITIVE_CORRELATION;
    g_staticADController.isValid = TRUE;

    for(i = 0; i < AD_CONTROLLER_NUM; i++)
    {

    	System_Delay(100);

		if(FALSE == AD5175_WriteRDAC(g_staticADController.adController[i].MCP4651,
									 g_staticADController.adController[i].addr,
									 g_staticADController.adController[i].defaultValue,
									 g_staticADController.adController[i].writeIndex))     //使用默认值设置MCP4651
		{
			g_staticADController.isValid = FALSE;    //初始化失败置功能位
			TRACE_ERROR("\n static ad control invalid.");
		}

    }

}

Bool StaticADControl_Start(Uint8 index, Uint32 targetAD, Bool isExtLED)
{
	Uint8 buffer[MCP4651_CONTROL_PARAM_FLASH_LEN] = {0};
	Uint8 offset = 0;

    if(index < AD_CONTROLLER_NUM)
    {
        if (ADCTL_IDLE == g_staticADController.adCtrlStatus)
        {
            if(index == LED_REF || index == LED_MEA)
            {
//                if(isExtLED == TRUE)  //使用附加的LED
//                {
//                    OpticalLed_TurnOn(1);
//                }
//                else
//                {
//                    OpticalLed_TurnOn(0);
//                }
            }
//            else if(index == LED_METER1)
//            {
//                MeterLED_TurnOn(1);
//            }
//            else if(index == LED_METER2)
//            {
//                MeterLED_TurnOn(2);
//            }


            g_staticADController.isExtLED = isExtLED;
            g_staticADController.currentIndex = index;
            g_staticADController.targetAD = targetAD;
            g_staticADController.currentAD = 0xFFFFFFFF;
            g_staticADController.lastAD = g_staticADController.currentAD;

            offset = index*MCP4651_CONTROL_PARAM_FLASH_LEN;
			McuFlash_Read(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset, sizeof(Uint16), buffer);
			memcpy(&g_staticADController.currentValue, buffer, sizeof(Uint16));
//			MCP4651_Reset(g_staticADController.adController[index].MCP4651);
//			System_Delay(5);
//			StaticADControl_SetRealValue(index, g_staticADController.currentValue);



//            g_staticADController.currentValue =  MCP4651_ReadRDAC(g_staticADController.adController[index].MCP4651, g_staticADController.adController[index].addr);
            g_staticADController.lastValue = 0xFFFF;
            if(index == LED_REF || index == LED_MEA)
            {
            	g_staticADController.maxValue = AD5175_MAX_VALUM;
            	g_staticADController.minValue = AD5175_MIN_VALUM;
            	g_staticADController.correlation = POSITIVE_CORRELATION;
            }
            else
            {
            	g_staticADController.maxValue = AD5175_MAX_VALUM;
            	g_staticADController.minValue = AD5175_MIN_VALUM;
            	g_staticADController.correlation = POSITIVE_CORRELATION;
            }
            g_staticADController.adCtrlOver = FALSE;
            g_staticADController.adCtrlResult = STATIC_AD_CONTROL_RESULT_UNFINISHED;
            g_staticADController.adCtrlStatus = ADCTL_BUSY;
            TRACE_INFO("\n static AD control start. index = %d, targetAD = %d, isExtLED = %d, currentValue = %d", index, targetAD, isExtLED, g_staticADController.currentValue);
            vTaskResume(g_staticADController.staticADControlHandle);
            return TRUE;
        }
        else
        {
            TRACE_ERROR("\n static AD control failed to start because it is running.");
            return FALSE;
        }
    }
    else
    {
        TRACE_ERROR("\n static AD control failed to start because index must between 0 - %d.", AD_CONTROLLER_NUM);
        return FALSE;
    }
}

void StaticADControl_Stop(void)
{
    if (ADCTL_BUSY == g_staticADController.adCtrlStatus)
    {
        if(g_staticADController.adCtrlOver == TRUE)
        {
            g_staticADController.adCtrlResult =  STATIC_AD_CONTROL_RESULT_FINISHED;
        }
        else
        {
            g_staticADController.adCtrlResult =  STATIC_AD_CONTROL_RESULT_UNFINISHED;
        }

        if(g_staticADController.currentIndex == LED_REF || g_staticADController.currentIndex == LED_MEA)
        {
//            if(g_staticADController.isExtLED)
//            {
//                OpticalLed_TurnOff(1);      //使用附加的LED
//            }
//            else
//            {
//                OpticalLed_TurnOff(0);
//            }
        }
        /*
        else if(g_staticADController.currentIndex == LED_METER1)
        {
            MeterLED_TurnOff(1);
        }
        else if(g_staticADController.currentIndex == LED_METER2)
        {
            MeterLED_TurnOff(2);
        }
        */

        if(g_staticADController.adCtrlSendEvent == TRUE)
        {
            // 发送结果事件
            DncpStack_SendEvent(DSCP_EVENT_OAI_STATIC_AD_CONTROL_RESULT, &g_staticADController.adCtrlResult, sizeof(StaticADControlResult));
            DncpStack_BufferEvent(DSCP_EVENT_OAI_STATIC_AD_CONTROL_RESULT, &g_staticADController.adCtrlResult, sizeof(StaticADControlResult));
            StaticADControl_SendEventClose();  //关闭事件发送
            TRACE_INFO("\n static ad control send result = %d addr = %0x", (Uint8)g_staticADController.adCtrlResult, g_staticADController.adController[g_staticADController.currentIndex].addr);
        }

        if(g_staticADController.adCtrlOver == TRUE)
        {
            StaticADControl_SetDefaultValue(g_staticADController.currentIndex, g_staticADController.currentValue, g_staticADController.isExtLED);
        }

        g_staticADController.isExtLED = FALSE;
        g_staticADController.adCtrlOver = FALSE;
        g_staticADController.adCtrlStatus = ADCTL_IDLE;
        TRACE_INFO("\n static ad control stop.");
    }
    else
    {
        TRACE_INFO("\n static ad controller is not running.");
    }
}

Uint16 StaticADControl_GetDefaultValue(Uint8 index, Bool isExtLED)
{
    Uint8 buffer[MCP4651_CONTROL_PARAM_FLASH_LEN] = {0};
    Uint16 offset = 0;
    Uint16 value = 0xFFFF;

    if(index < AD_CONTROLLER_NUM)
    {
        offset = index*MCP4651_CONTROL_PARAM_FLASH_LEN;

        if(isExtLED == TRUE)
        {
            McuFlash_Read(EXTMCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset, sizeof(Uint16), buffer);
        }
        else
        {
            McuFlash_Read(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset, sizeof(Uint16), buffer);
        }

        memcpy(&value, buffer, sizeof(Uint16));

        TRACE_INFO("\n static ad: %d default value %d.", index, value);
    }
    else
    {
        TRACE_ERROR("\n invalid index %d.", index);
    }

    return value;
}

void StaticADControl_SetDefaultValue(Uint8 index, Uint16 value, Bool isExtLED)
{
    Uint8 buffer[MCP4651_CONTROL_PARAM_FLASH_LEN] = {0};
    Uint16 offset = 0;
//    Uint16 useValue = value&MCP4651_MAX_VALUE;
    Uint16 useValue = value;


    if(index < AD_CONTROLLER_NUM)
    {
        offset = index*MCP4651_CONTROL_PARAM_FLASH_LEN;

        memcpy(buffer, &useValue, sizeof(Uint16));

        if(isExtLED == TRUE)
        {
            McuFlash_Write(EXTMCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset, sizeof(Uint16), buffer);
            g_staticADController.adController[index].extDefaultValue = useValue;
        }
        else
        {
            McuFlash_Write(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset, sizeof(Uint16), buffer);
            g_staticADController.adController[index].defaultValue = useValue;
        }

        TRACE_INFO("\n static ad ctrl set  index %d default value %d.", index, value);
    }
    else
    {
        TRACE_ERROR("\n invalid index %d.", index);
    }
}

Uint16 StaticADControl_GetRealValue(Uint8 index)
{
	Uint16 value = 0xFFFF;

//	if(index < LED_METER1)
//	{
//		 value = MCP4651_ReadRDAC(g_staticADController.adController[index].MCP4651, g_staticADController.adController[index].addr, g_staticADController.adController[index].writeIndex);
//		 TRACE_INFO("\n static ad ctrl get  index %d current value %d.", index, value);
//	}
//	else
	if (index < AD_CONTROLLER_NUM)
	{
		value = AD5175_ReadRDAC(g_staticADController.adController[index].MCP4651, g_staticADController.adController[index].addr, g_staticADController.adController[index].writeIndex);
		TRACE_INFO("\n static ad ctrl get  index %d current value %d.", index, value);
	}
	else
	{
		TRACE_ERROR("\n invalid index %d  return %d.", index, value);
	}

	return value;
}

static Bool StaticADControl_ResetRealValue(Uint8 index, Uint16 value)
{
    if(MCP4651_WriteRDAC(g_staticADController.adController[index].MCP4651,
    						g_staticADController.adController[index].addr,
								value,
									g_staticADController.adController[index].writeIndex))
    {
        TRACE_INFO("\n static ad ctrl set  index %d realtime value %d.", index, value);
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n static ad ctrl set  realtime value fail.");
        return FALSE;
    }
}

Bool StaticADControl_SetRealValue(Uint8 index, Uint16 value)
{
//	if(index < LED_METER1)
//	{
//		if(MCP4651_WriteRDAC(g_staticADController.adController[index].MCP4651,
//								g_staticADController.adController[index].addr,
//									value,
//										g_staticADController.adController[index].writeIndex))
//		{
//			TRACE_INFO("\n static ad ctrl set  index %d realtime value %d.", index, value);
//
//			Uint8 offset = index*2;
//			Uint8 param[2] = {0};
//			g_staticADController.adController[index].defaultValue = value;
//			memcpy(&param, &value, sizeof(Uint16));
//			McuFlash_Write(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset , MCP4651_CONTROL_PARAM_FLASH_LEN, param);
//			return TRUE;
//		}
//		else
//		{
//			TRACE_ERROR("\n static ad ctrl set  realtime value fail.");
//			return FALSE;
//		}
//	}
//	else
//	{
		if(AD5175_WriteRDAC(g_staticADController.adController[index].MCP4651,
										g_staticADController.adController[index].addr,
											value,
												g_staticADController.adController[index].writeIndex))
				{
					TRACE_INFO("\n static ad ctrl set  index %d realtime value %d.", index, value);

					Uint8 offset = index*2;
					Uint8 param[2] = {0};
					g_staticADController.adController[index].defaultValue = value;
					memcpy(&param, &value, sizeof(Uint16));
					McuFlash_Write(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset , MCP4651_CONTROL_PARAM_FLASH_LEN, param);
					return TRUE;
				}
				else
				{
					TRACE_ERROR("\n static ad ctrl set  realtime value fail.");
					return FALSE;
				}
//	}
}

static Uint32 StaticADControl_GetCurrentAD(Uint8 index)
{
    Uint8 filterNum = 10;
    Uint32 ad = 0;
    OpticalSignalAD optAD;
    OpticalMeterAD meterAD;

    switch(index)
    {
    	case LED_REF:
			for(int i = 0; i < filterNum; i++)
			{
				while(FALSE == OpticalControl_GetADS1146Data(&optAD))
				{
					System_Delay(2);
				}
				ad = ad + optAD.reference;
				TRACE_CODE("\n current LED_REF ad[%d] = %d", i, optAD.reference);
//                TRACE_INFO("\n current LED_REF ad[%d] = %d", i, optAD.reference);
				System_Delay(300);
			}
			ad = ad/filterNum;
			TRACE_DEBUG("\n current LED_REF ad average  = %d", ad);
			break;

    	case LED_MEA:
			for(int i = 0; i < filterNum; i++)
			{
				while(FALSE == OpticalControl_GetADS1146Data(&optAD))
				{
					System_Delay(2);
				}
				ad = ad + optAD.measure;
				TRACE_CODE("\n current LED_MEA ad[%d] = %d", i, optAD.measure);
//                TRACE_INFO("\n current LED_MEA ad[%d] = %d", i, optAD.measure);
				System_Delay(300);
			}
			ad = ad/filterNum;
			TRACE_DEBUG("\n current LED_MEA ad average  = %d", ad);
			break;

        case LED_METER1:
            for(int i = 0; i < filterNum; i++)
            {
                meterAD = Meter_GetOpticalAD();
                ad = ad + meterAD.adValue[0];
//                TRACE_CODE("\n current LED_METER1 ad[%d] = %d", i, meterAD.adValue[0]);
//                TRACE_INFO("\n current LED_METER1 ad[%d] = %d", i, meterAD.adValue[0]);
                System_Delay(100);
            }
            ad = ad/filterNum;
            TRACE_DEBUG("\n current LED_METER1 ad average  = %d", ad);
            break;

        case LED_METER2:
            for(int i = 0; i < filterNum; i++)
            {
                meterAD = Meter_GetOpticalAD();
                ad = ad + meterAD.adValue[1];
//                TRACE_CODE("\n current LED_METER2 ad[%d] = %d", i, meterAD.adValue[1]);
//                TRACE_INFO("\n current LED_METER2 ad[%d] = %d", i, meterAD.adValue[1]);
                System_Delay(100);
            }
            ad = ad/filterNum;
            TRACE_DEBUG("\n current LED_METER2 ad average  = %d", ad);
            break;

        default:
            break;
    }

    return ad;
}

void StaticADControl_ADHandleTask(void *argument)
{
    StaticADController* staticADController = (StaticADController*)argument;
    vTaskSuspend(NULL);
    static Uint8 errorCount = 0;
    while (1)
    {
        switch (staticADController->adCtrlStatus)
        {
            case ADCTL_IDLE:
            	errorCount = 0;
                vTaskSuspend(NULL);
                break;

            case ADCTL_BUSY:
                staticADController->currentAD = StaticADControl_GetCurrentAD(staticADController->currentIndex);
                TRACE_INFO("\n static AD control current value = %d, ad = %d", staticADController->currentValue, staticADController->currentAD);
//                TRACE_INFO("\n static AD control last value = %d, ad = %d", staticADController->lastValue, staticADController->lastAD);
                if(fabs((double)staticADController->lastValue - (double)staticADController->currentValue)  < 1 || fabs((double)staticADController->currentAD - (double)staticADController->targetAD) <= 0.01*(double)staticADController->targetAD)
                {
                    if(fabs((double)staticADController->lastAD - (double)staticADController->targetAD) <= fabs((double)staticADController->currentAD - (double)staticADController->targetAD))
                    {
                        staticADController->currentValue = staticADController->lastValue;
                    }
                    staticADController->adCtrlOver = TRUE;
                }
                else
                {
                    if(POSITIVE_CORRELATION == staticADController->correlation)  //正相关
                    {
                        if(staticADController->currentAD <= staticADController->targetAD)    //电阻越大 AD越大
                        {
                            staticADController->minValue = staticADController->currentValue;
                            staticADController->lastValue = staticADController->currentValue;
                            staticADController->lastAD = staticADController->currentAD;
                            staticADController->currentValue = (staticADController->currentValue + staticADController->maxValue)/2;
                        }
                        else
                        {
                            staticADController->maxValue = staticADController->currentValue;
                            staticADController->lastValue = staticADController->currentValue;
                            staticADController->lastAD = staticADController->currentAD;
                            staticADController->currentValue = (staticADController->currentValue + staticADController->minValue)/2;
                        }
                    }
                    else if(NEGATIVE_CORRELATION == staticADController->correlation)  //负相关
                    {
                        if(staticADController->currentAD <= staticADController->targetAD)    //电阻越小 AD越大
                        {
                            staticADController->maxValue = staticADController->currentValue;
                            staticADController->lastValue = staticADController->currentValue;
                            staticADController->lastAD = staticADController->currentAD;
                            staticADController->currentValue = (staticADController->currentValue + staticADController->minValue)/2;
                        }
                        else
                        {
                            staticADController->minValue = staticADController->currentValue;
                            staticADController->lastValue = staticADController->currentValue;
                            staticADController->lastAD = staticADController->currentAD;
                            staticADController->currentValue = (staticADController->currentValue + staticADController->maxValue)/2;
                        }
//                        TRACE_INFO("\n currentValue = %d, maxValue = %d, minValue = %d ", staticADController->currentValue, staticADController->maxValue, staticADController->minValue);
                    }


                    staticADController->adCtrlOver = FALSE;
                }
                TRACE_INFO("\n calculate new value = %d", staticADController->currentValue);

                {
                	if(FALSE == AD5175_WriteRDAC(staticADController->adController[staticADController->currentIndex].MCP4651,
                														staticADController->adController[staticADController->currentIndex].addr,
                															staticADController->currentValue,
                																staticADController->adController[staticADController->currentIndex].writeIndex)) //写失败后停止
					{
						errorCount = errorCount + 1;
						System_Delay(500);
						if (errorCount >= 4)
						{
							StaticADControl_Stop();
						}
					}
					else
					{
						errorCount = 0;
					}
                }

                if(staticADController->adCtrlOver == TRUE)
                {
                    TRACE_INFO("\n static AD control over. value = %d", staticADController->currentValue);
                    errorCount = 0;

                    StaticADControl_Stop();
                }
                System_Delay(1000);
                break;
        }
    }
}

void StaticADControl_SendEventOpen(void)
{
    g_staticADController.adCtrlSendEvent = TRUE;
}

void StaticADControl_SendEventClose(void)
{
    g_staticADController.adCtrlSendEvent = FALSE;
}

Bool StaticADControl_IsValid(void)
{
    return g_staticADController.isValid;
}

Bool StaticADControl_ResetMeaLedParam(Bool isExtLED)
{
    if(isExtLED == TRUE)
    {
    	StaticADControl_ResetRealValue(LED_REF, g_staticADController.adController[LED_REF].extDefaultValue);
        StaticADControl_ResetRealValue(LED_MEA, g_staticADController.adController[LED_MEA].extDefaultValue);
    }
    else
    {
    	StaticADControl_ResetRealValue(LED_REF, g_staticADController.adController[LED_REF].defaultValue);
    	StaticADControl_ResetRealValue(LED_MEA, g_staticADController.adController[LED_MEA].defaultValue);
    }

    return TRUE;
}

void StaticADControl_SetMultiple(float multiple)
{
	if (multiple>0)
	{
		s_staticMultiple = multiple;
		Printf("\nStatic Multiple %f",s_staticMultiple);
	}
}

 void StaticADControl_MCP4651InitHandleTask(void *argument)
 {
	 while(1)
	 {
		System_Delay(500);
		StaticADControl_InitSetting();
		TRACE_INFO("\n static ad controller init over");
		vTaskDelete(NULL);
	 }
 }


//***********供MCP4651下位机测试**********
 void StaticADControl_MCP4651Write(Uint8 index, Uint16 value)
 {
 	Uint8 offset = index*2;
 	Uint8 param[2] = {0};

	if(AD5175_WriteRDAC(g_staticADController.adController[index].MCP4651, g_staticADController.adController[index].addr, value, g_staticADController.adController[index].writeIndex))
	{
		g_staticADController.adController[index].defaultValue = value;
		memcpy(&param, &value, sizeof(Uint16));
		McuFlash_Write(MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + offset , MCP4651_CONTROL_PARAM_FLASH_LEN, param);
	}
	else
	{
		TRACE_ERROR("\n MCP4651 write flash failed \n.");
	}
 }


 void StaticADControl_MCP4651Read(Uint8 index)
 {
 	MCP4651_ReadRDAC(g_staticADController.adController[index].MCP4651, g_staticADController.adController[index].addr, g_staticADController.adController[index].writeIndex);
 	TRACE_ERROR("\n MCP4651 index : %d, receive addr: %x \n.", index, g_staticADController.adController[index].addr);
 }

