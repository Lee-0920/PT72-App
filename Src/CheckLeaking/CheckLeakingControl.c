/*
 * CheckLeakingControl.c
 *
 *  Created on: 2021年10月11日
 *      Author: liwenqin
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "SystemConfig.h"
#include "Common/Types.h"
#include "CheckLeakingControl.h"
#include "Tracer/Trace.h"
#include "DNCP/App/DscpSysDefine.h"
#include "SystemConfig.h"
#include "DNCP/Lai/LaiRS485Handler.h"
#include "DncpStack/DncpStack.h"
#include "LuipApi/OpticalAcquireInterface.h"
#include "TempDriver/TempADCollect.h"

static void CheckLeaking_TaskHandle(void *pvParameters);

static xTaskHandle s_xHandle;
static int s_CheckLeakingReportPeriod = 2000;
static Bool s_eventReported = FALSE;


void CheckLeakingControl_Init(void)
{
    xTaskCreate(CheckLeaking_TaskHandle, "CheckLeakingHandle", LEAK_MONITOR_STK_SIZE, NULL,
    		     LEAK_MONITOR_TASK_PRIO, &s_xHandle);

}
static void CheckLeaking_TaskHandle(void *pvParameters)
{
	(void) pvParameters;
    static Uint16 temp = 0;
    while(1)
    {
        vTaskDelay(s_CheckLeakingReportPeriod / portTICK_RATE_MS);
        if (TRUE == LaiRS485_GetHostStatus() && s_eventReported)
        {
            temp =CheckLeakingControl_GetAD();
            TRACE_MARK("\n Current check leaking AD:%d", temp);
            DncpStack_SendEvent(DSCP_EVENT_DSI_CHECK_LEAKING_NOTICE, &temp, sizeof(temp));
        }
   }
}

/*单位 ：秒*/
void CheckLeaking_SetCheckLeakingReportPeriod(float period)
{
    if (period > 0 && s_CheckLeakingReportPeriod != (int)(period*1000))
    {
		s_CheckLeakingReportPeriod = (int)(period*1000);
		TRACE_INFO("\n SetCheckLeakingReportPeriod %d Ms", s_CheckLeakingReportPeriod);
		s_eventReported = TRUE;
    }
    else
    {
    	s_eventReported = FALSE;
    }
}

Uint16 CheckLeakingControl_GetAD(void)
{
	Uint16 result = 0;
	result = TempADCollect_GetAD(1);
	return result;

}
