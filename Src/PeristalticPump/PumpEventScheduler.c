/*
 * PumpEventScheduler.c
 *
 *  Created on: 2016年5月3日
 *      Author: Administrator
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "DncpStack/DncpStack.h"
#include "systemConfig.h"
#include "PumpEventScheduler.h"

static xSemaphoreHandle s_semPumpEvent;
static PumpEvent s_pumpEvents[PUMPMANAGER_TOTAL_PUMP];
static void PumpEventScheduler_TaskHandle(void *pvParameters);

/**
 * @brief
 * @details
 */
void PumpEventScheduler_Init(void)
{
    memset(s_pumpEvents, 0, sizeof(s_pumpEvents));

    s_semPumpEvent = xSemaphoreCreateBinary();

    xTaskCreate(PumpEventScheduler_TaskHandle, "PumpEvent",
                PUMP_EVENT_STK_SIZE, NULL, PUMP_EVENT_TASK_PRIO, NULL);
}

static void PumpEventScheduler_TaskHandle(void *pvParameters)
{
    Uint8 data[10] = {0};

    while (1)
    {
        // xSemaphoreTake(s_semPumpEvent, 10);

        System_Delay(10);

        for(int i = 0; i < PUMPMANAGER_TOTAL_PUMP; i++)
        {
            if (TRUE == s_pumpEvents[i].isSendEvent)
            {
                memset(data, 0, sizeof(data));

                data[0] = s_pumpEvents[i].number;
                memcpy(data+sizeof(Uint8), &(s_pumpEvents[i].result), sizeof(s_pumpEvents[i].result));

                DncpStack_SendEvent(DSCP_EVENT_PPI_PUMP_RESULT, data , sizeof(Uint8) + sizeof(s_pumpEvents[i].result));

                s_pumpEvents[i].isSendEvent = FALSE;
            }
        }

        // PumpEventScheduler_SendEvent(0, PUMP_RESULT_FINISHED, TRUE);
        // PumpEventScheduler_SendEvent(1, PUMP_RESULT_FINISHED, TRUE);
        // PumpEventScheduler_SendEvent(2, PUMP_RESULT_FINISHED, TRUE);
        // PumpEventScheduler_SendEvent(3, PUMP_RESULT_FINISHED, TRUE);
    }
}

void PumpEventScheduler_SendEvent(Uint8 pumpNumber, enum PumpResult pumpResult, Bool isSendEvent)
{
    s_pumpEvents[pumpNumber].number = pumpNumber;
    s_pumpEvents[pumpNumber].result = pumpResult;
    s_pumpEvents[pumpNumber].isSendEvent = isSendEvent;
}
