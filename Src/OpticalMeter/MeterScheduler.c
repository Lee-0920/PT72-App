/*
 * MeterScheduler.c
 *
 *  Created on: 2016年6月23日
 *      Author: Administrator
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "SystemConfig.h"
#include "Common/Types.h"
#include "MeterScheduler.h"

static void MeterScheduler_TaskHandle(void *pvParameters);
static void MeterScheduler_MeterCheck(TimerHandle_t argument);
static void MeterScheduler_MeterADPeriod(TimerHandle_t argument);

static void (*MeterTaskHandle)(void) = NULL;
static void (*MeterCheckHandle)(void) = NULL;
static void (*MeterADPeriodHandle)(void) = NULL;
static xTaskHandle s_xHandle;
static xTimerHandle s_MeterCheckTimer;
static xTimerHandle s_MeterADNotifyPeriodTimer;

void MeterScheduler_Init(void)
{
    xTaskCreate(MeterScheduler_TaskHandle, "MeterHandle", METER_STK_SIZE, NULL,
            METER_TASK_PRIO, &s_xHandle);

    s_MeterCheckTimer = xTimerCreate("MeterCheack",
            (uint32_t) (2 / portTICK_RATE_MS), pdTRUE, (void *) METER_CHECK_TIMER_PRIO,
            MeterScheduler_MeterCheck);

    s_MeterADNotifyPeriodTimer = xTimerCreate("MeterADPeriod",
            (uint32_t) (1000 / portTICK_RATE_MS), pdTRUE, (void *) 2,
            MeterScheduler_MeterADPeriod);
}

static void MeterScheduler_TaskHandle(void *pvParameters)
{
    vTaskSuspend(NULL);
    while (1)
    {
        MeterTaskHandle();
    }
}

static void MeterScheduler_MeterCheck(TimerHandle_t argument)
{
    MeterCheckHandle();
}

static void MeterScheduler_MeterADPeriod(TimerHandle_t argument)
{
    MeterADPeriodHandle();
}

void MeterScheduler_RegisterTask(MeterHandle handle)
{
    MeterTaskHandle = handle;
}


void MeterScheduler_RegisterTimer(MeterHandle meterCheck, MeterHandle meterADPeriod)
{
    MeterCheckHandle = meterCheck;
    MeterADPeriodHandle = meterADPeriod;
}

void MeterScheduler_RegisterMeterCheck(MeterHandle meterCheck)
{
    MeterCheckHandle = meterCheck;
}

void MeterScheduler_MeterSuspend(void)
{
    vTaskSuspend(s_xHandle);
}

void MeterScheduler_MeterResume(void)
{
    vTaskResume(s_xHandle);
}

void MeterScheduler_StartMeterCheckTimer(void)
{
    xTimerStart(s_MeterCheckTimer, 0);
}

void MeterScheduler_StopMeterCheckTimer(void)
{
    xTimerStop(s_MeterCheckTimer, 0);
}

void MeterScheduler_SetMeterADReportPeriod(float period)
{
    if (period > 0)
    {
        xTimerChangePeriod(s_MeterADNotifyPeriodTimer,
                (uint32_t)((period * 1000) / portTICK_RATE_MS), 0);
        xTimerStart(s_MeterADNotifyPeriodTimer, 0);
    }
    else
    {
        xTimerStop(s_MeterADNotifyPeriodTimer, 0);
    }
}
