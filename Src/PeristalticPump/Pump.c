/*
 * Pump.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */
#include <LiquidDriver/PumpTimer.h>
#include <McuFlash.h>
#include "tracer/trace.h"
#include "DncpStack/DncpStack.h"
#include "LuipApi/PeristalticPumpInterface.h"
#include "SystemConfig.h"
#include "System.h"
#include "Pump.h"
#include "PumpEventScheduler.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define  EXTPUMP_OFFSET   2  //扩展泵的泵号相对Flash存储位置偏移

static void Pump_MoveHandler(Pump* pump);

void Pump_Init(Pump* pump)
{
    Uint8 buffer[PUMP_SIGN_FLASH_LEN] = { 0 };
    Uint32 flashFactorySign = 0;
    Uint32 baseAddr = PUMP_SIGN_FLASH_BASE_ADDR;

    pump->status = PUMP_IDLE;

    if(pump->number >= EXTPUMP_OFFSET)  //扩展的泵
    {
        McuFlash_Read(EXTPUMP_SIGN_FLASH_BASE_ADDR + PUMP_SIGN_FLASH_LEN * (pump->number-EXTPUMP_OFFSET), PUMP_SIGN_FLASH_LEN, buffer); //读取出厂标志位
    }
    else
    {
        McuFlash_Read(baseAddr + PUMP_SIGN_FLASH_LEN * pump->number, PUMP_SIGN_FLASH_LEN, buffer); //读取出厂标志位
    }

    memcpy(&flashFactorySign, buffer, PUMP_SIGN_FLASH_LEN);
    if (FLASH_FACTORY_SIGN == flashFactorySign) //表示已经过出厂设置
    {
        pump->factor = Pump_GetFactor(pump);
        pump->flashParam = Pump_GetMotionParam(pump);
    }
    else
    {
        PumpParam param;
        float factor;

        param.acceleration = 800;
        param.maxSpeed = 200;
        Pump_SetMotionParam(pump, param, WriteFLASH);

        factor = 0.000761;
        Pump_SetFactor(pump, factor);

        flashFactorySign = FLASH_FACTORY_SIGN;
        memcpy(buffer, &flashFactorySign, PUMP_SIGN_FLASH_LEN);
        if(pump->number >= EXTPUMP_OFFSET)  //扩展的泵
        {
            McuFlash_Write(EXTPUMP_SIGN_FLASH_BASE_ADDR + PUMP_SIGN_FLASH_LEN * (pump->number-EXTPUMP_OFFSET), PUMP_SIGN_FLASH_LEN, buffer);
        }
        else
        {
            McuFlash_Write(PUMP_SIGN_FLASH_BASE_ADDR + PUMP_SIGN_FLASH_LEN * pump->number, PUMP_SIGN_FLASH_LEN, buffer);
        }
    }

    pump->isSendEvent = FALSE;
    pump->alreadyVolume = 0;

    pump->remainStep = 0;
    pump->alreadyStep = 0;
    pump->LiquidVolume = 0;
    pump->AirVolume = 0;
    pump->speedDownTotalStep = 0;

    pump->moveStatus = MOVE_STATUS_IDLE;
    pump->isStatusSwitchStart = FALSE;

    pump->result = PUMP_RESULT_FINISHED;
    pump->isParamChange = FALSE;
    pump->isExcessiveSpeed = FALSE;
    pump->isLowSpeed = FALSE;
    //加速逻辑
    pump->upSpeed = 0;
    pump->alreadyUpTimer = 0;
    pump->upParam = pump->flashParam;

    pump->upInitSpeed = 0;
    pump->alreadyUpStep = 0;

    //减速逻辑
    pump->dowmSpeed = 0;
    pump->alreadyDownTimer = 0;
    pump->downParam = pump->flashParam;

    pump->downInitSpeed = 0;
    pump->alreadyDownStep = 0;

    pump->uniformSpeed = 0;
    pump->uniformParam = pump->flashParam;

}

void Pump_SetNumber(Pump* pump, Uint8 num)
{
    pump->number = num;
}
void Pump_SetSubdivision(Pump* pump, Uint8 subdivision)
{
    pump->subdivision = subdivision;
}

Uint8 Pump_GetSubdivision(Pump* pump)
{
    return (pump->subdivision);
}

float Pump_GetVolume(Pump* pump)
{
    pump->alreadyVolume = (float) (pump->alreadyStep * 1.0 / pump->subdivision
                * pump->factor);
    return (pump->alreadyVolume);
}

float Pump_GetAirVolume(Pump* pump)
{
	float AirVolume  = (float)(pump->AirVolume*1.0/pump->subdivision * pump->factor);
	return AirVolume;
}

Uint32 Pump_GetAlreadyStep(Pump* pump)
{
    return (pump->alreadyStep);
}

PumpParam Pump_GetMotionParam(Pump* pump)
{
    Uint8 readData[PUMP_MOTIONPARAM_FLASH_LEN] ={ 0 };
    PumpParam param;

    if(pump->number >= EXTPUMP_OFFSET)  //扩展的泵
    {
        //读取FLASH里的校准系数
        McuFlash_Read(EXTPUMP_MOTIONPARAM_FLASH_BASE_ADDR + PUMP_MOTIONPARAM_FLASH_LEN * (pump->number-EXTPUMP_OFFSET),
                PUMP_MOTIONPARAM_FLASH_LEN, readData);
    }
    else
    {
        //读取FLASH里的校准系数
        McuFlash_Read(PUMP_MOTIONPARAM_FLASH_BASE_ADDR + PUMP_MOTIONPARAM_FLASH_LEN * pump->number,
                PUMP_MOTIONPARAM_FLASH_LEN, readData);
    }
    memcpy(&param, readData, sizeof(PumpParam));
    return (param);
}
/**
 * @brief 设置电机的运动参数,电机按此设置的参数启动,并永久存在FLASH
 * @param pump
 * @param param
 */
Bool Pump_SetMotionParam(Pump* pump, PumpParam param, ParamFLashOp flashOp)
{
    Uint8 writeData[PUMP_MOTIONPARAM_FLASH_LEN] ={ 0 };

    TRACE_MARK("  ");
    System_PrintfFloat(TRACE_LEVEL_MARK, param.maxSpeed, 4);
    TRACE_MARK(" step/s,acc:");
    System_PrintfFloat(TRACE_LEVEL_MARK, param.acceleration, 4);
    TRACE_MARK(" step/(s^2)\n");

    if (WriteFLASH == flashOp && PUMP_IDLE == pump->status)
    {
        pump->flashParam = param;
        //运动参数写入FLASH
        memcpy(writeData, &param, sizeof(PumpParam));
        if(pump->number >= EXTPUMP_OFFSET)  //扩展的泵
        {
            McuFlash_Write(
                    EXTPUMP_MOTIONPARAM_FLASH_BASE_ADDR + PUMP_MOTIONPARAM_FLASH_LEN * (pump->number-EXTPUMP_OFFSET),
                    PUMP_MOTIONPARAM_FLASH_LEN, writeData);
        }
        else
        {
            McuFlash_Write(
                    PUMP_MOTIONPARAM_FLASH_BASE_ADDR + PUMP_MOTIONPARAM_FLASH_LEN * pump->number,
                    PUMP_MOTIONPARAM_FLASH_LEN, writeData);
        }
        return TRUE;
    }
    else if (NoWriteFLASH == flashOp)
    {
        pump->isParamChange = TRUE;
        pump->setParam = param;
        return TRUE;
    }
    TRACE_ERROR("\n The pump is running, prohibiting the change of the flash to save the movement parameters.");
    return FALSE;
}

float Pump_GetFactor(Pump* pump)
{
    Uint8 readData[PUMP_FACTOR_FLASH_LEN] ={ 0 };
    float factor;

    if(pump->number >= EXTPUMP_OFFSET)  //扩展的泵
    {
        //获取FLASH的系数
        McuFlash_Read(EXTPUMP_FACTOR_FLASH_BASE_ADDR + PUMP_FACTOR_FLASH_LEN * (pump->number-EXTPUMP_OFFSET),
                PUMP_FACTOR_FLASH_LEN, readData);
    }
    else
    {
        //获取FLASH的系数
        McuFlash_Read(PUMP_FACTOR_FLASH_BASE_ADDR + PUMP_FACTOR_FLASH_LEN * pump->number,
                PUMP_FACTOR_FLASH_LEN, readData);
    }
    memcpy(&factor, readData, sizeof(float));
    pump->factor = factor;
    return (pump->factor);
}

float Pump_GetTempFactor(Pump* pump)
{
    return pump->factor;
}

void Pump_SetFactor(Pump* pump, float factor)
{
    Uint8 writeData[PUMP_FACTOR_FLASH_LEN] ={ 0 };

    pump->factor = factor;
    //系数写入FLASH
    memcpy(writeData, &factor, sizeof(float));
    if(pump->number >= EXTPUMP_OFFSET)  //扩展的泵
    {
        McuFlash_Write(
                EXTPUMP_FACTOR_FLASH_BASE_ADDR + PUMP_FACTOR_FLASH_LEN * (pump->number-EXTPUMP_OFFSET),
                PUMP_FACTOR_FLASH_LEN, writeData);
    }
    else
    {
        McuFlash_Write(
                PUMP_FACTOR_FLASH_BASE_ADDR + PUMP_FACTOR_FLASH_LEN * pump->number,
                PUMP_FACTOR_FLASH_LEN, writeData);
    }
}

void Pump_SetTempFactor(Pump* pump, float factor)
{
    pump->factor = factor;
}


PumpStatus Pump_GetStatus(Pump* pump)
{
    return (pump->status);
}

static void Pump_MoveStatusSwitch(Pump* pump, PumpMoveStatus moveStatus)
{
    // 参数检查
    if (NULL == pump)
    {
        return;
    }
    // 状态切换
    pump->moveStatus = moveStatus;
    pump->isStatusSwitchStart = TRUE;
}

static void Pump_MoveStatusSwitchEnd(Pump* pump)
{
    // 参数检查
    if (NULL == pump)
    {
        return;
    }
    // 状态切换
    pump->isStatusSwitchStart = FALSE;
}

Bool Pump_Start(Pump* pump, Direction dir, float volume, ParamFLashOp flashOp)
{
    if ( FALSE == PumpTimer_RegisterHandle((TimerHandle) Pump_MoveHandler, pump))
    {
        return FALSE;
    }

    Pump_MoveStatusSwitch(pump, MOVE_STATUS_START);

    pump->remainStep =
            (Uint32) (volume * 1.0 / pump->factor * pump->subdivision);

    pump->status = PUMP_BUSY;

    pump->alreadyVolume = 0;
    pump->alreadyStep = 0;

    pump->LiquidVolume = 0;
    pump->AirVolume = 0;

    pump->speedDownTotalStep = 0;

    pump->isParamChange = FALSE;
    pump->isExcessiveSpeed = FALSE;
    pump->isLowSpeed = FALSE;
    //加速逻辑
    pump->upSpeed = 0;
    pump->alreadyUpTimer = 0;

    pump->upInitSpeed = 0;
    pump->alreadyUpStep = 0;

    //减速逻辑
    pump->dowmSpeed = 0;
    pump->alreadyDownTimer = 0;

    pump->downInitSpeed = 0;
    pump->alreadyDownStep = 0;
    pump->uniformSpeed = 0;

    pump->result = PUMP_RESULT_FINISHED;

    if(ReadFLASH == flashOp)
    {
        pump->upParam = pump->flashParam;
        pump->downParam = pump->flashParam;
        pump->uniformParam = pump->flashParam;
    }
    else if(NoReadFLASH == flashOp)
    {
        pump->upParam = pump->setParam;
        pump->downParam = pump->setParam;
        pump->uniformParam = pump->setParam;
        pump->isParamChange = FALSE;
    }
    PumpDriver_SetDirection(&pump->driver, dir);
    PumpTimer_SpeedSetting(pump, PUMP_INIT_SPEED * pump->subdivision);

    DncpStack_ClearBufferedEvent();

    PumpDriver_Enable(&pump->driver);
    PumpTimer_Start(pump);

//    TRACE_INFO("\n ###########Pump %d", pump->number);
//    TRACE_INFO("\n ###########Pump_Start");
//    TRACE_INFO("\n ***********remainStep %d, dir %d,volume ", pump->remainStep, dir);
//    System_PrintfFloat(TRACE_LEVEL_INFO, volume, 3);
//    TRACE_INFO(" ml,maxSpeed:");
//    System_PrintfFloat(TRACE_LEVEL_INFO, pump->upParam.maxSpeed, 4);
//    TRACE_INFO(" step/s,acc:");
//    System_PrintfFloat(TRACE_LEVEL_INFO, pump->upParam.acceleration, 4);
//    TRACE_INFO(" step/(s^2)");
    return TRUE;
}
/**
 * @brief 计算减速到初始速度需要的步数，保存到pump.speedDownTotalStep
 * @note 根据t = (V - V0) / a 和 s = V * t - 1/2 * a * t^2 公式算出s即步数。
 *  V是当前速度,步/秒。 V0是本系统泵启动的初始速度(运行后不能改变)。a是设置的加速度，步/平方秒。
 *  速度和加速度都是没有细分的，剩余步数是细分后的，算出s后需要乘以细分系数。
 *  减速步数是状态切换的判断条件，切换状态前必须为最新加速度和当前速度计算出来的剩余步数
 * @param pump 泵实体
 */
static void Pump_UpdateDownTotalStep(Pump* pump)
{
    float speed, acceleration, downTimer;
    switch (pump->moveStatus)
    {
    case MOVE_STATUS_SPEED_NOT_CHANGE:
        speed = pump->uniformSpeed;
        acceleration = pump->uniformParam.acceleration;
        break;
    case MOVE_STATUS_SPEED_UP:
        speed = pump->upSpeed;
        acceleration = pump->upParam.acceleration;
        break;
    case MOVE_STATUS_PEED_DOWM:
        speed = pump->dowmSpeed;
        acceleration = pump->downParam.acceleration;
        break;
    }
    downTimer = (speed - PUMP_INIT_SPEED) * 1.0 / acceleration;
    pump->speedDownTotalStep =
            (Uint32) ((speed - 0.5 * acceleration * downTimer) * downTimer
                    * pump->subdivision);
}

Bool Pump_RequestStop(Pump* pump)
{
    switch (pump->moveStatus)
    {
    case MOVE_STATUS_SPEED_NOT_CHANGE:
        Pump_MoveStatusSwitch(pump, MOVE_STATUS_PEED_DOWM);

        pump->dowmSpeed = pump->uniformSpeed;
        pump->downInitSpeed = pump->uniformSpeed;
        pump->downParam = pump->uniformParam;
        break;

    case MOVE_STATUS_SPEED_UP:
        Pump_MoveStatusSwitch(pump, MOVE_STATUS_PEED_DOWM);

        pump->dowmSpeed = pump->upSpeed;
        pump->downInitSpeed = pump->upSpeed;
        pump->downParam = pump->upParam;

        break;
    case MOVE_STATUS_PEED_DOWM:

        break;
    default:
        return FALSE;
    }

    pump->result = PUMP_RESULT_STOPPED;
    pump->isExcessiveSpeed = FALSE;
    pump->isLowSpeed = FALSE;
    Pump_UpdateDownTotalStep(pump);
    pump->remainStep = pump->speedDownTotalStep;
    TRACE_INFO("\n ###########Pump %d", pump->number);
    TRACE_INFO("\n ###########Pump_RequestStop");
    return TRUE;
}

static Bool Pump_Stop(Pump* pump)
{
    PumpTimer_Stop(pump);
    PumpDriver_PullLow(&pump->driver);
    PumpDriver_Disable(&pump->driver);

    if (FALSE == PumpTimer_CancelRegisterHandle(pump))
    {
        return FALSE;
    }
    pump->alreadyVolume = (float) (pump->alreadyStep * 1.0 / pump->subdivision
            * pump->factor);
    TRACE_DEBUG("\n ***********alreadyStep = %d", pump->alreadyStep);
    TRACE_MARK("\n ***********alreadyDownStep = %d alreadyUpStep = %d",
            pump->alreadyDownStep, pump->alreadyUpStep);
    TRACE_MARK("\n ***********dowmSpeed:");
    System_PrintfFloat(TRACE_LEVEL_MARK, pump->dowmSpeed, 4);
    TRACE_MARK(" step/s");

    pump->remainStep = 0;
    pump->speedDownTotalStep = 0;

    pump->isStatusSwitchStart = FALSE;

    pump->isParamChange = FALSE;
    pump->isExcessiveSpeed = FALSE;
    pump->isLowSpeed = FALSE;
    //加速逻辑
    pump->upSpeed = 0;
    pump->alreadyUpTimer = 0;
    pump->upParam = pump->flashParam;
    pump->upInitSpeed = 0;
    pump->alreadyUpStep = 0;

    //减速逻辑
    pump->dowmSpeed = 0;
    pump->alreadyDownTimer = 0;
    pump->downParam = pump->flashParam;

    pump->downInitSpeed = 0;
    pump->alreadyDownStep = 0;

    pump->uniformSpeed = 0;
    pump->uniformParam = pump->flashParam;

//    TRACE_INFO("\n ###########Pump_Stop");
//    TRACE_INFO("\n ***********lastVolume:");
//    System_PrintfFloat(TRACE_LEVEL_INFO, pump->alreadyVolume, 4);
//    TRACE_INFO(" ml");

    pump->status = PUMP_IDLE;
    pump->moveStatus = MOVE_STATUS_IDLE;
    pump->isStatusSwitchStart = FALSE;
    return TRUE;
}

void Pump_ChangeVolume(Pump* pump, float volume)
{
    Pump_GetVolume(pump);//更新已泵取体积。
    if (volume > pump->alreadyVolume)
    {
        pump->remainStep =
                (Uint32) (volume * 1.0 / pump->factor * pump->subdivision) - pump->alreadyStep;
        TRACE_DEBUG("\n ###########ChangeVolume remainStep: %d volume:", pump->remainStep);
        System_PrintfFloat(TRACE_LEVEL_DEBUG, volume, 4);
        TRACE_DEBUG(" ml");
    }
    else
    {
        Pump_RequestStop(pump);
    }
}

void Pump_SendEventOpen(Pump* pump)
{
    pump->isSendEvent = TRUE;
}

void Pump_SendEventClose(Pump* pump)
{
    pump->isSendEvent = FALSE;
}

static void Pump_SendEvent(Pump* pump)
{
    if(TRUE == pump->isSendEvent)
    {
        Uint8 data[10] = {0};
        data[0] = pump->number;
        memcpy(data+sizeof(Uint8), &pump->result, sizeof(pump->result));
        // DncpStack_SendEvent(DSCP_EVENT_PPI_PUMP_RESULT, data , sizeof(Uint8) + sizeof(pump->result));

        PumpEventScheduler_SendEvent(pump->number, pump->result, pump->isSendEvent);
        DncpStack_BufferEvent(DSCP_EVENT_PPI_PUMP_RESULT, data , sizeof(Uint8) + sizeof(pump->result));
    }
    pump->isSendEvent = FALSE;
}


Bool Pump_ImmediatelyStop(Pump* pump)
{
	TRACE_INFO("\n Pump Immediately Stop\n");
	return Pump_Stop(pump);
}

static void Pump_PrintfInfo(Pump* pump)
{
    TRACE_CODE(
            "\n ***********remainStep = %d alreadyUpStep = %d speedDownTotalStep = %d",
            pump->remainStep, pump->alreadyUpStep, pump->speedDownTotalStep);

    TRACE_CODE("\n ***********upSpeed:");
    System_PrintfFloat(TRACE_LEVEL_CODE, pump->upSpeed, 4);
    TRACE_CODE(" step/s");
    TRACE_CODE("  uniformSpeed:");
    System_PrintfFloat(TRACE_LEVEL_CODE, pump->uniformSpeed, 4);
    TRACE_CODE(" step/s");
    TRACE_CODE("  dowmSpeed:");
    System_PrintfFloat(TRACE_LEVEL_CODE, pump->dowmSpeed, 4);
    TRACE_CODE(" step/s");
}

extern Bool Meter_GetSensor(Uint8 index);

void Pump_MoveHandler(Pump* pump)
{
    // 参数检查
    if ((NULL == pump) || (0 == pump->remainStep))
    {
        pump->moveStatus = MOVE_STATUS_IDLE;
        Pump_Stop(pump);
        Pump_SendEvent(pump);
        return;
    }

    switch (pump->moveStatus)
    {

    case MOVE_STATUS_IDLE:
        Pump_Stop(pump);
        Pump_SendEvent(pump);
        break;

    case MOVE_STATUS_START:
        if (TRUE == pump->isStatusSwitchStart)
        {
            Pump_MoveStatusSwitchEnd(pump);

            pump->upSpeed = PUMP_INIT_SPEED;
            pump->upInitSpeed = PUMP_INIT_SPEED;

            Pump_MoveStatusSwitch(pump, MOVE_STATUS_SPEED_UP);
            TRACE_DEBUG("\n ###########START TO SPEED_UP");

            PumpDriver_PullLow(&pump->driver);
        }
        break;

    case MOVE_STATUS_SPEED_UP:
        //速度计算设置逻辑================================================================================================================
        if (TRUE == pump->isStatusSwitchStart)
        {
            Pump_MoveStatusSwitchEnd(pump);
            pump->alreadyUpTimer = 0; //每次进来的加速度或者速度可能不一样，Vt=a*t+V0直线不一样，t重新计数
        }
        else
        {
            //加速度或速度改变逻辑************************************************************************************
            if (TRUE == pump->isParamChange)
            {
                pump->isParamChange = FALSE;
                TRACE_CODE("\n @@@@@ParamSet");
                //加速度改变，设置初始速度为当前速度和清零加速时间
                if (pump->upParam.acceleration != pump->setParam.acceleration)
                {
                    pump->upInitSpeed = pump->upSpeed;
                    pump->alreadyUpTimer = 0;
                    TRACE_CODE("acc");
                }
                //当前速度大于最大速度，设置超速标志量
                if (pump->upSpeed > pump->setParam.maxSpeed)
                {
                    pump->isExcessiveSpeed = TRUE;
                    pump->isLowSpeed = FALSE; //把上一次因速度过低进入加速的标志量去掉
                    TRACE_CODE("ExcessiveSpeed");
                }
                pump->upParam = pump->setParam;
            }
            pump->alreadyUpStep++;
            //根据Vt=a*t+V0算出本脉冲的速度即频率(Vt是当前速度, a是加速度, t是使用此加速度的加速时间，V0是此加速度的)
            pump->alreadyUpTimer += 1.0 / (pump->upSpeed * pump->subdivision); //计算累积加速时间
            pump->upSpeed = pump->upParam.acceleration * pump->alreadyUpTimer
                    + pump->upInitSpeed; //计算设置速度
        }
        PumpTimer_SpeedSetting(pump, pump->upSpeed * pump->subdivision); //设置细分后的速度

        //输出脉冲和整体状态计数逻辑=======================================================================================================
        PumpDriver_PullHigh(&pump->driver);
        pump->alreadyStep++;
        pump->remainStep--;
		if (Meter_GetSensor(0))
		{
			pump->LiquidVolume++;
		}
		else
		{
			pump->AirVolume++;
		}
        for (int i = 0; i < 100; i++)
            ;
        PumpDriver_PullLow(&pump->driver);
        Pump_UpdateDownTotalStep(pump);
        //状态切换逻辑====================================================================================================================
        //如果减速步数等于小于剩余步数则进入减速阶段(加速度减少时可能出现小于情况)
        if (pump->speedDownTotalStep >= pump->remainStep)
        {
            pump->dowmSpeed = pump->upSpeed;
            pump->downInitSpeed = pump->upSpeed;
            pump->downParam = pump->upParam;

            pump->isExcessiveSpeed = FALSE;
            pump->isLowSpeed = FALSE;

            Pump_MoveStatusSwitch(pump, MOVE_STATUS_PEED_DOWM);
            TRACE_DEBUG(
                    "\n ###########DownTotalStep = remainStep SPEED_UP TO PEED_DOWM");
            Pump_PrintfInfo(pump);
        }
        //如果下一次再加速剩余步数不足以减速则进入匀速阶段
        else if (pump->speedDownTotalStep == (pump->remainStep - 1))
        {
            pump->uniformSpeed = pump->upSpeed;
            pump->uniformParam = pump->upParam;

            pump->isExcessiveSpeed = FALSE;
            pump->isLowSpeed = FALSE;

            Pump_MoveStatusSwitch(pump, MOVE_STATUS_SPEED_NOT_CHANGE);
            TRACE_DEBUG(
                    "\n ###########DownTotalStep = remainStep-1 SPEED_UP TO PEED_DOWM");
            Pump_PrintfInfo(pump);
        }
        else if (pump->upSpeed >= pump->upParam.maxSpeed) //当前速度大于等于最大速度(当前速度有很大几率大于最大速度)
        {
            if (TRUE == pump->isExcessiveSpeed)        //由设置最大速度引起的，需要进入减速
            {
                Pump_MoveStatusSwitch(pump, MOVE_STATUS_PEED_DOWM);

                pump->dowmSpeed = pump->upSpeed;
                pump->downInitSpeed = pump->upSpeed;
                pump->downParam = pump->upParam;

                TRACE_DEBUG(
                        "\n ###########isExcessiveSpeed =TRUE SPEED_UP TO PEED_DOWM");
                Pump_PrintfInfo(pump);
            }
            else        //加速到最大速度，进入匀速
            {
                pump->uniformSpeed = pump->upSpeed;
                pump->uniformParam = pump->upParam;
                pump->isLowSpeed = FALSE;

                Pump_MoveStatusSwitch(pump, MOVE_STATUS_SPEED_NOT_CHANGE);
                TRACE_DEBUG(
                        "\n ###########speed >= maxSpeed SPEED_UP TO NOT_CHANGE");
                Pump_PrintfInfo(pump);
            }
        }
        break;

    case MOVE_STATUS_SPEED_NOT_CHANGE:
        //速度计算设置逻辑================================================================================================================
        if (TRUE == pump->isStatusSwitchStart)
        {
            Pump_MoveStatusSwitchEnd(pump);
        }
        //加速度或速度改变逻辑************************************************************************************
        if (TRUE == pump->isParamChange)
        {
            pump->isParamChange = FALSE;
            TRACE_CODE("\n @@@@@ParamSet");
            //最大速度改变，设置超速或者慢速标志
            if (pump->uniformParam.maxSpeed != pump->setParam.maxSpeed)
            {
                if (pump->uniformSpeed > pump->setParam.maxSpeed)
                {
                    pump->isExcessiveSpeed = TRUE;
                    pump->isLowSpeed = FALSE;
                    TRACE_CODE("ExcessiveSpeed");
                }
                else if (pump->uniformSpeed < pump->setParam.maxSpeed)
                {
                    pump->isLowSpeed = TRUE;
                    pump->isExcessiveSpeed = FALSE;
                    TRACE_CODE("LowSpeed");
                }
            }
            pump->uniformParam = pump->setParam;
            Pump_UpdateDownTotalStep(pump);        //在加速度改变后重新计算剩余步数即可
        }

        //输出脉冲和整体状态计数逻辑=======================================================================================================
        PumpDriver_PullHigh(&pump->driver);
        pump->alreadyStep++;
        pump->remainStep--;
        if (Meter_GetSensor(0))
		{
			pump->LiquidVolume++;
		}
		else
		{
			pump->AirVolume++;
		}

        for (int i = 0; i < 100; i++)
            ;
        PumpDriver_PullLow(&pump->driver);

        //状态切换逻辑====================================================================================================================
        if (pump->speedDownTotalStep >= pump->remainStep)
        {
            Pump_MoveStatusSwitch(pump, MOVE_STATUS_PEED_DOWM);

            pump->dowmSpeed = pump->uniformSpeed;
            pump->downInitSpeed = pump->uniformSpeed;
            pump->downParam = pump->uniformParam;

            pump->isExcessiveSpeed = FALSE;
            pump->isLowSpeed = FALSE;
            TRACE_DEBUG(
                    "\n ###########UpTotalStep == remainStep NOT_CHANGE TO PEED_DOWM");
            Pump_PrintfInfo(pump);
        }
        else if (TRUE == pump->isExcessiveSpeed)        //需要减速
        {
            Pump_MoveStatusSwitch(pump, MOVE_STATUS_PEED_DOWM);

            pump->dowmSpeed = pump->uniformSpeed;
            pump->downInitSpeed = pump->uniformSpeed;
            pump->downParam = pump->uniformParam;

            TRACE_DEBUG(
                    "\n ###########isExcessiveSpeed =TRUE NOT_CHANGE TO PEED_DOWM");
            Pump_PrintfInfo(pump);
        }
        else if ((TRUE == pump->isLowSpeed))        //需要加速
        {
            Pump_MoveStatusSwitch(pump, MOVE_STATUS_SPEED_UP);

            pump->upSpeed = pump->uniformSpeed;
            pump->upInitSpeed = pump->uniformSpeed;
            pump->upParam = pump->uniformParam;

            TRACE_DEBUG("\n ###########isLowSpeed =TRUE NOT_CHANGE TO PEED_UP");
            Pump_PrintfInfo(pump);
        }
        break;

    case MOVE_STATUS_PEED_DOWM:
        //速度计算设置逻辑================================================================================================================
        if (TRUE == pump->isStatusSwitchStart)
        {
            Pump_MoveStatusSwitchEnd(pump);
            pump->alreadyDownTimer = 0;
        }
        //加速度或速度改变逻辑************************************************************************************
        if (TRUE == pump->isParamChange)
        {
            pump->isParamChange = FALSE;
            TRACE_CODE("\n @@@@@ParamSet");
            //加速度改变，设置初始速度为当前速度和清零加速时间
            if (pump->downParam.acceleration != pump->setParam.acceleration)
            {
                pump->downInitSpeed = pump->dowmSpeed;
                pump->alreadyDownTimer = 0;
                TRACE_CODE("acc");
            }
            //当前速度小于最大速度，设置慢速标志量
            if (pump->dowmSpeed < pump->setParam.maxSpeed)
            {
                pump->isLowSpeed = TRUE;
                pump->isExcessiveSpeed = FALSE;
                TRACE_CODE("isLowSpeed");
            }
            pump->downParam = pump->setParam;
        }
        if (pump->dowmSpeed > PUMP_INIT_SPEED)
        {
            pump->alreadyDownStep++;
            pump->alreadyDownTimer += 1.0
                    / (pump->dowmSpeed * pump->subdivision); //计算累积减速时间
            pump->dowmSpeed = pump->downInitSpeed
                    - pump->downParam.acceleration * pump->alreadyDownTimer; //根据Vt=V0-a*t算出本脉冲的速度即频率
            if(pump->dowmSpeed < PUMP_INIT_SPEED)
            {
                pump->dowmSpeed = PUMP_INIT_SPEED;
            }
            PumpTimer_SpeedSetting(pump, pump->dowmSpeed * pump->subdivision);
        }

        //输出脉冲和整体状态计数逻辑=======================================================================================================
        PumpDriver_PullHigh(&pump->driver);
        pump->alreadyStep++;
        pump->remainStep--;
        if (Meter_GetSensor(0))
        {
        	pump->LiquidVolume++;
        }
        else
        {
        	pump->AirVolume++;
        }
        for (int i = 0; i < 100; i++)
            ;
        PumpDriver_PullLow(&pump->driver);
        Pump_UpdateDownTotalStep(pump);
        //状态切换逻辑====================================================================================================================
        if (0 == pump->remainStep)
        {
            Pump_MoveStatusSwitch(pump, MOVE_STATUS_IDLE);
            TRACE_DEBUG("\n ########### 0 == pump->remainStep ");
            Pump_Stop(pump);
            Pump_SendEvent(pump);
        }
        else if (pump->speedDownTotalStep < pump->remainStep)
        {
            //减速完成
            if (TRUE == pump->isExcessiveSpeed
                    && pump->dowmSpeed <= pump->downParam.maxSpeed)
            {
                pump->isExcessiveSpeed = FALSE;

                Pump_MoveStatusSwitch(pump, MOVE_STATUS_SPEED_NOT_CHANGE);
                pump->uniformSpeed = pump->dowmSpeed;
                pump->uniformParam = pump->downParam;

                TRACE_DEBUG(
                        "\n ###########isExcessiveSpeed = FALSE  SPEED_DOWM TO NOT_CHANGE");
                Pump_PrintfInfo(pump);
            }
            else if (TRUE == pump->isLowSpeed) //需要加速
            {
                Pump_MoveStatusSwitch(pump, MOVE_STATUS_SPEED_UP);

                pump->upSpeed = pump->dowmSpeed;
                pump->upInitSpeed = pump->dowmSpeed;
                pump->upParam = pump->downParam;

                TRACE_DEBUG(
                        "\n ###########isLowSpeed = TRUE  SPEED_DOWM TO SPEED_UP");
                Pump_PrintfInfo(pump);
            }
        }
        break;
    default:
        break;
    }
}

void Pump_Reset(Pump* pump)
{
	PumpDriver_Reset(&pump->driver);
	TRACE_DEBUG("\n ###########Reset pump \n");
}

