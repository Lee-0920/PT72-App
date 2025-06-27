/*
 * Pump.h
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */

#ifndef SRC_PERISTALTICPUMP_PUMP_H_
#define SRC_PERISTALTICPUMP_PUMP_H_

#include <LiquidDriver/PumpDriver.h>
#include "Common/Types.h"
#include "LuipApi/PeristalticPumpInterface.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define PUMP_INIT_SPEED 20
#define PUMP_MAX_SPEED  1000
/**
 * @brief 泵的忙碌状态
 */
typedef enum
{
    PUMP_IDLE,
    PUMP_BUSY
} PumpStatus;


/**
 * @brief 泵运动参数
 */
typedef struct
{
    float acceleration;        //单位为 步/平方秒 没有细分
    float maxSpeed;           //单位为步/秒 没有细分（注:上位机发送过来的ml/秒的最大速度会在PumpManager上转换）
} PumpParam;

typedef enum
{
    WriteFLASH,
    NoWriteFLASH,
    ReadFLASH,
    NoReadFLASH
}ParamFLashOp;

/**
 * @brief 泵转动的状态机
 */
typedef enum
{
    MOVE_STATUS_IDLE,                // 空闲
    MOVE_STATUS_START,               // 开始
    MOVE_STATUS_SPEED_UP,            // 加速
    MOVE_STATUS_SPEED_NOT_CHANGE,    // 匀速
    MOVE_STATUS_PEED_DOWM,           // 减速
    MAX_MOVE_STATUS
} PumpMoveStatus;

/**
 * @brief 泵的操作方向
 */
typedef enum
{
    SUCK,                   //抽
    DRAIN,                  //排
    MAX_OPERATE_DIRECTION
} OperateDirection;

/**
 * @brief 泵实体
 */
typedef struct
{
    PumpDriver driver;               //泵的驱动引脚

    //基本参数
    Uint8 number;
    float factor;                    //泵的校准系数 存储在FLASH
    float alreadyVolume;                //泵上一次转动的体积
    PumpStatus status;               //泵的忙碌状态
    PumpParam flashParam;
    Bool isSendEvent;

    //运动状态共用逻辑
    Uint8 subdivision;               //泵的细分系数
    Uint32 remainStep;               //泵的剩余步数  已细分
    Uint32 alreadyStep;              //泵已转动的步数 已细分
    Uint32 AirVolume;				 //空气步数
    Uint32 LiquidVolume;			 //液体步数
    Uint32 speedDownTotalStep;
    __IO PumpParam setParam;

    __IO PumpMoveStatus moveStatus;       //泵转动的状态机
    __IO Bool isStatusSwitchStart;        //泵状态切换标志，TRUE为开始切换

    enum PumpResult result;
    Bool isParamChange;
    Bool isExcessiveSpeed;
    Bool isLowSpeed;
    //加速逻辑
    float upSpeed;
    float alreadyUpTimer;                   //加速阶段已过的时间
    PumpParam upParam;
    float upInitSpeed;              //加速阶段的初始速度 没有细分
    Uint32 alreadyUpStep;         //泵加速总步数

    //减速逻辑
    float dowmSpeed;
    float alreadyDownTimer;                 //减速阶段已过的时间
    PumpParam downParam;
    float downInitSpeed;             //减速阶段的初始速度 没有细分
    Uint32 alreadyDownStep;

    //匀速逻辑
    float uniformSpeed;
    PumpParam uniformParam;
} Pump;

void Pump_Init(Pump* pump);
void Pump_SetNumber(Pump* pump, Uint8 num);
void Pump_SetSubdivision(Pump* pump, Uint8 subdivision);
Uint8 Pump_GetSubdivision(Pump* pump);
float Pump_GetFactor(Pump* pump);
float Pump_GetTempFactor(Pump* pump);
Bool Pump_ImmediatelyStop(Pump* pump);
PumpParam Pump_GetMotionParam(Pump* pump);
Bool Pump_SetMotionParam(Pump* pump, PumpParam param, ParamFLashOp flashOp);
void Pump_SetFactor(Pump* pump, float factor);
void Pump_SetTempFactor(Pump* pump, float factor);
PumpStatus Pump_GetStatus(Pump* pump);
float Pump_GetVolume(Pump* pump);
float Pump_GetAirVolume(Pump* pump);
Uint32 Pump_GetAlreadyStep(Pump* pump);
Bool Pump_Start(Pump* pump, Direction dir, float volume, ParamFLashOp flashOp);
Bool Pump_RequestStop(Pump* pump);
void Pump_ChangeVolume(Pump* pump, float volume);
void Pump_SendEventOpen(Pump* pump);
void Pump_SendEventClose(Pump* pump);
void Pump_Reset(Pump* pump);

#ifdef __cplusplus
}
#endif

#endif /* SRC_PERISTALTICPUMP_PUMP_H_ */
