/*
 * Meter.c
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */

#include <math.h>
#include <McuFlash.h>
#include "string.h"
#include "SystemConfig.h"
#include "MeterScheduler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LuipApi/OpticalMeterInterface.h"
#include "DncpStack/DncpStack.h"
#include "SolenoidValve/ValveManager.h"
#include "System.h"
#include "DNCP/App/DscpSysDefine.h"
#include "DNCP/Lai/LaiRS485Handler.h"
#include "SensorControl/SensorManager.h"
#include "Meter.h"


#define METER_SENSOR_DI		0
#define METER_SENSOR_AD 	1
#define METER_SENSOR_TYPE	METER_SENSOR_AD

#define METER_SENSOR_AD_THRESHOLD       3600 //默认无水时候的定量AD
#define METER_SENSOR_AD_THRESHOLD_RANGE 200  //无水阈值判断范围
#define METER_SENSOR_VALUE_WARE			100   //流动液体判断阈值
static int dynamic_meter_sensor_AD[METER_POINT_NUM] = {0,0};

#define METER_VOlUME_MAX             12					//12mL 	抽液最大体积
#define METER_VOlUME_MIN             2					//2mL	排液最大体积

#define PUMP_HIGH_SPEED              150                 //高速，单位步/秒
#define PUMP_MEDIUM_SPEED            50                  //中速，单位步/秒
#define PUMP_LOW_SPEED               20                  //低速，单位步/秒

#define PUMP_HIGH_ACC                6000                //高加速，单位步/平方秒
#define PUMP_MEDIUM_ACC              800                 //中加速，单位步/平方秒
#define PUMP_LOW_ACC                 200                 //低加速，单位步/平方秒
//加速度和最大速度的单位转换，步/平方秒和步/秒分别转化为ml/平方秒,ml/秒
#define PUMP_ACC_SPEED(X)            (PumpManager_GetFactor(METER_PUMP_NUM) * X)

#define HARDWARE_RIGHT_MIN_AD        1000                // 判断硬件好坏的下限值
#define HARDWARE_RIGHT_MAX_AD        4095                // 判断硬件好坏的上限值

#define READ_AIR_AD_N                32                  // 读空气AD值个数
#define READ_AIR_AD_FILTER_LOW       10                  // 最小空气AD值数据过滤个数
#define READ_AIR_AD_FILTER_HIGH      10                  // 最大空气AD值数据过滤个数
#define READ_CUR_AD_N                1                   // 读当前AD值个数
#define LED_ON_DELAY_MS              2000                // LED打开后读AD值得延时
#define METER_MAX_OVER_VALVE         150
#define METER_DEF_OVER_VALVE         100

//积分定量参数定义
#define INTERGRAL_INTERVAL           200                 // 积分区间长度
#define DIFFERENTIAL_INTERVAL        5                   // 一次微分包含的AD值个数
#define DIFFERENTIAL_NUM             40                  // (INTERGRAL_INTERVAL/DIFFERENTIAL_INTERVAL)   规定的微分执行次数
/**
 * @brief 定量操作结果。
 */
typedef enum
{
    METER_RESULT_FINISHED = 0,                           //定量正常完成。
	METER_RESULT_FAILED = 1,                           	 //传感器失效。
	METER_RESULT_STOPPED = 2,                            //定量被停止。
    METER_RESULT_OVERFLOW = 3,                           //定量溢出。
    METER_RESULT_UNFINISHED = 4,                         //定量目标超时未达成。
    METER_RESULT_AIRBUBBLE = 5,                          //定量遇到气泡
} MeterResult;

//光学检测模块状态
typedef enum
{
    OPTICAL_STATUS_AD_AIR_STABLE,                        // 空气的AD值稳定，用于抽取
    OPTICAL_STATUS_AD_LIQUID_STABLE,                     // 液体的AD值稳定，用于排空
    OPTICAL_STATUS_AD_INCREASE,                          // AD值递增，用于排空
    OPTICAL_STATUS_AD_DECREASE,                          // AD值递减，用于抽取
    OPTICAL_STATUS_AD_COMPLETE,                          // 完成
    OPTICAL_STATUS_AD_WRONG,                             // 出错，出现气泡
    MAX_OPTICAL_STATUS
} enum_OpticalState;

/**
 * @brief 定量检测状态指示。
 */
typedef enum
{
	Meter_Measure_Idle = 0,
	Meter_Measure_WaitS1 = 1,
	Meter_Measure_WaitS2 = 2,
	Meter_Measure_WaitTagS = 3,
	Meter_Measure_WaitStop = 4,
	Meter_Measure_CheckS1 = 5,
	Meter_Measure_CheckS2 = 6,
	Meter_Measure_SuspendS1 = 7,
	Meter_Measure_BackstepS1 = 8,
	Meter_Measure_WaitForwardS1 = 9,
	Meter_Measure_ForwardS1 = 10,
	Meter_Measure_WaitStopS1 = 11,
}Meter_Measure_Status;

//static enum_OpticalState s_opticalState;                 //检查定量点状态机的状态

static MeterStatus s_status;                             //定量状态
static Bool s_isStatusSwitchStart;                       //定量切换标志量
//static MeterMode s_mode;                                 //定量模式
static MeterDir s_dir;                                   //定量方向，向上：从定量点下方定量到定量点或者上方。
static Bool s_isWaitPumpStop;                            //等待电机停止标志量
static Bool s_isSendEvent;                               //是否发送定量事件，只有在接收到DNCP的启动定量的命令才为true，定量结束后和其他时候为false
static MeterResult s_result;                             //定量结果

//static float s_smartPumpVolume;                          //智能定量需要抽取体积
//系统定量点体积，保存的有效的设置定量点体积是必须从小到大的，设置定量点体积为0表示无效，
//例如 定量点0 设置体积 1 真实体积 0.8 ； 定量点1 设置体积 0 真实体积 0； 定量点2 设置体积 2 真实体积 2.5
static MeterPoint s_meterPoint;
static float s_minMeterPoint = 0.08;                            //系统最小定量点
static float s_maxMeterPoint = 0.50;                            //系统最大定量点
static float s_PumpFactorCalc = 0;

//static Uint8 s_setMeterPoint;                            //设置定量点号
//static Bool s_excessJudgeOpen;                           //过量判断打开标志
//static Uint8 s_excessMeterPoint;                         //过量定量点号

//static float s_lastStatusVolume;                         //总共抽取的液体体积
static float s_limitVolume;                              //限制体积
//static float s_drainLimitVolume   =  2.0;            //下压限制体积
//static Bool s_limitJudgeOpen;                            //限制体积是否打开

//static Uint16 s_setMeterAirAD;                           //定量点空气AD值
//static Uint16 s_excessMeterAirAD;                        //过量定量点空气AD值
//static Uint16 s_MeterAirAD[METER_POINT_NUM] =            //各个定量点空气的AD值，用于向下定量
//{ 4095, 4095};

//static Bool s_isStartCheackMeter;                        //开启检查是否到达定量点
//static float s_airBubbleRatio;                           //空气AD值与积分平均值 有气泡的比率
static Bool s_isAutoCloseValve = TRUE;                   //是否每次定量结束自动关阀

static float s_setPumpSpeed;                             //设定泵的定量速度，此速度只能用于第一次抽取
static float s_pumpExtractSpeed;                         //定量时泵第一次抽取和智能抽取的速度，单位步/秒 //初次定量速度
static float s_pumpDrainSpeed;                           //定量时泵排空的速度，单位步/秒  //过量反压速度
//各个状态机启动的最大速度和加速度
#define FIRST_PUMPEXTRACT_ACC        PUMP_HIGH_ACC       //第一抽取加速度
#define FIRST_PUMPEXTRACT_MAX_SPEED  s_pumpExtractSpeed  //第一抽取最大速度，此参数由Meter_SetPumpParam设置

#define TWO_PUMPEXTRACT_ACC          PUMP_MEDIUM_ACC     //第二抽取加速度
#define TWO_PUMPEXTRACT_MAX_SPEED    PUMP_LOW_SPEED      //第二抽取最大速度

#define DRAIN_ACC                    PUMP_MEDIUM_ACC     //往下排空的加速度
#define DRAIN_MAX_SPEED              s_pumpDrainSpeed    //往下排空的最大速度，此参数由Meter_SetPumpParam设置

#define SMARTEXTRACT_ACC             PUMP_MEDIUM_ACC     //智能抽取的加速度
#define SMARTEXTRACT_MAX_SPEED       s_pumpExtractSpeed  //智能抽取的最大速度，此参数由Meter_SetPumpParam设置

//static Uint32 s_setStandardCnt;                          //用于计数，是否到达标准值，
//static Bool s_isDrianOverSwong;                          //向下排空是否过冲，过冲则采用微分方式,保证液体在定量点之下，与检查气泡s_isDrianCheckAirBubble互斥
//static Bool s_isExtractOverSwong;                        //向上抽取是否过冲，过冲则采用微分方式,保证液体在定量点之上  向上定量为TRUE
//static Bool s_isDrianCheckAirBubble;                     //向下排空是否检气泡，检则采用积分方式。与过冲s_isDrianOverSwong互斥   向上定量为TRUE

static Uint32 s_meterFinishValveMap = 0;
static Uint16 s_ropinessOverValue = 400;        //粘稠模式过冲参数
static Uint16 s_accurateOverValue = 100;        //精确模式过冲参数

//static float s_stableDiff = 100.0;

static Uint16 s_meterThreshold = 5;                 //精确定量结果判定门限

static Uint16 s_meterInterval = 10;                     //精确定量AD抽值间隔数目

#define METER_POINT_VOLUME_1        0.08       			//定量点1体积
#define METER_POINT_VOLUME_2        0.50       			//定量点2体积
static Uint8 s_curMeterPoint = 0;	//当前定量点
static float s_remainVolume = 0;
//static Bool  s_sensorChange = FALSE;
static float s_PumpFactor = 0.001; //泵速系数
static int s_BubbleTick = 10;
static void Meter_Handler(void);
//static void Meter_CheackMeterHandle(void);
static void Meter_MeterADPeriodHandle(void);
static void Meter_FindMeterVolume(MeterPoint *points);
static void Meter_SetPumpParam(float extraceSpeed, float drainSpeed);
static void Meter_Measure_Handle(void);
Bool Meter_GetSensor(Uint8 index);
static void Meter_calibration_Handle(void);
void MeterAD_Threshold_Value_Init(void);

void Meter_Init(void)
{
    Uint8 buffer[METER_FACTORY_SIGN_FLASH_LEN] =
    { 0 };
//    Uint8 meterBuffer[METER_OVER_VALVE_FLASH_LEN] =
//    { 0 };

    Uint32 flashFactorySign = 0;
#if METER_SENSOR_TYPE == METER_SENSOR_DI
    SensorManager_Init();
#else
    //MeterLED_Init();
    MerterADC_Init();
    MeterADC_Start();
#endif

    s_status = METER_STATUS_IDLE;
    s_isStatusSwitchStart = FALSE;

    McuFlash_Read(METER_FACTORY_SIGN_FLASH_BASE_ADDR,
    METER_FACTORY_SIGN_FLASH_LEN, buffer);             //读取出厂标志位
    memcpy(&flashFactorySign, buffer, METER_FACTORY_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign)             //表示已经过出厂设置
    {
        s_meterPoint = Meter_GetMeterPoints();
        if (s_meterPoint.num > 0 && s_meterPoint.num <= METER_POINT_NUM)
        {
            Meter_FindMeterVolume(&s_meterPoint);
        }
        else
        {
            memset(&s_meterPoint, 0, sizeof(MeterPoint));
        }
    }
    else             //未设置,使用默认值，并写入出厂标志
    {
        flashFactorySign = FLASH_FACTORY_SIGN;

        memset(&s_meterPoint, 0, sizeof(MeterPoint));
        s_meterPoint.num = 2;
        s_meterPoint.volume[0][SETVIDX] = METER_POINT_VOLUME_1;
        s_meterPoint.volume[0][REALIDX] = METER_POINT_VOLUME_1;
        s_meterPoint.volume[1][SETVIDX] = METER_POINT_VOLUME_2;
        s_meterPoint.volume[1][REALIDX] = METER_POINT_VOLUME_2;
        Meter_SetMeterPoints(&s_meterPoint);

        memcpy(buffer, &flashFactorySign, METER_FACTORY_SIGN_FLASH_LEN);

        McuFlash_Write(
        METER_FACTORY_SIGN_FLASH_BASE_ADDR,
        METER_FACTORY_SIGN_FLASH_LEN, buffer);
    }

    s_isSendEvent = FALSE;
    //s_isDrianOverSwong = FALSE;
    //s_isExtractOverSwong = FALSE;
    //s_isDrianCheckAirBubble = FALSE;
    MeterAD_Threshold_Value_Init();

    MeterScheduler_RegisterTask(Meter_Handler); // 定量过程管控、定量整体状态处理回调函数注册
    MeterScheduler_RegisterTimer(Meter_Measure_Handle, Meter_MeterADPeriodHandle); //定量点识别、AD值上报定时器回调函数注册

    //MeterScheduler_RegisterMeterCheck(Meter_calibration_Handle);
    MeterScheduler_Init();
    s_setPumpSpeed = 150; //刚启动时使用默认的设置速度
    Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
    //PumpManager_SetTempFactor(METER_PUMP_NUM, 0.00076);
}

void MeterAD_Threshold_Value_Set(int index, int meterAD)
{
	Uint16 tempAD[METER_POINT_NUM] = {0};
	Uint8 buffer[METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN] ={ 0 };
	dynamic_meter_sensor_AD[index] = meterAD;
	tempAD[0] = dynamic_meter_sensor_AD[0];
	tempAD[1] = dynamic_meter_sensor_AD[1];
	memcpy(buffer, tempAD,  METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN);
	McuFlash_Write(METERAD_THRESHOLD_VALUE_PARAM_FLASH_BASS_ADDR, METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN, buffer);
}

void MeterAD_Threshold_Value_Read()
{
	Uint8 bufferAD[METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN] ={ 0 };
	Uint16 tempAD[METER_POINT_NUM] = {0};
	McuFlash_Read(METERAD_THRESHOLD_VALUE_PARAM_FLASH_BASS_ADDR, METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN, bufferAD);
	memcpy(tempAD, bufferAD, METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN);
	dynamic_meter_sensor_AD[0] = tempAD[0];
	dynamic_meter_sensor_AD[1] = tempAD[1];
}

void MeterAD_Threshold_Value_Init(void)
{
	Uint32 flashFactorySign = 0;
	Uint8 buffer[METERAD_THRESHOLD_VALUE_FLASH_LEN] ={ 0 };
	McuFlash_Read(METERAD_THRESHOLD_VALUE_FLASH_BASS_ADDR, METERAD_THRESHOLD_VALUE_FLASH_LEN, buffer);             //读取出厂标志位
	memcpy(&flashFactorySign, buffer, METERAD_THRESHOLD_VALUE_FLASH_LEN);
	if (FLASH_FACTORY_SIGN == flashFactorySign)             //表示已经过出厂设置
	{
		MeterAD_Threshold_Value_Read();
	}
	else
	{
		flashFactorySign = FLASH_FACTORY_SIGN;
		dynamic_meter_sensor_AD[0] = METER_SENSOR_AD_THRESHOLD;
		dynamic_meter_sensor_AD[1] = METER_SENSOR_AD_THRESHOLD;
		MeterAD_Threshold_Value_Set(0, dynamic_meter_sensor_AD[0]);
        memcpy(buffer, &flashFactorySign, METER_FACTORY_SIGN_FLASH_LEN);
        McuFlash_Write(METERAD_THRESHOLD_VALUE_FLASH_BASS_ADDR, METERAD_THRESHOLD_VALUE_FLASH_LEN, buffer);
	}
}

/**
 * @brief 定量状态恢复初始化
 */
void Meter_Restore(void)
{
    s_isSendEvent = FALSE;//DNCP可能调用此函数，此时不能给上位机上报定量事件
    Meter_RequestStop();
}

/**
 * @brief 由DNCP或者控制台设置定量速度，单位ml/s
 * @param speed定量速度，单位ml/s
 */
void Meter_SetMeterSpeed(float speed)
{
    s_PumpFactor = PumpManager_GetTempFactor(METER_PUMP_NUM);
    s_setPumpSpeed = speed / PumpManager_GetFactor(METER_PUMP_NUM);
    TRACE_INFO("Meter set speed: ");
    System_PrintfFloat(TRACE_LEVEL_INFO, speed, 6);
    TRACE_INFO(" ml/s, ");
    System_PrintfFloat(TRACE_LEVEL_INFO, s_setPumpSpeed, 3);
    TRACE_INFO(" step/s");
    if (s_setPumpSpeed < PUMP_LOW_SPEED)
    {
        s_setPumpSpeed = PUMP_LOW_SPEED;
    }
    //定量过程中不能改变速度，定量结束会自动重新设置定量使用的运动参数组
    if (METER_STATUS_IDLE == Meter_GetMeterStatus())
    {
        Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
    }
}

/**
 * @brief 由DNCP或者控制台设置定量速度，单位ml/s
 * @return 定量速度，单位ml/s
 */
float Meter_GetMeterSpeed(void)
{
    return (s_setPumpSpeed * PumpManager_GetFactor(METER_PUMP_NUM));
}
/**
 * 设置定量过程使用的运动参数组
 * @param extraceSpeed 抽取的速度，改变FIRST_PUMPEXTRACT_MAX_SPEED(第一次抽取)， SMARTEXTRACT_MAX_SPEED(智能抽取)的速度
 * @param drainSpeed 往下排的速度，改变DRAIN_MAX_SPEED(往下排)的速度
 * @return 改变的值s_pumpExtractSpeed  s_pumpDrainSpeed
 */
static void Meter_SetPumpParam(float extraceSpeed, float drainSpeed)
{
    s_pumpExtractSpeed = extraceSpeed;
    s_pumpDrainSpeed = drainSpeed;
    if (s_pumpExtractSpeed < PUMP_LOW_SPEED)
    {
        s_pumpExtractSpeed = PUMP_LOW_SPEED;
    }
    if (s_pumpDrainSpeed < PUMP_LOW_SPEED)
    {
        s_pumpDrainSpeed = PUMP_LOW_SPEED;
    }
}

/**
 * @brief 获取定量状态
 * @return
 */
MeterStatus Meter_GetMeterStatus(void)
{
    return (s_status);
}

/**
 * @brief 查找最小定量点和最大定量点。
 * @note points里保存的有效定量点体积是必须从小到大的，定量点体积为0表示无效
 * @param points 本系统的定量体系
 * @return 改变的值 s_minMeterPoint s_maxMeterPoint
 */
static void Meter_FindMeterVolume(MeterPoint *points)
{
    for (Uint8 i = 0; i < points->num; i++)
    {
        if (points->volume[i][SETVIDX] > 0)
        {
            s_minMeterPoint = points->volume[i][REALIDX];
            break;
        }
    }
    for (int i = points->num - 1; i >= 0; i--)
    {
        if (points->volume[i][SETVIDX] > 0)
        {
            s_maxMeterPoint = points->volume[i][REALIDX];
            break;
        }
    }
    TRACE_INFO("\n minMeterPoint %f maxMeterPoint %f", s_minMeterPoint, s_maxMeterPoint);
}

/**
 * @brief 设置系统定量点,无效定量点为0
 * @param dot
 * @return 是否设置成功 改变的值 s_meterPoint s_minMeterPoint s_maxMeterPoint
 */
Bool Meter_SetMeterPoints(MeterPoint *dot)
{
    if (METER_STATUS_IDLE == s_status)
    {
        Uint8 writeData[METER_POINTS_FLASH_LEN] =
        { 0 };
        s_meterPoint = *dot;
        Meter_FindMeterVolume(&s_meterPoint);
        memcpy(writeData, &s_meterPoint, sizeof(MeterPoint));
        McuFlash_Write(
        METER_POINTS_FLASH_BASE_ADDR,
        METER_POINTS_FLASH_LEN, writeData);
        TRACE_MARK("\n num :%d ", s_meterPoint.num);
        for (Uint8 i = 0; i < s_meterPoint.num; i++)
        {
            TRACE_MARK("\n %d point ", i + 1);
            TRACE_MARK("SetV:");
            System_PrintfFloat(TRACE_LEVEL_MARK,
                    s_meterPoint.volume[i][SETVIDX], 2);
            TRACE_MARK(" ml RealV:");
            System_PrintfFloat(TRACE_LEVEL_MARK,
                    s_meterPoint.volume[i][REALIDX], 2);
            TRACE_MARK(" ml");
        }
        return TRUE;
    }
    else
    {
        TRACE_ERROR(
                "\n Set the volume of the meter point to fail because the meter is running");
    }
    return FALSE;
}

/**
 * @brief 获取系统定量点体积设置
 * @return
 */
MeterPoint Meter_GetMeterPoints(void)
{
    Uint8 readData[METER_POINTS_FLASH_LEN] =
    { 0 };
    MeterPoint dot;
    McuFlash_Read(METER_POINTS_FLASH_BASE_ADDR, METER_POINTS_FLASH_LEN,
            readData);
    memcpy(&dot, readData, sizeof(MeterPoint));
    return (dot);
}

float Meter_GetMeterPointVolum(int index)
{
    return s_meterPoint.volume[index][REALIDX];
}

/**
 * @brief 切换定量状态
 * @param status
 */
static void Meter_StatusSwitch(MeterStatus status)
{
    // 状态切换
    s_status = status;
    s_isStatusSwitchStart = TRUE;
}


/**
 * @brief 开始定量。
 * @param dir 定量方向，0为正向转动向上定量，1为向下定量。
 * @param mode 定量模式
 * @param volume 定量体积，单位为 ml。
 * @param limitVolume 限量体积，在定量过程中总的泵取体积不能超过该值，0表示不进行限量判定。单位为 ml。
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败(参数错误)
 */
Uint16 Meter_Start(MeterDir dir, MeterMode mode, float volume,
        float limitVolume)
{
	if(mode == METER_MODE_CALI)
	{
	    MeterScheduler_RegisterMeterCheck(Meter_calibration_Handle); //校准泵系数
	    TRACE_INFO("\n Meter_calibration_Handle");
	}
	else
	{
		MeterScheduler_RegisterMeterCheck(Meter_Measure_Handle); //普通定量，不校准泵系数
		TRACE_INFO("\n Meter_Measure_Handle");
	}
    if (METER_STATUS_IDLE == s_status && volume >= s_minMeterPoint)
    {
    	if (volume >= s_minMeterPoint && volume < s_maxMeterPoint)
    	{
    		s_curMeterPoint = 0;
    	}
    	else if (volume >= s_maxMeterPoint)
    	{
    		s_curMeterPoint = 1;
    	}

//    	dynamic_meter_sensor_AD[0] = MeterADC_GetCurAD(0);
//    	dynamic_meter_sensor_AD[1] = MeterADC_GetCurAD(1);
    	if(mode != METER_MODE_CALI)
    	{
			if (MeterADC_GetCurAD(s_curMeterPoint)< METER_SENSOR_AD_THRESHOLD - METER_SENSOR_AD_THRESHOLD_RANGE
					||MeterADC_GetCurAD(s_curMeterPoint)> METER_SENSOR_AD_THRESHOLD + METER_SENSOR_AD_THRESHOLD_RANGE)
			//if (Meter_GetSensor(s_curMeterPoint))
			{
				TRACE_ERROR("\n Current Sensor is covered ");
				return DSCP_ERROR;
			}
    	}


		DncpStack_ClearBufferedEvent();

		s_limitVolume = limitVolume;

		if (s_curMeterPoint == 0)
		{
			s_remainVolume = volume - s_minMeterPoint;
		}
		else
		{
			s_remainVolume = volume - s_maxMeterPoint;
		}
        
        //2*s_setPumpSpeed*s_PumpFactor => uL/tick tic=2us
        s_BubbleTick = (int)(5/(s_setPumpSpeed*s_PumpFactor)); //10uL  过冲长度，用于检测气泡
        TRACE_INFO("\n s_BubbleTick:%d", s_BubbleTick);

        s_PumpFactorCalc = (s_maxMeterPoint - s_minMeterPoint)*PumpManager_GetSubdivision(METER_PUMP_NUM);
        
		s_dir = dir;
		TRACE_INFO("\n Meter_Start");
		TRACE_INFO("\n dir:%d", s_dir);
		TRACE_INFO(" volume:");
		System_PrintfFloat(TRACE_LEVEL_INFO, volume, 2);
		TRACE_INFO("ml limitVolume:");
		System_PrintfFloat(TRACE_LEVEL_INFO, s_limitVolume, 2);
		TRACE_INFO("ml");
		TRACE_INFO("\n To START");
		Meter_StatusSwitch(METER_STATUS_START);
		MeterScheduler_MeterResume();
		return DSCP_OK;
    }
    else
    {
        TRACE_ERROR("\n Meter startup failed because the meter is runing. ");
        return DSCP_ERROR;
    }
}



/**
 * @brief 设置定量泵运动参数和启动定量泵函数
 * @param acceleration
 * @param maxSpeed
 * @param dir
 * @param volume
 * @return
 */
static Uint16 Meter_PumpStart(float acceleration, float maxSpeed, Direction dir,
        float volume)
{
    PumpManager_SetMotionParam(METER_PUMP_NUM, acceleration, maxSpeed,
            NoWriteFLASH);
    return (PumpManager_Start(METER_PUMP_NUM, dir, volume, NoReadFLASH));
}

/**
 * @brief 请求停止定量泵
 * @return
 */
static void Meter_RequestPumpStop(void)
{
    s_isWaitPumpStop = TRUE;
    PumpManager_Stop(METER_PUMP_NUM);
}

/**
 * @brief 请求停止定量
 * @return
 */
Bool Meter_RequestStop(void)
{
    if (METER_STATUS_IDLE != s_status)
    {
        TRACE_INFO("\n Meter request stop");
        s_result = METER_RESULT_STOPPED;
        Meter_RequestPumpStop();
        return TRUE;
    }
    else
    {
        TRACE_ERROR(
                "\n Because the meter is not running, the request to stop failure. ");
    }
    return FALSE;
}
/**
 * @brief 停止定量
 * @note
 */
static void Meter_Stop(MeterResult result)
{
    if (TRUE == s_isAutoCloseValve)
    {
        ValveManager_SetValvesMap(s_meterFinishValveMap);
    }

    s_status = METER_STATUS_IDLE;
    s_isStatusSwitchStart = FALSE;

    TRACE_INFO("\n Meter stop");
    if (TRUE == s_isSendEvent)
    {
        DncpStack_SendEvent(DSCP_EVENT_OMI_METER_RESULT, &result,
                sizeof(MeterResult));
        DncpStack_BufferEvent(DSCP_EVENT_OMI_METER_RESULT, &result,
                sizeof(MeterResult));
    }
    s_isSendEvent = FALSE;
    Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
}
/**
 * @brief 打开定量停止发送事件功能
 */
void Meter_SendMeterEventOpen(void)
{
    s_isSendEvent = TRUE;
}

Bool Meter_GetSensorStatus(Uint8 index)
{
#if METER_SENSOR_TYPE == METER_SENSOR_DI
	return SensorManager_ReadSensorStatus(index);
#else
	int change = dynamic_meter_sensor_AD[index] - MeterADC_GetCurAD(index);
	if(METER_SENSOR_AD_THRESHOLD > MeterADC_GetCurAD(index))
	{

		if(100 < change || -100 > change)
			{return TRUE;}
		else
			{return FALSE;}
	}
	else
	{
		return FALSE;
	}
#endif
}

Bool Meter_GetSensor(Uint8 index)
{
	static uint16_t MeterAD = 0;
#if METER_SENSOR_TYPE == METER_SENSOR_DI
	return SensorManager_ReadSensorStatus(index);
#else
	MeterAD = MeterADC_GetCurAD(index);
	if(dynamic_meter_sensor_AD[index] - METER_SENSOR_AD_THRESHOLD_RANGE <  MeterAD &&
			MeterAD < dynamic_meter_sensor_AD[index] + METER_SENSOR_AD_THRESHOLD_RANGE)
	{
		if(METER_SENSOR_VALUE_WARE < MeterADC_GetCurWare(index))
		{
			return TRUE;
		}
		return FALSE;
	}
	else
	{
		return TRUE;
	}
#endif
}

#define MAX_TIME_OUT_COUNT 55000 //110秒
#define BUBBLE_IGNORE 60
//软件定时器2ms调用周期
static void Meter_calibration_Handle(void)
{
    static Uint16 status = Meter_Measure_Idle;
    static Uint32 S1Steps = 0;
    static Uint32 S2Steps = 0;
    static int TimeOut = 0;
    static float PumpFactor = 0;
    static int BubbleIgnore = BUBBLE_IGNORE;

    if (TRUE == s_isWaitPumpStop)
    {
    	s_isWaitPumpStop = FALSE;
        PumpManager_ImmediatelyStop(METER_PUMP_NUM);
    	TimeOut = 0;
    	status = Meter_Measure_WaitStop;
        s_result = METER_RESULT_STOPPED;
    }

    if(s_dir == METER_DIR_EXTRACT)
    {
        switch (status)
        {
        case Meter_Measure_Idle:
            {
                status = Meter_Measure_WaitS1;
                BubbleIgnore = s_BubbleTick;
            }
            break;
        case Meter_Measure_WaitS1:
            {
                if(Meter_GetSensor(0))
                {
                    if(s_curMeterPoint == 1)
                    {
                        S1Steps = PumpManager_GetAlreadyStep(METER_PUMP_NUM);
                        status = Meter_Measure_CheckS2;
                        BubbleIgnore = s_BubbleTick;
                    }
                    else
                    {
                    	PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                    	s_result = METER_RESULT_FAILED;
                    	status = Meter_Measure_WaitStop;
                    }
                    TimeOut = 0;
                }
                else
                {
                	TimeOut++;
					if((TimeOut > MAX_TIME_OUT_COUNT) || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
					{
						TimeOut = 0;
						//未抽到水超时,
						if (PUMP_IDLE != PumpManager_GetStatus(METER_PUMP_NUM))
						{
							TRACE_INFO("\n Sensor 1 is OutTime\n");
							PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						}
						else
						{
							TRACE_INFO("\n meter PUMP_IDLE \n");
						}
						s_result = METER_RESULT_UNFINISHED;
						status = Meter_Measure_WaitStop;
					}
                }
            }
            break;
        case Meter_Measure_CheckS2:
			{
				if(FALSE == Meter_GetSensor(1))
				{
					if (1 > BubbleIgnore)
					{
						status = Meter_Measure_WaitS2;
					}
					BubbleIgnore--;
				}
				else
				{
					BubbleIgnore = s_BubbleTick;
				}
			}
			break;
        case Meter_Measure_WaitS2:
            {
                if(Meter_GetSensor(1))
                {
                    //BubbleValue = s_BubbleTick;
                    S2Steps = PumpManager_GetAlreadyStep(METER_PUMP_NUM);
                    PumpFactor = s_PumpFactorCalc/(S2Steps - S1Steps);
                     //更新临时的泵校准系数
                    if(PumpFactor>0.00045&&PumpFactor<0.0009)
                    {
                    	PumpManager_SetTempFactor(METER_PUMP_NUM, PumpFactor);
                    }
                    status = Meter_Measure_WaitTagS;
                    if((S2Steps - S1Steps)<100)
					{
                    	PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                    	TRACE_ERROR("\n Volume mismatch \n");
                    	s_result = METER_RESULT_FAILED;
                    	status = Meter_Measure_WaitStop;
					}
                    TRACE_INFO("\n S1:%d S2:%d", S1Steps,S2Steps);
                    TRACE_INFO("\n Factor:%f", PumpFactor);
                    TimeOut = 0;
                }
                else
                {
					TimeOut++;
					if(TimeOut > MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))) //15s
					{
						TimeOut = 0;
						//传感器2失效
						PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						TRACE_INFO("\n Sensor 2 is OutTime\n");
						s_result = METER_RESULT_FAILED;
						status = Meter_Measure_WaitStop;
					}
                }
            }
            break;
        case Meter_Measure_WaitTagS:
            {
                if(TRUE == Meter_GetSensor(s_curMeterPoint)) //遮挡
                {

					if(Meter_GetSensor(0)) //再次检查S1
					{
						if (s_remainVolume > 0)
						{
							float volume = PumpManager_GetVolume(METER_PUMP_NUM);
							PumpManager_ChangeVolume(METER_PUMP_NUM,(volume + s_remainVolume));
						}
						else
						{
							PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						}
						s_result = METER_RESULT_FINISHED;
						status = Meter_Measure_WaitStop;
					}
					else
					{
						//缺液
						PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						Printf("\n Lack of fluid \n");
						s_result = METER_RESULT_AIRBUBBLE;
						status = Meter_Measure_WaitStop;
					}
                    TimeOut = 0;
                }
                else //未遮挡
                {
                    //判断为气泡
                    TimeOut++;
                    if(TimeOut > MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
                    {
                    	TimeOut = 0;
                        //未抽到水超时,抽一半缺液
                        PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                        s_result = METER_RESULT_AIRBUBBLE;
                        status = Meter_Measure_WaitStop;
                    }
                }
            }
            break;
        case Meter_Measure_WaitStop:
            {
                if (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))
        		{
                	TimeOut = 0;
        			S1Steps = 0;
        			S2Steps = 0;
        			Meter_Stop(s_result);
        			status = Meter_Measure_Idle;
        			MeterScheduler_StopMeterCheckTimer();
        		}
            }
            break;
        }
    }
}
    
//软件定时器2ms调用周期
static void Meter_Measure_Handle(void)
{
//#define MAX_TIME_OUT_COUNT 40000 //80秒
//#define BUBBLE_IGNORE 60
    static Uint16 status = Meter_Measure_Idle;
    //static Uint32 S1Steps = 0;
    //static Uint32 S2Steps = 0;
    static int BubbleValue = 0;
    static int TimeOut = 0;
    //static float PumpFactor = 0;
    static int BubbleIgnore = BUBBLE_IGNORE;
    static int AirVolume = 0;
    static int TotalVolume = 0;
    static int Tik =0, TikTemp = 0;
    
    if (TRUE == s_isWaitPumpStop)
    {
    	s_isWaitPumpStop = FALSE;
        PumpManager_ImmediatelyStop(METER_PUMP_NUM);
    	//Meter_Stop(s_result);
    	TimeOut = 0;
    	status = Meter_Measure_WaitStop;
        s_result = METER_RESULT_STOPPED;
        TRACE_INFO("\n meter Request Stop\n");
    }
    
    if(s_dir == METER_DIR_EXTRACT)
    {
        switch (status)
        {
        case Meter_Measure_Idle:
            {
                status = Meter_Measure_WaitS1;
                BubbleIgnore = s_BubbleTick;
            }
            break;
        case Meter_Measure_WaitS1:
            {
                if(Meter_GetSensor(0))
                {
                    if(s_curMeterPoint == 0)
                    {
                        //BubbleValue = s_BubbleTick;
                        status = Meter_Measure_CheckS1;
                        Tik = (int)(Meter_GetMeterPointVolum(0)*500.0/Meter_GetMeterSpeed());
                        TikTemp = Tik;
                    }
                    else
                    {
                        //S1Steps = PumpManager_GetAlreadyStep(METER_PUMP_NUM);
                        status = Meter_Measure_WaitS2;
                        Tik = (int)((Meter_GetMeterPointVolum(1)-Meter_GetMeterPointVolum(0))*500.0/Meter_GetMeterSpeed());
                        TikTemp = Tik;
                        AirVolume = 0;
                    }
                    TimeOut = 0;
                }
                else if(Meter_GetSensor(1))
                {
                	BubbleIgnore--;
                	//AirVolume--;
                	if(BubbleIgnore <= 0)
                	{
						//传感器1失效
						PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						TRACE_INFO("\n Sensor 1 is Error\n");
						s_result = METER_RESULT_FAILED;
						status = Meter_Measure_WaitStop;
						TimeOut = 0;
						BubbleIgnore = BUBBLE_IGNORE;
                	}
                }
                else
                {
                	TimeOut++;
					if((TimeOut > MAX_TIME_OUT_COUNT) || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
					{
						TimeOut = 0;
						//未抽到水超时,
						if (PUMP_IDLE != PumpManager_GetStatus(METER_PUMP_NUM))
						{
							TRACE_INFO("\n Sensor 1 is OutTime\n");
							PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						}
						else
						{
							TRACE_INFO("\n meter PUMP_IDLE \n");
						}
						s_result = METER_RESULT_UNFINISHED;
						status = Meter_Measure_WaitStop;
					}
                }
            }
            break;
        case Meter_Measure_CheckS1:
            {
            	TotalVolume++;
                if(Meter_GetSensor(0))
                {
                    TikTemp--;
                    if(TikTemp<=0)
                    {
                        PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                        status = Meter_Measure_SuspendS1;
                    }
                }
                else
                {
                	AirVolume++;
                }

            }
            break;
        case Meter_Measure_SuspendS1:
            {
                TimeOut++;
                if(TimeOut > 1500)
                {
                    status = Meter_Measure_BackstepS1;
                    //反向
                    Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
									PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED),
									DRAIN, METER_VOlUME_MAX);
                    TikTemp = Tik;
                    TotalVolume += (Tik>>1);
                    if(TotalVolume>(7*Tik))
                    {
                    	TotalVolume = 7*Tik;
                    	AirVolume = 0;
                    }
                    TimeOut = 0;
                }
            }
            break;
        case Meter_Measure_BackstepS1:
            {
            	TotalVolume--;
                if(TotalVolume<=0)
                {
                    PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                    status = Meter_Measure_WaitForwardS1;
                }
            }
            break;
        case Meter_Measure_WaitForwardS1:
			{
				TimeOut++;
				if(TimeOut > 1500)
				{
					status = Meter_Measure_ForwardS1;
					//反向
					Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
									PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED),
									SUCK, METER_VOlUME_MAX);
					TimeOut = 0;
				}

			}
            break;
        case Meter_Measure_ForwardS1:
			{
				if(Meter_GetSensor(0))
				{
					status = Meter_Measure_WaitStopS1;
				}
			}
			break;
        case Meter_Measure_WaitStopS1:
			{
				AirVolume--;
				if(AirVolume <= 0)
				{
					PumpManager_ImmediatelyStop(METER_PUMP_NUM);
					status = Meter_Measure_WaitStop;
					AirVolume = 0;
				}
			}
        	break;
        case Meter_Measure_WaitS2:
            {
                if(FALSE == Meter_GetSensor(0))
                {
                	AirVolume++;
                }
                TikTemp--;
                if(Meter_GetSensor(1)&&TikTemp <= Tik/3)
                {

                    //BubbleValue = s_BubbleTick;
//                    S2Steps = PumpManager_GetAlreadyStep(METER_PUMP_NUM);
//                    PumpFactor = s_PumpFactorCalc/(S2Steps - S1Steps);
//                     //更新临时的泵校准系数
//                    if(PumpFactor>0.00035&&PumpFactor<0.001)
//                    {
//                    	PumpManager_SetTempFactor(METER_PUMP_NUM, PumpFactor);
//                    }
//                    status = Meter_Measure_CheckS2;
//                    if((S2Steps - S1Steps)<100)
//					{
//                    	PumpManager_ImmediatelyStop(METER_PUMP_NUM);
//                    	TRACE_ERROR("\n Volume mismatch \n");
//                    	s_result = METER_RESULT_FAILED;
//                    	status = Meter_Measure_WaitStop;
//					}
                	status = Meter_Measure_CheckS2;
                    TimeOut = 0;
                    TikTemp = 0;
                    TRACE_INFO("\n WaitS2:%d  \n", AirVolume);
                }
                else
                {
					TimeOut++;
					if(TimeOut > MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))) //15s
					{
						TimeOut = 0;
						//传感器2失效
						PumpManager_ImmediatelyStop(METER_PUMP_NUM);
						TRACE_INFO("\n Sensor 2 is OutTime\n");
						s_result = METER_RESULT_FAILED;
						status = Meter_Measure_WaitStop;
					}
                }
            }
            break; 
        case Meter_Measure_CheckS2:
			{
				if(FALSE == Meter_GetSensor(0))
				{
					AirVolume++;
					//TRACE_INFO("\n CheckS2:%d  \n", AirVolume);
				}
				AirVolume--;
				if(AirVolume <= 0)
				{
					if (s_remainVolume > 0)
					{
						float volume = PumpManager_GetVolume(METER_PUMP_NUM);
						PumpManager_ChangeVolume(METER_PUMP_NUM,(volume + s_remainVolume));
					}
					else
					{
						PumpManager_ImmediatelyStop(METER_PUMP_NUM);
					}
					s_result = METER_RESULT_FINISHED;
					status = Meter_Measure_WaitStop;
				}
			}
			break;
        case Meter_Measure_WaitTagS:
            {
                if(TRUE == Meter_GetSensor(s_curMeterPoint)) //遮挡
                {
//                    BubbleValue--;
//                    if((BubbleValue <= 0) && (AirVolume <= 0))
//                    if(AirVolume <= 0)
//                    {
                    	if(Meter_GetSensor(0)) //再次检查S1
                    	{
							if (s_remainVolume > 0)
							{
								float volume = PumpManager_GetVolume(METER_PUMP_NUM);
								PumpManager_ChangeVolume(METER_PUMP_NUM,(volume + s_remainVolume));
							}
							else
							{
								PumpManager_ImmediatelyStop(METER_PUMP_NUM);
							}
							s_result = METER_RESULT_FINISHED;
							status = Meter_Measure_WaitStop;
                    	}
                    	else
                    	{
                    		//缺液
                    		PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                    		Printf("\n Lack of fluid \n");
                    		s_result = METER_RESULT_AIRBUBBLE;
                    		status = Meter_Measure_WaitStop;
                    	}
//                    }
                    TimeOut = 0;
                }
                else //未遮挡
                {
                    //判断为气泡
                    TimeOut++;
                    if(TimeOut > MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
                    {
                    	TimeOut = 0;
                        //未抽到水超时,抽一半缺液
                        PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                        s_result = METER_RESULT_AIRBUBBLE;
                        status = Meter_Measure_WaitStop;
                    }
                }
            }
            break;
        case Meter_Measure_WaitStop:
            {
                if (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))
        		{
                	TimeOut = 0;
        			//S1Steps = 0;
        			//S2Steps = 0;
        			AirVolume = 0;
        			TotalVolume = 0;
        			Meter_Stop(s_result);
        			status = Meter_Measure_Idle;
        			MeterScheduler_StopMeterCheckTimer();
        		}
            }
            break;
        }
    }
    else if(s_dir == METER_DIR_DRAIN)
    {
        switch (status)
        {
        case Meter_Measure_Idle:
            {
                status = Meter_Measure_WaitS1;
            }
            break;
        case Meter_Measure_WaitS1:
            {
                if(TRUE == Meter_GetSensor(s_curMeterPoint))
                {
                    BubbleValue = 500;
                    status = Meter_Measure_WaitS2;
                    TimeOut = 0;
                }
                TimeOut++;
				if(TimeOut > MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
				{
					TimeOut = 0;
					//未抽到水超时
					PumpManager_ImmediatelyStop(METER_PUMP_NUM);
					Printf("\n No water was pumped\n");
					s_result = METER_RESULT_UNFINISHED;
					s_dir = METER_DIR_EXTRACT;
					status = Meter_Measure_WaitStop;
				}
            }
            break;
        case Meter_Measure_WaitS2:
            {
            	TimeOut++;
                if((TRUE == Meter_GetSensor(1-s_curMeterPoint))&&FALSE == Meter_GetSensor(s_curMeterPoint))
                {
                    BubbleValue--;
                    if(BubbleValue <= 0)
                    {
                    	PumpManager_ImmediatelyStop(METER_PUMP_NUM);
                    	Printf("\n tagSensor is exposed \n");
                    	status = Meter_Measure_WaitTagS;
                    }
                    TimeOut = 0;
                }
                else if(TimeOut > MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
				{

                	TimeOut = 0;
					//未能移出传感器
					PumpManager_ImmediatelyStop(METER_PUMP_NUM);
					Printf("\n No water was pumped\n");
					s_result = METER_RESULT_FAILED;
					s_dir = METER_DIR_EXTRACT;
					status = Meter_Measure_WaitStop;
				}
                else
                {
                	BubbleValue = 500;
                }

            }
            break;
        case Meter_Measure_WaitTagS:
			{
				TimeOut++;
				if(TimeOut>=500)
				{
					TimeOut = 0;
					s_dir = METER_DIR_EXTRACT;
					BubbleValue = s_BubbleTick;
					Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
									PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED),
									SUCK, METER_VOlUME_MAX);
					Printf("\n pump reverse \n");
				}
			}
            break;
        default :
			{
				PumpManager_ImmediatelyStop(METER_PUMP_NUM);
				s_result = METER_RESULT_FAILED;
				s_dir = METER_DIR_EXTRACT;
                status = Meter_Measure_WaitStop;
			}
			break;
        }
    }
    else
    {
        switch (status)
        {
        case Meter_Measure_Idle:
            {
                status = Meter_Measure_WaitS1;
            }
            break;
        case Meter_Measure_WaitS1:
            {
                if(Meter_GetSensor(1))
                {
                    status = Meter_Measure_WaitS2;
                    TimeOut = 0;
                }
                TimeOut++;
				if(TimeOut>=MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
				{
				    PumpManager_ImmediatelyStop(METER_PUMP_NUM);
				    TimeOut = 0;
				    s_result = METER_RESULT_FAILED;
				    s_dir = METER_DIR_EXTRACT;
                    status = Meter_Measure_WaitStop;
                }
            }
            break;
        case Meter_Measure_WaitS2:
            {
                if(Meter_GetSensor(0))
                {
                    status = Meter_Measure_WaitTagS;
                    TimeOut = 0;
                }
                TimeOut++;
				if(TimeOut>=MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
				{
				    PumpManager_ImmediatelyStop(METER_PUMP_NUM);
				    TimeOut = 0;
				    s_result = METER_RESULT_FAILED;
				    s_dir = METER_DIR_EXTRACT;
                    status = Meter_Measure_WaitStop;
                }
            }
            break;
        case Meter_Measure_WaitTagS:
            {
                if(FALSE == Meter_GetSensor(1))
                {
                    status = Meter_Measure_WaitStop;
                    TimeOut = 0;
                }
                TimeOut++;
				if(TimeOut>=MAX_TIME_OUT_COUNT || (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM)))
				{
				    PumpManager_ImmediatelyStop(METER_PUMP_NUM);
				    s_result = METER_RESULT_FAILED;
				    s_dir = METER_DIR_EXTRACT;
				    TimeOut = 0;
                    status = Meter_Measure_WaitStop;
                }
            }
            break;
        case Meter_Measure_WaitStop:
            {
                if(FALSE == Meter_GetSensor(0))
                {
                    s_result = METER_RESULT_FINISHED;
                    TimeOut = 0;
                    s_dir = METER_DIR_EXTRACT;
                }
                TimeOut++;
				if(TimeOut>=MAX_TIME_OUT_COUNT )
				{
				    PumpManager_ImmediatelyStop(METER_PUMP_NUM);
				    s_result = METER_RESULT_FAILED;
				    s_dir = METER_DIR_EXTRACT;
				    TimeOut = 0;
                }
            }
            break;
        default :
			{
				PumpManager_ImmediatelyStop(METER_PUMP_NUM);
				TimeOut = 0;
				s_result = METER_RESULT_FAILED;
				s_dir = METER_DIR_EXTRACT;
                status = Meter_Measure_WaitStop;
			}
			break;
        }
        
    }
}


static void Meter_Handler(void)
{
    vTaskDelay(2 / portTICK_RATE_MS);

    switch (s_status)
    {
    case METER_STATUS_IDLE:
        MeterScheduler_MeterSuspend();          //切换到空闲状态才挂起本任务，那么下次恢复会在这里恢复。
        break;

    case METER_STATUS_START:
        if (TRUE == s_isStatusSwitchStart)
        {
        	s_isStatusSwitchStart = FALSE;
        	if (s_dir == METER_DIR_EXTRACT)
        	{
        		Meter_StatusSwitch(METER_STATUS_SMART);
        	}
        	else if (s_dir == METER_DIR_DRAIN)
        	{
        		Meter_StatusSwitch(METER_STATUS_DRAIN);
        	}
        	else if (s_dir == METER_EMPTY)
        	{
        		Meter_StatusSwitch(METER_STATUS_DRAIN);
        	}
        	//s_sensorChange = FALSE;
        	MeterScheduler_StartMeterCheckTimer();
        }
        break;

    case METER_STATUS_SMART:
        if (TRUE == s_isStatusSwitchStart)
        {
        	s_isStatusSwitchStart = FALSE;
            if (DSCP_OK !=
            		Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
                            PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED),
							SUCK, s_limitVolume))
            {
                TRACE_ERROR("\n Meter Pump Start error");
                s_result = METER_RESULT_FAILED;
                Meter_Stop(s_result);
                break;
            }
        }
        break;

    case METER_STATUS_DRAIN:
        if (TRUE == s_isStatusSwitchStart)
        {
            s_isStatusSwitchStart = FALSE;
            if (DSCP_OK !=
            		Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
                            PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED),
							DRAIN, s_limitVolume))
            {
                TRACE_ERROR("\n Meter Pump Start error");
                s_result = METER_RESULT_FAILED;
                Meter_Stop(s_result);
                break;
            }
        }

        break;

    default:
        break;
    }

    //Meter_ErrorJudge();

//    if (TRUE == s_isWaitPumpStop)
//    {
//    	s_isWaitPumpStop = FALSE;
//    	Meter_Stop(s_result);
//    }
}

OpticalMeterAD Meter_GetOpticalAD(void)
{
    OpticalMeterAD pointsAD;
    pointsAD.num = s_meterPoint.num;
    for (Uint8 i = 0; i < pointsAD.num; i++)
    {
#if METER_SENSOR_TYPE == METER_SENSOR_DI
    	pointsAD.adValue[i] = SensorManager_ReadSensorStatus(i);
#else
        pointsAD.adValue[i] = MeterADC_GetCurAD(i);
#endif
    }
    return (pointsAD);
}

static void Meter_MeterADPeriodHandle(void)
{
    static OpticalMeterAD s_pointsAD;
    static Uint32 s_tempAD[METER_POINT_NUM] =
    { 0 };
    static Uint8 s_data[METER_POINT_NUM * sizeof(float) + 1] =
    { 0 };
    static Uint16 s_len;

    if (TRUE == LaiRS485_GetHostStatus())
    {
        s_pointsAD = Meter_GetOpticalAD();
        for (Uint8 i = 0; i < s_pointsAD.num; i++)
        {
            if (0 != s_meterPoint.volume[i][SETVIDX]) //如果此定量点体积为0表示不存在，则此定量点的AD值赋值为0
            {
                s_tempAD[i] = (Uint32) s_pointsAD.adValue[i];
            }
            TRACE_MARK("\n %d point adValue = %d , %d", i,
                    s_pointsAD.adValue[i], s_tempAD[i]);
        }

        s_len = sizeof(Uint8) + sizeof(Uint32) * (s_pointsAD.num);

        s_data[0] = s_pointsAD.num;
        memcpy(s_data + 1, s_tempAD, s_len - 1);

        DncpStack_SendEvent(DSCP_EVENT_OMI_OPTICAL_AD_NOTICE, s_data, s_len);
    }
}

void Meter_SetMeterADReportPeriod(float period)
{
    TRACE_MARK("\n period:");
    System_PrintfFloat(TRACE_LEVEL_MARK, period, 3);
    TRACE_MARK(" s");
    MeterScheduler_SetMeterADReportPeriod(period);
}

/**
 * 打开定量使用的LED灯，仅供上层命令控制LED
 * @param num 定量点1到定量点METER_POINT_NUM
 * @return TRUE 操作成功，FALSE操作失败
 */
Bool Meter_TurnOnLED(Uint8 num)
{
    //return MeterLED_TurnOn(num);
    return TRUE;
}

/**
 * 关闭定量使用的LED灯，仅供上层命令控制LED
 * @param Num 定量点1到定量点METER_POINT_NUM
 * @return TRUE 操作成功，FALSE操作失败
 */
Bool Meter_TurnOffLED(Uint8 num)
{
    //return MeterLED_TurnOff(num);
    return TRUE;
}

Bool Meter_AutoCloseValve(Bool isAutoCloseValve)
{
    s_isAutoCloseValve = isAutoCloseValve;
    return TRUE;
}

Bool Meter_SetMeterFinishValveMap(Uint32 map)
{
    if (map <= SOLENOIDVALVE_MAX_MAP)
    {
        TRACE_INFO("\nSetMeterFinishValveMap: 0x%x", map);
        s_meterFinishValveMap = map;
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n The map must be 0 to 0x%x.", SOLENOIDVALVE_MAX_MAP);
        return FALSE;
    }
}

Uint32 Meter_GetSingleOpticalAD(Uint8 num)
{
    Uint32 value = 0;
    if (num <= s_meterPoint.num && num > 0)
    {
#if METER_SENSOR_TYPE == METER_SENSOR_DI
        value = SensorManager_ReadSensorStatus(num - 1);
#else
        value = MeterADC_GetCurAD(num - 1);
#endif
    }
    return value;
}

Bool Meter_SetRopinessOverValue(Uint16 value)
{
    if(value >= 50 && value <= 1000)
    {
        s_ropinessOverValue = value;
        TRACE_INFO("\n Set ropiness meter over value = %d success", value);
        return TRUE;
    }
    else
    {
        TRACE_INFO("\n Set ropiness meter over value %d fail: out of limit[50, 1000]", value);
        return FALSE;
    }
}

Uint16 Meter_GetRopinessOverValue()
{
    TRACE_INFO("\n Get ropiness meter over value = %d", s_ropinessOverValue);
    return s_ropinessOverValue;
}

Uint16 Meter_GetMeterThreshold(void)
{
    TRACE_INFO("\n Get meter Threshold = %d", s_meterThreshold);
    return s_meterThreshold;
}

void Meter_SetMeterThreshold(Uint16 value)
{
    TRACE_INFO("\n Set meter Threshold = %d", value);
    s_meterThreshold = value;
}

Uint16 Meter_GetMeterInterval(void)
{
    TRACE_INFO("\n Get meter Interval = %d", s_meterInterval);
    return s_meterInterval;
}

void Meter_SetMeterInterval(Uint16 value)
{
    TRACE_INFO("\n Set meter Interval = %d", value);
    s_meterInterval = value;
}

Bool Meter_WriteAccurateModeOverValve(Uint16 valve)
{
	if (valve>0 && valve<=METER_MAX_OVER_VALVE)
	{
		s_accurateOverValue = valve;
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}

Uint16 Meter_ReadAccurateModeOverValve()
{
    return s_accurateOverValue;
}
