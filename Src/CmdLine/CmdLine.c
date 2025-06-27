/**
 * @addtogroup module_CmdLine
 * @{
 */

/**
 * @file
 * @brief 应用接口：命令行实现。
 * @details 定义各应用命令及其处理函数。
 * @version 1.0.0
 * @author kim.xiejinqiang
 * @date 2012-5-21
 */


#include <OpticalDriver/MCP4651Driver.h>
//#include <OpticalDriver/OpticalADCollect.h>
#include <OpticalDriver/OpticalChannel.h>
//#include <OpticalDriver/OpticalLed.h>
#include <OpticalDriver/MCP4651Driver.h>
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>
#include <TempDriver/BoxFanDriver.h>
#include "Tracer/Trace.h"
#include "Console/Console.h"
#include "Driver/System.h"
#include "CmdLine.h"
#include "SystemConfig.h"
#include "DeviceIndicatorLED.h"
#include "UpdateDriver.h"
#include "DeviceUpdate/UpdateHandle.h"
#include "McuFlash.h"
#include "Driver/HardwareType.h"
#include "Manufacture/VersionInfo.h"
#include "OpticalMeter/Meter.h"
#include "PeristalticPump/PumpManager.h"
#include "PeristalticPump/TMCConfig.h"
#include "Driver/LiquidDriver/TMCConfigDriver.h"
#include "SolenoidValve/ValveManager.h"

#include "OpticalControl/OpticalControl.h"
//#include "OpticalControl/LEDController.h"
#include "OpticalControl/StaticADControl.h"

#include "Driver/TempDriver/TempCollecterMap.h"
#include "Driver/TempDriver/ThermostatDeviceMap.h"
#include "TemperatureControl/ThermostatDeviceManager.h"
#include "TemperatureControl/ThermostatManager.h"
#include "TemperatureControl/TempCollecterManager.h"

#include "CheckLeaking/CheckLeakingControl.h"
#include "SpinValve/SpinValve.h"
#include "SensorControl/SensorManager.h"
// 命令行版本定义，命令有变更时，需要相应更新本版本号
const CmdLineVersion g_kCmdLineVersion =
{
        1,      // 主版本号
        0,      // 次版本号
        0,      // 修订版本号
        0       // 编译版本号
};
static  Uint8 s_currPumpNum = 0;

// 命令处理函数声明列表
static int Cmd_help(int argc, char *argv[]);
static int Cmd_welcome(int argc, char *argv[]);
static int Cmd_version(int argc, char *argv[]);
static int Cmd_showparam(int argc, char *argv[]);
static int Cmd_reset(int argc, char *argv[]);
static int Cmd_trace(int argc, char *argv[]);
static int Cmd_demo(int argc, char *argv[]);

//系统命令
static int Cmd_flash(int argc, char *argv[]);
static int Cmd_RestoreInit(int argc, char *argv[]);
static int Cmd_IAP(int argc, char *argv[]);
static int Cmd_SetBlink(int argc, char *argv[]);
static int Cmd_TaskState(int argc, char *argv[]);
static int Cmd_Hardware(int argc, char *argv[]);

//泵命令函数
static int Cmd_PumpNum(int argc, char *argv[]);
static int Cmd_PumpStatus(int argc, char *argv[]);
static int Cmd_PumpParam(int argc, char *argv[]);
static int Cmd_PumpFactor(int argc, char *argv[]);
static int Cmd_TotalPumps(int argc, char *argv[]);
static int Cmd_PumpVolume(int argc, char *argv[]);
static int Cmd_Pump(int argc, char *argv[]);
//阀命令函数
static int Cmd_valve(int argc, char *argv[]);
//定量命令函数
static int Cmd_MeterPoints(int argc, char *argv[]);
static int Cmd_Meter(int argc, char *argv[]);
//static int Cmd_MeterLED(int argc, char *argv[]);
static int Cmd_MeterAD(int argc, char *argv[]);
static int Cmd_MeterStatus(int argc, char *argv[]);
static int Cmd_MeterFactor(int argc, char *argv[]);
//测量信号命令
static int Cmd_Xonen(int argc, char *argv[]);
//static int Cmd_MeasureLED(int argc, char *argv[]);
static int Cmd_ChangePolar(int argc, char *argv[]);
static int Cmd_ADChl(int argc, char *argv[]);
static int Cmd_OAI(int argc, char *argv[]);
static int Cmd_StaticAD(int argc, char *argv[]);
//温控命令
static int Cmd_BoxFan(int argc, char *argv[]);
static int Cmd_Therm(int argc, char *argv[]);
static int Cmd_Temp(int argc, char *argv[]);
static int Cmd_ThermParam(int argc, char *argv[]);
static int Cmd_TempFactor(int argc, char *argv[]);
static int Cmd_SetTempReport(int argc, char *argv[]);
static int Cmd_ThermDevice(int argc, char *argv[]);
//TMC驱动命令
int Cmd_TMC(int argc, char *argv[]);

//MCP4651测试
static int Cmd_MCP4651(int argc, char *argv[]);

//漏液检测命令
static int Cmd_ChcekLeaking(int argc, char *argv[]);
static int Cmd_SpinValve(int argc, char *argv[]);
static int Cmd_LiquidSensor(int argc, char *argv[]);
static void InfoOutput(void *argument);
/**
 * @brief 命令行命令表，保存命令关键字与其处理函数之间的对应关系。
 * @details 每条命令对应一个结点，结点包括：命令关键字、处理函数、简短描述文本。
 */
const CmdLineEntry g_kConsoleCmdTable[] =
{
    { "demo",       Cmd_demo,       "\t\t: Demo for cmd implementation and param parse" },
    { "trace",      Cmd_trace,      "\t\t: Trace level" },
    { "welcome",    Cmd_welcome,    "\t\t: Display welcome message" },
    { "version",    Cmd_version,    "\t\t: Display version infomation about this application" },
    { "reset",      Cmd_reset,      "\t\t: Reset system" },
    { "help",       Cmd_help,       "\t\t: Display list of commands. Short format: h or ?" },

    { "flash",      Cmd_flash,      "\t\t: Flash read write erase operation." },
    { "RI",         Cmd_RestoreInit, "\t\t:Restore system initial state." },
    { "iap",        Cmd_IAP,         "\t\t:Provides erase and jump operations." },
    { "blink",      Cmd_SetBlink,    "\t\t:Set the duration of equipment indicator, on time and off time.Uint milliseconds." },
    { "taskstate",  Cmd_TaskState,  "\t\t: Out put system task state." },
    { "hardware",   Cmd_Hardware,  "\t\t: Read Hardware Info." },

    { "valve",      Cmd_valve,      "\t\t: valve set read operation." },
    { "pumpfactor", Cmd_PumpFactor, "\t\t: Setting and acquisition of pump calibration coefficient." },
    { "pumpstatus", Cmd_PumpStatus, "\t\t: Get the running state of the pump." },
    { "pumpparam",  Cmd_PumpParam,  "\t\t: Get the running state of the pump." },
    { "pumptotal",  Cmd_TotalPumps, "\t\t: Get pump total." },
    { "pumpvolume", Cmd_PumpVolume, "\t\t: Gets the volume of a pump on the current pump ." },
    { "pump",       Cmd_Pump,       "\t\t: The start and stop of the pump. " },
    { "pumpnum",    Cmd_PumpNum,    "\t\t: Sets the current pump number. " },
    { "meterpoints",Cmd_MeterPoints,"\t\t: Set and check the volume of the Meter point. " },
    { "meter",      Cmd_Meter,      "\t\t: Meter start and stop operation." },
    { "meterad",    Cmd_MeterAD,  "\t\t: Get quantitative point AD." },
    { "meterstatus",Cmd_MeterStatus,"\t\t: Get meter status." },
    { "meterfactor", Cmd_MeterFactor, "\t\t: Setting and acquisition of meter pump calibration coefficient." },
    //{ "meterled",   Cmd_MeterLED,   "\t\t:Meter LED on and off operation " },
	{ "xonen",     Cmd_Xonen, "\t\t: turn on or turn off xonen" },
    //{ "mealed",     Cmd_MeasureLED, "\t\t: turn on or turn off Measure led" },
    { "polay",      Cmd_ChangePolar,"\t\t:Change AD7791 polarity mode." },
    { "optchl",     Cmd_ADChl,      "\t\t: Optical Channel Select." },
    { "oai",        Cmd_OAI,        "\t\t: AD Signal acquisition operation ." },
    { "staticad",   Cmd_StaticAD,        "\t\t: Static AD control operation ." },

    { "boxfan",     Cmd_BoxFan,     "\t\t:  boxfan control"},
    { "therm",      Cmd_Therm,      "\t\t: Thermostat temperature" },
    { "thermpid",   Cmd_ThermParam,"\t\t: The operation parameters of PID thermostat." },
    { "tempfactor", Cmd_TempFactor, "\t\t: Temperature sensor coefficient operation." },
    { "temp",       Cmd_Temp,       "\t\t: get temperature" },
    { "tempreport", Cmd_SetTempReport, "\t\t: Set the temperature reporting cycle." },
    { "thermdevice", Cmd_ThermDevice, "\t\t: " },

	{ "tmc",        Cmd_TMC,       "\t\t: tmc register read or write. " },
	{ "mcp",        Cmd_MCP4651,       "\t\t: MCP4651 register read or write. " },
	{ "check",      Cmd_ChcekLeaking,       "\t\t: start |stop | set the check leaking period " },
	{ "sensor", 	Cmd_LiquidSensor, "\t\t: Liquid Sensor" },
    { "spinvalve",      Cmd_SpinValve,      "\t\t: spin valve operation." },
    { "?",          Cmd_help,       0 },
    { "h",          Cmd_help,       0 },
    { "showparam",  Cmd_showparam,  0 },
    { 0, 0, 0 }
};


/**
 * @brief 判断第一个字串等于第二个字串。
 * @details 本函数与strcmp相比，预先做了有效性检查。
 * @param str1 要比较的字串1。
 * @param str2 要比较的字串2，不能为NULL。
 * @return 是否相等。
 */
Bool IsEqual(const char* str1, const char* str2)
{
    return (0 == strcmp(str1, str2)) ? TRUE : FALSE;
}

static xTaskHandle s_InfoOutputHandle;
static Uint16 s_getInfoTime = 0;
typedef enum
{
    LC,
    OA,
    TC,
    SYS,
}InfoOutputMode;

static InfoOutputMode s_InfoOutputMode = LC;

void CmdLine_Init(void)
{
    xTaskCreate(InfoOutput, "InfoOutput", CMDCLINE_INFOOUTPUT_STK_SIZE, NULL,
            CMDCLINE_INFOOUTPUT_TASK_PRIO, &s_InfoOutputHandle);
}

static void InfoOutput(void *argument)
{
    (void) argument;
    vTaskSuspend(NULL);
    while (1)
    {
        vTaskDelay(s_getInfoTime / portTICK_RATE_MS);
        switch(s_InfoOutputMode)
        {
            case LC:
                MeterADC_PrintfInfo();
                break;
            case OA:
                OpticalControl_PrintfInfo();
                break;
            case SYS:
                System_TaskStatePrintf();
//                System_StateTimerValue();
                break;
        }
    }
}

//*****************************************************************************
//
// 系统常规命令处理函数
//
//*****************************************************************************
#include "console/port/driver/ConsoleUart.h"

int Cmd_TaskState(int argc, char *argv[])
{
    if(IsEqual(argv[1], "start"))
    {
        if (argv[2] && atoi(argv[2]) >= 10)
        {
            s_getInfoTime = atoi(argv[2]);
            s_InfoOutputMode = SYS;
            vTaskResume(s_InfoOutputHandle);
        }
        else
        {
            Printf("Invalid param %s\n", argv[2]);
        }
    }
    else if(IsEqual(argv[1], "stop"))
    {
        vTaskSuspend(s_InfoOutputHandle);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== taskstate commands ======\n");
        Printf(" taskstate start [TIME]:ms\n");
        Printf(" taskstate stop        :\n");
    }
    return (0);
}

int Cmd_Hardware(int argc, char *argv[])
{
    Uint8 type = 0;
    if (IsEqual(argv[1], "type"))
    {
        type = HardwareType_GetValue();
        Printf("Hardware Type: %d\n", type);
    }
    else
    {
    	type = HardwareType_GetValue();
    }
    return 0;
}

int Cmd_SetBlink(int argc, char *argv[])
{
    if (argv[1])
    {
        if (argv[2] && argv[3])
        {
            DeviceIndicatorLED_SetBlink(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== blink commands ======\n");
        Printf(" blink [DURATION] [ONTIME] [OFFTIME]:\n");
    }

    return (0);
}

int Cmd_IAP(int argc, char *argv[])
{
    if (IsEqual(argv[1], "erase"))
    {
        UpdateHandle_StartErase();
    }
    else if (IsEqual(argv[1], "write"))
    {
        if (argv[2] && argv[3] && argv[4])
        {
            UpdateHandle_WriteProgram((u8 *)argv[2], atoi(argv[3]), atoi(argv[4]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "check"))
    {
        if (argv[2])
        {
            UpdateHandle_CheckIntegrity(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "read"))
    {
        if (argv[2] && argv[3])
        {
            uint8_t str[30]={""};
            UpdateDriver_Read(atoi(argv[2]),atoi(argv[3]),str);
            Printf("\n");
            for(int i = 0; i< atoi(argv[3]); i++)
            {
                Printf("0x%02x ",str[i]);
            }
            Printf("\n%s",str);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "getmax"))
    {
        Printf("%d\n", UpdateHandle_GetMaxFragmentSize());
    }
    else if (IsEqual(argv[1], "getmode"))
    {
        DeviceRunMode mode = UpdateHandle_GetRunMode();
        Printf("%d\n", mode);
    }
#ifdef _CS_APP
    else if (IsEqual(argv[1], "updater"))
    {
        UpdateHandle_EnterUpdater();
    }
#else
    else if (IsEqual(argv[1], "app"))
    {
        UpdateHandle_EnterApplication();
    }
#endif
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== iap commands ======\n");
        Printf(" iap erase: \n");
        Printf(" iap write [TEXT]   [NUM]  [SEQ]: \n");
        Printf(" iap check [CRC16]              : \n");
        Printf(" iap read  [OFFSET] [NUM]       : \n");
#ifdef _CS_APP
        Printf(" iap updater                    : \n");
#else
        Printf(" iap app                        : \n");
#endif
        Printf(" iap getmax                     : \n");
        Printf(" iap getmode                    : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
   
int Cmd_flash(int argc, char *argv[])
{
    if (IsEqual(argv[1], "deletewrite"))//不保留原始数据的写
    {
        if (argv[2] && argv[3] && argv[4])
        {
            McuFlash_DeleteWrite(atoi(argv[2]), atoi(argv[3]),(u8 *)argv[4]);
            Printf("\nWriteAddr 0x%x ",atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "write"))//保留原始数据的写
    {
        if (argv[2] && argv[3] && argv[4])
        {
            McuFlash_Write(atoi(argv[2]), atoi(argv[3]),(u8 *)argv[4]);
            Printf("\nWriteAddr 0x%x ",atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "read"))//读FLASH数据
    {
        if (argv[2] && argv[3])
        {
            uint8_t str[30]={""};
            McuFlash_Read(atoi(argv[2]),atoi(argv[3]),str);
            Printf("\nReadAddr 0x%x\n",atoi(argv[2]));
            for(int i = 0; i< atoi(argv[3]); i++)
            {
                Printf("0x%02x ",str[i]);
            }
            Printf("\n%s",str);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "erase"))//擦除
    {
        if (argv[2])
        {
           McuFlash_EraseSector(atoi(argv[2]));
           Printf("\nEraseAddr 0x%x", atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== flash commands ======\n");
        Printf(" flash deletewrite [ADDR] [NUM] [TEXT]: \n");
        Printf(" flash write       [ADDR] [NUM] [TEXT]: \n");
        Printf(" flash read        [ADDR] [NUM]       : \n");
        Printf(" flash erase       [ADDR]             : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_RestoreInit(int argc, char *argv[])
{
    //以下操作可能会因为没有启动而控制台出现错误提醒。
    Meter_Restore();//关闭定量，如果正在定量则最终会关闭阀，LED和AD，定量泵，
    PumpManager_Restore();//依次关闭所有的泵
    if (METER_STATUS_IDLE == Meter_GetMeterStatus())
    {
        ValveManager_SetValvesMap(0);//这只保证定量运行的时候不会关闭所有的阀。
    }

    ThermostatManager_RestoreInit();
    ThermostatDeviceManager_RestoreInit();
    OpticalControl_Restore();
    ///OpticalLed_TurnOff();
    ///Thermostat_RestoreInit();
    return 0;
}

//*****************************************************************************
//
// 光学定量命令处理函数
//
//*****************************************************************************
int Cmd_MeterFactor(int argc, char *argv[])
{
    if (IsEqual(argv[1], "set"))
    {
        if (argv[2])
        {
            PumpManager_SetFactor(METER_PUMP_NUM, atof(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "get"))
    {
        float factor = PumpManager_GetFactor(METER_PUMP_NUM);
        Printf("Pump %d factor ", METER_PUMP_NUM);
        System_PrintfFloat(1, factor, 8);
        Printf(" ml/step");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== meterfactor commands ======\n");
        Printf(" meterfactor set [FACTOR]: Set calibration FACTOR for meter pump.Uint is ml/step\n");
        Printf(" meterfactor get         : Calibration parameters for reading the meter pump \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_MeterStatus(int argc, char *argv[])
{
    if(METER_STATUS_IDLE == Meter_GetMeterStatus())
    {
        Printf("Meter:  IDLE\n");
    }
    else
    {
        Printf("Meter:  BUSY\n");
    }
    return (0);
}

int Cmd_MeterAD(int argc, char *argv[])
{
    if (IsEqual(argv[1], "get"))
    {
        OpticalMeterAD pointsAD;
        pointsAD = Meter_GetOpticalAD();
        Printf("\n point total:%d \n", pointsAD.num);
        for (Uint8 i = 0; i< pointsAD.num; i++)
        {
            Printf("%d point AD: %d\n", i, pointsAD.adValue[i]);
        }
    }
    else if (IsEqual(argv[1], "start"))
    {
        if (argv[2] && atoi(argv[2]) >= 10)
        {
            s_InfoOutputMode = LC;
            s_getInfoTime = atoi(argv[2]);
            vTaskResume(s_InfoOutputHandle);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        vTaskSuspend(s_InfoOutputHandle);
    }
    else if (IsEqual(argv[1], "report"))
    {
        if (argv[2] && atof(argv[2]) >= 0)
        {
            Meter_SetMeterADReportPeriod(atof(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== meterad commands ======\n");
        Printf(" meterad get            :Acquisition of all the meter point of ad.\n");
        Printf(" meterad start [TIME]   :Start to collect all the ad values of the channel.T(ms) to obtain the time get of ad.Need to set the trace level to 6 or 7.\n");
        Printf(" meterad stop           :Stop collecting ad values for all channels.\n");
        Printf(" meterad report [TIME]  :Sets the current ad value reporting period. Uint is s.\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_MeterPoints(int argc, char *argv[])
{
    MeterPoint dot;
    memset(&dot, 0, sizeof(MeterPoint));
    if (IsEqual(argv[1], "get"))
    {
        dot = Meter_GetMeterPoints();
        Printf(" num :%d \n", dot.num);
        for (Uint8 i = 0; i < METER_POINT_NUM; i++)
        {
            Printf("\n %d point SetV:", i + 1);
            System_PrintfFloat(1, dot.volume[i][SETVIDX], 2);
            Printf(" ml RealV:");
            System_PrintfFloat(1, dot.volume[i][REALIDX], 2);
            Printf(" ml");
        }
    }
    else if (IsEqual(argv[1], "set"))
    {
        if (argv[2])
        {
            Uint8 i;
            dot.num = atoi(argv[2]);
            if (dot.num > 0 && dot.num <= METER_POINT_NUM)
            {
                for (i = 0; i < dot.num;)
                {
                    if (argv[3 + 2 * i] && argv[4 + 2 * i])
                    {
                        dot.volume[i][SETVIDX] = atof(argv[3 + 2 * i]);
                        dot.volume[i][REALIDX] = atof(argv[4 + 2 * i]);
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (i == dot.num)
                {
                    Meter_SetMeterPoints(&dot);
                }
                else
                {
                    Printf("A meter point that is not equal to the input. \n");
                }
            }
            else
            {
                Printf(
                        "\n Set the volume of the meter point to fail because The point num must be 1 to %d.",
                        METER_POINT_NUM);
            }
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") || IsEqual(argv[1], "?"))
    {
        Printf("====== meterpoints commands ======\n");
        Printf(
                " meterpoints get                     : Get the volume of each meter point of the system. \n");
        Printf(
                " meterpoints set [NUM] [SETV] [REALV]: Set the SETV and REALV of the system NUM points.Unit ml.The point num must be 1 to %d.\n",
                METER_POINT_NUM);
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_Meter(int argc, char *argv[])
{
    if (IsEqual(argv[1], "start"))       //不保留原始数据的写
    {
        MeterMode mode;
        MeterDir dir;
        Bool isParamOK = FALSE;
        if (argv[2] && argv[3] && argv[4] && argv[5])
        {
            if (IsEqual(argv[2], "extract") || IsEqual(argv[2], "e"))
            {
                isParamOK = TRUE;
                dir = METER_DIR_EXTRACT;
            }
            else if (IsEqual(argv[2], "drain") || IsEqual(argv[2], "d"))
            {
                isParamOK = TRUE;
                dir = METER_DIR_DRAIN;
            }
            if (TRUE == isParamOK)
            {
                isParamOK = FALSE;
                if (IsEqual(argv[3], "accurate") || IsEqual(argv[3], "a"))
                {
                    mode = METER_MODE_ACCURATE;
                    isParamOK = TRUE;
                }
                else if (IsEqual(argv[3], "smart") || IsEqual(argv[3], "s"))
                {
                    mode = METER_MODE_SMART;
                    isParamOK = TRUE;
                }
                else if (IsEqual(argv[3], "direct") || IsEqual(argv[3], "d"))
                {
                    mode = METER_MODE_DIRECT;
                    isParamOK = TRUE;
                }
                else if (IsEqual(argv[3], "ropiness") || IsEqual(argv[3], "r"))
                {
                    mode = METER_MODE_ROPINESS;
                    isParamOK = TRUE;
                }
                if (TRUE == isParamOK)
                {
                    Meter_Start(dir, mode, atof(argv[4]), atof(argv[5]));
                }
                else
                {
                    Printf("Invalid param: %s\n", argv[3]);
                }
            }
            else
            {
                Printf("Invalid param: %s\n", argv[2]);
            }
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        Meter_RequestStop();
    }
    else if (IsEqual(argv[1], "speed"))
    {
        if (IsEqual(argv[2], "set"))
        {
            if (argv[3])
            {
                Meter_SetMeterSpeed(atof(argv[3]));
            }
            else
            {
                Printf("Invalid param\n");
            }
        }
        else if (IsEqual(argv[2], "get"))
        {
            Printf("Meter speed: ");
            System_PrintfFloat(1, Meter_GetMeterSpeed(), 6);
            Printf(" ml/s, ");
        }
    }
    else if (IsEqual(argv[1], "overValue"))
    {
        if (IsEqual(argv[2], "set"))
        {
            if (argv[3])
            {
                Meter_SetRopinessOverValue(atoi(argv[3]));
            }
            else
            {
                Printf("Invalid param\n");
            }
        }
        else if (IsEqual(argv[2], "get"))
        {
            Meter_GetRopinessOverValue();
        }
    }
    else if (IsEqual(argv[1], "accOverValve"))
    {
        if (IsEqual(argv[2], "set"))
        {
            if (argv[3])
            {
            	Meter_WriteAccurateModeOverValve(atoi(argv[3]));
            }
            else
            {
                Printf("Invalid param\n");
            }
        }
        else if (IsEqual(argv[2], "get"))
        {
        	Uint32 num = Meter_ReadAccurateModeOverValve();
        	Printf("\n accOverValve = %d",num);
        }
    }
    else if (IsEqual(argv[1], "threshold"))
    {
        if (IsEqual(argv[2], "set"))
        {
            if (argv[3])
            {
                Meter_SetMeterThreshold(atoi(argv[3]));
            }
            else
            {
                Printf("Invalid param\n");
            }
        }
        else if (IsEqual(argv[2], "get"))
        {
            Meter_GetMeterThreshold();
        }
    }
    else if (IsEqual(argv[1], "interval"))
    {
        if (IsEqual(argv[2], "set"))
        {
            if (argv[3])
            {
                Meter_SetMeterInterval(atoi(argv[3]));
            }
            else
            {
                Printf("Invalid param\n");
            }
        }
        else if (IsEqual(argv[2], "get"))
        {
            Meter_GetMeterInterval();
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") || IsEqual(argv[1], "?"))
    {
        Printf("====== meter commands ======\n");
        Printf(
                " meter start [DIR] [MODE] [V] [LIMITV]:DIR:extract(e),drain(d).MODE:accurate(a),direct(d),smart(s).V is meter volume.LIMITV is limiting volume.Unit mL.\n");
        Printf(" meter stop                                    : \n");
        Printf(" meter speed  set [SPEED]               : Unit ml/s.\n");
        Printf(" meter speed  get                              : Unit ml/s.\n");
        System_Delay(10);
        Printf(" meter overValue  set [VALUE]       : set ropiness meter over value\n");
        Printf(" meter overValue  get                      : get ropiness meter over value\n");
        System_Delay(10);
        Printf(" meter accOverValue  set [VALUE]       : set accurate meter over value\n");
        Printf(" meter accOerValue  get                      : get accurate meter over value\n");
        System_Delay(10);
        Printf(" meter threshold  set [VALUE]        : set  meter final judge threshold count\n");
        Printf(" meter threshold  get                       : get meter final judge threshold count\n");
        Printf(" meter interval  set [VALUE]        : set  meter ad diff interval count\n");
        Printf(" meter interval  get                       : get meter ad diff interval count\n");
        System_Delay(5);
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
/*
int Cmd_MeterLED(int argc, char *argv[])
{
    if (IsEqual(argv[1], "on"))
    {
        if (argv[2])
        {
            Meter_TurnOnLED(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "off"))
    {
        if (argv[2])
        {
            Meter_TurnOffLED(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") || IsEqual(argv[1], "?"))
    {
        Printf("====== meterled commands ======\n");
        Printf(" meterled on  [NUM]: Open NUM LED,NUM: 1 - %d.\n", METER_POINT_NUM);
        Printf(" meterled off [NUM]: Close NUM LED,NUM: 1 - %d.\n", METER_POINT_NUM);
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
*/

//*****************************************************************************
//
// 阀命令处理函数
//
//*****************************************************************************
int Cmd_valve(int argc, char *argv[])
{
    if (IsEqual(argv[1], "map"))
    {
        if (argv[2] )
        {
            ValveManager_SetValvesMap(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param");
        }
    }
    else if (IsEqual(argv[1], "open"))
    {
        if (argv[2] && atoi(argv[2]) > 0)
        {
            ValveManager_SetValvesMap((Uint32)( 1 << (atoi(argv[2]) - 1)));
        }
        else
        {
            Printf("Invalid param");
        }
    }
    else if (IsEqual(argv[1], "closeall"))
    {
        ValveManager_SetValvesMap(0);
    }
    else if (IsEqual(argv[1], "get"))
    {
        Uint32 map = ValveManager_GetValvesMap();
        Printf("ValvesMap: 0x%4x\n", map);
    }
    else if (IsEqual(argv[1], "total"))
    {
        Printf("total: %d\n", ValveManager_GetTotalValves());
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== valve commands ======\n");
        Printf(" valve map  [MAP]: Set the map of the valve. map must be 0x0 - 0x%x.\n\n", SOLENOIDVALVE_MAX_MAP);
        Printf(" valve open [NUM]: Open valve NUM.Num must be 1 - %d.\n", ValveManager_GetTotalValves());
        Printf(" valve closeall  : Close all valves. \n");
        Printf(" valve get       : Mapping map of the solenoid valve. \n");
        Printf(" valve total     : Total number of solenoid valves. \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

//*****************************************************************************
//
// 泵命令处理函数
//
//*****************************************************************************
int Cmd_PumpNum(int argc, char *argv[])
{
    if (argv[1])
    {
        if (atoi(argv[1]) < PumpManager_GetTotalPumps())
        {
            s_currPumpNum = atoi(argv[1]);
        }
        else
        {
            Printf("NUM must to be 0 - %d\n", PumpManager_GetTotalPumps() - 1);
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") || IsEqual(argv[1], "?"))
    {
        Printf("====== pumpnum commands ======\n");
        Printf(
                " pumpnum [NUM]: Set the pump for the current operation to NUM.NUM must to be 0 - %d\n",
                PumpManager_GetTotalPumps() - 1);
    }
    return (0);
}

int Cmd_PumpStatus(int argc, char *argv[])
{
    PumpStatus status = PumpManager_GetStatus(s_currPumpNum);
    switch(status)
    {
    case PUMP_IDLE:
        Printf("Pump: %d IDLE\n",s_currPumpNum);
        break;
    case PUMP_BUSY:
        Printf("Pump: %d BUSY\n",s_currPumpNum);
        break;
    }
    return (0);
}

int Cmd_PumpFactor(int argc, char *argv[])
{
    if (IsEqual(argv[1], "set"))
    {
        if (argv[2])
        {
            PumpManager_SetFactor(s_currPumpNum, atof(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "get"))
    {
        float factor = PumpManager_GetFactor(s_currPumpNum);
        Printf("Pump %d Factor ", s_currPumpNum);
        System_PrintfFloat(1, factor, 8);
        Printf(" ml/step");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== pumpfactor commands ======\n");
        Printf(" pumpfactor set [FACTOR]: Set calibration FACTOR for current pump.Unit is ml/step\n");
        Printf(" pumpfactor get         : Calibration parameters for reading the current pump.\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_PumpParam(int argc, char *argv[])
{
    if (IsEqual(argv[1], "set"))
    {
        if (argv[2] && argv[3])
        {
            PumpManager_SetMotionParam(s_currPumpNum,atof(argv[2]),atof(argv[3]), WriteFLASH);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "get"))
    {
        PumpParam param= PumpManager_GetMotionParam(s_currPumpNum);

        Printf("Pump %d acc:", s_currPumpNum);
        System_PrintfFloat(1, param.acceleration, 4);
        Printf(" ml/(s^2),maxSpeed:");
        System_PrintfFloat(1, param.maxSpeed, 4);
        Printf(" ml/s\n");
    }
    else if (IsEqual(argv[1], "setmoveing"))
    {
        if (argv[2] && argv[3])
        {
            PumpManager_SetMotionParam(s_currPumpNum,atof(argv[2]),atof(argv[3]), NoWriteFLASH);
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("========== pumpparam commands ==========\n");
        Printf(" pumpparam set         [ACC] [MAXSPEED]: Set the ACC(ml/(s^2)) and MAXSPEED(ml/s) of the current pump, the motion parameters are saved to flash. \n");
        Printf(" pumpparam get                         : Gets the motion parameters of the current pump.\n");
        Printf(" pumpparam setmoveing  [ACC] [MAXSPEED]: Set temporary acceleration and speed of the current pump.\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_TotalPumps(int argc, char *argv[])
{
    Printf("TotalPumps: %d\n", PumpManager_GetTotalPumps());
    return (0);
}

int Cmd_PumpVolume(int argc, char *argv[])
{
   if (IsEqual(argv[1], "change"))
   {
       if (argv[2])
       {
           PumpManager_ChangeVolume(s_currPumpNum, atof(argv[2]));
       }
       else
       {
           Printf("Invalid param\n");
       }
   }
   else if (IsEqual(argv[1], "get"))
   {
       float volume = PumpManager_GetVolume(s_currPumpNum);
       Printf("Pump %d Volume", s_currPumpNum);
       System_PrintfFloat(1, volume, 4);
       Printf(" ml");
   }
   else if (0 == argv[1] || IsEqual(argv[1], "help") ||
            IsEqual(argv[1], "?"))
   {
       Printf("====== pumpvolume commands ======\n");
       Printf(" pumpvolume change [VOLUME]: Change the volume of the pump. Uint ml\n");
       Printf(" pumpvolume get            : drain VOLUME ml\n");
   }
   return (0);
}

int Cmd_Pump(int argc, char *argv[])
{

    Bool isParamOK = FALSE;
    Direction dir;

    if (IsEqual(argv[1], "extract") || IsEqual(argv[1], "e"))
    {
        isParamOK = TRUE;
        dir = COUNTERCLOCKWISE;
    }
    else if (IsEqual(argv[1], "drain") || IsEqual(argv[1], "d"))
    {
        isParamOK = TRUE;
        dir = CLOCKWISE;
    }
    if(TRUE == isParamOK)
    {
        if(argv[2])
        {
            if(argv[3] && argv[4])
            {
                PumpManager_SetMotionParam(s_currPumpNum, atof(argv[3]), atof(argv[4]),
                        NoWriteFLASH);
                PumpManager_Start(s_currPumpNum, dir, atof(argv[2]), NoReadFLASH);
            }
            else
            {
                PumpManager_Start(s_currPumpNum,dir,atof(argv[2]), ReadFLASH);
            }
        }
        else
        {
            Printf("Invalid param\n");
        }

    }
    else if (IsEqual(argv[1], "stop"))
    {
        PumpManager_Stop(s_currPumpNum);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== pump commands ======\n");
        Printf(" pump extract(e) [VOLUME] [ACC] [SPEED]: Extract VOLUME ml.ACC and SPEED is optional.Uint ml/s.\n");
        Printf(" pump drain(d)   [VOLUME] [ACC] [SPEED]: drain VOLUME ml.ACC and SPEED is optional.Uint ml/s. \n");
        Printf(" pump stop                             : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);

}

//*****************************************************************************
//
// 测量信号命令处理函数
//
//*****************************************************************************
int Cmd_ADChl(int argc, char *argv[])
{
    if (IsEqual(argv[1], "open"))
    {
        if (argv[2])
        {
            if(atoi(argv[2]) >= 1 && atoi(argv[2]) <= 2)
            {
                OpticalControl_CollectTestFun(1, atoi(argv[2]));
            }
            else
            {
                Printf("\n Channel 1 - 2");
            }
        }
        else
        {
            Printf("\n Channel 1 - 2");
        }
    }
    else if (IsEqual(argv[1], "close"))
    {
        OpticalControl_CollectTestFun(0, 1);
        Printf("\n Close collect test fun.");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== optchl commands ======\n");
        Printf(" optchl open [CHL] :Single channel signal acquisition and open, Chl for the test channel \n");
        Printf(" optchl close      :Single channel signal acquisition and close.\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_OAI(int argc, char *argv[])
{
    if (IsEqual(argv[1], "getad"))
    {
        if(IsEqual(argv[2], "start"))
        {
            if (argv[3] && atoi(argv[3]) >= 10)
            {
                s_getInfoTime = atoi(argv[3]);
                s_InfoOutputMode = OA;
                vTaskResume(s_InfoOutputHandle);
            }
            else
            {
                Printf("Invalid param\n");
            }
        }
        else if(IsEqual(argv[2], "stop"))
        {
            vTaskSuspend(s_InfoOutputHandle);
        }
        else
        {
            Printf("\n Invalid param");
        }
    }
    else if (IsEqual(argv[1], "start"))
    {
        if(argv[2] && atof(argv[2]) >= 0)
        {
            OpticalControl_SendEventClose();
            OpticalControl_StartAcquirer(1, atof(argv[3]));
        }
        else
        {
            Printf("\n Invalid param");
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        OpticalControl_SendEventClose();
        OpticalControl_StopAcquirer();
    }
    else if (IsEqual(argv[1], "report"))
    {
        if (argv[2] && atof(argv[2]) >= 0)
        {
            OpticalControl_SetSignalADNotifyPeriod(atof(argv[2]));
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== oai commands ======\n");
        Printf(" oai getad  start  [TIME]:Ad value of the optical is continuously display.Display interval [TIME].Uint is s.\n");
        Printf(" oai getad  stop         :Stop continue to collect the ad value of the current channel.\n");
        Printf(" oai start  [TIME]       :Start collecte ad value of the mea and the ref,acquisition time TIME s.\n");
        Printf(" oai stop                : \n");
        Printf(" oai report [TIME]      :Sets the current ad value reporting period. Uint is s.\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_StaticAD(int argc, char *argv[])
{
    if (IsEqual(argv[1], "start"))
    {
        if(argv[2])
        {
            if(argv[3] && argv[4])
            {
                StaticADControl_Start(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
            }
            else
            {
                Printf("Lack of param target\n");
            }
        }
        else
        {
            Printf("Lack of param index\n");
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        StaticADControl_Stop();
    }
    else if (IsEqual(argv[1], "isValid"))
    {
        if(TRUE == StaticADControl_IsValid())
        {
            Printf("AD control is valid\n");
        }
        else
        {
            Printf("AD control is invalid\n");
        }
    }
    else if (IsEqual(argv[1], "get"))
    {
        if(IsEqual(argv[2], "default"))
        {
            if(argv[3] && argv[4])
            {
                StaticADControl_GetDefaultValue(atoi(argv[3]), atoi(argv[4]));
            }
            else
            {
                Printf("Lack of param index\n");
            }
        }
        else if(IsEqual(argv[2], "real"))
        {
            if(argv[3])
            {
                StaticADControl_GetRealValue(atoi(argv[3]));
            }
            else
            {
                Printf("Lack of param index\n");
            }
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "set"))
    {
        if(IsEqual(argv[2], "default"))
        {
            if(argv[3]&&argv[4]&&argv[5])
            {
                StaticADControl_SetDefaultValue(atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
            }
            else
            {
                Printf("Invalid param index or value\n");
            }
        }
        else if(IsEqual(argv[2], "real"))
        {
            if(argv[3]&&argv[4])
            {
                StaticADControl_SetRealValue(atoi(argv[3]), atoi(argv[4]));
            }
            else
            {
                Printf("Invalid param index or value\n");
            }
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== Static AD Control Commands ======\n");
        Printf(" staticad start  [index] [target] [isExtLED]: start static ad control. [index: ref=0,mea=1,meter1=2,meter2=3] isExt:0-1\n");
        Printf(" staticad stop : staticad control stop \n");
        System_Delay(3);
        Printf(" staticad get default  [index]  [isExtLED]: get default value from flash \n");
        Printf(" staticad set default  [index] [value]  [isExtLED]: set default value to flash \n");
        System_Delay(3);
        Printf(" staticad get real  [index] : get rdac real value\n");
        Printf(" staticad set real  [index] [value] : set rdac real value \n");
        Printf(" staticad isValid : return ad control is valid or invalid \n");

    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}

int Cmd_Xonen(int argc, char *argv[])
{
    if (IsEqual(argv[1], "on"))
    {
        OpticalControl_TurnOnXonen(TRUE);
        Printf("The xonen on \n");
    }
    else if (IsEqual(argv[1], "off"))
    {
        OpticalControl_TurnOnXonen(FALSE);
        Printf("The xonen off \n");
    }
    else if (IsEqual(argv[1], "glitter"))
    {
        OpticalControl_GlitterXonen(TRUE);
        Printf("The xonen glitter \n");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== xonen commands ======\n");
        Printf(" xonen [ctrl] : turn off/on/glitter \n");
    }
    return (0);
}

//int Cmd_MeasureLED(int argc, char *argv[])
//{
//    if (IsEqual(argv[1], "on"))
//    {
//        if(argv[2])
//        {
//            LEDController_TurnOnLed(atoi(argv[2]));
//        }
//        else
//        {
//            TRACE_ERROR("\nLack of Param");
//        }
//    }
//    else if (IsEqual(argv[1], "start"))
//    {
//        if(argv[2])
//        {
//            LEDController_Start(atoi(argv[2]));
//        }
//        else
//        {
//            TRACE_ERROR("\nLack of Param");
//        }
//    }
//    else if (IsEqual(argv[1], "close"))
//    {
//        if(argv[2])
//        {
//            LEDController_Stop(atoi(argv[2]));
//        }
//        else
//        {
//            TRACE_ERROR("\nLack of Param");
//        }
//    }
//    else if (IsEqual(argv[1], "adjust"))
//    {
//        if (IsEqual(argv[2], "start"))
//        {
//            if (argv[3] && argv[4] && argv[5] && argv[6])
//            {
//                LEDController_SendEventClose(atoi(argv[3]));
//                LEDController_AdjustToValue(atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]));
//            }
//            else
//            {
//                Printf("No  enough param\n");
//            }
//        }
//        else if (IsEqual(argv[2], "stop"))
//        {
//            if (argv[3])
//            {
//                LEDController_SendEventClose(atoi(argv[3]));
//                LEDController_AdjustStop(atoi(argv[3]));
//            }
//            else
//            {
//                Printf("No  enough param\n");
//            }
//        }
//        else
//        {
//            Printf("Invalid param: %s\n", argv[2]);
//        }
//    }
//    else if (IsEqual(argv[1], "target"))
//    {
//        if (IsEqual(argv[2], "get"))
//        {
//            if (argv[3])
//            {
//                Printf("\n LED controller target: %d", LEDController_GetTarget(atoi(argv[3])));
//            }
//            else
//            {
//                Printf("No  enough param\n");
//            }
//        }
//        else if (IsEqual(argv[2], "set"))
//        {
//            if (argv[3] && argv[4] && atoi(argv[4]) > 0)
//            {
//                LEDController_SetTarget(atoi(argv[3]), atoi(argv[4]));
//            }
//            else
//            {
//                Printf("Invalid param: %s\n", argv[3]);
//            }
//
//        }
//        else
//        {
//            Printf("Invalid param: %s\n", argv[2]);
//        }
//    }
//    else if (IsEqual(argv[1], "pid"))
//    {
//        LEDControllerParam ledControllerParam;
//       if (IsEqual(argv[2], "get"))
//       {
//           if (argv[3])
//           {
//               ledControllerParam = LEDController_GetParam(atoi(argv[3]));
//               Printf("\n p = %f, i = %f, d = %f", ledControllerParam.proportion, ledControllerParam.integration, ledControllerParam.differential);
//           }
//           else
//           {
//               Printf("Invalid param: %s\n", argv[3]);
//           }
//       }
//       else if (IsEqual(argv[2], "set"))
//       {
//           if (argv[3] && argv[4] && argv[5] && argv[6])
//           {
//               ledControllerParam.proportion = atof(argv[4]);
//               ledControllerParam.integration = atof(argv[5]);
//               ledControllerParam.differential = atof(argv[6]);
//               LEDController_SetParam(atoi(argv[3]), ledControllerParam);
//           }
//           else
//           {
//               Printf("Invalid param\n");
//           }
//       }
//       else
//       {
//           Printf("Invalid param: %s\n", argv[2]);
//       }
//    }
//    else if (IsEqual(argv[1], "dacdef"))
//    {
//        if (IsEqual(argv[2], "get"))
//        {
//            if(argv[3])
//            {
//                Printf("\n LED DAC default value : %f", OpticalLed_GetDefaultValue());
//            }
//            else
//            {
//                Printf("Invalid param: %s\n", argv[3]);
//            }
//        }
//        else if (IsEqual(argv[2], "set"))
//        {
//            if (argv[3] )
//            {
//                OpticalLed_SetDefaultValue(atof(argv[3]));
//            }
//            else
//            {
//                Printf("Invalid param: %s\n", argv[3]);
//            }
//        }
//        else
//        {
//            Printf("Invalid param: %s\n", argv[2]);
//        }
//    }
//    else if (IsEqual(argv[1], "dacout"))
//    {
//        if (argv[2] >= 0)
//        {
//            OpticalLed_ChangeDACOut(atof(argv[2]));
//        }
//        else
//        {
//            Printf("Invalid param: %s \n", argv[2]);
//        }
//    }
//    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
//             IsEqual(argv[1], "?"))
//    {
//        Printf("====== mealed commands ======\n");
//        Printf(" mealed on [index]                : Just open LED. index: 0 - %d\n", DAC_CHANNEL_NUM - 1);
//        Printf(" mealed start [index]               : Turn on the LED light and start the PID controller.\n");
//        Printf(" mealed close [index]               : Turn off the LED light. If the PID controller is started, it will be closed. \n");
//        System_Delay(10);
//        Printf(" mealed adjust start [index][targetAD][tolerance][timeout]              : Turn on the LED light and adjust the PID to targetAD.\n");
//        Printf(" mealed adjust stop [index]             : Turn off the LED light and stop the PID adjust.\n");
//        System_Delay(10);
//        Printf(" mealed target get [index]           : Get PID target value.\n");
//        Printf(" mealed target set [index][AD]    : Set PID target value.\n");
//        System_Delay(10);
//        Printf(" mealed pid get [index]           : Get PID parameters. \n");
//        Printf(" mealed pid set [index] [P] [I] [D]: Set PID parameters. \n");
//        System_Delay(10);
//        Printf(" mealed dacdef get [index]   : Get the LED DAC default voltage. \n");
//        Printf(" mealed dacdef set [index][v]    : Set the LED DAC default voltage. \n");
//        Printf(" mealed dacout [index][v]     : Change the control voltage of LED. \n");
//    }
//    else
//    {
//        Printf("Invalid param: %s\n", argv[1]);
//    }
//    return (0);
//}

int Cmd_ChangePolar(int argc, char *argv[])
{
    if (IsEqual(argv[1], "unipolay"))
    {
//        OpticalADCollect_ChangePolar(AD7791_MODE_UNIPOLAR);
    }
    else if (IsEqual(argv[1], "bipolay"))
    {
//        OpticalADCollect_ChangePolar(AD7791_MODE_BIPOLAR);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== polay commands ======\n");
        Printf(" polay unipolay: \n");
        Printf(" polay bipolay : \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return 0;
}

//*****************************************************************************
//
// 温控命令处理函数
//
//*****************************************************************************
int Cmd_Therm(int argc, char *argv[])
{
    if (IsEqual(argv[1], "auto"))
    {
        if(argv[2] && argv[3] && argv[4] && argv[5])
        {
            ThermostatManager_SendEventClose(atoi(argv[2]));
            ThermostatManager_Start(atoi(argv[2]),
                    THERMOSTAT_MODE_AUTO, atof(argv[3]), atof(argv[4]), atof(argv[5]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "heater"))
    {
        if(argv[2] && argv[3] && argv[4] && argv[5])
        {
            ThermostatManager_SendEventClose(atoi(argv[2]));
            ThermostatManager_Start(atoi(argv[2]),
                    THERMOSTAT_MODE_HEATER, atof(argv[3]), atof(argv[4]), atof(argv[5]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "refrigerate"))
    {
        if(argv[2] && argv[3] && argv[4] && argv[5])
        {
            ThermostatManager_SendEventClose(atoi(argv[2]));
            ThermostatManager_Start(atoi(argv[2]),
                    THERMOSTAT_MODE_REFRIGERATE, atof(argv[3]), atof(argv[4]), atof(argv[5]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "natural"))
    {
        if(argv[2] && argv[3] && argv[4] && argv[5])
        {
            ThermostatManager_SendEventClose(atoi(argv[2]));
            ThermostatManager_Start(atoi(argv[2]),
                    THERMOSTAT_MODE_NATURAL, atof(argv[3]), atof(argv[4]), atof(argv[5]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        if(argv[2])
        {
            ThermostatManager_SendEventClose(atoi(argv[2]));
            ThermostatManager_RequestStop(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "status"))
    {
        if(argv[2])
        {
            ThermostatStatus result = ThermostatManager_GetStatus(atoi(argv[2]));
            if (THERMOSTAT_IDLE == result)
            {
                Printf("%s Thermostat IDLE \n", ThermostatManager_GetName(atoi(argv[2])));
            }
            else
            {
                Printf("%s Thermostat BUSY \n", ThermostatManager_GetName(atoi(argv[2])));
            }
        }
        else
        {
            Printf("Invalid param\n");
        }
    }
    else if (IsEqual(argv[1], "number"))
    {
        Printf("\n GetTotalThermostat %d", TOTAL_THERMOSTAT);
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== therm commands ======\n");
        Printf(" therm auto        [INDEX] [TEMP] [ALW] [TIME]: \n");
        Printf(" therm heater      [INDEX] [TEMP] [ALW] [TIME]: \n");
        Printf(" therm refrigerate [INDEX] [TEMP] [ALW] [TIME]: \n");
        System_Delay(2);
        Printf(" therm natural     [INDEX] [TEMP] [ALW] [TIME]: \n");
        Printf(" therm stop        [INDEX]                    : \n");
        Printf(" therm status      [INDEX]                    : \n");
        Printf(" therm number                                 : \n");
        System_Delay(10);
        for (Uint8 i = 0; i < TOTAL_THERMOSTAT; i++)
        {
            Printf(" No %d name %s\n", i, ThermostatManager_GetName(i));
        }
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}

int Cmd_Temp(int argc, char *argv[])
{
    if (IsEqual(argv[1], "get"))
    {
        if(argv[2])
        {
            float temp = TempCollecterManager_GetTemp(atoi(argv[2]));
            Printf("\n %s Temp :  %f", TempCollecterManager_GetName(atoi(argv[2])), temp);
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "env"))
    {
        Printf("\n EnvironmentTemp :  %f", TempCollecterManager_GetEnvironmentTemp());
    }
    else if (IsEqual(argv[1], "all"))
    {
        Printf("\n EnvironmentTemp :  %f", TempCollecterManager_GetEnvironmentTemp());
        for (Uint8 i = 0; i < TOTAL_THERMOSTAT; i++)
        {
            float temp = ThermostatManager_GetCurrentTemp(i);
            Printf("\n %s Thermostat Temp :  %f", ThermostatManager_GetName(i), temp);
        }
    }
    else if (IsEqual(argv[1], "factor"))
    {
        TempCalibrateParam calibrateFactor;
        if (IsEqual(argv[2], "get"))
        {
            if (argv[3])
            {
                calibrateFactor = TempCollecterManager_GetCalibrateFactor(atoi(argv[3]));
                TRACE_INFO("\n %s Temp \n negativeInput = ", TempCollecterManager_GetName(atoi(argv[3])));
                System_PrintfFloat(1, calibrateFactor.negativeInput, 8);
                Printf("\n vref =");
                System_PrintfFloat(1, calibrateFactor.vref, 8);
                Printf("\n vcal =");
                System_PrintfFloat(1, calibrateFactor.vcal, 8);
            }
            else
            {
                Printf("Invalid param!\n");
            }
        }
        else if (IsEqual(argv[2], "set"))
        {
            if (argv[3] && argv[4] && argv[5] && argv[6])
            {
                calibrateFactor.negativeInput = atof(argv[4]);
                calibrateFactor.vref = atof(argv[5]);
                calibrateFactor.vcal = atof(argv[6]);
                TempCollecterManager_SetCalibrateFactor(atoi(argv[3]), calibrateFactor);
                Printf("\n set ok");
            }
            else
            {
                Printf("Invalid param!\n");
            }
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== temp commands ======\n");
        Printf(" temp get [ID]                         :\n");
        Printf(" temp env                              :\n");
        Printf(" temp all                              :\n");
        Printf(" temp factor get [INDEX]               :\n");
        Printf(" temp factor set [INDEX] [NI] [VR] [VC]:\n");
        for (Uint8 i = 0; i < TOTAL_TEMP; i++)
        {
            Printf(" No %d name %s\n", i, TempCollecterManager_GetName(i));
        }
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_SetTempReport(int argc, char *argv[])
{
    if (argv[1] && atof(argv[1]) >= 0)
    {
        ThermostatManager_SetTempReportPeriod(atof(argv[1]));
        Printf("\n set ok");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== tempreport commands ======\n");
        Printf(" tempreport [TIME] :\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_TempFactor(int argc, char *argv[])
{
    TempCalibrateParam calibrateFactor;
    if (IsEqual(argv[1], "get"))
    {
        if (argv[2])
        {
            calibrateFactor = ThermostatManager_GetCalibrateFactor(atoi(argv[2]));
            TRACE_INFO("\n %s Thermostat \n negativeInput = ", ThermostatManager_GetName(atoi(argv[2])));
            System_PrintfFloat(1, calibrateFactor.negativeInput, 8);
            Printf("\n vref =");
            System_PrintfFloat(1, calibrateFactor.vref, 8);
            Printf("\n vcal =");
            System_PrintfFloat(1, calibrateFactor.vcal, 8);
        }
        else
        {
            Printf("Invalid param!\n");
        }
    }
    else if (IsEqual(argv[1], "set"))
    {
        if (argv[2] && argv[3] && argv[4] && argv[5])
        {
            calibrateFactor.negativeInput = atof(argv[3]);
            calibrateFactor.vref = atof(argv[4]);
            calibrateFactor.vcal = atof(argv[5]);
            ThermostatManager_SetCalibrateFactor(atoi(argv[2]), calibrateFactor);
            Printf("\n set ok");
        }
        else
        {
            Printf("Invalid param!\n");
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== tempfactor commands ======\n");
        Printf(" tempfactor get [INDEX]               :\n");
        Printf(" tempfactor set [INDEX] [NI] [VR] [VC]:\n");
        System_Delay(10);
        for (Uint8 i = 0; i < TOTAL_THERMOSTAT; i++)
        {
            Printf(" No %d name %s\n", i, ThermostatManager_GetName(i));
        }
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_ThermParam(int argc, char *argv[])
{
   ThermostatParam thermostatparam;
   if (IsEqual(argv[1], "get"))
   {
       if (argv[2])
       {
           thermostatparam = ThermostatManager_GetPIDParam(atoi(argv[2]));
           Printf("\n %s Thermostat p =", ThermostatManager_GetName(atoi(argv[2])));
           System_PrintfFloat(1, thermostatparam.proportion, 3);
           Printf("\n i =");
           System_PrintfFloat(1, thermostatparam.integration, 3);
           Printf("\n d =");
           System_PrintfFloat(1, thermostatparam.differential, 3);
       }
       else
       {
           Printf("Invalid param: %s\n", argv[2]);
       }
   }
   else if (IsEqual(argv[1], "set"))
   {
       if (argv[2] && argv[3] && argv[4] && argv[5])
       {
           thermostatparam.proportion = atof(argv[3]);
           thermostatparam.integration = atof(argv[4]);
           thermostatparam.differential = atof(argv[5]);
           Printf("\n set %d ", ThermostatManager_SetPIDParam(atoi(argv[2]), thermostatparam));
       }
       else
       {
           Printf("Invalid param: 1:%s 2:%s 3:%s 4:%s\n", argv[2], argv[3], argv[4], argv[5]);
       }
   }
   else if (IsEqual(argv[1], "getcur"))
   {
       if (argv[2])
       {
           thermostatparam = ThermostatManager_GetCurrentPIDParam(atoi(argv[2]));
           Printf("\n %s Thermostat p =", ThermostatManager_GetName(atoi(argv[2])));
           System_PrintfFloat(1, thermostatparam.proportion, 3);
           Printf("\n i =");
           System_PrintfFloat(1, thermostatparam.integration, 3);
           Printf("\n d =");
           System_PrintfFloat(1, thermostatparam.differential, 3);
       }
       else
       {
           Printf("Invalid param: %s\n", argv[2]);
       }
   }
   else if (IsEqual(argv[1], "setcur"))
   {
       if (argv[2] && argv[3] && argv[4] && argv[5])
       {
           thermostatparam.proportion = atof(argv[3]);
           thermostatparam.integration = atof(argv[4]);
           thermostatparam.differential = atof(argv[5]);
           Printf("\n set %d ", ThermostatManager_SetCurrentPIDParam(atoi(argv[2]), thermostatparam));
       }
       else
       {
           Printf("Invalid param: 1:%s 2:%s 3:%s 4:%s\n", argv[2], argv[3], argv[4], argv[5]);
       }
   }
   else if (0 == argv[1] || IsEqual(argv[1], "help") || IsEqual(argv[1], "?"))
   {
       Printf("====== thermpid commands ======\n");
       Printf(" thermpid get    [INDEX]            :\n");
       Printf(" thermpid set    [INDEX] [P] [I] [D]:\n");
       Printf(" thermpid getcur [INDEX]            :\n");
       Printf(" thermpid setcur [INDEX] [P] [I] [D]:\n");
       System_Delay(10);
       for (Uint8 i = 0; i < TOTAL_THERMOSTAT; i++)
       {
           Printf(" No %d name %s\n", i, ThermostatManager_GetName(i));
       }
   }
   else
   {
       Printf("Invalid param: %s\n", argv[1]);
   }
   return (0);
}

int Cmd_BoxFan(int argc, char *argv[])
{
    if (IsEqual(argv[1], "level"))
    {
        if(argv[2] && atof(argv[2]) >= 0 && atof(argv[2]) <= 1)
        {
            BoxFanDriver_SetOutput(atof(argv[2]));
            Printf("\n======Boxfan Out ======");
        }
        else
        {
            Printf("Invalid param!level: 0 - 1 .\n");
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        BoxFanDriver_SetOutput(0);
        Printf("\n======Boxfan Stop======");
    }
    else if (IsEqual(argv[1], "status"))
    {
        Printf("\n  Boxfan Is Open %d", BoxFanDriver_IsOpen());
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== boxfan commands ======\n");
        Printf(" boxfan level [PER]: boxfan output\n");
        Printf(" boxfan stop       :\n");
        Printf(" boxfan status       : return boxfan status\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}
 
 int Cmd_ThermDevice(int argc, char *argv[])
{
    if (IsEqual(argv[1], "level"))
    {
        if(argv[2] && argv[3])
        {
            Printf("%s Device SetOutput :", ThermostatDeviceManager_GetName(atoi(argv[2])));
            System_PrintfFloat(1, atof(argv[3]) * 100, 3);
            Printf(" %%");
            ThermostatDeviceManager_SetOutput(atoi(argv[2]), atof(argv[3]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
        if(argv[2])
        {
            Printf("%s Device stop", ThermostatDeviceManager_GetName(atoi(argv[2])));
            ThermostatDeviceManager_SetOutput(atoi(argv[2]), 0);
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "isopen"))
    {
        if(argv[2])
        {
            Printf("\n %s ThermDevice ", ThermostatDeviceManager_GetName(atoi(argv[2])));
            ThermostatDeviceManager_IsOpen(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "setduty"))
    {
        if(argv[2] && argv[3])
        {
            Printf("\n %s Device  SetHeaterMaxDutyCycle :", ThermostatDeviceManager_GetName(atoi(argv[2])));
            System_PrintfFloat(1, atof(argv[3]) * 100, 3);
            Printf(" %%");
            ThermostatDeviceManager_SetMaxDutyCycle(atoi(argv[2]), atof(argv[3]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "getduty"))
    {
        if(argv[2])
        {
            Printf("\n %s Device max duty cycle:", ThermostatDeviceManager_GetName(atoi(argv[2])));
            ThermostatDeviceManager_GetMaxDutyCycle(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
            IsEqual(argv[1], "?"))
    {
        Printf("====== thermdevice commands ======\n");
        Printf(" thermdevice level   [INDEX] [PER] : 0 - %d device output\n", TOTAL_THERMOSTATDEVICE);
        Printf(" thermdevice stop    [INDEX]       :\n");
        Printf(" thermdevice isopen  [INDEX]       :\n");
        Printf(" thermdevice setduty [INDEX] [DUTY]:\n");
        Printf(" thermdevice getduty [INDEX]       :\n");
        for (Uint8 i = 0; i < TOTAL_THERMOSTATDEVICE; i++)
        {
            Printf(" No %d name %s\n", i, ThermostatDeviceManager_GetName(i));
            System_Delay(1);
        }
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

 int Cmd_TMC(int argc, char *argv[])
 {
     Uint32 data;

     if (IsEqual(argv[1], "list"))
      {
          if (argv[2])
          {
              TMCConfig_RegList(atoi(argv[2]));
          }
          else
          {
              Printf("Invalid param");
          }
      }
     else if (IsEqual(argv[1], "init"))
     {
         TMCConfig_Reinit();
     }
     else if (IsEqual(argv[1], "read"))
      {
          if (argv[2] && argv[3])
          {
              if(TRUE == TMCConfig_ReadData(atoi(argv[2]), atoi(argv[3]), &data))
              {
                  Printf("\nTMC Read Data  = 0x%08X", data);
              }
          }
          else
          {
              Printf("Invalid param");
          }
      }
      else if (IsEqual(argv[1], "write"))
      {
          if (argv[2] && argv[3] && argv[4])
          {
              TMCConfig_WriteData(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
          }
          else
          {
              Printf("Invalid param");
          }
      }
      else if (IsEqual(argv[1], "err"))
      {
          if (argv[2])
          {
              TMCConfig_ReadDriveError(atoi(argv[2]));
          }
          else
          {
              Printf("Invalid param");
          }
      }
      else if (IsEqual(argv[1], "div"))
      {
          if (IsEqual(argv[2], "get") && argv[3])
          {
              TMCConfig_ReadSubdivision(atoi(argv[3]));
          }
          else if(IsEqual(argv[2], "set") && argv[3] && argv[4])
          {
              TMCConfig_WriteSubdivision(atoi(argv[3]), atoi(argv[4]));
          }
          else
          {
              Printf("Invalid param");
          }
      }
      else if (IsEqual(argv[1], "cur"))
      {
          if(IsEqual(argv[2], "set") && argv[3] && argv[4] && argv[5] && argv[6])
          {
              TMCConfig_CurrentSet(atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]));
          }
          else
          {
              Printf("Invalid param");
          }
      }
      else if (0 == argv[1] || IsEqual(argv[1], "help") ||
               IsEqual(argv[1], "?"))
      {
          Printf("====== SpinValve commands ======\n");
          Printf(" tmc init       : all tmc driver reinit . \n");
          Printf(" tmc list [SLAVE]        : read tmc driver [n] all register . \n");
          Printf(" tmc err [SLAVE]        : read tmc driver [n] error . \n");
          System_Delay(5);
          Printf(" tmc div get[SLAVE]      : tmc read subdivision. \n");
          Printf(" tmc div set [SLAVE] [DIV]    : tmc write subdivision. \n");
          Printf(" tmc cur set [SLAVE][IHOLD][IRUN][DLY]  : tmc driver current set. \n");
          System_Delay(5);
          Printf(" tmc read [SLAVE] [REG]    : tmc read register. \n");
          Printf(" tmc write [SLAVE] [REG] [DATA]     : tmc write register. \n");
      }
      else
      {
          Printf("Invalid param: %s\n", argv[1]);
      }
      return (0);
 }
 
//*****************************************************************************
//
// 命令处理函数
//
//*****************************************************************************
// 显示帮助，简单显示命令列表
int Cmd_help(int argc, char *argv[])
{
    CmdLineEntry *cmdEntry;

    ConsoleOut("\nAvailable commands\n");
    ConsoleOut("------------------\n");

    cmdEntry = (CmdLineEntry *) &g_kConsoleCmdTable[0];

    // 遍历整个命令表，打印出有提示信息的命令
    while (cmdEntry->cmdKeyword)
    {
        // 延时一下，等待控制台缓冲区空出足够的空间
        System_Delay(10);

        if (cmdEntry->cmdHelp)
            ConsoleOut("%s%s\n", cmdEntry->cmdKeyword, cmdEntry->cmdHelp);

        cmdEntry++;
    }

    return (0);
}

int Cmd_version(int argc, char *argv[])
{
    ManufVersion softVer = VersionInfo_GetSoftwareVersion();
    ManufVersion hardVer = VersionInfo_GetHardwareVersion();
    ConsoleOut("Cmd Ver: %d.%d.%d.%d\n", g_kCmdLineVersion.major, g_kCmdLineVersion.minor, g_kCmdLineVersion.revision, g_kCmdLineVersion.build);
    ConsoleOut("Soft Ver: %d.%d.%d.%d\n", softVer.major, softVer.minor, softVer.revision, softVer.build);
    ConsoleOut("Pcb Ver: %d.%d.%d.%d\n", hardVer.major, hardVer.minor, hardVer.revision, hardVer.build);

    return(0);
}

int Cmd_welcome(int argc, char *argv[])
{
    Console_Welcome();
    return(0);
}


// 显示参数
int Cmd_showparam(int argc, char *argv[])
{
    int i = 0;

    ConsoleOut("The params is:\n");
    for (i = 1; i < argc; i++)
    {
        ConsoleOut("    Param %d: %s\n", i, argv[i]);
    }

    return(0);
}


int Cmd_reset(int argc, char *argv[])
{
    Printf("\n\n\n");
    System_Delay(10);
    System_Reset();
    return (0);
}

int Cmd_trace(int argc, char *argv[])
{
    if (argv[1])
    {
        Trace_SetLevel(atoi(argv[1]));
    }
    else
    {
        Printf("L: %d\n", Trace_GetLevel());
    }

    return (0);
}

// 命令处理函数示例
int Cmd_demo(int argc, char *argv[])
{
    if (IsEqual(argv[1], "subcmd1"))
    {
        if (IsEqual(argv[2], "param"))
        {
            // 调用相关功能函数
            Printf("Exc: subcmd1 param");
        }
    }
    else if (IsEqual(argv[1], "subcmd2"))
    {
        Printf("Exc: subcmd2");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== Sub commands ======\n");
        Printf(" mycmd subcmd1 param : Sub command description\n");
        Printf(" mycmd subcmd2       : Sub command description\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}

int Cmd_MCP4651(int argc, char *argv[])
{
	if (IsEqual(argv[1], "read"))
	{
		StaticADControl_MCP4651Read(atoi(argv[2]));
	}
	else if (IsEqual(argv[1], "write"))
	{
		StaticADControl_MCP4651Write(atoi(argv[2]), atoi(argv[3]));
	}
	else if (0 == argv[1] || IsEqual(argv[1], "help") ||
			IsEqual(argv[1], "?"))
   {
	   Printf("====== MCP4651 commands ======\n");
	   Printf(" mcp write [index][value] : meter index 2~3\n");
	   Printf(" mcp read  [index]  :  \n");
//       Printf(" mcp test [index][addr][value][time][period]:  \n");
//       Printf(" mcp random [index][addr]: return the random read value \n");
//       Printf(" mcp continuous [index][addr]: return 32 bit value \n");
   }
	else
	{
		Printf("Invalid param: %s\n", argv[1]);
	}
	return (0);
}

// 漏液检测
int Cmd_ChcekLeaking(int argc, char *argv[])
{
    if (IsEqual(argv[1], "start"))
    {
        if(argv[2])
        {
        	CheckLeaking_SetCheckLeakingReportPeriod(atof(argv[2]));
        	Printf("\nCheck leaking start");
        }
        else
        {
        	 Printf("Invalid param: %s\n", argv[2]);
        }
    }
    else if (IsEqual(argv[1], "stop"))
    {
    	CheckLeaking_SetCheckLeakingReportPeriod(0);
        Printf("\nCheck leaking stop");
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== Sub commands ======\n");
        Printf(" check start [period] : start checking task, period unit (s)\n");
        Printf(" check stop       : stop checking task\n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}

//*****************************************************************************
//
// 轮盘阀命令函数
//
//*****************************************************************************
int Cmd_SpinValve(int argc, char *argv[])
{
   if (IsEqual(argv[1], "open"))
    {
        if (argv[2] && atoi(argv[2]) > 0)
        {
            SpinValve_OpenValve(atoi(argv[2]), FALSE);
        }
        else
        {
            Printf("Invalid param %d", atoi(argv[2]));
        }
    }
    else if (IsEqual(argv[1], "reset"))
    {
        SpinValve_Reset(FALSE);
    }
    else if (IsEqual(argv[1], "stop"))
    {
        SpinValve_Stop(FALSE);
    }
    else if (IsEqual(argv[1], "get"))
    {
        SpinValve_GetCurrentValveIndex();
    }
    else if (IsEqual(argv[1], "status"))
    {
        SpinValve_GetStatus();
    }
    else if (IsEqual(argv[1], "list"))
    {
        SpinValve_CmdList();
    }
    else if (IsEqual(argv[1], "query"))
    {
        if (argv[2] && atoi(argv[2]) >= 0x20 && atoi(argv[2]) < 0x40)
        {
            SpinValve_CmdQueryTest(atoi(argv[2]));
        }
        else
        {
            Printf("Invalid param %d", atoi(argv[2]));
        }
    }
    else if (IsEqual(argv[1], "factory"))
    {
        if (argv[2] && atoi(argv[2]) >= 0x00 && atoi(argv[2]) <= 0x10)
        {
            if(argv[3])
            {
                SpinValve_CmdFactoryTest(atoi(argv[2]), atoi(argv[3]));
            }
            else
            {
                Printf("Invalid spinvalve factory cmd param %d", atoi(argv[3]));
            }
        }
        else
        {
            Printf("Invalid spinvalve factory cmd %d", atoi(argv[2]));
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== SpinValve commands ======\n");
        Printf(" spinvalve list : print all cmd list. \n");
        Printf(" spinvalve reset       : Spin valve reset. \n");
        Printf(" spinvalve open [INDEX]  : spinValve switch to [INDEX] channel. \n");
        Printf(" spinvalve stop       : Spin valve stop. \n");
        Printf(" spinvalve get  : spin valve get current index. \n");
        Printf(" spinvalve status   : spin valve get current status. \n");
        Printf(" spinvalve query [CMD]       : Spin valve query cmd test. \n");
        Printf(" spinvalve factory [CMD] [PARAM] : Spin valve factory cmd test. \n");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }
    return (0);
}

int Cmd_LiquidSensor(int argc, char *argv[])
{
    if (IsEqual(argv[1], "get"))
    {
    	if(argv[2])
    	{
        	Bool ret = SensorManager_ReadSensorStatus(atoi(argv[2]));
        	Printf("\n Sensor Current Status is %d",ret);
    	}
        else
        {
            Printf("Invalid spinvalve factory cmd param %d", atoi(argv[2]));
        }
    }
    else if (0 == argv[1] || IsEqual(argv[1], "help") ||
             IsEqual(argv[1], "?"))
    {
        Printf("====== sensor commands ======\n");
        Printf(" sensor get [index]: Get sensor current status");
    }
    else
    {
        Printf("Invalid param: %s\n", argv[1]);
    }

    return (0);
}
/** @} */
