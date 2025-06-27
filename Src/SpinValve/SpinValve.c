/*
 * SpinValveManager.c
 *
 *  Created on: 2019年9月11日
 *      Author: Administrator
 */

#include "SpinValve.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Tracer/Trace.h"
#include "SystemConfig.h"
#include "Driver/LiquidDriver/SpinValveDriver.h"
#include "LuipApi/SolenoidValveInterface.h"

typedef enum
{
    /******工厂指令*******/
    CMD_SET_ADDR = 			0x00,			///< [0]设定地址
    CMD_SET_RS232_BAUD = 	0x01,			///< [1]设定RS232波特率
    CMD_SET_RS485_BAUD = 	0x02,			///< [2]设定RS485波特率
    CMD_SET_CAN_BAUD = 		0x03,			///< [3]设定CAN波特率
    CMD_SET_MAX_SPEED = 	0x07,			///< [7]设定最大转速(转/分)
    CMD_SET_MAX_NUM = 		0x0A,			///< [10]设定码盘一圈计数
    CMD_SET_RESET_SPEED = 	0x0B,			///< [11]设定复位速度
    CMD_SET_RESET_DIR = 	0x0C,			///< [12]设定复位方向
    CMD_SET_POWER_RESET = 	0x0E,			///< [14]设定上电自动复位
    CMD_SET_CAN_ADDR = 		0x10,			///< [16]设定CAN目的地址

    /******查询指令*******/
    CMD_GET_ADDR = 			0x20,			///< [32]查询地址
    CMD_GET_RS232_BAUD = 	0x21,			///< [33]查询RS232波特率
    CMD_GET_RS485_BAUD = 	0x22,			///< [34]查询RS485波特率
    CMD_GET_CAN_BAUD = 		0x23,			///< [35]查询CAN波特率
    CMD_GET_MAX_SPEED = 	0x27,			///< [39]查询最大转速(转/分)
    CMD_GET_MAX_NUM = 		0x2A,			///< [42]查询码盘一圈计数
    CMD_GET_RESET_SPEED = 	0x2B,			///< [43]查询复位速度
    CMD_GET_RESET_DIR = 	0x2C,			///< [44]查询复位方向
    CMD_GET_POWER_RESET = 	0x2E,			///< [46]查询上电自动复位
    CMD_GET_CAN_ADDR = 		0x30,			///< [48]查询CAN目的地址
    CMD_GET_CURRENT_INDEX = 0x3E,           ///< [62]查询当前通道位置
    CMD_GET_VERSION = 		0x3F,			///< [63]查询当前版本

    /******控制指令*******/
    CMD_CTL_VALVE_OPEN = 0x44,              ///< [68]码盘转动打开对应阀通道(智能走最短路径)
	CMD_CTL_VALVE_OPEN_DIR = 0xA4,			//按照需求方向切换阀
	CMD_CTL_VALVE_CLOSE = 0xB4, 		    //切换到阀号中间
    CMD_CTL_RESET = 0x45,                   ///< [69]复位
    CMD_CTL_STOP = 0x49,                    ///< [73]强停
    CMD_CTL_GET_STATUS = 0x4A,              ///< [74]查询电机状态
    CMD_CTL_CLEAR_DATA = 0xFF,              ///< [255]重置驱动器内部数据
}SpinValveCmd;

typedef enum
{
    SpinValveIdle,
    SpinValveBusy,
}SpinValveCtrlState;

typedef enum
{
    Control = 0,
    Query = 1,
    Factory = 2,
}SpinValveCmdType;



/** ----------------------- Task ----------------------------------------**/
static xTaskHandle s_spinValveCmdHandle;

static void SpinValveCmdHandle(void *argument);

/** ----------------------- Variables ----------------------------------------**/
static SpinValveCtrlState s_currentState = SpinValveIdle;
static Uint8 s_currentCmd = 0x4A;               //查询电机状态
static Uint8 s_currentParamData[2] = {0};
static Uint8 s_passwordData[4] = {0xFF, 0xEE, 0xBB, 0xAA};
static Uint8 s_factoryParamData[8] = {0};
static Bool s_waitEvent = FALSE;
static Uint32 s_waitTimeout = 10000;
static SpinValveResp s_currentResp;
static Uint8 s_currentRespData[2] = {0};
static SpinValveResult s_currentResult;
static Bool s_requestStop = FALSE;
static SpinValveCmdType s_currentCmdType = Control;
/**
 * @brief 初始化
 */
void SpinValve_Init(void)
{
    SpinValveDriver_RS232Init();

    xTaskCreate(SpinValveCmdHandle, "SpinValveCmd",
            SPINVALVE_CTRL_STK_SIZE, NULL,
            SPINVALVE_CTRL_TASK_PRIO, &s_spinValveCmdHandle);
}

/**
 * @brief 控制-轮盘阀复位
 */
Bool SpinValve_Reset(Bool waitEvent)
{
    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = CMD_CTL_RESET;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            s_requestStop = FALSE;
            s_waitEvent = waitEvent;
            s_currentResp = UnknowError;
            memset(s_currentRespData, 0, sizeof(s_currentRespData));
            s_currentState = SpinValveBusy;
            s_currentResult = SPINVALVE_RESULT_FAILED;
            s_currentCmdType = Control;
            vTaskResume(s_spinValveCmdHandle);
            TRACE_INFO("\nSpin valve reset success!");
            return TRUE;
        }
        else
        {
            TRACE_INFO("\nSpin valve reset fail!");
            return FALSE;
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
        return FALSE;
    }
}

/**
 * @brief 控制-轮盘阀立即停止
 */
Bool SpinValve_Stop(Bool waitEvent)
{
        s_currentCmd = CMD_CTL_STOP;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            s_requestStop = TRUE;
            s_waitEvent = waitEvent;
            s_currentResp = UnknowError;
            memset(s_currentRespData, 0, sizeof(s_currentRespData));
            s_currentState = SpinValveBusy;
            s_currentResult = SPINVALVE_RESULT_STOPPED;
            s_currentCmdType = Control;
            vTaskResume(s_spinValveCmdHandle);
            TRACE_INFO("\nSpin valve stop success!");
            return TRUE;
        }
        else
        {
            TRACE_INFO("\nSpin valve reset fail!");
            return FALSE;
        }
}

/**
 * @brief 控制-轮盘阀开阀
 */
Bool SpinValve_OpenValve(Uint8 index, Bool waitEvent)
{
    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = CMD_CTL_VALVE_OPEN;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        s_currentParamData[0] = index;
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            s_requestStop = FALSE;
            s_waitEvent = waitEvent;
            s_currentResp = UnknowError;
            memset(s_currentRespData, 0, sizeof(s_currentRespData));
            s_currentState = SpinValveBusy;
            s_currentResult = SPINVALVE_RESULT_FAILED;
            s_currentCmdType = Control;
            vTaskResume(s_spinValveCmdHandle);
            TRACE_INFO("\nSpin valve open channel %d success!", index);
            return TRUE;
        }
        else
        {
            TRACE_INFO("\nSpin valve open channel %d fail!", index);
            return FALSE;
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
        return FALSE;
    }
}

/**
 * @brief 控制-轮盘阀开阀
 */
Bool SpinValve_Control(Uint8 cmd, Uint8 param1, Uint8 param2, Bool waitEvent)
{
    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = cmd;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        s_currentParamData[0] = param1;
        s_currentParamData[1] = param2;
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            s_requestStop = FALSE;
            s_waitEvent = waitEvent;
            s_currentResp = UnknowError;
            memset(s_currentRespData, 0, sizeof(s_currentRespData));
            s_currentState = SpinValveBusy;
            s_currentResult = SPINVALVE_RESULT_FAILED;
            s_currentCmdType = Control;
            vTaskResume(s_spinValveCmdHandle);
            TRACE_INFO("\nSpin valve cmd:%d,%d,%d success!", cmd,param1,param2);
            return TRUE;
        }
        else
        {
            TRACE_INFO("\nSpin valve cmd:%d,%d,%d fail!", cmd,param1,param2);
            return FALSE;
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
        return FALSE;
    }
}

/**
 * @brief 查询-获取当前阀索引
 */
Uint8 SpinValve_GetCurrentValveIndex(void)
{
    Uint16 index  = 0x0;

    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = CMD_GET_CURRENT_INDEX;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, 50))
            {
                if(s_currentResp == Normal)
                {
                    memcpy(&index, s_currentRespData, sizeof(Uint16));
                    TRACE_INFO("\nSpin valve current index = %d", index);
                }
                else
                {
                    TRACE_INFO("\nSpin valve resp error!");
                }
            }
            else
            {
                TRACE_INFO("\nSpin valve wait resp timeout!");
            }
        }
        else
        {
            TRACE_INFO("\nSpin valve send cmd fail!");
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
    }

    return (Uint8)index;
}

/**
 * @brief 查询-当前轮盘阀状态
 */
Uint8 SpinValve_GetStatus(void)
{
    Uint8 status  = 0xFF;

    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = CMD_CTL_GET_STATUS;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, 50))
            {
                status = s_currentResp;
                TRACE_INFO("\nSpin valve current status = %d", status);
            }
            else
            {
                TRACE_INFO("\nSpin valve wait resp timeout!");
            }
        }
        else
        {
            TRACE_INFO("\nSpin valve send cmd fail!");
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
    }

    return (Uint8)status;
}

/**
 * @brief 查询-最大转速
 */
Uint16 SpinValve_GetMaxSpeed(void)
{
    Uint16 value  = 0;

    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = CMD_GET_MAX_SPEED;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, 50))
            {
                memcpy(&value, s_currentRespData, sizeof(Uint16));
                TRACE_INFO("\nSpin valve current value = %d", value);
            }
            else
            {
                TRACE_INFO("\nSpin valve wait resp timeout!");
            }
        }
        else
        {
            TRACE_INFO("\nSpin valve send cmd fail!");
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
    }

    return value;
}

/**
 * @brief 工厂命令-设置最大转速
 */
Bool SpinValve_SetMaxSpeed(Uint16 value)
{
    Bool ret = FALSE;
    Uint32 param = (Uint32)value;
    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = CMD_SET_MAX_SPEED;
        memcpy(&s_factoryParamData[0], 0, sizeof(s_passwordData));
        memcpy(&s_factoryParamData[4], &param, 4*sizeof(Uint8));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_factoryParamData, sizeof(s_factoryParamData)))
        {
            if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, 100))
            {
                if(s_currentResp == Normal)
                {
                    ret = TRUE;
                    TRACE_INFO("\nSpin Valve Set Max Speed Succeed");
                }
                else
                {
                    TRACE_INFO("\nSpin Valve Set Max Speed Faile. Resp = %d", s_currentResp);
                }
            }
            else
            {
                TRACE_INFO("\nSpin valve wait resp timeout!");
            }
        }
        else
        {
            TRACE_INFO("\nSpin valve send cmd fail!");
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
    }

    return ret;
}

/**
 * @brief 轮盘阀命令处理任务
 */
void SpinValveCmdHandle(void *argument)
{
    vTaskSuspend(NULL);
    while (1)
    {
        switch(s_currentState)
        {
            case SpinValveIdle:
                vTaskSuspend(s_spinValveCmdHandle);
                break;

            case SpinValveBusy:
                if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, s_waitTimeout))
                {
                    switch(s_currentResp)
                    {
                        case Normal:
                            if(s_requestStop == TRUE)
                            {
                                s_currentResult = SPINVALVE_RESULT_STOPPED;
                            }
                            else
                            {
                                s_currentResult = SPINVALVE_RESULT_SUCCEED;
                            }
                            break;
                        case FrameError:
                        case ParamError:
                        case SensorError:
                        case MotorBusy:
                        case MotorLocked:
                        case UnknownPoint:
                        case CmdRefuse:
                        case TaskSuspend:
                        case UnknowError:
                            s_currentResult = SPINVALVE_RESULT_FAILED;
                            break;
                        default:
                            s_currentResult = SPINVALVE_RESULT_FAILED;
                            break;
                    }
                }
                else
                {
                    s_currentResult = SPINVALVE_RESULT_TIMEOUT;
                }
                TRACE_INFO("\n SpinValveCmd resp = %d", s_currentResp);

                if(s_waitEvent == TRUE)
                {
                    Uint8 event = 0;
                    memcpy(&event, &s_currentResult, sizeof(Uint8));
                    //DncpStack_SendEvent(DSCP_EVENT_SVI_SPINVALVE_RESULT, s_currentResult, sizeof(s_currentResult));
                    //DncpStack_BufferEvent(DSCP_EVENT_SVI_SPINVALVE_RESULT,s_currentResult, sizeof(s_currentResult));
                }
                s_requestStop = FALSE;
                s_currentState = SpinValveIdle;
                break;

            default:
                vTaskSuspend(s_spinValveCmdHandle);
                break;
        }
        vTaskDelay(5);
    }
}

/**
 * @brief 查询轮盘阀最大数目
 */
Uint16 SpinValve_GetTotalValves(void)
{
    return MAX_SPIN_VALVE_NUM;
}

/**
 * @brief 查询命令测试
 */
void SpinValve_CmdQueryTest(Uint8 cmd)
{
    Uint16 data = 0;
    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = cmd;
        memset(s_currentParamData, 0, sizeof(s_currentParamData));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_currentParamData, sizeof(s_currentParamData)))
        {
            if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, 100))
            {
                memcpy(&data, s_currentRespData, sizeof(data));
                TRACE_INFO("Spin Valve Query Cmd Resp = %d, Data = %d",s_currentResp, data);
            }
            else
            {
                TRACE_INFO("\nSpin valve wait resp timeout!");
            }
        }
        else
        {
            TRACE_INFO("\nSpin valve send cmd fail!");
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
    }
}

/**
 * @brief 工厂命令测试
 */
void SpinValve_CmdFactoryTest(Uint8 cmd, Uint32 param)
{
    Uint16 data = 0;
    if(s_currentState == SpinValveIdle)
    {
        s_currentCmd = cmd;
        memcpy(&s_factoryParamData[0], 0, sizeof(s_passwordData));
        memcpy(&s_factoryParamData[4], &param, 4*sizeof(Uint8));
        if(SpinValveDriver_SendCmd(s_currentCmd, s_factoryParamData, sizeof(s_factoryParamData)))
        {
            if(SpinValveDriver_WaitResp(&s_currentResp, s_currentRespData, 100))
            {
                memcpy(&data, s_currentRespData, sizeof(data));
                TRACE_INFO("Spin Valve Factory Cmd Resp = %d, Data = %d",s_currentResp, data);
            }
            else
            {
                TRACE_INFO("\nSpin valve wait resp timeout!");
            }
        }
        else
        {
            TRACE_INFO("\nSpin valve send cmd fail!");
        }
    }
    else
    {
        TRACE_INFO("\nSpin valve handle is busy.");
    }
}

/**
 * @brief 打印轮盘阀命令
 */
void SpinValve_CmdList(void)
{
    Printf("\n----------------SpinValve Cmd Table-----------------\n");
    Printf("Control Command:\n");
    Printf("CMD_CTL_VALVE_OPEN = 0x44\n");
    Printf("CMD_CTL_RESET = 0x45\n");
    Printf("CMD_CTL_STOP = 0x49\n");
    Printf("CMD_CTL_GET_STATUS = 0x4A\n");
    Printf("CMD_CTL_CLEAR_DATA = 0xFF\n");
    System_Delay(10);
    Printf("Query Command:\n");
    Printf("CMD_GET_ADDR = 0x20\n");
    Printf("CMD_GET_RS232_BAUD = 0x21\n");
    Printf("CMD_GET_RS485_BAUD = 0x22\n");
    Printf("CMD_GET_CAN_BAUD = 0x23\n");
    Printf("CMD_GET_MAX_SPEED = 0x27\n");
    Printf("CMD_GET_MAX_NUM = 0x2A\n");
    Printf("CMD_GET_RESET_SPEED = 0x2B\n");
    Printf("CMD_GET_RESET_DIR = 0x2C\n");
    Printf("CMD_GET_POWER_RESET = 0x2E\n");
    Printf("CMD_GET_CAN_ADDR = 0x30\n");
    Printf("CMD_GET_CURRENT_INDEX = 0x3E\n");
    Printf("CMD_GET_VERSION = 0x3F\n");
    System_Delay(10);
    Printf("Factory Command:\n");
    Printf("CMD_SET_ADDR = 0x00\n");
    Printf("CMD_SET_RS232_BAUD = 0x01\n");
    Printf("CMD_SET_RS485_BAUD = 0x02\n");
    Printf("CMD_SET_CAN_BAUD = 0x03\n");
    Printf("CMD_SET_MAX_SPEED = 0x07\n");
    Printf("CMD_SET_MAX_NUM = 0x0A\n");
    Printf("CMD_SET_RESET_SPEED = 0x0B\n");
    Printf("CMD_SET_RESET_DIR = 0x0C\n");
    Printf("CMD_SET_POWER_RESET = 0x0E\n");
    Printf("CMD_SET_CAN_ADDR = 0x10\n");
    System_Delay(5);
    Printf("\n");
}

