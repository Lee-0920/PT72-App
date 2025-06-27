/*
 * SystemConfig.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_SYSTEMCONFIG_H_
#define SRC_SYSTEMCONFIG_H_
#include "PeristalticPump/PumpManager.h"
#include "OpticalMeter/Meter.h"

//外设中断抢占式优先级配置=============================================================================
#define    WATCHDOG_IRQ_PRIORITY                    3
#define    LAIRS485ADAPTER_IRQ_PRIORITY             7
#define    CONSOLEUART_IRQ_PRIORITY                 7
#define    PUMP_TIMERA_PRIORITY                     9
#define    PUMP_TIMERB_PRIORITY                     9
#define    PUMP_TIMERC_PRIORITY                     9
#define    PUMP_TIMERD_PRIORITY                     9
#define    PWM_TIMER_PRIORITY                     9
#define    RS232DRIVER_IRQ_PRIORITY             	8
#define    METER_ADCDMA_PRIORITY                    8
#define    TEMP_ADCDMA_PRIORITY                    8
#define    SYSTEM_STATE_TIMER_PRIORITY              5
#define    TMC_UART_IRQ_PRIORITY                    6
#define    OPT_XONEN_TIMER_IQR_PRIORITY               4
//freeRTOS 任务优先级和堆栈大小配置===================================================================
//基本任务
#define CONSOLESCHEDULER_TASK_PRIO                  5   //控制台命令处理任务
#define CONSOLESCHEDULER_STK_SIZE                   256

#define LAIRS485COMMITTOUPPER_TASK_PRIO             6   //LAI485接收任务
#define LAIRS485COMMITTOUPPER_STK_SIZE              256

#define LAIRS485SENDREQUEST_TASK_PRIO               6   //LAI485发送任务
#define LAIRS485SENDREQUEST_STK_SIZE                256

#define LAIRS485MONITORHOST_TASK_PRIO               6   //LAI485监控任务
#define LAIRS485MONITORHOST_STK_SIZE                128

#define DNCPSTACKDSCPCMD_TASK_PRIO                  6   //DNCP命令处理任务
#define DNCPSTACKDSCPCMD_STK_SIZE                   256

#define DEVICEINDICATOR_LED_TASK_PRIO               4   //运行指示灯任务
#define DEVICEINDICATOR_LED_STK_SIZE                128

#define UPDATER_ERASE_TASK_PRIO                     4   //升级擦除任务
#define UPDATER_ERASE_STK_SIZE                      128

//控制台循环输出信息任务
#define CMDCLINE_INFOOUTPUT_TASK_PRIO               4   //命令台常规信息输出
#define CMDCLINE_INFOOUTPUT_STK_SIZE                256

//液路功能任务
#define METER_TASK_PRIO                             6   //定量过程控制任务
#define METER_STK_SIZE                              256

#define METER_CHECK_TIMER_PRIO                      1   //定量点检测软定时器

#define METERAD_NOTIFY_PERIOD_TIMER_PRIO            4   //定量点AD值上报软定时器

// 接受电机动作完成事件
#define PUMP_EVENT_TASK_PRIO                        6  //接受电机事件任务
#define PUMP_EVENT_STK_SIZE                         128

//信号采集功能
#define MEASURE_LED_PID_HANDLE_TASK_PRIO            6   //测量LED PID处理任务
#define MEASURE_LED_PID_HANDLE_STK_SIZE             128

#define OPTICALCONTROL_ADHANDLE_TASK_PRIO           6   //测量信号采集滤波任务
#define OPTICALCONTROL_ADHANDLE_STK_SIZE            256

#define OPTICALCONTROL_ADCOLLECT_TASK_PRIO          6   //测量信号AD采集任务
#define OPTICALCONTROL_ADCOLLECT_STK_SIZE           128

#define OPTICAL_AD_NOTIFY_PERIOD_TIMER_REIO         5   //测量信号上报软定时

#define STATIC_AD_CONTROL_TASK_PRIO                      4   //静态AD调节控制任务
#define STATIC_AD_CONTROL_STK_SIZE                       128

#define STATIC_AD_MCPINIT_TASK_PRIO                      6   //静态AD调节控制任务
#define STATIC_AD_MCPINIT_STK_SIZE                       128

//温控功能
#define TEMP_MONITOR_TASK_PRIO                      4   //温度监控任务
#define TEMP_MONITOR_STK_SIZE                       256

#define THERMOSTAT_TASK_PRIO                        5   //恒温器任务
#define THERMOSTAT_STK_SIZE                         328

#define TEMP_REPORT_TASK_PRIO                       3   //温度上报任务
#define TEMP_REPORT_STK_SIZE                        256

#define THERMOSTAT_TIMER_REIO                       2   //恒温器软定时器

//漏液检测功能
#define LEAK_MONITOR_TASK_PRIO                      6   //漏液监控任务
#define LEAK_MONITOR_STK_SIZE                       256

//轮盘阀控制任务
#define SPINVALVE_CTRL_TASK_PRIO                     5
#define SPINVALVE_CTRL_STK_SIZE                      128

//用户FLASH地址和大小配置==============================================================================
#define UPDATE_FLASH_START                          0x08000000//UPDATE程序空间48K
#define UPDATE_FLASH_END                            0x0800BFFF
#define UPDATE_DATA_FLASH_START                     0x0800C000//UPDATE数据空间16K
#define UPDATE_DATA_FLASH_END                       0x0800FFFF
#define APP_DATA_FLASH_START                        0x08010000//APP数据空间64K
#define APP_DATA_FLASH_END                          0x0801FFFF
#define APP_FLASH_START                             0x08020000 
#define APP_FLASH_END                               0x081FFFFF


#define FLASH_FACTORY_SIGN                          0xAA55AA55
#define FLASH_USE_BASE                              APP_DATA_FLASH_START
//板卡信息：共96byte
#define DEVICE_INFO_SIGN_FLASH_BASE_ADDR            FLASH_USE_BASE
#define DEVICE_INFO_SIGN_FLASH_LEN                  4

#define DEVICE_INFO_TYPE_ADDRESS                    (DEVICE_INFO_SIGN_FLASH_BASE_ADDR + DEVICE_INFO_SIGN_FLASH_LEN)
#define DEVICE_INFO_TYPE_LEN                        16
#define DEVICE_INFO_MODEL_ADDRESS                   (DEVICE_INFO_TYPE_ADDRESS + DEVICE_INFO_TYPE_LEN)
#define DEVICE_INFO_MODEL_LEN                       24
#define DEVICE_INFO_SN_ADDRESS                      (DEVICE_INFO_MODEL_ADDRESS + DEVICE_INFO_MODEL_LEN)
#define DEVICE_INFO_SN_LEN                          32
#define DEVICE_INFO_MANUF_ADDRESS                   (DEVICE_INFO_SN_ADDRESS +  DEVICE_INFO_SN_LEN)
#define DEVICE_INFO_MANUF_LEN                       20
#define DEVICE_INFO_DATE_ADDRESS                    (DEVICE_INFO_MANUF_ADDRESS + DEVICE_INFO_MANUF_LEN)
#define DEVICE_INFO_DATE_LEN                        4

//泵参数：共28yte   //原先预留了2个泵的参数位置
#define PUMP_SIGN_FLASH_BASE_ADDR                   (DEVICE_INFO_DATE_ADDRESS + DEVICE_INFO_DATE_LEN)
#define PUMP_SIGN_FLASH_LEN                         4
#define PUMP_SIGN_FLASH_LEN_TOTAL           8

#define PUMP_MOTIONPARAM_FLASH_BASE_ADDR            (PUMP_SIGN_FLASH_BASE_ADDR + PUMP_SIGN_FLASH_LEN_TOTAL)
#define PUMP_MOTIONPARAM_FLASH_LEN                 8
#define PUMP_MOTIONPARAM_FLASH_LEN_TOTAL     16

#define PUMP_FACTOR_FLASH_BASE_ADDR                 (PUMP_MOTIONPARAM_FLASH_BASE_ADDR + PUMP_MOTIONPARAM_FLASH_LEN_TOTAL)
#define PUMP_FACTOR_FLASH_LEN                       4
#define PUMP_FACTOR_FLASH_LEN_TOTAL          8

//定量点体系参数：共21byte
#define METER_FACTORY_SIGN_FLASH_BASE_ADDR          (PUMP_FACTOR_FLASH_BASE_ADDR + PUMP_FACTOR_FLASH_LEN_TOTAL)
#define METER_FACTORY_SIGN_FLASH_LEN                4

#define METER_POINTS_FLASH_BASE_ADDR                (METER_FACTORY_SIGN_FLASH_BASE_ADDR + METER_FACTORY_SIGN_FLASH_LEN)
#define METER_POINTS_FLASH_LEN                      sizeof(MeterPoint)

//测量LED PID参数：4+4+12
#define LEDCONTROLLER_SIGN_FLASH_BASE_ADDR          (METER_POINTS_FLASH_BASE_ADDR + METER_POINTS_FLASH_LEN)
#define LEDCONTROLLER_SIGN_FLASH_LEN                4
#define LEDCONTROLLER_TARGET_ADDRESS                (LEDCONTROLLER_SIGN_FLASH_BASE_ADDR + LEDCONTROLLER_SIGN_FLASH_LEN)
#define LEDCONTROLLER_TARGET_FLASH_LEN              4
#define LEDCONTROLLER_PARAM_ADDRESS                 (LEDCONTROLLER_TARGET_ADDRESS + LEDCONTROLLER_TARGET_FLASH_LEN)
#define LEDCONTROLLER_PARAM_FLASH_LEN               12

//温控功能参数：2个温度校准参数 + 1个恒温器PID参数
#define TEMPERATURE_BASE                                 (LEDCONTROLLER_PARAM_ADDRESS + LEDCONTROLLER_PARAM_FLASH_LEN)
#define TEMPERATURE_CALIBRATE_FACTOR_ADDRESS             (TEMPERATURE_BASE + 0)
#define TEMPERATURE_CALIBRATE_FACTOR_LEN                 12
#define TEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL     24  //实际只用了前12个字节
#define THERMOSTAT_PARAM_ADDRESS                         (TEMPERATURE_CALIBRATE_FACTOR_ADDRESS + TEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL)
#define THERMOSTAT_PARAM_LEN                             12
#define TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR                (THERMOSTAT_PARAM_ADDRESS + THERMOSTAT_PARAM_LEN)
#define TEMPERATURE_FACTORY_SIGN_FLASH_LEN                      4
#define THERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR                 (TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN)
#define THERMOSTAT_FACTORY_SIGN_FLASH_LEN                       4

//MCP4651  默认设置标志与参数：4+2*4
#define MCP4651_CONTROL_SIGN_FLASH_BASE_ADDR                 (THERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + THERMOSTAT_FACTORY_SIGN_FLASH_LEN)
#define MCP4651_CONTROL_SIGN_FLASH_LEN                       4
#define MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR                 (MCP4651_CONTROL_SIGN_FLASH_BASE_ADDR + MCP4651_CONTROL_SIGN_FLASH_LEN)
#define MCP4651_CONTROL_PARAM_FLASH_LEN                       2
#define MCP4651_CONTROL_PARAM_FLASH_LEN_TOTAL          8

//测量LED DAC默认输出参数：4+4
#define LEDDAC_SIGN_FLASH_BASE_ADDR                              (MCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + MCP4651_CONTROL_PARAM_FLASH_LEN_TOTAL)
#define LEDDAC_SIGN_FLASH_LEN                4
#define LEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS                (LEDDAC_SIGN_FLASH_BASE_ADDR + LEDDAC_SIGN_FLASH_LEN)
#define LEDDAC_DEFAULT_VALUE_FLASH_LEN              4

//扩展测量LED DAC默认输出参数：4+4
#define EXTLEDDAC_SIGN_FLASH_BASE_ADDR                              (LEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS + LEDDAC_DEFAULT_VALUE_FLASH_LEN)
#define EXTLEDDAC_SIGN_FLASH_LEN                4
#define EXTLEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS                (EXTLEDDAC_SIGN_FLASH_BASE_ADDR + EXTLEDDAC_SIGN_FLASH_LEN)
#define EXTLEDDAC_DEFAULT_VALUE_FLASH_LEN              4

//扩展泵参数：额外预留2个泵的参数位置
#define EXTPUMP_SIGN_FLASH_BASE_ADDR                   (EXTLEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS + EXTLEDDAC_DEFAULT_VALUE_FLASH_LEN)
#define EXTPUMP_SIGN_FLASH_LEN                         PUMP_SIGN_FLASH_LEN
#define EXTPUMP_SIGN_FLASH_LEN_TOTAL           8

#define EXTPUMP_MOTIONPARAM_FLASH_BASE_ADDR            (EXTPUMP_SIGN_FLASH_BASE_ADDR + EXTPUMP_SIGN_FLASH_LEN_TOTAL)
#define EXTPUMP_MOTIONPARAM_FLASH_LEN                 PUMP_MOTIONPARAM_FLASH_LEN
#define EXTPUMP_MOTIONPARAM_FLASH_LEN_TOTAL     16

#define EXTPUMP_FACTOR_FLASH_BASE_ADDR                 (EXTPUMP_MOTIONPARAM_FLASH_BASE_ADDR + EXTPUMP_MOTIONPARAM_FLASH_LEN_TOTAL)
#define EXTPUMP_FACTOR_FLASH_LEN                       PUMP_FACTOR_FLASH_LEN
#define EXTPUMP_FACTOR_FLASH_LEN_TOTAL          8

//扩展测量LED PID参数：4+4+12
#define EXTLEDCONTROLLER_SIGN_FLASH_BASE_ADDR          (EXTPUMP_FACTOR_FLASH_BASE_ADDR + EXTPUMP_FACTOR_FLASH_LEN_TOTAL)
#define EXTLEDCONTROLLER_SIGN_FLASH_LEN                4
#define EXTLEDCONTROLLER_TARGET_ADDRESS                (EXTLEDCONTROLLER_SIGN_FLASH_BASE_ADDR + EXTLEDCONTROLLER_SIGN_FLASH_LEN)
#define EXTLEDCONTROLLER_TARGET_FLASH_LEN              4
#define EXTLEDCONTROLLER_PARAM_ADDRESS                 (EXTLEDCONTROLLER_TARGET_ADDRESS + EXTLEDCONTROLLER_TARGET_FLASH_LEN)
#define EXTLEDCONTROLLER_PARAM_FLASH_LEN               12

//扩展的温控功能参数：预留2个温度计
#define EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR                (EXTLEDCONTROLLER_PARAM_ADDRESS + EXTLEDCONTROLLER_PARAM_FLASH_LEN)
#define EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN                      4
#define EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN_TOTAL         8
#define EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS             (EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + EXTTEMPERATURE_FACTORY_SIGN_FLASH_LEN_TOTAL)
#define EXTTEMPERATURE_CALIBRATE_FACTOR_LEN                 12
#define EXTTEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL         24

//扩展的温控功能参数：预留2个恒温器参数
#define EXTTHERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR                 (EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS + EXTTEMPERATURE_CALIBRATE_FACTOR_LEN_TOTAL)
#define EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN                       4
#define EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN_TOTAL          8
#define EXTTHERMOSTAT_PARAM_ADDRESS                         (EXTTHERMOSTAT_FACTORY_SIGN_FLASH_BASE_ADDR + EXTTHERMOSTAT_FACTORY_SIGN_FLASH_LEN_TOTAL)
#define EXTTHERMOSTAT_PARAM_LEN                             12
#define EXTTHERMOSTAT_PARAM_LEN_TOTAL                24

//扩展MCP4651  默认设置标志与参数：4+2*4
#define EXTMCP4651_CONTROL_SIGN_FLASH_BASE_ADDR                 (EXTTHERMOSTAT_PARAM_ADDRESS + EXTTHERMOSTAT_PARAM_LEN_TOTAL)
#define EXTMCP4651_CONTROL_SIGN_FLASH_LEN                       4
#define EXTMCP4651_CONTROL_PARAM_FLASH_BASE_ADDR                 (EXTMCP4651_CONTROL_SIGN_FLASH_BASE_ADDR + EXTMCP4651_CONTROL_SIGN_FLASH_LEN)
#define EXTMCP4651_CONTROL_PARAM_FLASH_LEN                       2
#define EXTMCP4651_CONTROL_PARAM_FLASH_LEN_TOTAL          8

#define METER_OVER_VALVE_FLASH_BASE_ADDR                 (EXTMCP4651_CONTROL_PARAM_FLASH_BASE_ADDR + EXTMCP4651_CONTROL_PARAM_FLASH_LEN_TOTAL)
#define METER_OVER_VALVE_FLASH_LEN                       4
#define METER_OVER_VALVE_PARAM_FLASH_BASE_ADDR           (METER_OVER_VALVE_FLASH_BASE_ADDR + METER_OVER_VALVE_FLASH_LEN)
#define METER_OVER_VALVE_PARAM_FLASH_LEN                 4

#define METERAD_THRESHOLD_VALUE_FLASH_BASS_ADDR				 (METER_OVER_VALVE_PARAM_FLASH_BASE_ADDR + METER_OVER_VALVE_PARAM_FLASH_LEN)
#define METERAD_THRESHOLD_VALUE_FLASH_LEN					 4
#define METERAD_THRESHOLD_VALUE_PARAM_FLASH_BASS_ADDR       (METERAD_THRESHOLD_VALUE_FLASH_BASS_ADDR + METERAD_THRESHOLD_VALUE_FLASH_LEN)
#define METERAD_THRESHOLD_VALUE_PARAM_FLASH_LEN				4

//所有使用的FLASH 209
#define FLASH_USE_SIZE                              ((u16)400)

#endif /* SRC_SYSTEMCONFIG_H_ */
