/*
 * OpticalMeter.h
 *
 *  Created on: 2016年6月27日
 *      Author: Administrator
 */

#ifndef SRC_DNCPSTACK_OPTICALMETER_H_
#define SRC_DNCPSTACK_OPTICALMETER_H_

#include "Common/Types.h"
#include "DNCP/App/DscpDevice.h"
#include "LuipApi/OpticalMeterInterface.h"

#ifdef __cplusplus
extern "C"
{
#endif

//声明处理函数
void OpticalMeter_GetPumpFactor(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SetPumpFactor(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_GetMeterPoints(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SerMeterPoints(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_GetMeterStatus(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_StartMeter(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_StopMeter(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SetOpticalADNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_TurnOnLED(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_TurnOffLED(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_AutoCloseValve(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SetMeterSpeed(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_GetMeterSpeed(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SetMeterFinishValveMap(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_GetSingleOpticalAD(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SetRopinessMeterOverValue(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_GetRopinessMeterOverValue(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_SetAccurateModeOverValve(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalMeter_GetAccurateModeOverValve(DscpDevice* dscp, Byte* data, Uint16 len);

//命令入口，全局宏定义：每一条命令对应一个命令码和处理函数
#define CMD_TABLE_OPTICAL_METER \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_PUMP_FACTOR, OpticalMeter_GetPumpFactor), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_PUMP_FACTOR, OpticalMeter_SetPumpFactor ),\
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_METER_POINTS, OpticalMeter_GetMeterPoints),\
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_METER_POINTS, OpticalMeter_SerMeterPoints), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_METER_STATUS, OpticalMeter_GetMeterStatus), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_START_METER, OpticalMeter_StartMeter),\
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_STOP_METER, OpticalMeter_StopMeter),\
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_OPTICAL_AD_NOTIFY_PERIOD, OpticalMeter_SetOpticalADNotifyPeriod),\
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_TURN_ON_LED, OpticalMeter_TurnOnLED),\
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_TURN_OFF_LED, OpticalMeter_TurnOffLED), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_AUTO_CLOSE_VALVE, OpticalMeter_AutoCloseValve), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_METER_SPEED, OpticalMeter_SetMeterSpeed), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_METER_SPEED, OpticalMeter_GetMeterSpeed), \
	DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_METER_FINISH_VALVE_MAP, OpticalMeter_SetMeterFinishValveMap), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_SINGLE_OPTICAL_AD, OpticalMeter_GetSingleOpticalAD), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_ROPINESS_METER_OVER_VALUE, OpticalMeter_SetRopinessMeterOverValue), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_ROPINESS_METER_OVER_VALUE, OpticalMeter_GetRopinessMeterOverValue), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_SET_ACCURATE_MODE_STANDARD_CNT, OpticalMeter_SetAccurateModeOverValve), \
    DSCP_CMD_ENTRY(DSCP_CMD_OMI_GET_ACCURATE_MODE_STANDARD_CNT, OpticalMeter_GetAccurateModeOverValve)

#ifdef __cplusplus
}
#endif

#endif /* SRC_DNCPSTACK_OPTICALMETER_H_ */
