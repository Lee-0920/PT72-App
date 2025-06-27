/**
 * @file SolenoidValve.h
 * @brief 电磁阀控制执行头文件。
 * @version 1.0.0
 * @author xingfan
 * @date 2016-05-27
 */
#ifndef SRC_DNCPSTACK_SOLENOIDVALVE_H_
#define SRC_DNCPSTACK_SOLENOIDVALVE_H_
#include "Common/Types.h"
#include "DNCP/App/DscpDevice.h"
#include "LuipApi/SolenoidValveInterface.h"

//声明处理函数
void SolenoidVale_GetTotalValves(DscpDevice* dscp, Byte* data, Uint16 len);
void SolenoidVale_GetValvesMap(DscpDevice* dscp, Byte* data, Uint16 len);
void SolenoidVale_SetValvesMap(DscpDevice* dscp, Byte* data, Uint16 len);
void SolenoidVale_CtrlSpinValve(DscpDevice* dscp, Byte* data, Uint16 len);
void SolenoidVale_SetValve16(DscpDevice* dscp, Byte* data, Uint16 len);

//命令入口，全局宏定义：每一条命令对应一个命令码和处理函数
#define CMD_TABLE_SOLENOID_VALVE \
    DSCP_CMD_ENTRY(DSCP_CMD_SVI_GET_TOTAL_VALVES, SolenoidVale_GetTotalValves), \
    DSCP_CMD_ENTRY(DSCP_CMD_SVI_GET_VALVE_MAP, SolenoidVale_GetValvesMap), \
    DSCP_CMD_ENTRY(DSCP_CMD_SVI_SET_VALVE_MAP, SolenoidVale_SetValvesMap),\
	DSCP_CMD_ENTRY(DSCP_CMD_SVI_CTL_SPINVALVE, SolenoidVale_CtrlSpinValve),\
	DSCP_CMD_ENTRY(DSCP_CMD_SVI_SET_VALVE16, SolenoidVale_SetValve16)

#endif /* SRC_DNCPSTACK_SOLENOIDVALVE_H_ */
