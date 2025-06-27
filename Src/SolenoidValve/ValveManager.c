/*
 * ValveManager.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */
#include <LiquidDriver/ValveDriver.h>
#include <LiquidDriver/ValveMap.h>
#include "Tracer/Trace.h"
#include "string.h"
#include "ValveManager.h"
#include "SpinValve/SpinValve.h"

//系统中使用的继电器总数目

static Valve s_valves[SOLENOIDVALVECONF_TOTALVAlVES - ROTARY_TOTALVALVES];
typedef struct
{
	Uint32 Mark;
	Uint8 index;
    bool (*Control)(Uint8 index, bool isOpen);
}ValveControl;
bool valve_control(Uint8 index, bool isOpen);
bool SpinValve_control(Uint8 index, bool isOpen);

const ValveControl s_ValveControl[SOLENOIDVALVECONF_TOTALVAlVES-1] = {
		{0x00001, 1,  SpinValve_control},  	//1:旋切阀
		{0x00002, 2,  SpinValve_control},  	//2:旋切阀
		{0x00004, 3,  SpinValve_control},	//3:旋切阀
		{0x00008, 4,  SpinValve_control},	//4:旋切阀
		{0x00010, 5,  SpinValve_control},	//5:旋切阀
		{0x00020, 6,  SpinValve_control}, 	//6:旋切阀
		{0x00040, 7,  SpinValve_control},	//7:旋切阀
		{0x00080, 8,  SpinValve_control},	//8:旋切阀
		{0x00100, 9,  SpinValve_control},	//9:旋切阀
		{0x00200, 10, SpinValve_control},	//10:旋切阀
		{0x00400, 11, SpinValve_control},	//11:旋切阀(12)
		{0x00800, 12, SpinValve_control},	//12:旋切阀(12)
		{0x01000, 0,  valve_control},	    //13:上高压阀
		{0x02000, 1,  valve_control},	    //14:下高压阀
		{0x04000, 2,  valve_control},	    //15:预留接口
	    //{0x08000, 3,  valve_control},	    //16:预留接口 //三通阀
};
static Uint32 ValveMap = 0x00;

bool valve_control(Uint8 index, bool isOpen)
{
	if(isOpen)
	{
		ValveDriver_Control(&s_valves[index], VAlVE_OPEN);
	}
	else
	{
		ValveDriver_Control(&s_valves[index], VAlVE_CLOSE);
	}
	return TRUE;
}
bool SpinValve_control(Uint8 index, bool isOpen)
{
	if(isOpen)
	{
		return SpinValve_OpenValve(index, FALSE);
	}
	return TRUE;
}

void ValveManager_Init(void)
{
    memset(s_valves, 0, sizeof(Valve) * (SOLENOIDVALVECONF_TOTALVAlVES - ROTARY_TOTALVALVES));

    ValveMap_Init(s_valves);
}

Uint16 ValveManager_GetTotalValves(void)
{
    return (SOLENOIDVALVECONF_TOTALVAlVES-1);
}

Bool ValveManager_SetValvesMap(Uint32 map)
{
    Uint8 i;
    if (map <= SOLENOIDVALVE_MAX_MAP)
    {
    	ValveMap = map;
        //TRACE_INFO("\nSetMap: 0x%x", map);
        for (i = 0; i < SOLENOIDVALVECONF_TOTALVAlVES-1; i++)
        {
        	if(s_ValveControl[i].Mark & map)
        	{
        		s_ValveControl[i].Control(s_ValveControl[i].index, TRUE);
        	}
        	else
        	{
        		s_ValveControl[i].Control(s_ValveControl[i].index, FALSE);
        	}
        }
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n The map must be 0 to 0x%x. Map:0x%x", SOLENOIDVALVE_MAX_MAP, map);
        return FALSE;
    }
}

Bool ValveManager_CtrlSpinValve(Uint8 cmd, Uint8 param1, Uint8 Param2)
{
	return SpinValve_Control(cmd, param1, Param2, FALSE);
}

Uint32 ValveManager_GetValvesMap(void)
{
    return ValveMap;
}

