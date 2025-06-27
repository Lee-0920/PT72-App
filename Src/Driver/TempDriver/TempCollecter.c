/**
 * @file
 * @brief 温度采集器。
 * @details 提供温度采集、上报功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2012-10-30
 */

#include "Driver/System.h"
#include "Driver/McuFlash.h"
#include "Tracer/Trace.h"
#include <string.h>
#include "TempADCollect.h"
#include "SystemConfig.h"
#include "PT1000.h"
#include "TempCollecter.h"

static float TempCollecter_CalcPt1000Temperature(TempCollecter *tempCollecter, uint16_t ad);

/**
 * @brief 温度采集器初始化
 * @param
 */
void TempCollecter_Init(TempCollecter *tempCollecter, TempCalibrateParam kTempCalculateParam)
{
    Uint8 buffer[TEMPERATURE_FACTORY_SIGN_FLASH_LEN] = { 0 };
    Uint32 flashFactorySign = 0;

    if(tempCollecter->number > 0)  //扩展的温度计
    {
        McuFlash_Read(EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * (tempCollecter->number - 1),
        TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);               //读取出厂标志位
    }
    else
    {
        McuFlash_Read(TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * tempCollecter->number,
        TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);               //读取出厂标志位
    }
    memcpy(&flashFactorySign, buffer, TEMPERATURE_FACTORY_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign)               //表示已经过出厂设置
    {
        tempCollecter->tempCalbrateFactor = TempCollecter_GetCalibrateFactor(tempCollecter);
    }
    else               //未设置,使用默认值，并写入出厂标志
    {
        TempCollecter_SetCalibrateFactor(tempCollecter, kTempCalculateParam);

        flashFactorySign = FLASH_FACTORY_SIGN;
        memcpy(buffer, &flashFactorySign, TEMPERATURE_FACTORY_SIGN_FLASH_LEN);
        if(tempCollecter->number > 0)  //扩展的温度计
        {
            McuFlash_Write(
            EXTTEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * (tempCollecter->number-1),
            TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);
        }
        else
        {
            McuFlash_Write(
            TEMPERATURE_FACTORY_SIGN_FLASH_BASE_ADDR + TEMPERATURE_FACTORY_SIGN_FLASH_LEN * tempCollecter->number,
            TEMPERATURE_FACTORY_SIGN_FLASH_LEN, buffer);
        }
    }
}

void TempCollecter_SetNumber(TempCollecter *tempCollecter, Uint8 number)
{
    tempCollecter->number = number;
}
/**
 * @brief 读温度传感器的校准系数
 * @param
 */
TempCalibrateParam TempCollecter_GetCalibrateFactor(TempCollecter *tempCollecter)
{
    Uint8 readData[TEMPERATURE_CALIBRATE_FACTOR_LEN] = { 0 };
    TempCalibrateParam calibratefactor;

    if(tempCollecter->number > 0)  //扩展的温度计
    {
        McuFlash_Read(EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS +TEMPERATURE_CALIBRATE_FACTOR_LEN * (tempCollecter->number-1),
                TEMPERATURE_CALIBRATE_FACTOR_LEN, readData);
    }
    else
    {
        McuFlash_Read(TEMPERATURE_CALIBRATE_FACTOR_ADDRESS +TEMPERATURE_CALIBRATE_FACTOR_LEN * tempCollecter->number,
                TEMPERATURE_CALIBRATE_FACTOR_LEN, readData);
    }
    memcpy(&calibratefactor, readData, sizeof(TempCalibrateParam));
    return calibratefactor;
}

/**
 * @brief 写温度传感器的校准系数
 * @param data
 */
void TempCollecter_SetCalibrateFactor(TempCollecter *tempCollecter, TempCalibrateParam calibratefactor)
{
    Uint8 writeData[TEMPERATURE_CALIBRATE_FACTOR_LEN] = { 0 };

    memcpy(writeData, &calibratefactor, sizeof(TempCalibrateParam));

    if(tempCollecter->number > 0)  //扩展的温度计
    {
        McuFlash_Write(EXTTEMPERATURE_CALIBRATE_FACTOR_ADDRESS + TEMPERATURE_CALIBRATE_FACTOR_LEN * (tempCollecter->number-1),
                TEMPERATURE_CALIBRATE_FACTOR_LEN, writeData);
    }
    else
    {
        McuFlash_Write(TEMPERATURE_CALIBRATE_FACTOR_ADDRESS + TEMPERATURE_CALIBRATE_FACTOR_LEN * tempCollecter->number,
                TEMPERATURE_CALIBRATE_FACTOR_LEN, writeData);
    }

    tempCollecter->tempCalbrateFactor = calibratefactor;

    TRACE_INFO("\n No %d Temp Set negativeInput = ", tempCollecter->number);
    System_PrintfFloat(TRACE_LEVEL_INFO, calibratefactor.negativeInput, 8);
    TRACE_INFO("\n vref = ");
    System_PrintfFloat(TRACE_LEVEL_INFO, calibratefactor.vref, 8);
    TRACE_INFO("\n vcal = ");
    System_PrintfFloat(TRACE_LEVEL_INFO, calibratefactor.vcal, 8);
}

/**
 * @brief 温度计算
 * @param
 */
float TempCollecter_CalcPt1000Temperature(TempCollecter *tempCollecter, uint16_t ad)
{
    double rt;        //PT1000阻值
    float TempRet = -3000;
    Uint16 line, lineStart, lineStop;
    Int16 column;

    rt = tempCollecter->getResistanceValueFunc(&tempCollecter->tempCalbrateFactor, ad);
//    TRACE_INFO("\n rt:%f \n", rt);

    if (rt >= 1000)
    {
        //根据计算得到的rt值缩小查找范围
        if (rt >= g_PT1000TempTable[199][9]) //150℃以上进行查找 Rt>=1573.251
        {
            lineStart = 200;
            lineStop = 250;
        }
        else if (rt >= g_PT1000TempTable[149][9]) //100~150℃进行查找 Rt>=1385.055
        {
            lineStart = 150;
            lineStop = 199;
        }
        else if (rt >= g_PT1000TempTable[99][9]) //50~99.9℃进行查找 Rt>=1193.971
        {
            lineStart = 100;
            lineStop = 149;
        }
        else //0~49.9℃进行查找 Rt>=1000
        {
            lineStart = 50;
            lineStop = 99;
        }

        for (line = lineStart; line <= lineStop; line++)
        {
            for (column = 0; column < 10; column++)
            {
                if (g_PT1000TempTable[line][column] < rt)
                {
                    continue;
                }
                else
                {
                    TempRet = (line - 50) + column / 10.0;
                    System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);

                    return TempRet;
                }
            }
        }
    }
    else if (rt < 1000 && rt > g_PT1000TempTable[49][0])
    {
        float tempTable[10] = {1000, 999.609, 999.218, 998.827, 998.437,
                        998.046,997.655, 997.264, 996.873, 996.482};
        for (int i = 9; i >= 0; i--)
        {
            if (tempTable[i] < rt)
            {
                continue;
            }
            else
            {
                TempRet = -1 * i / 10.0;
                System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);
                return TempRet;
            }
        }
    }
    else //负温度范围
    {
        for (line = 0; line < 50; line++)
        {
            for (column = 9; column >= 0; column--)
            {
                if (g_PT1000TempTable[line][column] < rt)
                {
                    continue;
                }
                else
                {
                    TempRet = (line - 50) - column / 10.0;
                    System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);
                    return TempRet;
                }
            }
        }

        if(rt >= g_PT1000TempTable[0][0] && TempRet < -1000)
        {
            TempRet =  -(1 - (rt - g_PT1000TempTable[49][0])/(g_PT1000TempTable[50][0] - g_PT1000TempTable[49][0]));
            System_PrintfFloat(TRACE_LEVEL_CODE, TempRet, 1);
            return TempRet;
        }
    }
    // TRACE_ERROR("\n error rt %d", rt);
    return TempRet;
}

/**
 * @brief 查询消解器温度
 * @param
 */
float TempCollecter_GetTemperature(TempCollecter *tempCollecter)
{
    float temperature = 0;
    temperature = TempCollecter_CalcPt1000Temperature(tempCollecter,
            TempADCollect_GetAD(0));
    return temperature;
}

