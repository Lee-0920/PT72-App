/**
 * @file
 * @brief 光学信号LED驱动。
 * @details 提供光学信号LED控制功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#include <OpticalDriver/OpticalLed.h>
#include "stm32f4xx.h"
#include "SystemConfig.h"
#include "Driver/McuFlash.h"
#include "Driver/System.h"
#include "Driver/HardwareType.h"

// **************** 引脚定义 ****************
#define OPT_DAC_PIN          GPIO_Pin_5
#define OPT_DAC_PORT         GPIOA
#define OPT_DAC_RCC          RCC_AHB1Periph_GPIOA

#define OPT_DAC2_PIN          GPIO_Pin_4
#define OPT_DAC2_PORT         GPIOA
#define OPT_DAC2_RCC          RCC_AHB1Periph_GPIOA

#define OPT_LED_PIN          GPIO_Pin_6
#define OPT_LED_PORT         GPIOA
#define OPT_LED_RCC          RCC_AHB1Periph_GPIOA
#define OPT_LED_OFF()       GPIO_SetBits(OPT_LED_PORT, OPT_LED_PIN)
#define OPT_LED_ON()        GPIO_ResetBits(OPT_LED_PORT, OPT_LED_PIN)

#define DAC_DEFAULT_VALUE  1.0

static float s_DACDefValue[2] = {1.0, 1.0};
static float s_DACOutValue[2] = {1.0, 1.0};

static GPIO_BaseStruct g_ledPower;
static GPIO_BaseStruct g_ledDac[2];
static Bool g_ledValid[2] = {FALSE, FALSE};

static void OpticalLed_Config(void)
{
//    g_ledPower.pin = GPIO_Pin_6;
//    g_ledPower.port = GPIOA;
//    g_ledPower.rcc = RCC_AHB1Periph_GPIOA;

    g_ledDac[0].pin = GPIO_Pin_4;
    g_ledDac[0].port = GPIOA;
    g_ledDac[0].rcc = RCC_AHB1Periph_GPIOA;
    g_ledValid[0] = TRUE;
}

void OpticalLed_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitStructure;

    OpticalLed_Config();

    RCC_AHB1PeriphClockCmd(g_ledPower.rcc | g_ledDac[0].rcc | g_ledDac[1].rcc, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

//    // 变量初始化
//    GPIO_InitStructure.GPIO_Pin = g_ledPower.pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(g_ledPower.port, &GPIO_InitStructure);
//    //OPT_LED_OFF();
//    GPIO_SetBits(g_ledPower.port, g_ledPower.pin);

    GPIO_InitStructure.GPIO_Pin =g_ledDac[0].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//配置为模拟输入，连接到DAC时会连接到模拟输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(g_ledDac[0].port, &GPIO_InitStructure);

//    if(g_ledValid[1] == TRUE)
//    {
//        GPIO_InitStructure.GPIO_Pin = g_ledDac[1].pin;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//配置为模拟输入，连接到DAC时会连接到模拟输出
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//        GPIO_Init(g_ledDac[1].port, &GPIO_InitStructure);
//    }

    DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;//不使用触发功能
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//不使用波形发生
    DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//屏蔽/幅值选择器
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//输出缓存关闭
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);
    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_SetChannel2Data(DAC_Align_12b_R, 0);//12位右对齐数据格式设置DAC，输出0V

//    if(g_ledValid[1] == TRUE)
//    {
//        DAC_Init(DAC_Channel_1, &DAC_InitStructure);
//        DAC_Cmd(DAC_Channel_1, ENABLE);
//        DAC_SetChannel1Data(DAC_Align_12b_R, 0);//12位右对齐数据格式设置DAC，输出0V
//    }

    OpticalLed_InitParam(0);
//    OpticalLed_InitParam(1);
}

void OpticalLed_InitParam(Uint8 index)
{
    Uint8 buffer[LEDDAC_SIGN_FLASH_LEN] = { 0 };
    Uint32 flashFactorySign = 0;

    if(index > 0)  //扩展的LED
    {
        McuFlash_Read(EXTLEDDAC_SIGN_FLASH_BASE_ADDR, LEDDAC_SIGN_FLASH_LEN, buffer); //读取出厂标志位
    }
    else
    {
        McuFlash_Read(LEDDAC_SIGN_FLASH_BASE_ADDR, LEDDAC_SIGN_FLASH_LEN, buffer); //读取出厂标志位
    }
    memcpy(&flashFactorySign, buffer, LEDDAC_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign) //表示已经过出厂设置
    {
        s_DACDefValue[index] = OpticalLed_GetDefaultValue(index);
    }
    else
    {
        OpticalLed_SetDefaultValue(index, DAC_DEFAULT_VALUE);

        flashFactorySign = FLASH_FACTORY_SIGN;
        memcpy(buffer, &flashFactorySign, LEDDAC_SIGN_FLASH_LEN);

        if(index > 0)  //扩展的LED
        {
            McuFlash_Write(EXTLEDDAC_SIGN_FLASH_BASE_ADDR, LEDDAC_SIGN_FLASH_LEN, buffer);  //写入出厂标志
        }
        else
        {
            McuFlash_Write(LEDDAC_SIGN_FLASH_BASE_ADDR, LEDDAC_SIGN_FLASH_LEN, buffer);  //写入出厂标志
        }
    }
}

float OpticalLed_GetDefaultValue(Uint8 index)
{
    float value = 0;
    if(index < DAC_CHANNEL_NUM)
    {
        Uint8 readData[LEDDAC_DEFAULT_VALUE_FLASH_LEN] = { 0 };

        if(index > 0)  //扩展的LED
        {
            McuFlash_Read(EXTLEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS, LEDDAC_DEFAULT_VALUE_FLASH_LEN, readData);
        }
        else
        {
            McuFlash_Read(LEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS, LEDDAC_DEFAULT_VALUE_FLASH_LEN, readData);
        }
        memcpy(&value, readData, sizeof(float));
    }
    else
    {
        TRACE_ERROR("\nError DAC channel index %d", index);
    }

    return value;
}

Bool OpticalLed_SetDefaultValue(Uint8 index, float value)
{
    Uint8 writeData[LEDDAC_DEFAULT_VALUE_FLASH_LEN] = { 0 };

    if(index < DAC_CHANNEL_NUM)
    {
        if(value  >= 0 && value <= DAC_MAX_LIMIT_VALUE)
        {
            memcpy(writeData, &value, sizeof(float));
            if(index > 0)  //扩展的LED
            {
                McuFlash_Write(EXTLEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS, LEDDAC_DEFAULT_VALUE_FLASH_LEN, writeData);
            }
            else
            {
                McuFlash_Write(LEDDAC_DEFAULT_VALUE_FLASH_BASE_ADDRESS, LEDDAC_DEFAULT_VALUE_FLASH_LEN, writeData);
            }
            s_DACDefValue[index] = value;

            TRACE_INFO("\n LED DAC %d set default value : %f v", index, value);
            return TRUE;
        }
        else
        {
            TRACE_INFO("\n Error param : %f . DAC value between 0 ~ 2.5 V", value);
            return FALSE;
        }
    }
    else
    {
        TRACE_ERROR("\nError DAC channel index %d", index);
        return FALSE;
    }
}


void OpticalLed_TurnOn(Uint8 index)
{
    Uint16 vol;
    if(index < DAC_CHANNEL_NUM && g_ledValid[index] == TRUE)
    {
        //OPT_LED_ON();
//        GPIO_ResetBits(g_ledPower.port, g_ledPower.pin);
        vol = (Uint16)(s_DACDefValue[index] * 4095 / 2.5);
        if(index == 0)
        {
            DAC_SetChannel1Data(DAC_Align_12b_R,vol);
        }
//        else if(index == 1)
//        {
//            DAC_SetChannel1Data(DAC_Align_12b_R,vol);
//        }
    }
    else
    {
        TRACE_ERROR("\nError DAC channel index %d", index);
    }
}

void OpticalLed_TurnOff(Uint8 index)
{
    if(index < DAC_CHANNEL_NUM && g_ledValid[index] == TRUE)
    {
        s_DACOutValue[index] = 0;
        if(index == 0)
        {
            DAC_SetChannel1Data(DAC_Align_12b_R,0);
        }
//        else if(index == 1)
//        {
//            DAC_SetChannel1Data(DAC_Align_12b_R,0);
//        }

        if(s_DACOutValue[0] < 1 && s_DACOutValue[1] < 1)
        {
            //OPT_LED_OFF();
//            GPIO_SetBits(g_ledPower.port, g_ledPower.pin);
        }
    }
    else
    {
        TRACE_ERROR("\nError DAC channel index %d", index);
    }
}

void OpticalLed_ChangeDACOut(Uint8 index, float valve)
{
    Uint16 vol;
    if(index < DAC_CHANNEL_NUM && g_ledValid[index] == TRUE)
    {
        s_DACOutValue[index] = valve;
        vol = (Uint16)(s_DACOutValue[index] * 4095 / 2.5);
        if(index == 0)
        {
            DAC_SetChannel1Data(DAC_Align_12b_R,vol);
        }
//        else if(index == 1)
//        {
//            DAC_SetChannel1Data(DAC_Align_12b_R,vol);
//        }
    }
    else
    {
        TRACE_ERROR("\nError DAC channel index %d", index);
    }
}

