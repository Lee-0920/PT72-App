/**
 * @file
 * @brief ADS11146信号采集驱动。
 * @details 提供电极信号AD采集功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#include <OpticalDriver/ADS1146Collect.h>
#include <OpticalDriver/OpticalChannel.h>
#include <OpticalDriver/OpticalXonen.h>
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "FreeRTOS.h"
#include "task.h"

// **************** 命令 ****************
#define ADS1146_CMD_WAKEUP      0x01        //
#define ADS1146_CMD_SLEEP       0x03        //
#define ADS1146_CMD_SYNC        0x05        //
#define ADS1146_CMD_RESET       0x07        //
#define ADS1146_CMD_NOP         0xFF        //
#define ADS1146_CMD_RDATA       0x13        //
#define ADS1146_CMD_RDATAC      0x15        //
#define ADS1146_CMD_SDATAC      0x17        //
#define ADS1146_CMD_RREG        0x20        //
#define ADS1146_CMD_WREG        0x40        //
#define ADS1146_CMD_SYSOCAL     0x60        //
#define ADS1146_CMD_SYSGCAL     0x61        //
#define ADS1146_CMD_SELFOCAL    0x62        //
#define ADS1146_CMD_RESTRICTED  0xF1        //

// **************** 地址 ****************
#define ADS1146_REG_ADDR_BCS    0x00
#define ADS1146_REG_ADDR_VBIAS  0x01
#define ADS1146_REG_ADDR_MUX1   0x02
#define ADS1146_REG_ADDR_SYS0   0x03
#define ADS1146_REG_ADDR_OFC0   0x04
#define ADS1146_REG_ADDR_OFC1   0x05
#define ADS1146_REG_ADDR_OFC2   0x06
#define ADS1146_REG_ADDR_FSC0   0x07
#define ADS1146_REG_ADDR_FSC1   0x08
#define ADS1146_REG_ADDR_FSC2   0x09
#define ADS1146_REG_ADDR_ID     0x0A

// **************** BSC ****************
#define BCS0                    (1 << 6)
#define BCS1                    (1 << 7)

// **************** VBIAS ****************
#define VBIAS0                  (1 << 0)
#define VBIAS1                  (1 << 1)

// **************** MUXCAL ****************
#define DEFAULT_CAL             0x00
#define OFFSET_CAL              0x01
#define GAIN_CAL                0x02
#define TEMP_CAL                0x03

// **************** SYS0 ****************
// ######## GAIN ########
#define GAIN_1                  0x00
#define GAIN_2                  0x10
#define GAIN_4                  0x20
#define GAIN_8                  0x30
#define GAIN_16                 0x40
#define GAIN_32                 0x50
#define GAIN_64                 0x60
#define GAIN_128                0x70
// ######## RATE ########
#define RATE_5                  0x00
#define RATE_10                 0x01
#define RATE_20                 0x02
#define RATE_40                 0x03
#define RATE_80                 0x04
#define RATE_160                0x05
#define RATE_320                0x06
#define RATE_640                0x07
#define RATE_1000               0x08
#define RATE_2000               0x09

#define ADS1146_COMMUNICATION_TIMEOUT_MS  8             // 等待转换完成超时时间

#define SAMPLE_SYNC_TIMEOUT_MS            40            // 等待采样同步信号超时时间

// CS_MUTEX
//#define ADS1146_MUTEX_PIN           GPIO_Pin_2
//#define ADS1146_MUTEX_GPIO          GPIOE
//#define ADS1146_MUTEX_RCC           RCC_AHB1Periph_GPIOE
//#define ADS1146_MUTEX_HIGH()        GPIO_SetBits(ADS1146_MUTEX_GPIO, ADS1146_MUTEX_PIN)
//#define ADS1146_MUTEX_LOW()         GPIO_ResetBits(ADS1146_MUTEX_GPIO, ADS1146_MUTEX_PIN)

// CS
#define ADS1146_CS_PIN           GPIO_Pin_4
#define ADS1146_CS_GPIO          GPIOE
#define ADS1146_CS_RCC           RCC_AHB1Periph_GPIOE
#define ADS1146_CS_HIGH()        GPIO_SetBits(ADS1146_CS_GPIO, ADS1146_CS_PIN)
#define ADS1146_CS_LOW()         GPIO_ResetBits(ADS1146_CS_GPIO, ADS1146_CS_PIN)

// SCLK
#define ADS1146_SCLK_PIN         GPIO_Pin_2
#define ADS1146_SCLK_GPIO        GPIOE
#define ADS1146_SCLK_RCC         RCC_AHB1Periph_GPIOE
#define ADS1146_SCLK_HIGH()      GPIO_SetBits(ADS1146_SCLK_GPIO, ADS1146_SCLK_PIN)
#define ADS1146_SCLK_LOW()       GPIO_ResetBits(ADS1146_SCLK_GPIO, ADS1146_SCLK_PIN)

// DIN
#define ADS1146_DIN_PIN          GPIO_Pin_6
#define ADS1146_DIN_GPIO         GPIOE
#define ADS1146_DIN_RCC          RCC_AHB1Periph_GPIOE
#define ADS1146_DIN_HIGH()       GPIO_SetBits(ADS1146_DIN_GPIO, ADS1146_DIN_PIN)
#define ADS1146_DIN_LOW()        GPIO_ResetBits(ADS1146_DIN_GPIO, ADS1146_DIN_PIN)

// DOUT
#define ADS1146_DOUT_PIN         GPIO_Pin_5
#define ADS1146_DOUT_GPIO        GPIOE
#define ADS1146_DOUT_RCC         RCC_AHB1Periph_GPIOE
#define ADS1146_DOUT_READ()      GPIO_ReadInputDataBit(ADS1146_DOUT_GPIO, ADS1146_DOUT_PIN)

// START
#define ADS1146_START_PIN        GPIO_Pin_13
#define ADS1146_START_GPIO       GPIOC
#define ADS1146_START_RCC        RCC_AHB1Periph_GPIOC
#define ADS1146_START_HIGH()     GPIO_SetBits(ADS1146_START_GPIO, ADS1146_START_PIN)
#define ADS1146_START_LOW()      GPIO_ResetBits(ADS1146_START_GPIO, ADS1146_START_PIN)

// DRDY
#define ADS1146_DRDY_PIN         GPIO_Pin_3
#define ADS1146_DRDY_GPIO        GPIOE
#define ADS1146_DRDY_RCC         RCC_AHB1Periph_GPIOE
#define ADS1146_DRDY_READ()      GPIO_ReadInputDataBit(ADS1146_DRDY_GPIO, ADS1146_DRDY_PIN)

/**
 * @brief ADS1146发送字节
 * @param
 */
static uint8_t ADS1146Collect_ADS1146SendByte(uint8_t senddata)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;

    for (i = 0; i < 8; i++)
    {
        // 发送
        ADS1146_SCLK_HIGH();
        if (senddata & 0x80)
        {
            ADS1146_DIN_HIGH();
        }
        else
        {
            ADS1146_DIN_LOW();
        }
        senddata <<= 1;

        ADS1146_SCLK_LOW();

        // 接收
        receivedata <<= 1;
        if (ADS1146_DOUT_READ())
        {
            receivedata |= 0x01;
        }
    }

    return receivedata;
}

/**
 * @brief 光学采集驱动初始化
 * @param
 */
void ADS1146Collect_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(ADS1146_CS_RCC |
//            ADS1146_MUTEX_RCC |
            ADS1146_SCLK_RCC |
            ADS1146_DIN_RCC |
            ADS1146_DOUT_RCC |
            ADS1146_START_RCC |
            ADS1146_DRDY_RCC , ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS_MUTEX
//    GPIO_InitStructure.GPIO_Pin = ADS1146_MUTEX_PIN;
//    GPIO_Init(ADS1146_MUTEX_GPIO, &GPIO_InitStructure);

    // CS
    GPIO_InitStructure.GPIO_Pin = ADS1146_CS_PIN;
    GPIO_Init(ADS1146_CS_GPIO, &GPIO_InitStructure);

    // SCLK
    GPIO_InitStructure.GPIO_Pin = ADS1146_SCLK_PIN;
    GPIO_Init(ADS1146_SCLK_GPIO, &GPIO_InitStructure);

    // DIN
    GPIO_InitStructure.GPIO_Pin = ADS1146_DIN_PIN;
    GPIO_Init(ADS1146_DIN_GPIO, &GPIO_InitStructure);
    // START
    GPIO_InitStructure.GPIO_Pin = ADS1146_START_PIN;
    GPIO_Init(ADS1146_START_GPIO, &GPIO_InitStructure);

    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = ADS1146_DOUT_PIN;
    GPIO_Init(ADS1146_DOUT_GPIO, &GPIO_InitStructure);

    // DRDY
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = ADS1146_DRDY_PIN;
    GPIO_Init(ADS1146_DRDY_GPIO, &GPIO_InitStructure);

    //禁用冲突片选
//    ADS1146_MUTEX_HIGH();

    // 初始引脚
    ADS1146_CS_HIGH();
    ADS1146_SCLK_HIGH();
    ADS1146_DIN_HIGH();
    ADS1146_START_LOW();

    // SPI接口初始化
    ADS1146_CS_HIGH();       // 失能
    System_NonOSDelay(1);
    ADS1146_CS_LOW();        // 使能

    ADS1146_START_HIGH();

    // 复位
    ADS1146Collect_ADS1146SendByte(ADS1146_CMD_RESET);

    // 等待 至少0.6ms
    System_NonOSDelay(1);//1ms

    // 设置增益和转换频率
    ADS1146Collect_ADS1146SendByte(ADS1146_CMD_WREG | ADS1146_REG_ADDR_SYS0);
    ADS1146Collect_ADS1146SendByte(0x00);
    ADS1146Collect_ADS1146SendByte(GAIN_64 | RATE_160);

    // 自动校准
    ADS1146Collect_ADS1146SendByte(ADS1146_CMD_SELFOCAL);

    ADS1146_START_LOW();

    ADS1146_CS_HIGH();       // 失能

    TRACE_DEBUG("\n ADS1146 Collect Init Over");
}

/**
 * @brief 检查ADS1146是否转换完成
 * @param
 */
static Bool ADS1146Collect_ADS1146Ready(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 8;
    looptimes = timeoutms / loopms;     // 循环查询次数

    while (ADS1146_DRDY_READ())         // 等待DOUT由期间拉低
    {
        System_Delay(loopms);

        if (0 == looptimes)
        {
            return FALSE;
        }
        looptimes--;
    }

    return TRUE;
}

/**
 * @brief 读取ADS1146 16位数据
 * @param
 */
static uint32_t ADS1146Collect_ADS1146Read16bitData()
{
    uint32_t read_data = 0;

    // 读数据
    read_data = 0;
    read_data |= ADS1146Collect_ADS1146SendByte(ADS1146_CMD_NOP) << 8;
    read_data |= ADS1146Collect_ADS1146SendByte(ADS1146_CMD_NOP);

    return read_data;
}

Uint16 ADS1146Collect_GetAD(Uint8 channel)
{
    Uint16 data = 0;

    OpticalChannel_Select(channel);

    ADS1146_CS_LOW();    // 使能
    ADS1146_START_HIGH();
    System_Delay(1);
    ADS1146_START_LOW();        // 停止转换

    if (FALSE == ADS1146Collect_ADS1146Ready(ADS1146_COMMUNICATION_TIMEOUT_MS))
    {
        TRACE_ERROR("\n ADS1146 collect time over");
        return 0;   // 如果超时
    }

    // 读取数据
    ADS1146Collect_ADS1146SendByte(ADS1146_CMD_RDATA);
    data = ADS1146Collect_ADS1146Read16bitData();

    if (data > 0x7fff)
    {
        data = 0;
    }
    ADS1146_CS_HIGH();          // 失能

    return (data & 0xFFFF);     // 16bit
}

/**
 * @brief 检查采样同步信号是否到来
 * @param
 */
static Bool ADS1146Collect_IsGetADAble(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;

    loopms = 4;
    looptimes = timeoutms / loopms;             // 循环查询次数

    while(FALSE == collectSyncFlag)
    {
        System_Delay(loopms);
        if (0 == looptimes)
        {
            return FALSE;
        }
        looptimes--;
    }
    return TRUE;
}

Uint16 ADS1146Collect_GetADWithSync(Uint8 channel)
{
    Uint16 result;
    if (FALSE == ADS1146Collect_IsGetADAble(SAMPLE_SYNC_TIMEOUT_MS))
    {
       TRACE_ERROR("\n sample sync time over ");
       return FALSE;
    }
    result = ADS1146Collect_GetAD(channel);
    collectSyncFlag = FALSE;
    return result;
}

Bool ADS1146Collect_GetDoubleChannelADWithSync(Uint16 *channel1Buff, Uint8 channel1, Uint16 *channel2Buff, Uint8 channel2, Uint8 len, Uint32 sampleTimeOut)
{
    TickType_t sampleTimeout, sampleInitTime;
    Uint16 data   = 0;
    Uint8 dataLen = 0;
    Uint8 i       = 0;
    Uint8 channel[2] = {channel1, channel2};
    Uint16 *dataBuff[2] = {channel1Buff, channel2Buff};

    if (FALSE == ADS1146Collect_IsGetADAble(SAMPLE_SYNC_TIMEOUT_MS))
    {
       TRACE_ERROR("\n sample sync time over ");
       return FALSE;
    }

    for (Uint8 j = 0 ; j < 2; j++)
    {
        sampleInitTime = xTaskGetTickCount();
        sampleTimeout = sampleTimeOut  + sampleInitTime;
        for (i = 0; i < len; i++)
        {
           data = ADS1146Collect_GetAD(channel[j]);

           dataBuff[j][i] = data;

           if (xTaskGetTickCount() >= sampleTimeout)
           {
//               TRACE_INFO("\ncur %d  timeout %d init %d", xTaskGetTickCount(), sampleTimeout, sampleInitTime);
               dataLen = 0;
               TRACE_ERROR("\n %d channel sample time over", channel[j]);
               break;
           }
           dataLen = i + 1;
        }
//        TRACE_INFO("\n %d channel sample data length = %d sample Time = %dms ", channel[j], dataLen, xTaskGetTickCount() - sampleInitTime);

        if (dataLen < len)
        {
            TRACE_ERROR("\n %d channel sample len = %d, curlen = %d", channel[j], len, dataLen);
            return FALSE;
        }
//        for (i = 0; i < dataLen; i++)
//        {
//            TRACE_INFO("\n ad = %d", dataBuff[j][i]);
//        }
    }
    collectSyncFlag = FALSE;
    return TRUE;
}

Bool ADS1146Collect_GetDoubleChannelAD(Uint16 *channel1Buff, Uint8 channel1, Uint16 *channel2Buff, Uint8 channel2, Uint8 len, Uint32 sampleTimeOut)
{
    TickType_t sampleTimeout, sampleInitTime;
    Uint16 data   = 0;
    Uint8 dataLen = 0;
    Uint8 i       = 0;
    Uint8 channel[2] = {channel1, channel2};
    Uint16 *dataBuff[2] = {channel1Buff, channel2Buff};

    for (Uint8 j = 0 ; j < 2; j++)
    {
        sampleInitTime = xTaskGetTickCount();
        sampleTimeout = sampleTimeOut  + sampleInitTime;
        for (i = 0; i < len; i++)
        {
           data = ADS1146Collect_GetAD(channel[j]);

           dataBuff[j][i] = data;

           if (xTaskGetTickCount() >= sampleTimeout)
           {
               dataLen = 0;
               TRACE_ERROR("\n %d channel sample time over", channel[j]);
               break;
           }
           dataLen = i + 1;
        }
//        TRACE_INFO("\n %d channel sample data length = %d sample Time = %dms ", channel[j], dataLen, xTaskGetTickCount() - sampleInitTime);

        if (dataLen < len)
        {
            TRACE_ERROR("\n %d channel sample len = %d, curlen = %d", len, dataLen);
            return FALSE;
        }
//        for (i = 0; i < dataLen; i++)
//        {
//            TRACE_INFO("\n ad = %d", dataBuff[j][i]);
//        }
    }
    return TRUE;
}

