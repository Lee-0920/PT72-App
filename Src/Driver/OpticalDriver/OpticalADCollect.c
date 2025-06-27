/**
 * @file
 * @brief 光学信号AD采集驱动。
 * @details 提供光学信号AD采集功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */


#include <OpticalDriver/OpticalADCollect.h>
#include <OpticalDriver/OpticalChannel.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"

// **************** 命令 ****************
#define AD7791_CMD_READ_STATUS      0x08    // 读状态
#define AD7791_CMD_SETTING_MODE     0x10    // 设置模式
#define AD7791_CMD_SETTING_FILTER   0x20    // 设置频率(频率与滤波相关)
#define AD7791_CMD_READ_SINGLE      0x38    // 单次读取数据
#define AD7791_CMD_READ_CONTINUOUS  0x3C    // 连续读取数据

// **************** 参数 ****************


// 缓冲
#define AD7791_MODE_BUFFER     (1 << 1)     // 缓存模式
#define AD7791_MODE_UNBUFFER   0x00         // 非缓存模式

// 转换方式
#define AD7791_MODE_CONV_SINGLE      0x80   // 单次转换
#define AD7791_MODE_CONV_CONTINUOUS  0x00   // 连续转换

// 虚位
#define AD7791_DUMMY_BYTE 0xFF

#define DRDY_MODE               (1 << 3)
//#define AD7791_COMMUNICATION_TIMEOUT_MS  25
#define AD7791_COMMUNICATION_TIMEOUT_MS  125
// 频率
#define AD7791_UPDATE_RATES_120    0x00     // 120Hz，25 dB @ 60 Hz
#define AD7791_UPDATE_RATES_100    0x01     // 100Hz，25 dB @ 50 Hz
#define AD7791_UPDATE_RATES_33_3   0x02     // 33.3Hz
#define AD7791_UPDATE_RATES_20     0x03     // 20Hz，80 dB @ 60 Hz
#define AD7791_UPDATE_RATES_16_6   0x04     // 16.6Hz，65 dB @ 50 Hz/60 Hz (Default Setting)
#define AD7791_UPDATE_RATES_16_7   0x05     // 16.7Hz，80 dB @ 50 Hz
#define AD7791_UPDATE_RATES_13_3   0x06     // 13.3Hz
#define AD7791_UPDATE_RATES_9_5    0x07     // 9.5Hz，67 dB @ 50/60 Hz

// CS
#define AD7791_CS_PIN           GPIO_Pin_4
#define AD7791_CS_GPIO          GPIOE
#define AD7791_CS_RCC           RCC_AHB1Periph_GPIOE
#define AD7791_CS_HIGH()        GPIO_SetBits(AD7791_CS_GPIO, AD7791_CS_PIN)
#define AD7791_CS_LOW()         GPIO_ResetBits(AD7791_CS_GPIO, AD7791_CS_PIN)

// SCLK
#define AD7791_SCLK_PIN         GPIO_Pin_2
#define AD7791_SCLK_GPIO        GPIOE
#define AD7791_SCLK_RCC         RCC_AHB1Periph_GPIOE
#define AD7791_SCLK_HIGH()      GPIO_SetBits(AD7791_SCLK_GPIO, AD7791_SCLK_PIN)
#define AD7791_SCLK_LOW()       GPIO_ResetBits(AD7791_SCLK_GPIO, AD7791_SCLK_PIN)

// DIN
#define AD7791_DIN_PIN          GPIO_Pin_6
#define AD7791_DIN_GPIO         GPIOE
#define AD7791_DIN_RCC          RCC_AHB1Periph_GPIOE
#define AD7791_DIN_HIGH()       GPIO_SetBits(AD7791_DIN_GPIO, AD7791_DIN_PIN)
#define AD7791_DIN_LOW()        GPIO_ResetBits(AD7791_DIN_GPIO, AD7791_DIN_PIN)

// DOUT
#define AD7791_DOUT_PIN         GPIO_Pin_5
#define AD7791_DOUT_GPIO        GPIOE
#define AD7791_DOUT_RCC         RCC_AHB1Periph_GPIOE
#define AD7791_DOUT_READ()      GPIO_ReadInputDataBit(AD7791_DOUT_GPIO, AD7791_DOUT_PIN)

#define IGL_CTRL_PIN            GPIO_Pin_13
#define IGL_CTRL_PORT           GPIOE
#define IGL_CTRL_RCC            RCC_AHB1Periph_GPIOE
#define IGL_CTRL_HIGH()         GPIO_SetBits(IGL_CTRL_PORT, IGL_CTRL_PIN)
#define IGL_CTRL_LOW()          GPIO_ResetBits(IGL_CTRL_PORT, IGL_CTRL_PIN)

#define IGL_RESET_PIN           GPIO_Pin_14
#define IGL_RESET_PORT          GPIOE
#define IGL_RESET_RCC           RCC_AHB1Periph_GPIOE
#define IGL_RESET_HIGH()        GPIO_SetBits(IGL_RESET_PORT, IGL_RESET_PIN)
#define IGL_RESET_LOW()         GPIO_ResetBits(IGL_RESET_PORT, IGL_RESET_PIN)


static unsigned char s_AD7791PolarMode = AD7791_MODE_BIPOLAR;
/**
 * @brief AD7791发送字节
 * @param
 */
static uint8_t OpticalADCollect_AD7791SendByte(uint8_t senddata)
{
    uint8_t i = 0;
    uint8_t receivedata = 0;
    for (i = 0; i < 8; i++)
    {
        // 发送
        AD7791_SCLK_LOW();
        if (senddata & 0x80)
        {
            AD7791_DIN_HIGH();
        }
        else
        {
            AD7791_DIN_LOW();
        }
        senddata <<= 1;
        AD7791_SCLK_HIGH();

        // 接收
        receivedata <<= 1;
        if (AD7791_DOUT_READ())
        {
            receivedata |= 0x01;
        }
    }
    return receivedata;
}

/**
 * @brief 检查AD7791是否转换完成
 * @param
 */
static Bool OpticalADCollect_AD7791Ready(uint32_t timeoutms)
{
    uint32_t loopms = 0;
    uint32_t looptimes = 0;
    loopms = 2;
    looptimes = timeoutms / loopms;     // 循环查询次数
    while (AD7791_DOUT_READ())       // 等待DOUT由期间拉低
    {
        System_Delay(loopms);
        if (0 == looptimes)
        {
            return 0;
        }
        looptimes--;
    }
    return 1;
}

/**
 * @brief 读取AD7791 24位数据
 * @param
 */
static uint32_t OpticalADCollect_AD7791Read24bitData()
{
    uint32_t read_data = 0;
    // 读数据
    read_data = 0;
    read_data |= OpticalADCollect_AD7791SendByte(AD7791_DUMMY_BYTE) << 16;
    read_data |= OpticalADCollect_AD7791SendByte(AD7791_DUMMY_BYTE) << 8;
    read_data |= OpticalADCollect_AD7791SendByte(AD7791_DUMMY_BYTE);
    return read_data;
}


/**
 * @brief 光学采集驱动初始化
 * @param
 */
void OpticalADCollect_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //时钟配置
    RCC_AHB1PeriphClockCmd(AD7791_CS_RCC |
    AD7791_SCLK_RCC |
    AD7791_DIN_RCC |
    AD7791_DOUT_RCC |
    IGL_CTRL_RCC |
    IGL_RESET_RCC, ENABLE);

    //IO配置
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // CS
    GPIO_InitStructure.GPIO_Pin = AD7791_CS_PIN;
    GPIO_Init(AD7791_CS_GPIO, &GPIO_InitStructure);
    // SCLK
    GPIO_InitStructure.GPIO_Pin = AD7791_SCLK_PIN;
    GPIO_Init(AD7791_SCLK_GPIO, &GPIO_InitStructure);
    // DIN
    GPIO_InitStructure.GPIO_Pin = AD7791_DIN_PIN;
    GPIO_Init(AD7791_DIN_GPIO, &GPIO_InitStructure);


    // DOUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = AD7791_DOUT_PIN;
    GPIO_Init(AD7791_DOUT_GPIO, &GPIO_InitStructure);

    // 初始引脚
    AD7791_CS_HIGH();
    AD7791_SCLK_HIGH();
    AD7791_DIN_HIGH();

    // 复位
    AD7791_CS_LOW();    // 使能
    OpticalADCollect_AD7791SendByte(0xff);       // 复位芯片，最少32个时钟的 DIN 高电平
    OpticalADCollect_AD7791SendByte(0xff);
    OpticalADCollect_AD7791SendByte(0xff);
    OpticalADCollect_AD7791SendByte(0xff);
    OpticalADCollect_AD7791SendByte(0xff);
    OpticalADCollect_AD7791SendByte(0xff);
    AD7791_CS_HIGH();   // 失能

    // 设置采样频率
    AD7791_CS_LOW();    // 使能
    OpticalADCollect_AD7791SendByte(AD7791_CMD_SETTING_FILTER);
    OpticalADCollect_AD7791SendByte(AD7791_UPDATE_RATES_33_3 & 0x07);

    AD7791_CS_HIGH();   // 失能

    OpticalChannel_Init();
}

uint32_t OpticalADCollect_GetAD(Uint8 channel)
{
    uint32_t data = 0;

    OpticalChannel_Select(channel);
    System_Delay(20);
    AD7791_CS_LOW();    // 使能
    OpticalADCollect_AD7791SendByte(AD7791_CMD_SETTING_MODE);
    OpticalADCollect_AD7791SendByte(
            AD7791_MODE_CONV_SINGLE | AD7791_MODE_BUFFER | s_AD7791PolarMode);
    if (0 == OpticalADCollect_AD7791Ready(AD7791_COMMUNICATION_TIMEOUT_MS)) // 等待转换完成
    {
        TRACE_ERROR("\n mea time over");
        return 0;   // 如果超时
    }
    // 读取数据
    OpticalADCollect_AD7791SendByte(AD7791_CMD_READ_SINGLE);
    data = OpticalADCollect_AD7791Read24bitData();
    AD7791_CS_HIGH();   // 失能
    return (data & 0xFFFFFF);  // 24bit
}

void OpticalADCollect_ChangePolar(unsigned char polar)
{
    s_AD7791PolarMode = polar;
}
