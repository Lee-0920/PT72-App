/*
 * MerterPDDriver.c
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */

#include <LiquidDriver/MeterADC.h>
#include "SystemConfig.h"
#include "System.h"

#define ADC_CHANNEL_NUM           2
#define ADC_DATABUFFER_MAX_SIZE   20

static uint16_t s_ADCDataBufferSize = ADC_DATABUFFER_MAX_SIZE;
static vu16 s_ADCData[ADC_DATABUFFER_MAX_SIZE][ADC_CHANNEL_NUM];
static vu16 s_filterADCData[ADC_CHANNEL_NUM];
static vu16 s_Wave[ADC_CHANNEL_NUM];
#define ADCx                      ADC1
#define ADCx_CLK                  RCC_APB2Periph_ADC1

#define ADCx_CHANNELA_GPIO_CLK    RCC_AHB1Periph_GPIOC
#define ADCx_CHANNELB_GPIO_CLK    RCC_AHB1Periph_GPIOC

#define ADC_CHANNELA              ADC_Channel_12
#define ADC_CHANNELB              ADC_Channel_13

#define GPIO_CHANNELA_PIN         GPIO_Pin_2
#define GPIO_CHANNELA_PORT        GPIOC
#define GPIO_CHANNELB_PIN         GPIO_Pin_3
#define GPIO_CHANNELB_PORT        GPIOC

#define DMA_CHANNELx              DMA_Channel_0
#define DMA_STREAMx               DMA2_Stream0
#define DMA_STREAMx_IRQn          DMA2_Stream0_IRQn
#define DMA_STREAMx_IRQHANDLER    DMA2_Stream0_IRQHandler

static void MerterADC_RCCConfiguration(void)
{
    RCC_AHB1PeriphClockCmd(
            ADCx_CHANNELA_GPIO_CLK | ADCx_CHANNELB_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    RCC_APB2PeriphResetCmd(ADCx_CLK, ENABLE);
    RCC_APB2PeriphResetCmd(ADCx_CLK, DISABLE);
}

static void MerterADC_GPIOConfiguration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_CHANNELA_PIN;
    GPIO_Init(GPIO_CHANNELA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_CHANNELB_PIN;
    GPIO_Init(GPIO_CHANNELB_PORT, &GPIO_InitStructure);
}

static void MerterADC_NVICConfiguration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA_STREAMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
            METER_ADCDMA_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void MerterADC_DMAConfiguration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA_STREAMx);
    while (DMA_GetCmdStatus(DMA_STREAMx) != DISABLE)
        ; //等待 DMA 可配置

    DMA_InitStructure.DMA_Channel = DMA_CHANNELx; //设置 DMA 数据流对应的通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR; //设置 DMA 传输的外设基地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &s_ADCData; //存放 DMA 传输数据的内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //设置数据传输方向
    DMA_InitStructure.DMA_BufferSize = ADC_CHANNEL_NUM*ADC_DATABUFFER_MAX_SIZE; //设置一次传输数据量
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //设置传输数据的时候外设地址是不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //设置传输数据时候内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设的数据长度
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //设置内存的数据长度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //设置 DMA 模式循环采集
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //设置 DMA 通道的优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //是否开启 FIFO 模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //用来选择 FIFO 阈值
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; //配置存储器突发传输配置
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //配置外设突发传输配置
    DMA_Init(DMA_STREAMx, &DMA_InitStructure);
    DMA_ITConfig(DMA_STREAMx, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA_STREAMx, ENABLE);
    while ((DMA_GetCmdStatus(DMA_STREAMx) != ENABLE))
    {
    }
}

static void MerterADC_ADCConfiguration(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = ADC_CHANNEL_NUM;
    ADC_Init(ADCx, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADCx, ADC_CHANNELA, 1, ADC_SampleTime_480Cycles);
    ADC_RegularChannelConfig(ADCx, ADC_CHANNELB, 2, ADC_SampleTime_480Cycles);
//    ADC_RegularChannelConfig(ADCx, ADC_CHANNELA, 2, ADC_SampleTime_480Cycles);
//    ADC_RegularChannelConfig(ADCx, ADC_CHANNELB, 1, ADC_SampleTime_480Cycles);

    ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);

    ADC_DMACmd(ADCx, ENABLE);

    ADC_Cmd(ADCx, ENABLE);
}

void MerterADC_Init(void)
{
    MerterADC_RCCConfiguration();
    MerterADC_DMAConfiguration();
    MerterADC_NVICConfiguration();
    MerterADC_GPIOConfiguration();
    MerterADC_ADCConfiguration();
}

void MeterADC_Stop(void)
{
    DMA_ITConfig(DMA_STREAMx, DMA_IT_TC, DISABLE);
    DMA_Cmd(DMA_STREAMx, DISABLE); //开启 DMA 传输
    while (DMA_GetCmdStatus(DMA_STREAMx) != DISABLE)
        ; //确保 DMA 可以被设置
    ADC_DMARequestAfterLastTransferCmd(ADCx, DISABLE);
    ADC_DMACmd(ADCx, DISABLE);
    ADC_Cmd(ADCx, DISABLE);
}

void MeterADC_Start(void)
{
    DMA_ITConfig(DMA_STREAMx, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA_STREAMx, ENABLE); //开启 DMA 传输
    while ((DMA_GetCmdStatus(DMA_STREAMx) != ENABLE))
        ;
    ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);
    ADC_DMACmd(ADCx, ENABLE);
    ADC_Cmd(ADCx, ENABLE);
    ADC_SoftwareStartConv(ADCx);
}

void MerterADC_SetBufferSize(uint16_t ndtr)
{
    if (ndtr <= ADC_DATABUFFER_MAX_SIZE)
    {
        MeterADC_Stop();
        DMA_SetCurrDataCounter(DMA_STREAMx, ndtr * ADC_CHANNEL_NUM); //数据传输量
        s_ADCDataBufferSize = ndtr;
    }
}

void MeterADC_GetData(uint8_t index, uint16_t *data, uint16_t num)
{
    for (uint16_t i = 0; i < num; i++)
    {
        data[i] = s_ADCData[i][index];
    }
}

static void MeterADC_BubbleSort(uint16_t *data, uint32_t Count)
{
    uint32_t iTemp = 0;
    uint32_t i, j;

    for (i = 1; i < Count; i++)
    {
        for (j = Count - 1; j >= i; j--)
        {
            if (data[j] < data[j - 1])
            {
                iTemp = data[j - 1];
                data[j - 1] = data[j];
                data[j] = iTemp;
            }
        }
    }
}

uint16_t MeterADC_AirADFilter(uint8_t index, uint32_t filterLow,
        uint32_t filterHigh)
{
    uint16_t data[ADC_DATABUFFER_MAX_SIZE] =
    { 0 };
    uint32_t i, result = 0, len;


    for (i = 0; i < s_ADCDataBufferSize; i++)
    {
        data[i] = s_ADCData[i][index];
    }
    if (1 == s_ADCDataBufferSize)
    {
        return data[0];
    }
    if (2 == s_ADCDataBufferSize)
    {
        result = (data[0] + data[1]) / 2;
        return result;
    }

    MeterADC_BubbleSort(data, s_ADCDataBufferSize);

    // 第一个值和最后一个值丢弃,取平均
    len = s_ADCDataBufferSize - filterLow - filterHigh;
    memcpy(data, (data + filterLow), len * sizeof(uint16_t));
    for (i = 0; i < len; i++)
    {
        result += data[i];
    }
    result = result / len;
    return result;
}

static uint32_t MeterADC_CurADFilter(uint8_t index)
{
	//static int count = 0;
    uint16_t data[ADC_DATABUFFER_MAX_SIZE] = { 0 };
    uint16_t result = 0, len, i;
    int wave = 0;
    if (s_ADCDataBufferSize > 1)
    {
        // 第一个值丢弃,剩余取中值
        len = s_ADCDataBufferSize - 1;
        for (i = 0; i < len; i++)
        {
            data[i] = s_ADCData[i + 1][index];
        }
        MeterADC_BubbleSort(data, len);
        if(len % 2)//如果长度为奇数
        {
            result = data[len / 2];
        }
        else
        {
            result = data[len / 2 - 1] + data[len / 2];
            result /= 2;
        }

        for(i = 0; i < len/2; i++)
        {
        	wave += result - data[i];
        }
        for(i = len/2; i < len; i++)
        {
        	wave += data[i] - result;
        }
        s_Wave[index] = wave;
//        if(index == 1)
//        {
//        	Printf("%d] ", wave);
//        	count++;
//        	if(count > 10)
//        	{
//        		count = 0;
//        		Printf(";\n");
//        	}
//        }
//        else
//        {
//        	Printf("[%d,", wave);
//        }

    }
    else
    {
        result = s_ADCData[0][index];
    }
    return result;
}

uint16_t MeterADC_GetCurAD(uint8_t index)
{
    return (s_filterADCData[index]);
}

uint16_t MeterADC_GetCurWare(uint8_t index)
{
    return (s_Wave[index]);
}


void MeterADC_PrintfInfo(void)
{
    Printf("\nADC_data:");
    for (uint16_t i = 0; i < s_ADCDataBufferSize; i++)
    {
        Printf("A:%d,", s_ADCData[i][0]);
        System_PrintfFloat(TRACE_LEVEL_MARK, s_ADCData[i][0] * 2.5 / 4095, 3);
        TRACE_MARK(" V,");
        Printf("B:%d,", s_ADCData[i][1]);
        System_PrintfFloat(TRACE_LEVEL_MARK, s_ADCData[i][1] * 2.5 / 4095, 3);
        TRACE_MARK(" V");
    }
    Printf("\n");

}

void DMA_STREAMx_IRQHANDLER(void)
{
    if (DMA_GetITStatus(DMA_STREAMx, DMA_IT_TCIF0))
    {
        DMA_ClearITPendingBit(DMA_STREAMx, DMA_IT_TCIF0);
        for(uint32_t i = 0; i < ADC_CHANNEL_NUM; i++)
        {
            s_filterADCData[i] = MeterADC_CurADFilter(i);
        }
    }
}
