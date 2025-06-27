/*
 * SpinValveDriver.c
 *
 *  Created on: 2019年9月11日
 *      Author: Administrator
 */

#include "SpinValveDriver.h"
#include "SystemConfig.h"
#include "Tracer/Trace.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdint.h"

#define RS232DRIVER_IRQn                                USART2_IRQn
#define RS232DRIVER_UART_IRQHANDLER             		USART2_IRQHandler
#define RS232DRIVER_CLK_CONFIG                          RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
#define RS232DRIVER_RX_GPIO_CLK_CONFIG          		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define RS232DRIVER_TX_GPIO_CLK_CONFIG          		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define RS232DRIVER_GPIO_AF                             GPIO_AF_USART2
#define RS232DRIVER_TX_PIN                              GPIO_Pin_2
#define RS232DRIVER_TX_PinSource                        GPIO_PinSource2
#define RS232DRIVER_TX_GPIO_PORT                      	GPIOA
#define RS232DRIVER_RX_PIN                              GPIO_Pin_3
#define RS232DRIVER_RX_PinSource                        GPIO_PinSource3
#define RS232DRIVER_RX_GPIO_PORT                      	GPIOA
#define RS232DRIVER_USARTx                              USART2
#define RS232DRIVER_USARTx_BAUD                     	9600

// 消息队列
xQueueHandle g_spinValveDriverQueue = NULL;

// 接收状态机
#define RS232DRIVER_RX_STATE_IDLE               0x00
#define RS232DRIVER_RX_STATE_GOT_FRAME_HEAD     0x01    // 已经检测到帧头
#define RS232DRIVER_RX_STATE_GOT_FRAME_ERROR    0x02    // 出现错误
#define RX232DRIVER_RX_STATE_WAIT_CHECK_LOW     0x03    //等待校验和低位
#define RX232DRIVER_RX_STATE_WAIT_CHECK_HIGH    0x04    //等待校验和高位
// 错误填充
#define PARITY_ERROR_BYTE   				0x81    // 校验错误填充字节
#define HIGH_BIT_ERROR_BYTE 				0x80    // 高位错误填充字节

// 帧头
#define RS232DRIVER_FRAME_HEAD 0xCC
// 帧尾
#define RS232DRIVER_FRAME_TAIL  			0xDD

// 接收帧最大长度 //不含校验和
#define RS232DRIVER_MAX_RECV_FRAME_LEN  12

// 普通命令帧长度  //不含校验和
#define RS232DRIVER_CMD_FRAME_LEN  6
// 普通命令帧参数长度
#define RS232DRIVER_CMD_PARAM_LEN 2

// 工厂命令帧长度  //不含校验和
#define RS232DRIVER_FACTORY_CMD_FRAME_LEN  	12
// 工厂命令帧密码长度
#define RS232DRIVER_FACTORY_CMD_PWD_LEN 	4
// 工厂命令帧参数长度
#define RS232DRIVER_FACTORY_CMD_PARAM_LEN 	4

// 回应帧长度  //不含校验和
#define RS232DRIVER_RESP_FRAME_LEN  6

// 回应码索引
#define RS232DRIVER_RESP_CODE_INDEX 		2

// 缓存大小
#define RS232DRIVER_TX_BUFF_SIZE  			128
#define RS232DRIVER_RX_BUFF_SIZE  			RS232DRIVER_MAX_RECV_FRAME_LEN

// 缓存
static uint8_t s_RxBuffer[RS232DRIVER_RX_BUFF_SIZE] = {0};
static uint8_t s_TxBuffer[RS232DRIVER_TX_BUFF_SIZE] = {0};
static uint8_t s_TxBufferHead 	= 0;     	// 发送缓冲区头指针,读指针
static uint8_t s_TxBufferTail 	= 0;     	// 发送缓冲区尾指针,写指针
static uint8_t s_TxBufferLen 	= 0;		// 发送缓冲区未发送数据个数

//地址
static uint8_t s_addr = 0;

// ISR hook
//static void (*uart1_ISR_hook)(void) = NULL;

// 发送消息参数
//static signed portBASE_TYPE uart1_xLinSendTaskHigherPriorityTaskWoken = 0;

/** ----------------------- Function ----------------------------------------**/
static void SpinValveDriver_PrintData(Uint8* buffer, Uint8 len);
static uint32_t SpinValveDriver_WriteData(uint8_t *pTx_Buff, uint8_t len);

/**
 * @brief 串口配置初始化
 */
void SpinValveDriver_RS232Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    //打开引脚时钟,串口时钟
    RS232DRIVER_CLK_CONFIG;
    RS232DRIVER_RX_GPIO_CLK_CONFIG;
    RS232DRIVER_TX_GPIO_CLK_CONFIG;

    //配置收发引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = RS232DRIVER_TX_PIN;
    GPIO_Init(RS232DRIVER_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RS232DRIVER_RX_PIN;
    GPIO_Init(RS232DRIVER_RX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(RS232DRIVER_TX_GPIO_PORT, RS232DRIVER_TX_PinSource, RS232DRIVER_GPIO_AF);
    GPIO_PinAFConfig(RS232DRIVER_RX_GPIO_PORT, RS232DRIVER_RX_PinSource, RS232DRIVER_GPIO_AF);

    // 配置USART
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_BaudRate = RS232DRIVER_USARTx_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(RS232DRIVER_USARTx, &USART_InitStructure);
    {
        uint16_t mantissa;
        uint16_t fraction;

        float temp = (float) (45 * 1000000) / (RS232DRIVER_USARTx_BAUD * 16);
        mantissa = (uint16_t) temp;
        fraction = (uint16_t) ((temp - mantissa) * 16);
        mantissa <<= 4;
        mantissa += fraction;
        RS232DRIVER_USARTx->BRR = mantissa;
    }

    // 配置中断向量管理器的USARTx_IRQn的中断
    NVIC_InitStructure.NVIC_IRQChannel = RS232DRIVER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RS232DRIVER_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    g_spinValveDriverQueue = xQueueCreate( 1,  RS232DRIVER_MAX_RECV_FRAME_LEN);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //使能接收中断
    USART_Cmd(USART2, ENABLE);
}

/**
 * @brief 重置回应包消息
 */
void SpinValveDriver_ClearResp()
{
        xQueueReset(g_spinValveDriverQueue);
}

/**
 * @brief 接收解析回应包消息
 */
//Bool SpinValveDriver_WaitResp(SpinValveResp* resp, Uint16 timeoutMS)
//{
//    Bool   waitResp = FALSE;
//
//    if( pdTRUE == xQueueReceive(g_spinValveDriverQueue, resp, portTICK_RATE_MS * timeoutMS ) )
//    {
//        waitResp = TRUE;
//    }
//
//    return waitResp;
//}
Bool SpinValveDriver_WaitResp(SpinValveResp* resp, Uint8* data, Uint16 timeoutMS)
{
    Uint8 respBuffer[RS232DRIVER_RESP_FRAME_LEN] = {0};
    Bool   waitResp = FALSE;

    if( pdTRUE == xQueueReceive(g_spinValveDriverQueue, respBuffer, portTICK_RATE_MS * timeoutMS ) )
    {
        if((respBuffer[0] == RS232DRIVER_FRAME_HEAD) &&
            (respBuffer[RS232DRIVER_RESP_FRAME_LEN - 1] == RS232DRIVER_FRAME_TAIL))
        {
            *resp = (SpinValveResp)respBuffer[2];
            memcpy(data, &respBuffer[3], 2*sizeof(Uint8));
        }
        else
        {
            *resp = FrameError;
        }

        waitResp = TRUE;
    }
    //TRACE_INFO("\nWait Resp: ");
    //SpinValveDriver_PrintData(respBuffer, RS232DRIVER_RESP_FRAME_LEN);

    return waitResp;
}

/**
 * @brief 发送数据到消息邮箱
 */
//static void SpinValveDriver_PostResp(SpinValveResp resp)
//{
//    xQueueSendFromISR(g_spinValveDriverQueue, ( const void * )&resp, pdFALSE);
//}
static void SpinValveDriver_PostRespData(Uint8* respData)
{
    xQueueSendFromISR(g_spinValveDriverQueue, ( const Uint8* )respData, pdFALSE);
}

/**
 * @brief 发送命令
 */
Bool SpinValveDriver_SendCmd(Uint8 cmd, Uint8* paramData, Uint8 paramLen)
{
    Uint8 cmdData[RS232DRIVER_CMD_FRAME_LEN + 2] = {0};
    Uint16 checkSum = 0;
    Uint8 len = 0;
    Uint8 i = 0;

    if(paramLen != RS232DRIVER_CMD_PARAM_LEN)
    {
        TRACE_ERROR("\nSpin valve driver send cmd param len error.");
        return FALSE;
    }

    SpinValveDriver_ClearResp();

    cmdData[len++] = RS232DRIVER_FRAME_HEAD;
    cmdData[len++] = s_addr;
    cmdData[len++] = cmd;
    for(i = 0; i < RS232DRIVER_CMD_PARAM_LEN; i++)
    {
        cmdData[len++] = *paramData++;
    }
    cmdData[len++] = RS232DRIVER_FRAME_TAIL;

    for(i = 0; i < RS232DRIVER_CMD_FRAME_LEN; i++)
    {
        checkSum += cmdData[i];
    }
    memcpy(&cmdData[RS232DRIVER_CMD_FRAME_LEN], &checkSum, sizeof(Uint16));

    TRACE_INFO("\nSend Cmd: ");
    SpinValveDriver_PrintData(cmdData, RS232DRIVER_CMD_FRAME_LEN + 2);
    if(sizeof(cmdData) == SpinValveDriver_WriteData(cmdData, sizeof(cmdData)))
    {
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\nSpin valve driver write data fail.");
        return FALSE;
    }
}

/**
 * @brief 发送工厂命令
 */
Bool SpinValveDriver_SendFactoryCmd(Uint8 cmd, Uint8* pwdData, Uint8 pwdLen, Uint8* paramData, Uint8 paramLen)
{
    Uint8 cmdData[RS232DRIVER_FACTORY_CMD_FRAME_LEN + 2] = {0};
    Uint16 checkSum = 0;
    Uint8 len = 0;
    Uint8 i = 0;

    if((pwdLen != RS232DRIVER_FACTORY_CMD_PWD_LEN) || (paramLen != RS232DRIVER_FACTORY_CMD_PARAM_LEN))
    {
        TRACE_ERROR("\nSpin valve driver send cmd param len error.");
        return FALSE;
    }

    cmdData[len++] = RS232DRIVER_FRAME_HEAD;
    cmdData[len++] = s_addr;
    cmdData[len++] = cmd;
    for(i = 0; i < RS232DRIVER_FACTORY_CMD_PWD_LEN; i++)
    {
        cmdData[len++] = *pwdData++;
    }
    for(i = 0; i < RS232DRIVER_FACTORY_CMD_PARAM_LEN; i++)
    {
        cmdData[len++] = *paramData++;
    }
    cmdData[len++] = RS232DRIVER_FRAME_TAIL;

    for(Uint8 i = 0; i < RS232DRIVER_FACTORY_CMD_FRAME_LEN; i++)
    {
        checkSum += cmdData[i];
    }
    memcpy(&cmdData[RS232DRIVER_FACTORY_CMD_FRAME_LEN], &checkSum, sizeof(Uint16));

    if(sizeof(cmdData) == SpinValveDriver_WriteData(cmdData, sizeof(cmdData)))
    {
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\nSpin valve driver write data fail.");
        return FALSE;
    }
}

//static SpinValveResp SpinValveDriver_AnalyzeResponse(Uint8* respData, Uint8 respLen)
//{
//    SpinValveResp resp = UnknowError;
//
//    if(respLen != 8)
//    {
//        resp = FrameError;
//    }
//    else
//    {
//        resp = (SpinValveResp)respData[2];
//    }
//
//    SpinValveDriver_PostResp(resp);
//
//    return resp;
//}

/**
 * @brief 接收数据状态机处理(中断调用)
 */
static void SpinValveDriver_ReceiveDataHandle(uint8_t data)
{
    static uint8_t s_RxState = RS232DRIVER_RX_STATE_IDLE;
    static uint8_t s_RxBufferLen = 0;
    static uint16_t s_RxCheckSum = 0;   //校验和
    static uint8_t s_RxCheckBuffer[2] = {0};  //校验和接收区
    uint16_t check = 0;

    switch(s_RxState)
    {
        /// ******** 空闲状态 ********
        case RS232DRIVER_RX_STATE_IDLE:
            if(RS232DRIVER_FRAME_HEAD == data) // 如果是帧头
            {
                //改变状态机状态
                s_RxState = RS232DRIVER_RX_STATE_GOT_FRAME_HEAD;

                //保存数据
                s_RxBuffer[s_RxBufferLen++] = data;
            }
            //其他数据，直接丢弃
            break;

        /// ******** 已经找到帧头 ********
        case RS232DRIVER_RX_STATE_GOT_FRAME_HEAD:
            // 如果检测到帧尾
            if(RS232DRIVER_FRAME_TAIL == data)
            {
                // 保存数据
                s_RxBuffer[s_RxBufferLen++] = data;
                s_RxCheckSum += data;

                s_RxState = RX232DRIVER_RX_STATE_WAIT_CHECK_LOW;
            }
            // 接收数据正常
            else
            {
                if(s_RxBufferLen >= RS232DRIVER_MAX_RECV_FRAME_LEN) // 如果数据超长
                {
                    //保存数据
                    s_RxBuffer[s_RxBufferLen++] = data;
                    ///s_RxBuffer[RS232DRIVER_RESP_CODE_INDEX] = (uint8_t)FrameError;

                    // 发送回应数据
                    SpinValveDriver_PostRespData(s_RxBuffer);

                    //改变状态机状态，恢复初始状态
                    s_RxState = RS232DRIVER_RX_STATE_IDLE;
                    s_RxBufferLen = 0;
                    s_RxCheckSum = 0;
                    memset((uint8_t*)s_RxBuffer, 0, sizeof(s_RxBuffer));
                }
                else
                {
                    //保存数据
                    s_RxBuffer[s_RxBufferLen++] = data;
                    s_RxCheckSum += data;
                }
            }
            break;

        /// ******** 接收校验和低字节 ********
        case RX232DRIVER_RX_STATE_WAIT_CHECK_LOW:
        {
            s_RxCheckBuffer[0] = data;

            //改变状态机状态
            s_RxState = RX232DRIVER_RX_STATE_WAIT_CHECK_HIGH;
        }
            break;

        /// ******** 接收校验和高字节 ********
        case RX232DRIVER_RX_STATE_WAIT_CHECK_HIGH:
        {
            s_RxCheckBuffer[1] = data;

            memcpy(&check, s_RxCheckBuffer, sizeof(s_RxCheckBuffer));

            if(s_RxCheckSum != check) //校验和错误
            {
                ///s_RxBuffer[RS232DRIVER_RESP_CODE_INDEX] = (uint8_t)FrameError;
            }

            // 发送回应数据
            SpinValveDriver_PostRespData(s_RxBuffer);

            // 改变状态机状态，恢复初始状态
            s_RxState = RS232DRIVER_RX_STATE_IDLE;
            s_RxBufferLen = 0;
            s_RxCheckSum = 0;
            memset((uint8_t*)s_RxBuffer, 0, sizeof(s_RxBuffer));
        }
            break;

        default :
            break;
    }
}

/**
 * @brief 发送数据(外部调用)
 */
uint32_t SpinValveDriver_WriteData(uint8_t *pTx_Buff, uint8_t len)
{
    uint32_t i = 0;
    uint32_t send_count = 0;

    // 参数检查
    if ((NULL == pTx_Buff) || (0 == len))
    {
        return 0;
    }

    while (send_count < len)
    {
        // 进入临界区
        taskENTER_CRITICAL();
        for (i = send_count; i < len; i++)
        {
            // 如果缓冲区已满
            if (s_TxBufferLen >= RS232DRIVER_TX_BUFF_SIZE)
            {
                break;
            }

            // 写入缓冲区,修改缓冲区尾指针
            s_TxBuffer[s_TxBufferTail] = pTx_Buff[i];
            s_TxBufferLen++;
            s_TxBufferTail++;
            if (s_TxBufferTail >= RS232DRIVER_TX_BUFF_SIZE)
            {
                s_TxBufferTail = 0;
            }
            send_count++;
        }
        // 退出临界区
        taskEXIT_CRITICAL();

        // 使能发送中断
        USART_ITConfig(RS232DRIVER_USARTx, USART_IT_TXE, ENABLE);
    }

    return send_count;
}

/**
 * @brief 从本地发送缓存读出写入到串口发送缓冲区(中断调用)
 */
static Bool SpinValveDriver_ReadTxBuffer(uint8_t *pByte)
{
    // 参数检查
    if (NULL == pByte)
    {
        return FALSE;
    }

    // 如果可发送数据长度为0
    if (0 == s_TxBufferLen)
    {
        return FALSE;
    }

    // 读取数据,修改缓冲区头指针
    *pByte = s_TxBuffer[s_TxBufferHead];
    s_TxBufferLen--;
    s_TxBufferHead++;
    if (s_TxBufferHead >= RS232DRIVER_TX_BUFF_SIZE)
    {
        s_TxBufferHead = 0;
    }

    return TRUE;
}

/**
 * @brief 串口收发中断
 */
void RS232DRIVER_UART_IRQHANDLER(void)
{
    static uint8_t s_byte_data = 0;

    // 接收处理
    if (USART_GetITStatus(RS232DRIVER_USARTx, USART_IT_RXNE) != RESET)
    {
        // 读取数据
        s_byte_data = USART_ReceiveData(RS232DRIVER_USARTx);

        // 对数据进行判断，对状态进行处理
        SpinValveDriver_ReceiveDataHandle(s_byte_data);

        /* Clear the USART Receive interrupt */
        USART_ClearITPendingBit(RS232DRIVER_USARTx, USART_IT_RXNE);
    }

    // 发送处理
    if (USART_GetITStatus(RS232DRIVER_USARTx, USART_IT_TXE) != RESET)
    {
        /* Clear the USART transmit interrupt */
        // USART_ClearITPendingBit(RS232DRIVER_USARTx, USART_IT_TXE);   // 写操作会自动清除TXE中断标识

        // 从缓冲区中读取数据
        if(SpinValveDriver_ReadTxBuffer(&s_byte_data))
        {
            /* Write one byte to the transmit data register */
            USART_SendData(RS232DRIVER_USARTx, s_byte_data);
        }
        else //数据发送完毕
        {
            /* Disable the USART Transmit interrupt */
            USART_ITConfig(RS232DRIVER_USARTx, USART_IT_TXE, DISABLE);
        }
    }

    // 若溢出错误
    if (USART_GetFlagStatus(RS232DRIVER_USARTx, USART_FLAG_ORE) != RESET)
    {
        // 读取数据可清除溢出错误标志
        USART_ReceiveData(RS232DRIVER_USARTx);

        /* Clear the USART Receive interrupt */
        USART_ClearITPendingBit(RS232DRIVER_USARTx, USART_IT_RXNE);
    }
}

void SpinValveDriver_PrintData(Uint8* buffer, Uint8 len)
{
    for(Uint8 i = 0; i < len; i++)
    {
        Printf(" %02X", buffer[i]);
    }
    Printf("\n");
}

