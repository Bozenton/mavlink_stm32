#include "openmv.h"

uint8_t opmv_rxbuff[OPMV_RXBUFF_SIZE];
uint8_t opmvHor_rxbuff[OPMV_RXBUFF_SIZE];
extern opmvInterface omi;
extern opmvHorInterface omhi;
extern QueueHandle_t fsmEventQueue;         // 向FSM传递事件的condition

void prvOpmvType1CallBack(void);
void prvOpmvType2CallBack(void);
void prvOpmvType3CallBack(void);
void prvOpmvType4CallBack(void);

void prvOpmvHorType1CallBack(void);
void prvOpmvHorType2CallBack(void);
void prvOpmvHorType3CallBack(void);
void prvOpmvHorType4CallBack(void);


/**
 * @brief openmv所使用串口初始化
 * 
 */
void initOPMV(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(OPMV_RCC_USART, ENABLE);
    RCC_APB2PeriphClockCmd(OPMV_RCC_GPIO, ENABLE);

    USART_DeInit(OPMV_USART);

    // UART4_TX PC10 连 openmv 红线
    GPIO_InitStructure.GPIO_Pin = OPMV_PIN_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(OPMV_GPIO, &GPIO_InitStructure);

    // UART4_RX PC11 连openmv 黑线
    GPIO_InitStructure.GPIO_Pin = OPMV_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(OPMV_GPIO, &GPIO_InitStructure);

    // UART4 NVIC
    NVIC_InitStructure.NVIC_IRQChannel = OPMV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = OPMV_PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = OPMV_SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    // UART4
    USART_InitStructure.USART_BaudRate = OPMV_BaudRate;           // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(OPMV_USART, &USART_InitStructure);
    USART_ClearITPendingBit(OPMV_USART, USART_IT_RXNE);
    USART_ITConfig(OPMV_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(OPMV_USART, ENABLE);
}


void initOPMVHOR(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(OPMVHOR_RCC_USART, ENABLE);
    RCC_APB2PeriphClockCmd(OPMVHOR_RCC_GPIO_TX, ENABLE);
    RCC_APB2PeriphClockCmd(OPMVHOR_RCC_GPIO_RX, ENABLE);

    USART_DeInit(OPMVHOR_USART);

    // UART5_TX PC12 连 openmv P5
    GPIO_InitStructure.GPIO_Pin = OPMVHOR_PIN_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(OPMVHOR_GPIO_TX, &GPIO_InitStructure);

    // UART5_RX PD2 连openmv P4
    GPIO_InitStructure.GPIO_Pin = OPMVHOR_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(OPMVHOR_GPIO_RX, &GPIO_InitStructure);

    // UART5 NVIC
    NVIC_InitStructure.NVIC_IRQChannel = OPMVHOR_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = OPMVHOR_PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = OPMVHOR_SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    // UART5
    USART_InitStructure.USART_BaudRate = OPMVHOR_BaudRate;           // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(OPMVHOR_USART, &USART_InitStructure);
    USART_ClearITPendingBit(OPMVHOR_USART, USART_IT_RXNE);
    USART_ITConfig(OPMVHOR_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(OPMVHOR_USART, ENABLE);
}


uint16_t OPMV_REC_STA = 0; // 接收状态
// bit15 标记收到包头
// bit14 标记收报数据长度位
// bit13,12,11 标记接收到的数据类型
// bit10 标记数据部分接收完成
// bit7~0 对数据部分的计数
uint8_t OPMV_RXPACK_LEN = 0; // 存放接收数据包的
uint8_t OPMV_RXPACK_CHECK = 0; // 校验和

/**
 * @brief openmv所用串口的数据解析函数
 * 
 * @param recbyte 接收到的字节
 * @return uint8_t 解析状态
 *      OPMV_RXPACK_UNFINISHED      0   // 开始了但是没结束
 *      OPMV_RXPACK_FINISHED        1   // 成功完成
 *      OPMV_RXPACK_CHECK_FAILED    2   // 出现错误
 *      OPMV_RXPACK_UNKNOWN_FAILED  3   // 未知错误
 */
uint8_t prvOpmvParseChar(uint8_t recbyte)
{
    if((recbyte == OPMV_PACK_HEAD) && ((OPMV_REC_STA&OPMV_RXSTA_HEAD_BIT)==0)) // 收到包头且head位为0
    {
        OPMV_REC_STA = 0;
        OPMV_REC_STA |= OPMV_RXSTA_HEAD_BIT; // 标记已经收到包头
        OPMV_RXPACK_CHECK = 0;
        return OPMV_RXPACK_UNFINISHED;
    }
    else if( OPMV_REC_STA & OPMV_RXSTA_HEAD_BIT ) // 已经收到包头
    {
        if( (OPMV_REC_STA & OPMV_RXSTA_LEN_BIT) == 0) // 收到长度位
        {
            OPMV_REC_STA |= OPMV_RXSTA_LEN_BIT;
            OPMV_RXPACK_LEN = recbyte; // data部分的字节长度
            
            // printf("len=%d\r\n", recbyte);
        }
        else if( (OPMV_REC_STA&OPMV_RXSTA_TYPE_BITS) == 0 ) // 收到类型位
        {
            OPMV_REC_STA |= ((uint16_t)recbyte) << 11;
            // printf("id=%d\t", recbyte); printf("sta=%d\r\n", OPMV_REC_STA); 
        }
        else if( ((OPMV_REC_STA & OPMV_RXSTA_DATA_BIT)==0)&&(OPMV_RXPACK_LEN>0) ) // 收到数据部分
        {
            uint8_t idx = (uint8_t)(OPMV_REC_STA&OPMV_RXSTA_CNT_BITS);
            // if( idx < OPMV_RXPACK_LEN )
            // {
            opmv_rxbuff[idx] = recbyte;
            OPMV_RXPACK_CHECK += recbyte;
            OPMV_REC_STA ++;
            // printf("rec=%d\t", recbyte);
            // }
            if( (idx+1) >= OPMV_RXPACK_LEN ) // 数据部分已经接收完最后一个字节
            // else
            {
                OPMV_REC_STA |= OPMV_RXSTA_DATA_BIT;
                // printf("\r\n");
            }
        }
        else // 收到校验位
        {
            if( recbyte != OPMV_RXPACK_CHECK )
                return OPMV_RXPACK_CHECK_FAILED;
            return OPMV_RXPACK_FINISHED;
        }
        return OPMV_RXPACK_UNFINISHED;
    }
    else // 未知错误
        return OPMV_RXPACK_UNKNOWN_FAILED;
}


uint16_t OPMVHOR_REC_STA = 0; // 接收状态
// bit15 标记收到包头
// bit14 标记收报数据长度位
// bit13,12,11 标记接收到的数据类型
// bit10 标记数据部分接收完成
// bit7~0 对数据部分的计数
uint8_t OPMVHOR_RXPACK_LEN = 0; // 存放接收数据包的
uint8_t OPMVHOR_RXPACK_CHECK = 0; // 校验和

/**
 * @brief openmv所用串口的数据解析函数
 * 
 * @param recbyte 接收到的字节
 * @return uint8_t 解析状态
 *      OPMV_RXPACK_UNFINISHED      0   // 开始了但是没结束
 *      OPMV_RXPACK_FINISHED        1   // 成功完成
 *      OPMVHOR_RXPACK_CHECK_FAILED    2   // 出现错误
 *      OPMV_RXPACK_UNKNOWN_FAILED  3   // 未知错误
 */
uint8_t prvOpmvHorParseChar(uint8_t recbyte)
{
    if((recbyte == OPMV_PACK_HEAD) && ((OPMVHOR_REC_STA&OPMV_RXSTA_HEAD_BIT)==0)) // 收到包头且head位为0
    {
        OPMVHOR_REC_STA = 0;
        OPMVHOR_REC_STA |= OPMV_RXSTA_HEAD_BIT; // 标记已经收到包头
        OPMVHOR_RXPACK_CHECK = 0;
        return OPMV_RXPACK_UNFINISHED;
    }
    else if( OPMVHOR_REC_STA & OPMV_RXSTA_HEAD_BIT ) // 已经收到包头
    {
        if( (OPMVHOR_REC_STA & OPMV_RXSTA_LEN_BIT) == 0) // 收到长度位
        {
            OPMVHOR_REC_STA |= OPMV_RXSTA_LEN_BIT;
            OPMVHOR_RXPACK_LEN = recbyte; // data部分的字节长度
        }
        else if( (OPMVHOR_REC_STA&OPMV_RXSTA_TYPE_BITS) == 0 ) // 收到类型位
        {
            OPMVHOR_REC_STA |= ((uint16_t)recbyte) << 11;
        }
        else if( ((OPMVHOR_REC_STA & OPMV_RXSTA_DATA_BIT)==0)&&(OPMVHOR_RXPACK_LEN>0) ) // 收到数据部分
        {
            uint8_t idx = (uint8_t)(OPMVHOR_REC_STA&OPMV_RXSTA_CNT_BITS);
            opmvHor_rxbuff[idx] = recbyte;
            OPMVHOR_RXPACK_CHECK += recbyte;
            OPMVHOR_REC_STA ++;
            if( (idx+1) >= OPMVHOR_RXPACK_LEN )
            {
                OPMVHOR_REC_STA |= OPMV_RXSTA_DATA_BIT;
            }
        }
        else // 收到校验位
        {
            if( recbyte != OPMVHOR_RXPACK_CHECK )
                return OPMV_RXPACK_CHECK_FAILED;
            return OPMV_RXPACK_FINISHED;
        }
        return OPMV_RXPACK_UNFINISHED;
    }
    else // 未知错误
        return OPMV_RXPACK_UNKNOWN_FAILED;
}



extern SemaphoreHandle_t opmvRecSemphr;         // openmv串口接收数据完成标志
extern SemaphoreHandle_t opmvHorRecSemphr;      // 水平openmv串口接收数据完成标志



/**
 * @brief openmv所用串口接收中断服务函数
 * 
 */
void UART4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    if(USART_GetITStatus(OPMV_USART, USART_IT_RXNE))
    {
        uint8_t recbyte = USART_ReceiveData(OPMV_USART);
        uint8_t flag = prvOpmvParseChar(recbyte);
        // printf("f:%d\r\n", flag);

        switch(flag)
        {
            case OPMV_RXPACK_UNFINISHED: // 正在进行中
            {
                break;
            }
            case OPMV_RXPACK_FINISHED: // 接收完成
            {
                if(opmvRecSemphr != NULL)
                {
                    xSemaphoreGiveFromISR(opmvRecSemphr, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
                }
                break;
                // 这里暂且不将OPMV_REC_STA清零，因为后面的任务中要用
                // 所以记得在任务中用完后将其清零
            }
            case OPMV_RXPACK_CHECK_FAILED: // 出现错误
            {
                #if SWITCH_DEBUG
                printf("ck!\r\n");
                #endif 
                OPMV_REC_STA = 0;
                break;
            }
            case OPMV_RXPACK_UNKNOWN_FAILED: // 出现错误
            {
                #if SWITCH_DEBUG
                printf("err!\r\n");
                #endif
                OPMV_REC_STA = 0;
                break;
            }
        }
    }
    // USART_ClearITPendingBit(OPMV_USART, USART_IT_RXNE);
}



/**
 * @brief openmv所用串口接收中断服务函数
 * 
 */
void UART5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    if(USART_GetITStatus(OPMVHOR_USART, USART_IT_RXNE))
    {
        uint8_t recbyte = USART_ReceiveData(OPMVHOR_USART);
        uint8_t flag = prvOpmvHorParseChar(recbyte);
        // if(recbyte == 4)
        //     printf("rec\r\n");

        switch(flag)
        {
            case OPMV_RXPACK_UNFINISHED: // 正在进行中
            {
                break;
            }
            case OPMV_RXPACK_FINISHED: // 接收完成
            {
                if(opmvHorRecSemphr != NULL)
                {
                    xSemaphoreGiveFromISR(opmvHorRecSemphr, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
                }
                // 这里暂且不将OPMVHOR_REC_STA清零，因为后面的任务中要用
                // 所以记得在任务中用完后将其清零
                // OPMVHOR_REC_STA = 0;
                break;
            }
            case OPMV_RXPACK_CHECK_FAILED: // 出现错误
            {
                #if SWITCH_DEBUG
                // printf("ck!\r\n");
                #endif
                OPMVHOR_REC_STA = 0;
                break;
            }
            case OPMV_RXPACK_UNKNOWN_FAILED: // 出现错误
            {
                #if SWITCH_DEBUG
                // printf("err!\r\n");
                #endif
                OPMVHOR_REC_STA = 0;
                break;
            }
        }
    }
}



extern SemaphoreHandle_t opmvHandleSemphr; // openmv解析完成标志
extern SemaphoreHandle_t opmvHorHandleSemphr;  // 水平openmv解析完成标志


/**
 * @brief openmv数据解析任务
 * 
 * @param pvParameters 
 */
void task_opmvHandle(void *pvParameters)
{
    BaseType_t err = pdFALSE;
    uint8_t opmvType = 0;
    while(1)
    {
        if(opmvRecSemphr != NULL)
        {
            err = xSemaphoreTake(opmvRecSemphr, portMAX_DELAY);
            if(err == pdTRUE)
            {
                err = pdFALSE;
                opmvType = (OPMV_REC_STA&OPMV_RXSTA_TYPE_BITS) >> 11;
                // printf("the sta = %d\r\n", OPMV_REC_STA);
                switch(opmvType)
                {
                    case OPMV_REC_ID_1:
                    {
                        prvOpmvType1CallBack();
                        if(opmvHandleSemphr != NULL)
                            xSemaphoreGive(opmvHandleSemphr);
                        break;
                    }
                    case OPMV_REC_ID_2:
                    {
                        prvOpmvType2CallBack();
                        if(opmvHandleSemphr != NULL)
                            xSemaphoreGive(opmvHandleSemphr);
                        break;
                    }
                    case OPMV_REC_ID_3:
                    {
                        prvOpmvType3CallBack();
                        if(opmvHandleSemphr != NULL)
                            xSemaphoreGive(opmvHandleSemphr);
                        break;
                    }
                    case OPMV_REC_ID_4:
                    {
                        prvOpmvType4CallBack();
                        if(opmvHandleSemphr != NULL)
                            xSemaphoreGive(opmvHandleSemphr);
                        break;
                    }
                    default:
                        break;
                }
                OPMV_REC_STA = 0;
            }
            else
                continue;
        }
    }
}


/**
 * @brief 水平openmv数据解析任务
 * 
 * @param pvParameters 
 */
void task_opmvHorHandle(void *pvParameters)
{
    BaseType_t err = pdFALSE;
    uint8_t opmvType = 0;
    while(1)
    {
        if(opmvHorRecSemphr != NULL)
        {
            err = xSemaphoreTake(opmvHorRecSemphr, portMAX_DELAY);
            if(err == pdTRUE)
            {
                err = pdFALSE;
                opmvType = (OPMVHOR_REC_STA&OPMV_RXSTA_TYPE_BITS) >> 11;
                // printf("the sta = %d\r\n", OPMVHOR_REC_STA);
                // printf("%d \r\n", opmvType);
                switch(opmvType)
                {
                    case OPMVHOR_REC_ID_1:
                    {
                        prvOpmvHorType1CallBack();
                        if(opmvHorHandleSemphr != NULL)
                            xSemaphoreGive(opmvHorHandleSemphr);
                        break;
                    }
                    case OPMVHOR_REC_ID_2:
                    {
                        prvOpmvHorType2CallBack();
                        if(opmvHorHandleSemphr != NULL)
                            xSemaphoreGive(opmvHorHandleSemphr);
                        break;
                    }
                    case OPMVHOR_REC_ID_3:
                    {
                        prvOpmvHorType3CallBack();
                        if(opmvHorHandleSemphr != NULL)
                            xSemaphoreGive(opmvHorHandleSemphr);
                        break;
                    }
                    case OPMVHOR_REC_ID_4:
                    {
                        prvOpmvHorType4CallBack();
                        if(opmvHorHandleSemphr != NULL)
                            xSemaphoreGive(opmvHorHandleSemphr);
                        break;
                    }
                    default:
                        break;
                }
                OPMVHOR_REC_STA = 0;
            }
            else
                continue;
        }
    }
}

/* ----------------------------- 发送数据 ------------------------ */

/**
 * @brief 将数据发送到openmv
 * 
 * @param p 
 * @param length 
 */
void opmvSend(uint8_t *p, uint16_t length)
{
    for(uint16_t i=0;i<length;i++)
    {
        USART_SendData(OPMV_USART, *p++);
        while(USART_GetFlagStatus(OPMV_USART, USART_FLAG_TXE)==RESET)
        {}
    }
}

/**
 * @brief 向openmv发送指令
 * 
 * @param sendID 
 */
void opmvSendPakage(OPMV_SEND_ID sendID)
{
    uint8_t buff[OPMV_TXBUFF_SIZE] = {0};
    buff[0] = (uint8_t)OPMV_PACK_HEAD;  // 包头
    buff[1] = 0;                        // 数据部分长度
    buff[2] = sendID;                   // ID
    buff[3] = 0;
    buff[4] = 0;                        
    uint16_t len = 5;
    opmvSend(buff, len);
}

/**
 * @brief 将数据发送到水平openmv
 * 
 * @param p 
 * @param length 
 */
void opmvHorSend(uint8_t *p, uint16_t length)
{
    for(uint16_t i=0;i<length;i++)
    {
        USART_SendData(OPMVHOR_USART, *p++);
        while(USART_GetFlagStatus(OPMVHOR_USART, USART_FLAG_TXE)==RESET)
        {}
    }
}

/**
 * @brief 向水平openmv发送指令
 * 
 * @param sendID 
 */
void opmvHorSendPakage(OPMVHOR_SEND_ID sendID)
{
    uint8_t buff[OPMV_TXBUFF_SIZE] = {0};
    buff[0] = (uint8_t)OPMV_PACK_HEAD;  // 包头
    buff[1] = 0;                        // 数据部分长度
    buff[2] = sendID;                   // ID
    buff[3] = 0;
    buff[4] = 0;                        
    uint16_t len = 5;
    opmvHorSend(buff, len);
}



/* ---------------------------- openmv 数据处理回调函数 ---------------------------- */

/**
 * @brief openmv类型1数据处理
 * 
 */
void prvOpmvType1CallBack(void)
{
    // #if SWITCH_DEBUG
    // printf("ID of pkg from opmv is: 1\r\n");
    // printf("%d\r\n", OPMV_REC_STA);
    // printf("len=%d\r\n", OPMV_RXPACK_LEN);
    // #endif
    // uint32_t xy[2] = {0};
    // uint32_t idc;
    // int i=0;

    // idc = (uint32_t)(&xy);
    // for(i=0; i<OPMV_RXPACK_LEN; i++)
    // {
    //     (*( (uint8_t*)idc )) = opmv_rxbuff[i];
    //     idc++;
    // }
    // omi.pkg1_x = xy[0];
    // omi.pkg1_y = xy[1];
    // #if SWITCH_DEBUG
    // printf("x=%d, y=%d \r\n", xy[0], xy[1]);
    // #endif
}

/**
 * @brief openmv类型2数据处理
 * 
 */
void prvOpmvType2CallBack(void)
{
    uint32_t idc; // 地址
    float temp[3] = {0};
    int i = 0;

    idc = (uint32_t)(&temp);
    for(i=0; i<OPMV_RXPACK_LEN; i++)
    {
        (*( (uint8_t*)idc )) = opmv_rxbuff[i];
        idc++;
    }
    // temp[1] 图像中的x
    // temp[2] 图像中的y
    omi.pkg2_flag = temp[0];
    omi.pkg2_x = temp[1];
    omi.pkg2_y = temp[2];
}

/**
 * @brief openmv类型3数据处理
 * 
 */
void prvOpmvType3CallBack(void)
{
    // #if SWITCH_DEBUG
    // printf("ID of pkg from opmv is: 3\r\n");
    // printf("%d\r\n", OPMV_REC_STA);
    // printf("len=%d\r\n", OPMV_RXPACK_LEN);
    // #endif
}


/**
 * @brief openmv类型4数据处理
 * 
 */
void prvOpmvType4CallBack(void)
{
    uint32_t idc; // 地址
    float temp[2] = {0};
    int i = 0;

    idc = (uint32_t)(&temp);
    for(i=0; i<OPMV_RXPACK_LEN; i++)
    {
        (*( (uint8_t*)idc )) = opmv_rxbuff[i];
        idc++;
    }
    // omhi.pkg4_recognition_result = temp[0];
    // omhi.pkg4_recognition_number = temp[1];
    omi.pkg4_recognition_result = temp[0];
    omi.pkg4_recognition_number = temp[1];
    // printf("openmv 1 result = %f, num = %f \r\n", omi.pkg4_recognition_result, omi.pkg4_recognition_number);
}

/* ---------------------------- 水平openmv 数据处理回调函数 ---------------------------- */



/**
 * @brief openmv类型1数据处理
 * 
 */
void prvOpmvHorType1CallBack(void)
{
    // #if SWITCH_DEBUG
    // printf("ID of pkg from opmv is: 1\r\n");
    // printf("%d\r\n", OPMVHOR_REC_STA);
    // printf("len=%d\r\n", OPMVHOR_RXPACK_LEN);
    // #endif
    // uint32_t xy[2] = {0};
    // uint32_t idc;
    // int i=0;

    // idc = (uint32_t)(&xy);
    // for(i=0; i<OPMVHOR_RXPACK_LEN; i++)
    // {
    //     (*( (uint8_t*)idc )) = opmv_rxbuff[i];
    //     idc++;
    // }
    // omhi.pkg1_xx = xy[0];
    // omhi.pkg1_yy = xy[1];
    // #if SWITCH_DEBUG
    // printf("x=%d, y=%d \r\n", xy[0], xy[1]);
    // #endif
}

/**
 * @brief openmv类型2数据处理
 * 
 */
void prvOpmvHorType2CallBack(void)
{

}

/**
 * @brief openmv类型3数据处理
 * 
 */
void prvOpmvHorType3CallBack(void)
{

}

void prvOpmvHorType4CallBack(void)
{
    uint32_t idc; // 地址
    float temp[2] = {0};
    int i = 0;

    idc = (uint32_t)(&temp);
    for(i=0; i<OPMVHOR_RXPACK_LEN; i++)
    {
        (*( (uint8_t*)idc )) = opmvHor_rxbuff[i];
        idc++;
    }
    omhi.pkg4_recognition_result = temp[0];
    omhi.pkg4_recognition_number = temp[1];
    // printf("openmv 2 result = %f, num = %f \r\n", omhi.pkg4_recognition_result, omhi.pkg4_recognition_number);
}

