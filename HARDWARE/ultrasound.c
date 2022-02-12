#include "ultrasound.h"

uint16_t TIM4CH3_CAPTURE_STA = 0;
/**
 * 0x8000:      成功捕获
 * 0x4000:      已经捕获到高电平了
 * bit1~bit14:  溢出次数
 */
uint16_t TIM4CH3_CAPTURE_VAL;
uint8_t TRIGGER_TOUCHED = 0; // 标记是否已经触发一次测距
/**
 * @brief 定时器3初始化
 *      用于产生单脉冲
 */
void TIM3Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 使能定时器3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   // 使能GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    // 使能复用功能模块时钟

    // 重映射
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); // TIM3的CH2完全重映射到PC7

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    // 初始化TIM3
    TIM_TimeBaseStructure.TIM_Period = 40-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1; // f=72MHz/72=1MHz, T = 1us
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    // 不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 

    // 设置为单脉冲模式
    TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC2Ref);

    // 初始化PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // 在向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道2为有效电平，否则为无效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 20; // TIMx->CCR2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_Cmd(TIM3, DISABLE);
    
}


/**
 * @brief 定时器4初始化
 *          用于捕获超声波模块
 */ 
void TIM4Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM4_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // GPIOB8 TIM4_CH3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 下拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);

    // 初始化TIM4
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 初始化TIM4输入捕获参数
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频
    TIM4_ICInitStructure.TIM_ICFilter = 0X00;
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig( TIM4, TIM_IT_Update | TIM_IT_CC3, ENABLE ); //允许更新中断 ,允许CC3IE捕获中断	
                        // TIM_IT_Update: 计数器向上溢出/向下溢出，计数器初始化(通过软件或者内部/外部触发)
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief 超声波模块初始化
 * 
 */
void ultrasoundInit(void)
{
    TIM3Init();
    TIM4Init();
    TIM4CH3_CAPTURE_STA = 0;
    TRIGGER_TOUCHED = 0;
}



void TIM4_IRQHandler(void)
{
    if( (TIM4CH3_CAPTURE_STA&REC_FINISHED) == 0 ) // 还未捕捉成功
    {
        if( TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET )
        {
            if( TIM4CH3_CAPTURE_STA & REC_HIGH ) // 已经捕捉到上升沿了
            {
                if( (TIM4CH3_CAPTURE_STA&REC_OVERFLOW) == REC_OVERFLOW ) // 溢出次数已经达到上限了，不要再溢出啦
                {
                    TIM4CH3_CAPTURE_STA |= REC_FINISHED; // 停止
                    TIM4CH3_CAPTURE_VAL = 0xffff;
                }
                else
                    TIM4CH3_CAPTURE_STA ++;
            }
        }
        if( TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET ) // 捕捉到上升沿或者下降沿
        {
            if(TIM4CH3_CAPTURE_STA & REC_HIGH) // 已经捕捉到了上升沿，说明这次捕捉到的是下降沿
            {
                TIM4CH3_CAPTURE_STA |= REC_FINISHED;
                TIM4CH3_CAPTURE_VAL = TIM_GetCapture3(TIM4);
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising);
            }
            else // 捕捉到上升沿
            {
                TIM4CH3_CAPTURE_STA = 0;
                TIM4CH3_CAPTURE_VAL = 0;
                TIM_SetCounter(TIM4, 0);
                TIM4CH3_CAPTURE_STA |= REC_HIGH;
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);
            }
        }
    }
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3|TIM_IT_Update);
}



/**
 * @brief 进行一次超声波测距
 * 
 */
void ultrasoundTrigger(void)
{ 
    // printf("trigger: touched = %d\r\n", TRIGGER_TOUCHED);
    if( (TRIGGER_TOUCHED==0) && 
        ((TIM4CH3_CAPTURE_STA&REC_HIGH)==0) &&
        ((TIM4CH3_CAPTURE_STA&REC_FINISHED)==0) ) // 没有触发测距，超声波返回
    {
        // taskENTER_CRITICAL(); 

        TIM_Cmd(TIM3, ENABLE);
        TRIGGER_TOUCHED = 1;

        // taskEXIT_CRITICAL();
    }
}

/**
 * @brief 
 * 
 * @param intervalTime 两次测量的时间间隔(ms)
 * @return float 测量得到的距离(m)，若为-1说明测量失败
 */
float ultrasoundGetDis(void)
{
    uint32_t temp;
    float distance = -1;
    // printf("ceju sta %d \r\n", TIM4CH3_CAPTURE_STA);
    if( TIM4CH3_CAPTURE_STA & REC_FINISHED )
    {
        temp = (TIM4CH3_CAPTURE_STA & REC_OVERFLOW) * 0xffff;
        temp += TIM4CH3_CAPTURE_VAL;

        distance = ((float)temp) / 2.0 * 0.034 / 100.0; // 单位cm

        // 为了下一次的测量，将这两个变量置零
        TIM4CH3_CAPTURE_STA = 0; 
        TRIGGER_TOUCHED = 0;

        // 延时一段时间再进行下一次测量
        // delay_xms(intervalTime); 
        // delay_ms(intervalTime); // Freertos中的延时
    }
    return distance;
}

#define RINGBUFFLEN 3
extern QueueHandle_t distanceQueue;
/**
 * @brief 超声波测距任务
 * 
 * @param pvParameters 
 */
void task_ultrasound(void *pvParameters)
{
    float dis;
    float dis_avg = 0;
    float ringBuff[RINGBUFFLEN] = {0};
    static uint8_t ptr = 0;
	static uint8_t cnt = 0;

    int i = 0;

    while(1)
    {
        ultrasoundTrigger();
        delay_ms(10);
        dis = ultrasoundGetDis();
        if(dis > 0 && (distanceQueue!=NULL))
        {
            // 存入环形缓冲区，进行均值滤波
            if(cnt < RINGBUFFLEN)
            {
                cnt ++;
                xQueueOverwrite(distanceQueue, &dis);
            }
            else
            {
                dis_avg = 0;
                // 进行均值滤波
                for(i=0; i<RINGBUFFLEN; i++)
                {
                    dis_avg += ringBuff[i];
                }
                dis_avg = dis_avg / ((float)RINGBUFFLEN);
                xQueueOverwrite(distanceQueue, &dis_avg);
            }

            // 更新环形缓冲区的索引
            if(ptr >= RINGBUFFLEN)
                ptr = 0;
            ringBuff[ptr] = dis;
            ptr++;
        }
        

        
        
        // #if SWITCH_BT && SWITCH_UTS
		// float dis;
		// dis = ultrasoundGetDis(250);
        // // ---------- 将测距结果通过蓝牙发送到手机 ----------
        // if(dis > 0 && (distanceQueue!=NULL)) // 测距成功
        // {
        //     BaseType_t err = xQueueSend(distanceQueue, &dis, 10); // 等10个节拍，若仍然堵塞，则不等
		// 	if( err == errQUEUE_FULL)
		// 		printf("The distance queue is full \r\n");
        // }
        // #endif
    }
}



