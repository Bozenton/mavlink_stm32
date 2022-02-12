#include "bluetooth.h"

/* 数据包 */
// 发送数据包与接收数据包的字节长度
const unsigned short  TXPACK_BYTE_SIZE = ((TX_BOOL_NUM+7)>>3)+TX_BYTE_NUM+(TX_SHORT_NUM<<1)+(TX_INT_NUM<<2)+(TX_FLOAT_NUM<<2);
const unsigned short  RXPACK_BYTE_SIZE = ((RX_BOOL_NUM+7)>>3)+RX_BYTE_NUM+(RX_SHORT_NUM<<1)+(RX_INT_NUM<<2)+(RX_FLOAT_NUM<<2);
// 接收数据包的原数据加上包头、校验和包尾 之后的字节长度
unsigned short rx_pack_length = RXPACK_BYTE_SIZE+3;

long rxIndex=0;         // 接收计数-记录当前的数据接收进度
                        // 接收计数每次随串口的接收中断后 +1

long rdIndex=0;         // 读取计数-记录当前的数据包读取进度，读取计数会一直落后于接收计数，
                        // 当读取计数与接收计数之间距离超过一个接收数据包的长度时，会启动一次数据包的读取。
                        // 读取计数每次在读取数据包后增加 +(数据包长度)

unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE]; // 用于环形缓冲区的数组，环形缓冲区的大小可以在.h文件中定义VALUEPACK_BUFFER_SIZE

unsigned char vp_txbuff[TXPACK_BYTE_SIZE+3];    // 用于暂存发送数据的数组


/**
 * @brief 蓝牙使用的串口初始化
 * 
 */
void initBT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(BT_RCC_USART, ENABLE);
	RCC_APB2PeriphClockCmd(BT_RCC_GPIO, ENABLE);

	USART_DeInit(BT_USART);

    // USART2_TX PA2
    GPIO_InitStructure.GPIO_Pin = BT_PIN_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(BT_GPIO, &GPIO_InitStructure); // 初始化GPIOA.2

    // USART2_RX PA3
    GPIO_InitStructure.GPIO_Pin = BT_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(BT_GPIO, &GPIO_InitStructure); // 初始化GPIOA.3

    // Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = BT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BT_PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = BT_SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART 初始化设置
    USART_InitStructure.USART_BaudRate = BT_BaudRate;           // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式

    USART_Init(BT_USART, &USART_InitStructure);         // 初始化串口
    USART_ClearITPendingBit(BT_USART, USART_IT_RXNE);
    USART_ITConfig(BT_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(BT_USART, ENABLE);                        // 使能串口
	
}

// 数据包环形缓冲区计数
unsigned int vp_circle_rx_index = 0;
/**
 * @brief 串口接收中断服务函数
 * 
 * 每次接收到数据后将字节存入环形缓冲区中，从头存到尾。
 * 所谓的环形缓冲区就是当接收环形缓冲区计数大于等于缓冲
 * 区的大小时（即数据到达缓冲区的尾部时）
 * 数据会在缓冲区的头部继续存储，覆盖掉最早的数据。
 */
void USART2_IRQHandler(void)
{
	// 判断是否是BT_USART接收了数据
	if(USART_GetITStatus(BT_USART, USART_IT_RXNE)) 
	{
	    // 读取数据到缓冲区中
		vp_rxbuff[vp_circle_rx_index] = USART_ReceiveData(BT_USART);	//读取接收到的数据
	        
		// 将环形缓冲接收计数加一
		vp_circle_rx_index++;
	    // 数据到达缓冲区尾部后，转移到头部
		if(vp_circle_rx_index>=VALUEPACK_BUFFER_SIZE)
			vp_circle_rx_index=0;
	        
		// 将全局接收计数加一
		rxIndex++;
	}
	USART_ClearITPendingBit(BT_USART,USART_IT_RXNE);
}




unsigned short rdi,rdii,idl,idi,bool_index,bool_bit;    // 数据读取涉及到的变量
uint32_t  idc;                  // 变量地址
unsigned int err=0;             // 记录读取的错误字节的次数
unsigned char sum=0;            // 用于和校验
unsigned char isok;             // 存放数据包读取的结果

/**
 * @brief 从缓冲区中读取数据包
 *      尝试从缓冲区中读取数据包
 * 
 * @param rx_pack_ptr
 *      传入接收数据结构体的指针，从环形缓冲区中读取出数据包，
 *      并将各类数据存储到rx_pack_ptr指向的结构体中
 * @return 
 *      如果成功读取到数据包，则返回1，否则返回0
 */

unsigned char readValuePack(RxPack *rx_pack_ptr)
{
	isok = 0;
	// 确保读取计数和接收计数之间的距离小于2个数据包的长度
	while(rdIndex<(rxIndex-((rx_pack_length)*2)))
        rdIndex += rx_pack_length;	
	
	// 如果读取计数落后于接收计数超过 1个 数据包的长度，则尝试读取
	while(rdIndex<=(rxIndex-rx_pack_length))
	{
		rdi = rdIndex % VALUEPACK_BUFFER_SIZE;
		rdii=rdi+1;
		if( vp_rxbuff[rdi]==PACK_HEAD) // 比较包头
		{
			if(vp_rxbuff[(rdi+RXPACK_BYTE_SIZE+2)%VALUEPACK_BUFFER_SIZE]==PACK_TAIL) // 比较包尾 确定包尾后，再计算校验和
			{
				//  计算校验和
				sum=0;
			      for(short s=0;s<RXPACK_BYTE_SIZE;s++)
				{
					rdi++;
					if(rdi>=VALUEPACK_BUFFER_SIZE)
					  rdi -= VALUEPACK_BUFFER_SIZE;
					sum += vp_rxbuff[rdi];
				}	
						rdi++;
					if(rdi>=VALUEPACK_BUFFER_SIZE)
					  rdi -= VALUEPACK_BUFFER_SIZE;
					
                                if(sum==vp_rxbuff[rdi]) // 校验和正确，则开始将缓冲区中的数据读取出来
				{
					//  提取数据包数据 一共有五步， bool byte short int float
					// 1. bool
					#if  RX_BOOL_NUM>0
						
					  idc = (uint32_t)rx_pack_ptr->bools;
					  idl = (RX_BOOL_NUM+7)>>3;
					
					bool_bit = 0;
					for(bool_index=0;bool_index<RX_BOOL_NUM;bool_index++)
					{
						*((unsigned char *)(idc+bool_index)) = (vp_rxbuff[rdii]&(0x01<<bool_bit))?1:0;
						bool_bit++;
						if(bool_bit>=8)
						{
						  bool_bit = 0;
							rdii ++;
						}
					}
					if(bool_bit)
						rdii ++;
					
				        #endif
					// 2.byte
					#if RX_BYTE_NUM>0
						idc = (uint32_t)(rx_pack_ptr->bytes);
					  idl = RX_BYTE_NUM;
					  for(idi=0;idi<idl;idi++)
					  {
					    if(rdii>=VALUEPACK_BUFFER_SIZE)
					      rdii -= VALUEPACK_BUFFER_SIZE;
					    (*((unsigned char *)idc))= vp_rxbuff[rdii];
							rdii++;
							idc++;
					  }
				        #endif
					// 3.short
					#if RX_SHORT_NUM>0
						idc = (uint32_t)(rx_pack_ptr->shorts);
					  idl = RX_SHORT_NUM<<1;
					  for(idi=0;idi<idl;idi++)
					  {
					    if(rdii>=VALUEPACK_BUFFER_SIZE)
					      rdii -= VALUEPACK_BUFFER_SIZE;
					    (*((unsigned char *)idc))= vp_rxbuff[rdii];
							rdii++;
							idc++;
					  }
				        #endif
					// 4.int
					#if RX_INT_NUM>0
						idc = (uint32_t)(&(rx_pack_ptr->integers[0]));
					  idl = RX_INT_NUM<<2;
					  for(idi=0;idi<idl;idi++)
					  {
					    if(rdii>=VALUEPACK_BUFFER_SIZE)
					      rdii -= VALUEPACK_BUFFER_SIZE;
					    (*((unsigned char *)idc))= vp_rxbuff[rdii];
							rdii++;
							idc++;
					  }
				        #endif
					// 5.float
					#if RX_FLOAT_NUM>0
						idc = (uint32_t)(&(rx_pack_ptr->floats[0]));
					  idl = RX_FLOAT_NUM<<2;
					  for(idi=0;idi<idl;idi++)
					  {
					    if(rdii>=VALUEPACK_BUFFER_SIZE)
					      rdii -= VALUEPACK_BUFFER_SIZE;
					    (*((unsigned char *)idc))= vp_rxbuff[rdii];
							rdii++;
							idc++;
					  }
				  #endif
				      
				        // 更新读取计数
					rdIndex+=rx_pack_length;
					isok = 1;
				}else
				{ 
				// 校验值错误 则 err+1 且 更新读取计数
				  rdIndex++;
			          err++;
				}
			}else
			{
				// 包尾错误 则 err+1 且 更新读取计数
				rdIndex++;
				err++;
			}		
		}else
		{ 
			// 包头错误 则 err+1 且 更新读取计数
			rdIndex++;
			err++;
		}		
	}
	return isok;
}



/**
 * @brief 发送数据包
 * 
 * @param p 传入指针
 * @param length 字节长度
 */
void sendBuffer(unsigned char *p,unsigned short length)
{
	  for(int i=0;i<length;i++)
   { 
      USART_SendData(BT_USART, *p++); 
      while(USART_GetFlagStatus(BT_USART, USART_FLAG_TXE) == RESET) 
      {} 
    }
}

// 数据包发送涉及的变量
unsigned short loop;
unsigned char valuepack_tx_bit_index;
unsigned char valuepack_tx_index;

/**
 * @brief 将发送数据结构体中的变量打包，并发送出去
 * 
 * @param tx_pack_ptr 待发送数据包的指针 
 */
void sendValuePack(TxPack *tx_pack_ptr)
{
  int i;
  vp_txbuff[0]=0xa5;
  sum=0;
  //  由于结构体中不同类型的变量在内存空间的排布不是严格对齐的，中间嵌有无效字节，因此需要特殊处理	 
  valuepack_tx_bit_index = 0;
  valuepack_tx_index = 1;
	
	#if TX_BOOL_NUM>0 	  
	  for(loop=0;loop<TX_BOOL_NUM;loop++)
	  {
		  if(tx_pack_ptr->bools[loop])
			vp_txbuff[valuepack_tx_index] |= 0x01<<valuepack_tx_bit_index;
		  else
			vp_txbuff[valuepack_tx_index] &= ~(0x01<<valuepack_tx_bit_index);
		  valuepack_tx_bit_index++;
	  
		  if(valuepack_tx_bit_index>=8)
		  {
			  valuepack_tx_bit_index = 0;
			  valuepack_tx_index++;
		  }	
	  }
	  if(valuepack_tx_bit_index!=0)
		  valuepack_tx_index++;			
	#endif
	#if TX_BYTE_NUM>0 
	  
	  for(loop=0;loop<TX_BYTE_NUM;loop++)
	  {
		  vp_txbuff[valuepack_tx_index] = tx_pack_ptr->bytes[loop];
		  valuepack_tx_index++;			
	  }
	#endif
	
	#if TX_SHORT_NUM>0 
	  for(loop=0;loop<TX_SHORT_NUM;loop++)
	  {
		  vp_txbuff[valuepack_tx_index] = tx_pack_ptr->shorts[loop]&0xff;
		  vp_txbuff[valuepack_tx_index+1] = tx_pack_ptr->shorts[loop]>>8;
		  valuepack_tx_index+=2;			
	  }
	#endif
		
	#if TX_INT_NUM>0   
	  for(loop=0;loop<TX_INT_NUM;loop++)
	  {
		  i = tx_pack_ptr->integers[loop];	
		  vp_txbuff[valuepack_tx_index] = i&0xff;	
		  vp_txbuff[valuepack_tx_index+1] = (i>>8)&0xff;
		  vp_txbuff[valuepack_tx_index+2] =(i>>16)&0xff;
		  vp_txbuff[valuepack_tx_index+3] = (i>>24)&0xff;
		  valuepack_tx_index+=4;			
		}
	#endif
	
	#if TX_FLOAT_NUM>0   
	  for(loop=0;loop<TX_FLOAT_NUM;loop++)
	  {
		  i = *(int *)(&(tx_pack_ptr->floats[loop]));		
		  vp_txbuff[valuepack_tx_index] = i&0xff;
		  vp_txbuff[valuepack_tx_index+1] = (i>>8)&0xff;
		  vp_txbuff[valuepack_tx_index+2] =(i>>16)&0xff;
		  vp_txbuff[valuepack_tx_index+3] = (i>>24)&0xff;
		  valuepack_tx_index+=4;				
	  }
	#endif	

	for(unsigned short d=1;d<=TXPACK_BYTE_SIZE;d++)
		sum+=vp_txbuff[d];
		
	vp_txbuff[TXPACK_BYTE_SIZE+1] = sum;
	vp_txbuff[TXPACK_BYTE_SIZE+2] = 0x5a;
	sendBuffer(vp_txbuff,TXPACK_BYTE_SIZE+3);
}


extern QueueHandle_t distanceQueue;
/**
 * @brief 蓝牙发送任务
 * 
 * @param pvParameters 
 */
void task_BtSend(void *pvParameters)
{
	#if SWITCH_BT && SWITCH_UTS
	float dis = 0;
	BaseType_t err_uts;
	#endif // SWITCH_UTS
	TxPack txpack;
	while(1)
	{
		// 初始化txpack
		txpack.floats[0] = 0; 

		#if SWITCH_BT && SWITCH_UTS
		/* ------------- 发送超声波测距数据 ------------- */
		// 取数据
		// err_uts = xQueuePeek(distanceQueue, &dis, 0);
		xQueuePeek(distanceQueue, &dis, 0);
		if(err_uts != errQUEUE_EMPTY)
		{
			txpack.floats[0] = dis;
		}
		#endif // SWITCH_UTS

		#if SWITCH_CTRL
		// float currentHeight = api.current_messages.optical_flow_rad.distance;
		// float local_ned_x = ;
		// float local_ned_y = ;
		// float local_ned_z = api.current_messages.local_position_ned.z;
		txpack.floats[1] = api.current_messages.optical_flow_rad.distance;
		txpack.floats[2] = api.current_messages.local_position_ned.x;
		txpack.floats[3] = api.current_messages.local_position_ned.y;
		txpack.floats[4] = api.current_messages.local_position_ned.z;
		// txpack.floats[5] = api.current_messages.altitude.altitude_local;
		#endif

		#if SWITCH_FSM
		txpack.bytes[0] = queryCurrentState();
		#endif

		sendValuePack(&txpack);
		delay_ms(100);
	}
}


extern RxPack rxpack;
extern SemaphoreHandle_t btRecSemphr;      // 蓝牙串口接收数据完成标志
extern SemaphoreHandle_t ctrlStartSemphr;	// 启动控制
extern SemaphoreHandle_t taskQuerySemphr;
extern SemaphoreHandle_t ctrlMutexSemphr;  // 互斥信号量，用于给控制指令流发送消息
extern QueueHandle_t ctrlQueue;      // 来自蓝牙的控制指令流

extern Autopilot_Interface api;

#if SWITCH_GIMBAL
extern Gimbal_t Gimbal;
#endif

/**
 * @brief 蓝牙接收任务
 * 
 * @param pvParameters 
 */
void task_BtReceive(void *pvParameters)
{
	#if RX_BOOL_NUM > 0
		rxpack.bools[0] = 0;
	#endif

	while(1)
	{
		if( readValuePack(&rxpack) )
		{
			xSemaphoreGive(btRecSemphr);

			#if SWITCH_CTRL
				unsigned char toggle = rxpack.bools[0];
				uint8_t ch = rxpack.bytes[0];
				if(ctrlQueue != NULL)
				{
					BaseType_t err;

					#if SWITCH_FSM // mutex锁
					xSemaphoreTake(ctrlMutexSemphr, portMAX_DELAY);
					#endif
					
					err = xQueueSend(ctrlQueue, &ch, 0);
					if(err == errQUEUE_FULL)
						printf("The ctrl queue is full");

					#if SWITCH_FSM
					xSemaphoreGive(ctrlMutexSemphr);
					#endif
				}
				if(toggle && ctrlStartSemphr != NULL)
				{
					api.targetHeight = rxpack.floats[0];
					xSemaphoreGive(ctrlStartSemphr);
				}
			#endif

			#if SWITCH_QUERY
				unsigned char query = rxpack.bools[1];
				if(query && taskQuerySemphr!=NULL)
					xSemaphoreGive(taskQuerySemphr);
			#endif

			#if SWITCH_GIMBAL
				uint8_t theta1 = rxpack.shorts[0];
				uint8_t theta2 = rxpack.shorts[1];
				// servo1Angle(theta1);
				// servo2Angle(theta2);
				Gimbal.f_setGoalAngle(&Gimbal, 0, theta1);
				Gimbal.f_setGoalAngle(&Gimbal, 1, theta2);

			#endif
			
		}
		else
			delay_ms(30);
	}
}





