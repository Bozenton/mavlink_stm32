/**
 * @file finiteStateMachine.c
 * @author your name (you@domain.com)
 * @brief 有限状态机
 * @version 0.1
 * @date 2021-10-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <stdint.h>
#include <string.h>
#include "stateMachine.h"
#include "finiteStateMachine.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "delay.h"
#include "mavInterface.h"
#include "operation.h"
#include "openmv.h"
#include "controller.h"
#include "led.h"
#include "pid.h"
#include "ultrasound.h"
#include "openmv.h"
#include "laser.h"

/* ----------------------- 外部全局变量 ----------------------- */
extern QueueHandle_t fsmEventQueue;         // 向FSM传递事件的condition
extern SemaphoreHandle_t opmvHandleSemphr; 	// openmv解析完成标志
extern SemaphoreHandle_t ctrlMutexSemphr;  	// 互斥信号量，用于给控制指令流发送消息
extern QueueHandle_t ctrlQueue;      		// 来自蓝牙的控制指令流
extern QueueHandle_t distanceQueue;			// 来自超声波的距离量


extern Autopilot_Interface api;
extern opmvInterface omi;

/* ----------------------- 全局变量 ----------------------- */
PID_t fixHeightPid;
float fixHeightPid_P = 0.618;
float fixHeightPid_D = 0.02;




/* ---------------------------- 回调函数 ---------------------------- */
void prvTakeoffStateActionCallback(void);
void prvXtoAStateActionCallback(void);
void prvCruiseStateActionCallback(void);
void prvBackStateActionCallback(void);



// guard
static bool compareChar(void *ch, struct event *event);

// -------------- action during transition --------------
static void takeoff(void *oldStateData, struct event *event, void* newStateData);
static void land(void *oldStateData, struct event *event, void* newStateData);
static void disarm(void *oldStateData, struct event *event, void* newStateData);

// -------------- entry action ---------------
void showEnter(void *stateData, struct event *event);
void printErrMsg( void *stateData, struct event *event );

// -------------- exit action ---------------
void showExit(void *stateData, struct event *event);


// -------------- declare states ahead --------------
static struct state GroundState,		// 在地面上 
					TakingOffState,		// 起飞中
					XtoAState, 			// 从十字飞向A
					CruiseState,		// 遍历各个格子 
					BackState, 			// 返航
					LandingState,		// 降落中
					ErrorState;			// 错误状态 

// 注意，各个state的data首字母不能相同！
// 因为要把它们传递给外部

static struct state GroundState = {
	.parentState = NULL,
	.entryState = NULL, 
	.transitions = (struct transition[]){
		{
			Event_Fly, 					// eventType
			(void *)(intptr_t)CONDITION_TAKEOFF, 	// condition
			&compareChar, 				// guard
			&takeoff, 					// action during transition
			&TakingOffState				// next state
		}
	},
	.numTransitions = 1,
	.data = "ground",
	.entryAction = &showEnter,
	.exitAction = &showExit,
};

static struct state TakingOffState = {
	.parentState = NULL, 
	.entryState = NULL, 
	.transitions = (struct transition[]){
		{Event_Fly, 
		(void *)(intptr_t)CONDITION_REACH, 
		&compareChar, 
		NULL, 
		&XtoAState},
	},
	.numTransitions = 1,
	.data = "takingoff",
	.entryAction = &showEnter, 
	.exitAction = &showExit, 
};

static struct state XtoAState = {
	.parentState = NULL, 
	.entryState = NULL, 
	.transitions = (struct transition[]){
		{
			Event_Fly, 							// eventType
			(void *)(intptr_t)CONDITION_REACH, 	// condition
			&compareChar,		 				// guard
			NULL,	 							// action during transition
			&CruiseState						// next state
		},
		{
			Event_Fly, 							// eventType
			(void *)(intptr_t)CONDITION_LAND, 	// condition
			&compareChar,		 				// guard
			land,	 							// action during transition
			&LandingState						// next state
		},
	}, 
	.numTransitions = 2, 
	.data = "xtoa", 
	.entryAction = &showEnter, 
	.exitAction = &showExit, 
};


static struct state CruiseState = {
	.parentState = NULL, 
	.entryState = NULL, 
	.transitions = (struct transition[]){
		{
			Event_Fly, 								// eventType
			(void *)(intptr_t)CONDITION_FINISH, 	// condition
			&compareChar,		 					// guard
			NULL,	 								// action during transition
			&BackState							// next state
		},
	},
	.numTransitions = 1,
	.data = "cruise", 
	.entryAction = &showEnter, 
	.exitAction = &showExit,
};

static struct state BackState = {
	.parentState = NULL, 
	.entryState = NULL, 
	.transitions = (struct transition[]){
		{
			Event_Fly, 							// eventType
			(void *)(intptr_t)CONDITION_LAND, 	// condition
			&compareChar,		 				// guard
			land,	 							// action during transition
			&LandingState						// next state
		},
	}, 
	.numTransitions = 1, 
	.data = "back", 
	.entryAction = &showEnter, 
	.exitAction = &showExit,
};


// static struct state FlyingGroupState = {
// 	.parentState = NULL, 
// 	.entryState = &CruiseState, 
// 	.transitions = (struct transition[]){
// 		{
// 			Event_Fly, 							// eventType
// 			(void *)(intptr_t)CONDITION_LAND,	// condition
// 			&compareChar,						// guard
// 			land,	 							// action during transition
// 			&LandingState						// next state
// 		},
// 		{
// 			Event_Fly, 					// eventType
// 			NULL, 						// condition
// 			NULL,		 				// guard
// 			reEnterCruise,	 			// action during transition
// 			&CruiseState				// next state
// 		},
// 	},
// 	.numTransitions = 2, 
// 	.data = "flyingGroup", 
// 	.entryAction = &showEnter, 
// 	.exitAction = &showExit,
// };

// static struct state CruiseState = {
// 	.parentState = &FlyingGroupState, 
// 	.entryState = NULL, 
// 	.transitions = (struct transition[]){
// 		{
// 			Event_Fly, 								// eventType
// 			(void *)(intptr_t)CONDITION_DETECT, 	// condition
// 			&compareChar,		 					// guard
// 			NULL,	 								// action during transition
// 			&NearState								// next state
// 		},
// 	},
// 	.numTransitions = 1,
// 	.data = "cruise", 
// 	.entryAction = &showEnter, 
// 	.exitAction = &showExit,
// };


// static struct state NearState = {
// 	.parentState = &FlyingGroupState, 
// 	.entryState = NULL, 
// 	.transitions = (struct transition[]){
// 		{
// 			Event_Fly, 							// eventType
// 			(void *)(intptr_t)CONDITION_REACH, 	// condition
// 			&compareChar,		 				// guard
// 			NULL,	 							// action during transition
// 			&ScanState							// next state
// 		},
// 	},
// 	.numTransitions = 1, 
// 	.data = "near", 
// 	.entryAction = &showEnter, 
// 	.exitAction = &showExit, 
// };

// static struct state ScanState = {
// 	.parentState = &FlyingGroupState, 
// 	.entryState = NULL, 
// 	.transitions = (struct transition[]){
// 		{
// 			Event_Fly, 							// eventType
// 			(void *)(intptr_t)CONDITION_SCAN, 	// condition
// 			&compareChar,		 				// guard
// 			NULL,	 							// action during transition
// 			&AwayState							// next state
// 		},
// 	}, 
// 	.numTransitions = 1, 
// 	.data = "scan", 
// 	.entryAction = &showEnter, 
// 	.exitAction = &showExit, 
// };

// static struct state AwayState = {
// 	.parentState = &FlyingGroupState, 
// 	.entryState = NULL, 
// 	.transitions = (struct transition[]){
// 		{
// 			Event_Fly, 							// eventType
// 			(void *)(intptr_t)CONDITION_REACH, 	// condition
// 			&compareChar,		 				// guard
// 			NULL,	 							// action during transition
// 			&CruiseState						// next state
// 		},
// 	}, 
// 	.numTransitions = 1, 
// 	.data = "away", 
// 	.entryAction = &showEnter, 
// 	.exitAction = &showExit,
// };

static struct state LandingState = {
	.parentState = NULL, 
	.entryState = NULL, 
	.transitions = (struct transition[]){
		{
			Event_Fly, 							// eventType
			(void *)(intptr_t)CONDITION_REACH, 	// condition
			&compareChar,		 				// guard
			disarm,	 							// action during transition
			&GroundState						// next state
		},
	},
	.numTransitions = 1, 
	.data = "landing", 
	.entryAction = &showEnter, 
	.exitAction = &showExit, 
};

static struct state ErrorState = {
	.entryAction = &printErrMsg,
};


// int main()
// {
// 	struct stateMachine m; 
// 	stateM_init( &m, &GroundState, &ErrorState );
// 	char ch; 
// 	while( (ch=getc(stdin)) != EOF )
// 	{
// 		if(ch=='\n')
// 			continue;
// 		stateM_handleEvent( &m, &(struct event){ Event_Fly,
//             (void *)(intptr_t)ch } );
// 	}
	
// 	return 0;
// }

// -------------------------------- transition guard function --------------------------------
static bool compareChar(void *ch, struct event *event)
{
	if(event->type != Event_Fly)
		return false;
	return (intptr_t)ch == (intptr_t)event->data;
}


// -------------------------------- action during transition --------------------------------
static void takeoff(void *oldStateData, struct event *event, void* newStateData)
{
	#if SWITCH_DEBUG
	printf("The autopilot is taking off ...\r\n");
	#endif
	
	opTakeoff(&api);

	return ;
}

static void land(void *oldStateData, struct event *event, void* newStateData)
{
	#if SWITCH_DEBUG
	printf("The autopilot is landing ... \r\n");
	#endif
	
	opLand(&api);

	return ;
}

static void disarm(void *oldStateData, struct event *event, void* newStateData)
{
	#if SWITCH_DEBUG
	printf("Disarm the autopilot\r\n");
	#endif

	mavDisArm();


	return ;

}

// -------------------------------- entry action --------------------------------
void showEnter(void *stateData, struct event *event)
{
	// #if SWITCH_DEBUG
	// printf("Entering %s state\r\n", (char*)stateData);
	
	// if(strcmp((char*)stateData, "takingoff")==0)
	// 	printf("taking off!!! LED0 ON, LED1 OFF\r\n");
	// else if(strcmp((char*)stateData, "landing")==0)
	// 	printf("landing!!! LED0 OFF, LED1 ON\r\n");
	// else if(strcmp((char*)stateData, "cruise")==0)
	// 	printf("cruising LED0 ON, LED1 ON\r\n");
	// #endif

	char temp = ((char*)stateData)[0];
	if(temp == STATE_TAKINGOFF)
	{
		LED0 = 0; LED1 = 1;
	}
	else if(temp == STATE_LANDING)
	{
		LED0 = 1; LED1 = 0;
	}
	else if(temp == STATE_CRUISE)
	{
		LED0 = 0; LED1 = 0;
	}
	
	return ;
}

void printErrMsg( void *stateData, struct event *event )
{
// 	#if SWITCH_DEBUG
//    puts( "ENTERED ERROR STATE!\r\n" );
//    #endif
}


// -------------------------------- exit action --------------------------------
void showExit(void *stateData, struct event *event)
{
	// #if SWITCH_DEBUG
	// printf("Exiting %s state\r\n", (char*)stateData);
	// #endif
	return ;
}




// ================================ 对外部的接口 ================================
struct stateMachine m;

/**
 * @brief 初始化状态机
 * 
 */
void fsmInit(void)
{
	stateM_init( &m, &GroundState, &ErrorState );

	// 初始化定高PID控制器
	PID_StructInit(&fixHeightPid);
	fixHeightPid.f_ParamInit(&fixHeightPid, fixHeightPid_P, 0.0, fixHeightPid_D, 5, 0.5);

}

/**
 * @brief 查询当前状态
 * 
 * @return FLY_STATE 
 */
char queryCurrentState(void)
{
	struct state * ptrState; 
	ptrState = stateM_currentState(&m);
	return ((char*)ptrState->data)[0];
}


// -------------------------------- FSM Task --------------------------------

/**
 * @brief 
 * 
 * @param pvParameters 
 */
void task_fsmHandleEvent(void * pvParameters)
{
	BaseType_t err; 
	uint8_t evt;
	while(1)
	{
		if(fsmEventQueue != NULL)
		{
			err = xQueueReceive(fsmEventQueue, &evt, portMAX_DELAY);
			if(err != errQUEUE_EMPTY)
			{
				#if SWITCH_DEBUG
				printf("FSM receive event condition %c\r\n", evt);
				#endif
				stateM_handleEvent( &m, &(struct event){ Event_Fly, (void *)(intptr_t)evt } );
			}
		}
		else
			delay_ms(100);
	}
}

/**
 * @brief 各个状态中执行的动作
 * 			此任务的优先级应较低
 * 
 * @param pvParameters 
 */
void task_fsmAction(void * pvParameters)
{
	uint8_t cond;	// 向fsmEventQueue中传递的condition

	BaseType_t err;
	char currentState;
	struct state * ptrState; 

	float currentHeight = 0;
	static uint8_t cnt_takeoff = 0;


	while(1)
	{
		ptrState = stateM_currentState(&m);
		currentState = ((char*)(ptrState->data))[0];
		switch (currentState)
		{
			case STATE_GROUND:	// 地面状态中 -------------------------------------------------
			{
				// pass
				delay_ms(100);
				break;
			}

			case STATE_TAKINGOFF:	// 起飞状态中 -------------------------------------------------
			{
				currentHeight = api.current_messages.optical_flow_rad.distance;
				prvTakeoffStateActionCallback();
				// // 如果本次测距结果与上次偏差过大，则限幅
				// if( (first_flag!=1) && (  (currentHeight-lastHeight>0.5) || (currentHeight-lastHeight<-0.5)  ) )
				// 	currentHeight = lastHeight;
				// first_flag = 0;
				// lastHeight = currentHeight;
				// takeoffHeight = api.targetHeight;
				// errHeight = takeoffHeight - currentHeight;

				// if(currentHeight < 0.8 || currentHeight > 1.8)
				// {
				// 	delay_ms(100);
				// }
				// else
				// {
				// 	cnt_takeoff ++; 
				// 	if(cnt_takeoff > 10)
				// 		prvFixHeightStateActionCallback();
				// 	// mavSetVelocity( 0, // [m]
				// 	// 				0, // [m]
				// 	// 				-1.0* pid_P *errHeight, // [m]
				// 	// 				&spv);
				// 	// mavUpdateSetPoint(spv);
				// 	// mavWriteSetPoint();

				// 	// if( (errHeight>-0.1) && (errHeight<0.1) )
				// 	// {
				// 	// 	cond = CONDITION_REACH;
				// 	// 	err = xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
				// 	// }
				// }

				// if(cnt_takeoff < 150)
				// {
				// 	delay_ms(100);
				// 	mavWriteSetPoint();
				// 	cnt_takeoff ++;
				// }
				// else
				// {
				// 	cnt_takeoff = 0;
				// 	cond = CONDITION_REACH;
				// 	err = xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
				// }

				// prvFixHeightStateActionCallback();

				// // // 测试状态机
				// delay_ms(15000);
				// cond = CONDITION_REACH;
				// err = xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
				// if(ctrlQueue != NULL)
				// {
				// 	uint8_t ch = CTRL_NULL;
				// 	xSemaphoreTake(ctrlMutexSemphr, portMAX_DELAY);
				// 	err = xQueueSend(ctrlQueue, &ch, 0);
				// 	if(err == errQUEUE_FULL)
				// 	{
				// 		;
				// 	}
				// 	xSemaphoreGive(ctrlMutexSemphr);
				// }

				break;
			}

			case STATE_XTOA:
			{
				prvXtoAStateActionCallback();
				break;
			}

			case STATE_CRUISE:	// 巡航状态中 -------------------------------------------------
			{
				prvCruiseStateActionCallback();
				break;
			}

			case STATE_BACK:
			{
				prvBackStateActionCallback();
				break;
			}

			case STATE_LANDING:	// 降落状态中 -------------------------------------------------
			{
				// 测试状态机
				// delay_ms(9000);
				int ii = 0;
				for(ii=0; ii<8; ii++)
				{
					opLand(&api);
					delay_ms(1000);
				}
				
				cond = CONDITION_REACH;
				err = xQueueSend(fsmEventQueue, &cond, 0);
				
				break;
			}

			default:
				break;
		}
	}
}


/* -------------------- 各个状态中动作 -------------------- */

void prvXtoAStateActionCallback(void)
{
	mavlink_set_position_target_local_ned_t sp;
	static uint8_t cnt = 0;
	uint8_t maxcnt = 4;
	uint8_t cond;

	// 向openmv发送包
	opmvSendPakage(OPMV_SEND_ID_2);
	// 处理

	if(omi.pkg2_flag == 1)
	{
		cond = CONDITION_REACH; 
		xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
		LASER = 0;
		delay_ms(1000);
		LASER = 1;
	}
	else
	{
		cnt ++;

		if(cnt < maxcnt)
		{
			mavlink_local_position_ned_t cp = api.current_messages.local_position_ned;
			mavSetPosition( cp.x - 0.2,       // [m]
							cp.y ,       // [m]
							cp.z ,		// [m]
							&sp         );
			mavUpdateSetPoint(sp);
			mavWriteSetPoint();
			delay_ms(1000);
			mavSetVelocity(	0.1, 0.0, 0.0, &sp);
			mavUpdateSetPoint(sp);
			mavWriteSetPoint();
			delay_ms(1500);
			
			mavSetVelocity(	0.0, 0.0, 0.0, &sp);
			mavUpdateSetPoint(sp);
			mavWriteSetPoint();
			delay_ms(1000);
		}
		else
		{
			cond = CONDITION_LAND;
			xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
		}
		
	}

	// 首先稳定一段时间
	// int i = 0;
	// for(i=0; i<8; i++)
	// {
	// 	mavSetAttitudeZero();
	// 	delay_ms(300);
	// }
	// mavSetAttitudeZero();

	// 向前飞行一段距离
	
	

	// for(i=0; i<8; i++)
	// {
	// 	mavSetAttitudeZero();
	// 	delay_ms(100);
	// }
	// delay_ms(1000);
	// mavSetAttitudeZero();
	
	// delay_ms(5000);


}

void prvCruiseStateActionCallback(void)
{
	mavlink_set_position_target_local_ned_t sp;
	mavlink_local_position_ned_t cp = api.current_messages.local_position_ned;
	mavSetPosition( cp.x,       // [m]
					cp.y - 0.2,       // [m]
					cp.z ,		// [m]
					&sp);
	mavUpdateSetPoint(sp);
	mavWriteSetPoint();
	delay_ms(1000);
	mavSetVelocity(	0.0, 0.2, 0.0, &sp);
	mavUpdateSetPoint(sp);
	mavWriteSetPoint();
	delay_ms(1500);
	
	mavSetVelocity(	0.0, 0.0, 0.0, &sp);
	mavUpdateSetPoint(sp);
	mavWriteSetPoint();
	delay_ms(1000);

	uint8_t cond = CONDITION_FINISH; 
	xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
}



void prvBackStateActionCallback(void)
{

	uint8_t cond = CONDITION_LAND; 
	xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
}


///**
// * @brief 巡航状态中的动作
// * 
// */
//void prvCruiseStateActionCallback(void)
//{
//	// // 寻找线
//	// opmvSendPakage(OPMV_SEND_ID_1);
//	// xSemaphoreTake(opmvHandleSemphr, portMAX_DELAY);
//	// uint8_t cmd; 
//	// if( omi.pkg1_x < 100 )
//	// 	cmd = CTRL_LEFT_ROTATE;
//	// else if( omi.pkg1_x > 150 )	
//	// 	cmd = CTRL_RIGHT_ROTATE;
//	// xSemaphoreTake(ctrlMutexSemphr, portMAX_DELAY);
//	// xQueueSend(ctrlQueue, &cmd, 0);
//	// xSemaphoreGive(ctrlMutexSemphr);
//	
//	// 测试状态机
//	BaseType_t err;
//	// delay_ms(1000);

//	// mavlink_local_position_ned_t cp = api.current_messages.local_position_ned;
//	mavlink_set_position_target_local_ned_t sp;
//	// mavSetPosition( cp.x + 0.3,       // [m]
//    //                 cp.y ,       // [m]
//    //                 cp.z ,		// [m]
//    //                 &sp         );
//	// mavSetVelocity(	0.1, 
//	// 				0.0,
//	// 				0.0,
//	// 				&sp		);
//	// sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POS_VEL;
//	// sp.type_mask |= POSITION_TARGET_TYPEMASK_Y_IGNORE;
//	// sp.type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE;
//	
//	// mavUpdateSetPoint(sp);
//	// mavWriteSetPoint();
//	// delay_ms(2000);

//	mavSetVelocity(	0.0, 
//					0.0,
//					0.0,
//					&sp		);
//	mavUpdateSetPoint(sp);
//	mavWriteSetPoint();
//	delay_ms(2000);


//	// cp = api.current_messages.local_position_ned;
//	// mavSetPosition( cp.x ,       // [m]
//    //                 cp.y + 0.3,       // [m]
//    //                 cp.z ,		// [m]
//    //                 &sp         );
//	// mavSetVelocity(	0.0, 
//	// 				0.1,
//	// 				0.0,
//	// 				&sp		);
//	// sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POS_VEL;
//	// sp.type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE;
//	// sp.type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE;
//	// mavUpdateSetPoint(sp);
//	// mavWriteSetPoint();
//	// delay_ms(2000);

//	uint8_t cond = CONDITION_LAND; 
//	err = xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
//	#if SWITCH_DEBUG
//		if(err == errQUEUE_FULL)
//			printf("The fsmEventQueue is full\r\n");
//	#endif 
//}


/**
 * @brief 定高飞行模式中的动作
 * 
 */


void prvTakeoffStateActionCallback(void)
{
	static uint8_t cnt = 0;
	fixHeightPid.TargetValue = 1.2;
	// float currentHeight = api.current_messages.optical_flow_rad.distance;
	static float currentHeight = 0;
	xQueuePeek(distanceQueue, &currentHeight, 0);
	fixHeightPid.f_Calculate(&fixHeightPid, currentHeight);
	float outputVel = fixHeightPid.output;
	mavlink_set_position_target_local_ned_t spv;

	mavSetVelocity( 0, // [m/s]
					0, // [m/s]
					-1.0 * outputVel, // [m/s]
					&spv);
	spv.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	mavUpdateSetPoint(spv);
	mavWriteSetPoint();
	
	// uint8_t cond;	// 向fsmEventQueue中传递的condition
	if( (fixHeightPid.Error>-0.1) && (fixHeightPid.Error<0.1) )
	{
		cnt ++; 
		if(cnt > 10)
		{
			uint8_t cond = CONDITION_REACH;
			xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
		}
	}
}






