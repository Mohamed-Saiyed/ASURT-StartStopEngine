#include "Tasks.h"
#include "StartStopEngine_Config.h"

typedef struct
{
	uint32_t StartStopEngineStarterMotorMaxOnTime;
	uint32_t StartStopEngineStarterMotorMaxHoldTime;	
	uint32_t StartStopEngineLongPressMaxTime;
	uint16_t StartStopEngineIdealRpmValue;
	int16_t  StartStopEngineRpm;
	uint8_t  StartStopEngineButtonStatus;
	uint8_t  StartStopEnginePreviousButtonStatus;
	uint8_t  StartStopEngineCurrentButtonStatus;
	uint8_t  StartStopEngineStatus;
	uint8_t  StartStopEnginePreStatus;
	uint8_t  StartStopEngineFeatureSwitchStatus;
	uint8_t  StartStopEngineBreakSwitchStatus;
	uint8_t  StartStopEngineTryCount;
	uint8_t  StartStopEngineMaxTryCount;
	uint8_t  CurrentVehicleSpeed;
	uint8_t  CurrentEngineGear;
	uint8_t  MaxVehicleSpeedToStop;
	uint8_t  StartStopEngineECUStatus;
	uint8_t StartStopEngineHmiSymbolStatus;
	uint8_t StartStopEngineFeatureState;
	
}StartStopEngine;

uint8_t  LongPressFlag = 0;
uint64_t NumberOfMessagesRecievedFromEcu;

SemaphoreHandle_t SSETimeControlTaskSemaphore	    = NULL;
static SemaphoreHandle_t SSEStateControlTaskSemaphore	    = NULL;

static TaskHandle_t StartStopStateControlHandler 			= NULL;
static TaskHandle_t StartStopReadButtonHandler 				= NULL;
static TaskHandle_t StartStopTimeControlHandler   			= NULL;
static TaskHandle_t StartStopFeatureSwitchControlHandler	= NULL;	


static StartStopEngine StartStopEngineInfo;
static CAN_RxHeaderTypeDef SSE_CANRxHeader;


static inline void StopStarterMotor(void);
static inline void StartStarterMotor(void);
static inline void StarterMotorHold(void);
static inline void StarterMotorReset(void);
static inline void TurnOnSSEButtonLed(void);
static inline void TurnOffSSEButtonLed(void);
static inline void DisableStartStopEngine(void);
static inline void ECU_ShutdownEngine(void);
static inline void ReadBreakSwitch(void);
static inline void CheckIfEcuIsAwake(void);
static void StartStopEngine_ReceiveEngineRpmValue(void);
static void CAN_Filter_Config(void);
static void StartStopEngine_Init(void);
static void ECU_RpmCANRemoteFrameSend(void);
static void StartStopEngineCreateAllTasks(void);

uint8_t ExternalSourceFlag;
void StartStopFeatureSwitchControl(void * pvParameters)
{	
	vTaskSetApplicationTaskTag(NULL , (void* )4);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	static uint8_t PrvState = BUTTON_STATUS_RELEASED;

	while(1)
	{
		 if(!ExternalSourceFlag)
		 {
		 	StartStopEngineInfo.StartStopEngineFeatureSwitchStatus = HAL_GPIO_ReadPin(SSE_ON_OFF_SWITCH_PORT , SSE_ON_OFF_SWITCH_PIN);
		 }
			
			if(PrvState != StartStopEngineInfo.StartStopEngineFeatureSwitchStatus)
			{
						
				switch(StartStopEngineInfo.StartStopEngineFeatureSwitchStatus)
				{
					
					case BUTTON_STATUS_RELEASED:
						
						StarterMotorReset();
					
						if(StartStopEngineInfo.StartStopEngineStatus != SSE_STATUS_FINISHED_OK || StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_LONG_PRESS)
						{
							StartStopEngine_Init();
						}
						else
						{
							TurnOnSSEButtonLed();
						}
					
						/* start stop sign is ON on the dashboard */
						StartStopEngineInfo.StartStopEngineHmiSymbolStatus = TURN_ON;
						StartStopEngineInfo.StartStopEngineFeatureState = SSE_FEATURE_ENABLED;	
												
						//	vTaskResume(StartStopReadButtonHandler);	
																
					break;
					case BUTTON_STATUS_PRESSED:
						
						StopStarterMotor();
						DisableStartStopEngine();
						
						
						TurnOffSSEButtonLed();
						StartStopEngineInfo.StartStopEngineHmiSymbolStatus = TURN_OFF;
												
						if((StartStopEngineInfo.StartStopEngineFeatureState != SSE_FEATURE_DISABLED_BY_SOFTWARE) &&\
							 (StartStopEngineInfo.StartStopEngineFeatureState != SSE_FEATURE_DISABLED_BY_EXTERNAL_SOURCE))
						{
							StartStopEngineInfo.StartStopEngineFeatureState = SSE_FEATURE_DISABLED_BY_HARDWARE;	
						}
					
						//vTaskSuspend(StartStopReadButtonHandler);	
					
					break;
					
					default:
					
					break;
				}
				
			}
		
		CAN_TransmitMessage(SSE_HMI_SYMBOL_CAN_ID,1,&StartStopEngineInfo.StartStopEngineHmiSymbolStatus);
		CAN_TransmitMessage(SSE_FEATURE_STATE_CAN_ID,1,&StartStopEngineInfo.StartStopEngineFeatureState);
		 if(ExternalSourceFlag)
		 {
				ExternalSourceFlag = 0;
		 }

		 PrvState = StartStopEngineInfo.StartStopEngineFeatureSwitchStatus;
			
		vTaskDelayUntil(&xLastWakeTime,2);
	}
}

void StartStopEngineStateControl(void * pvParameters)
{	
	vTaskSetApplicationTaskTag(NULL , (void* )1);
 	SSETimeControlTaskSemaphore = xSemaphoreCreateBinary();
	SSEStateControlTaskSemaphore = xSemaphoreCreateBinary();
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
	
		if(xSemaphoreTake( SSEStateControlTaskSemaphore, portMAX_DELAY ) == pdTRUE )
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			if((StartStopEngineInfo.StartStopEngineECUStatus == ECU_IS_SLEEP) && (StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_WAITING))
			{
				for(uint8_t LocalVar = 0 ; LocalVar < 2 ; LocalVar++)
				{
					ECU_RpmCANRemoteFrameSend();
				}
				
				
				if(StartStopEngineInfo.StartStopEngineECUStatus != ECU_IS_AWAKE)
				{
					/*Send Message to the Dashboard that ECU is Not Awake */
					 StartStopEngineInfo.StartStopEngineButtonStatus = BUTTON_STATUS_RELEASED;
					 CAN_TransmitMessage(SSE_ECU_IS_NOT_AWAKE_CAN_ID,1,&StartStopEngineInfo.StartStopEngineECUStatus);
				}
			}
									
				switch(StartStopEngineInfo.StartStopEngineStatus)
				{
					case SSE_STATUS_WAITING:
						
					if(StartStopEngineInfo.StartStopEngineECUStatus == ECU_IS_AWAKE)
					{
							if(SSE_USE_BREAK_SWITCH == TURN_ON && StartStopEngineInfo.StartStopEngineBreakSwitchStatus == BUTTON_STATUS_RELEASED)
							{
									CAN_TransmitMessage(SSE_BRAKE_SWITCH_IS_NOT_PRESSED,1,&StartStopEngineInfo.StartStopEngineBreakSwitchStatus);	
							}
							else
							{
									StartStarterMotor();
									/* should go before */
									TurnOnSSEButtonLed();
									StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_ONGOING;
							}
							StartStopEngineInfo.StartStopEngineButtonStatus = BUTTON_STATUS_RELEASED;
					}
					else
					{
					
					}
										
					break;
					
					case SSE_STATUS_ONGOING:
					
						if(StartStopEngineInfo.StartStopEngineRpm >= StartStopEngineInfo.StartStopEngineIdealRpmValue)
						{
							StopStarterMotor();
							StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_FINISHED_OK;
							xSemaphoreTake( SSETimeControlTaskSemaphore, ( TickType_t ) 10 );
						}
						else
						{
						
						}
					
					break;	
					
					case SSE_STATUS_FINISHED_OK:
						
						 StopStarterMotor();
					
					break;
					
					case SSE_STATUS_FINISHED_NOK:
										
						StopStarterMotor();
						DisableStartStopEngine();
						TurnOffSSEButtonLed();
						CAN_TransmitMessage(SSE_MAX_STARTER_MOTOR_TRY_COUNT_CAN_ID,1,&StartStopEngineInfo.StartStopEngineTryCount);
					
					break;
					case SSE_STATUS_LONG_PRESS: 
					
							if (__HAL_TIM_GET_COUNTER(&htim3) >= StartStopEngineInfo.StartStopEngineLongPressMaxTime)
							{
								if(StartStopEngineInfo.CurrentVehicleSpeed <= StartStopEngineInfo.MaxVehicleSpeedToStop ||
									((StartStopEngineInfo.CurrentVehicleSpeed > StartStopEngineInfo.MaxVehicleSpeedToStop) && (SSE_STOP_VEHICLE_ON_HIGH_SPEED == TURN_ON)))
								{
									__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE); 
									HAL_TIM_Base_Stop(&htim3);
									__HAL_TIM_SET_COUNTER(&htim3,0);
									
									ECU_ShutdownEngine();
									StartStopEngine_Init();
									
									LongPressFlag = 1;
								}  
								else
								{
									/* You Can Send a Message */
								}
							}
						
					break;
				
				}
								
		}
	
		vTaskDelayUntil(&xLastWakeTime,5);
	
	}
}

void StartStopEngineStarterTimingControl(void *pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )3);
	xLastWakeTime = xTaskGetTickCount();
		
	while(1)
	{
		if( xSemaphoreTake( SSETimeControlTaskSemaphore, ( TickType_t ) portMAX_DELAY  ) == pdTRUE )
		{
			if(StartStopEngineInfo.StartStopEngineTryCount >= StartStopEngineInfo.StartStopEngineMaxTryCount * 2)
			{
				StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_FINISHED_NOK;
				xSemaphoreGive(SSEStateControlTaskSemaphore);

			}	
			else if(StartStopEngineInfo.StartStopEngineTryCount % 2 == 0)
			{
				StarterMotorHold();
			}
			else
			{
				StartStarterMotor();
			}
			
			StartStopEngineInfo.StartStopEngineTryCount++;
		}
		else
		{
		
		}
		
		vTaskDelayUntil(&xLastWakeTime,5);
	}
}



static inline void CheckIfEcuIsAwake(void)
{
	static uint64_t PrvNumber;
	static uint64_t Tries;
	
	if(PrvNumber == NumberOfMessagesRecievedFromEcu)
	{
		  Tries++;
			if(Tries > 200)
			{
				StartStopEngineInfo.StartStopEngineECUStatus = ECU_IS_SLEEP;
				CAN_TransmitMessage(SSE_ECU_IS_NOT_AWAKE_CAN_ID,1,&StartStopEngineInfo.StartStopEngineECUStatus);
				Tries = 0;
			}				
	}
	else
	{
		Tries = 0;
	}
	PrvNumber	= NumberOfMessagesRecievedFromEcu;
}

void StartStopEngineButtonHandle(void *pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )7);
	xLastWakeTime = xTaskGetTickCount();
		
	while(1)
	{
		
		StartStopEngineInfo.StartStopEngineCurrentButtonStatus = HAL_GPIO_ReadPin(SSE_BUTTON_PORT , SSE_BUTTON_PIN_NUMBER);

		if(StartStopEngineInfo.StartStopEngineCurrentButtonStatus == BUTTON_STATUS_RELEASED &&\
			 StartStopEngineInfo.StartStopEnginePreviousButtonStatus == BUTTON_STATUS_PRESSED)
		{
			StartStopEngineInfo.StartStopEngineButtonStatus = BUTTON_STATUS_PRESSED;		
		}
		
		if((StartStopEngineInfo.StartStopEngineButtonStatus == BUTTON_STATUS_PRESSED) &&\
			((StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_WAITING)))
		{
			if(LongPressFlag == 1)
			{
				LongPressFlag = 0;
				StartStopEngineInfo.StartStopEngineButtonStatus = BUTTON_STATUS_RELEASED;
			}
			else
			{
				xSemaphoreGive(SSEStateControlTaskSemaphore);
			}
		}
		else if(StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_ONGOING)
		{
			if(StartStopEngineInfo.StartStopEngineButtonStatus == BUTTON_STATUS_PRESSED)
			{
				StopStarterMotor();
				StartStopEngine_Init();			
			}
		}
		else if((StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_FINISHED_OK) &&\
		   (StartStopEngineInfo.StartStopEngineCurrentButtonStatus == BUTTON_STATUS_PRESSED && StartStopEngineInfo.StartStopEnginePreviousButtonStatus == BUTTON_STATUS_PRESSED))
		{
			HAL_TIM_Base_Start(&htim3);
			StartStopEngineInfo.StartStopEnginePreStatus = StartStopEngineInfo.StartStopEngineStatus;
			StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_LONG_PRESS;
			xSemaphoreGive(SSEStateControlTaskSemaphore);
		}	
		else if(StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_LONG_PRESS && StartStopEngineInfo.StartStopEngineCurrentButtonStatus == BUTTON_STATUS_PRESSED)
		{
			xSemaphoreGive(SSEStateControlTaskSemaphore);
		}
		else if(StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_LONG_PRESS && StartStopEngineInfo.StartStopEngineCurrentButtonStatus == BUTTON_STATUS_RELEASED)
		{
			HAL_TIM_Base_Stop(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			
		  if(StartStopEngineInfo.StartStopEnginePreStatus == SSE_STATUS_FINISHED_OK)
			{
				StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_FINISHED_OK;
			}
			else
			{
				StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_FINISHED_NOK;
			}
										
		}
		else
		{
		
		}
		
		StartStopEngineInfo.StartStopEnginePreviousButtonStatus = StartStopEngineInfo.StartStopEngineCurrentButtonStatus;
		
		ReadBreakSwitch();
		CheckIfEcuIsAwake();
		vTaskDelayUntil(&xLastWakeTime,5);
	}
}


void StartStopEngineInit(void)
{
	StartStopEngine_Init();
	CAN_Filter_Config();
	StartStopEngineCreateAllTasks();

}


void StartStopEngineCreateAllTasks(void)
{
	xTaskCreate(StartStopEngineStateControl , "Task-1" , 500 , NULL , 3 , &StartStopStateControlHandler);
	xTaskCreate(StartStopEngineStarterTimingControl , "Task-2" , 500 , NULL , 3 , &StartStopTimeControlHandler);
	xTaskCreate(StartStopFeatureSwitchControl  , "Task-4" , 500 , NULL , 5 , &StartStopFeatureSwitchControlHandler);
	xTaskCreate(StartStopEngineButtonHandle , "Task-5" , 500 , NULL , 3 , &StartStopReadButtonHandler);
}

static void StartStopEngine_Init(void)
{
	StartStopEngineInfo.StartStopEngineButtonStatus = BUTTON_STATUS_RELEASED;
	StartStopEngineInfo.StartStopEnginePreviousButtonStatus = BUTTON_STATUS_RELEASED;
	StartStopEngineInfo.StartStopEngineCurrentButtonStatus = BUTTON_STATUS_RELEASED;
	StartStopEngineInfo.StartStopEngineStatus = SSE_STATUS_WAITING;
	StartStopEngineInfo.StartStopEngineFeatureState = SSE_FEATURE_ENABLED;
	StartStopEngineInfo.StartStopEngineBreakSwitchStatus = HAL_GPIO_ReadPin(SSE_BREAK_SWITCH_PORT,SSE_BREAK_SWITCH_PORT_PIN);
	StartStopEngineInfo.StartStopEngineFeatureSwitchStatus = HAL_GPIO_ReadPin(SSE_ON_OFF_SWITCH_PORT,SSE_ON_OFF_SWITCH_PIN);
	StartStopEngineInfo.StartStopEngineIdealRpmValue = IDEAL_RPM_VALUE;
	StartStopEngineInfo.StartStopEngineMaxTryCount = SSE_MAX_TRY_COUNT;
	StartStopEngineInfo.StartStopEngineHmiSymbolStatus = TURN_ON;
	StartStopEngineInfo.StartStopEngineECUStatus = ECU_IS_SLEEP;
	StartStopEngineInfo.StartStopEngineTryCount = 0;
	StartStopEngineInfo.StartStopEngineRpm = -1;
	StartStopEngineInfo.MaxVehicleSpeedToStop = VEHICLE_STOP_MAX_SPPED;
	StartStopEngineInfo.StartStopEngineStarterMotorMaxOnTime   = (((STARTERMOTOR_ON_TIME * 10000) / 2 ) - 1);
	StartStopEngineInfo.StartStopEngineStarterMotorMaxHoldTime = (((STARTERMOTOR_HOLD_TIME * 10000) / 2 ) - 1);
	StartStopEngineInfo.StartStopEngineLongPressMaxTime = (((SSE_LONG_PRESS_MAX_TIME * 10000) / 2 ) - 1);		
	TurnOffSSEButtonLed();
	__HAL_TIM_CLEAR_IT(SSE_TIME_HANDLER,TIM_IT_UPDATE);	
	
}

static void ECU_RpmCANRemoteFrameSend(void)
{
	uint8_t data;
	uint32_t mailbox;
	CAN_TxHeaderTypeDef EngineRpm;
	
	EngineRpm.StdId = SSE_ENGINE_RPM_CAN_ID;
	EngineRpm.ExtId = 0;
	EngineRpm.DLC   = 2;
	EngineRpm.IDE   = CAN_ID_STD;
	EngineRpm.RTR   = CAN_RTR_REMOTE;
	EngineRpm.TransmitGlobalTime = DISABLE;
	
	HAL_CAN_AddTxMessage(&hcan,&EngineRpm,&data,&mailbox);

}


void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef can1_filter_init;

	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank  = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x0000;
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0x0000;
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	if( HAL_CAN_ConfigFilter(&hcan,&can1_filter_init) != HAL_OK)
	{
		
	}
	
 	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF);
	HAL_CAN_Start(&hcan);

}



/******************************************************************************************************************/

static inline void StarterMotorHold(void)
{
	StopStarterMotor();
	/* you should Suspend control Task */
	__HAL_TIM_SET_AUTORELOAD(SSE_TIME_HANDLER,StartStopEngineInfo.StartStopEngineStarterMotorMaxHoldTime);
	__HAL_TIM_CLEAR_IT(SSE_TIME_HANDLER,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(SSE_TIME_HANDLER);
}

static inline void StopStarterMotor(void)
{
	HAL_GPIO_WritePin(SSE_STARTER_MOTOR_RELAY_PORT , SSE_STARTER_MOTOR_RELAY_PIN , BUTTON_STATUS_RELEASED);
	HAL_TIM_Base_Stop_IT(SSE_TIME_HANDLER);
	__HAL_TIM_SET_COUNTER(SSE_TIME_HANDLER,0);
}


static inline void StartStarterMotor(void)
{
	HAL_TIM_Base_Stop_IT(SSE_TIME_HANDLER);
	__HAL_TIM_SET_COUNTER(SSE_TIME_HANDLER,0);
	__HAL_TIM_SET_AUTORELOAD(SSE_TIME_HANDLER,StartStopEngineInfo.StartStopEngineStarterMotorMaxOnTime);
	__HAL_TIM_CLEAR_IT(SSE_TIME_HANDLER,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(SSE_TIME_HANDLER);
	
	HAL_GPIO_WritePin(SSE_STARTER_MOTOR_RELAY_PORT , SSE_STARTER_MOTOR_RELAY_PIN , BUTTON_STATUS_PRESSED);
	
}



static inline void DisableStartStopEngine(void)
{
	StopStarterMotor();
	StartStopEngineInfo.StartStopEngineFeatureState = SSE_FEATURE_DISABLED_BY_SOFTWARE;	
	HAL_GPIO_WritePin(SSE_ON_OFF_SWITCH_PORT , SSE_ON_OFF_SWITCH_PIN , BUTTON_STATUS_PRESSED);

}

static inline void StarterMotorReset(void)
{
	HAL_GPIO_WritePin(SSE_ON_OFF_SWITCH_PORT , SSE_ON_OFF_SWITCH_PIN , BUTTON_STATUS_RELEASED);
}

static inline void TurnOnSSEButtonLed(void)
{
	HAL_GPIO_WritePin(SSE_BUTTON_LED_PORT , SSE_BUTTON_LED_PIN , GPIO_PIN_RESET);

}

static inline void TurnOffSSEButtonLed(void)
{
	HAL_GPIO_WritePin(SSE_BUTTON_LED_PORT , SSE_BUTTON_LED_PIN , GPIO_PIN_SET);
}

static inline void ECU_ShutdownEngine(void)
{
	uint8_t FuelCutState = 1;
	CAN_TransmitMessage(SSE_ECU_SHUTDOWN_CAN_ID,1,&FuelCutState);
	FuelCutState = 0;
	CAN_TransmitMessage(SSE_ECU_SHUTDOWN_CAN_ID,1,&FuelCutState);
	
}

void CAN_TransmitMessage(uint32_t ID , uint32_t DLC, uint8_t aData[])
{
	CAN_TxHeaderTypeDef MessageToTransmit;
	uint32_t MailBox;
	
	MessageToTransmit.StdId = ID;
	MessageToTransmit.IDE   = CAN_ID_STD;	
	MessageToTransmit.ExtId = 0;
	MessageToTransmit.RTR   = CAN_RTR_DATA;
	MessageToTransmit.DLC	= DLC;
	MessageToTransmit.TransmitGlobalTime = DISABLE;
	
	HAL_CAN_AddTxMessage(&hcan,&MessageToTransmit,aData,&MailBox);
	
}

static inline void ReadBreakSwitch(void)
{
		StartStopEngineInfo.StartStopEngineBreakSwitchStatus = HAL_GPIO_ReadPin(SSE_BREAK_SWITCH_PORT , SSE_BREAK_SWITCH_PORT_PIN);
}

/******************************************************************************************************************/
uint8_t ReceivedCANMessage[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Temp =0; 
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&SSE_CANRxHeader,ReceivedCANMessage) == HAL_OK)
	{

	}
	switch(SSE_CANRxHeader.StdId)
	{
		case SSE_ENGINE_RPM_CAN_ID:
			
			NumberOfMessagesRecievedFromEcu++;
		
			StartStopEngineInfo.StartStopEngineECUStatus = ECU_IS_AWAKE;

			StartStopEngineInfo.StartStopEngineRpm = (uint16_t)((ReceivedCANMessage[0] <<8) | (ReceivedCANMessage[1] ));
			
			if((StartStopEngineInfo.StartStopEngineRpm > StartStopEngineInfo.StartStopEngineIdealRpmValue) && (StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_ONGOING))
			{
				xSemaphoreGive(SSEStateControlTaskSemaphore);
			}
			
			if((StartStopEngineInfo.StartStopEngineRpm < 200) && (StartStopEngineInfo.StartStopEngineStatus == SSE_STATUS_FINISHED_OK))
			{
				StartStopEngine_Init();
			}
			

		break;
		case SSE_VEHICLE_SPEED_CAN_ID:
		
			StartStopEngineInfo.CurrentVehicleSpeed = (uint8_t)(ReceivedCANMessage[0]);
			
		break;
		case SSE_ENGINE_NEUTRAL_CAN_ID:
		
			StartStopEngineInfo.CurrentEngineGear = (uint8_t)(ReceivedCANMessage[0]);
			
		break;
		case SSE_FEATURE_ENABLE_CAN_ID:
			
		  ExternalSourceFlag = 1;
		
			if(StartStopEngineInfo.StartStopEngineFeatureState != SSE_FEATURE_DISABLED_BY_HARDWARE)
			{
				StartStopEngineInfo.StartStopEngineFeatureSwitchStatus = BUTTON_STATUS_RELEASED;
			}
			else
			{
				/*Send a message to ignore this condtion*/
			}
					
		break;
		case SSE_FEATURE_DISABLE_CAN_ID:
			
			ExternalSourceFlag = 1;
				
			StartStopEngineInfo.StartStopEngineFeatureSwitchStatus = BUTTON_STATUS_PRESSED;
			
		break;
		case SSE_PARM_CHANGE_CAN_ID:
		
			StartStopEngineInfo.StartStopEngineIdealRpmValue = (uint16_t)((ReceivedCANMessage[0] <<8) | (ReceivedCANMessage[1] ));
			StartStopEngineInfo.StartStopEngineStarterMotorMaxOnTime = ((((uint32_t)ReceivedCANMessage[2] * 10000) / 2 ) - 1);
			StartStopEngineInfo.StartStopEngineStarterMotorMaxHoldTime = ((((uint32_t)ReceivedCANMessage[3] * 10000) / 2 ) - 1);
			StartStopEngineInfo.StartStopEngineMaxTryCount = ReceivedCANMessage[4] ;
			StartStopEngineInfo.StartStopEngineLongPressMaxTime = ((((uint32_t)ReceivedCANMessage[5] * 10000) / 2 ) - 1);
			StartStopEngineInfo.MaxVehicleSpeedToStop = ReceivedCANMessage[6];
			
		break;
	}
	
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{

}