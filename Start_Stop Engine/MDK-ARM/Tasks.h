/* Check ECU is Awake */
			/* if the the start stop feature switch is off turn off its led on the dashboard */
			/* if the ECU is not awake send an error signal to the dashboard */
			/* if it is awake turn on start stop led on the dashboard then start timer and start starter Motor */
			/* Wait for the RPM to go to ideal value */
			/* If there is there No ideal RPM Response for 10 seconds
					stop the starter for 1 second and start again
						do this opertaion for three times
							if also there is no response open start stop feature Relay 
			*/
			/* Read the signal pin on the starter relaly if it is high and the engine is Working send warning on the dashboard and turn of start stop feature*/
/*led for feature*/			
/*further I need to confgure CAN Filter
	RPM Message Timeout
*/			
/* Buzzer */
/* if Engine is but the car has a speed and you need to restart the engine you must put it on N*/ /*Display a message */ 
/* if vehcile speed is 10 kilo you can stop the car by one preess */
/* LCD is turned on when the button is started *//*Display a message */
/*IF FINISHED OK check rpm*/
#ifndef SSE_TASKS_H
#define SSE_TASKS_H

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern CAN_HandleTypeDef hcan;

#define SSE_ENGINE_RPM_CAN_ID										0x50
#define SSE_ENGINE_NEUTRAL_CAN_ID								0x56
#define SSE_VEHICLE_SPEED_CAN_ID								0x53

#define SSE_HMI_SYMBOL_CAN_ID										0x70
#define SSE_FEATURE_STATE_CAN_ID								0x71
#define SSE_MAX_STARTER_MOTOR_TRY_COUNT_CAN_ID	0x72
#define SSE_ECU_IS_NOT_AWAKE_CAN_ID							0x73
#define SSE_BRAKE_SWITCH_IS_NOT_PRESSED					0x74

#define SSE_ECU_SHUTDOWN_CAN_ID									0x75

#define SSE_FEATURE_DISABLE_CAN_ID							0x76
#define SSE_FEATURE_ENABLE_CAN_ID				  			0x77
#define SSE_PARM_CHANGE_CAN_ID					  			0x78


#define  SSE_STATUS_WAITING 		   1U
#define  SSE_STATUS_ONGOING  		   2U
#define  SSE_STATUS_FINISHED_OK  	 3U
#define  SSE_STATUS_LONG_PRESS   	 4U
#define  SSE_STATUS_FINISHED_NOK   5U

#define  BUTTON_STATUS_PRESSED   0U
#define  BUTTON_STATUS_RELEASED  1U

#define SSE_FEATURE_ENABLED											0
#define SSE_FEATURE_DISABLED_BY_SOFTWARE				1
#define SSE_FEATURE_DISABLED_BY_HARDWARE				2
#define SSE_FEATURE_DISABLED_BY_EXTERNAL_SOURCE	3
#define SSE_FEATURE_ENABLED_BY_EXTERNAL_SOURCE	4

#define ECU_IS_SLEEP	0
#define ECU_IS_AWAKE	1

#define TURN_ON  1
#define TURN_OFF 0


void CAN_TransmitMessage(uint32_t ID , uint32_t DLC, uint8_t aData[]);

void StartStopEngineStateControl(void * pvParameters);
void StartStopEngineStarterTimingControl(void *pvParameters);
void StartStopEngineInit(void);
void StartStopEngineButtonHandle(void *pvParameters);

#endif /* SSE_TASKS_H */