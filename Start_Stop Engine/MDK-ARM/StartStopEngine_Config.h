#ifndef SSE_CONFIG_H
#define SSE_CONFIG_H

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"


#define SSE_TIME_HANDLER    &htim2

#define SSE_BUTTON_PORT      			      StartStopEngineButton_GPIO_Port
#define SSE_BUTTON_PIN_NUMBER						StartStopEngineButton_Pin
	
#define SSE_ON_OFF_SWITCH_PORT        		StartStopFeatureSwitch_GPIO_Port
#define SSE_ON_OFF_SWITCH_PIN			  	    StartStopFeatureSwitch_Pin

#define SSE_STARTER_MOTOR_RELAY_PORT        StarterTimerControledSignal_GPIO_Port
#define SSE_STARTER_MOTOR_RELAY_PIN         StarterTimerControledSignal_Pin


#define SSE_START_STOP_FEATURE_SWITCH_PORT  StartStopFeatureSwitchFeedBack_GPIO_Port
#define SSE_START_STOP_FEATURE_SWITCH_PIN  	StartStopFeatureSwitchFeedBack_Pin


#define SSE_BREAK_SWITCH_PORT    		    BrakeSwitch_GPIO_Port
#define SSE_BREAK_SWITCH_PORT_PIN       BrakeSwitch_Pin



#define SSE_BUTTON_LED_PORT        		StartStopEngineButtonLed_N_GPIO_Port
#define SSE_BUTTON_LED_PIN						StartStopEngineButtonLed_N_Pin

#define SSE_USE_BREAK_SWITCH				TURN_OFF

#define SSE_STOP_VEHICLE_ON_HIGH_SPEED		TURN_ON

#define IDEAL_RPM_VALUE						1200
#define VEHICLE_STOP_MAX_SPPED			10
#define SSE_MAX_TRY_COUNT					1
#define SSE_LONG_PRESS_MAX_TIME				2
#define STARTERMOTOR_ON_TIME				3
#define STARTERMOTOR_HOLD_TIME				1



#endif /* SSE_CONFIG_H */