#include "fvc_software_faults.h"
#include "fvc.h"
#include "GopherCAN.h"
#include "gopher_sense.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>


// Rules Required Faults Variables
bool both_pedals_fault_state;
bool rules_fault_state;

// ======================================= Rules Required Defines ======================================
//APPS1 Out of Range Check
SOFTWARE_FAULT APPS1_Range_Fault = {
    .data = 0,
	.max_threshold = APPS_MAX_ERROR_POS_mm,
	.min_threshold = APPS_MIN_ERROR_POS_mm,
	.fault_timer = 0,
	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
	.state = false
};

//APPS2 Out of Range Check
SOFTWARE_FAULT APPS2_Range_Fault = {
    .data = 0,
	.max_threshold = APPS_MAX_ERROR_POS_mm,
	.min_threshold = APPS_MIN_ERROR_POS_mm,
	.fault_timer = 0,
	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
	.state = false
};

SOFTWARE_FAULT Pedal_Correlation_Fault = {
    .data = 0, //float absolute value
	.max_threshold = APPS_CORRELATION_THRESH_percent,
	.min_threshold = -FLT_MIN, //not using this one, put in smallest mimumum value of a float so won't trigger
	.fault_timer = 0,
	.input_delay_threshold = INPUT_TRIP_DELAY_ms,
	.state = false
};

SOFTWARE_FAULT* TIMED_SOFTWARE_FAULTS[NUM_OF_TIMED_FAULTS] = {
    &APPS1_Range_Fault,
    &APPS2_Range_Fault,
    &Pedal_Correlation_Fault
};

// ====================================================================================================

// ======================================= Efuse Defines ==============================================
FVC_LOW_POWER_CHANNEL SNS_3V3_Chan = {
	.enable_gpio = {
		.port = SNS_3V3_EN_GPIO_Port,
		.pin  = SNS_3V3_EN_Pin
	},
    .fault_gpio = {
		.port = SNS_3V3_Flt_GPIO_Port,
		.pin  = SNS_3V3_Flt_Pin
	},
    .fault_LED_gpio = {
		.port = SNS_3V3_Flt_LED_GPIO_Port,
		.pin  = SNS_3V3_Flt_LED_Pin
	},
	.hardware_state = LOW_POW_EFUSE_NO_FLT,
	.mode = NORMAL,
    .flt_state = FALSE,     
	.trip_start_ms = 0,
    .retry_delay_ms = EFUSE_RETRY_DELAY_ms,
	.retries_left = TOTAL_EFUSE_RETRIES
};

FVC_LOW_POWER_CHANNEL SNS1_5V_Chan = {
	.enable_gpio = {
		.port = SNS_5V_EN1_GPIO_Port,
		.pin  = SNS_5V_EN1_Pin
	},
    .fault_gpio = {
		.port = SNS_5V_Flt_1_GPIO_Port,
		.pin  = SNS_5V_Flt_1_Pin
	},
    .fault_LED_gpio = {
		.port = SNS_5V_Flt_1_LED_GPIO_Port,
		.pin  = SNS_5V_Flt_1_LED_Pin
	},
    .mode = NORMAL,
	.hardware_state = LOW_POW_EFUSE_NO_FLT,
    .flt_state = FALSE,     
	.trip_start_ms = 0,
    .retry_delay_ms = EFUSE_RETRY_DELAY_ms,
	.retries_left = TOTAL_EFUSE_RETRIES
};

FVC_LOW_POWER_CHANNEL SNS2_5V_Chan = {
	.enable_gpio = {
		.port = SNS_5V_EN2_GPIO_Port,
		.pin  = SNS_5V_EN2_Pin
	},
    .fault_gpio = {
		.port = SNS_5V_Flt_2_GPIO_Port,
		.pin  = SNS_5V_Flt_2_Pin
	},
    .fault_LED_gpio = {
		.port = SNS_5V_Flt_2_LED_GPIO_Port,
		.pin  = SNS_5V_Flt_2_LED_Pin
	},
    .mode = NORMAL,
	.hardware_state = LOW_POW_EFUSE_NO_FLT,
    .flt_state = FALSE,     
	.trip_start_ms = 0,
    .retry_delay_ms = EFUSE_RETRY_DELAY_ms,
	.retries_left = TOTAL_EFUSE_RETRIES
};

FVC_LOW_POWER_CHANNEL SWM_5V_Chan = {
	.enable_gpio = {
		.port = SWM_5V_EN_GPIO_Port,
		.pin  = SWM_5V_EN_Pin
	},
    .fault_gpio = {
		.port = SWM_5V_Flt_GPIO_Port,
		.pin  = SWM_5V_Flt_Pin
	},
    .fault_LED_gpio = {
		.port = SWM_Flt_LED_GPIO_Port,
		.pin  = SWM_Flt_LED_Pin
	},
    .mode = NORMAL,
	.hardware_state = LOW_POW_EFUSE_NO_FLT,
    .flt_state = FALSE,     
	.trip_start_ms = 0,
    .retry_delay_ms = EFUSE_RETRY_DELAY_ms,
	.retries_left = TOTAL_EFUSE_RETRIES
};

FVC_LOW_POWER_CHANNEL* FVC_LOW_POWER_CHANNELS[] = {
    &SNS_3V3_Chan,
    &SNS1_5V_Chan,
    &SNS2_5V_Chan,
    &SWM_5V_Chan
};


FVC_HIGH_POWER_CHANNEL SNS_12V_Chan = {
	.enable_gpio = {
		.port = SNS_5V_EN2_GPIO_Port,
		.pin  = SNS_5V_EN2_Pin
	},
    .fault_gpio = {
		.port = SNS_5V_Flt_2_GPIO_Port,
		.pin  = SNS_5V_Flt_2_Pin
	},
    .fault_LED_gpio = {
		.port = SNS_5V_Flt_2_LED_GPIO_Port,
		.pin  = SNS_5V_Flt_2_LED_Pin
	},
    .mode = NORMAL,
	.hardware_state = LOW_POW_EFUSE_NO_FLT,
    .flt_state = FALSE,     
	.trip_start_ms = 0,
    .retry_delay_ms = EFUSE_RETRY_DELAY_ms,
	.retries_left = TOTAL_EFUSE_RETRIES,
	.amp_max 	  = SNS_12V_CURRENT_LIMIT_A,
	.amp_can_param	  = &fvcEfuse12VSNSCurrent_A
};

FVC_HIGH_POWER_CHANNEL DISP_12V_Chan = {
	.enable_gpio = {
		.port = SWM_5V_EN_GPIO_Port,
		.pin  = SWM_5V_EN_Pin
	},
    .fault_gpio = {
		.port = SWM_5V_Flt_GPIO_Port,
		.pin  = SWM_5V_Flt_Pin
	},
    .fault_LED_gpio = {
		.port = SWM_Flt_LED_GPIO_Port,
		.pin  = SWM_Flt_LED_Pin
	},
    .mode = NORMAL,
	.hardware_state = LOW_POW_EFUSE_NO_FLT,
    .flt_state = FALSE,     
	.trip_start_ms = 0,
    .retry_delay_ms = EFUSE_RETRY_DELAY_ms,
	.retries_left = TOTAL_EFUSE_RETRIES,
	.amp_max 	  = DISPLAY_CURRENT_LIMIT_A,
	.amp_can_param	  = &fvcEfuse12VDispCurrent_A
};

FVC_HIGH_POWER_CHANNEL* FVC_HIGH_POWER_CHANNELS[] = {
    &SNS_12V_Chan,
    &DISP_12V_Chan
};

void update_fault_data(){
	// Rules Required Data
	APPS1_Range_Fault.data = fvcPedalPosition1_mm.data;
	APPS2_Range_Fault.data = fvcPedalPosition2_mm.data;
	Pedal_Correlation_Fault.data = fabsf(fvcPedalPosition1_percent.data - fvcPedalPosition2_percent.data);

	// Efuse Data
	// Low Power
	for(int i = 0; i < NUM_OF_LOW_POWER_CHANNELS;  i++){
		FVC_LOW_POWER_CHANNEL *efuse = FVC_LOW_POWER_CHANNELS[i];
		efuse->hardware_state = HAL_GPIO_ReadPin(efuse->fault_gpio.port, efuse->fault_gpio.pin);
	}

	// High Power
	for(int i = 0; i < NUM_OF_HIGH_POWER_CHANNELS;  i++){
		FVC_HIGH_POWER_CHANNEL *efuse = FVC_HIGH_POWER_CHANNELS[i];
		efuse->hardware_state = HAL_GPIO_ReadPin(efuse->fault_gpio.port, efuse->fault_gpio.pin);
	}
}


// Rules Required Faults
void update_rules_fault_state(){
	bool rules_fault_state_local = false;

	// Inclues APPS1 Range Fault, APPS2 Range Fault, Correlation Check
	SOFTWARE_FAULT* fault;
	for(int i = 0; i < NUM_OF_TIMED_FAULTS; i++){
		fault = TIMED_SOFTWARE_FAULTS[i];
		bool data_too_high = fault->data > fault->max_threshold;
		bool data_too_low  = fault->data < fault->min_threshold;

		if(data_too_low || data_too_high){ //correlation has no min, but edge case accounted for in defines
			fault->fault_timer++;
			if(fault->fault_timer >= fault->input_delay_threshold){
				fault->fault_timer = fault->input_delay_threshold; //cap at delay_threshold so no unnecesary counts
				fault->state = true;
			}
		}
		else{
			fault->fault_timer = 0;
			fault->state = false;
		}
		
		rules_fault_state_local |= fault->state;
	}

	//Input Fault:
	// Includes Current Sensor & Rear Brake Pressure Range Fault
	rules_fault_state_local |= rvcBspdInputFault_state.data;

	// APPS/Brake Plausibility Fault:
	// When Brake + Accelerator pushed at same time
	if(fvcBrakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && fvcPedalPosition1_percent.data > 25) {
		both_pedals_fault_state = TRUE;
	} else if (fvcPedalPosition1_percent.data <= 5) {
		both_pedals_fault_state = FALSE;
	}

	rules_fault_state_local |= both_pedals_fault_state;

	// Local copy made so critical section is small
	rules_fault_state = rules_fault_state_local;
	
}

bool get_both_pedals_fault_state(){
	return both_pedals_fault_state;
}

bool get_rules_fault_state(){
	return rules_fault_state;
}

// ======================================= Efuse Faults ======================================
void init_efuses(){
	for(int i = 0; i < NUM_OF_LOW_POWER_CHANNELS;  i++){
		FVC_LOW_POWER_CHANNEL *efuse = FVC_LOW_POWER_CHANNELS[i];
		HAL_GPIO_WritePin(efuse->enable_gpio.port, efuse->enable_gpio.pin, LOW_POW_EFUSE_ENABLED);
		efuse->enabled = TRUE;
	}

	// High Power
	for(int i = 0; i < NUM_OF_HIGH_POWER_CHANNELS;  i++){
		FVC_HIGH_POWER_CHANNEL *efuse = FVC_HIGH_POWER_CHANNELS[i];
		HAL_GPIO_WritePin(efuse->enable_gpio.port, efuse->enable_gpio.pin, HIGH_POW_EFUSE_ENABLED);
		efuse->enabled = TRUE;
	}
}

void update_low_power_efuses(){
	for(int i = 0; i < NUM_OF_LOW_POWER_CHANNELS;  i++){
		bool ready_to_retry;
		bool retries_left;
		FVC_LOW_POWER_CHANNEL *efuse = FVC_LOW_POWER_CHANNELS[i];

		switch(efuse->mode){
			case NORMAL:
				if (efuse->hardware_state == LOW_POW_EFUSE_FLT){
					efuse->flt_state = TRUE;
					HAL_GPIO_WritePin(efuse->fault_LED_gpio.port, efuse->fault_LED_gpio.pin, ON);
					efuse->trip_start_ms = HAL_GetTick();
					efuse->mode = TRIPPED; 
				}
				break;
			case TRIPPED:
				ready_to_retry = (HAL_GetTick() - efuse->trip_start_ms) > efuse->retry_delay_ms;
				retries_left = efuse->retries_left > 0;
				if(!retries_left){
					efuse->mode = SHUTDOWN; 
				}
				else if (ready_to_retry){
					efuse->flt_state = FALSE;
					HAL_GPIO_WritePin(efuse->fault_LED_gpio.port, efuse->fault_LED_gpio.pin, OFF);
					efuse->retries_left--;
					efuse->mode = NORMAL;
				}
				break;
			default:
			case SHUTDOWN:
				// fuse is permanently open for this power cycle
				break;	
		}
	}
}

void update_high_power_efuses(){
	for(int i = 0; i < NUM_OF_HIGH_POWER_CHANNELS;  i++){
		bool ready_to_retry;
		bool retries_left;
		FVC_HIGH_POWER_CHANNEL *efuse = FVC_HIGH_POWER_CHANNELS[i];

		switch(efuse->mode){
			case NORMAL:
				// thermal shutdown
				if (efuse->hardware_state == HIGH_POW_EFUSE_FLT){
					efuse->flt_state = TRUE;
					efuse->retries_left = 0;
					HAL_GPIO_WritePin(efuse->fault_LED_gpio.port, efuse->fault_LED_gpio.pin, ON);
					efuse->mode = SHUTDOWN;
				}
				if (efuse->amp_can_param->data > efuse->amp_max){
					efuse->flt_state = TRUE;
					HAL_GPIO_WritePin(efuse->fault_LED_gpio.port, efuse->fault_LED_gpio.pin, ON);
					efuse->trip_start_ms = HAL_GetTick();
					efuse->mode = TRIPPED; 
				}
				break;
			case TRIPPED:
				ready_to_retry = (HAL_GetTick() - efuse->trip_start_ms) > efuse->retry_delay_ms;
				retries_left = efuse->retries_left > 0;
				if(!retries_left){
					efuse->mode = SHUTDOWN; 
				}
				else if (ready_to_retry){
					efuse->flt_state = FALSE;
					HAL_GPIO_WritePin(efuse->fault_LED_gpio.port, efuse->fault_LED_gpio.pin, OFF);
					efuse->retries_left--;
					efuse->mode = NORMAL;
				}
				break;
			default:
			case SHUTDOWN:
				// fuse is permanently open for this power cycle
				break;	
		}
	}
}