#include "fvc.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"
//Heartbeat LED
void LED_task(){
	static uint32_t last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(HBeat_GPIO_Port, HBeat_Pin);
		last_led = HAL_GetTick();
	}
}

// DC Current Limit Calculation
float calculate_dc_current_limit(){
	float dc_current_limit_A = 0;
	// BSPD Tractive Brake Fault tripped(if this lasts for .5s car the BSPD fault is tripped and HV is shut off)
	if(bspdTractiveSystemBrakingFault_state.data){
		if(inputInverterVoltage_V.data != 0)
			dc_current_limit_A = BSPD_POWER_LIMIT / inputInverterVoltage_V.data / TOTAL_INVERTERS; //stay below 5 kW I = P/V
	} else {
		dc_current_limit_A = MAX_DC_CURRENT_LIMIT;
	}
	return dc_current_limit_A;
}


bool is_vehicle_faulting(){
	//pulled high when a fault is tripped, intialized to 0
	bool fault_tripped = 0;

	// Inclues APPS1 Range Fault, APPS2 Range Fault, Correlation Check
	SOFTWARE_FAULT* fault;
	for(int i = 0; i < NUM_OF_TIMED_FAULTS; i++){
		fault = TIMED_SOFTWARE_FAULTS[i];
		if(fault->data > fault->max_threshold || fault->data < fault->min_threshold){ //correlation has no min, but edge case accounted for in defines
			fault->fault_timer++;
			if(fault->fault_timer > fault->input_delay_threshold)
				fault->fault_timer = fault->input_delay_threshold + 1; //cap at delay_threshold + 1 so that it trips but doesn't count up more
		}
		else{
			fault->fault_timer = 0;
			fault->state = false;
		}

		if(fault->fault_timer > fault->input_delay_threshold){
			fault->state = true;
		}
		fault->can_param->data = fault->state;
		fault_tripped |= fault->state;
	}

	//Input Fault:
	// Inclues Current Sensor & Rear Brake Pressure Range Fault
	fault_tripped |= bspdInputFault_state.data;

	// APPS/Brake Plausibility Fault:
	// When Brake + Accelerator pushed at same time
	bool appsBrakeLatched_state;
	if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_percent.data > 25) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_percent.data <= 5) {
		appsBrakeLatched_state = FALSE;
	}
	fault_tripped |= appsBrakeLatched_state;

	return fault_tripped;
}