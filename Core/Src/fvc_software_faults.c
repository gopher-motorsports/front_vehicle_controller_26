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

// ======================================= Rules Required Faults ======================================
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

void update_fault_data(){
	APPS1_Range_Fault.data = fvcPedalPosition1_mm.data;
	APPS2_Range_Fault.data = fvcPedalPosition2_mm.data;
	Pedal_Correlation_Fault.data = fabsf(fvcPedalPosition1_percent.data - fvcPedalPosition2_percent.data);
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
			if(fault->fault_timer >= fault->input_delay_threshold)
				fault->fault_timer = fault->input_delay_threshold; //cap at delay_threshold so no unnecesary counts
				fault->state = true;
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
void update_efuse_fault_states(){
	// TODO efuse function
}