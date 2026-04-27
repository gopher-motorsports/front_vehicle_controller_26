#include "fvc.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>

// Inverter CAN Info
U8_CAN_STRUCT *inv_enable_fbk_statuses[] = {&driveEnableInvStatus_FL_state, &driveEnableInvStatus_FR_state, &driveEnableInvStatus_RL_state, &driveEnableInvStatus_RR_state};
U8_CAN_STRUCT *inv_fault_codes[]         = {&faultCode_FL, &faultCode_FR, &faultCode_RL, &faultCode_RR};

// ========================================LED Functions======================================================
//Heartbeat LED
void LED_task(){
	static uint32_t last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(HBeat_GPIO_Port, HBeat_Pin);
		last_led = HAL_GetTick();
	}
}

// Dash Light Functionality:
// If AMS(means BMS) fault --> BMS light goes on
// If IMD Fault --> IMD Fault Goes on
void set_dash_lights(){
	if(amsFault_state.data)
		HAL_GPIO_WritePin(BMS_Dash_Light_GPIO_Port, BMS_Dash_Light_Pin, NMOS_ON);
	else
		HAL_GPIO_WritePin(BMS_Dash_Light_GPIO_Port, BMS_Dash_Light_Pin, NMOS_OFF);

	if(imdFault_state.data)
		HAL_GPIO_WritePin(IMD_Dash_Light_GPIO_Port, IMD_Dash_Light_Pin, NMOS_ON);
	else
		HAL_GPIO_WritePin(IMD_Dash_Light_GPIO_Port, IMD_Dash_Light_Pin, NMOS_OFF);	
}
// ==============================================================================================

// ======================================== General Utilities ======================================================
int16_t max4Ints(int16_t a, int16_t b, int16_t c, int16_t d) {
    int16_t max = a;
    if (b > max) max = b;
    if (c > max) max = c;
    if (d > max) max = d;
    return max;
}

// ==============================================================================================

// ======================================== Rules Required Faults ======================================================
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
	fault_tripped |= rvcBspdInputFault_state.data;

	// APPS/Brake Plausibility Fault:
	// When Brake + Accelerator pushed at same time
	bool appsBrakeLatched_state;
	if(fvcBrakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && fvcPedalPosition1_percent.data > 25) {
		fvcBothPedalsPressedFault_state.data = TRUE;
		appsBrakeLatched_state = TRUE;
	} else if (fvcPedalPosition1_percent.data <= 5) {
		fvcBothPedalsPressedFault_state.data = FALSE;
		appsBrakeLatched_state = FALSE;
	}
	fault_tripped |= appsBrakeLatched_state;

	return fault_tripped;
}
// ==============================================================================================



// ======================================== Inverter Limit Functions ======================================================

// DC Current Limit Calculation
float calculate_dc_current_limit(){
	float dc_current_limit_A = 0;

	// BSPD Tractive Brake Fault tripped(if this lasts for .5s car the BSPD fault is tripped and HV is shut off)
	if(rvcBspdRunawayFault_state.data){
		int16_t current_max_inv_voltage = max4Ints(inputInverterVoltage_FL_V.data,
								   inputInverterVoltage_FR_V.data,
								   inputInverterVoltage_RL_V.data,
								   inputInverterVoltage_RR_V.data);
		if(current_max_inv_voltage != 0)
			dc_current_limit_A = BSPD_POWER_LIMIT / inputInverterVoltage_FL_V.data; //stay below 5 kW I = P/V
	} else {
		dc_current_limit_A = DC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_A;
	}
	return dc_current_limit_A;
}

void determine_current_limits(float *ac_currentLimit_Apk, float *dc_currentlimit_A, VEHICLE_STATE_t state){
	if (state != VEHICLE_DRIVING){
		*ac_currentLimit_Apk = 0;
		*dc_currentlimit_A = 0;
	}
	else{
		*ac_currentLimit_Apk = is_vehicle_faulting() ? 0 : AC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_Apk;
		*dc_currentlimit_A = calculate_dc_current_limit();
	}
}

// ======================================== Inverter Condition Functions ======================================================
bool predrive_conditions_met(){
	bool predrive_conditions = (fvcBrakePressureFront_psi.data >= PREDRIVE_BRAKE_THRESH_psi);

	predrive_conditions &= (PREDRIVE_BUTTON_PARAM.data == PRESSED);
	
	// Check that predrive voltages are high enough
	predrive_conditions &= inputInverterVoltage_FL_V.data >= TS_ON_THRESHOLD_VOLTAGE_V;
	predrive_conditions &= inputInverterVoltage_FR_V.data >= TS_ON_THRESHOLD_VOLTAGE_V;
	predrive_conditions &= inputInverterVoltage_RL_V.data >= TS_ON_THRESHOLD_VOLTAGE_V;
	predrive_conditions &= inputInverterVoltage_RR_V.data >= TS_ON_THRESHOLD_VOLTAGE_V;

	return predrive_conditions;
}

// Maybe return like a status struct that has 4 booleans and tells you if you have
// Comms from any interver
// Comms from front 2 inverters
// Comms from rear 2 inverters
// Comms from all 4 inverters
// this would be something nice to log in data also 
typdef struct { 
	bool any_comms;
	bool front_comms;
	bool rear_comms;
	bool all_comms;
} InverterComms_Status_t;

InverterComms_Status_t get_inverter_comms_status(){
	uint32_t time_stamp = HAL_GetTick();

	InverterComms_Status_t status = {0};

	bool fl_ok = (time_stamp - inv_enable_fbk_statuses[0]->info.last_rx < INVERTER_TIMEOUT_ms);
	bool fr_ok = (time_stamp - inv_enable_fbk_statuses[1]->info.last_rx < INVERTER_TIMEOUT_ms);
	bool rl_ok = (time_stamp - inv_enable_fbk_statuses[2]->info.last_rx < INVERTER_TIMEOUT_ms);
	bool rr_ok = (time_stamp - inv_enable_fbk_statuses[3]->info.last_rx < INVERTER_TIMEOUT_ms);
	
	status.any_comms = (fl_ok || fr_ok || rl_ok || rr_ok);
	status.front_comms = (fl_ok && fr_ok);
	status.rear_comms = (rr_ok && rl_ok);
	status.all_comms = (status.front_comms && status.rear_comms);

	return status;

}
typdef struct {
	bool any;
	bool rear;
	bool front;
	bool all;
} InverterFault_Status_t

InverterFault_Status_t get_Inverter_Fault_Status(){
	InverterFault_Status_t fStatus = {0};
	bool fl_Status = ((inv_fault_codes[0]->data & INVERTER_OV_FAULT) || (inv_fault_codes[0]->data & INVERTER_UV_FAULT) || (inv_fault_codes[0]->data & INVERTER_CTRL_TEMP_FAULT) || (inv_fault_codes[0]->data & INVERTER_MOTOR_TEMP_FAULT));
	bool fr_Status = ((inv_fault_codes[1]->data & INVERTER_OV_FAULT) || (inv_fault_codes[1]->data & INVERTER_UV_FAULT) || (inv_fault_codes[1]->data & INVERTER_CTRL_TEMP_FAULT) || (inv_fault_codes[1]->data & INVERTER_MOTOR_TEMP_FAULT));
	bool rl_Status = ((inv_fault_codes[2]->data & INVERTER_OV_FAULT) || (inv_fault_codes[2]->data & INVERTER_UV_FAULT) || (inv_fault_codes[2]->data & INVERTER_CTRL_TEMP_FAULT) || (inv_fault_codes[2]->data & INVERTER_MOTOR_TEMP_FAULT));
	bool fl_Status = ((inv_fault_codes[3]->data & INVERTER_OV_FAULT) || (inv_fault_codes[3]->data & INVERTER_UV_FAULT) || (inv_fault_codes[3]->data & INVERTER_CTRL_TEMP_FAULT) || (inv_fault_codes[3]->data & INVERTER_MOTOR_TEMP_FAULT));

	fStatus.any = (fl_Status || fr_Status || rl_Status || rr_Status);
	fStatus.front = (fl_Status && fr_Status);
	fStatus.rear = (rl_Status && rr_Status);
	fStatus.all = (fStatus.front && fStatus.rear);

	return fStatus;

}


bool has_inverter_comms(){
	uint32_t time_stamp = HAL_GetTick();
	bool has_comms = TRUE;

	for(int i = 0; i < TOTAL_INVERTERS; i++){
		if (time_stamp - inv_enable_fbk_statuses[i]->info.last_rx >= INVERTER_TIMEOUT_ms)
			has_comms = FALSE;
	}

	return has_comms;
}

bool inverter_fault_active(){
	bool fault_active = FALSE;
	for(int i = 0; i < TOTAL_INVERTERS; i++){
		bool overvoltage_fault = inv_fault_codes[i]->data & INVERTER_OV_FAULT;
		bool undervoltage_fault = inv_fault_codes[i]->data & INVERTER_UV_FAULT;
		bool controller_overtemp = inv_fault_codes[i]->data & INVERTER_CTRL_TEMP_FAULT;
		bool motor_overtemp = inv_fault_codes[i]->data & INVERTER_MOTOR_TEMP_FAULT;

		if(overvoltage_fault || undervoltage_fault || controller_overtemp || motor_overtemp)
			fault_active = TRUE;
	} 

	return fault_active;
}