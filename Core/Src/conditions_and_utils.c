#include "fvc.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>

// Inverter CAN Info
U8_CAN_STRUCT *inv_enable_fbk_statuses[] = {&driveEnableInvStatus_FL_state, &driveEnableInvStatus_FR_state, &driveEnableInvStatus_RL_state, &driveEnableInvStatus_RR_state};
U8_CAN_STRUCT *inv_fault_codes[]         = {&faultCode_FL, &faultCode_FR, &faultCode_RL, &faultCode_RR};
bool appsBrakeLatched_state;

// Status Defines
bool Hbeat_LED_status = OFF;

// Drive Defines
bool slow_mode = OFF;

// ========================================LED Functions======================================================
//Heartbeat LED
void Hbeat_blink(){
	static uint32_t last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(HBeat_GPIO_Port, HBeat_Pin);
		Hbeat_LED_status = !(Hbeat_LED_status);
		last_led = HAL_GetTick();
	}
}

bool get_Hbeat_status(){
	return Hbeat_LED_status;
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

float clamp(float data, float min, float max){
	if(data < min){
		return min;
	} else if (data > max){
		return max;
	}

	return data;
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
	if(fvcBrakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && fvcPedalPosition1_percent.data > 25) {
		appsBrakeLatched_state = TRUE;
	} else if (fvcPedalPosition1_percent.data <= 5) {
		appsBrakeLatched_state = FALSE;
	}
	fault_tripped |= appsBrakeLatched_state;

	return fault_tripped;
}

bool get_both_pedals_fault_state(){
	return appsBrakeLatched_state;
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

void update_drive_control_inputs(bool *inverter_drive_enable){

	// Wheel Speeds
	open_diff_inputs.car_speed      = vnavVelBodyX.data;
	open_diff_inputs.wheel_speed_FL = fvcWheelSpeedFrontLeft_m_per_s.data;
	open_diff_inputs.wheel_speed_FR = fvcWheelSpeedFrontRight_m_per_s.data;
	open_diff_inputs.wheel_speed_RL = fvcWheelSpeedRearLeft_m_per_s.data;
	open_diff_inputs.wheel_speed_RR = fvcWheelSpeedRearRight_m_per_s.data;

	// Throttle
	open_diff_inputs.throttle_percent = fvcPedalPosition1_percent.data;

	// Current Limits + Enable
	float max_AC_inv_limit = (slow_mode) ? AC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_Apk : AC_CURRENT_SLOW_MODE_MAX_Apk;
	if (vehicle_state != VEHICLE_DRIVING){
		open_diff_inputs.ac_currentMaxLimit_Apk = 0;
		open_diff_inputs.dc_currentMaxlimit_A   = 0;
		*inverter_drive_enable = FALSE; 
	}
	else{
		open_diff_inputs.ac_currentMaxLimit_Apk = is_vehicle_faulting() ? 0 : max_AC_inv_limit;
		open_diff_inputs.dc_currentMaxlimit_A = calculate_dc_current_limit();
		*inverter_drive_enable = TRUE; 
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

bool get_slow_mode_status(){
	return slow_mode;
}