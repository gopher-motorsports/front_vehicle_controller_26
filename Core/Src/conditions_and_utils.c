#include "fvc.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>

// Inverter CAN Info
U8_CAN_STRUCT *inv_enable_fbk_statuses[] = {&driveEnableInvStatus_FL_state, &driveEnableInvStatus_FR_state, &driveEnableInvStatus_RL_state, &driveEnableInvStatus_RR_state};
U8_CAN_STRUCT *inv_fault_codes[]         = {&faultCode_FL, &faultCode_FR, &faultCode_RL, &faultCode_RR};

// Status Defines
bool Hbeat_LED_status = OFF;

// ========================================LED Functions======================================================
//Heartbeat LED
void hbeat_blink(){
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

// ======================================== Inverter Limit Functions ======================================================

// DC Current Limit Calculation
float calculate_dc_current_limit(){
	float dc_current_limit_A = 0;

	int16_t current_max_inv_voltage = max4Ints(inputInverterVoltage_FL_V.data,
								   inputInverterVoltage_FR_V.data,
								   inputInverterVoltage_RL_V.data,
								   inputInverterVoltage_RR_V.data);

	// BSPD Tractive Brake Fault tripped(if this lasts for .5s car the BSPD fault is tripped and HV is shut off)
	if(rvcBspdRunawayFault_state.data){
		if(current_max_inv_voltage > 0)
			dc_current_limit_A = BSPD_POWER_LIMIT_W / (float) current_max_inv_voltage; //stay below 5 kW I = P/V
	} else {
		if(current_max_inv_voltage > 0)
			dc_current_limit_A = RULES_POWER_LIMIT_W / current_max_inv_voltage;
		else
			dc_current_limit_A = DC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_A;
	}
	return dc_current_limit_A;
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
		uint8_t code = inv_fault_codes[i]->data;

        if (code == INVERTER_OV_FAULT ||
            code == INVERTER_UV_FAULT ||
            code == INVERTER_CTRL_TEMP_FAULT ||
            code == INVERTER_MOTOR_TEMP_FAULT) {
            fault_active = TRUE;
        }
	} 

	return fault_active;
}

bool get_slow_mode_status(){
	return slow_mode;
}