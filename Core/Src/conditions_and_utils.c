#include "fvc.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"
#include "git_info.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>

// Inverter CAN Info
U8_CAN_STRUCT *inv_enable_fbk_statuses[] = {&driveEnableInvStatus_FL_state, &driveEnableInvStatus_FR_state, &driveEnableInvStatus_RL_state, &driveEnableInvStatus_RR_state};
U8_CAN_STRUCT *inv_fault_codes[]         = {&faultCode_FL, &faultCode_FR, &faultCode_RL, &faultCode_RR};

// Status Defines
bool Hbeat_LED_status = OFF;

// Lowpass Filter
LOWPASS_FILTER APPS1_LPF;
LOWPASS_FILTER APPS2_LPF;
LOWPASS_FILTER speed_LPF_FL;
LOWPASS_FILTER speed_LPF_FR;
LOWPASS_FILTER speed_LPF_RL;
LOWPASS_FILTER speed_LPF_RR;
LOWPASS_FILTER steering_angle_LPF;

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
	if(bmsFault_state.data)
		HAL_GPIO_WritePin(BMS_Dash_Light_GPIO_Port, BMS_Dash_Light_Pin, NMOS_ON);
	else
		HAL_GPIO_WritePin(BMS_Dash_Light_GPIO_Port, BMS_Dash_Light_Pin, NMOS_OFF);

	if(imdFault_state.data)
		HAL_GPIO_WritePin(IMD_Dash_Light_GPIO_Port, IMD_Dash_Light_Pin, NMOS_ON);
	else
		HAL_GPIO_WritePin(IMD_Dash_Light_GPIO_Port, IMD_Dash_Light_Pin, NMOS_OFF);	
}

void init_git_LEDs(){
	if(CURRENT_BRANCH_TYPE == BRANCH_TYPE_MAIN){
		HAL_GPIO_WritePin(GIT_Main_GPIO_Port, GIT_Main_Pin,    ON);
		HAL_GPIO_WritePin(GIT_Feature_GPIO_Port, GIT_Feature_Pin, OFF);
		HAL_GPIO_WritePin(GIT_Other_GPIO_Port, GIT_Other_Pin,  OFF);
	}
	else if (CURRENT_BRANCH_TYPE == BRANCH_TYPE_FEATURE){
		HAL_GPIO_WritePin(GIT_Main_GPIO_Port, GIT_Main_Pin,    OFF);
		HAL_GPIO_WritePin(GIT_Feature_GPIO_Port, GIT_Feature_Pin, ON);
		HAL_GPIO_WritePin(GIT_Other_GPIO_Port, GIT_Other_Pin,  OFF);
	}
	else{
		HAL_GPIO_WritePin(GIT_Main_GPIO_Port, GIT_Main_Pin,    OFF);
		HAL_GPIO_WritePin(GIT_Feature_GPIO_Port, GIT_Feature_Pin, OFF);
		HAL_GPIO_WritePin(GIT_Other_GPIO_Port, GIT_Other_Pin,  ON);
	}
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

	return clamp(dc_current_limit_A, 0, 100.0f);
}

// AC Current Limit Calculation
float calculate_ac_current_limit(){
	float max_AC_inv_limit;
	if(drive_speed_mode == SLOW)
		max_AC_inv_limit = TOTAL_INVERTER_MAX_CURRENT_Apk / 3;
	else
		max_AC_inv_limit = TOTAL_INVERTER_MAX_CURRENT_Apk;
	
	return max_AC_inv_limit;
}

// Determine Drive Power Limits
void determine_drive_power_limits(float *ac_limit_Apk, float *dc_limit_A, bool *drive_enable){
	if (vehicle_state != VEHICLE_DRIVING){
		*ac_limit_Apk = 0;
		*dc_limit_A   = 0;
		*drive_enable  = FALSE; 
	}
	else{
		*ac_limit_Apk = get_rules_fault_state() ? 0 : calculate_ac_current_limit();
		*dc_limit_A   = calculate_dc_current_limit();
		*drive_enable  = TRUE; 
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

	if (time_stamp - driveEnableInvStatus_RL_state.info.last_rx >= INVERTER_TIMEOUT_ms){
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

void determine_drive_speed_mode(){
	static uint8_t last_button_press = 0;
	static uint32_t last_button_press_time = 0;
	static boolean mode_change_pending = 0;

	if(SLOW_MODE_BUTTON.data == 1 && last_button_press == 0){
		mode_change_pending = TRUE;
		last_button_press_time = HAL_GetTick();
	}

	if(SLOW_MODE_BUTTON.data == 0){
		mode_change_pending = FALSE;
	}

	if(mode_change_pending &&
		(SLOW_MODE_BUTTON.data == 1) &&
		(HAL_GetTick() - last_button_press_time > SLOW_MODE_HOLD_TIME_THRESH)){
		drive_speed_mode = (drive_speed_mode + 1) % 2;
		mode_change_pending = FALSE;
	}

	last_button_press = SLOW_MODE_BUTTON.data;
}

void determine_drive_model(){
    static uint8_t last_button_press = 0;
    static uint32_t last_button_press_time = 0;
    static boolean mode_change_pending = FALSE;

    static uint8_t selectable_mode_index = 0;
    static boolean last_GNSS_fixed = FALSE;

    static const DRIVE_MODEL_MODES_t gnss_fixed_modes[] = {
        TORQUE_BY_4,
        TORQUE_BY_2_FWD,
        TORQUE_BY_2_RWD_RL,
        OPEN_DIFF_NO_PID,
        OPEN_DIFF,
        TORQUE_VECTORING
    };

    static const DRIVE_MODEL_MODES_t gnss_not_fixed_modes[] = {
        TORQUE_BY_4,
        TORQUE_BY_2_FWD,
        TORQUE_BY_2_RWD_RL
    };

    boolean GNSS_fixed = (vnavINS_status.data & (1 << 2)) > 0;

    const DRIVE_MODEL_MODES_t *selectable_drive_modes;
    uint8_t num_selectable_drive_modes;

    if (GNSS_fixed) {
        selectable_drive_modes = gnss_fixed_modes;
        num_selectable_drive_modes = sizeof(gnss_fixed_modes) / sizeof(gnss_fixed_modes[0]);
    } else {
        selectable_drive_modes = gnss_not_fixed_modes;
        num_selectable_drive_modes = sizeof(gnss_not_fixed_modes) / sizeof(gnss_not_fixed_modes[0]);
    }

    // If GNSS state changed, make sure current drive_model is valid in the new allowed list
    if (GNSS_fixed != last_GNSS_fixed) {
        boolean current_mode_valid = FALSE;

        for (uint8_t i = 0; i < num_selectable_drive_modes; i++) {
            if (drive_model == selectable_drive_modes[i]) {
                selectable_mode_index = i;
                current_mode_valid = TRUE;
                break;
            }
        }

        if (!current_mode_valid) {
            drive_model = TORQUE_BY_4;
            selectable_mode_index = 0;
        }

        mode_change_pending = FALSE;
        last_GNSS_fixed = GNSS_fixed;
    }

	// button hold logic
    if (DRIVE_MODEL_BUTTON.data == 1 && last_button_press == 0) {
        mode_change_pending = TRUE;
        last_button_press_time = HAL_GetTick();
    }

    if (DRIVE_MODEL_BUTTON.data == 0) {
        mode_change_pending = FALSE;
    }

    if (mode_change_pending &&
        (DRIVE_MODEL_BUTTON.data == 1) &&
        (HAL_GetTick() - last_button_press_time > DRIVE_MODEL_HOLD_TIME_THRESH)) {

        selectable_mode_index =
            (selectable_mode_index + 1) % num_selectable_drive_modes;

        drive_model = selectable_drive_modes[selectable_mode_index];

        mode_change_pending = FALSE;
    }

    last_button_press = DRIVE_MODEL_BUTTON.data;
}

// Lowpass Filtering
float LPF_compute_alpha(float cutoff_freq_Hz, float sample_time_s){
    if (cutoff_freq_Hz <= 0.0f || sample_time_s <= 0.0f)
    {
        return 1.0f; // No filtering / passthrough fallback
    }

    float tau = 1.0f / (2.0f * (float)M_PI * cutoff_freq_Hz);

    return sample_time_s / (tau + sample_time_s);
}

void LPF_init(LOWPASS_FILTER *filter, float cutoff_freq_Hz, float sample_time_s){
    if (filter == NULL)
    {
        return;
    }

    filter->cutoff_freq_Hz = cutoff_freq_Hz;
    filter->sample_time_s  = sample_time_s;
    filter->alpha = LPF_compute_alpha(cutoff_freq_Hz, sample_time_s);

    filter->prev_output = 0.0f;
    filter->initialized = false;
}

float LPF(LOWPASS_FILTER *filter, float input){
    if (filter == NULL)
    {
        return input;
    }

    if (!filter->initialized)
    {
        filter->prev_output = input;
        filter->initialized = true;
        return input;
    }

    float output = filter->alpha * input
                 + (1.0f - filter->alpha) * filter->prev_output;

    filter->prev_output = output;

    return output;
}