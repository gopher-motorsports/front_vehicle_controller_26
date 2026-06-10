#include "fvc.h"
#include "sensor_and_CAN.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"
#include "open_differential_no_PID.h"
#include "open_differential.h"
#include <math.h>
#include <string.h>

// HAL_CAN Structs
CAN_HandleTypeDef* CAN_CARSIDE;
CAN_HandleTypeDef* CAN_FRONT_INVERTERS;
CAN_HandleTypeDef* CAN_REAR_INVERTERS;

// HAL_UART Structs
UART_HandleTypeDef* UART_DEBUG;

// Inverter State Machine Defines:
uint32_t preDriveStart_ms;

uint32_t drive_control_start_tick_local;
uint32_t drive_control_end_tick_local;
uint32_t drive_timestep_number_local;

VEHICLE_STATE_t vehicle_state;
FVC_DRIVE_SENSOR_DATA fvc_drive_sensor_data_global;
DRIVE_CONTROL_SNAPSHOT drive_snapshot;

bool all_inverter_enable; // enable for all 4 inverters in this case
DRIVE_SPEED_MODE_t drive_speed_mode = FULL_POWER;
DRIVE_MODEL_MODES_t drive_model     = TORQUE_BY_4;

TORQUE_BY_4_INPUTS tau_by_4_inputs = {
	.throttle_percent = 0.0,
	.tauMaxLimit_Nm = TOTAL_TORQUE_LIMIT_Nm,
	.ac_currentMaxLimit_Apk = 0.0,
	.dc_currentMaxlimit_A = 0.0
};

TORQUE_BY_4_OUTPUTS tau_by_4_outputs = {
	.tauFL_Nm = 0.0,
	.tauFR_Nm = 0.0,
	.tauRL_Nm = 0.0,
	.tauRR_Nm = 0.0,
	.tauTotalCMD_Nm = 0.0,
	.currentCMDFL_Apk = 0.0,
	.currentCMDFR_Apk = 0.0,
	.currentCMDRL_Apk = 0.0,
	.currentCMDRR_Apk = 0.0,
	.currentCMDTotal_Apk = 0.0
};

OPEN_DIFF_INPUTS open_diff_inputs = {
	.slip_tract_limit_decimal = TRACTION_LIMIT_decimal,
	.car_speed      = 0.0,
	.integral_reset  = 0,
	.P = 0.0,
	.I = 0.0,
	.fvc_drive_sensor_data = {
		.wheel_speed_FL = 0.0,
		.wheel_speed_FR = 0.0,
		.wheel_speed_RL = 0.0,
		.wheel_speed_RR = 0.0,
		.throttle_percent = 0.0
	},
	.tauMaxLimit_Nm = TOTAL_TORQUE_LIMIT_Nm,
	.ac_currentMaxLimit_Apk = 0.0,
	.dc_currentMaxlimit_A = 0.0,
};

OPEN_DIFF_OUTPUTS open_diff_outputs = {
	.slipFL_percent = 0.0,
	.slipFR_percent = 0.0,
	.slipRL_percent = 0.0,
	.slipRR_percent = 0.0,
	.is_FL_slipping = FALSE,
	.is_FR_slipping = FALSE,
	.is_RL_slipping = FALSE,
	.is_RR_slipping = FALSE,
	.tauFL_Nm = 0.0,
	.tauFR_Nm = 0.0,
	.tauRL_Nm = 0.0,
	.tauRR_Nm = 0.0,
	.tauTotalCMD_Nm = 0.0,
	.currentCMDFL_Apk = 0.0,
	.currentCMDFR_Apk = 0.0,
	.currentCMDRL_Apk = 0.0,
	.currentCMDRR_Apk = 0.0,
	.currentCMDTotal_Apk = 0.0
};

OPEN_DIFF_NO_PID_INPUTS  open_diff_no_pid_inputs = {
	.slip_tract_limit_decimal = TRACTION_LIMIT_decimal,
	.car_speed      = 0.0,
	.yaw_rate 		= 0.0,
	.fvc_drive_sensor_data = {
		.wheel_speed_FL = 0.0,
		.wheel_speed_FR = 0.0,
		.wheel_speed_RL = 0.0,
		.wheel_speed_RR = 0.0,
		.throttle_percent = 0.0
	},
	.tauMaxLimit_Nm = TOTAL_TORQUE_LIMIT_Nm,
	.ac_currentMaxLimit_Apk = 0.0,
	.dc_currentMaxlimit_A = 0.0,
};

OPEN_DIFF_NO_PID_OUTPUTS open_diff_no_pid_outputs = {
	.slipFL_percent = 0.0,
	.slipFR_percent = 0.0,
	.slipRL_percent = 0.0,
	.slipRR_percent = 0.0,
	.is_FL_slipping = FALSE,
	.is_FR_slipping = FALSE,
	.is_RL_slipping = FALSE,
	.is_RR_slipping = FALSE,
	.tauFL_Nm = 0.0,
	.tauFR_Nm = 0.0,
	.tauRL_Nm = 0.0,
	.tauRR_Nm = 0.0,
	.tauTotalCMD_Nm = 0.0,
	.currentCMDFL_Apk = 0.0,
	.currentCMDFR_Apk = 0.0,
	.currentCMDRL_Apk = 0.0,
	.currentCMDRR_Apk = 0.0,
	.currentCMDTotal_Apk = 0.0
};

// Debug Params
float fvc_debug_params[DEBUG_PARAM_COUNT];
float inv1_debug_params[DEBUG_PARAM_COUNT];
float inv2_debug_params[DEBUG_PARAM_COUNT];
float inv3_debug_params[DEBUG_PARAM_COUNT];
float inv4_debug_params[DEBUG_PARAM_COUNT];

// Init FVC
// What needs to happen on FVC startup 
void init_fvc(CAN_HandleTypeDef* BUS_1, CAN_HandleTypeDef* BUS_2, CAN_HandleTypeDef* BUS_3, UART_HandleTypeDef* huart_debug){
	CAN_CARSIDE = BUS_1;
	CAN_FRONT_INVERTERS = BUS_2;
	CAN_REAR_INVERTERS = BUS_3;

	init_can(CAN_CARSIDE, GCAN0);
	init_can(CAN_FRONT_INVERTERS, GCAN1);
	init_can(CAN_REAR_INVERTERS, GCAN2);

	UART_DEBUG = huart_debug;

	init_git_LEDs();
	init_efuses();
	open_differential_initialize();
}


// ======================================== FVC FreeRTOS Tasks ======================================================
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	service_can_rx_buffer();
	
	// handle the transmission hardware for each CAN bus
	service_can_tx(CAN_CARSIDE);
	service_can_tx(CAN_FRONT_INVERTERS);
	service_can_tx(CAN_REAR_INVERTERS);
}

void idle_task(){
	hbeat_blink();
}

void debug_task(){
	clear_serial_monitor();
	update_debug_params();
	send_debug_param_group("FVC",  fvc_debug_params);
    send_debug_param_group("INV1", inv1_debug_params);
    send_debug_param_group("INV2", inv2_debug_params);
	send_debug_param_group("INV3", inv3_debug_params);
    send_debug_param_group("INV4", inv4_debug_params);
}

void drive_task(){
	determine_drive_speed_mode();
	determine_drive_model();
	process_inverter();
}

void fault_task(){
	update_fault_data();
	update_rules_fault_state();
	update_low_power_efuses();
	update_high_power_efuses();
	set_dash_lights();
}

void telemetry_task(){
	update_non_ADC_CAN_params();
}

// ======================================== Inverter State Machine ======================================================
void process_inverter() {

	if (!has_inverter_comms())
		vehicle_state = VEHICLE_NO_COMMS;
	else if (inverter_fault_active()){
		vehicle_state = VEHICLE_FAULT;
	}

	switch (vehicle_state)
	{
	case VEHICLE_NO_COMMS:
		// check too see we are receiving messages from inverter to validate comms
		if (has_inverter_comms())
		{
			vehicle_state = VEHICLE_STANDBY;
		}

		break;

	case VEHICLE_FAULT:
		//check to see if fault goes away
		if(!inverter_fault_active()) {
			vehicle_state = VEHICLE_NO_COMMS;
		}

		break;

	case VEHICLE_STANDBY:
		// everything is good to go in this state, we are just waiting to enable the RTD button
		if (predrive_conditions_met()) {
			vehicle_state = VEHICLE_PREDRIVE;
			preDriveStart_ms = HAL_GetTick();
		}

		break;

	case VEHICLE_PREDRIVE:
		// buzz the RTD buzzer for the correct amount of time
		if(HAL_GetTick() - preDriveStart_ms > PREDRIVE_TIME_ms) {
			vehicle_state = VEHICLE_DRIVING;
		}

		break;

	case VEHICLE_DRIVING:
		
		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		break;
	}

	update_drive_inputs();
	run_simulink_model_and_update_drive_outputs();
	publish_drive_control_snapshot();
}


// ======================================== Inverter Controls Functions ======================================================
//In FVC.c to be explicit about producer/consumer data relationships
void update_drive_inputs(){

	switch (drive_model)
	{
		case OPEN_DIFF_NO_PID:
			open_diff_no_pid_inputs.car_speed      = vnavVelBodyX.data;
			open_diff_no_pid_inputs.yaw_rate		= vnavGyroBodyZ.data;

			// Wheel Slip Traction Threshold
			open_diff_no_pid_inputs.slip_tract_limit_decimal = swDial_b_ul.data;

			// Wheel Speed + Throttle
			osMutexWait(fvcDriveSensorsMutexHandle, osWaitForever);
			open_diff_no_pid_inputs.fvc_drive_sensor_data = fvc_drive_sensor_data_global;
			osMutexRelease(fvcDriveSensorsMutexHandle);

			determine_drive_power_limits(&open_diff_no_pid_inputs.ac_currentMaxLimit_Apk,
										 &open_diff_no_pid_inputs.dc_currentMaxlimit_A, &all_inverter_enable);
			break;
		case OPEN_DIFF:
			open_diff_inputs.car_speed      = vnavVelBodyX.data;
			open_diff_inputs.yaw_rate		= vnavGyroBodyZ.data;

			// PID Terms:
			open_diff_inputs.P = swDial_a_ul.data;
			open_diff_inputs.I = 0;
			open_diff_inputs.integral_reset = 0;
			
			// Wheel Slip Traction Threshold
			open_diff_inputs.slip_tract_limit_decimal = swDial_b_ul.data;

			// Wheel Speed + Throttle
			osMutexWait(fvcDriveSensorsMutexHandle, osWaitForever);
			open_diff_inputs.fvc_drive_sensor_data = fvc_drive_sensor_data_global;
			osMutexRelease(fvcDriveSensorsMutexHandle);
			
			determine_drive_power_limits(&open_diff_inputs.ac_currentMaxLimit_Apk,
										 &open_diff_inputs.dc_currentMaxlimit_A, &all_inverter_enable);
			break;
		case TORQUE_VECTORING:
			break;
		case TORQUE_BY_4:
		default:
			osMutexWait(fvcDriveSensorsMutexHandle, osWaitForever);
			tau_by_4_inputs.throttle_percent = fvc_drive_sensor_data_global.throttle_percent;
			osMutexRelease(fvcDriveSensorsMutexHandle);

			determine_drive_power_limits(&tau_by_4_inputs.ac_currentMaxLimit_Apk,
										 &tau_by_4_inputs.dc_currentMaxlimit_A, &all_inverter_enable);
			break;
	}
	
	// Drive Tick
	drive_control_start_tick_local = HAL_GetTick();
}

void run_simulink_model_and_update_drive_outputs(){
	switch (drive_model)
	{
		case OPEN_DIFF_NO_PID:
			simulink_OD_No_PID_inports.Wheel_Speed_FL = open_diff_no_pid_inputs.fvc_drive_sensor_data.wheel_speed_FL;
			simulink_OD_No_PID_inports.Wheel_Speed_FR = open_diff_no_pid_inputs.fvc_drive_sensor_data.wheel_speed_FR;
			simulink_OD_No_PID_inports.Wheel_Speed_RL = open_diff_no_pid_inputs.fvc_drive_sensor_data.wheel_speed_RL;
			simulink_OD_No_PID_inports.Wheel_Speed_RR = open_diff_no_pid_inputs.fvc_drive_sensor_data.wheel_speed_RR;
			simulink_OD_No_PID_inports.Car_SpeedVx = open_diff_no_pid_inputs.car_speed;

			simulink_OD_No_PID_inports.Yaw_Ratedegs = open_diff_no_pid_inputs.yaw_rate;
			simulink_OD_No_PID_inports.Maximum_Torque = open_diff_no_pid_inputs.tauMaxLimit_Nm;
			simulink_OD_No_PID_inports.Slip_Traction_Lim = open_diff_no_pid_inputs.slip_tract_limit_decimal;
			simulink_OD_No_PID_inports.Throttle = open_diff_no_pid_inputs.fvc_drive_sensor_data.throttle_percent;
			simulink_OD_No_PID_inports.Current_Limit = open_diff_no_pid_inputs.ac_currentMaxLimit_Apk;

			open_differential_no_PID_step();

			open_diff_no_pid_outputs.slipFL_percent = simulink_OD_No_PID_outports.Slip_FL;
			open_diff_no_pid_outputs.slipFR_percent = simulink_OD_No_PID_outports.Slip_FR;
			open_diff_no_pid_outputs.slipRL_percent = simulink_OD_No_PID_outports.Slip_RL;
			open_diff_no_pid_outputs.slipRR_percent = simulink_OD_No_PID_outports.Slip_RR;
			open_diff_no_pid_outputs.is_FL_slipping = simulink_OD_No_PID_outports.slip_status_FL;
			open_diff_no_pid_outputs.is_FR_slipping = simulink_OD_No_PID_outports.slip_status_FR;
			open_diff_no_pid_outputs.is_RL_slipping = simulink_OD_No_PID_outports.slip_status_RL;
			open_diff_no_pid_outputs.is_RR_slipping = simulink_OD_No_PID_outports.slip_status_RR;

			open_diff_no_pid_outputs.tauFL_Nm = simulink_OD_No_PID_outports.Torque_FL;
			open_diff_no_pid_outputs.tauFR_Nm = simulink_OD_No_PID_outports.Torque_FR;
			open_diff_no_pid_outputs.tauRL_Nm = simulink_OD_No_PID_outports.Torque_RL;
			open_diff_no_pid_outputs.tauRR_Nm = simulink_OD_No_PID_outports.Torque_RR;
			open_diff_no_pid_outputs.tauTotalCMD_Nm = simulink_OD_No_PID_outports.Total_Torque_Cmd;

			open_diff_no_pid_outputs.currentCMDFL_Apk = simulink_OD_No_PID_outports.Current_FL;
			open_diff_no_pid_outputs.currentCMDFR_Apk = simulink_OD_No_PID_outports.Current_FR;
			open_diff_no_pid_outputs.currentCMDRL_Apk = simulink_OD_No_PID_outports.Current_RL;
			open_diff_no_pid_outputs.currentCMDRR_Apk = simulink_OD_No_PID_outports.Current_RR;
			open_diff_no_pid_outputs.currentCMDTotal_Apk = simulink_OD_No_PID_outports.Total_Current_Cmd;
			break;
		case OPEN_DIFF:
			simulink_OD_inports.Wheel_Speed_FL = open_diff_inputs.fvc_drive_sensor_data.wheel_speed_FL;
			simulink_OD_inports.Wheel_Speed_FR = open_diff_inputs.fvc_drive_sensor_data.wheel_speed_FR;
			simulink_OD_inports.Wheel_Speed_RL = open_diff_inputs.fvc_drive_sensor_data.wheel_speed_RL;
			simulink_OD_inports.Wheel_Speed_RR = open_diff_inputs.fvc_drive_sensor_data.wheel_speed_RR;
			simulink_OD_inports.Car_SpeedVx = open_diff_inputs.car_speed;

			simulink_OD_inports.Maximum_Torque = open_diff_inputs.tauMaxLimit_Nm;
			simulink_OD_inports.Slip_Traction_Lim = open_diff_inputs.slip_tract_limit_decimal;
			simulink_OD_inports.Throttle = open_diff_inputs.fvc_drive_sensor_data.throttle_percent;
			simulink_OD_inports.Current_Limit = open_diff_inputs.ac_currentMaxLimit_Apk;
			simulink_OD_inports.Yaw_Ratedegs = open_diff_inputs.yaw_rate;

			simulink_OD_inports.Integral_Reset = open_diff_inputs.integral_reset;
			simulink_OD_inports.P_h = open_diff_inputs.P;
			simulink_OD_inports.I = open_diff_inputs.I;

			open_differential_step();

			open_diff_outputs.slipFL_percent = simulink_OD_outports.Slip_FL;
			open_diff_outputs.slipFR_percent = simulink_OD_outports.Slip_FR;
			open_diff_outputs.slipRL_percent = simulink_OD_outports.Slip_RL;
			open_diff_outputs.slipRR_percent = simulink_OD_outports.Slip_RR;
			open_diff_outputs.is_FL_slipping = simulink_OD_outports.slip_status_FL;
			open_diff_outputs.is_FR_slipping = simulink_OD_outports.slip_status_FR;
			open_diff_outputs.is_RL_slipping = simulink_OD_outports.slip_status_RL;
			open_diff_outputs.is_RR_slipping = simulink_OD_outports.slip_status_RR;

			open_diff_outputs.tauFL_Nm = simulink_OD_outports.Torque_FL;
			open_diff_outputs.tauFR_Nm = simulink_OD_outports.Torque_FR;
			open_diff_outputs.tauRL_Nm = simulink_OD_outports.Torque_RL;
			open_diff_outputs.tauRR_Nm = simulink_OD_outports.Torque_RR;
			open_diff_outputs.tauTotalCMD_Nm = simulink_OD_outports.Total_Torque_Cmd;

			open_diff_outputs.currentCMDFL_Apk = simulink_OD_outports.Current_FL;
			open_diff_outputs.currentCMDFR_Apk = simulink_OD_outports.Current_FR;
			open_diff_outputs.currentCMDRL_Apk = simulink_OD_outports.Current_RL;
			open_diff_outputs.currentCMDRR_Apk = simulink_OD_outports.Current_RR;
			open_diff_outputs.currentCMDTotal_Apk = simulink_OD_outports.Total_Current_Cmd;
			break;
		case TORQUE_VECTORING:
			break;
		case TORQUE_BY_4:
		default:
			// Torque
			tau_by_4_outputs.tauTotalCMD_Nm = (tau_by_4_inputs.throttle_percent / 100.0f) * (tau_by_4_inputs.tauMaxLimit_Nm);

			// if AC current limit is pulled down then 0 torque cmd also
			if(tau_by_4_inputs.ac_currentMaxLimit_Apk <= 0){
				tau_by_4_outputs.tauTotalCMD_Nm = 0;
			}

			float per_inverter_tau = tau_by_4_outputs.tauTotalCMD_Nm / TOTAL_INVERTERS;
			tau_by_4_outputs.tauFL_Nm = per_inverter_tau;
			tau_by_4_outputs.tauFR_Nm = per_inverter_tau;
			tau_by_4_outputs.tauRL_Nm = per_inverter_tau;
			tau_by_4_outputs.tauRR_Nm = per_inverter_tau;

			// Current
			tau_by_4_outputs.currentCMDTotal_Apk = clamp(tau_by_4_outputs.tauTotalCMD_Nm / Kt, 0, 
														 tau_by_4_inputs.ac_currentMaxLimit_Apk);
			
			float per_inverter_current = tau_by_4_outputs.currentCMDTotal_Apk / TOTAL_INVERTERS;
			tau_by_4_outputs.currentCMDFL_Apk = per_inverter_current;
			tau_by_4_outputs.currentCMDFR_Apk = per_inverter_current;
			tau_by_4_outputs.currentCMDRL_Apk = per_inverter_current;
			tau_by_4_outputs.currentCMDRR_Apk = per_inverter_current;
			break;
	}

	drive_control_end_tick_local = HAL_GetTick();
	drive_timestep_number_local++;
}

void publish_drive_control_snapshot(){
	DRIVE_CONTROL_SNAPSHOT drive_snapshot_local;

	switch (drive_model)
	{
		case OPEN_DIFF_NO_PID:
			zero_tau_by_4_io(&drive_snapshot_local);
			zero_open_diff_io(&drive_snapshot_local);
			zero_torque_vectoring_io(&drive_snapshot_local);

			drive_snapshot_local.open_diff_no_pid_control_inputs  = open_diff_no_pid_inputs;
			drive_snapshot_local.open_diff_no_pid_control_outputs = open_diff_no_pid_outputs;
			break;

		case OPEN_DIFF:
			zero_tau_by_4_io(&drive_snapshot_local);
			zero_open_diff_no_pid_io(&drive_snapshot_local);
			zero_torque_vectoring_io(&drive_snapshot_local);

			drive_snapshot_local.open_diff_control_inputs  = open_diff_inputs;
			drive_snapshot_local.open_diff_control_outputs = open_diff_outputs;
			break;

		case TORQUE_VECTORING:
			zero_tau_by_4_io(&drive_snapshot_local);
			zero_open_diff_no_pid_io(&drive_snapshot_local);
			zero_open_diff_io(&drive_snapshot_local);
			break;

		case TORQUE_BY_4:
		default:
			zero_open_diff_no_pid_io(&drive_snapshot_local);
			zero_open_diff_io(&drive_snapshot_local);
			zero_torque_vectoring_io(&drive_snapshot_local);

			drive_snapshot_local.tau_by_4_control_inputs  = tau_by_4_inputs;
			drive_snapshot_local.tau_by_4_control_outputs = tau_by_4_outputs;
			break;
	}	

	drive_snapshot_local.drive_control_start_tick = drive_control_start_tick_local;
	drive_snapshot_local.drive_control_end_tick   = drive_control_end_tick_local;
	drive_snapshot_local.drive_enable_state	   	  = all_inverter_enable;
	drive_snapshot_local.drive_vehicle_state	  = vehicle_state;
	drive_snapshot_local.drive_timestep_number	  = drive_timestep_number_local;
	drive_snapshot_local.drive_active_model		  = drive_model;
	osMutexWait(driveSnapshotMutexHandle, osWaitForever);
	drive_snapshot = drive_snapshot_local;
	osMutexRelease(driveSnapshotMutexHandle);

}

void zero_open_diff_io(DRIVE_CONTROL_SNAPSHOT *snapshot){
	snapshot->open_diff_control_inputs  = (OPEN_DIFF_INPUTS){0};
	snapshot->open_diff_control_outputs = (OPEN_DIFF_OUTPUTS){0};
}

void zero_open_diff_no_pid_io(DRIVE_CONTROL_SNAPSHOT *snapshot){
	snapshot->open_diff_no_pid_control_inputs  = (OPEN_DIFF_NO_PID_INPUTS){0};
	snapshot->open_diff_no_pid_control_outputs = (OPEN_DIFF_NO_PID_OUTPUTS){0};
}

void zero_torque_vectoring_io(DRIVE_CONTROL_SNAPSHOT *snapshot){
	
}

void zero_tau_by_4_io(DRIVE_CONTROL_SNAPSHOT *snapshot){
	snapshot->tau_by_4_control_inputs  = (TORQUE_BY_4_INPUTS){0};
	snapshot->tau_by_4_control_outputs = (TORQUE_BY_4_OUTPUTS){0};
}

// ======================================== Inverter/FVC Debug Functions ======================================================
void clear_serial_monitor(){
	 const char clear_screen[] = "\033[2J\033[H";

    HAL_UART_Transmit(UART_DEBUG,
                      (uint8_t *)clear_screen,
                      strlen(clear_screen),
                      HAL_MAX_DELAY);
}

void send_debug_param_group(const char *name, float params[DEBUG_PARAM_COUNT]){
    char msg[192];

    int len = snprintf(msg, sizeof(msg),
                       "%s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",
                       name,
                       params[0], params[1], params[2], params[3], params[4],
                       params[5], params[6], params[7], params[8], params[9]);

    if (len > 0 && len < sizeof(msg)) {
        HAL_UART_Transmit(UART_DEBUG, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
    }
}

void update_debug_params(void){
    fvc_debug_params[0] = fvcPedalPosition1_mm.data;
    fvc_debug_params[1] = fvcPedalPosition2_mm.data;
    fvc_debug_params[2] = TIMED_SOFTWARE_FAULTS[0]->state;
    fvc_debug_params[3] = TIMED_SOFTWARE_FAULTS[1]->state;
    fvc_debug_params[4] = TIMED_SOFTWARE_FAULTS[2]->state;
    fvc_debug_params[5] = get_both_pedals_fault_state();
    fvc_debug_params[6] = all_inverter_enable;
    fvc_debug_params[7] = tau_by_4_inputs.ac_currentMaxLimit_Apk;
    fvc_debug_params[8] = tau_by_4_inputs.dc_currentMaxlimit_A;
    fvc_debug_params[9] = vehicle_state;

    // // INV1 / FL
    // inv1_debug_params[0] = electricalRPM_FL_erpm.data;
    // inv1_debug_params[1] = motorCurrent_FL_Apk.data;
    // inv1_debug_params[2] = controllerTemp_FL_C.data;
    // inv1_debug_params[3] = iqPeak_FL_A.data;
    // inv1_debug_params[4] = throttleSignal_FL_percent.data;
    // inv1_debug_params[5] = controlMode_FL.data;
    // inv1_debug_params[6] = availableMaxCurrent_FL_Apk.data;
    // inv1_debug_params[7] = availableMaxCurrent_FL_A.data;
    // inv1_debug_params[8] = faultCode_FL.data;
    // inv1_debug_params[9] = inputInverterVoltage_FL_V.data;

    // // INV2 / FR
    // inv2_debug_params[0] = electricalRPM_FR_erpm.data;
    // inv2_debug_params[1] = motorCurrent_FR_Apk.data;
    // inv2_debug_params[2] = controllerTemp_FR_C.data;
    // inv2_debug_params[3] = iqPeak_FR_A.data;
    // inv2_debug_params[4] = throttleSignal_FR_percent.data;
    // inv2_debug_params[5] = controlMode_FR.data;
    // inv2_debug_params[6] = availableMaxCurrent_FR_Apk.data;
    // inv2_debug_params[7] = availableMaxCurrent_FR_A.data;
    // inv2_debug_params[8] = faultCode_FR.data;
    // inv2_debug_params[9] = inputInverterVoltage_FR_V.data;

    // // INV3 / RL
    // inv3_debug_params[0] = electricalRPM_RL_erpm.data;
    // inv3_debug_params[1] = motorCurrent_RL_Apk.data;
    // inv3_debug_params[2] = controllerTemp_RL_C.data;
    // inv3_debug_params[3] = iqPeak_RL_A.data;
    // inv3_debug_params[4] = throttleSignal_RL_percent.data;
    // inv3_debug_params[5] = controlMode_RL.data;
    // inv3_debug_params[6] = availableMaxCurrent_RL_Apk.data;
    // inv3_debug_params[7] = availableMaxCurrent_RL_A.data;
    // inv3_debug_params[8] = faultCode_RL.data;
    // inv3_debug_params[9] = inputInverterVoltage_RL_V.data;

    // // INV4 / RR
    // inv4_debug_params[0] = electricalRPM_RR_erpm.data;
    // inv4_debug_params[1] = motorCurrent_RR_Apk.data;
    // inv4_debug_params[2] = controllerTemp_RR_C.data;
    // inv4_debug_params[3] = iqPeak_RR_A.data;
    // inv4_debug_params[4] = throttleSignal_RR_percent.data;
    // inv4_debug_params[5] = controlMode_RR.data;
    // inv4_debug_params[6] = availableMaxCurrent_RR_Apk.data;
    // inv4_debug_params[7] = availableMaxCurrent_RR_A.data;
    // inv4_debug_params[8] = faultCode_RR.data;
    // inv4_debug_params[9] = inputInverterVoltage_RR_V.data;

	    // INV1 / FL
    inv1_debug_params[0] = inputInverterVoltage_FL_V.info.last_rx;
    inv1_debug_params[1] = motorCurrent_FL_Apk.info.last_rx;
    inv1_debug_params[2] = controllerTemp_FL_C.info.last_rx;
    inv1_debug_params[3] = iqPeak_FL_A.info.last_rx;
    inv1_debug_params[4] = throttleSignal_FL_percent.info.last_rx;
    inv1_debug_params[5] = controlMode_FL.info.last_rx;
    inv1_debug_params[6] = availableMaxCurrent_FL_Apk.info.last_rx;
    inv1_debug_params[7] = availableMaxCurrent_FL_A.info.last_rx;
    inv1_debug_params[8] = 0;
    inv1_debug_params[9] = 0;

    // INV2 / FR
    inv2_debug_params[0] = inputInverterVoltage_FL_V.info.last_rx;
    inv2_debug_params[1] = motorCurrent_FR_Apk.info.last_rx;
    inv2_debug_params[2] = controllerTemp_FR_C.info.last_rx;
    inv2_debug_params[3] = iqPeak_FR_A.info.last_rx;
    inv2_debug_params[4] = throttleSignal_FR_percent.info.last_rx;
    inv2_debug_params[5] = controlMode_FR.info.last_rx;
    inv2_debug_params[6] = availableMaxCurrent_FR_Apk.info.last_rx;
    inv2_debug_params[7] = availableMaxCurrent_FR_A.info.last_rx;
    inv2_debug_params[8] = 0;
    inv2_debug_params[9] = 0;

    // INV3 / RL
    inv3_debug_params[0] = inputInverterVoltage_FL_V.info.last_rx;
    inv3_debug_params[1] = motorCurrent_RL_Apk.info.last_rx;
    inv3_debug_params[2] = controllerTemp_RL_C.info.last_rx;
    inv3_debug_params[3] = iqPeak_RL_A.info.last_rx;
    inv3_debug_params[4] = throttleSignal_RL_percent.info.last_rx;
    inv3_debug_params[5] = controlMode_RL.info.last_rx;
    inv3_debug_params[6] = availableMaxCurrent_RL_Apk.info.last_rx;
    inv3_debug_params[7] = availableMaxCurrent_RL_A.info.last_rx;
    inv3_debug_params[8] = 0;
    inv3_debug_params[9] = 0;

    // INV4 / RR
    inv4_debug_params[0] = inputInverterVoltage_FL_V.info.last_rx;
    inv4_debug_params[1] = motorCurrent_RR_Apk.info.last_rx;
    inv4_debug_params[2] = controllerTemp_RR_C.info.last_rx;
    inv4_debug_params[3] = iqPeak_RR_A.info.last_rx;
    inv4_debug_params[4] = throttleSignal_RR_percent.info.last_rx;
    inv4_debug_params[5] = controlMode_RR.info.last_rx;
    inv4_debug_params[6] = availableMaxCurrent_RR_Apk.info.last_rx;
    inv4_debug_params[7] = availableMaxCurrent_RR_A.info.last_rx;
    inv4_debug_params[8] = 0;
    inv4_debug_params[9] = 0;
}