#include "fvc.h"
#include "sensor_and_CAN.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* CAN_CARSIDE;
CAN_HandleTypeDef* CAN_FRONT_INVERTERS;
CAN_HandleTypeDef* CAN_REAR_INVERTERS;

// Inverter State Machine Defines:
uint32_t preDriveStart_ms;

uint32_t drive_control_start_tick_local;
uint32_t drive_control_end_tick_local;
uint32_t drive_timestep_number_local;

VEHICLE_STATE_t vehicle_state;
FVC_DRIVE_SENSOR_DATA fvc_drive_sensor_data_global;
DRIVE_CONTROL_SNAPSHOT drive_snapshot;

bool all_inverter_enable; // enable for all 4 inverters in this case
bool slow_mode;

DRIVE_CONTROL_INPUTS open_diff_inputs = {
	.slip_tract_limit_percent = TRACTION_LIMIT_percent,
	.car_speed      = 0.0,
	.fvc_drive_sensor_data = {
		.wheel_speed_FL = 0.0,
		.wheel_speed_FR = 0.0,
		.wheel_speed_RL = 0.0,
		.wheel_speed_RR = 0.0,
		.throttle_percent = 0.0
	},
	.tauMaxLimit_Nm = MOTOR_MAX_TORQUE_Nm,
	.ac_currentMaxLimit_Apk = 0.0,
	.dc_currentMaxlimit_A = 0.0,
};

DRIVE_CONTROL_OUTPUTS open_diff_outputs = {
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

// Init FVC
// What needs to happen on FVC startup 
void init_fvc(CAN_HandleTypeDef* BUS_1, CAN_HandleTypeDef* BUS_2, CAN_HandleTypeDef* BUS_3){
	CAN_CARSIDE = BUS_1;
	CAN_FRONT_INVERTERS = BUS_2;
	CAN_REAR_INVERTERS = BUS_3;

	init_can(CAN_CARSIDE, GCAN0);
	init_can(CAN_FRONT_INVERTERS, GCAN1);
	init_can(CAN_REAR_INVERTERS, GCAN2);

	init_git_LEDs();
	init_efuses();
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
	// TODO add UART functionality here
}

void drive_task(){
	process_inverter();
}

void fault_task(){
	update_fault_data();
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


/// ======================================== Inverter Controls Functions ======================================================
//In FVC.c to be explicit about producer/consumer data relationships
void update_drive_inputs(){
	open_diff_inputs.car_speed      = vnavVelBodyX.data;

	osMutexWait(fvcDriveSensorsMutexHandle, osWaitForever);
	open_diff_inputs.fvc_drive_sensor_data = fvc_drive_sensor_data_global;
	osMutexRelease(fvcDriveSensorsMutexHandle);

	// Current Limits + Enable
	float max_AC_inv_limit = (slow_mode) ? AC_CURRENT_SLOW_MODE_MAX_Apk : AC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_Apk;
	if (vehicle_state != VEHICLE_DRIVING){
		open_diff_inputs.ac_currentMaxLimit_Apk = 0;
		open_diff_inputs.dc_currentMaxlimit_A   = 0;
		all_inverter_enable = FALSE; 
	}
	else{
		open_diff_inputs.ac_currentMaxLimit_Apk = get_rules_fault_state() ? 0 : max_AC_inv_limit;
		open_diff_inputs.dc_currentMaxlimit_A = calculate_dc_current_limit();
		all_inverter_enable = TRUE; 
	}
	
	drive_control_start_tick_local = HAL_GetTick();
}

void run_simulink_model_and_update_drive_outputs(){
	drive_control_end_tick_local = HAL_GetTick();
	drive_timestep_number_local++;
}

void publish_drive_control_snapshot(){
	DRIVE_CONTROL_SNAPSHOT drive_snapshot_local;

	drive_snapshot_local.drive_control_inputs  	  = open_diff_inputs;
	drive_snapshot_local.drive_control_outputs 	  = open_diff_outputs;
	drive_snapshot_local.drive_control_start_tick = drive_control_start_tick_local;
	drive_snapshot_local.drive_control_end_tick   = drive_control_end_tick_local;
	drive_snapshot_local.drive_enable_state	   	  = all_inverter_enable;
	drive_snapshot_local.drive_vehicle_state	  = vehicle_state;
	drive_snapshot_local.drive_timestep_number	  = drive_timestep_number_local;
	
	osMutexWait(driveSnapshotMutexHandle, osWaitForever);
	drive_snapshot = drive_snapshot_local;
	osMutexRelease(driveSnapshotMutexHandle);

}