#include "fvc.h"
#include "conditions_and_utils.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* CAN_CARSIDE;
CAN_HandleTypeDef* CAN_FRONT_INVERTERS;
CAN_HandleTypeDef* CAN_REAR_INVERTERS;

// Inverter State Machine Defines:
uint32_t preDriveStart_ms;
uint32_t drive_control_timestep_start;
bool global_inverter_enable;

DRIVE_CONTROL_INPUTS open_diff_inputs = {
	.slip_tract_limit_percent = TRACTION_LIMIT_percent,
	.car_speed      = 0.0,
	.wheel_speed_FL = 0.0,
	.wheel_speed_FR = 0.0,
	.wheel_speed_RL = 0.0,
	.wheel_speed_RR = 0.0,
	.throttle_percent = 0.0,
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
}


// can_buffer_handling_loop
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	service_can_rx_buffer();
	
	// handle the transmission hardware for each CAN bus
	service_can_tx(CAN_CARSIDE);
	service_can_tx(CAN_FRONT_INVERTERS);
	service_can_tx(CAN_REAR_INVERTERS);
}


// main_loop
// called every 1ms
void main_loop()
{
	Hbeat_blink();
}

// ======================================== Inverter State Machine Getters/Setters ======================================================
uint32_t get_drive_control_timestep_start(){
	return drive_control_timestep_start;
}

// ======================================== Inverter State Machine ======================================================
void process_inverter() {

	if (!has_inverter_comms())
		vehicle_state = VEHICLE_NO_COMMS;
	else if (inverter_fault_active()){
		vehicle_state = VEHICLE_FAULT;
	}

	update_drive_control_inputs(&global_inverter_enable);

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
	
	// Create a VEHCILE_FRONT_WHEEL_DRIVE, VEHCILE_REAR_WHEEL_DRIVE, VEHICLE_ALL_WHEEL_DRIVE
	// Will need to pass to open diff/torque vectoring simulink controller which wheels should get power
	case VEHICLE_DRIVING:
		drive_control_timestep_start = HAL_GetTick();
		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		break;
	}

	// send the current request
	//update_inverter_params(vehicle_state, desiredCurrent_A, maxcurrentLimit_A, 200, vehicle_state == VEHICLE_DRIVING);
}
