#include "fvc.h"
#include "conditions_and_utils.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* CAN_CARSIDE;
CAN_HandleTypeDef* CAN_FRONT_INVERTERS;
CAN_HandleTypeDef* CAN_REAR_INVERTERS;

// Inverter State Machine Defines:
VEHICLE_STATE_t vehicle_state;
uint32_t preDriveTimer_ms;
float ac_currentLimit_Apk;
float dc_currentlimit_A;
U8_CAN_STRUCT *inv_enable_fbk_statuses[] = {&driveEnableInvStatus_FL_state, &driveEnableInvStatus_FR_state, &driveEnableInvStatus_RL_state, &driveEnableInvStatus_RR_state};
U8_CAN_STRUCT *inv_fault_codes[] = {&faultCode_FL, &faultCode_FR, &faultCode_RL, &faultCode_RR};
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
	LED_task();
}

// ======================================== Inverter State Machine Functions ======================================================
void determine_current_limits(VEHICLE_STATE_t state){
	if (state != VEHICLE_DRIVING){
		ac_currentLimit_Apk = 0;
		dc_currentlimit_A = 0;
	}
	else{
		ac_currentLimit_Apk = is_vehicle_faulting() ? 0 : AC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_Apk;
		dc_currentlimit_A = calculate_dc_current_limit();
	}
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
void process_inverter() {

	if (!has_inverter_comms())
		vehicle_state = VEHICLE_NO_COMMS;
	else if (inverter_fault_active()){
		vehicle_state = VEHICLE_FAULT;
	}

	determine_current_parameters(vehicle_state);

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
			preDriveTimer_ms = 0;
		}

		break;

	case VEHICLE_PREDRIVE:
		// buzz the RTD buzzer for the correct amount of time
		if(++preDriveTimer_ms > PREDRIVE_TIME_ms) {
			vehicle_state = VEHICLE_DRIVING;
		}

		break;

	case VEHICLE_DRIVING:

		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		break;
	}

	// send the current request
	//update_inverter_params(vehicle_state, desiredCurrent_A, maxcurrentLimit_A, 200, vehicle_state == VEHICLE_DRIVING);
}
