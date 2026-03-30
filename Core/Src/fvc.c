#include "fvc.h"
#include "conditions_and_utils.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* CAN_CARSIDE;
CAN_HandleTypeDef* CAN_FRONT_INVERTERS;
CAN_HandleTypeDef* CAN_REAR_INVERTERS;

// Inverter State Machine Defines:
VEHICLE_STATE_t vehicle_state;
uint32_t preDriveStart_ms;
float ac_currentLimit_Apk;
float dc_currentlimit_A;

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
void process_inverter() {

	if (!has_inverter_comms())
		vehicle_state = VEHICLE_NO_COMMS;
	else if (inverter_fault_active()){
		vehicle_state = VEHICLE_FAULT;
	}

	determine_current_limits(&ac_currentLimit_Apk, &dc_currentlimit_A, vehicle_state);
	InverterComms_Status_t comms = get_inverter_comms_status();
	
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
			if(comms.all_comms){
				vehicle_state = VEHICLE_4WD;
			}
			else if(comms.front_comms){
				vehicle_state = VEHICLE_FWD;
			}
			else{
				vehicle_state = VEHICLE_RWD;
			}
		}

		break;
	
	// Create a VEHCILE_FRONT_WHEEL_DRIVE, VEHCILE_REAR_WHEEL_DRIVE, VEHICLE_ALL_WHEEL_DRIVE
	// Will need to pass to open diff/torque vectoring simulink controller which wheels should get power
	
	//create swich cases for the comms here?
	case VEHICLE_FWD:
		if(comms.all_comms){
			vehicle_state = VEHICLE_4WD;
		} else if(comms.front_comms != 1){
			vehicle_state = VEHICLE_RWD;
		}
		break;
	case VEHICLE_RWD:
		if(comms.all_comms){
			vehicle_state = VEHICLE_4WD;
			//send diff/torque inside if statements depending on which case
		} else if(comms.rear_comms != 1){
			vehicle_state = VEHICLE_FWD;
		}
		break;
	case VEHICLE_4WD:
		if(comms.all_comms !=1){
			if(comms.front_comms){
				vehicle_state = VEHICLE_FWD;
			} else {
				vehicle_state = VEHICLE_RWD;
			}
		}
		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		break;
	}

	// send the current request
	//update_inverter_params(vehicle_state, desiredCurrent_A, maxcurrentLimit_A, 200, vehicle_state == VEHICLE_DRIVING);
}
