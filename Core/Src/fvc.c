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
VEHICLE_STATE_t last_vehicle_state;
VEHICLE_STATE_t target_vehicle_state;

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

	InverterComms_Status_t comms = get_inverter_comms_status();
	InverterFault_Status_t fault = get_Inverter_Fault_Status();
	if (!has_inverter_comms())
		vehicle_state = VEHICLE_NO_COMMS;
	else if (inverter_fault_active()){
		vehicle_state = VEHICLE_FAULT;
	}
	// can having this before the switch case just be a catch all so i dont have to check inside of each case?
	if(!comms.front_comms && !comms.rear_comms && comms.any_comms){
			//should be only one inverter than 
			vehicle_state = VEHICLE_FAULT;
		}


	determine_current_limits(&ac_currentLimit_Apk, &dc_currentlimit_A, vehicle_state);
	
	
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
	
	
	case VEHICLE_FWD:
		if (!comms.any_comms|| (!comms.front_comms && !comms.rear_comms) || (fault.front && fault.rear) 
		|| (fault.front && !comms.rear_comms) || (fault.rear && !comms.front_comms)){
			vehicle_state = VEHICLE_FAULT; // immeadiately 
		} 
		else if(comms.all_comms && fault.any !=1){
			target_vehicle_state = VEHICLE_4WD;	
			vehicle_state = TRANSITION;
		} else if(fault.front || !comms.front_comms){//already know that theres either front or rear comms and one is not faulted
			target_vehicle_state = VEHICLE_RWD;
			vehicle_State = TRANSITION;
		}
		last_vehicle_state = VEHICLE_FWD;
		break;
	case VEHICLE_RWD:
		//draw out to make sure 
		if (!comms.any_comms|| (!comms.front_comms && !comms.rear_comms) || (fault.front && fault.rear) 
		|| (fault.front && !comms.rear_comms) || (fault.rear && !comms.front_comms)){
			vehicle_state = VEHICLE_FAULT; // immeadiately 
		} 
		else if(comms.all_comms && fault.any !=1){
			target_vehicle_State = VEHICLE_4WD;
			vehicle_state = TRANSITION;
		}
		else if(comms.rear_comms != 1 || fault.rear){//already checked that both front and rear are not both faulted
			target_vehicle_state = VEHICLE_FWD;
			vehicle_state = TRANSITION;
		}
		last_vehicle_state = VEHICLE_RWD;
		break;
	case VEHICLE_4WD:
		if(comms.all_comms !=1 || fault.any == 1){
			bool one_comm = (!comms.front_comms && !comms.rear_comms && comms.any_comms)
			bool no_comms = (!comms.any_comms)
			bool no_front_comms_live = comms.front_comms;
			bool no_rear_comms_live = comms.rear_comms;

			if(one_comm || no_comms || (no_front_comms_live || no_rear_comms_live) || (fault.front && fault.rear)){
				vehicle_state = VEHICLE_FAULT; //immediately
			}
			else if(comms.front_comms || fault.rear == 1){
				target_vehicle_state = VEHICLE_FWD;
				vehicle_state = TRANSITION;
			} else if (comms.rear_comms || fault.front == 1){
				target_vehicle_state = VEHICLE_RWD;
				vehicle_state = TRANSITION;
			}
		}
		last_vehicle_state = VEHICLE_4WD;
		
		break;
	
	case TRANSITION:
		if(HAL_getTick() - last_trans_time > 5000){ //give time to react
			if(targe_vehicle_state == VEHICLE_4WD){
				if(last_vehicle_state == RWD){
					//print RWD->4WD
				} else if(last_vehicle_state == FWD){
					//print FWD -> 4WD
			}
			else if(target_vehicle_state == VEHICLE_RWD){
				if(last_vehicle_state == VEHICLE_4WD){
					//print 4WD->RWD
				}else if(last_vehicle_state == VEHICLE_FWD){
					//print FWD->RWD
				}

			}else if(target_vehicle_state ==VEHIClE_FWD){
				if(last_vehicle_state == VEHICLE_4WD){
					//print 4WD->FWD
				} else if(last_vehicle_state == VEHICLE_RWD){
					//print RWD->FWD
				}
				
			}
			
		}

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		break;
	}

	//very last line
	vehicle_state = target_vehicle_state;
	// send the current request
	//update_inverter_params(vehicle_state, desiredCurrent_A, maxcurrentLimit_A, 200, vehicle_state == VEHICLE_DRIVING);
}
