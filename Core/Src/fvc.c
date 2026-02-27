#include "fvc.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* CAN_CARSIDE;
CAN_HandleTypeDef* CAN_FRONT_INVERTERS;
CAN_HandleTypeDef* CAN_REAR_INVERTERS;

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
// This loop will handle CAN RX software task and CAN TX hardware task. Should be
// called every 1ms or as often as received messages should be handled
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
// another loop. This includes logic for sending a CAN command. Designed to be
// called every 1ms
void main_loop()
{

}

// Dash Light Functionality:
// If AMS("Accumulator management system, so BMS :/") fault --> BMS light goes on
// If IMD Fault --> IMD Fault Goes on
void set_dash_lights(){
	if(amsFault_state.data) {
		//If amsFault_state HIGH, PB10 ON
		HAL_GPIO_WritePin(BMS_Dash_Light_GPIO_Port, BMS_Dash_Light_Pin, GPIO_PIN_SET);

	} else{
		//If amsFault_state LOW, PB10 OFF
		HAL_GPIO_WritePin(BMS_Dash_Light_GPIO_Port, BMS_Dash_Light_Pin, GPIO_PIN_RESET);
	}

	if(imdFault_state.data) {
		//If imdFault_state HIGH, PB11 ON
		HAL_GPIO_WritePin(IMD_Dash_Light_GPIO_Port, IMD_Dash_Light_Pin, GPIO_PIN_SET);
		
	}else{
		//If imdFault_state LOW, PB11 OFF
		HAL_GPIO_WritePin(IMD_Dash_Light_GPIO_Port, IMD_Dash_Light_Pin, GPIO_PIN_RESET);
	}
	
}

