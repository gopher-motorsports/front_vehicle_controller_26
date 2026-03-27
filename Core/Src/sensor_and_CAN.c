/*
 * sensor_processing.c
 *
 *  Created on: Feb 13, 2025
 *      Author: chris
 */

#include "FVC.h"
#include "gopher_sense.h"
#include "stdlib.h"
#include "sensor_and_CAN.h"
#include "fvc_software_faults.h"
//Always Periodic --> pedalPosition1 %, pedalPosition2 %, wheel speed front left, wheel speed front right, out of range --> apps1, apps2, brake front, cor
//Inverter State Machine Periodic --> desired_current, max_current, enable state, vehicle state
//Change based --> vcuPedalPosition1Fault_state, vcuPedalPosition2Fault_state, vcuBrakePressureSensorFault_state, vcuTractiveSystemCurrentSensorFault_state
// vcuPedalPositionCorrelationFault_state, vcuPedalPositionBrakingFault_state

void update_non_ADC_CAN_params(){
	// Medium Frequency: Pedal Percentages
	update_and_queue_param_float(&fvcPedalPosition1_percent, calc_pedal_percent(fvcPedalPosition1_mm, APPS_1_MIN_CURRENT_POS_mm, APPS_1_TOTAL_TRAVEL_mm));
	update_and_queue_param_float(&fvcPedalPosition2_percent, calc_pedal_percent(fvcPedalPosition2_mm, APPS_2_MIN_CURRENT_POS_mm, APPS_2_TOTAL_TRAVEL_mm));

	// Wheel Speeds
	update_and_queue_param_float(&fvcWheelSpeedFrontLeft_m_per_s,  calculate_wheel_rpm(electricalRPM_FL_erpm.data));
	update_and_queue_param_float(&fvcWheelSpeedFrontRight_m_per_s, calculate_wheel_rpm(electricalRPM_FR_erpm.data));
	update_and_queue_param_float(&fvcWheelSpeedRearLeft_m_per_s,   calculate_wheel_rpm(electricalRPM_RL_erpm.data));
	update_and_queue_param_float(&fvcWheelSpeedRearRight_m_per_s,  calculate_wheel_rpm(electricalRPM_RR_erpm.data));

	//Fault Statuses
	update_and_queue_param_u8(&fvcDisplayFaultStatus_state, 		   determine_disp_fault_status());
	update_and_queue_param_u8(&fvcPedalPosition1Fault_state, 		   TIMED_SOFTWARE_FAULTS[0]->state);
	update_and_queue_param_u8(&fvcPedalPosition2Fault_state, 		   TIMED_SOFTWARE_FAULTS[1]->state);
	update_and_queue_param_u8(&fvcPedalPositionCorrelationFault_state, TIMED_SOFTWARE_FAULTS[2]->state);
	update_and_queue_param_u8(&fvcBothPedalsPressedFault_state,		   get_both_pedals_fault_state());

	// Inverter Diagnostics
	
	// SDC Params
	update_and_queue_param_u8(&fvcSdcStatus3, HAL_GPIO_ReadPin(SDC1_GPIO_Port, SDC1_Pin));
	update_and_queue_param_u8(&fvcSdcStatus4, HAL_GPIO_ReadPin(SDC2_GPIO_Port, SDC2_Pin));

	// Forwarded Parameters
	update_and_queue_param_float(&fvcControllerTemp_FL_C, controllerTemp_FL_C.data);
	update_and_queue_param_float(&fvcControllerTemp_FR_C, controllerTemp_FR_C.data);
	update_and_queue_param_float(&fvcControllerTemp_RR_C, controllerTemp_RL_C.data);
	update_and_queue_param_float(&fvcControllerTemp_RL_C, controllerTemp_RR_C.data);

    update_and_queue_param_float(&fvcMotorTemp_FL_C, motorTemp_FL_C.data);
	update_and_queue_param_float(&fvcMotorTemp_FL_C, motorTemp_FR_C.data);
	update_and_queue_param_float(&fvcMotorTemp_FL_C, motorTemp_RL_C.data);
	update_and_queue_param_float(&fvcMotorTemp_FL_C, motorTemp_RR_C.data);

}

void update_inverter_params(uint8_t vehicle_state, float desired_current, float max_current, float max_dc_current, uint8_t enable){
	//update global vehicle state, enable, desired/max current, dc max current
	update_and_queue_param_u8(&vehicleState_state, vehicle_state);
	update_and_queue_param_float(&desiredInvCurrentPeak_A, desired_current);
	update_and_queue_param_float(&maxCurrentLimitPeak_A, max_current);
	update_and_queue_param_float(&maxDCCurrentLimit_A, max_dc_current);
	update_and_queue_param_u8(&driveEnable_state, enable);

	// We have to send group here to prevent timeout because GCAN won't update values if they're the same
	send_group(driveEnable_state.info.GROUP_ID);
	send_group(desiredInvCurrentPeak_A.info.GROUP_ID);
}

void calculate_pedal_percent(float pedalPos_mm, float min_position_mm, float total_range_mm){
	float pedal_percent;

	pedal_percent = 100.0*(pedalPos_mm - min_position_mm) / total_range_mm;
	pedal_percent = clamp(pedal_percent, 0.0, 100.0);

	return pedal_percent;
}

float calculate_wheel_rpm(float electrical_rpm){
	float motor_rpm;
	float wheel_rpm;
	float wheel_m_per_s;

	motor_rpm = electrical_rpm / MOTOR_POLE_PAIRS;
	wheel_rpm = motor_rpm / DRIVE_TRAIN_GEAR_RATIO;
	wheel_m_per_s = (wheel_rpm * 2 * WHEEL_RADIUS * MATH_PI) / SECONDS_PER_MINUTE;
	
	return wheel_m_per_s;
}

uint8_t determine_disp_fault_status() {
	int status = NONE;
	if(amsFault_state.data) status = AMS_FAULT;
	else if (vehicle_state == VEHICLE_FAULT) status = INVERTER_FAULT;
	else if(fvcBothPedalsPressedFault_state.data) status = RELEASE_PEDAL;
	else if(rvcBspdFaultActive_state.data) status = BRAKING_FAULT;
	else if(fvcPedalPositionCorrelationFault_state.data) status = APPS_FAULT;
	else if(rvcBspdFaultLatched_state.data) status = BSPD_FAULT;

	return status;
}
