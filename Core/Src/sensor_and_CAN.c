#include "gopher_sense.h"
#include "stdlib.h"
#include "sensor_and_CAN.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"

void update_non_ADC_CAN_params(){
	// ADC Params Automatically Queued via gophersense

	//High Frequency(200Hz): Wheel Speeds, Shocks(ADC)
	FVC_DRIVE_SENSOR_DATA fvc_drive_sensor_data_local; // Telemetry Task is producer of wheels speed/throttle sensor data
	fvc_drive_sensor_data_local.wheel_speed_FL = calc_wheel_m_per_s(electricalRPM_FL_erpm.data);
	fvc_drive_sensor_data_local.wheel_speed_FR = calc_wheel_m_per_s(electricalRPM_FR_erpm.data);
	fvc_drive_sensor_data_local.wheel_speed_RL = calc_wheel_m_per_s(electricalRPM_RL_erpm.data);
	fvc_drive_sensor_data_local.wheel_speed_RR = calc_wheel_m_per_s(electricalRPM_RR_erpm.data);
	fvc_drive_sensor_data_local.throttle_percent = calc_pedal_percent(fvcPedalPosition1_mm.data, APPS_1_MIN_CURRENT_POS_mm, APPS_1_TOTAL_TRAVEL_mm);

	osMutexWait(fvcDriveSensorsMutexHandle, osWaitForever);
	fvc_drive_sensor_data_global = fvc_drive_sensor_data_local;
	osMutexRelease(fvcDriveSensorsMutexHandle);

	update_and_queue_param_float(&fvcWheelSpeedFrontLeft_m_per_s,  fvc_drive_sensor_data_local.wheel_speed_FL);
	update_and_queue_param_float(&fvcWheelSpeedFrontRight_m_per_s, fvc_drive_sensor_data_local.wheel_speed_FR);
	update_and_queue_param_float(&fvcWheelSpeedRearLeft_m_per_s,   fvc_drive_sensor_data_local.wheel_speed_RL);
	update_and_queue_param_float(&fvcWheelSpeedRearRight_m_per_s,  fvc_drive_sensor_data_local.wheel_speed_RR);


	// Medium Frequency(100Hz): 
	// Pedal Pos(ADC), Pedal %, Steer Angle(ADC), Vehicle State, Controls Timestep, Slip %, Slip Statuses, Tau Cmd, Wheel Tau, Current Cmd, Inverter Cmds
	update_and_queue_param_float(&fvcPedalPosition1_percent, fvc_drive_sensor_data_local.throttle_percent);
	update_and_queue_param_float(&fvcPedalPosition2_percent, calc_pedal_percent(fvcPedalPosition2_mm.data, APPS_2_MIN_CURRENT_POS_mm, APPS_2_TOTAL_TRAVEL_mm));

	// Snapshot of Recent Controls Timestep
	static DRIVE_CONTROL_SNAPSHOT last_drive_snapshot_local;
	DRIVE_CONTROL_SNAPSHOT drive_snapshot_local;
	osMutexWait(driveSnapshotMutexHandle, osWaitForever);
	drive_snapshot_local = drive_snapshot;
	osMutexRelease(driveSnapshotMutexHandle);

	if (drive_snapshot_local.drive_timestep_number != last_drive_snapshot_local.drive_timestep_number){
		update_and_queue_param_float(&fvcSlip_FL_percent, drive_snapshot_local.drive_control_outputs.slipFL_percent);
		update_and_queue_param_float(&fvcSlip_FR_percent, drive_snapshot_local.drive_control_outputs.slipFR_percent);
		update_and_queue_param_float(&fvcSlip_RL_percent, drive_snapshot_local.drive_control_outputs.slipRL_percent);
		update_and_queue_param_float(&fvcSlip_RR_percent, drive_snapshot_local.drive_control_outputs.slipRR_percent);

		update_and_queue_param_u8(&fvcSlipStatus_FL_state, drive_snapshot_local.drive_control_outputs.is_FL_slipping);
		update_and_queue_param_u8(&fvcSlipStatus_FR_state, drive_snapshot_local.drive_control_outputs.is_FR_slipping);
		update_and_queue_param_u8(&fvcSlipStatus_RL_state, drive_snapshot_local.drive_control_outputs.is_RL_slipping);
		update_and_queue_param_u8(&fvcSlipStatus_RR_state, drive_snapshot_local.drive_control_outputs.is_RR_slipping);

		update_and_queue_param_float(&fvcTauTotalCMD_Nm, drive_snapshot_local.drive_control_outputs.tauTotalCMD_Nm);
		update_and_queue_param_float(&fvcTau_FL_Nm, 	 drive_snapshot_local.drive_control_outputs.tauFL_Nm);
		update_and_queue_param_float(&fvcTau_FR_Nm, 	 drive_snapshot_local.drive_control_outputs.tauFR_Nm);
		update_and_queue_param_float(&fvcTau_RL_Nm, 	 drive_snapshot_local.drive_control_outputs.tauRL_Nm);
		update_and_queue_param_float(&fvcTau_RR_Nm, 	 drive_snapshot_local.drive_control_outputs.tauRR_Nm);

		update_and_queue_param_float(&fvcCurrentTotalCMD_Apk, drive_snapshot_local.drive_control_outputs.currentCMDTotal_Apk);

		update_and_queue_param_u32(&fvcDriveControlTickStart_ms, drive_snapshot_local.drive_control_start_tick);
		update_and_queue_param_u32(&fvcDriveControlTickEnd_ms, 	 drive_snapshot_local.drive_control_end_tick);
		update_and_queue_param_u8(&fvcVehicleState_state, 		 drive_snapshot_local.drive_vehicle_state);
		
		// Update Inverter Params
		invCurrentCmd_FL_Apk.data = drive_snapshot_local.drive_control_outputs.currentCMDFL_Apk;
		invCurrentCmd_FR_Apk.data = drive_snapshot_local.drive_control_outputs.currentCMDFR_Apk;
		invCurrentCmd_RL_Apk.data = drive_snapshot_local.drive_control_outputs.currentCMDRL_Apk;
		invCurrentCmd_RR_Apk.data = drive_snapshot_local.drive_control_outputs.currentCMDRR_Apk;

		//TODO gotta change Current limits to be dynamic with 4 wheel/2 wheel drive
		float per_inverter_ac_currentMaxLimit = (drive_snapshot_local.drive_control_inputs.ac_currentMaxLimit_Apk) / TOTAL_INVERTERS;
		invMaxCurrentLimitCmd_FL_Apk.data = per_inverter_ac_currentMaxLimit;
		invMaxCurrentLimitCmd_FR_Apk.data = per_inverter_ac_currentMaxLimit;
		invMaxCurrentLimitCmd_RL_Apk.data = per_inverter_ac_currentMaxLimit;
		invMaxCurrentLimitCmd_RR_Apk.data = per_inverter_ac_currentMaxLimit;

		float per_inverter_dc_currentMaxLimit = (drive_snapshot_local.drive_control_inputs.dc_currentMaxlimit_A) / TOTAL_INVERTERS;
		invMaxDCCurrentLimitCmd_FL_A.data = per_inverter_dc_currentMaxLimit;
		invMaxDCCurrentLimitCmd_FR_A.data = per_inverter_dc_currentMaxLimit;
		invMaxDCCurrentLimitCmd_RL_A.data = per_inverter_dc_currentMaxLimit;
		invMaxDCCurrentLimitCmd_RR_A.data = per_inverter_dc_currentMaxLimit;

		bool per_inverter_enable = drive_snapshot_local.drive_enable_state;
		invDriveEnable_FL_state.data = per_inverter_enable;
		invDriveEnable_FR_state.data = per_inverter_enable;
		invDriveEnable_RL_state.data = per_inverter_enable;
		invDriveEnable_RR_state.data = per_inverter_enable;
		
		send_group(invCurrentCmd_FL_Apk.info.GROUP_ID);
		send_group(invCurrentCmd_FR_Apk.info.GROUP_ID);
		send_group(invCurrentCmd_RL_Apk.info.GROUP_ID);
		send_group(invCurrentCmd_RR_Apk.info.GROUP_ID);

		send_group(invMaxCurrentLimitCmd_FL_Apk.info.GROUP_ID);
		send_group(invMaxCurrentLimitCmd_FR_Apk.info.GROUP_ID);
		send_group(invMaxCurrentLimitCmd_RL_Apk.info.GROUP_ID);
		send_group(invMaxCurrentLimitCmd_RR_Apk.info.GROUP_ID);

		send_group(invMaxDCCurrentLimitCmd_FL_A.info.GROUP_ID);
		send_group(invMaxDCCurrentLimitCmd_FR_A.info.GROUP_ID);
		send_group(invMaxDCCurrentLimitCmd_RL_A.info.GROUP_ID);
		send_group(invMaxDCCurrentLimitCmd_RR_A.info.GROUP_ID);

		send_group(invDriveEnable_FL_state.info.GROUP_ID);
		send_group(invDriveEnable_FR_state.info.GROUP_ID);
		send_group(invDriveEnable_RL_state.info.GROUP_ID);
		send_group(invDriveEnable_RR_state.info.GROUP_ID);
	}
	last_drive_snapshot_local = drive_snapshot_local;

	// Low Freqeuncy(5Hz):
	// Brake Temps(ADC), Ride Height(ADC), Rules Required Flts, Display Flt, Hbeat Flt, SDC, Drive Speed Mode, Motor/Inv Temps, Efuse Flts

	// Rules Required Faults:
	update_and_queue_param_u8(&fvcPedalPosition1RangeFault_state, 	   TIMED_SOFTWARE_FAULTS[0]->state);
	update_and_queue_param_u8(&fvcPedalPosition2RangeFault_state, 	   TIMED_SOFTWARE_FAULTS[1]->state);
	update_and_queue_param_u8(&fvcPedalPositionCorrelationFault_state, TIMED_SOFTWARE_FAULTS[2]->state);
	update_and_queue_param_u8(&fvcBothPedalsPressedFault_state,		   get_both_pedals_fault_state());

	// Display Faults
	update_and_queue_param_u8(&fvcDisplayFaultStatus_state, determine_disp_fault_status());

	// Hbeat Status
	update_and_queue_param_u8(&fvcMcuStatus_state, get_Hbeat_status());

	// SDC Statuses
	update_and_queue_param_u8(&fvcSdcStatus3, HAL_GPIO_ReadPin(SDC1_GPIO_Port, SDC1_Pin));
	update_and_queue_param_u8(&fvcSdcStatus4, HAL_GPIO_ReadPin(SDC2_GPIO_Port, SDC2_Pin));

	// Drive Speed Mode
	update_and_queue_param_u8(&fvcDriveSpeedMode_state, get_slow_mode_status());

	// Forwarded Motor/Inv Temps
	update_and_queue_param_float(&fvcControllerTemp_FL_C, controllerTemp_FL_C.data);
	update_and_queue_param_float(&fvcControllerTemp_FR_C, controllerTemp_FR_C.data);
	update_and_queue_param_float(&fvcControllerTemp_RL_C, controllerTemp_RL_C.data);
	update_and_queue_param_float(&fvcControllerTemp_RR_C, controllerTemp_RR_C.data);

    update_and_queue_param_float(&fvcMotorTemp_FL_C, motorTemp_FL_C.data);
	update_and_queue_param_float(&fvcMotorTemp_FR_C, motorTemp_FR_C.data);
	update_and_queue_param_float(&fvcMotorTemp_RL_C, motorTemp_RL_C.data);
	update_and_queue_param_float(&fvcMotorTemp_RR_C, motorTemp_RR_C.data);

	// Efuse Fault States
	// TODO for Efuse Task
}

float calc_pedal_percent(float pedalPos_mm, float min_position_mm, float total_range_mm){
	float pedal_percent;

	pedal_percent = 100.0*(pedalPos_mm - min_position_mm) / total_range_mm;
	pedal_percent = clamp(pedal_percent, 0.0, 100.0);

	return pedal_percent;
}

float calc_wheel_m_per_s(float electrical_rpm){
	return electrical_rpm * ERPM_TO_M_PER_S;
}

DISPLAY_FAULT_STATUS_t determine_disp_fault_status() {
	int status = NONE;
	if(amsFault_state.data) status = AMS_FAULT;
	else if (vehicle_state == VEHICLE_FAULT) status = INVERTER_FAULT;
	else if(get_both_pedals_fault_state() == TRUE) status = RELEASE_PEDAL;
	else if(rvcBspdFaultActive_state.data) status = BRAKING_FAULT;
	else if(TIMED_SOFTWARE_FAULTS[2]->state) status = APPS_FAULT;
	else if(rvcBspdFaultLatched_state.data) status = BSPD_FAULT;

	return status;
}
