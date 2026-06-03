#include "gopher_sense.h"
#include "stdlib.h"
#include "sensor_and_CAN.h"
#include "conditions_and_utils.h"
#include "fvc_software_faults.h"
#include "git_info.h"

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
	update_drive_control_params();

	// Low Freqeuncy(5Hz):
	// Brake Temps(ADC), Ride Height(ADC), Rules Required Flts, Display Flt, Hbeat Flt, SDC, Drive Speed Mode, Motor/Inv Temps, Efuse Flts, Efuse Current (ADC)

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

	// Drive Speed/Model Modes
	update_and_queue_param_u8(&fvcDriveSpeedMode_state,   drive_speed_mode);
	update_and_queue_param_u8(&fvcActiveDriveModel_state, drive_model);

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
	update_and_queue_param_u8(&fvcEfuse3V3SNSFault_state, FVC_LOW_POWER_CHANNELS[0]->flt_state);
	update_and_queue_param_u8(&fvcEfuse5VSNSFault1_state, FVC_LOW_POWER_CHANNELS[1]->flt_state);
	update_and_queue_param_u8(&fvcEfuse5VSNSFault2_state, FVC_LOW_POWER_CHANNELS[2]->flt_state);
	update_and_queue_param_u8(&fvcEfuse5VSWMFault_state,  FVC_LOW_POWER_CHANNELS[3]->flt_state);

	update_and_queue_param_u8(&fvcEfuse12VSNSFault_state,  FVC_HIGH_POWER_CHANNELS[0]->flt_state);
	update_and_queue_param_u8(&fvcEfuse12VDispFault_state, FVC_HIGH_POWER_CHANNELS[1]->flt_state);

	// Git Data
	update_and_queue_param_u8(&fvcGitBranchName, CURRENT_BRANCH);
	update_and_queue_param_u32(&fvcGitHash_decimal, CURRENT_GIT_HASH_DECIMAL);
	update_and_queue_param_u8(&fvcGitHasUncommitedChanges, CURRENT_GIT_HAS_CHANGES);
	update_and_queue_param_u16(&fvcGitHasUncommitedChanges, CURRENT_GIT_CHANGES_COUNT);

}
//drive_snap_loc.drive_enable_state;	
void update_drive_control_can_params(){
	static uin32_t last_drive_snapshot_time_step_num;
	DRIVE_CONTROL_SNAPSHOT drive_snap_loc;
	osMutexWait(driveSnapshotMutexHandle, osWaitForever);
	drive_snap_loc = drive_snapshot;
	osMutexRelease(driveSnapshotMutexHandle);

	float inv_ac_max_curr;
	float inv_dc_max_curr;
	if (drive_snap_loc.drive_timestep_number != last_drive_snapshot_time_step_num){
		switch (drive_model)
		{
			case OPEN_DIFF_NO_PID:
				// Open Diff Specific
				update_and_queue_param_float(&fvcSlip_FL_percent, drive_snap_loc.open_diff_control_outputs.slipFL_percent);
				update_and_queue_param_float(&fvcSlip_FR_percent, drive_snap_loc.open_diff_control_outputs.slipFR_percent);
				update_and_queue_param_float(&fvcSlip_RL_percent, drive_snap_loc.open_diff_control_outputs.slipRL_percent);
				update_and_queue_param_float(&fvcSlip_RR_percent, drive_snap_loc.open_diff_control_outputs.slipRR_percent);

				update_and_queue_param_u8(&fvcSlipStatus_FL_state, drive_snap_loc.open_diff_control_outputs.is_FL_slipping);
				update_and_queue_param_u8(&fvcSlipStatus_FR_state, drive_snap_loc.open_diff_control_outputs.is_FR_slipping);
				update_and_queue_param_u8(&fvcSlipStatus_RL_state, drive_snap_loc.open_diff_control_outputs.is_RL_slipping);
				update_and_queue_param_u8(&fvcSlipStatus_RR_state, drive_snap_loc.open_diff_control_outputs.is_RR_slipping);

				// Total Commands:
				update_and_queue_param_float(&fvcTauTotalCMD_Nm,      drive_snap_loc.open_diff_control_outputs.tauTotalCMD_Nm);
				update_and_queue_param_float(&fvcCurrentTotalCMD_Apk, drive_snap_loc.open_diff_control_outputs.currentCMDTotal_Apk);

				// Wheel Torques
				update_torque_can_params(drive_snap_loc.open_diff_control_outputs.tauFL_Nm, drive_snap_loc.open_diff_control_outputs.tauFR_Nm,
										 drive_snap_loc.open_diff_control_outputs.tauRL_Nm, drive_snap_loc.open_diff_control_outputs.tauRR_Nm,
										 drive_snap_loc.open_diff_control_inputs.tauMaxLimit_Nm);
				
				// Inverter Params
				inv_dc_max_curr = drive_snap_loc.open_diff_control_inputs.dc_currentMaxlimit_A   / 4;
				inv_ac_max_curr = drive_snap_loc.open_diff_control_inputs.ac_currentMaxLimit_Apk / 4;
				update_inverter_can_params(drive_snap_loc.open_diff_control_outputs.currentCMDFL_Apk, drive_snap_loc.open_diff_control_outputs.currentCMDFR_Apk,
										   drive_snap_loc.open_diff_control_outputs.currentCMDRL_Apk, drive_snap_loc.open_diff_control_outputs.currentCMDRR_Apk,
										   inv_ac_max_curr, inv_ac_max_curr, inv_ac_max_curr, inv_ac_max_curr,
										   inv_dc_max_curr, inv_dc_max_curr, inv_dc_max_curr, inv_dc_max_curr,
										   drive_snap_loc.drive_enable_state);
				break;

			case TORQUE_VECTORING:

				break;

			case TORQUE_BY_4:
			default:
				// Total Commands:
				update_and_queue_param_float(&fvcTauTotalCMD_Nm,      drive_snap_loc.tau_by_4_control_outputs.tauTotalCMD_Nm);
				update_and_queue_param_float(&fvcCurrentTotalCMD_Apk, drive_snap_loc.tau_by_4_control_outputs.currentCMDTotal_Apk);
				
				update_torque_can_params(drive_snap_loc.tau_by_4_control_outputs.tauFL_Nm,
									     drive_snap_loc.tau_by_4_control_outputs.tauFR_Nm,
									     drive_snap_loc.tau_by_4_control_outputs.tauRL_Nm,
									     drive_snap_loc.tau_by_4_control_outputs.tauRR_Nm,
									     drive_snap_loc.tau_by_4_control_inputs.tauMaxLimit_Nm);
				
				inv_dc_max_curr = drive_snap_loc.tau_by_4_control_inputs.dc_currentMaxlimit_A / 4;
				inv_ac_max_curr = drive_snap_loc.tau_by_4_control_inputs.ac_currentMaxLimit_Apk / 4;
				update_inverter_can_params(drive_snap_loc.tau_by_4_control_outputs.currentCMDFL_Apk,
										   drive_snap_loc.tau_by_4_control_outputs.currentCMDFR_Apk,
										   drive_snap_loc.tau_by_4_control_outputs.currentCMDRL_Apk,
										   drive_snap_loc.tau_by_4_control_outputs.currentCMDRR_Apk,
										   inv_ac_max_curr, inv_ac_max_curr, inv_ac_max_curr, inv_ac_max_curr,
										   inv_dc_max_curr, inv_dc_max_curr, inv_dc_max_curr, inv_dc_max_curr,
										   drive_snap_loc.drive_enable_state);
				break;
		}	
		

		update_and_queue_param_float(&fvcCurrentTotalCMD_Apk, drive_snap_loc.open_diff_control_outputs.currentCMDTotal_Apk);
		update_and_queue_param_u32(&fvcDriveControlTickStart_ms, drive_snap_loc.drive_control_start_tick);
		update_and_queue_param_u32(&fvcDriveControlTickEnd_ms, 	 drive_snap_loc.drive_control_end_tick);
		update_and_queue_param_u8(&fvcVehicleState_state, 		 drive_snap_loc.drive_vehicle_state);
	}
	last_drive_snapshot_time_step_num = drive_snap_loc.drive_timestep_number;
}

void update_torque_can_params(float tau_cmd_FL, float tau_cmd_FR, float tau_cmd_RL, float tau_cmd_RR, float total_tau_cmd){
	update_and_queue_param_float(&fvcTauTotalCMD_Nm, total_tau_cmd);
	update_and_queue_param_float(&fvcTau_FL_Nm, 	 tau_cmd_FL);
	update_and_queue_param_float(&fvcTau_FR_Nm, 	 tau_cmd_FR);
	update_and_queue_param_float(&fvcTau_RL_Nm, 	 tau_cmd_RL);
	update_and_queue_param_float(&fvcTau_RR_Nm, 	 tau_cmd_RR);
}

void update_inverter_can_params(float ac_cmd_FL, float ac_cmd_FR, float ac_cmd_RL, float ac_cmd_RR,
								float ac_lim_FL, float ac_lim_FR, float ac_lim_RL, float ac_lim_RR,
								float dc_lim_FL, float dc_lim_FR, float dc_lim_RL, float dc_lim_RR,
								bool drive_enable){
	
	// AC Current Command
	invCurrentCmd_FL_Apk.data = ac_cmd_FL;
	invCurrentCmd_FR_Apk.data = ac_cmd_FR;
	invCurrentCmd_RL_Apk.data = ac_cmd_RL;
	invCurrentCmd_RR_Apk.data = ac_cmd_RR;

	// AC Current Max Limit
	invMaxCurrentLimitCmd_FL_Apk.data = ac_lim_FL;
	invMaxCurrentLimitCmd_FR_Apk.data = ac_lim_FR;
	invMaxCurrentLimitCmd_RL_Apk.data = ac_lim_RL;
	invMaxCurrentLimitCmd_RR_Apk.data = ac_lim_RR;
	
	// DC Current Max Limit
	invMaxDCCurrentLimitCmd_FL_A.data = dc_lim_FL;
	invMaxDCCurrentLimitCmd_FR_A.data = dc_lim_FR;
	invMaxDCCurrentLimitCmd_RL_A.data = dc_lim_RL;
	invMaxDCCurrentLimitCmd_RR_A.data = dc_lim_RR;
	
	// Drive Enable
	invDriveEnable_FL_state.data = drive_enable;
	invDriveEnable_FR_state.data = drive_enable;
	invDriveEnable_RL_state.data = drive_enable;
	invDriveEnable_RR_state.data = drive_enable;

	// Send all Params
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
	if(bmsFault_state.data) status = AMS_FAULT;
	else if (vehicle_state == VEHICLE_FAULT) status = INVERTER_FAULT;
	else if(get_both_pedals_fault_state() == TRUE) status = RELEASE_PEDAL;
	else if(rvcBspdFaultActive_state.data) status = BRAKING_FAULT;
	else if(TIMED_SOFTWARE_FAULTS[2]->state) status = APPS_FAULT;
	else if(rvcBspdFaultLatched_state.data) status = BSPD_FAULT;

	return status;
}
