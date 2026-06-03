#ifndef INC_SENSOR_AND_CAN_H_
#define INC_SENSOR_AND_CAN_H_
#include <stdbool.h>

typedef enum {
	NONE = 0,
	RELEASE_PEDAL,
	BRAKING_FAULT,
	APPS_FAULT,
	BSPD_FAULT,
	AMS_FAULT,
	IMD_FAULT,
	VCU_FAULT,
	BMS_FAULT,
	INVERTER_FAULT
} DISPLAY_FAULT_STATUS_t;

void update_non_ADC_CAN_params();
void update_drive_control_can_params();
void update_torque_can_params(float tau_cmd_FL, 
							  float tau_cmd_FR, 
							  float tau_cmd_RL, 
							  float tau_cmd_RR);
							
void update_inverter_can_params(float ac_cmd_FL, float ac_cmd_FR, float ac_cmd_RL, float ac_cmd_RR,
								float ac_lim_FL, float ac_lim_FR, float ac_lim_RL, float ac_lim_RR,
								float dc_lim_FL, float dc_lim_FR, float dc_lim_RL, float dc_lim_RR,
								bool drive_enable);

float calc_pedal_percent(float pedalPos_mm, float min_position_mm, float total_range_mm);
float calc_wheel_m_per_s(float electrical_rpm);
DISPLAY_FAULT_STATUS_t determine_disp_fault_status();
#endif /* INC_SENSOR_AND_CAN_H_ */
