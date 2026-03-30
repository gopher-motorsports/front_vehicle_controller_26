#ifndef INC_SENSOR_AND_CAN_H_
#define INC_SENSOR_AND_CAN_H_

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
void update_inverter_params();

float calc_pedal_percent(float pedalPos_mm, float min_position_mm, float total_range_mm);
float calc_wheel_m_per_s(float electrical_rpm);
DISPLAY_FAULT_STATUS_t determine_disp_fault_status();
#endif /* INC_SENSOR_AND_CAN_H_ */
