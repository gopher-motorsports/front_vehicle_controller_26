#ifndef INC_FVC_H
#define INC_FVC_H

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include <stdbool.h>
#include "stm32f7xx_hal_can.h"

// ======================================== I/O General PARAMETERS ======================================
#define NMOS_ON (GPIO_PIN_SET)
#define NMOS_OFF (GPIO_PIN_RESET)
#define PRESSED 1
#define RELEASED 0
#define ON 1
#define OFF 0
// ==============================================================================================

// ============================= TRACTIVE SYSTEM PARAMS =============================
//Limits
#define TRACTIVE_MARGIN                         
#define BSPD_POWER_LIMIT		                  	4000.0   // 5 kW limit before a car shutdown is required, 4 kW with buffer
#define DC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_A    	125.15 // DC Current Limit at Max Pack Voltage
#define AC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_Apk  	390    // TODO: PLACEHOLDER Replace with better value
#define AC_CURRENT_SLOW_MODE_MAX_Apk  				AC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_Apk / 3    // TODO: PLACEHOLDER Replace with better value
#define MAX_PACK_VOLTAGE                          	599.2  
#define NOMINAL_PACK_VOLTAGE                      	420   // 3.0V per cell * 140 cells = 420V
//Properties
#define TOTAL_INVERTERS         4       // 2 Dual Package F-Sic Inverters = 4 total inverters
#define MOTOR_POLE_PAIRS   		4 	    // Amount of Pole Pairs of the MTS Motor, 4 pole pairs = 8 poles
#define DRIVE_TRAIN_GEAR_RATIO	12.8	// Drive Train Gear Ratio
#define WHEEL_RADIUS			0.2032	// Wheel Raidus = 8 inches --> 0.2032 in
// ============================= CAR PARAMS =============================
//Thresholds for "fvc.software_faults.c" 
#define INPUT_TRIP_DELAY_ms   85     // The amount of time it takes an input fault to take effect, margin from 100

#define APPS_1_MAX_CURRENT_POS_mm  18.25f // The position of the pedal at 100% torque
#define APPS_1_MIN_CURRENT_POS_mm  6.25f  // The position of the pedal at 0% torque
#define APPS_2_MAX_CURRENT_POS_mm  17.50f // The position of the pedal at 100% torque
#define APPS_2_MIN_CURRENT_POS_mm  4.50f  // The position of the pedal at 0% torque
#define APPS_1_TOTAL_TRAVEL_mm ( APPS_1_MAX_CURRENT_POS_mm - APPS_1_MIN_CURRENT_POS_mm )
#define APPS_2_TOTAL_TRAVEL_mm ( APPS_2_MAX_CURRENT_POS_mm - APPS_2_MIN_CURRENT_POS_mm )

#define APPS_MAX_ERROR_POS_mm 25.00f // position where the error begins
#define APPS_MIN_ERROR_POS_mm 0.50f  // position where the error begins
#define APPS_CORRELATION_THRESH_percent 10 // greater than 10% difference is a fault between pedals

#define FRONT_BRAKE_PRESS_MAX_psi   1700   // maximum brake pressure 1600, do 1700 for margin
#define FRONT_BRAKE_PRESS_MIN_psi   -100   // minimum brake pressue 0, do -100 for margin

//Thresholds for APPS/Brake Plausibility
#define APPS_BRAKE_PRESS_THRESH_psi  100.0 //6.25% of brake pressure = mechanical breaks engaged

// ============================= INVERTER PARAMS =============================
#define INVERTER_TIMEOUT_ms     2000    // The time after which the vehicle will enter STARTUP

//Ready to Drive
#define PREDRIVE_BUTTON_PARAM      swButon4_state
#define PRESSED 				   1
#define RELEASED 				   0
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define PREDRIVE_BRAKE_THRESH_psi  20   // The minimum brake pressure to enter the driving state
#define TS_ON_THRESHOLD_VOLTAGE_V  60   // Minimum pack voltage to enter driving TODO replace with 420V?

//Inverter Faults
#define INVERTER_NO_FAULT         0x00  // The data of the fault code when there is no inverter fault
#define INVERTER_OV_FAULT	  	  0x01  // Inverter Controller Overtemp
#define INVERTER_UV_FAULT         0x02  // Undervoltage fault code for the inverter, have to set threshold via DTI tool
#define INVERTER_CTRL_TEMP_FAULT  0x04  // Inverter Controller Overtemp of user setpoint
#define INVERTER_MOTOR_TEMP_FAULT 0x05	// Inverter Motor Overtemp of user setpoint
//Inverter Variables:

// ============================= Drive Control PARAMS =============================
#define TRACTION_LIMIT_percent    15.0	// Past this wheel slip we have broken traction
#define MOTOR_MAX_TORQUE_Nm	  36				  
// ============================= Misc/MATH PARAMS =============================
#define MATH_PI             3.14159265
#define SECONDS_PER_MINUTE	60
#define ERPM_TO_M_PER_S		(1 / MOTOR_POLE_PAIRS) * (1 / DRIVE_TRAIN_GEAR_RATIO) * (2 * WHEEL_RADIUS * MATH_PI) * (1 / SECONDS_PER_MINUTE)

typedef enum
{
	VEHICLE_NO_COMMS  = 0, // When the inverter first turns on and if there is ever a loss of communication
	VEHICLE_FAULT     = 1, // The vehicle can detect that the inverter is Faulting
	VEHICLE_STANDBY   = 2, // The inverter has exited lockout but no torque commands will be sent
	VEHICLE_PREDRIVE  = 3, // The vehicle buzzer is active and the driving state will be entered
	VEHICLE_DRIVING   = 4, // Torque commands are actively being sent from APPS positions
} VEHICLE_STATE_t;

typedef struct {
	float slip_tract_limit_percent;
	float car_speed;
	float wheel_speed_FL;
	float wheel_speed_FR;
	float wheel_speed_RL;
	float wheel_speed_RR;
	float throttle_percent;
	float tauMaxLimit_Nm;
	float ac_currentMaxLimit_Apk;
	float dc_currentMaxlimit_A;
} DRIVE_CONTROL_INPUTS;

typedef struct {
	float slipFL_percent;
	float slipFR_percent;	
	float slipRL_percent;
	float slipRR_percent;
	bool  is_FL_slipping;
	bool  is_FR_slipping;
	bool  is_RL_slipping;
	bool  is_RR_slipping;
	float tauFL_Nm;
	float tauFR_Nm;
	float tauRL_Nm;
	float tauRR_Nm;
	float tauTotalCMD_Nm;
	float currentCMDFL_Apk;
	float currentCMDFR_Apk;
	float currentCMDRL_Apk;
	float currentCMDRR_Apk;
	float currentCMDTotal_Apk;
} DRIVE_CONTROL_OUTPUTS;

void init_fvc(CAN_HandleTypeDef* BUS_1, CAN_HandleTypeDef* BUS_2, CAN_HandleTypeDef* BUS_3);
void can_buffer_handling_loop();
uint32_t get_drive_control_timestep_start();
void main_loop();
void process_inverter();

extern VEHICLE_STATE_t vehicle_state;
extern DRIVE_CONTROL_INPUTS open_diff_inputs;
extern DRIVE_CONTROL_OUTPUTS open_diff_outputs;
#endif /* INC_steering_wheel_module_26_H */