#ifndef INC_FVC_H
#define INC_FVC_H

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f7xx_hal_can.h"

// ======================================== I/O General PARAMETERS ======================================
#define NMOS_ON (GPIO_PIN_SET)
#define NMOS_OFF (GPIO_PIN_RESET)
#define PRESSED 1
#define RELEASED 0
#define ON 1
#define OFF 0
#define CLOSED 1
#define OPEN   0
// ==============================================================================================

// ============================= TRACTIVE SYSTEM PARAMS =============================
//Limits              
#define BSPD_POWER_LIMIT_W		                  	4000.0f   // 5 kW limit before a car shutdown is required, 4 kW with buffer
#define RULES_POWER_LIMIT_W						    78000.0f  // 78 kw limit for an 80kw limit with 2 kw margin 
#define DC_CURRENT_LIMIT_AT_MAX_PACK_VOLTAGE_A    	125.15f // DC Current Limit at Max Pack Voltage

#define INVERTER_MAX_CURRENT_Apk  					120.0f	 // Max Apk command for each inverter
#define TOTAL_INVERTER_MAX_CURRENT_Apk				INVERTER_MAX_CURRENT_Apk * 4
#define MAX_PACK_VOLTAGE                          	599.2f  
#define NOMINAL_PACK_VOLTAGE                      	420.0f   // 3.0V per cell * 140 cells = 420V

// Motor Properties
#define Kt						0.257f
#define TOTAL_INVERTERS         4       // 2 Dual Package F-Sic Inverters = 4 total inverters
#define TOTAL_INVERTERS_2WD     2		// 1 Dual package F-Sic Inverter  = 2 total inverters in 2 wheel drive
#define MOTOR_POLE_PAIRS   		4 	    // Amount of Pole Pairs of the MTS Motor, 4 pole pairs = 8 poles
#define DRIVE_TRAIN_GEAR_RATIO	12.8f	// Drive Train Gear Ratio
#define WHEEL_RADIUS			0.2032f	// Wheel Raidus = 8 inches --> 0.2032 in

// Motor Limits
#define MOTOR_MAX_TORQUE_Nm	  		30.84 // 120 Apk * .257 = 30.84Nm
#define TOTAL_TORQUE_LIMIT_Nm		MOTOR_MAX_TORQUE_Nm * 4
#define TOTAL_TORQUE_LIMIT_2WD_Nm   MOTOR_MAX_TORQUE_Nm * 2
#define TOTAL_TORQUE_LIMIT_1WD_Nm	MOTOR_MAX_TORQUE_Nm * 1
// ============================= CAR PARAMS =============================
//Thresholds for "fvc.software_faults.c" 
#define INPUT_TRIP_DELAY_ms   85     // The amount of time it takes an input fault to take effect, margin from 100

#define APPS_1_MAX_CURRENT_POS_mm  25.00f // The position of the pedal at 100% torque
#define APPS_1_MIN_CURRENT_POS_mm  11.61f  // The position of the pedal at 0% torque, 3.85mm resting + 1mm = 4.85mm
#define APPS_2_MAX_CURRENT_POS_mm  25.00f // The position of the pedal at 100% torque
#define APPS_2_MIN_CURRENT_POS_mm  9.84f  // The position of the pedal at 0% torque, 1.86 mm resting + 1mm = 2.86mm
#define APPS_1_TOTAL_TRAVEL_mm ( APPS_1_MAX_CURRENT_POS_mm - APPS_1_MIN_CURRENT_POS_mm )
#define APPS_2_TOTAL_TRAVEL_mm ( APPS_2_MAX_CURRENT_POS_mm - APPS_2_MIN_CURRENT_POS_mm )

#define APPS_MAX_ERROR_POS_mm 25.00f // position where the error begins
#define APPS_MIN_ERROR_POS_mm 0.50f  // position where the error begins
#define APPS_CORRELATION_THRESH_percent 10.0f // greater than 10% difference is a fault between pedals

#define FRONT_BRAKE_PRESS_MAX_psi   1700.0f   // maximum brake pressure 1600, do 1700 for margin
#define FRONT_BRAKE_PRESS_MIN_psi   -100.0f   // minimum brake pressue 0, do -100 for margin

//Thresholds for APPS/Brake Plausibility
#define APPS_BRAKE_PRESS_THRESH_psi  100.0f //6.25% of brake pressure = mechanical breaks engaged

// ============================= INVERTER PARAMS =============================
#define INVERTER_TIMEOUT_ms     1000    // Amount of time before inverter times out, enable must ping it twice as fast, Set in DTI Can Tool

//Ready to Drive
#define PREDRIVE_BUTTON_PARAM      swButon4_state
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define PREDRIVE_BRAKE_THRESH_psi  20.0f  // The minimum brake pressure to enter the driving state
#define TS_ON_THRESHOLD_VOLTAGE_V  60.0f  // Minimum pack voltage to enter driving TODO replace with 420V?

//Inverter Faults
#define INVERTER_NO_FAULT         0x00  // The data of the fault code when there is no inverter fault
#define INVERTER_OV_FAULT	  	  0x01  // Inverter Controller Overtemp
#define INVERTER_UV_FAULT         0x02  // Undervoltage fault code for the inverter, have to set threshold via DTI tool
#define INVERTER_CTRL_TEMP_FAULT  0x04  // Inverter Controller Overtemp of user setpoint
#define INVERTER_MOTOR_TEMP_FAULT 0x05	// Inverter Motor Overtemp of user setpoint
//Inverter Variables:

// ============================= Drive Mode PARAMS =============================
#define SLOW_MODE_BUTTON     	   swButon1_state
#define DRIVE_MODEL_BUTTON         swButon5_state
#define TOTAL_DRIVE_MODES				4
#define SLOW_MODE_HOLD_TIME_THRESH		2000
#define DRIVE_MODEL_HOLD_TIME_THRESH 	1000
// ============================= Drive Control PARAMS =============================
#define TRACTION_LIMIT_decimal          0.15f	// Past this wheel slip we have broken traction

// ============================= FreeRTOS PARAMS =============================
#define IDLE_TASK_hz	  		  		5
#define GCAN_BUFFER_TASK_hz	  	  		1000
#define DEBUG_TASK_hz			  		2
#define DRIVE_TASK_hz			  		100
#define FAULT_TASK_hz			  		1000
#define TELEMETRY_TASK_hz		  		200

#define IDLE_TASK_PERIOD_ms	  			1000 / IDLE_TASK_hz
#define GCAN_BUFFER_TASK_PERIOD_ms		1000 / GCAN_BUFFER_TASK_hz
#define DEBUG_TASK_PERIOD_ms			1000 / DEBUG_TASK_hz
#define DRIVE_TASK_PERIOD_ms			1000 / DRIVE_TASK_hz
#define FAULT_TASK_PERIOD_ms			1000 / FAULT_TASK_hz
#define TELEMETRY_TASK_PERIOD_ms		1000 / TELEMETRY_TASK_hz
// ============================= Misc/MATH PARAMS =============================
#define MATH_PI             3.14159265f
#define SECONDS_PER_MINUTE	60.0f
#define ERPM_TO_M_PER_S		(1.0f / MOTOR_POLE_PAIRS) * (1.0f / DRIVE_TRAIN_GEAR_RATIO) * (2.0f * WHEEL_RADIUS * MATH_PI) * (1.0f / SECONDS_PER_MINUTE)
#define DEBUG_PARAM_COUNT	10

typedef enum
{
	FULL_POWER = 0,
	SLOW 	   = 1
} DRIVE_SPEED_MODE_t;

typedef enum
{
	TORQUE_BY_4 	 	= 0,
	OPEN_DIFF_NO_PID 	= 1,
	OPEN_DIFF        	= 2,
	TORQUE_VECTORING 	= 3,
	TORQUE_BY_2_FWD     = 4,
	TORQUE_BY_2_RWD_RL  = 5
} DRIVE_MODEL_MODES_t;

typedef enum
{
	VEHICLE_NO_COMMS  = 0, // When the inverter first turns on and if there is ever a loss of communication
	VEHICLE_FAULT     = 1, // The vehicle can detect that the inverter is Faulting
	VEHICLE_STANDBY   = 2, // The inverter has exited lockout but no torque commands will be sent
	VEHICLE_PREDRIVE  = 3, // The vehicle buzzer is awhat if Ictive and the driving state will be entered
	VEHICLE_DRIVING   = 4, // Torque commands are actively being sent from APPS positions
} VEHICLE_STATE_t;

typedef struct {
	float wheel_speed_FL;
	float wheel_speed_FR;
	float wheel_speed_RL;
	float wheel_speed_RR;
	float throttle_percent;
	float steering_angle;
} FVC_DRIVE_SENSOR_DATA;

typedef struct {
	float throttle_percent;
	float tauMaxLimit_Nm;
	float ac_currentMaxLimit_Apk;
	float dc_currentMaxlimit_A;
} TORQUE_BY_4_INPUTS;

typedef struct {
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
} TORQUE_BY_4_OUTPUTS;

typedef struct {
	float slip_tract_limit_decimal;
	float car_speed;
	float yaw_rate;
	FVC_DRIVE_SENSOR_DATA fvc_drive_sensor_data;
	float tauMaxLimit_Nm;
	float ac_currentMaxLimit_Apk;
	float dc_currentMaxlimit_A;
} OPEN_DIFF_NO_PID_INPUTS;

typedef struct {
	float slip_tract_limit_decimal;
	float car_speed;
	float yaw_rate;
	bool integral_reset;
	float P;
	float I;
	FVC_DRIVE_SENSOR_DATA fvc_drive_sensor_data;
	float tauMaxLimit_Nm;
	float ac_currentMaxLimit_Apk;
	float dc_currentMaxlimit_A;
} OPEN_DIFF_INPUTS;

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
} OPEN_DIFF_NO_PID_OUTPUTS;

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
} OPEN_DIFF_OUTPUTS;

typedef union {
    OPEN_DIFF_NO_PID_INPUTS OD_no_pid;
    OPEN_DIFF_INPUTS        OD;
	TORQUE_BY_4_INPUTS      T_by_4;
    TORQUE_BY_4_INPUTS		T_by_2_fwd;
    TORQUE_BY_4_INPUTS	    T_by_2_rwd;
} DRIVE_MODEL_INPUTS;

typedef union {
    OPEN_DIFF_NO_PID_OUTPUTS OD_no_pid;
    OPEN_DIFF_OUTPUTS        OD;
    TORQUE_BY_4_OUTPUTS      T_by_4;
	TORQUE_BY_4_OUTPUTS		 T_by_2_fwd;
	TORQUE_BY_4_OUTPUTS		 T_by_2_rwd;
} DRIVE_MODEL_OUTPUTS;

typedef struct{
	DRIVE_MODEL_INPUTS    inputs;
    DRIVE_MODEL_OUTPUTS   outputs;
    uint32_t 			  drive_control_start_tick;
	uint32_t 			  drive_control_end_tick;
	bool 				  drive_enable_state;
	VEHICLE_STATE_t		  drive_vehicle_state;
	uint32_t			  drive_timestep_number;
	DRIVE_MODEL_MODES_t	  drive_active_model;
} DRIVE_CONTROL_SNAPSHOT;

// Init
void init_fvc(CAN_HandleTypeDef* BUS_1, CAN_HandleTypeDef* BUS_2, CAN_HandleTypeDef* BUS_3, UART_HandleTypeDef* huart_debug);

// FreeRTOS
void can_buffer_handling_loop();
void idle_task();
void debug_task();
void drive_task();
void telemetry_task();
void fault_task();

extern osMutexId fvcDriveSensorsMutexHandle;
extern osMutexId driveSnapshotMutexHandle;

// Drive Control
void process_inverter();
void update_drive_inputs();
void run_simulink_model_and_update_drive_outputs();
void publish_drive_control_snapshot();

// FVC/Inverter Debug
void clear_serial_monitor();
void send_debug_param_group(const char *name, float params[DEBUG_PARAM_COUNT]);
void update_debug_params(void);

extern VEHICLE_STATE_t vehicle_state;
extern DRIVE_SPEED_MODE_t drive_speed_mode;
extern DRIVE_MODEL_MODES_t drive_model;

extern FVC_DRIVE_SENSOR_DATA fvc_drive_sensor_data_global;
extern DRIVE_CONTROL_SNAPSHOT drive_snapshot;
#endif /* INC_steering_wheel_module_26_H */