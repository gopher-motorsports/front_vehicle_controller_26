#ifndef INC_FVC_H
#define INC_FVC_H

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include "stm32f7xx_hal_can.h"

// ============================= TRACTIVE SYSTEM PARAMS =============================
//Limits
#define BSPD_POWER_LIMIT		4000   // 5 kW limit before a car shutdown is required, 4 kW with buffer
#define MAX_DC_CURRENT_LIMIT    200

//Properties
#define TOTAL_INVERTERS         4      // 2 Dual Package F-Sic Inverters = 4 total inverters
#define MOTOR_POLE_PAIRS   		4 	   // Amount of Pole Pairs of the MTS Motor, 4 pole pairs = 8 poles

// ============================= CAR PARAMS =============================
//Thresholds for "fvc.software_faults.c" 
#define INPUT_TRIP_DELAY_ms   85     // The amount of time it takes an input fault to take effect, margin from 100

#define APPS_MAX_ERROR_POS_mm 23.00f // position where the error begins
#define APPS_MIN_ERROR_POS_mm 1.00f  // position where the error begins

#define FRONT_BRAKE_PRESS_MAX_psi   1700   // maximum brake pressure 1600, do 1700 for margin
#define FRONT_BRAKE_PRESS_MIN_psi   -100   // minimum brake pressue 0, do -100 for margin

#define APPS_CORRELATION_THRESH_percent 10 // greater than 10% difference is a fault between pedals

//Thresholds for APPS/Brake Plausibility
#define APPS_BRAKE_PRESS_THRESH_psi  100.0 //6.25% of brake pressure = mechanical breaks engaged


void init_fvc(CAN_HandleTypeDef* BUS_1, CAN_HandleTypeDef* BUS_2, CAN_HandleTypeDef* BUS_3);
void can_buffer_handling_loop();
void main_loop();

void set_dash_lights();
#endif /* INC_steering_wheel_module_26_H */