#ifndef INC_CONDITIONS_AND_UTILS_H_
#define INC_CONDITIONS_AND_UTILS_H_

#include <stdbool.h>
#include <stdint.h>

// LED Code
#define HBEAT_LED_DELAY_TIME_ms 500
void hbeat_blink();
bool get_Hbeat_status();
void set_dash_lights();

// General Utilities
int16_t max4Ints(int16_t a, int16_t b, int16_t c, int16_t d);
float clamp(float data, float min, float max);

// Rules Required Faults
bool get_both_pedals_fault_state();

// Inverter Limit Functions
float calculate_dc_current_limit();

// Inverter Condition Functions
bool predrive_conditions_met();
bool has_inverter_comms();
bool inverter_fault_active();
bool get_slow_mode_status();

// Git LEDs
void init_git_LEDs();

// Drive Speed/Model Modes
void determine_drive_speed_mode();
void determine_drive_model();

#endif /* INC_CONDITIONS_AND_UTILS_H_ */
