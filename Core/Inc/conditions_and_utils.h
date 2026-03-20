#ifndef INC_CONDITIONS_AND_UTILS_H_
#define INC_CONDITIONS_AND_UTILS_H_

#include <stdbool.h>
// LED Code
#define HBEAT_LED_DELAY_TIME_ms 500
void LED_task();

// Sensor Faults/Limits
float calculate_dc_current_limit();
bool is_vehicle_faulting();
void set_dash_lights();
bool predrive_conditions_met();

#endif /* INC_CONDITIONS_AND_UTILS_H_ */
