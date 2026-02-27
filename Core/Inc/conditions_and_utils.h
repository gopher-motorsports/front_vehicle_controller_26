#ifndef INC_CONDITIONS_AND_UTILS_H_
#define INC_CONDITIONS_AND_UTILS_H_

// LED Code
#define HBEAT_LED_DELAY_TIME_ms 500
void LED_task();

// Tractive System Faults/Limits
float calculate_dc_current_limit();
#endif /* INC_CONDITIONS_AND_UTILS_H_ */
