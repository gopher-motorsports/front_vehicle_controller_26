#ifndef INC_CONDITIONS_AND_UTILS_H_
#define INC_CONDITIONS_AND_UTILS_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct
{
    float cutoff_freq_Hz;   // Lowpass cutoff frequency
    float sample_time_s;    // Time step between filter updates
    float alpha;            // Filter coefficient

    float prev_output;      // Previous filtered value
    bool initialized;       // Prevents startup transient
} LOWPASS_FILTER;

#define THROTTLE_CUTOFF_FREQ_Hz       10
#define STEERING_ANGLE_CUTOFF_FREQ_Hz 10
#define WHEEL_SPEED_CUTOFF_FREQ_Hz    25
#define SENSOR_SAMPLE_TIME_ms         0.005         

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
void determine_drive_power_limits(float *ac_limit_Apk, float *dc_limit_A, bool *drive_enable);

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

// Filtering
float LPF_compute_alpha(float cutoff_freq_Hz, float sample_time_s);
void  LPF_init(LOWPASS_FILTER *filter, float cutoff_freq_Hz, float sample_time_s);
float LPF(LOWPASS_FILTER *filter, float input);

extern LOWPASS_FILTER APPS1_LPF;
extern LOWPASS_FILTER APPS2_LPF;
extern LOWPASS_FILTER speed_LPF_FL;
extern LOWPASS_FILTER speed_LPF_FR;
extern LOWPASS_FILTER speed_LPF_RL;
extern LOWPASS_FILTER speed_LPF_RR;
extern LOWPASS_FILTER steering_angle_LPF;

#endif /* INC_CONDITIONS_AND_UTILS_H_ */
