#ifndef INC_FVC_SOFTWARE_FAULTS_H_
#define INC_FVC_SOFTWARE_FAULTS_H_

#include "fvc.h"
#include <stdbool.h>

#define NUM_OF_TIMED_FAULTS 3
typedef struct {
	float data;
    float max_threshold;
    float min_threshold;
    uint16_t fault_timer;
    uint16_t input_delay_threshold;
    bool state;
} SOFTWARE_FAULT;

extern SOFTWARE_FAULT* TIMED_SOFTWARE_FAULTS[NUM_OF_TIMED_FAULTS];

// Rules Requried + Efuse Data
void update_fault_data();

// Rules Required Check
void update_rules_fault_state();

// Rules Required Getters
bool get_both_pedals_fault_state();
bool get_rules_fault_state();

// Efuse Fault States
void update_efuse_fault_states();
#endif /* INC_FVC_SOFTWARE_FAULTS_H_ */
