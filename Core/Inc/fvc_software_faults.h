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
    U8_CAN_STRUCT * can_param;
} SOFTWARE_FAULT;

extern SOFTWARE_FAULT* TIMED_SOFTWARE_FAULTS[NUM_OF_TIMED_FAULTS];
void update_struct_fault_data();

#endif /* INC_FVC_SOFTWARE_FAULTS_H_ */
