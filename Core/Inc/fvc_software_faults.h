#ifndef INC_FVC_SOFTWARE_FAULTS_H_
#define INC_FVC_SOFTWARE_FAULTS_H_

#include "fvc.h"
#include <stdbool.h>

#define NUM_OF_TIMED_FAULTS 3
#define NUM_OF_LOW_POWER_CHANNELS 4
#define NUM_OF_HIGH_POWER_CHANNELS 2
#define TOTAL_EFUSE_RETRIES 3
#define EFUSE_RETRY_DELAY_ms 1000

#define LOW_POW_EFUSE_ENABLED  0
#define LOW_POW_EFUSE_DISABLED 1

#define LOW_POW_EFUSE_FLT      0
#define LOW_POW_EFUSE_NO_FLT   1

#define HIGH_POW_EFUSE_ENABLED   1
#define HIGH_POW_EFUSE_DISABLED  0

#define HIGH_POW_EFUSE_FLT      0
#define HIGH_POW_EFUSE_NO_FLT   1

#define DISPLAY_CURRENT_LIMIT_A 0.600f
#define SNS_12V_CURRENT_LIMIT_A 0.130f
typedef struct {
	float data;
    float max_threshold;
    float min_threshold;
    uint16_t fault_timer;
    uint16_t input_delay_threshold;
    bool state;
} SOFTWARE_FAULT;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_PAIR;

typedef enum {
    NORMAL = 0,
    TRIPPED = 1,
    SHUTDOWN = 2
} EFUSE_MODE;

typedef struct {
    GPIO_PAIR enable_gpio;
    GPIO_PAIR fault_gpio;
    GPIO_PAIR fault_LED_gpio;
    EFUSE_MODE mode;
    bool enabled;
    bool hardware_state;
    bool flt_state;
    uint32_t trip_start_ms;
    uint32_t retry_delay_ms;
    uint8_t  retries_left;
} FVC_LOW_POWER_CHANNEL;

typedef struct {
    GPIO_PAIR enable_gpio;
    GPIO_PAIR fault_gpio;
    GPIO_PAIR fault_LED_gpio;
    EFUSE_MODE mode;
    bool enabled;
    bool hardware_state; // section 8.3.6 in datasheet, overtemp or overvoltage
    bool flt_state;
    uint32_t trip_start_ms;
    uint32_t retry_delay_ms;
    uint8_t  retries_left;
    float    amp_max;
    FLOAT_CAN_STRUCT *amp_can_param;
} FVC_HIGH_POWER_CHANNEL;

extern FVC_LOW_POWER_CHANNEL* FVC_LOW_POWER_CHANNELS[NUM_OF_LOW_POWER_CHANNELS];
extern FVC_HIGH_POWER_CHANNEL* FVC_HIGH_POWER_CHANNELS[NUM_OF_HIGH_POWER_CHANNELS];

extern SOFTWARE_FAULT* TIMED_SOFTWARE_FAULTS[NUM_OF_TIMED_FAULTS];

// Rules Requried + Efuse Data
void update_fault_data();

// Rules Required Check
void update_rules_fault_state();

// Rules Required Getters
bool get_both_pedals_fault_state();
bool get_rules_fault_state();

// Efuse Functions
void init_efuses();
void update_low_power_efuses();
void update_high_power_efuses();
#endif /* INC_FVC_SOFTWARE_FAULTS_H_ */
