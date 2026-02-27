#ifndef INC_FVC_H
#define INC_FVC_H

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include "stm32f7xx_hal_can.h"

void init_fvc(CAN_HandleTypeDef* BUS_1, CAN_HandleTypeDef* BUS_2, CAN_HandleTypeDef* BUS_3);
void can_buffer_handling_loop();
void main_loop();


void set_dash_lights();

#endif /* INC_steering_wheel_module_26_H */