/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BrakePress_Front_Pin GPIO_PIN_1
#define BrakePress_Front_GPIO_Port GPIOC
#define ShockPot_FR_Pin GPIO_PIN_0
#define ShockPot_FR_GPIO_Port GPIOA
#define ShockPot_FL_Pin GPIO_PIN_1
#define ShockPot_FL_GPIO_Port GPIOA
#define Steering_Angle_Pin GPIO_PIN_2
#define Steering_Angle_GPIO_Port GPIOA
#define APPS1_Pin GPIO_PIN_3
#define APPS1_GPIO_Port GPIOA
#define BrakeTemp_FL_Pin GPIO_PIN_6
#define BrakeTemp_FL_GPIO_Port GPIOA
#define BrakeTemp_FR_Pin GPIO_PIN_7
#define BrakeTemp_FR_GPIO_Port GPIOA
#define APPS2_Pin GPIO_PIN_4
#define APPS2_GPIO_Port GPIOC
#define RideHeight_Pin GPIO_PIN_5
#define RideHeight_GPIO_Port GPIOC
#define Display_12V_Current_Pin GPIO_PIN_0
#define Display_12V_Current_GPIO_Port GPIOB
#define Sensor_12V_Current_Pin GPIO_PIN_1
#define Sensor_12V_Current_GPIO_Port GPIOB
#define BMS_Dash_Light_Pin GPIO_PIN_10
#define BMS_Dash_Light_GPIO_Port GPIOB
#define IMD_Dash_Light_Pin GPIO_PIN_11
#define IMD_Dash_Light_GPIO_Port GPIOB
#define DISP_12V_Flt_Pin GPIO_PIN_8
#define DISP_12V_Flt_GPIO_Port GPIOD
#define SNS_12V_Flt_Pin GPIO_PIN_9
#define SNS_12V_Flt_GPIO_Port GPIOD
#define DISP_12V_EN_Pin GPIO_PIN_10
#define DISP_12V_EN_GPIO_Port GPIOD
#define SNS_12V_EN_Pin GPIO_PIN_11
#define SNS_12V_EN_GPIO_Port GPIOD
#define DISP_FLT_LED_Pin GPIO_PIN_14
#define DISP_FLT_LED_GPIO_Port GPIOD
#define SNS_12V_Flt_LED_Pin GPIO_PIN_15
#define SNS_12V_Flt_LED_GPIO_Port GPIOD
#define SWM_Flt_LED_Pin GPIO_PIN_6
#define SWM_Flt_LED_GPIO_Port GPIOC
#define SNS_5V_Flt_2_LED_Pin GPIO_PIN_7
#define SNS_5V_Flt_2_LED_GPIO_Port GPIOC
#define SNS_5V_Flt_1_LED_Pin GPIO_PIN_8
#define SNS_5V_Flt_1_LED_GPIO_Port GPIOC
#define SNS_3V3_Flt_LED_Pin GPIO_PIN_9
#define SNS_3V3_Flt_LED_GPIO_Port GPIOC
#define SDC2_Pin GPIO_PIN_9
#define SDC2_GPIO_Port GPIOA
#define SDC1_Pin GPIO_PIN_10
#define SDC1_GPIO_Port GPIOA
#define SWM_5V_EN_Pin GPIO_PIN_10
#define SWM_5V_EN_GPIO_Port GPIOC
#define SNS_5V_EN2_Pin GPIO_PIN_11
#define SNS_5V_EN2_GPIO_Port GPIOC
#define SNS_5V_EN1_Pin GPIO_PIN_12
#define SNS_5V_EN1_GPIO_Port GPIOC
#define SNS_3V3_EN_Pin GPIO_PIN_0
#define SNS_3V3_EN_GPIO_Port GPIOD
#define SNS_5V_Flt_2_Pin GPIO_PIN_1
#define SNS_5V_Flt_2_GPIO_Port GPIOD
#define SNS_5V_Flt_1_Pin GPIO_PIN_2
#define SNS_5V_Flt_1_GPIO_Port GPIOD
#define SNS_3V3_Flt_Pin GPIO_PIN_3
#define SNS_3V3_Flt_GPIO_Port GPIOD
#define SWM_5V_Flt_Pin GPIO_PIN_4
#define SWM_5V_Flt_GPIO_Port GPIOD
#define Fault_Pin GPIO_PIN_5
#define Fault_GPIO_Port GPIOB
#define Gsense_Pin GPIO_PIN_6
#define Gsense_GPIO_Port GPIOB
#define HBeat_Pin GPIO_PIN_7
#define HBeat_GPIO_Port GPIOB
#define GIT_Other_Pin GPIO_PIN_9
#define GIT_Other_GPIO_Port GPIOB
#define GIT_Feature_Pin GPIO_PIN_0
#define GIT_Feature_GPIO_Port GPIOE
#define GIT_Main_Pin GPIO_PIN_1
#define GIT_Main_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
