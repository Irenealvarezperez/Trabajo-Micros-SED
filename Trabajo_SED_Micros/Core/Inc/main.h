/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENABLE_PUENTE_H_Pin GPIO_PIN_5
#define ENABLE_PUENTE_H_GPIO_Port GPIOE
#define Sensor_temperatura_Pin GPIO_PIN_1
#define Sensor_temperatura_GPIO_Port GPIOC
#define BOTON_BLOQUEO_Pin GPIO_PIN_0
#define BOTON_BLOQUEO_GPIO_Port GPIOA
#define BOTON_BLOQUEO_EXTI_IRQn EXTI0_IRQn
#define BOTON_ABRIR_PUERTA_Pin GPIO_PIN_1
#define BOTON_ABRIR_PUERTA_GPIO_Port GPIOA
#define BOTON_ABRIR_PUERTA_EXTI_IRQn EXTI1_IRQn
#define TRIG_ULTRASONIDOS_Pin GPIO_PIN_2
#define TRIG_ULTRASONIDOS_GPIO_Port GPIOA
#define LDR_Pin GPIO_PIN_3
#define LDR_GPIO_Port GPIOA
#define BOTON_ALARMA_Pin GPIO_PIN_4
#define BOTON_ALARMA_GPIO_Port GPIOA
#define BOTON_ALARMA_EXTI_IRQn EXTI4_IRQn
#define ECHO_ULTRASONIDOS_Pin GPIO_PIN_9
#define ECHO_ULTRASONIDOS_GPIO_Port GPIOE
#define ZUMBADOR_Pin GPIO_PIN_12
#define ZUMBADOR_GPIO_Port GPIOD
#define LED_sensorTemp_Pin GPIO_PIN_14
#define LED_sensorTemp_GPIO_Port GPIOD
#define IN2_Pin GPIO_PIN_6
#define IN2_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_7
#define IN1_GPIO_Port GPIOC
#define LUZ_Pin GPIO_PIN_8
#define LUZ_GPIO_Port GPIOC
#define LED_GARAJE_Pin GPIO_PIN_9
#define LED_GARAJE_GPIO_Port GPIOC
#define IN2_ventilador_Pin GPIO_PIN_8
#define IN2_ventilador_GPIO_Port GPIOA
#define IN1_ventilador_Pin GPIO_PIN_9
#define IN1_ventilador_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_15
#define SERVO_GPIO_Port GPIOA
#define ENABLE_VENTILADOR_Pin GPIO_PIN_8
#define ENABLE_VENTILADOR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
