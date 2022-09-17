/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
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
#define GB_CS_Pin GPIO_PIN_1
#define GB_CS_GPIO_Port GPIOA
#define GB_WRITE_Pin GPIO_PIN_2
#define GB_WRITE_GPIO_Port GPIOA
#define GB_READ_Pin GPIO_PIN_3
#define GB_READ_GPIO_Port GPIOA
#define GB_RESET_Pin GPIO_PIN_4
#define GB_RESET_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define ADDRESS_BUS GPIOB
#define MEMORY_BUS GPIOA

#define BIOS_START_ADDR (uint16_t)0x00 // bios and app will prob be stored in spi flash

#define CART_MEM_SIZE 0x4000

typedef enum
{
	BIOS = 0,
	GAME,
	CONTROLLER
}State_e;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
