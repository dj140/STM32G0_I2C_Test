/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "stm32g0xx_ll_i2c.h"
#include "stm32g0xx_ll_iwdg.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_adc.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
  * @brief Slave settings
  */
#include "math.h"

#define HW_i2C
//#define Soft_i2C
//#define ENABLE_LOGGING

#define SLAVE_OWN_ADDRESS                0x48 << 1
#define finger_num                       2

#define RESET_Pin PA15
#define INT_Output_Pin PB5
#define INTn_ZT7548 PB4
#define CE_Pin PA1

typedef struct
{
  uint8_t x;
  uint8_t y;
  uint8_t xy;
  uint8_t strength;
  uint8_t status;
}Melfas_coord;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t strength;
  uint8_t sub_status;
}Zinitix_coord;

typedef struct
{
  uint8_t status;
  uint8_t finger_cnt;
  uint8_t time_stamp; //(Option)
}zinitix_point_info;

extern Melfas_coord Melfas[];
extern Zinitix_coord Zinitix[];
extern uint8_t read_buf[16];




void Error_Handler(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
