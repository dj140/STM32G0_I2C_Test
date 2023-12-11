/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio_init.h"
#include "Wire.h"
#include "Arduino.h"
#include "ZT7548.h"

Melfas_coord Melfas[finger_num];

Zinitix_coord Zinitix[finger_num];

uint8_t read_buf[16];

#ifdef Soft_i2C
TwoWire ZT7548(SCL_Pin, SDA_Pin, SOFT_FAST);
#endif


void SystemClock_Config(void);


/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure the system clock */
  SystemClock_Config();
  Delay_Init();
  // Pull ZT7548_RESET & INTR Pin high
  MX_GPIO_Init();
  /* Initialize all configured peripherals */
  //I2C slave init
  MX_I2C1_Init();
  //I2C master init
  MX_I2C2_Init();
#ifdef ENABLE_LOGGING
  Serial2.begin(115200);
  Serial2.println("Serial printing...");
#endif
  //watchdog init
  MX_IWDG_Init();
  ZT7548_init();
  //Initialize CE & ZT7548 interrupt pin
  interrupt_gpio_Init();

  /* Infinite loop */
  while (1)
  {
    LL_IWDG_ReloadCounter(IWDG);
  }

}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}



/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
#ifdef ENABLE_LOGGING
  Serial2.println("ZT7548 Communication ERROR !!!!!");
  Serial2.println("Please restart the system !!!!!");
#endif
  __disable_irq();
  NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}


