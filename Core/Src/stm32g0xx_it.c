/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */

//  /* USER CODE END SysTick_IRQn 0 */

//  /* USER CODE BEGIN SysTick_IRQn 1 */

//  /* USER CODE END SysTick_IRQn 1 */
//}
/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */

	void EXTI4_15_IRQHandler(void)
	{
		/* USER CODE BEGIN EXTI4_15_IRQn 0 */

		/* USER CODE END EXTI4_15_IRQn 0 */
		if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_4) != RESET)
		{
			LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_4);
			/* USER CODE BEGIN LL_EXTI_LINE_15_RISING */
			
			/* Handle user button press in dedicated function */
			ZT7548_INT(); 
			/* USER CODE END LL_EXTI_LINE_15_RISING */
		}
		/* USER CODE BEGIN EXTI4_15_IRQn 1 */


		/* USER CODE END EXTI4_15_IRQn 1 */
	}


/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */

	void EXTI0_1_IRQHandler(void)
	{
		/* USER CODE BEGIN EXTI0_1_IRQn 0 */

		/* USER CODE END EXTI0_1_IRQn 0 */
		if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_1) != RESET)
		{
			LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_1);
			/* USER CODE BEGIN LL_EXTI_LINE_1_FALLING */
//      LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_5);
    	NVIC_SystemReset();
			/* USER CODE END LL_EXTI_LINE_1_FALLING */
		}
		/* USER CODE BEGIN EXTI0_1_IRQn 1 */

		/* USER CODE END EXTI0_1_IRQn 1 */
	}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
    uint32_t status = I2C1->ISR;
 
    if(status & I2C_ISR_ADDR)
    {   // I2C1 Address match event occurs
        LL_I2C_ClearFlag_ADDR(I2C1);
        Address_Matching_Callback();
    }
    else if(status & I2C_ISR_RXNE)
    {
        I2C1->ISR |= I2C_ISR_TXE;
        Slave_Reception_Callback();
    }
    else if(status & I2C_ISR_TXIS)
    {
        I2C1->ISR |= I2C_ISR_TXE;
        Slave_Sending_Callback();
    }
    else if(status & I2C_ISR_STOPF)
    {
			  LL_I2C_ClearFlag_STOP(I2C1);
			  Slave_Complete_Callback();
		}
		else if(status & I2C_ISR_NACKF)
		{
		    LL_I2C_ClearFlag_NACK(I2C1);
		}
		else
		{
    /* Call Error function */
			Error_Callback();
		}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
