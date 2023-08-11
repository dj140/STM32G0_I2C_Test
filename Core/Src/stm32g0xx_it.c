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
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
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
		/* USER CODE BEGIN I2C1_IRQn 0 */
	/* Check ADDR flag value in ISR register */
	if (LL_I2C_IsActiveFlag_ADDR(I2C1))
	{
		/* Verify the Address Match with the OWN Slave address */
		if (LL_I2C_GetAddressMatchCode(I2C1) == SLAVE_OWN_ADDRESS)
		{
				/* Call function Slave Reception Callback */
			Address_Matching_Callback();
			/* Verify the transfer direction, a write direction, Slave enters receiver mode */
			if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
			{
				/* Clear ADDR flag value in ISR register */
				LL_I2C_ClearFlag_ADDR(I2C1);

				/* Enable Receive Interrupt */
				LL_I2C_EnableIT_RX(I2C1);
			}
			else
			{
				/* Clear ADDR flag value in ISR register */
				LL_I2C_ClearFlag_ADDR(I2C1);

				/* Call Error function */
				Error_Callback();
			}
		}
		else
		{
			/* Clear ADDR flag value in ISR register */
			LL_I2C_ClearFlag_ADDR(I2C1);

			/* Call Error function */
			Error_Callback();
		}
	}
  /* Check NACK flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_NACK(I2C1))
  {
    /* End of Transfer */
  LL_I2C_ClearFlag_NACK(I2C1);
  }
	/* Check RXNE flag value in ISR register */
	else if (LL_I2C_IsActiveFlag_RXNE(I2C1))
	{
		/* Call function Slave Reception Callback */
		Slave_Reception_Callback();
	}
	else if(LL_I2C_IsActiveFlag_TXIS(I2C1))
	{
			/* Call function Slave Sending Callback */
		Slave_Sending_Callback();
	}
	/* Check STOP flag value in ISR register */
	else if (LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		/* End of Transfer */
		LL_I2C_ClearFlag_STOP(I2C1);
    /* Check TXE flag value in ISR register */
    if (!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
      /* Flush the TXDR register */
      LL_I2C_ClearFlag_TXE(I2C1);
    }
	}
	  /* Check TXE flag value in ISR register */
  else if (!LL_I2C_IsActiveFlag_TXE(I2C1))
  {
    /* Do nothing */
    /* This Flag will be set by hardware when the TXDR register is empty */
    /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */
  }
	else
	{
		/* Call Error function */
		Error_Callback();
	}
		/* USER CODE END I2C1_IRQn 0 */

		/* USER CODE BEGIN I2C1_IRQn 1 */

		/* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
