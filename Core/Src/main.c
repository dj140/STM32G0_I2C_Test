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
#include "gpio.h"
#include "Arduino.h"

/**
  * @brief Variables related to SlaveReceive process
  */
uint8_t      aReceiveBuffer[0xF] = {0};
__IO uint8_t ubReceiveIndex      = 0;

uint8_t  Buffer_Rx_IIC1[40];
uint8_t  Rx_Idx_IIC1=0;
uint8_t  Tx_Idx_IIC1=0;
uint8_t Response_Message[40];

uint8_t read_buf[40];
uint8_t cmd_buf[10];
uint8_t flag = 1;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void MX_GPIO_Init(void)
{

  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);

  /**/
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE8);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_8;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* LSI configuration and activation */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 12, LL_RCC_PLLR_DIV_4);
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

  LL_Init1msTick(48000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void Address_Matching_Callback(void)
{
		    Rx_Idx_IIC1=0;
        Tx_Idx_IIC1=0;
}
/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Reception_Callback(void)
{
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */
  Buffer_Rx_IIC1[Rx_Idx_IIC1++] = LL_I2C_ReceiveData8(I2C1);
	switch(Buffer_Rx_IIC1[0])
	{
			case 0xAE:  
					Response_Message[0] = 0x26;				
					break;
			case 0x0F:  
					Response_Message[0] = 0x08;				
					break;
			case 0xE1:  
					Response_Message[0] = 0x04;			
					Response_Message[1] = 0x82;				
					Response_Message[2] = 0x07;				
					Response_Message[3] = 0x00;				
					break;	
			case 0xF6:  
					Response_Message[0] = 0x55;
					Response_Message[1] = 0x56;
					Response_Message[2] = 0x32;
					Response_Message[3] = 0x00;
					Response_Message[4] = 0x00;
					Response_Message[5] = 0x00;
					Response_Message[6] = 0x00;
					Response_Message[7] = 0x00;
					Response_Message[8] = 0x11;
					Response_Message[9] = 0x08;			
					break;	
			case 0x02:  
					Response_Message[0] = 0x54;
					Response_Message[1] = 0x00;
					Response_Message[2] = 0x00;
					Response_Message[3] = 0x00;		
					break;		
			case 0x0B:  
					Response_Message[0] = 0x13;
					Response_Message[1] = 0x0C;
					Response_Message[2] = 0x00;		
					break;		
			case 0xE2:  
					Response_Message[0] = 0x82;
					break;		
			case 0xE3:  
					Response_Message[0] = 0x07;
					break;				
			case 0x10:  // Get Point status
					for(int i = 0; i < 40; i++)
					{
						
							Response_Message[i] = 0x00;
					}
					if(read_buf[2] == 0x02)		// if detet 2 finger
					{
						flag++;
						if(flag == 0x01)
						{
							//Finger 0 event info (touch / event type / hover / palm / event id[0~3])
							Response_Message[0] = 0xA2;
							
							//Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
							Response_Message[1] = (read_buf[13] << 4) | (read_buf[11] & 0x0F);
						
							//Finger 0 x coordinate (bit 7 ~ bit 0)
							Response_Message[2] = read_buf[10] ;
						
							//Finger 0 y coordinate (bit 7 ~ bit 0)
							Response_Message[3] = read_buf[12] ;

							//Finger 0 z (strength)
							Response_Message[4] = read_buf[14] ;
						}
						else
						{
							//Finger 0 event info (touch / event type / hover / palm / event id[0~3])
							Response_Message[0] =  0xA1;									
										
							//Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
							Response_Message[1] = (read_buf[7] << 4) | (read_buf[5] & 0x0F);
						
							//Finger 0 x coordinate (bit 7 ~ bit 0)
							Response_Message[2] = read_buf[4] ;
						
							//Finger 0 y coordinate (bit 7 ~ bit 0)
							Response_Message[3] = read_buf[6] ;

							//Finger 0 z (strength)
							Response_Message[4] = read_buf[8] ;
							
							flag = 0;
							}
						}
						else
						{
							//Finger 0 event info (touch / event type / hover / palm / event id[0~3])
							Response_Message[0] = (read_buf[1]<<4 | 0x7F) & 0xA1;									
										
							//Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
							Response_Message[1] = (read_buf[7] << 4) | (read_buf[5] & 0x0F);
						
							//Finger 0 x coordinate (bit 7 ~ bit 0)
							Response_Message[2] = read_buf[4] ;
						
							//Finger 0 y coordinate (bit 7 ~ bit 0)
							Response_Message[3] = read_buf[6] ;

							//Finger 0 z (strength)
							Response_Message[4] = read_buf[8] ;
						}
//						//Finger 1 event info (touch / event type / hover / palm / event id[0~3])		
//						if(read_buf[2] == 0x02)		
//						{							
//							Response_Message[8] = 0xA2;
//						}
//						else
//						{
//							Response_Message[8] = 0x22;						
//						}
//						//Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
//						Response_Message[9] = (read_buf[13] << 4) | (read_buf[11] & 0x0F);					

//						//Finger 1 x coordinate (bit 7 ~ bit 0)
//						Response_Message[10] = read_buf[10] ;
//					
//						//Finger 1 y coordinate (bit 7 ~ bit 0)
//						Response_Message[11] = read_buf[12] ;

//						//Finger 1 z (strength)
//						Response_Message[12] = read_buf[14] ;
					break;
					default: break;
	}

}
/**
  * @brief  Function called from I2C IRQ Handler when TXIS flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Sending_Callback(void)
{
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */
	LL_I2C_TransmitData8(I2C1, Response_Message[Tx_Idx_IIC1++]);
}
/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED3 is On if data are correct.
  * @param  None
  * @retval None
  */
void Slave_Complete_Callback(void)
{
  /* Check if data request to turn on the LED3 */
//  if (Buffercmp8((uint8_t *)aReceiveBuffer, (uint8_t *)aLedOn, (ubReceiveIndex - 1)) == 0)
//  {
//    /* Turn LED3 On */
//    /* Expected bytes have been received */
//    /* Slave Rx sequence completed successfully*/
////    LED_On();
//  }
//  else
//  {
//    /* Call Error function */
//    Error_Callback();
//  }
}

/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  /* Disable I2C1_IRQn */
  NVIC_DisableIRQ(I2C1_IRQn);

  /* Unexpected event : Set LED3 to Blinking mode to indicate error occurs */
//  LED_Blinking(LED_BLINK_ERROR);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */