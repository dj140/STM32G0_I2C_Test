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
#include "Arduino.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#ifdef __cplusplus
 extern "C" {
#endif
   
void Address_Matching_Callback(void);
void Slave_Sending_Callback(void);
void Slave_Reception_Callback(void);
void Slave_Complete_Callback(void);
void UserButton_Callback(void);
void Error_Callback(void);
   
uint8_t  Buffer_Rx_IIC1[16];
uint8_t Response_Message[16];
uint8_t  Rx_Idx_IIC1=0;
uint8_t  Tx_Idx_IIC1=0;
uint8_t finger_two_flag = 0,finger_one_flag = 0;
uint8_t finger_one_up_flag = 0;
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
    // Set report package size
    case 0x0F:
      //Default package size 0x10
      Response_Message[0] = 0x08;
    
      //if both finger move report package size 0x10
      if(bitRead(read_buf[9],2) == 1 & bitRead(read_buf[15],2) == 1)
      {
        Response_Message[0] = 0x10;
      }
//      //if only have one finger move report package size 0x08
//      if(bitRead(read_buf[9],2) == 1 & bitRead(read_buf[15],2) == 0)
//      {
//        Response_Message[0] = 0x08;
//      }
//      if(bitRead(read_buf[9],2) == 0 & bitRead(read_buf[15],2) == 1)
//      {
//        Response_Message[0] = 0x08;
//      }
//      //if only have one finger exit report package size 0x08
//      if(bitRead(read_buf[9],0) == 1 | bitRead(read_buf[15],0) == 1)
//      {
//        Response_Message[0] = 0x08;
//      }
      
      if(finger_two_flag == 1 & bitRead(read_buf[15],0) == 0 )
      {
        Response_Message[0] = 0x10;
      }
    break;
      
    //For MST_UI tool to work
    case 0xAE:  
      Response_Message[0] = 0x26;
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
    
    // Get Point status
    case 0x10:
      // clear all Response data
      for(int i = 0; i < 16; i++)
      {
        Response_Message[i] = 0x00;
      }
      
      //only one finger down, one finger up condition
      if(finger_one_flag ==1 &bitRead(read_buf[9],0) ==0)
      {
        Response_Message[0] = 0x21;
        //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[1] = Melfas[0].xy;

        //Finger 0 x coordinate (bit 7 ~ bit 0)
        Response_Message[2] = Melfas[0].x;

        //Finger 0 y coordinate (bit 7 ~ bit 0)
        Response_Message[3] = Melfas[0].y;

        //Finger 0 z (strength)
        Response_Message[5] = Melfas[0].strength;
      }
      
      // only finger one press down
      if(bitRead(read_buf[9],0) == 1 & bitRead(read_buf[15],0) == 0)
      {
        finger_one_flag =1;
        //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0xA1;
        //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[1] = Melfas[0].xy;

        //Finger 0 x coordinate (bit 7 ~ bit 0)
        Response_Message[2] = Melfas[0].x;

        //Finger 0 y coordinate (bit 7 ~ bit 0)
        Response_Message[3] = Melfas[0].y;

        //Finger 0 z (strength)
        Response_Message[5] = Melfas[0].strength;
        
        //plam mapping
        if(bitRead(read_buf[9],7) == 1)	
        {
          Response_Message[0] = 0xB1;
        }
      } else finger_one_flag =0;
      
      // only finger two press down
      if(bitRead(read_buf[9],0) == 0 & bitRead(read_buf[15],0) == 1)
      {
        //Finger 1 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0xA2;
        //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[1] = Melfas[1].xy;

        //Finger 1 x coordinate (bit 7 ~ bit 0)
        Response_Message[2] = Melfas[1].x;

        //Finger 1 y coordinate (bit 7 ~ bit 0)
        Response_Message[3] = Melfas[1].y;

        //Finger 1 z (strength)
        Response_Message[5] = Melfas[1].strength;
        
        //plam mapping
        if(bitRead(read_buf[15],7) == 1)
        {
          Response_Message[0] = 0xB2;
        }
      }
      
      //two finger down  -->  finger one up
      if(finger_two_flag == 1 & bitRead(read_buf[9],0) == 0 )
      {
        //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0x21;
        
        //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[1] = Melfas[0].xy;

        //Finger 0 x coordinate (bit 7 ~ bit 0)
        Response_Message[2] = Melfas[0].x;

        //Finger 0 y coordinate (bit 7 ~ bit 0)
        Response_Message[3] = Melfas[0].y;

        //Finger 0 z (strength)
        Response_Message[5] = Melfas[0].strength;
        
        finger_one_up_flag =1;
      }
      
      //two finger down  -->  finger one up  -->  finger two up
      if(finger_one_up_flag == 1 & bitRead(read_buf[15],0) == 0 )
      {
        //Finger 1 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0x22;
        
        //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[1] = Melfas[1].xy;

        //Finger 1 x coordinate (bit 7 ~ bit 0)
        Response_Message[2] = Melfas[1].x;

        //Finger 1 y coordinate (bit 7 ~ bit 0)
        Response_Message[3] = Melfas[1].y;

        //Finger 1 z (strength)
        Response_Message[5] = Melfas[1].strength;
        finger_one_up_flag =0;
      }
      
      //two finger down  -->  finger one&two up
      if(finger_two_flag == 1 & bitRead(read_buf[9],0) == 0 & bitRead(read_buf[15],0) == 0 )
      {
        
        //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0x21;
        
        //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[1] = Melfas[0].xy;

        //Finger 0 x coordinate (bit 7 ~ bit 0)
        Response_Message[2] = Melfas[0].x;

        //Finger 0 y coordinate (bit 7 ~ bit 0)
        Response_Message[3] = Melfas[0].y;

        //Finger 0 z (strength)
        Response_Message[5] = Melfas[0].strength;
        
        //Finger 1 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[8] = 0x22;
        
        //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[9] = Melfas[1].xy;

        //Finger 1 x coordinate (bit 7 ~ bit 0)
        Response_Message[10] = Melfas[1].x;

        //Finger 1 y coordinate (bit 7 ~ bit 0)
        Response_Message[11] = Melfas[1].y;

        //Finger 1 z (strength)
        Response_Message[13] = Melfas[1].strength;
      }
      
      if( bitRead(read_buf[15],0) == 0 )
      {
        Response_Message[8] = 0x22;
        //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
        Response_Message[9] = Melfas[1].xy;

        //Finger 1 x coordinate (bit 7 ~ bit 0)
        Response_Message[10] = Melfas[1].x;

        //Finger 1 y coordinate (bit 7 ~ bit 0)
        Response_Message[11] = Melfas[1].y;

        //Finger 1 z (strength)
        Response_Message[13] = Melfas[1].strength;
      }
      // both finger press down
      if(bitRead(read_buf[9],0) == 1 & bitRead(read_buf[15],0) == 1)
      {
        finger_two_flag = 1;
        // if both finger move report two finger coordinate
//        if(bitRead(read_buf[9],2) == 1 & bitRead(read_buf[15],2) == 1)
//        {
          //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
          Response_Message[0] = 0xA1;
          //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
          Response_Message[1] = Melfas[0].xy;

          //Finger 0 x coordinate (bit 7 ~ bit 0)
          Response_Message[2] = Melfas[0].x;

          //Finger 0 y coordinate (bit 7 ~ bit 0)
          Response_Message[3] = Melfas[0].y;

          //Finger 0 z (strength)
          Response_Message[5] = Melfas[0].strength;
          //Finger 1 event info (touch / event type / hover / palm / event id[0~3])

          Response_Message[8] = 0xA2;
          //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
          Response_Message[9] = Melfas[1].xy;

          //Finger 1 x coordinate (bit 7 ~ bit 0)
          Response_Message[10] = Melfas[1].x;

          //Finger 1 y coordinate (bit 7 ~ bit 0)
          Response_Message[11] = Melfas[1].y;

          //Finger 1 z (strength)
          Response_Message[13] = Melfas[1].strength;
          if(bitRead(read_buf[9],7) == 1)	
          {
            Response_Message[0] = 0xB1;
          }
          if(bitRead(read_buf[15],7) == 1)
          {
            Response_Message[8] = 0xB2;
          }
//        }
        // if only finger two move report finger two coordinate
        if(bitRead(read_buf[9],2) == 0 & bitRead(read_buf[15],2) == 1)
        {
          //Finger 1 event info (touch / event type / hover / palm / event id[0~3])
          Response_Message[0] = 0xA2;
          //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
          Response_Message[1] = Melfas[1].xy;

          //Finger 1 x coordinate (bit 7 ~ bit 0)
          Response_Message[2] = Melfas[1].x;

          //Finger 1 y coordinate (bit 7 ~ bit 0)
          Response_Message[3] = Melfas[1].y;

          //Finger 1 z (strength)
          Response_Message[5] = Melfas[1].strength;
          if(bitRead(read_buf[15],7) == 1)
          {
            Response_Message[0] = 0xB2;
          }
        }
//        // if only finger one move report finger one coordinate
//        if(bitRead(read_buf[9],2) == 1 & bitRead(read_buf[15],2) == 0)
//        {
//          //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
//          Response_Message[0] = 0xA1;
//          //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
//          Response_Message[1] = Melfas[0].xy;

//          //Finger 0 x coordinate (bit 7 ~ bit 0)
//          Response_Message[2] = Melfas[0].x;

//          //Finger 0 y coordinate (bit 7 ~ bit 0)
//          Response_Message[3] = Melfas[0].y;

//          //Finger 0 z (strength)
//          Response_Message[5] = Melfas[0].strength;
//          if(bitRead(read_buf[9],7) == 1)
//          {
//            Response_Message[0] = 0xB1;
//          }
//        }
      } else finger_two_flag =0;
      
      //Input Gesture Direction
      if(read_buf[2] == 0x05)
      {
        Response_Message[7] = 0x07;//up
      }
      if(read_buf[2] == 0x01)
      {
        Response_Message[7] = 0x03;//down
      }
      if(read_buf[2] == 0x03)
      {
        Response_Message[7] = 0x05;//left
      }
      if(read_buf[2] == 0x07)
      {
        Response_Message[7] = 0x01;//right
      }
      if(bitRead(read_buf[2],0) | bitRead(read_buf[2],1) | bitRead(read_buf[2],2) == 1)
      {
        Response_Message[6] = 0x01;//flick
      }
      
      //Pinch in/out
      if(out_flag == 1)
      {
        Response_Message[6] = 0x03;
        Response_Message[14] = 0x03;
      }
      if(in_flag == 1)
      {
        Response_Message[6] = 0x04;
        Response_Message[14] = 0x04;
      }

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
  if(Buffer_Rx_IIC1[0] == 0x10)
  {
    digitalWrite(INT_Output_Pin, HIGH);
  }
}

/**
* @brief  Function called in case of error detected in I2C IT Handler
* @param  None
* @retval None
*/
void Error_Callback(void)
{
  /* Disable I2C1_IRQn */
  Serial2.println("I2C Salve Communication ERROR !!!!!");		
  Serial2.println("Please restart the system !!!!!");		
  NVIC_DisableIRQ(I2C1_IRQn);
  NVIC_SystemReset();
}
#ifdef __cplusplus
}
#endif
/* USER CODE END 1 */
