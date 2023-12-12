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
#include "ZT7548.h"
#include "i2c.h"

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
uint16_t start_x = 0, start_y = 0, start_distance = 0, end_distance = 0;
bool  both_finger_down_flag = 0, both_finger_up_flag = 0, finger_one_up_flag = 0, finger_two_up_flag = 0;
bool in_flag = 0, out_flag = 0;

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
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void ZT7548_interrupt_handler()
{

#ifdef HW_i2C
  I2C_read_reg(I2C2,ZT7548_SLAVE_ADDR, ZT7538_POINT_STATUS_REG, read_buf, 16);
  delay_us(50);
  I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CLEAR_INT_STATUS_CMD, NULL, NULL);

  Zinitix[0].x = read_buf[5]<<8 | read_buf[4];
  Zinitix[0].y = read_buf[7]<<8 | read_buf[6];
  Zinitix[1].x = read_buf[11]<<8 | read_buf[10];
  Zinitix[1].y = read_buf[13]<<8 | read_buf[12];

  Melfas[0].x  = read_buf[4];
  Melfas[0].y  = read_buf[6];
  Melfas[0].xy = read_buf[7] << 4 |  (read_buf[5] & 0x0F);
  Melfas[0].strength = read_buf[8];
  
  Melfas[1].x  = read_buf[10];
  Melfas[1].y  = read_buf[12];
  Melfas[1].xy = read_buf[13] << 4 | (read_buf[11] & 0x0F);
  Melfas[1].strength = read_buf[14];
  //Pinch in/out algorithm
  if(bitRead(read_buf[15],0) == 1)
  {
    if((bitRead(read_buf[15],2) | bitRead(read_buf[9],2)) == 1)
    {
      start_x = abs(Zinitix[0].x - Zinitix[1].x);
      start_y = abs(Zinitix[0].y - Zinitix[1].y);
      start_distance = sqrt(float(sq(start_x) + sq(start_y)));

      if(end_distance > 0)
      {
        if(start_distance < end_distance)
        {
          in_flag = 1;
        }
        if(start_distance > end_distance)
        {
          out_flag = 1;
        }
      }
      end_distance = start_distance;
    }
  }
  else
  {
    end_distance = start_distance = 0;
    in_flag = out_flag = 0;
  }

#endif

#ifdef ENABLE_LOGGING
  if(out_flag == 1)
  {
    Serial2.println("Zoom out");
  }
  if(in_flag == 1)
  {
    Serial2.println("Zoom in");
  }
  if(read_buf[2] == 0x05)
  {
    Serial2.println("up");
  }
  if(read_buf[2] == 0x01)
  {
    Serial2.println("down");
  }
  if(read_buf[2] == 0x03)
  {
    Serial2.println("left");
  }
  if(read_buf[2] == 0x07)
  {
    Serial2.println("right");
  }
  Serial2.printf("x1 : %d  y1 : %d\n", Zinitix[0].x, Zinitix[0].y);

  if(bitRead(read_buf[15],0) == 1)
  {
    Serial2.printf("x2 : %d  y2 : %d\n", Zinitix[1].x, Zinitix[1].y);
  }
#endif

#ifdef Soft_i2C
  ZT7548.beginTransmission(ZT7548_SLAVE_ADDR);
  ZT7548.write(0x80);
  ZT7548.write(0x00);
  ZT7548.endTransmission(); 
  delay_us(50);

  ZT7548.requestFrom(ZT7548_SLAVE_ADDR, 40);    

  while (ZT7548.available()) 
  { 
    for(uint8_t i = 0; i < 40; i++)
    {
      read_buf[i] = ZT7548.read(); 
    }
  }

  ZT7548.beginTransmission(ZT7548_SLAVE_ADDR);
  ZT7548.write(0x03);
  ZT7548.write(0x00);
  ZT7548.endTransmission(); 

#endif

digitalWrite(INT_Output_Pin, LOW);
//delay(1);

}

void EXTI4_15_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_4) != RESET)
  {
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_4);
    ZT7548_interrupt_handler(); 
  }

}


/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */

void EXTI0_1_IRQHandler(void)
{

  if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_1);
    delay_us(20);
    if(digitalRead(CE_Pin) == LOW)
    {  
      ZT7548_reset();
      LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_4);
      LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_1);
      NVIC_DisableIRQ(I2C1_IRQn);
      NVIC_DisableIRQ(EXTI4_15_IRQn);
      digitalWrite(INT_Output_Pin, HIGH);
    }
  }
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_1);
    delay_us(20);
    if(digitalRead(CE_Pin) == HIGH)
    {
      NVIC_EnableIRQ(I2C1_IRQn);
      NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
  }

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

void Gesture_mapping(void)
{
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
}

void finger_one_coord(void)
{

  //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
  Response_Message[1] = Melfas[0].xy;

  //Finger 0 x coordinate (bit 7 ~ bit 0)
  Response_Message[2] = Melfas[0].x;

  //Finger 0 y coordinate (bit 7 ~ bit 0)
  Response_Message[3] = Melfas[0].y;

  //Finger 0 z (strength)
  Response_Message[5] = Melfas[0].strength;

  //plam mapping
  if(bitRead(read_buf[9],7) == 1 & bitRead(read_buf[9],0) == 1)
  {
    Response_Message[0] = 0xB1;
  }
}

void finger_two_coord(void)
{
  //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
  Response_Message[1] = Melfas[1].xy;

  //Finger 1 x coordinate (bit 7 ~ bit 0)
  Response_Message[2] = Melfas[1].x;

  //Finger 1 y coordinate (bit 7 ~ bit 0)
  Response_Message[3] = Melfas[1].y;

  //Finger 1 z (strength)
  Response_Message[5] = Melfas[1].strength;

  //plam mapping
  if(bitRead(read_buf[15],7) == 1 & bitRead(read_buf[15],0) == 1)
  {
    Response_Message[0] = 0xB2;
  }
}

void both_finger_coord(void)
{
  //Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
  Response_Message[1] = Melfas[0].xy;

  //Finger 0 x coordinate (bit 7 ~ bit 0)
  Response_Message[2] = Melfas[0].x;

  //Finger 0 y coordinate (bit 7 ~ bit 0)
  Response_Message[3] = Melfas[0].y;

  //Finger 0 z (strength)
  Response_Message[5] = Melfas[0].strength;
  //Finger 1 event info (touch / event type / hover / palm / event id[0~3])

  //Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
  Response_Message[9] = Melfas[1].xy;

  //Finger 1 x coordinate (bit 7 ~ bit 0)
  Response_Message[10] = Melfas[1].x;

  //Finger 1 y coordinate (bit 7 ~ bit 0)
  Response_Message[11] = Melfas[1].y;

  //Finger 1 z (strength)
  Response_Message[13] = Melfas[1].strength;
  

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
      //if both finger up at the same time
      if(both_finger_down_flag == 1 & bitRead(read_buf[9],0) == 0 & bitRead(read_buf[15],0) == 0)
      {
        Response_Message[0] = 0x10;
        both_finger_up_flag =1;
      }
      //if both finger two up but finger one still moving
      if(both_finger_down_flag == 1 & bitRead(read_buf[9],2) == 1 & bitRead(read_buf[15],0) == 0)
      {
        Response_Message[0] = 0x10;
        finger_two_up_flag =1;
      }
      //if both finger one up but finger two still moving
      if(both_finger_down_flag == 1 & bitRead(read_buf[9],0) == 0 & bitRead(read_buf[15],2) == 1)
      {
        Response_Message[0] = 0x10;
        finger_one_up_flag =1;
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
    case 0xF0:  
      Response_Message[0] = 0x04;
    break;
    case 0xF1:  
      Response_Message[0] = 0x45;
    break;
    case 0xF2:  
      Response_Message[0] = 0x45;
    break;
    case 0xF3:  
      Response_Message[0] = 0x07;
    break;
    case 0xF4:  
      Response_Message[0] = 0x07;
    break;
    case 0xF5:  
      Response_Message[0] = 0x07;
    break;
    
    // Get Point status
    case 0x10:
      // clear all Response data
      for(int i = 0; i < 16; i++)
      {
        Response_Message[i] = 0x00;
      }

      // only finger one press down
      if(bitRead(read_buf[9],0) == 1)
      {
        //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0xA1;
        finger_one_coord();
      }
      
      // only finger two press down
      if(bitRead(read_buf[9],0) == 0 & bitRead(read_buf[15],0) == 1)
      {
        //Finger 1 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0xA2;
        finger_two_coord();
      }

      // both finger press down
      if(bitRead(read_buf[9],0) == 1 & bitRead(read_buf[15],0) == 1)
      {
         both_finger_down_flag = 1;

        //Finger 0 event info (touch / event type / hover / palm / event id[0~3])
        Response_Message[0] = 0xA1;
        Response_Message[8] = 0xA2;
        both_finger_coord();
        // if only finger two move report finger two coordinate
        if(bitRead(read_buf[9],2) == 0 & bitRead(read_buf[15],2) == 1)
        {
          //Finger 1 event info (touch / event type / hover / palm / event id[0~3])
          Response_Message[0] = 0xA2;
          finger_two_coord();
          if(bitRead(read_buf[9],7) == 1 & bitRead(read_buf[9],0) == 1)
          {
            Response_Message[0] = 0xB1;
          }
          if(bitRead(read_buf[15],7) == 1 & bitRead(read_buf[15],0) == 1)
          {
            Response_Message[8] = 0xB2;
          }
        }
      } else both_finger_down_flag = 0;
      //report finger up status
      if(bitRead(read_buf[0],0) == 1 & bitRead(read_buf[0],3) == 1)
      {
        if(read_buf[9] == 0x08 | read_buf[9] == 0x88)
        {
          Response_Message[0] = 0x21;
          finger_one_coord();
        }
        if(read_buf[15] == 0x08 | read_buf[15] == 0x88)
        {
          Response_Message[0] = 0x22;
          finger_two_coord();
        }
        if(both_finger_up_flag == 1)
        {
          both_finger_up_flag = 0;
          Response_Message[0] = 0x21;
          Response_Message[8] = 0x22;
          both_finger_coord();
        }
        if(finger_one_up_flag == 1)
        {
          finger_one_up_flag = 0;
          Response_Message[0] = 0x21;
          Response_Message[8] = 0xA2;
          both_finger_coord();
        }
        if(finger_two_up_flag == 1)
        {
          finger_two_up_flag = 0;
          Response_Message[0] = 0xA1;
          Response_Message[8] = 0x22;
          both_finger_coord();
        }
      }
      
      Gesture_mapping();

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
#ifdef ENABLE_LOGGING
  Serial2.println("I2C Salve Communication ERROR !!!!!");
  Serial2.println("Please restart the system !!!!!");
#endif
  NVIC_DisableIRQ(I2C1_IRQn);
  NVIC_SystemReset();
}
#ifdef __cplusplus
}
#endif
/* USER CODE END 1 */
