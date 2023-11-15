/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"
#include "ZT7548.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	/* Timing register value is computed with the STM32CubeMX Tool,
  * Fast Mode @400kHz with I2CCLK = 64 MHz,
  * rise time = 100ns, fall time = 10ns
  * Timing Value = (uint32_t)0x00C0216C*/
  I2C_InitStruct.Timing = 0x00C0216C;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = SLAVE_OWN_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);

  /* USER CODE BEGIN I2C1_Init 2 */

  /* Enable I2C1 address match/error interrupts:
  *  - Enable Address Match Interrupt
  *  - Enable Not acknowledge received interrupt
  *  - Enable Error interrupts
  *  - Enable Stop interrupt
  */
  LL_I2C_EnableIT_ADDR(I2C1);
  LL_I2C_EnableIT_NACK(I2C1);
//  LL_I2C_EnableIT_ERR(I2C1);
  LL_I2C_EnableIT_STOP(I2C1);
	LL_I2C_EnableIT_RX(I2C1);
	LL_I2C_EnableIT_TX(I2C1);
  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**I2C2 GPIO Configuration
  PA11 [PA9]   ------> I2C2_SCL
  PA12 [PA10]   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00C0216C;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = ZT7548_SLAVE_ADDR;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C2);
  LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

uint8_t I2C_write_reg(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC, uint16_t addr_reg, uint8_t* buf, uint8_t len){
    SlaveAddr_IC = SlaveAddr_IC<<1;
    uint32_t counter = 0;
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET){
        counter++;
        if( counter == 25000 ) {
            Error_Handler();
            return ERROR;
        }
    }
    LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC,LL_I2C_ADDRSLAVE_7BIT, len + 2 ,LL_I2C_MODE_AUTOEND,LL_I2C_GENERATE_START_WRITE ); //LL_I2C_GENERATE_START_READ
    while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);
    LL_I2C_TransmitData8(I2Cx, addr_reg);
	  while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET)
		{
        counter++;
        if( counter == 25000 ) {
            Error_Handler();
            return ERROR;
        }
    }
	  LL_I2C_TransmitData8(I2Cx, addr_reg>>8);
    counter=0;
    while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET){
        counter++;
        if( counter == 25000 ){//~ 150ms
            LL_I2C_ClearFlag_TXE(I2Cx);
            Error_Handler();
            return ERROR;
        }
    }
		for(uint8_t i = 0; i < len; i++)
		{
			LL_I2C_TransmitData8(I2Cx, buf[i]);
			while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);
		}
    while(LL_I2C_IsActiveFlag_STOP(I2Cx)==RESET);
    LL_I2C_ClearFlag_STOP(I2Cx);
    return SUCCESS;
}

uint16_t I2C_read_reg(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC, uint16_t addr_reg, uint8_t *pui8RxBuffer, uint8_t len){
//    uint8_t i=0;
    SlaveAddr_IC = SlaveAddr_IC<<1;
    uint32_t counter = 0;
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx)==SET){
        counter++;
        if( counter == 25000 ){//aproximate 150ms
            Error_Handler();
            return ERROR;
        }
    }
    LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC,LL_I2C_ADDRSLAVE_7BIT, 2,LL_I2C_MODE_SOFTEND,LL_I2C_GENERATE_START_WRITE ); //LL_I2C_GENERATE_START_READ
    while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);
    LL_I2C_TransmitData8(I2Cx, addr_reg);
		while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);
    LL_I2C_TransmitData8(I2Cx, addr_reg>>8);
		
    counter=0;
    while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET){
        counter++;
        if( counter == 25000 ){//aproximate 150ms
            LL_I2C_ClearFlag_TXE(I2Cx);
            Error_Handler();
            return ERROR;
        }
    }
    while(LL_I2C_IsActiveFlag_TC(I2Cx)==RESET);
    LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ); //LL_I2C_MODE_SOFTEND
    while(!LL_I2C_IsActiveFlag_STOP(I2Cx)){
        if(LL_I2C_IsActiveFlag_RXNE(I2Cx)){
            *pui8RxBuffer = LL_I2C_ReceiveData8(I2Cx);
            pui8RxBuffer++;
        }
    }
    LL_I2C_ClearFlag_STOP(I2Cx);
    return SUCCESS;
}


/* USER CODE END 1 */
