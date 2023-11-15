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
#include "Wire.h"
#include "Arduino.h"
#include "ZT7548.h"

//#define Soft_i2C
#define HW_i2C
//#define ENABLE_LOGGING


TwoWire ZT7548(SCL_Pin, SDA_Pin, SOFT_FAST);

Melfas_coord Melfas[finger_num];

Zinitix_coord Zinitix[finger_num];

/**
  * @brief Variables related to SlaveReceive process
  */

uint8_t  Buffer_Rx_IIC1[16];
uint8_t  Rx_Idx_IIC1=0;
uint8_t  Tx_Idx_IIC1=0;
uint8_t Response_Message[16];

uint8_t read_buf[16];
uint8_t cmd_buf[10];
uint8_t flag = 1;
uint16_t Chip_ID;
uint16_t CHECKSUM;

uint8_t in_time = 0, out_time = 0;
uint8_t in_flag = 0, out_flag = 0;

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
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE1);

  /**/
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTB, LL_EXTI_CONFIG_LINE4);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI4_15_IRQn, 1);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void ZT7548_INT()
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
	
	 Melfas[1].x  = read_buf[10];
	 Melfas[1].y  = read_buf[12];
	 Melfas[1].xy = read_buf[13] << 4 | (read_buf[11] & 0x0F);
	
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
//        delay(1);

}

void ZT7548_init()
{
  digitalWrite(RESET_Pin, HIGH);
	delay(2);
//	digitalWrite(RESET_Pin, LOW);
//	delay(100);
//	digitalWrite(RESET_Pin, HIGH);
//	delay(10);
		
	#ifdef HW_i2C
//	  MX_I2C2_Init();
	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CHECKSUM_RESULT, read_buf, 2);
//	CHECKSUM = read_buf[1] << 8 | read_buf[0];
//  Serial2.printf("ZT7538_CHECKSUM_RESULT = 0x%X\n", CHECKSUM);
//	if(CHECKSUM != 0x55AA)
//	{
//	    Serial2.println("Do firmware upgrade");
//	}
	cmd_buf[0] = 0x01;
	cmd_buf[1] = 0x00;
	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC000, cmd_buf, 2);

	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0xCC00, read_buf, 2);
	Chip_ID = read_buf[1] << 8 | read_buf[0];
  Serial2.printf("chip code = 0x%X\n", Chip_ID);

	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC004, NULL, NULL);
	
	cmd_buf[0] = 0x01;
	cmd_buf[1] = 0x00;
	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC002, cmd_buf, 2);
	delay(2);
	
	cmd_buf[0] = 0x01;
	cmd_buf[1] = 0x00;
	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0xC001, cmd_buf, 2);
	delay(50);	
	
//	cmd_buf[0] = 0x0A;
//	cmd_buf[1] = 0x00;
//	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0x000A, cmd_buf, 2);
	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0011, read_buf, 1);
//  Serial2.printf("ic_name = 0x%X\n", read_buf[0]);
//	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x001C, read_buf, 1);
//  Serial2.printf("ic_vendor_id = 0x%X\n", read_buf[0]);
//	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0014, read_buf, 1);
//  Serial2.printf("hw_id = 0x%X\n", read_buf[0]);
//	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x001E, read_buf, 1);
//  Serial2.printf("tsm_module_id = 0x%X\n", read_buf[0]);
//	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0012, read_buf, 1);
//  Serial2.printf("major_fw_version = 0x%X\n", read_buf[0]);
//	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0121, read_buf, 1);
//  Serial2.printf("minor_fw_version = 0x%X\n", read_buf[0]);
//	
//	I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0013, read_buf, 1);
//  Serial2.printf("release_version = 0x%X\n", read_buf[0]);
	
//	cmd_buf[0] = 0x00;
//	cmd_buf[1] = 0x00;
//	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_TOUCH_MODE, cmd_buf, 2);
//	delay(1);		
//	I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CLEAR_INT_STATUS_CMD, NULL, NULL);

  #endif
  #ifdef Soft_i2C
	  ZT7548.begin();
	
		cmd_buf[0] = 0x00;
		cmd_buf[1] = 0xC0;
	  cmd_buf[2] = 0x01;
		cmd_buf[3] = 0x00;
		ZT7548.beginTransmission(ZT7548_SLAVE_ADDR); 		      		
		ZT7548.write(cmd_buf, 4);        		
		ZT7548.endTransmission(); 
		delay(10);

		cmd_buf[0] = 0x02;
		cmd_buf[1] = 0xC0;
		cmd_buf[2] = 0x01;
		cmd_buf[3] = 0x00;
		ZT7548.beginTransmission(ZT7548_SLAVE_ADDR); 		     		
		ZT7548.write(cmd_buf, 4);        		
		ZT7548.endTransmission(); 
		delay(10);
		

		ZT7548.beginTransmission(ZT7548_SLAVE_ADDR); 		
		ZT7548.write(0x04);        		
		ZT7548.write(0xC0);        		
		ZT7548.endTransmission();
		delay(10);
		
		cmd_buf[0] = 0x01;
		cmd_buf[1] = 0xC0;
		cmd_buf[2] = 0x01;
		cmd_buf[3] = 0x00;
		ZT7548.beginTransmission(ZT7548_SLAVE_ADDR); 		     		
		ZT7548.write(cmd_buf, 4);        		
		ZT7548.endTransmission(); 
		delay(10);
		#endif
	Serial2.println("ZT7548 initial succeed");
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
//  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  Delay_Init();
  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */
//	pinMode(INT_Output_Pin, OUTPUT);
//	digitalWrite(INT_Output_Pin, HIGH);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_5);
	
  /* Initialize all configured peripherals */
  MX_I2C1_Init();
  MX_I2C2_Init();
	
	Serial2.begin(115200);
	Serial2.println("Serial printing...");
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	pinMode(RESET_Pin, OUTPUT);
//    pinMode(INTn_ZT7548, INPUT_PULLUP);
//    attachInterrupt(INTn_ZT7548, LED_Toogle, FALLING);
  ZT7548_init();
  MX_GPIO_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      LL_IWDG_ReloadCounter(IWDG);
  }

  /* USER CODE END 3 */
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
					if(bitRead(read_buf[15],0) == 1)		// if detet 2 finger
					{
						Response_Message[0] = 0x10;				
					}
					else
          {
					  Response_Message[0] = 0x08;				
          }
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
					for(int i = 0; i < 16; i++)
					{
						
							Response_Message[i] = 0x00;
					}


					//Finger 0 event info (touch / event type / hover / palm / event id[0~3])
					if(bitRead(read_buf[9],0) == 1)
					{
					 Response_Message[0] = 0xA1;			
					}		
					if(bitRead(read_buf[9],0) == 0)
					{
					 Response_Message[0] = 0x21;			
					}						
						if(bitRead(read_buf[9],7) == 1)	
					{
						Response_Message[0] = 0xB1;						
					}							
					//Finger 0 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
					Response_Message[1] = Melfas[0].xy;
				
					//Finger 0 x coordinate (bit 7 ~ bit 0)
					Response_Message[2] = Melfas[0].x;
				
					//Finger 0 y coordinate (bit 7 ~ bit 0)
					Response_Message[3] = Melfas[0].y;

					//Finger 0 z (strength)
					Response_Message[5] = read_buf[8];
					
					//Input Gesture Direction
					Response_Message[7] = read_buf[2];
					if(bitRead(read_buf[2],0) | bitRead(read_buf[2],1) | bitRead(read_buf[2],2) == 1)
					{
										Response_Message[6] = 0x01;
					}
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
					
					//Finger 1 event info (touch / event type / hover / palm / event id[0~3])		
					if(bitRead(read_buf[15],0) == 1)		
					{							
						Response_Message[8] = 0xA2;
					}
					else
					{
						Response_Message[8] = 0x22;						
					}				
					if(bitRead(read_buf[15],7) == 1)	
					{
						Response_Message[8] = 0xB2;						
					}

					//Finger 1 xy coordinate (high)  y coordinate (bit 11 ~ bit 8) x coordinate (bit 11 ~ bit 8)
					Response_Message[9] = Melfas[1].xy;					

					//Finger 1 x coordinate (bit 7 ~ bit 0)
					Response_Message[10] = Melfas[1].x;
				
					//Finger 1 y coordinate (bit 7 ~ bit 0)
					Response_Message[11] = Melfas[1].y;

					//Finger 1 z (strength)
					Response_Message[13] = read_buf[14];
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
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }			
	Serial2.println("ZT7548 Communication ERROR !!!!!");		
	Serial2.println("Please restart the system !!!!!");		
	NVIC_SystemReset();
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
