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

#ifdef Soft_i2C
TwoWire ZT7548(SCL_Pin, SDA_Pin, SOFT_FAST);
#endif

/**
* @brief Variables related to SlaveReceive process
*/

uint8_t read_buf[16];
uint8_t cmd_buf[10];

uint16_t Chip_ID;
uint16_t CHECKSUM;



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



void ZT7548_init()
{
  digitalWrite(RESET_Pin, HIGH);
  delay(2);
  //digitalWrite(RESET_Pin, LOW);
  //delay(100);
  //digitalWrite(RESET_Pin, HIGH);
  //delay(10);

#ifdef HW_i2C
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CHECKSUM_RESULT, read_buf, 2);
  //CHECKSUM = read_buf[1] << 8 | read_buf[0];
  //Serial2.printf("ZT7538_CHECKSUM_RESULT = 0x%X\n", CHECKSUM);
  //if(CHECKSUM != 0x55AA)
  //{
  //    Serial2.println("Do firmware upgrade");
  //}
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

  //cmd_buf[0] = 0x0A;
  //cmd_buf[1] = 0x00;
  //I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, 0x000A, cmd_buf, 2);

  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0011, read_buf, 1);
  //Serial2.printf("ic_name = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x001C, read_buf, 1);
  //Serial2.printf("ic_vendor_id = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0014, read_buf, 1);
  //Serial2.printf("hw_id = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x001E, read_buf, 1);
  //Serial2.printf("tsm_module_id = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0012, read_buf, 1);
  //Serial2.printf("major_fw_version = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0121, read_buf, 1);
  //Serial2.printf("minor_fw_version = 0x%X\n", read_buf[0]);
  //
  //I2C_read_reg(I2C2, ZT7548_SLAVE_ADDR, 0x0013, read_buf, 1);
  //Serial2.printf("release_version = 0x%X\n", read_buf[0]);

  //cmd_buf[0] = 0x00;
  //cmd_buf[1] = 0x00;
  //I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_TOUCH_MODE, cmd_buf, 2);
  //delay(1);
  //I2C_write_reg(I2C2, ZT7548_SLAVE_ADDR, ZT7538_CLEAR_INT_STATUS_CMD, NULL, NULL);

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
//  pinMode(INT_Output_Pin, OUTPUT);
//  digitalWrite(INT_Output_Pin, HIGH);
  //pull INTR pin high
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_5);

  /* Initialize all configured peripherals */
  //I2C slave init
  MX_I2C1_Init();
  //I2C master init
  MX_I2C2_Init();

  Serial2.begin(115200);
  Serial2.println("Serial printing...");
  //watchdog init
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  pinMode(RESET_Pin, OUTPUT);
  //pinMode(INTn_ZT7548, INPUT_PULLUP);
  //attachInterrupt(INTn_ZT7548, LED_Toogle, FALLING);
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
