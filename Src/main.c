/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.h"
#include "usbd_cdc_if.h"
#include <string.h>

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
void print_usb(uint8_t *buffer, uint8_t len);
void float_to_string(uint8_t *buffer, float val);
void test_success(void);
void test_failed(void);
void print_float_usb(float number);
void println_float_usb(float number);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  struct MPU9250_Handle_s MPU9250_Handle;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  int i = 0;
  while (MPU9250_begin(&MPU9250_Handle) < 0) {
	  i++;
	  if (i > 2) {
		  test_failed();
	  }
  }

  i = 0;
  while (MPU9250_readSensor(&MPU9250_Handle) < 0) {
	  i++;
	  if (i > 2) {
		  test_failed();
	  }
  }

  test_success();

  uint8_t comma = ',';

  MPU9250_setAccelRange(ACCEL_FS_SEL_16G, &MPU9250_Handle);
  MPU9250_setGyroRange(GYRO_FS_SEL_2000DPS, &MPU9250_Handle);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MPU9250_readSensor(&MPU9250_Handle)

	  print_float_usb(MPU9250_getAccelX_mss(&MPU9250_Handle);
	  print_usb(&comma, 1);
	  print_float_usb(MPU9250_getAccelY_mss(&MPU9250_Handle));
	  print_usb(&comma, 1);
	  print_float_usb(MPU9250_getAccelZ_mss(&MPU9250_Handle));
	  print_usb(&comma, 1);

	  print_float_usb(MPU9250_getGyroX_rads(&MPU9250_Handle));
	  print_usb(&comma, 1);
	  print_float_usb(MPU9250_getGyroY_rads(&MPU9250_Handle));
	  print_usb(&comma, 1);
	  print_float_usb(MPU9250_getGyroZ_rads(&MPU9250_Handle));
	  print_usb(&comma, 1);

	  print_float_usb(MPU9250_getMagX_uT(&MPU9250_Handle));
	  print_usb(&comma, 1);
	  print_float_usb(MPU9250_getMagY_uT(&MPU9250_Handle));
	  print_usb(&comma, 1);
	  print_float_usb(MPU9250_getMagZ_uT(&MPU9250_Handle));
	  print_usb(&comma, 1);

	  println_float_usb(MPU9250_getTemperature_C(&MPU9250_Handle));
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void print_float_usb(float number) {
	uint8_t string[11];

	float_to_string(string, number);

	print_usb(string, 11);
}

void println_float_usb(float number) {
	uint8_t string[12];

	float_to_string(string, number);

	string[11] = '\n';

	print_usb(string, 12);
}

void print_usb(uint8_t *buffer, uint8_t len) {

	while (CDC_Transmit_FS(buffer, len) != USBD_OK);

	return;
}

// string should be 11 bytes in length
void float_to_string(uint8_t *buffer, float val){

    if (val < 0) {
        val *= -1;
        buffer[0] = '-';
    } else {
        buffer[0] = ' ';
    }

    buffer[1]  = (int)val / 10000 + '0';
    buffer[2]  = (int)val % 10000 / 1000 + '0';
    buffer[3]  = (int)val % 1000 / 100 + '0';
	buffer[4]  = (int)val % 100 / 10 + '0';
	buffer[5]  = (int)val % 10 + '0';
	buffer[6]  = '.';
	buffer[7]  = (int)((val - (int)val) * 10) + '0';
	buffer[8]  = (int)((val - (int)val) * 100) % 10 + '0';
	buffer[9]  = (int)((val - (int)val) * 1000) % 10 + '0';
	buffer[10] = (int)((val - (int)val) * 10000) % 10 + '0';

	return;
}

void test_success(void) {
	// Turn on LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void test_failed(void) {
	// Turn off LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
