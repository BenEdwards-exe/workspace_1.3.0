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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Sprites.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Screen_Start (unsigned char*)0x20020000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t spritemem[1500];
int xPos = 150;
int yPos = 100;

uint8_t i2cData[10];
HAL_StatusTypeDef res;
int ax;
int ay;
int az;

uint8_t txData[20] = "x: XXXXX, y: XXXXX\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BitBlit(unsigned char* ptr_screen, unsigned char* ptr_sprite, int x, int y, int sprite_w, int sprite_h);
void ClearSprite(unsigned char* ptr_screen, int x, int y, int sprite_w, int sprite_h);

/* USER CODE END 0 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LDRSprites(spritemem);
  BitBlit(Screen_Start, FindSprite(spritemem, INV2), xPos, yPos, INV2_W, INV2_H);

  i2cData[0] = 0x20;
  i2cData[1] = 0x47;
  res = HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2cData, 2, 10);

  i2cData[0] = 0x23;
  i2cData[1] = 0x30;
  res = HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2cData, 2, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(1);
	  i2cData[0] = 0xA8;
	  res = HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2cData, 1, 10);
	  res = HAL_I2C_Master_Receive(&hi2c1, 0x32, i2cData, 6, 10);

	  ax = *((int16_t*)i2cData);
	  ay = *((int16_t*)(i2cData+2));
	  az = *((int16_t*)(i2cData+4));

	  ax /= 100;
	  ay /= 100;

	  ClearSprite(Screen_Start, xPos, yPos, INV2_W, INV2_H);

	  xPos += ax;
	  yPos += ay;

	  // Clamp screen coordinates
	  if (xPos < 0) {
		  xPos = 0;
	  }
	  else if (xPos >= 320 - INV2_W) {
		  xPos = 320 - INV2_W;
	  }
	  if (yPos < 0) {
		  yPos = 0;
	  }
	  else if (yPos >= 200 - INV2_H) {
		  yPos = 200 - INV2_H;
	  }

	  BitBlit(Screen_Start, FindSprite(spritemem, INV2), xPos, yPos, INV2_W, INV2_H);

	  int xVel = abs(ax);
	  int yVel = abs(ay);

	  txData[3] = (xVel / 10000) % 10 + 48;
	  txData[4] = (xVel / 1000) % 10 + 48;
	  txData[5] = (xVel / 100) % 10 + 48;
	  txData[6] = (xVel / 10) % 10 + 48;
	  txData[7] = (xVel / 1) % 10 + 48;

	  txData[13] = (yVel / 10000) % 10 + 48;
	  txData[14] = (yVel / 1000) % 10 + 48;
	  txData[15] = (yVel / 100) % 10 + 48;
	  txData[16] = (yVel / 10) % 10 + 48;
	  txData[17] = (yVel / 1) % 10 + 48;

	  if (ax < 0) {
		  for (int i = 7; i > 3; i--) {
			  if(txData[i] == 48) {
				  txData[i] = 45;
				  break;
			  }
		  }
	  }
	  if (ay < 0) {
		  for (int i = 17; i > 13; i--) {
			  if(txData[i] == 48) {
				  txData[i] = 45;
				  break;
			  }
		  }
	  }

	  HAL_UART_Transmit(&huart1, txData, 20, 10);



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void BitBlit(unsigned char* ptr_screen, unsigned char* ptr_sprite, int x, int y, int sprite_w, int sprite_h)
{
	for (int r = 0; r < sprite_h; ++r) {
		for (int c = 0; c < sprite_w; ++c) {

			if (*(ptr_sprite + r*sprite_w + c) != 0) {
				*(ptr_screen + (y+r)*320 + x + c) = *(ptr_sprite + r*sprite_w + c);
			}

		}

	}
}
void ClearSprite(unsigned char* ptr_screen, int x, int y, int sprite_w, int sprite_h)
{
	for (int r = 0; r < sprite_h; ++r) {
		for (int c = 0; c < sprite_w; ++c) {
			*(ptr_screen + (y+r)*320 + x + c) = 0;
		}
	}
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
