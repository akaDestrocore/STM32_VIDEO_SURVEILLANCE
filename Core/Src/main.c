/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stm32f407xx.h>
#include "main.h"
#include "fatfs.h"
#include "libjpeg.h"
#include "usb_host.h"
#include <core_cm4.h>
#include <usbh_platform.h>

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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

/* USER CODE BEGIN PV */
FATFS fs;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_FATFS_Init();
  MX_LIBJPEG_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  MX_DriverVbusFS(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInit_t RCC_OscInitStruct = {0};
  RCC_ClkInit_t RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  RCC->APB1ENR.bit.pwren = SET;
  PWR->CR.bit.vos = 1;

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.State = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.Source = 1;
  RCC_OscInitStruct.PLL.M = 4;
  RCC_OscInitStruct.PLL.N= 168;
  RCC_OscInitStruct.PLL.P= 0;
  RCC_OscInitStruct.PLL.Q= 7;
  RCC_OscConfig(&RCC_OscInitStruct);


  //LSI config
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscConfig(&RCC_OscInitStruct);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClockConfig(&RCC_ClkInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClockConfig(&RCC_ClkInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK1;
  RCC_ClockConfig(&RCC_ClkInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK2;
  RCC_ClockConfig(&RCC_ClkInitStruct);

  RCC_MCOConfig(RCC_MCO2, 1, RCC_MCODIV_4);

}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_Handle_t GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  RCC->AHB1ENR.bit.gpioeen = SET;
  RCC->AHB1ENR.bit.gpiohen = SET;
  RCC->AHB1ENR.bit.gpiocen = SET;
  RCC->AHB1ENR.bit.gpioaen = SET;
  RCC->AHB1ENR.bit.gpioden = SET;
  RCC->AHB1ENR.bit.gpioben = SET;

  /*Configure GPIO pin Output Level */
  GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
  //TODO:
  //  GPIO_WritePin(GPIOC, CAMERA_PWDN_Pin, RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.pGPIOx = GPIOC;
  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_0;
  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_OUTPUT;
  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
  GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
  GPIO_Init(&GPIO_InitStruct);

  //TODO:
//  GPIO_InitStruct.GPIO_Config.PinNumber = CAMERA_PWDN_Pin;
//  GPIO_Init(&GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.pGPIOx = USER_Btn_GPIO_Port;
  GPIO_InitStruct.GPIO_Config.PinNumber = USER_Btn_Pin;
  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_IT_RT;
  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_DOWN;
  GPIO_Init(&GPIO_InitStruct);


  /*Configure GPIO pin : PC9 */
  /*Configure GPIO pin : DCMI_XCLX_Pin */
  GPIO_InitStruct.pGPIOx = DCMI_XCLX_GPIO_Port;
  GPIO_InitStruct.GPIO_Config.PinNumber = DCMI_XCLX_Pin;
  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
  GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 0;
  GPIO_Init(&GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
