/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
#include "main.h"
#include <stm32f407xx_gpio.h>
#include <stdio.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dcmi;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  RCC->APB2ENR.bit.syscfgen = SET;
  RCC->APB1ENR.bit.pwren = SET;

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief DCMI MSP Initialization
* This function configures the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
  GPIO_Handle_t GPIO_InitStruct = {0};
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* Peripheral clock enable */
	  RCC->AHB2ENR.bit.dcmien = SET;

	  RCC->AHB1ENR.bit.gpioeen = SET;
	  RCC->AHB1ENR.bit.gpioaen = SET;
	  RCC->AHB1ENR.bit.gpiocen = SET;
	  RCC->AHB1ENR.bit.gpioben = SET;

    /**DCMI GPIO Configuration
    PE4     ------> DCMI_D4
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCLK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PC8     ------> DCMI_D2
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE1     ------> DCMI_D3
    */
	  GPIO_InitStruct.pGPIOx = GPIOE;
	  GPIO_InitStruct.GPIO_Config.PinNumber = 4;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = 0;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 5;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 6;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 1;
	  GPIO_Init(&GPIO_InitStruct);


	  GPIO_InitStruct.pGPIOx = GPIOA;
	  GPIO_InitStruct.GPIO_Config.PinNumber = 4;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = 0;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 6;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.pGPIOx = GPIOC;
	  GPIO_InitStruct.GPIO_Config.PinNumber = 6;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = 0;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 7;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 8;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.pGPIOx = GPIOB;
	  GPIO_InitStruct.GPIO_Config.PinNumber = 6;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = 0;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = 7;
	  GPIO_Init(&GPIO_InitStruct);

	  printf("Error test in msp\n");

    /* DCMI DMA Init */
    /* DCMI Init */
    hdma_dcmi.Instance = DMA2_Stream1;
    hdma_dcmi.Init.Channel = DMA_CHANNEL_1;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_CIRCULAR;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_dcmi.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    {
      Error_Handler();
    }

    printf("after DMA init in msp\n");

    __HAL_LINKDMA(hdcmi,DMA_Handle,hdma_dcmi);

    /* DCMI interrupt Init */
    HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
  /* USER CODE BEGIN DCMI_MspInit 1 */

  /* USER CODE END DCMI_MspInit 1 */
  }

}

/**
* @brief DCMI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_CLK_DISABLE();

    /**DCMI GPIO Configuration
    PE4     ------> DCMI_D4
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCLK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PC8     ------> DCMI_D2
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE1     ------> DCMI_D3
    */
//    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_1);
//
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_6);
//
//    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8);
//
//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* DCMI DMA DeInit */
    HAL_DMA_DeInit(hdcmi->DMA_Handle);

    /* DCMI interrupt DeInit */
    HAL_NVIC_DisableIRQ(DCMI_IRQn);
  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
