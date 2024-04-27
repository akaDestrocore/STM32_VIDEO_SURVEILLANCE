/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32f407xx_gpio.h>
#include <stdio.h>

extern DMA_HandleTypeDef hdma_dcmi;

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  RCC->APB2ENR.bit.syscfgen = SET;
  RCC->APB1ENR.bit.pwren = SET;

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
	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_4;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_5;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_6;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_1;
	  GPIO_Init(&GPIO_InitStruct);


	  GPIO_InitStruct.pGPIOx = GPIOA;
	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_4;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_6;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.pGPIOx = GPIOC;
	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_6;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_7;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_8;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.pGPIOx = GPIOB;
	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_6;
	  GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	  GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	  GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	  GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 13;
	  GPIO_Init(&GPIO_InitStruct);

	  GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_7;
	  GPIO_Init(&GPIO_InitStruct);


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

    __HAL_LINKDMA(hdcmi,DMA_Handle,hdma_dcmi);

    /* DCMI interrupt Init */
    HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);

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
    /* Peripheral clock disable */
	RCC->AHB2ENR.bit.dcmien = RESET;

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

    /* DCMI DMA DeInit */
    HAL_DMA_DeInit(hdcmi->DMA_Handle);

    /* DCMI interrupt DeInit */
    HAL_NVIC_DisableIRQ(DCMI_IRQn);
  }

}
