#include <stm32f407xx.h>
#include "main.h"
#include "fatfs.h"
#include "libjpeg.h"
#include "usb_host.h"
#include <usbh_platform.h>
#include <simple_delay.h>
#include <ov2640.h>



/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
FATFS fs;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void GPIO_Config(void);
static void DMA_Config(void);
static void DCMI_Config(void);
void MX_USB_HOST_Process(void);


int main(void)
{

	HAL_Init();

	// configure the system clock
	SystemClock_Config();

	// configure the peripherals common clocks */
	PeriphCommonClock_Config();

	// peripheral initialization
	GPIO_Config();
	TIM1_Config();
	DMA_Config();
	DCMI_Config();
	MX_FATFS_Init();
	MX_LIBJPEG_Init();
	MX_USB_HOST_Init();

	// turn on USB powering
	MX_DriverVbusFS(0);

	while (1)
	{

		MX_USB_HOST_Process();

	}

}


void SystemClock_Config(void)
{

	RCC_OscInit_t RCC_OscInitStruct = {0};
	RCC_ClkInit_t RCC_ClkInitStruct = {0};

	RCC->APB1ENR.bit.pwren = SET;
	PWR->CR.bit.vos = 1;

	//HSE config
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.State = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.Source = 1;
	RCC_OscInitStruct.PLL.M = 4;
	RCC_OscInitStruct.PLL.N= 168;
	RCC_OscInitStruct.PLL.P= 0;	// PLL division by 2
	RCC_OscInitStruct.PLL.Q= 7;
	RCC_OscConfig(&RCC_OscInitStruct);

	//LSI configuration
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscConfig(&RCC_OscInitStruct);

	// configuration of the rest of the clocks for buses and CPU
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	RCC_ClockConfig(&RCC_ClkInitStruct);

	// SYSCLK initialization
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClockConfig(&RCC_ClkInitStruct);

	// PCLK1 initialization
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK1;
	RCC_ClockConfig(&RCC_ClkInitStruct);

	//PCLK2 initialization
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK2;
	RCC_ClockConfig(&RCC_ClkInitStruct);

	// configuration of the MCO2 that is used for camera XCLK
	RCC_MCOConfig(RCC_MCO2, 1, RCC_MCODIV_4);

}

void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
	Error_Handler();
	}
}

static void DCMI_Config(void)
{
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
}

static void DMA_Config(void)
{
	// enable DMA2 clock
	RCC->AHB1ENR.bit.dma2en = SET;

	// initialize DMA2_Stream1 interrupt which is used for DCMI
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}


static void GPIO_Config(void)
{
	GPIO_Handle_t GPIO_InitStruct = {0};

	// enable all necessary GPIO clocks
	RCC_AHB1ENR_Reg_t AHB1ENR_temp;
	AHB1ENR_temp.reg = RCC->AHB1ENR.reg;
	AHB1ENR_temp.bit.gpioeen = SET;
	AHB1ENR_temp.bit.gpiohen = SET;
	AHB1ENR_temp.bit.gpiocen = SET;
	AHB1ENR_temp.bit.gpioaen = SET;
	AHB1ENR_temp.bit.gpioden = SET;
	AHB1ENR_temp.bit.gpioben = SET;
	RCC->AHB1ENR.reg = AHB1ENR_temp.reg;

	// make sure PC0 is low; this pin is used for Drive_VBUS_FS
	GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);

	// default PWDN pin state when camera is working; you may connect camera's PWDN pin to GND as well
	GPIO_WritePin(GPIOC, CAMERA_PWDN_Pin, RESET);

	// Drive_VBUS_FS pin configuration
	GPIO_InitStruct.pGPIOx = GPIOC;
	GPIO_InitStruct.GPIO_Config.PinNumber = GPIO_PIN_0;
	GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_OUTPUT;
	GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&GPIO_InitStruct);

	// PWDN pin configuration
	GPIO_InitStruct.GPIO_Config.PinNumber = CAMERA_PWDN_Pin;
	GPIO_Init(&GPIO_InitStruct);

	// configure USER_Btn to be used as EXTI; or use any other EXTI you want
	GPIO_InitStruct.pGPIOx = USER_Btn_GPIO_Port;
	GPIO_InitStruct.GPIO_Config.PinNumber = USER_Btn_Pin;
	GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_IT_RT;
	GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_DOWN;
	GPIO_Init(&GPIO_InitStruct);



	// configure DCMI_XCLX_Pin
	GPIO_InitStruct.pGPIOx = DCMI_XCLX_GPIO_Port;
	GPIO_InitStruct.GPIO_Config.PinNumber = DCMI_XCLX_Pin;
	GPIO_InitStruct.GPIO_Config.PinMode = GPIO_MODE_AF;
	GPIO_InitStruct.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	GPIO_InitStruct.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIO_InitStruct.GPIO_Config.PinSpeed = GPIO_SPEED_LOW;
	GPIO_InitStruct.GPIO_Config.PinAltFuncMode = 0;
	GPIO_Init(&GPIO_InitStruct);
}

void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}
