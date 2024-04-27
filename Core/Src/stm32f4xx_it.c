
/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx_it.h>


/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t begin_rec;


extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_dcmi;
extern DCMI_HandleTypeDef hdcmi;


void EXTI0_IRQHandler(void)
{
	static uint32_t last_time = 0;
	uint32_t current_time = HAL_GetTick();
	 if((current_time - last_time) > 300)	//debouncing
	 {
		 begin_rec = 1;
		 if (read_avi_output_status() == AVI_START)
		 {
			 set_avi_output_status(AVI_PENDING); // stop recording
		 }
		 last_time = current_time; // Update the last_time
	 }
	 EXTI->PR.bit.pr0 = SET;	 // clear interrupt pending bit for EXTI Line 0
}

void RTC_Alarm_IRQHandler(void)
{
	EXTI->PR.bit.pr17 = SET;

	if(SET == RTC->CR.bit.alraie)
	{
		if(SET == RTC->ISR.bit.alraf)
		{
			RTC->ISR.bit.alraf = RESET;
			begin_rec = 1;
			if (read_avi_output_status() == AVI_START)
			{
				set_avi_output_status(AVI_PENDING); // stop recording
			}
		}
	}
}

void NMI_Handler(void)
{
   while (1)
  {
  }

}

void HardFault_Handler(void)
{

  while (1)
  {

  }
}

void MemManage_Handler(void)
{

  while (1)
  {

  }
}

void BusFault_Handler(void)
{

  while (1)
  {

  }
}


void UsageFault_Handler(void)
{

  while (1)
  {

  }
}

void SVC_Handler(void)
{

}

void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}


void SysTick_Handler(void)
{
	HAL_IncTick();
}


void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_dcmi);
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{

  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
}


void DCMI_IRQHandler(void)
{

  HAL_DCMI_IRQHandler(&hdcmi);

}
