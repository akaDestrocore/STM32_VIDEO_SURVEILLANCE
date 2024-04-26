#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_0
#define USER_Btn_GPIO_Port GPIOA
#define USER_Btn_EXTI_IRQn EXTI0_IRQn
#define DCMI_XCLX_Pin GPIO_PIN_9
#define DCMI_XCLX_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
