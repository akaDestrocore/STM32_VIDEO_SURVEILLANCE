/*
 @file     ov2640.h								@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 @brief    ov2640 camera module driver for		@@@@@@@@@@@@@@@@##@@@@@@@@`%@@@@@@@@@@@@@@@@@@@@
		   STM32F4								@@@@@@@@@@@@@@@@‾‾* `        ` *@@@@@@@@@@@@@@@@@
												@@@@@@@@@@@@@@#                   #@@@@@@@@@@@@@
												@@@@@@@@@@@@                        @@@@@@@@@@@@
												@@@@@@@@@@@          _ @@@@@@@\     ``\@@@@@@@@@
This file provides functions to manage the 	    @@@@@@@@%       %@@@@ ``*@@@@@@\_      \@@@@@@@@
following functionalities of ov2640 camera 		@@@@@@@*      +@@@@@  /@@#  `*@@@@\_    \@@@@@@@
module:											@@@@@@/      /@@@@@   /@@  @@@@@@@@@|    !@@@@@@
Initialization in three working modes: 			@@@@/       /@@@@@@@%  *  /` ___*@@@|    |@@@@@@
+ movie mode									@@@#       /@@@@@@@@@       ###}@@@@|    |@@@@@@
+ monitor mode									@@@@@|     |@@@@@@@@@      	  __*@@@      @@@@@@
+ picture mode									@@@@@*     |@@@@@@@@@        /@@@@@@@/     '@@@@
												@@@@@@|    |@@ \@@          @@@@@@@@@      /@@@@
												@@@@@@|     |@@ _____     @@@@@@@@*       @@@@@@
												@@@@@@*     \@@@@@@@@@    @@@@@@@/         @@@@@
												@@@@@@@\     \@@@@@@@@@  @@@@@@@%        @@@@@@@
												@@@@@@@@\     \@@@@@@@@  @\  ‾‾‾           @@@@@@
												@@@@@@@@@@\    \@@@@@@@  @@/ _==> $     @@@@@@@@
												@@@@@@@@@@@@*    \@@@@@@@@@@@##‾‾   ``  @@@@@@@@@
												@@@@@@@@@@@@@@@@\     ___*@@`*    /@@@@@@@@@@@@@
												@@@@@@@@@@@@@@@@@@@@@--`@@@@__@@@@@@@@@@@@@@@@@@
												@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
												@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@													*/

#ifndef __OV2640_H
#define __OV2640_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f407xx.h>
#include <simple_delay.h>


#define CAMERA_PWDN_Pin GPIO_PIN_12		// also may be connected to GND after successful configuration
#define CAMERA_PWDN_GPIO_Port GPIOC
#define CAMERA_RESET_Pin GPIO_PIN_0
#define CAMERA_RESET_GPIO_Port GPIOD


/*
 * Camera working modes
 */
typedef enum
{
	CAMERA_Monitor 	= 0,
	CAMERA_Movie 	= 1,
	CAMERA_Picture 	= 2
}CAMERA_Mode_t;

void     ov2640_Init(uint16_t DeviceAddr, uint8_t action);
void     CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value);
void     CAMERA_Delay(uint32_t delay);


#ifdef __cplusplus
}
#endif
#endif /* __OV2640_H */
