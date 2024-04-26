/*
 @file     stm32f407xx_system.h					@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 @brief    STM32F407xx System Configuration 	@@@@@@@@@@@@@@@@##@@@@@@@@`%@@@@@@@@@@@@@@@@@@@@
  		   driver.								@@@@@@@@@@@@@@@@‾‾* `        ` *@@@@@@@@@@@@@@@@@
 @author   destrocore							@@@@@@@@@@@@@@#                   #@@@@@@@@@@@@@
 @version  1.0									@@@@@@@@@@@@                        @@@@@@@@@@@@
												@@@@@@@@@@@          _ @@@@@@@\     ``\@@@@@@@@@
This file provides firmware functions to manage @@@@@@@@%       %@@@@ ``*@@@@@@\_      \@@@@@@@@
the following functionalities of the system: 	@@@@@@@*      +@@@@@  /@@#  `*@@@@\_    \@@@@@@@
+ Initialization function 						@@@@@@/      /@@@@@   /@@  @@@@@@@@@|    !@@@@@@
+ Tick function									@@@@/       /@@@@@@@%  *  /` ___*@@@|    |@@@@@@
												@@@#       /@@@@@@@@@       ###}@@@@|    |@@@@@@
												@@@@@|     |@@@@@@@@@      	  __*@@@      @@@@@@
												@@@@@*     |@@@@@@@@@        /@@@@@@@/     '@@@@
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
#ifndef __STM32F407XX_SYSTEM_H_
#define __STM32F407XX_SYSTEM_H_

#include <stm32f407xx.h>


void SysTick_Init(void);
void SysTick_Handler(void);
uint32_t GetTick(void);


#endif /* INC_STM32F407XX_SYSTEM_H_ */
