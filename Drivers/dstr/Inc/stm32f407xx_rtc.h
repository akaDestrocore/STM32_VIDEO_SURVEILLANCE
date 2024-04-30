/*
 @file     stm32f407xx_rtc.h					@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 @brief    STM32F407xx Device Peripheral		@@@@@@@@@@@@@@@@##@@@@@@@@`%@@@@@@@@@@@@@@@@@@@@
  		   RTC driver.							@@@@@@@@@@@@@@@@‾‾* `        ` *@@@@@@@@@@@@@@@@@
 @author   destrocore							@@@@@@@@@@@@@@#                   #@@@@@@@@@@@@@
 @version  1.0									@@@@@@@@@@@@                        @@@@@@@@@@@@
												@@@@@@@@@@@          _ @@@@@@@\     ``\@@@@@@@@@
This file provides firmware functions to manage @@@@@@@@%       %@@@@ ``*@@@@@@\_      \@@@@@@@@
the following functionalities of the Real	 	@@@@@@@*      +@@@@@  /@@#  `*@@@@\_    \@@@@@@@
Time Clock peripheral:							@@@@@@/      /@@@@@   /@@  @@@@@@@@@|    !@@@@@@
peripheral: 	 	 	 	 	 	 	 	 	@@@@/       /@@@@@@@%  *  /` ___*@@@|    |@@@@@@
+ Initialization and de-initialization function	@@@#       /@@@@@@@@@       ###}@@@@|    |@@@@@@
+ Peripheral Control functions					@@@@@|     |@@@@@@@@@      	  __*@@@      @@@@@@
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
#ifndef __STM32F407XX_RTC_H_
#define __STM32F407XX_RTC_H_

#include <stdint.h>
#include <stm32f407xx.h>

typedef enum
{
	RTC_DISABLE,
	RTC_ENABLE,
	RTC_RESET
}RTC_Clock_State_t;

typedef struct
{
	uint8_t RTC_HourFormat;			//RTC Hour format
	uint8_t RTC_AsynchPrediv;		//RTC Asynchronous prescaler value
	uint16_t RTC_SynchPrediv;		//RTC Synchronous prescaler value
}RTC_Config_t;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t AM_PM;
}Time_t;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t hour_format;
}Current_Time_t;

typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t weekDay;
}Date_t;

typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t weekDay;
}Current_Date_t;

typedef struct
{
	Current_Time_t Time;
	Current_Date_t	Date;
}Current_Date_Handle_t;

typedef struct
{
	RTC_Config_t RTC_Config;		//RTC configuration structure
	Time_t		 Time;
	Date_t	     Date;
}RTC_Handle_t;

typedef enum
{
	RTC_HOURFORMAT_24,
	RTC_HOURFORMAT_12
}HourFormat_t;

typedef enum
{
	RTC_12_HOURS_AM,
	RTC_12_HOURS_PM
}RTC_AM_PM_t;

typedef enum
{
	MONDAY  	= 0x1U,
	TUESDAY		= 0x2U,
	WEDNESDAY	= 0x3U,
	THURSDAY	= 0x4U,
	FRIDAY		= 0x5U,
	SATURDAY	= 0x6U,
	SUNDAY		= 0x7U
}Week_Day_t;

typedef enum
{
	JANUARY  	= 0x1U,
	FEBRUARY 	= 0x2U,
	MARCH	 	= 0x3U,
	APRIL	 	= 0x4U,
	MAY 	 	= 0x5U,
	JUNE	 	= 0x6U,
	JULY	 	= 0x7U,
	AUGUST 	 	= 0x8U,
	SEPTEMBER 	= 0x9U,
	OCTOBER		= 0xAU,
	NOVEMBER	= 0xBU,
	DECEMBER	= 0xCU
}Month_t;

typedef enum
{
	RTC_ALARM_MASK_UNMASKED	   = 0,
	RTC_ALARM_MASK_DATEWEEKDAY = 1,
	RTC_ALARM_MASK_HOURS       = 1,
	RTC_ALARM_MASK_MINUTES     = 1,
	RTC_ALARM_MASK_SECONDS     = 1
}RTC_AlarmMask_t;

typedef struct
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    RTC_AlarmMask_t sec_msk;
    RTC_AlarmMask_t min_msk;
    RTC_AlarmMask_t hour_msk;
    RTC_AlarmMask_t dateweek_msk;
}RTC_Alarm_t;

typedef enum {
    RTC_ALARM_A = 0, // Alarm A
    RTC_ALARM_B = 1  // Alarm B
} RTC_AlarmType_t;



//RTC clock enable/disable
void RTC_ClockControl(RTC_Clock_State_t state);

//RTC initialization
void RTC_Init(RTC_Handle_t *pRTCHandle);

//RTC Time configuration
void RTC_SetTime(RTC_Handle_t *pRTCHandle);

//RTC Date configuration
void RTC_SetDate(RTC_Handle_t *pRTCHandle);

//RTC Time Information
void RTC_GetTime(RTC_Handle_t * pRTCHandle, Current_Date_Handle_t* pCurrentDateHandle);

//RTC Date Information
void RTC_GetDate(RTC_Handle_t *pRTCHandle, Current_Date_Handle_t* pCurrentDateHandle);

//RTC System Clock Configuration
void RTC_SystemClock_Config(uint32_t clk);

//RTC Alarm configuration
void RTC_SetAlarm_IT(RTC_Handle_t *pRTCHandle, RTC_AlarmType_t alarmType, RTC_Alarm_t *pAlarm);


#endif /* __STM32F407XX_RTC_H_ */
