/*										@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
										@@@@@@@@@@@@@@@@##@@@@@@@@`%@@@@@@@@@@@@@@@@@@@@
										@@@@@@@@@@@@@@@@‾‾* `        ` *@@@@@@@@@@@@@@@@@
										@@@@@@@@@@@@@@#                   #@@@@@@@@@@@@@
										@@@@@@@@@@@@                        @@@@@@@@@@@@
										@@@@@@@@@@@          _ @@@@@@@\     ``\@@@@@@@@@
										@@@@@@@@%       %@@@@ ``*@@@@@@\_      \@@@@@@@@
										@@@@@@@*      +@@@@@  /@@#  `*@@@@\_    \@@@@@@@
										@@@@@@/      /@@@@@   /@@  @@@@@@@@@|    !@@@@@@
										@@@@/       /@@@@@@@%  *  /` ___*@@@|    |@@@@@@
										@@@#       /@@@@@@@@@       ###}@@@@|    |@@@@@@
										@@@@@|     |@@@@@@@@@         __*@@@      @@@@@@
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
#include <stm32f407xx.h>
#include "main.h"
#include <fatfs.h>
#include <libjpeg.h>
#include <usb_host.h>
#include <usbh_platform.h>
#include <stm32f407xx_rtc.h>
#include <simple_delay.h>
#include <ov2640.h>
#include <avi.h>
#include <stdio.h>
#include <string.h>



/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
I2C_Handle_t hi2c1;
RTC_Handle_t hrtc;
FATFS fs;
FRESULT res;
FIL file;
HAL_StatusTypeDef status;

volatile uint8_t begin_rec = 0;
Current_Date_t video_dates[10];
Current_Time_t video_times[10];
int video_count = 0;


#define MAX_VIDEOS 10


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void GPIO_Config(void);
static void DMA_Config(void);
static void DCMI_Config(void);
void MX_USB_HOST_Process(void);
static void I2C_Config(void);
static void RTC_Config(void);


int compare_timestamp(const Current_Date_t* ts1_date, const Current_Time_t* ts1_time,
                      const Current_Date_t* ts2_date, const Current_Time_t* ts2_time)
{
    if (ts1_date->year != ts2_date->year)
    {
        return ts1_date->year - ts2_date->year;
    }
    if (ts1_date->month != ts2_date->month)
    {
        return ts1_date->month - ts2_date->month;
    }
    if (ts1_date->date != ts2_date->date)
    {
        return ts1_date->date - ts2_date->date;
    }
    if (ts1_time->hour != ts2_time->hour)
    {
        return ts1_time->hour - ts2_time->hour;
    }
    if (ts1_time->minute != ts2_time->minute)
    {
        return ts1_time->minute - ts2_time->minute;
    }
    return ts1_time->second - ts2_time->second;
}

void add_video_timestamp(const Current_Date_t* date, const Current_Time_t* time)
{
    if (video_count < MAX_VIDEOS)
    {
        memcpy(&video_dates[video_count], date, sizeof(Current_Date_t));
        memcpy(&video_times[video_count], time, sizeof(Current_Time_t));
        video_count++;
    }
}

void delete_oldest_video(void)
{
    if (video_count > 0)
    {
        int oldest_index = 0;
        for (int i = 1; i < video_count; ++i)
        {
            if (compare_timestamp(&video_dates[i], &video_times[i],
                                  &video_dates[oldest_index], &video_times[oldest_index]) < 0)
            {
                oldest_index = i;
            }
        }

        // Delete the oldest video file
        char filename[64];
        sprintf(filename, "%02dy%02dm%02dd_REC_%02d_%02d_%02d.avi",
        		video_dates[oldest_index].year, video_dates[oldest_index].month, video_dates[oldest_index].date,video_times[oldest_index].hour, video_times[oldest_index].minute, video_times[oldest_index].second);
        char filepath[128];
        sprintf(filepath, "/Video/%s", filename);
        FRESULT fr = f_unlink(filepath);
        if (fr == FR_OK) {
            printf("Deleted oldest video: %s\n", filename);
        } else {
            printf("Failed to delete oldest video: %s\n", filename);
        }

        // Remove the oldest timestamp from the list
        for (int i = oldest_index; i < video_count - 1; ++i) {
            memcpy(&video_dates[i], &video_dates[i + 1], sizeof(Current_Time_t));
            memcpy(&video_times[i], &video_times[i + 1], sizeof(Current_Time_t));
        }
        video_count--;
    }
}

void check_delete_oldest_video(void)
{
    if (video_count >= (MAX_VIDEOS+1))
    {
        delete_oldest_video();
    }
}

void rec_begin(void)
{

	Current_Date_Handle_t sCurrent;
    RTC_GetTime(&hrtc, &sCurrent);
    RTC_GetDate(&hrtc, &sCurrent);

    // Add the timestamp to the list
    add_video_timestamp(&sCurrent.Date, &sCurrent.Time);

    // Delete the oldest video if there are already 10 videos recorded
    check_delete_oldest_video();

	char fn[64];
	memset(fn, 0, 64);

	sprintf(fn, "0:/VIDEO/%02dy%02dm%02dd_REC_%02d_%02d_%02d.avi",
			sCurrent.Date.year, sCurrent.Date.month, sCurrent.Date.date, sCurrent.Time.hour, sCurrent.Time.minute, sCurrent.Time.second);
	res = f_open(&file, fn, FA_CREATE_ALWAYS|FA_WRITE);
	if (res==FR_OK)
	{
	  start_output_mjpeg_avi(&file, &hdcmi, (uint8_t)3, (uint8_t)1);
	}
}


int main(void)
{

	HAL_Init();

	// configure the system clock
	SystemClock_Config();

	// configure the peripherals common clocks */
	PeriphCommonClock_Config();

	// peripheral initialization
	TIM1_Config();
	MX_USB_HOST_Init();
	GPIO_Config();
	DMA_Config();
	MX_FATFS_Init();
	DCMI_Config();
	I2C_Config();
	MX_LIBJPEG_Init();
	RTC_Config();



	// turn on USB powering
	MX_DriverVbusFS(0);

	while (1)
	{
		if((begin_rec != 0) && (read_avi_output_status() == AVI_READY))
		{
			HAL_DCMI_Stop(&hdcmi);
			rec_begin();
		}
		if (read_avi_output_status() == AVI_CLOSED_OUT) //output finished
		{
		  set_avi_output_status(AVI_READY);
		}
		MX_USB_HOST_Process();

	}

}


void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	RCC->APB1ENR.bit.pwren = SET;
	PWR->CR.bit.vos = 1;

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_4);
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

	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

static void I2C_Config(void)
{
	GPIO_Handle_t sI2C_GPIO = {0};

	hi2c1.pI2Cx = I2C1;
	hi2c1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STANDARD;
	hi2c1.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	hi2c1.I2C_Config.I2C_DeviceAddress = 0;
	hi2c1.I2C_Config.I2C_AckControl = I2C_ACK_DISABLE;
	I2C_Init(&hi2c1);

	/*I2C1 GPIO Configuration
	PB8     ------> I2C1_SCL
	PB9     ------> I2C1_SDA */

	sI2C_GPIO.pGPIOx = GPIOB;
	sI2C_GPIO.GPIO_Config.PinNumber = GPIO_PIN_8;
	sI2C_GPIO.GPIO_Config.PinMode = GPIO_MODE_AF;
	sI2C_GPIO.GPIO_Config.PinOPType = GPIO_OUTPUT_OD;
	sI2C_GPIO.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
	sI2C_GPIO.GPIO_Config.PinSpeed = GPIO_SPEED_HIGH;
	sI2C_GPIO.GPIO_Config.PinAltFuncMode = 4;
	GPIO_Init(&sI2C_GPIO);

	sI2C_GPIO.GPIO_Config.PinNumber = GPIO_PIN_9;
	GPIO_Init(&sI2C_GPIO);

	RCC->APB1ENR.bit.i2c1en = SET;

}

static void RTC_Config(void)
{
	RTC_Alarm_t sAlarm = {0};

	// initialize RTC working mode
	hrtc.RTC_Config.RTC_HourFormat = RTC_HOURFORMAT_24;
	hrtc.RTC_Config.RTC_AsynchPrediv = 128;
	hrtc.RTC_Config.RTC_SynchPrediv = 256;

	// set RTC time
	hrtc.Time.hour = 11;
	hrtc.Time.minute = 22;
	hrtc.Time.second = 33;

	// set RTC date
	hrtc.Date.weekDay = WEDNESDAY;
	hrtc.Date.month = JANUARY;
	hrtc.Date.date = 1;
	hrtc.Date.year = 25;
	RTC_Init(&hrtc);

	// alarm A configuration
	sAlarm.hour = 00;
	sAlarm.minute = 00;
	sAlarm.second = 00;
	sAlarm.sec_msk = RTC_ALARM_MASK_UNMASKED;
	sAlarm.min_msk = RTC_ALARM_MASK_MINUTES;
	sAlarm.hour_msk = RTC_ALARM_MASK_HOURS;
	sAlarm.dateweek_msk = RTC_ALARM_MASK_DATEWEEKDAY;
	RTC_SetAlarm_IT(&hrtc, RTC_ALARM_A, &sAlarm);

	EXTI->IMR.bit.mr17 = SET;
	EXTI->RTSR.bit.tr17 = SET;

	RTC->WPR.bit.key = 0xFFU;

	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

}

void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}
