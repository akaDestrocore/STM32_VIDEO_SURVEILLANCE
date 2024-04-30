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


#define MAX_VIDEOS 4


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void GPIO_Config(void);
static void DMA_Config(void);
static void DCMI_Config(void);
void MX_USB_HOST_Process(void);
static void I2C_Config(void);
static void RTC_Config(void);


/********************************************************************************************************/
/* @function name        - compare_timestamp                                                           	*/
/*                                                                                                      */
/* @brief               	 - Compares two timestamps and determines their chronological order             */
/*                                                                                                      */
/* @parameter[in]        - pointer to the date structure of the first timestamp                         */
/*                                                                                                      */
/* @parameter[in]        - pointer to the time structure of the first timestamp                         */
/*                                                                                                      */
/* @parameter[in]        - pointer to the date structure of the second timestamp                        */
/*                                                                                                      */
/* @parameter[in]        - pointer to the time structure of the second timestamp                        */
/*                                                                                                      */
/* @return               - An integer representing the chronological order of the timestamps:           */
/*                         0 if equal, a positive number if the first is later, or a negative number    */
/*                         if the second is later                                                       */
/*                                                                                                      */
/* @Note                 - The function sequentially compares year, month, date, hour, minute, and 		*/
/*                         second of the two timestamps. It returns as soon as a difference is found    */
/********************************************************************************************************/

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

/********************************************************************************************************/
/* @function name        - add_video_timestamp                                                         	*/
/*                                                                                                      */
/* @brief               	 - adds a timestamp to the video timestamp array                                */
/*                                                                                                      */
/* @parameter[in]        - pointer to the date structure containing the date of the video               */
/*                                                                                                      */
/* @parameter[in]        - pointer to the time structure containing the time of the video               */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - The function stores the date and time of a video into separate arrays. It    */
/*                         increments the video count as long as it does not exceed the maximum number  */
/*                         of allowed videos.                                                           */
/********************************************************************************************************/
void add_video_timestamp(const Current_Date_t* date, const Current_Time_t* time)
{
    if (video_count < MAX_VIDEOS)
    {
        memcpy(&video_dates[video_count], date, sizeof(Current_Date_t));
        memcpy(&video_times[video_count], time, sizeof(Current_Time_t));
        video_count++;
    }
}

/********************************************************************************************************/
/* @function name        - delete_oldest_video                                                         	*/
/*                                                                                                      */
/* @brief                - deletes the oldest video file based on timestamp comparison                  */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function identifies the oldest video by comparing timestamps, constructs*/
/*                         the filename based on the date and time, deletes the file from the filesystem*/
/*                         and then removes the timestamp from the tracking arrays. It decrements the   */
/*                         video count to reflect the deletion.                                         */
/********************************************************************************************************/
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
        sprintf(filepath, "/VIDEO/%s", filename);
        f_unlink(filepath);

        // Remove the oldest timestamp from the list
        for (int i = oldest_index; i < video_count - 1; ++i)
        {
            memcpy(&video_dates[i], &video_dates[i + 1], sizeof(Current_Time_t));
            memcpy(&video_times[i], &video_times[i + 1], sizeof(Current_Time_t));
        }
        video_count--;
    }
}

/********************************************************************************************************/
/* @function name        - check_delete_oldest_video                                                   	*/
/*                                                                                                      */
/* @brief                - checks if the video count exceeds the maximum limit and deletes the oldest   */
/*                         video if necessary                                                           */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function is a safeguard to prevent exceeding the maximum number of      */
/*                         videos allowed. It calls the function to delete the oldest video when the    */
/*                         video count surpasses the maximum threshold.                                 */
/********************************************************************************************************/
void check_delete_oldest_video(void)
{
    if (video_count >= MAX_VIDEOS)
    {
        delete_oldest_video();
    }
}

/********************************************************************************************************/
/* @function name        - rec_begin                                                                   	*/
/*                                                                                                      */
/* @brief                - initiates the recording of a video by creating a new file with a timestamp   */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function retrieves the current date and time, adds the timestamp to the */
/*                         list of recorded videos, checks and deletes the oldest video if necessary,   */
/*                         and then starts the recording process by creating a new AVI file named with  */
/*                         the current timestamp and beginning the MJPEG AVI output.                    */
/********************************************************************************************************/
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

/********************************************************************************************************/
/* @function name        - main function body                                                           */
/*                                                                                                      */
/* @brief                - the main entry point of the program                                          */
/*                                                                                                      */
/* @Note                 - This function initializes the hardware abstraction layer, configures system  */
/*                         clock, peripheral common clocks, and various peripherals. It then enters an  */
/*                         infinite loop where it checks and handles the recording status, and processes*/
/*                         USB host related tasks.                                                      */
/********************************************************************************************************/
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

/********************************************************************************************************/
/* @function name        - SystemClock_Config                                                           */
/*                                                                                                      */
/* @brief                - configures the system clock and oscillator parameters                        */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function sets up the main internal regulator output voltage, initializes*/
/*                         the RCC oscillators with specified parameters, and configures the CPU, AHB,  */
/*                         and APB buses clocks. It also handles potential errors during configuration  */
/*                         and sets up the MCU output clock (MCO).                         				*/
/********************************************************************************************************/
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

/********************************************************************************************************/
/* @function name        - PeriphCommonClock_Config                                                    	*/
/*                                                                                                      */
/* @brief                - configures the peripheral common clock (PLL I2S)                             */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function initializes the peripheral clock configuration structure and   */
/*                         sets the PLL I2S clock parameters. It then applies these settings via the    */
/*                         HAL_RCCEx_PeriphCLKConfig function. If the configuration is not successful,  */
/*                         the Error_Handler function is called.                                        */
/********************************************************************************************************/

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

/********************************************************************************************************/
/* @function name        - DCMI_Config                                                                  */
/*                                                                                                      */
/* @brief                - configures the Digital Camera Interface (DCMI) peripheral                    */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function initializes the DCMI peripheral with specific synchronization, */
/*                         polarity, capture rate, data mode, and JPEG mode settings. If the            */
/*                         initialization fails, it calls the Error_Handler function.                   */
/********************************************************************************************************/
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

/********************************************************************************************************/
/* @function name        - DMA_Config                                                                   */
/*                                                                                                      */
/* @brief                - configures the Direct Memory Access (DMA) for use with the DCMI              */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function enables the clock for DMA2 and initializes the DMA2_Stream1    */
/*                         interrupt with priority settings. This stream is configured for use with the */
/*                         Digital Camera Interface (DCMI) for image data transfer.                     */
/********************************************************************************************************/
static void DMA_Config(void)
{
	// enable DMA2 clock
	RCC->AHB1ENR.bit.dma2en = SET;

	// initialize DMA2_Stream1 interrupt which is used for DCMI
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/********************************************************************************************************/
/* @function name        - GPIO_Config                                                                  */
/*                                                                                                      */
/* @brief                - Configures the General-Purpose Input/Output (GPIO) pins                      */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function enables the necessary GPIO clocks, ensures specific pins are   */
/*                         set to desired states, and configures various pins for their intended        */
/*                         functions such as output, input, alternate function, and external interrupts.*/
/*                         It also sets the priority and enables interrupts for EXTI line 0.            */
/********************************************************************************************************/
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

/********************************************************************************************************/
/* @function name        - I2C_Config                                                                  	*/
/*                                                                                                      */
/* @brief                - Configures the Inter-Integrated Circuit (I2C) interface                      */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function initializes the I2C1 peripheral with standard speed, disables  */
/*                         acknowledgments, and sets the duty cycle. It also configures the GPIO pins   */
/*                         PB8 and PB9 for I2C SCL and SDA functions, respectively, and enables the I2C1*/
/*                         clock in the RCC peripheral clock enable register.                           */
/********************************************************************************************************/
static void I2C_Config(void)
{
	GPIO_Handle_t sI2C_GPIO = {0};

	hi2c1.pI2Cx = I2C1;
	hi2c1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STANDARD;
	hi2c1.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	hi2c1.I2C_Config.I2C_DeviceAddress = 0;
	hi2c1.I2C_Config.I2C_AckControl = I2C_ACK_DISABLE;
	I2C_Init(&hi2c1);

	//I2C1 GPIO Configuration
	// PB8 --> I2C1_SCL
	// PB9 --> I2C1_SDA

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

/********************************************************************************************************/
/* @function name        - RTC_Config                                                                   */
/*                                                                                                      */
/* @brief                - Configures the Real-Time Clock (RTC) and alarm settings                      */
/*                                                                                                      */
/* @return               - none                                                                         */
/*                                                                                                      */
/* @Note                 - This function sets up the RTC with a 24-hour format, initializes the RTC with*/
/*                         predefined prescalers, sets the current time and date, configures an alarm   */
/*                         with specific mask settings, and enables the RTC alarm interrupt with priority*/
/*                         settings. It also unlocks the RTC write protection for further configurations*/
/********************************************************************************************************/
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
	hrtc.Date.date = 0x1;
	hrtc.Date.year = 0x25;
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
