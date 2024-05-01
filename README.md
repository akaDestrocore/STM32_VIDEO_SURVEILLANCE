
# STM32 VIDEO SURVEILLANCE 

This project is designed for video recording with a digital camera, utilizing the capabilities of an STM32F4 microcontroller. In this project specifically STM32F407 Discovery development kit is used to encode the video. 

![Development board](https://github.com/akaDestrocore/STM32_VIDEO_SURVEILLANCE/blob/main/controller.jpg) 

## System functions

- **Video recording:** System is recording 320x240 [*RIFF-AVI Motion JPEG*](https://learn.microsoft.com/en-us/windows/win32/directshow/avi-riff-file-reference) up to 3 frames per second. 
- **Storing cpatured video data on external USB drive:** Video file is being encoded on external video drive. Standart built-in FatFs library from CubeMX and HAL USB HOST driver is used for these purposes.
- **Memory space management:** Stores up to **MAX_VIDEOS** amount of videos which in my case is **10**. Video recording time is bound to RTC so in this case each video is 1 minute long. May be adjusted according to your needs.

## Main Components

* STM32F4 Discovery board
    * STM32F407 VGT (Cortex-M4)  
* Camera module
    * Waveshare OV2640 board without FIFO
* FAT formated USB drive
    * 32GB pendrive

## Software

* **FATFS**
* **LIBJPEG**
* **STM32 HAL** is used for DCMI, DMA and RCC. 

## Peripheral configurations

* **I2C1** to control OV2640 camera module: 
    * Default configurations: **Standard Mode**, **100000 Hz**, **no stretching**, **7-bit address length**
    * Make sure to set I2C pins to Pull Up mode and maximum output speed
* **MCO2** to output DCMI XCLK: 
    * **PLL** is used as System Clock source to generate 168 MHz;
    * **PLL Q** is set to **/7** to generate 48 MHz (this is required for USB OTG);
    * **PLLI2SCLK** is used as source for **MCO2**, and division is **/4** which sets **MCO2** to **24 MHz** (*this allows stable operation of OV2640*)    
 * **DCMI interface**:
    * **DCMI mode**: Slave 8 bits External Synchro 
    * **PIXCLK**:  Active on **Rising edge**
    * **VSYNC**: Active Low
    * **HSYNC**: Active Low
    * **Frame capture**: All frames are captured
    * **JPEG mode**: On
    * **DMA2 Stream1** is used to stream DCMI data in Circular mode and Full FIFO Threshhold
* **Real Time Clock**:
    * **Hour format**: 24 hours
    * **Async prediv**: 128
    * **Synch prediv**: 256
    * **Alarm**: **Alarm A** is used for triggering video recording ending and beginning of the new video recording. 
* **Timer 1**: 
    * TIM1 is used for <simple_delay.h> module

## Pinmap
Note: *Use short jumper wires for OV2640 (less than 7 cm)*

 **PE4**: DCMI_D4  
 **PE5**: DCMI_D6  
 **PE6**: DCMI_D7   
 **PA4**: DCMI_HSYNC  
 **PA6**: DCMI_PIXCLK  
 **PC6**: DCMI_D0  
 **PC7**: DCMI_D1  
 **PC8**: DCMI_D2  
 **PC9**: DCMI_XCLX  
 **PB8**: I2C1_SCL  
 **PB9**: I2C1_SDA  
 **PE1**: DCMI_D3  
 **PC12**: CAMERA_PWDN (or connect it to any GND)  
 **PD0**: CAMERA_RESET  
 **PB6**: DCMI_D5  
 **PB7**: DCMI_VSYNC  
 **PA9**: USB_OTG_FS_VBUS  
 **PA11**: USB_OTG_FS_DM  
 **PA12**: USB_OTG_FS_DP
 **PC0**: GPIO_Output for Drive_VBUS_FS  
 **PA0**: used as example of EXTI used in order to terminate ongoing video recording   




## System overview

**OV2640 camera module**:  
The OV2640 module used in this зкщоусе is a 8bit (D2-D9) output, including VSYNC, HREF, RET, PWDN, SIOC, SIOD, VCC and GND pins. The maximum output is 2M, and the format can be RGB565 or JPEG format. Please refer to the datasheet for more details.  

![OV2640](https://www.waveshare.com/media/catalog/product/cache/1/image/560x560/9df78eab33525d08d6e5fb8d27136e95/o/v/ov2640-camera-board_l_1_5.jpg)

There are two registers for DSP and Sensor (0xFF is 0 or 1). Brief description of some of them:  
* **0xDA** (when 0xFF = 0): Image Mode;
* **0x5A** (when 0xFF = 0): Output width; divide the actual width by 4;  
* **0x5B** (when 0xFF = 0): Output height; Divide the actual height by 4;  
* **0x5C** (when 0xFF = 0): Bit[1:0] OUTW, Bit[2] OUTH. Output 1280x960 JPEG image with value 0x01
Data from OV2640 is being output to the corresponding DMA buffer through DMA2 Stream1.The output size is 320x240, and STM32F4 Discovery's hardware environment can only achieve 3 frames per second. Adopts the RIFF-AVI Header Motion JPEG format using **LIBJPEG**.

#### Initialization and Configuration
The driver initializes the OV2640 camera module and configures it for MJPEG image capture.
It sets up the necessary structures and parameters for capturing and encoding JPEG images.
The OV2640 camera module is configured to capture frames in MJPEG format, which is suitable for efficient video compression.
#### AVI File Header Generation
**avi.c** driver generates the header for the AVI file format, including the RIFF chunk, AVI main header (**avih**), and stream header (**strh**), among others.
Various parameters such as frames per second (FPS), video resolution, and compression settings are configured in the AVI file header.
#### Recording Process
The recording process is initiated by calling the ***start_output_mjpeg_avi()*** function.
This function captures JPEG frames from the OV2640 camera module and writes them to the AVI file.
Each frame is checked for valid JPEG headers before being written to the file. JPEG image binary should begin with **0xFFD8** and end with **FFD9**.
The recording continues until a stop condition is met, such as reaching a predefined duration or triggering an external event.
#### Stop and Finalization
When recording is complete, the ***stop_output_mjpeg_avi()*** function is called to finalize the AVI file.
This function writes the 'idx1' chunk, updates the file size, frame count, and other header information.
It closes the temporary index file used during recording and sets the recording status to **AVI_CLOSED_OUT**.
## Conclusion
In summary, the OV2640 camera driver for MJPEG AVI recording on STM32F4 microcontrollers provides a robust and efficient solution for capturing and encoding video footage. By leveraging the capabilities of the OV2640 camera module and the processing power of the STM32F4 microcontroller, video recording functionality may be inplemented in embedded applications with ease and flexibility.

![Recorded videos on the pendrive](https://github.com/akaDestrocore/STM32_VIDEO_SURVEILLANCE/blob/main/pendrive.png)

![Example of recorded 1 minute long video converted to gif](https://github.com/akaDestrocore/STM32_VIDEO_SURVEILLANCE/blob/main/24y04m23d_REC_11_06_00.gif)
## Related sources

1.[**RM0090 Reference manual**](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)

2.[**OV2640_DS**](https://files.waveshare.com/upload/6/6b/OV2640_DS%281.6%29.pdf)



