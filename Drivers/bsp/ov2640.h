#ifndef __OV2640_H
#define __OV2640_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f407xx.h>
#include <simple_delay.h>


#define CAMERA_PWDN_Pin GPIO_PIN_12
#define CAMERA_PWDN_GPIO_Port GPIOC
#define CAMERA_RESET_Pin GPIO_PIN_0
#define CAMERA_RESET_GPIO_Port GPIOD


#define CAMERA_Monitor	1
#define CAMERA_Movie	2
#define CAMERA_Picture	3

void     ov2640_Init(uint16_t DeviceAddr, uint8_t action);
void     CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value);
void     CAMERA_Delay(uint32_t delay);


#ifdef __cplusplus
}
#endif
#endif /* __OV2640_H */
