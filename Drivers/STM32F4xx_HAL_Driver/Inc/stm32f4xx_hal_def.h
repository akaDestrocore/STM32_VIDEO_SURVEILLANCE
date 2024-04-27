/**
  ******************************************************************************
  * @file    stm32f4xx_hal_def.h
  * @author  MCD Application Team
  * @brief   This file contains HAL common defines, enumeration, macros and 
  *          structures definitions. 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_DEF
#define __STM32F4xx_HAL_DEF

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Legacy/stm32_hal_legacy.h"
#include <stddef.h>

/**
* @brief USB_OTG_Core_Registers
*/
typedef struct
{
volatile uint32_t GOTGCTL;              /*!< USB_OTG Control and Status Register          000h */
volatile uint32_t GOTGINT;              /*!< USB_OTG Interrupt Register                   004h */
volatile uint32_t GAHBCFG;              /*!< Core AHB Configuration Register              008h */
volatile uint32_t GUSBCFG;              /*!< Core USB Configuration Register              00Ch */
volatile uint32_t GRSTCTL;              /*!< Core Reset Register                          010h */
volatile uint32_t GINTSTS;              /*!< Core Interrupt Register                      014h */
volatile uint32_t GINTMSK;              /*!< Core Interrupt Mask Register                 018h */
volatile uint32_t GRXSTSR;              /*!< Receive Sts Q Read Register                  01Ch */
volatile uint32_t GRXSTSP;              /*!< Receive Sts Q Read & POP Register            020h */
volatile uint32_t GRXFSIZ;              /*!< Receive FIFO Size Register                   024h */
volatile uint32_t DIEPTXF0_HNPTXFSIZ;   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
volatile uint32_t HNPTXSTS;             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
uint32_t Reserved30[2];             /*!< Reserved                                     030h */
volatile uint32_t GCCFG;                /*!< General Purpose IO Register                  038h */
volatile uint32_t CID;                  /*!< User ID Register                             03Ch */
uint32_t  Reserved40[48];           /*!< Reserved                                0x40-0xFF */
volatile uint32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg               100h */
volatile uint32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO                        */
} USB_OTG_GlobalTypeDef;

#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_OTG_HS          ((USB_OTG_GlobalTypeDef *) USB_OTG_HS_PERIPH_BASE)



/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSEON_Pos                   (16U)
#define RCC_CR_HSEON_Msk                   (0x1UL << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSEON                       RCC_CR_HSEON_Msk
#define RCC_CR_HSERDY_Pos                  (17U)
#define RCC_CR_HSERDY_Msk                  (0x1UL << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */
#define RCC_CR_HSERDY                      RCC_CR_HSERDY_Msk
#define RCC_CR_HSEBYP_Pos                  (18U)
#define RCC_CR_HSEBYP_Msk                  (0x1UL << RCC_CR_HSEBYP_Pos)         /*!< 0x00040000 */
#define RCC_CR_HSEBYP                      RCC_CR_HSEBYP_Msk
#define RCC_CR_CSSON_Pos                   (19U)
#define RCC_CR_CSSON_Msk                   (0x1UL << RCC_CR_CSSON_Pos)          /*!< 0x00080000 */
#define RCC_CR_CSSON                       RCC_CR_CSSON_Msk
#define RCC_CR_PLLON_Pos                   (24U)
#define RCC_CR_PLLON_Msk                   (0x1UL << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */
#define RCC_CR_PLLON                       RCC_CR_PLLON_Msk
#define RCC_CR_PLLRDY_Pos                  (25U)
#define RCC_CR_PLLRDY_Msk                  (0x1UL << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */
#define RCC_CR_PLLRDY                      RCC_CR_PLLRDY_Msk

#define RCC_CR_HSION_Pos                   (0U)
#define RCC_CR_HSION_Msk                   (0x1UL << RCC_CR_HSION_Pos)          /*!< 0x00000001 */
#define RCC_CR_HSION                       RCC_CR_HSION_Msk
#define RCC_CR_HSIRDY_Pos                  (1U)
#define RCC_CR_HSIRDY_Msk                  (0x1UL << RCC_CR_HSIRDY_Pos)         /*!< 0x00000002 */
#define RCC_CR_HSIRDY                      RCC_CR_HSIRDY_Msk

#define RCC_CR_HSITRIM_Pos                 (3U)
#define RCC_CR_HSITRIM_Msk                 (0x1FUL << RCC_CR_HSITRIM_Pos)       /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                     RCC_CR_HSITRIM_Msk
#define RCC_CR_HSITRIM_0                   (0x01UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                   (0x02UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                   (0x04UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                   (0x08UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                   (0x10UL << RCC_CR_HSITRIM_Pos)       /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos                  (8U)
#define RCC_CR_HSICAL_Msk                  (0xFFUL << RCC_CR_HSICAL_Pos)        /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                      RCC_CR_HSICAL_Msk
#define RCC_CR_HSICAL_0                    (0x01UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                    (0x02UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                    (0x04UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                    (0x08UL << RCC_CR_HSICAL_Pos)        /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                    (0x10UL << RCC_CR_HSICAL_Pos)        /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                    (0x20UL << RCC_CR_HSICAL_Pos)        /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                    (0x40UL << RCC_CR_HSICAL_Pos)        /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                    (0x80UL << RCC_CR_HSICAL_Pos)        /*!< 0x00008000 */

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 series)
 */
#define RCC_PLLI2S_SUPPORT                                                     /*!< Support PLLI2S oscillator */

#define RCC_CR_PLLI2SON_Pos                (26U)
#define RCC_CR_PLLI2SON_Msk                (0x1UL << RCC_CR_PLLI2SON_Pos)       /*!< 0x04000000 */
#define RCC_CR_PLLI2SON                    RCC_CR_PLLI2SON_Msk
#define RCC_CR_PLLI2SRDY_Pos               (27U)
#define RCC_CR_PLLI2SRDY_Msk               (0x1UL << RCC_CR_PLLI2SRDY_Pos)      /*!< 0x08000000 */
#define RCC_CR_PLLI2SRDY                   RCC_CR_PLLI2SRDY_Msk

/********************  Bit definition for RCC_PLLCFGR register  ***************/

#define RCC_PLLCFGR_PLLN_Pos               (6U)
#define RCC_PLLCFGR_PLLN_Msk               (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN                   RCC_PLLCFGR_PLLN_Msk
#define RCC_PLLCFGR_PLLN_0                 (0x001UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1                 (0x002UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2                 (0x004UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3                 (0x008UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4                 (0x010UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5                 (0x020UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6                 (0x040UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7                 (0x080UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8                 (0x100UL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLP_Pos               (16U)
#define RCC_PLLCFGR_PLLP_Msk               (0x3UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP                   RCC_PLLCFGR_PLLP_Msk
#define RCC_PLLCFGR_PLLP_0                 (0x1UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1                 (0x2UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00020000 */

#define RCC_PLLCFGR_PLLSRC_Pos             (22U)
#define RCC_PLLCFGR_PLLSRC_Msk             (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC                 RCC_PLLCFGR_PLLSRC_Msk
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk
#define RCC_PLLCFGR_PLLSRC_HSI             0x00000000U

#define RCC_PLLCFGR_PLLQ_Pos               (24U)
#define RCC_PLLCFGR_PLLQ_Msk               (0xFUL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ                   RCC_PLLCFGR_PLLQ_Msk
#define RCC_PLLCFGR_PLLQ_0                 (0x1UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1                 (0x2UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2                 (0x4UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3                 (0x8UL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x08000000 */

#define RCC_PLLCFGR_PLLM_Pos               (0U)
#define RCC_PLLCFGR_PLLM_Msk               (0x3FUL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM                   RCC_PLLCFGR_PLLM_Msk
#define RCC_PLLCFGR_PLLM_0                 (0x01UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1                 (0x02UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2                 (0x04UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3                 (0x08UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4                 (0x10UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5                 (0x20UL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000020 */


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                    (0U)
#define RCC_CFGR_SW_Msk                    (0x3UL << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */
#define RCC_CFGR_SW                        RCC_CFGR_SW_Msk                     /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                      (0x1UL << RCC_CFGR_SW_Pos)           /*!< 0x00000001 */
#define RCC_CFGR_SW_1                      (0x2UL << RCC_CFGR_SW_Pos)           /*!< 0x00000002 */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                  (4U)
#define RCC_CFGR_HPRE_Msk                  (0xFUL << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                      RCC_CFGR_HPRE_Msk                   /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                    (0x1UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                    (0x2UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                    (0x4UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                    (0x8UL << RCC_CFGR_HPRE_Pos)         /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

#define RCC_CFGR_SWS                       (0x3UL << 2U)
#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock                   */
/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)
#define RCC_CFGR_SWS_Msk                   (0x3UL << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C *//
#define RCC_CFGR_SWS_0                     (0x1UL << RCC_CFGR_SWS_Pos)          /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                     (0x2UL << RCC_CFGR_SWS_Pos)          /*!< 0x00000008 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)
#define RCC_CFGR_PPRE1_Msk                 (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk                  /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                   (0x1UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                   (0x2UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                   (0x4UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001000 */


/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)
#define RCC_CFGR_PPRE2_Msk                 (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk                  /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                   (0x1UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                   (0x2UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                   (0x4UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos                (16U)
#define RCC_CFGR_RTCPRE_Msk                (0x1FUL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE                    RCC_CFGR_RTCPRE_Msk
#define RCC_CFGR_RTCPRE_0                  (0x01UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1                  (0x02UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2                  (0x04UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3                  (0x08UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4                  (0x10UL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00100000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos                  (21U)
#define RCC_CFGR_MCO1_Msk                  (0x3UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00600000 */
#define RCC_CFGR_MCO1                      RCC_CFGR_MCO1_Msk
#define RCC_CFGR_MCO1_0                    (0x1UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1                    (0x2UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00400000 */

#define RCC_CFGR_I2SSRC_Pos                (23U)
#define RCC_CFGR_I2SSRC_Msk                (0x1UL << RCC_CFGR_I2SSRC_Pos)       /*!< 0x00800000 */
#define RCC_CFGR_I2SSRC                    RCC_CFGR_I2SSRC_Msk

#define RCC_CFGR_MCO1PRE_Pos               (24U)
#define RCC_CFGR_MCO1PRE_Msk               (0x7UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE                   RCC_CFGR_MCO1PRE_Msk
#define RCC_CFGR_MCO1PRE_0                 (0x1UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1                 (0x2UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2                 (0x4UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_Pos               (27U)
#define RCC_CFGR_MCO2PRE_Msk               (0x7UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE                   RCC_CFGR_MCO2PRE_Msk
#define RCC_CFGR_MCO2PRE_0                 (0x1UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1                 (0x2UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2                 (0x4UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x20000000 */

#define RCC_CFGR_MCO2_Pos                  (30U)
#define RCC_CFGR_MCO2_Msk                  (0x3UL << RCC_CFGR_MCO2_Pos)         /*!< 0xC0000000 */
#define RCC_CFGR_MCO2                      RCC_CFGR_MCO2_Msk
#define RCC_CFGR_MCO2_0                    (0x1UL << RCC_CFGR_MCO2_Pos)         /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1                    (0x2UL << RCC_CFGR_MCO2_Pos)         /*!< 0x80000000 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos                (0U)
#define RCC_CIR_LSIRDYF_Msk                (0x1UL << RCC_CIR_LSIRDYF_Pos)       /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                    RCC_CIR_LSIRDYF_Msk
#define RCC_CIR_LSERDYF_Pos                (1U)
#define RCC_CIR_LSERDYF_Msk                (0x1UL << RCC_CIR_LSERDYF_Pos)       /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                    RCC_CIR_LSERDYF_Msk
#define RCC_CIR_HSIRDYF_Pos                (2U)
#define RCC_CIR_HSIRDYF_Msk                (0x1UL << RCC_CIR_HSIRDYF_Pos)       /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                    RCC_CIR_HSIRDYF_Msk
#define RCC_CIR_HSERDYF_Pos                (3U)
#define RCC_CIR_HSERDYF_Msk                (0x1UL << RCC_CIR_HSERDYF_Pos)       /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                    RCC_CIR_HSERDYF_Msk
#define RCC_CIR_PLLRDYF_Pos                (4U)
#define RCC_CIR_PLLRDYF_Msk                (0x1UL << RCC_CIR_PLLRDYF_Pos)       /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                    RCC_CIR_PLLRDYF_Msk
#define RCC_CIR_PLLI2SRDYF_Pos             (5U)
#define RCC_CIR_PLLI2SRDYF_Msk             (0x1UL << RCC_CIR_PLLI2SRDYF_Pos)    /*!< 0x00000020 */
#define RCC_CIR_PLLI2SRDYF                 RCC_CIR_PLLI2SRDYF_Msk

#define RCC_CIR_CSSF_Pos                   (7U)
#define RCC_CIR_CSSF_Msk                   (0x1UL << RCC_CIR_CSSF_Pos)          /*!< 0x00000080 */
#define RCC_CIR_CSSF                       RCC_CIR_CSSF_Msk
#define RCC_CIR_LSIRDYIE_Pos               (8U)
#define RCC_CIR_LSIRDYIE_Msk               (0x1UL << RCC_CIR_LSIRDYIE_Pos)      /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                   RCC_CIR_LSIRDYIE_Msk
#define RCC_CIR_LSERDYIE_Pos               (9U)
#define RCC_CIR_LSERDYIE_Msk               (0x1UL << RCC_CIR_LSERDYIE_Pos)      /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                   RCC_CIR_LSERDYIE_Msk
#define RCC_CIR_HSIRDYIE_Pos               (10U)
#define RCC_CIR_HSIRDYIE_Msk               (0x1UL << RCC_CIR_HSIRDYIE_Pos)      /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                   RCC_CIR_HSIRDYIE_Msk
#define RCC_CIR_HSERDYIE_Pos               (11U)
#define RCC_CIR_HSERDYIE_Msk               (0x1UL << RCC_CIR_HSERDYIE_Pos)      /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                   RCC_CIR_HSERDYIE_Msk
#define RCC_CIR_PLLRDYIE_Pos               (12U)
#define RCC_CIR_PLLRDYIE_Msk               (0x1UL << RCC_CIR_PLLRDYIE_Pos)      /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                   RCC_CIR_PLLRDYIE_Msk
#define RCC_CIR_PLLI2SRDYIE_Pos            (13U)
#define RCC_CIR_PLLI2SRDYIE_Msk            (0x1UL << RCC_CIR_PLLI2SRDYIE_Pos)   /*!< 0x00002000 */
#define RCC_CIR_PLLI2SRDYIE                RCC_CIR_PLLI2SRDYIE_Msk

#define RCC_CIR_LSIRDYC_Pos                (16U)
#define RCC_CIR_LSIRDYC_Msk                (0x1UL << RCC_CIR_LSIRDYC_Pos)       /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                    RCC_CIR_LSIRDYC_Msk
#define RCC_CIR_LSERDYC_Pos                (17U)
#define RCC_CIR_LSERDYC_Msk                (0x1UL << RCC_CIR_LSERDYC_Pos)       /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                    RCC_CIR_LSERDYC_Msk
#define RCC_CIR_HSIRDYC_Pos                (18U)
#define RCC_CIR_HSIRDYC_Msk                (0x1UL << RCC_CIR_HSIRDYC_Pos)       /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                    RCC_CIR_HSIRDYC_Msk
#define RCC_CIR_HSERDYC_Pos                (19U)
#define RCC_CIR_HSERDYC_Msk                (0x1UL << RCC_CIR_HSERDYC_Pos)       /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                    RCC_CIR_HSERDYC_Msk
#define RCC_CIR_PLLRDYC_Pos                (20U)
#define RCC_CIR_PLLRDYC_Msk                (0x1UL << RCC_CIR_PLLRDYC_Pos)       /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                    RCC_CIR_PLLRDYC_Msk
#define RCC_CIR_PLLI2SRDYC_Pos             (21U)
#define RCC_CIR_PLLI2SRDYC_Msk             (0x1UL << RCC_CIR_PLLI2SRDYC_Pos)    /*!< 0x00200000 */
#define RCC_CIR_PLLI2SRDYC                 RCC_CIR_PLLI2SRDYC_Msk

#define RCC_CIR_CSSC_Pos                   (23U)
#define RCC_CIR_CSSC_Msk                   (0x1UL << RCC_CIR_CSSC_Pos)          /*!< 0x00800000 */
#define RCC_CIR_CSSC                       RCC_CIR_CSSC_Msk

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define RCC_AHB1RSTR_GPIOARST_Pos          (0U)
#define RCC_AHB1RSTR_GPIOARST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOARST_Pos) /*!< 0x00000001 */
#define RCC_AHB1RSTR_GPIOARST              RCC_AHB1RSTR_GPIOARST_Msk
#define RCC_AHB1RSTR_GPIOBRST_Pos          (1U)
#define RCC_AHB1RSTR_GPIOBRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOBRST_Pos) /*!< 0x00000002 */
#define RCC_AHB1RSTR_GPIOBRST              RCC_AHB1RSTR_GPIOBRST_Msk
#define RCC_AHB1RSTR_GPIOCRST_Pos          (2U)
#define RCC_AHB1RSTR_GPIOCRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOCRST_Pos) /*!< 0x00000004 */
#define RCC_AHB1RSTR_GPIOCRST              RCC_AHB1RSTR_GPIOCRST_Msk
#define RCC_AHB1RSTR_GPIODRST_Pos          (3U)
#define RCC_AHB1RSTR_GPIODRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIODRST_Pos) /*!< 0x00000008 */
#define RCC_AHB1RSTR_GPIODRST              RCC_AHB1RSTR_GPIODRST_Msk
#define RCC_AHB1RSTR_GPIOERST_Pos          (4U)
#define RCC_AHB1RSTR_GPIOERST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOERST_Pos) /*!< 0x00000010 */
#define RCC_AHB1RSTR_GPIOERST              RCC_AHB1RSTR_GPIOERST_Msk
#define RCC_AHB1RSTR_GPIOFRST_Pos          (5U)
#define RCC_AHB1RSTR_GPIOFRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOFRST_Pos) /*!< 0x00000020 */
#define RCC_AHB1RSTR_GPIOFRST              RCC_AHB1RSTR_GPIOFRST_Msk
#define RCC_AHB1RSTR_GPIOGRST_Pos          (6U)
#define RCC_AHB1RSTR_GPIOGRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOGRST_Pos) /*!< 0x00000040 */
#define RCC_AHB1RSTR_GPIOGRST              RCC_AHB1RSTR_GPIOGRST_Msk
#define RCC_AHB1RSTR_GPIOHRST_Pos          (7U)
#define RCC_AHB1RSTR_GPIOHRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOHRST_Pos) /*!< 0x00000080 */
#define RCC_AHB1RSTR_GPIOHRST              RCC_AHB1RSTR_GPIOHRST_Msk
#define RCC_AHB1RSTR_GPIOIRST_Pos          (8U)
#define RCC_AHB1RSTR_GPIOIRST_Msk          (0x1UL << RCC_AHB1RSTR_GPIOIRST_Pos) /*!< 0x00000100 */
#define RCC_AHB1RSTR_GPIOIRST              RCC_AHB1RSTR_GPIOIRST_Msk
#define RCC_AHB1RSTR_CRCRST_Pos            (12U)
#define RCC_AHB1RSTR_CRCRST_Msk            (0x1UL << RCC_AHB1RSTR_CRCRST_Pos)   /*!< 0x00001000 */
#define RCC_AHB1RSTR_CRCRST                RCC_AHB1RSTR_CRCRST_Msk
#define RCC_AHB1RSTR_DMA1RST_Pos           (21U)
#define RCC_AHB1RSTR_DMA1RST_Msk           (0x1UL << RCC_AHB1RSTR_DMA1RST_Pos)  /*!< 0x00200000 */
#define RCC_AHB1RSTR_DMA1RST               RCC_AHB1RSTR_DMA1RST_Msk
#define RCC_AHB1RSTR_DMA2RST_Pos           (22U)
#define RCC_AHB1RSTR_DMA2RST_Msk           (0x1UL << RCC_AHB1RSTR_DMA2RST_Pos)  /*!< 0x00400000 */
#define RCC_AHB1RSTR_DMA2RST               RCC_AHB1RSTR_DMA2RST_Msk
#define RCC_AHB1RSTR_ETHMACRST_Pos         (25U)
#define RCC_AHB1RSTR_ETHMACRST_Msk         (0x1UL << RCC_AHB1RSTR_ETHMACRST_Pos) /*!< 0x02000000 */
#define RCC_AHB1RSTR_ETHMACRST             RCC_AHB1RSTR_ETHMACRST_Msk
#define RCC_AHB1RSTR_OTGHRST_Pos           (29U)
#define RCC_AHB1RSTR_OTGHRST_Msk           (0x1UL << RCC_AHB1RSTR_OTGHRST_Pos)  /*!< 0x20000000 */
#define RCC_AHB1RSTR_OTGHRST               RCC_AHB1RSTR_OTGHRST_Msk

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define RCC_AHB2RSTR_DCMIRST_Pos           (0U)
#define RCC_AHB2RSTR_DCMIRST_Msk           (0x1UL << RCC_AHB2RSTR_DCMIRST_Pos)  /*!< 0x00000001 */
#define RCC_AHB2RSTR_DCMIRST               RCC_AHB2RSTR_DCMIRST_Msk
#define RCC_AHB2RSTR_RNGRST_Pos            (6U)
#define RCC_AHB2RSTR_RNGRST_Msk            (0x1UL << RCC_AHB2RSTR_RNGRST_Pos)   /*!< 0x00000040 */
#define RCC_AHB2RSTR_RNGRST                RCC_AHB2RSTR_RNGRST_Msk
#define RCC_AHB2RSTR_OTGFSRST_Pos          (7U)
#define RCC_AHB2RSTR_OTGFSRST_Msk          (0x1UL << RCC_AHB2RSTR_OTGFSRST_Pos) /*!< 0x00000080 */
#define RCC_AHB2RSTR_OTGFSRST              RCC_AHB2RSTR_OTGFSRST_Msk
/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define RCC_AHB3RSTR_FSMCRST_Pos           (0U)
#define RCC_AHB3RSTR_FSMCRST_Msk           (0x1UL << RCC_AHB3RSTR_FSMCRST_Pos)  /*!< 0x00000001 */
#define RCC_AHB3RSTR_FSMCRST               RCC_AHB3RSTR_FSMCRST_Msk


/********************  Bit definition for RCC_APB1RSTR register  **************/
#define RCC_APB1RSTR_TIM2RST_Pos           (0U)
#define RCC_APB1RSTR_TIM2RST_Msk           (0x1UL << RCC_APB1RSTR_TIM2RST_Pos)  /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST               RCC_APB1RSTR_TIM2RST_Msk
#define RCC_APB1RSTR_TIM3RST_Pos           (1U)
#define RCC_APB1RSTR_TIM3RST_Msk           (0x1UL << RCC_APB1RSTR_TIM3RST_Pos)  /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST               RCC_APB1RSTR_TIM3RST_Msk
#define RCC_APB1RSTR_TIM4RST_Pos           (2U)
#define RCC_APB1RSTR_TIM4RST_Msk           (0x1UL << RCC_APB1RSTR_TIM4RST_Pos)  /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST               RCC_APB1RSTR_TIM4RST_Msk
#define RCC_APB1RSTR_TIM5RST_Pos           (3U)
#define RCC_APB1RSTR_TIM5RST_Msk           (0x1UL << RCC_APB1RSTR_TIM5RST_Pos)  /*!< 0x00000008 */
#define RCC_APB1RSTR_TIM5RST               RCC_APB1RSTR_TIM5RST_Msk
#define RCC_APB1RSTR_TIM6RST_Pos           (4U)
#define RCC_APB1RSTR_TIM6RST_Msk           (0x1UL << RCC_APB1RSTR_TIM6RST_Pos)  /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST               RCC_APB1RSTR_TIM6RST_Msk
#define RCC_APB1RSTR_TIM7RST_Pos           (5U)
#define RCC_APB1RSTR_TIM7RST_Msk           (0x1UL << RCC_APB1RSTR_TIM7RST_Pos)  /*!< 0x00000020 */
#define RCC_APB1RSTR_TIM7RST               RCC_APB1RSTR_TIM7RST_Msk
#define RCC_APB1RSTR_TIM12RST_Pos          (6U)
#define RCC_APB1RSTR_TIM12RST_Msk          (0x1UL << RCC_APB1RSTR_TIM12RST_Pos) /*!< 0x00000040 */
#define RCC_APB1RSTR_TIM12RST              RCC_APB1RSTR_TIM12RST_Msk
#define RCC_APB1RSTR_TIM13RST_Pos          (7U)
#define RCC_APB1RSTR_TIM13RST_Msk          (0x1UL << RCC_APB1RSTR_TIM13RST_Pos) /*!< 0x00000080 */
#define RCC_APB1RSTR_TIM13RST              RCC_APB1RSTR_TIM13RST_Msk
#define RCC_APB1RSTR_TIM14RST_Pos          (8U)
#define RCC_APB1RSTR_TIM14RST_Msk          (0x1UL << RCC_APB1RSTR_TIM14RST_Pos) /*!< 0x00000100 */
#define RCC_APB1RSTR_TIM14RST              RCC_APB1RSTR_TIM14RST_Msk
#define RCC_APB1RSTR_WWDGRST_Pos           (11U)
#define RCC_APB1RSTR_WWDGRST_Msk           (0x1UL << RCC_APB1RSTR_WWDGRST_Pos)  /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST               RCC_APB1RSTR_WWDGRST_Msk
#define RCC_APB1RSTR_SPI2RST_Pos           (14U)
#define RCC_APB1RSTR_SPI2RST_Msk           (0x1UL << RCC_APB1RSTR_SPI2RST_Pos)  /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST               RCC_APB1RSTR_SPI2RST_Msk
#define RCC_APB1RSTR_SPI3RST_Pos           (15U)
#define RCC_APB1RSTR_SPI3RST_Msk           (0x1UL << RCC_APB1RSTR_SPI3RST_Pos)  /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST               RCC_APB1RSTR_SPI3RST_Msk
#define RCC_APB1RSTR_USART2RST_Pos         (17U)
#define RCC_APB1RSTR_USART2RST_Msk         (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST             RCC_APB1RSTR_USART2RST_Msk
#define RCC_APB1RSTR_USART3RST_Pos         (18U)
#define RCC_APB1RSTR_USART3RST_Msk         (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST             RCC_APB1RSTR_USART3RST_Msk
#define RCC_APB1RSTR_UART4RST_Pos          (19U)
#define RCC_APB1RSTR_UART4RST_Msk          (0x1UL << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */
#define RCC_APB1RSTR_UART4RST              RCC_APB1RSTR_UART4RST_Msk
#define RCC_APB1RSTR_UART5RST_Pos          (20U)
#define RCC_APB1RSTR_UART5RST_Msk          (0x1UL << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */
#define RCC_APB1RSTR_UART5RST              RCC_APB1RSTR_UART5RST_Msk
#define RCC_APB1RSTR_I2C1RST_Pos           (21U)
#define RCC_APB1RSTR_I2C1RST_Msk           (0x1UL << RCC_APB1RSTR_I2C1RST_Pos)  /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST               RCC_APB1RSTR_I2C1RST_Msk
#define RCC_APB1RSTR_I2C2RST_Pos           (22U)
#define RCC_APB1RSTR_I2C2RST_Msk           (0x1UL << RCC_APB1RSTR_I2C2RST_Pos)  /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST               RCC_APB1RSTR_I2C2RST_Msk
#define RCC_APB1RSTR_I2C3RST_Pos           (23U)
#define RCC_APB1RSTR_I2C3RST_Msk           (0x1UL << RCC_APB1RSTR_I2C3RST_Pos)  /*!< 0x00800000 */
#define RCC_APB1RSTR_I2C3RST               RCC_APB1RSTR_I2C3RST_Msk
#define RCC_APB1RSTR_CAN1RST_Pos           (25U)
#define RCC_APB1RSTR_CAN1RST_Msk           (0x1UL << RCC_APB1RSTR_CAN1RST_Pos)  /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST               RCC_APB1RSTR_CAN1RST_Msk
#define RCC_APB1RSTR_CAN2RST_Pos           (26U)
#define RCC_APB1RSTR_CAN2RST_Msk           (0x1UL << RCC_APB1RSTR_CAN2RST_Pos)  /*!< 0x04000000 */
#define RCC_APB1RSTR_CAN2RST               RCC_APB1RSTR_CAN2RST_Msk
#define RCC_APB1RSTR_PWRRST_Pos            (28U)
#define RCC_APB1RSTR_PWRRST_Msk            (0x1UL << RCC_APB1RSTR_PWRRST_Pos)   /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                RCC_APB1RSTR_PWRRST_Msk
#define RCC_APB1RSTR_DACRST_Pos            (29U)
#define RCC_APB1RSTR_DACRST_Msk            (0x1UL << RCC_APB1RSTR_DACRST_Pos)   /*!< 0x20000000 */
#define RCC_APB1RSTR_DACRST                RCC_APB1RSTR_DACRST_Msk

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APB2RSTR_TIM1RST_Pos           (0U)
#define RCC_APB2RSTR_TIM1RST_Msk           (0x1UL << RCC_APB2RSTR_TIM1RST_Pos)  /*!< 0x00000001 */
#define RCC_APB2RSTR_TIM1RST               RCC_APB2RSTR_TIM1RST_Msk
#define RCC_APB2RSTR_TIM8RST_Pos           (1U)
#define RCC_APB2RSTR_TIM8RST_Msk           (0x1UL << RCC_APB2RSTR_TIM8RST_Pos)  /*!< 0x00000002 */
#define RCC_APB2RSTR_TIM8RST               RCC_APB2RSTR_TIM8RST_Msk
#define RCC_APB2RSTR_USART1RST_Pos         (4U)
#define RCC_APB2RSTR_USART1RST_Msk         (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_USART1RST             RCC_APB2RSTR_USART1RST_Msk
#define RCC_APB2RSTR_USART6RST_Pos         (5U)
#define RCC_APB2RSTR_USART6RST_Msk         (0x1UL << RCC_APB2RSTR_USART6RST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_USART6RST             RCC_APB2RSTR_USART6RST_Msk
#define RCC_APB2RSTR_ADCRST_Pos            (8U)
#define RCC_APB2RSTR_ADCRST_Msk            (0x1UL << RCC_APB2RSTR_ADCRST_Pos)   /*!< 0x00000100 */
#define RCC_APB2RSTR_ADCRST                RCC_APB2RSTR_ADCRST_Msk
#define RCC_APB2RSTR_SDIORST_Pos           (11U)
#define RCC_APB2RSTR_SDIORST_Msk           (0x1UL << RCC_APB2RSTR_SDIORST_Pos)  /*!< 0x00000800 */
#define RCC_APB2RSTR_SDIORST               RCC_APB2RSTR_SDIORST_Msk
#define RCC_APB2RSTR_SPI1RST_Pos           (12U)
#define RCC_APB2RSTR_SPI1RST_Msk           (0x1UL << RCC_APB2RSTR_SPI1RST_Pos)  /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST               RCC_APB2RSTR_SPI1RST_Msk
#define RCC_APB2RSTR_SYSCFGRST_Pos         (14U)
#define RCC_APB2RSTR_SYSCFGRST_Msk         (0x1UL << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_SYSCFGRST             RCC_APB2RSTR_SYSCFGRST_Msk
#define RCC_APB2RSTR_TIM9RST_Pos           (16U)
#define RCC_APB2RSTR_TIM9RST_Msk           (0x1UL << RCC_APB2RSTR_TIM9RST_Pos)  /*!< 0x00010000 */
#define RCC_APB2RSTR_TIM9RST               RCC_APB2RSTR_TIM9RST_Msk
#define RCC_APB2RSTR_TIM10RST_Pos          (17U)
#define RCC_APB2RSTR_TIM10RST_Msk          (0x1UL << RCC_APB2RSTR_TIM10RST_Pos) /*!< 0x00020000 */
#define RCC_APB2RSTR_TIM10RST              RCC_APB2RSTR_TIM10RST_Msk
#define RCC_APB2RSTR_TIM11RST_Pos          (18U)
#define RCC_APB2RSTR_TIM11RST_Msk          (0x1UL << RCC_APB2RSTR_TIM11RST_Pos) /*!< 0x00040000 */
#define RCC_APB2RSTR_TIM11RST              RCC_APB2RSTR_TIM11RST_Msk

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN                RCC_AHB1ENR_GPIOAEN_Msk
#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)
#define RCC_AHB1ENR_GPIOBEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)   /*!< 0x00000002 */
#define RCC_AHB1ENR_GPIOBEN                RCC_AHB1ENR_GPIOBEN_Msk
#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)
#define RCC_AHB1ENR_GPIOCEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIOCEN                RCC_AHB1ENR_GPIOCEN_Msk
#define RCC_AHB1ENR_GPIODEN_Pos            (3U)
#define RCC_AHB1ENR_GPIODEN_Msk            (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIODEN                RCC_AHB1ENR_GPIODEN_Msk
#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)
#define RCC_AHB1ENR_GPIOEEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOEEN                RCC_AHB1ENR_GPIOEEN_Msk
#define RCC_AHB1ENR_GPIOFEN_Pos            (5U)
#define RCC_AHB1ENR_GPIOFEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOFEN_Pos)   /*!< 0x00000020 */
#define RCC_AHB1ENR_GPIOFEN                RCC_AHB1ENR_GPIOFEN_Msk
#define RCC_AHB1ENR_GPIOGEN_Pos            (6U)
#define RCC_AHB1ENR_GPIOGEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOGEN_Pos)   /*!< 0x00000040 */
#define RCC_AHB1ENR_GPIOGEN                RCC_AHB1ENR_GPIOGEN_Msk
#define RCC_AHB1ENR_GPIOHEN_Pos            (7U)
#define RCC_AHB1ENR_GPIOHEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOHEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB1ENR_GPIOHEN                RCC_AHB1ENR_GPIOHEN_Msk
#define RCC_AHB1ENR_GPIOIEN_Pos            (8U)
#define RCC_AHB1ENR_GPIOIEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOIEN_Pos)   /*!< 0x00000100 */
#define RCC_AHB1ENR_GPIOIEN                RCC_AHB1ENR_GPIOIEN_Msk
#define RCC_AHB1ENR_CRCEN_Pos              (12U)
#define RCC_AHB1ENR_CRCEN_Msk              (0x1UL << RCC_AHB1ENR_CRCEN_Pos)     /*!< 0x00001000 */
#define RCC_AHB1ENR_CRCEN                  RCC_AHB1ENR_CRCEN_Msk
#define RCC_AHB1ENR_BKPSRAMEN_Pos          (18U)
#define RCC_AHB1ENR_BKPSRAMEN_Msk          (0x1UL << RCC_AHB1ENR_BKPSRAMEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1ENR_BKPSRAMEN              RCC_AHB1ENR_BKPSRAMEN_Msk
#define RCC_AHB1ENR_CCMDATARAMEN_Pos       (20U)
#define RCC_AHB1ENR_CCMDATARAMEN_Msk       (0x1UL << RCC_AHB1ENR_CCMDATARAMEN_Pos) /*!< 0x00100000 */
#define RCC_AHB1ENR_CCMDATARAMEN           RCC_AHB1ENR_CCMDATARAMEN_Msk
#define RCC_AHB1ENR_DMA1EN_Pos             (21U)
#define RCC_AHB1ENR_DMA1EN_Msk             (0x1UL << RCC_AHB1ENR_DMA1EN_Pos)    /*!< 0x00200000 */
#define RCC_AHB1ENR_DMA1EN                 RCC_AHB1ENR_DMA1EN_Msk
#define RCC_AHB1ENR_DMA2EN_Pos             (22U)
#define RCC_AHB1ENR_DMA2EN_Msk             (0x1UL << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */
#define RCC_AHB1ENR_DMA2EN                 RCC_AHB1ENR_DMA2EN_Msk
#define RCC_AHB1ENR_ETHMACEN_Pos           (25U)
#define RCC_AHB1ENR_ETHMACEN_Msk           (0x1UL << RCC_AHB1ENR_ETHMACEN_Pos)  /*!< 0x02000000 */
#define RCC_AHB1ENR_ETHMACEN               RCC_AHB1ENR_ETHMACEN_Msk
#define RCC_AHB1ENR_ETHMACTXEN_Pos         (26U)
#define RCC_AHB1ENR_ETHMACTXEN_Msk         (0x1UL << RCC_AHB1ENR_ETHMACTXEN_Pos) /*!< 0x04000000 */
#define RCC_AHB1ENR_ETHMACTXEN             RCC_AHB1ENR_ETHMACTXEN_Msk
#define RCC_AHB1ENR_ETHMACRXEN_Pos         (27U)
#define RCC_AHB1ENR_ETHMACRXEN_Msk         (0x1UL << RCC_AHB1ENR_ETHMACRXEN_Pos) /*!< 0x08000000 */
#define RCC_AHB1ENR_ETHMACRXEN             RCC_AHB1ENR_ETHMACRXEN_Msk
#define RCC_AHB1ENR_ETHMACPTPEN_Pos        (28U)
#define RCC_AHB1ENR_ETHMACPTPEN_Msk        (0x1UL << RCC_AHB1ENR_ETHMACPTPEN_Pos) /*!< 0x10000000 */
#define RCC_AHB1ENR_ETHMACPTPEN            RCC_AHB1ENR_ETHMACPTPEN_Msk
#define RCC_AHB1ENR_OTGHSEN_Pos            (29U)
#define RCC_AHB1ENR_OTGHSEN_Msk            (0x1UL << RCC_AHB1ENR_OTGHSEN_Pos)   /*!< 0x20000000 */
#define RCC_AHB1ENR_OTGHSEN                RCC_AHB1ENR_OTGHSEN_Msk
#define RCC_AHB1ENR_OTGHSULPIEN_Pos        (30U)
#define RCC_AHB1ENR_OTGHSULPIEN_Msk        (0x1UL << RCC_AHB1ENR_OTGHSULPIEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1ENR_OTGHSULPIEN            RCC_AHB1ENR_OTGHSULPIEN_Msk
/********************  Bit definition for RCC_AHB2ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 series)
 */
#define RCC_AHB2_SUPPORT                   /*!< AHB2 Bus is supported */

#define RCC_AHB2ENR_DCMIEN_Pos             (0U)
#define RCC_AHB2ENR_DCMIEN_Msk             (0x1UL << RCC_AHB2ENR_DCMIEN_Pos)    /*!< 0x00000001 */
#define RCC_AHB2ENR_DCMIEN                 RCC_AHB2ENR_DCMIEN_Msk
#define RCC_AHB2ENR_RNGEN_Pos              (6U)
#define RCC_AHB2ENR_RNGEN_Msk              (0x1UL << RCC_AHB2ENR_RNGEN_Pos)     /*!< 0x00000040 */
#define RCC_AHB2ENR_RNGEN                  RCC_AHB2ENR_RNGEN_Msk
#define RCC_AHB2ENR_OTGFSEN_Pos            (7U)
#define RCC_AHB2ENR_OTGFSEN_Msk            (0x1UL << RCC_AHB2ENR_OTGFSEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB2ENR_OTGFSEN                RCC_AHB2ENR_OTGFSEN_Msk

/********************  Bit definition for RCC_AHB3ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 series)
 */
#define RCC_AHB3_SUPPORT                   /*!< AHB3 Bus is supported */

#define RCC_AHB3ENR_FSMCEN_Pos             (0U)
#define RCC_AHB3ENR_FSMCEN_Msk             (0x1UL << RCC_AHB3ENR_FSMCEN_Pos)    /*!< 0x00000001 */
#define RCC_AHB3ENR_FSMCEN                 RCC_AHB3ENR_FSMCEN_Msk

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos             (0U)
#define RCC_APB1ENR_TIM2EN_Msk             (0x1UL << RCC_APB1ENR_TIM2EN_Pos)    /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                 RCC_APB1ENR_TIM2EN_Msk
#define RCC_APB1ENR_TIM3EN_Pos             (1U)
#define RCC_APB1ENR_TIM3EN_Msk             (0x1UL << RCC_APB1ENR_TIM3EN_Pos)    /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                 RCC_APB1ENR_TIM3EN_Msk
#define RCC_APB1ENR_TIM4EN_Pos             (2U)
#define RCC_APB1ENR_TIM4EN_Msk             (0x1UL << RCC_APB1ENR_TIM4EN_Pos)    /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN                 RCC_APB1ENR_TIM4EN_Msk
#define RCC_APB1ENR_TIM5EN_Pos             (3U)
#define RCC_APB1ENR_TIM5EN_Msk             (0x1UL << RCC_APB1ENR_TIM5EN_Pos)    /*!< 0x00000008 */
#define RCC_APB1ENR_TIM5EN                 RCC_APB1ENR_TIM5EN_Msk
#define RCC_APB1ENR_TIM6EN_Pos             (4U)
#define RCC_APB1ENR_TIM6EN_Msk             (0x1UL << RCC_APB1ENR_TIM6EN_Pos)    /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                 RCC_APB1ENR_TIM6EN_Msk
#define RCC_APB1ENR_TIM7EN_Pos             (5U)
#define RCC_APB1ENR_TIM7EN_Msk             (0x1UL << RCC_APB1ENR_TIM7EN_Pos)    /*!< 0x00000020 */
#define RCC_APB1ENR_TIM7EN                 RCC_APB1ENR_TIM7EN_Msk
#define RCC_APB1ENR_TIM12EN_Pos            (6U)
#define RCC_APB1ENR_TIM12EN_Msk            (0x1UL << RCC_APB1ENR_TIM12EN_Pos)   /*!< 0x00000040 */
#define RCC_APB1ENR_TIM12EN                RCC_APB1ENR_TIM12EN_Msk
#define RCC_APB1ENR_TIM13EN_Pos            (7U)
#define RCC_APB1ENR_TIM13EN_Msk            (0x1UL << RCC_APB1ENR_TIM13EN_Pos)   /*!< 0x00000080 */
#define RCC_APB1ENR_TIM13EN                RCC_APB1ENR_TIM13EN_Msk
#define RCC_APB1ENR_TIM14EN_Pos            (8U)
#define RCC_APB1ENR_TIM14EN_Msk            (0x1UL << RCC_APB1ENR_TIM14EN_Pos)   /*!< 0x00000100 */
#define RCC_APB1ENR_TIM14EN                RCC_APB1ENR_TIM14EN_Msk
#define RCC_APB1ENR_WWDGEN_Pos             (11U)
#define RCC_APB1ENR_WWDGEN_Msk             (0x1UL << RCC_APB1ENR_WWDGEN_Pos)    /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                 RCC_APB1ENR_WWDGEN_Msk
#define RCC_APB1ENR_SPI2EN_Pos             (14U)
#define RCC_APB1ENR_SPI2EN_Msk             (0x1UL << RCC_APB1ENR_SPI2EN_Pos)    /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                 RCC_APB1ENR_SPI2EN_Msk
#define RCC_APB1ENR_SPI3EN_Pos             (15U)
#define RCC_APB1ENR_SPI3EN_Msk             (0x1UL << RCC_APB1ENR_SPI3EN_Pos)    /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN                 RCC_APB1ENR_SPI3EN_Msk
#define RCC_APB1ENR_USART2EN_Pos           (17U)
#define RCC_APB1ENR_USART2EN_Msk           (0x1UL << RCC_APB1ENR_USART2EN_Pos)  /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN               RCC_APB1ENR_USART2EN_Msk
#define RCC_APB1ENR_USART3EN_Pos           (18U)
#define RCC_APB1ENR_USART3EN_Msk           (0x1UL << RCC_APB1ENR_USART3EN_Pos)  /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN               RCC_APB1ENR_USART3EN_Msk
#define RCC_APB1ENR_UART4EN_Pos            (19U)
#define RCC_APB1ENR_UART4EN_Msk            (0x1UL << RCC_APB1ENR_UART4EN_Pos)   /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN                RCC_APB1ENR_UART4EN_Msk
#define RCC_APB1ENR_UART5EN_Pos            (20U)
#define RCC_APB1ENR_UART5EN_Msk            (0x1UL << RCC_APB1ENR_UART5EN_Pos)   /*!< 0x00100000 */
#define RCC_APB1ENR_UART5EN                RCC_APB1ENR_UART5EN_Msk
#define RCC_APB1ENR_I2C1EN_Pos             (21U)
#define RCC_APB1ENR_I2C1EN_Msk             (0x1UL << RCC_APB1ENR_I2C1EN_Pos)    /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                 RCC_APB1ENR_I2C1EN_Msk
#define RCC_APB1ENR_I2C2EN_Pos             (22U)
#define RCC_APB1ENR_I2C2EN_Msk             (0x1UL << RCC_APB1ENR_I2C2EN_Pos)    /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                 RCC_APB1ENR_I2C2EN_Msk
#define RCC_APB1ENR_I2C3EN_Pos             (23U)
#define RCC_APB1ENR_I2C3EN_Msk             (0x1UL << RCC_APB1ENR_I2C3EN_Pos)    /*!< 0x00800000 */
#define RCC_APB1ENR_I2C3EN                 RCC_APB1ENR_I2C3EN_Msk
#define RCC_APB1ENR_CAN1EN_Pos             (25U)
#define RCC_APB1ENR_CAN1EN_Msk             (0x1UL << RCC_APB1ENR_CAN1EN_Pos)    /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN                 RCC_APB1ENR_CAN1EN_Msk
#define RCC_APB1ENR_CAN2EN_Pos             (26U)
#define RCC_APB1ENR_CAN2EN_Msk             (0x1UL << RCC_APB1ENR_CAN2EN_Pos)    /*!< 0x04000000 */
#define RCC_APB1ENR_CAN2EN                 RCC_APB1ENR_CAN2EN_Msk
#define RCC_APB1ENR_PWREN_Pos              (28U)
#define RCC_APB1ENR_PWREN_Msk              (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk
#define RCC_APB1ENR_DACEN_Pos              (29U)
#define RCC_APB1ENR_DACEN_Msk              (0x1UL << RCC_APB1ENR_DACEN_Pos)     /*!< 0x20000000 */
#define RCC_APB1ENR_DACEN                  RCC_APB1ENR_DACEN_Msk

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos             (0U)
#define RCC_APB2ENR_TIM1EN_Msk             (0x1UL << RCC_APB2ENR_TIM1EN_Pos)    /*!< 0x00000001 */
#define RCC_APB2ENR_TIM1EN                 RCC_APB2ENR_TIM1EN_Msk
#define RCC_APB2ENR_TIM8EN_Pos             (1U)
#define RCC_APB2ENR_TIM8EN_Msk             (0x1UL << RCC_APB2ENR_TIM8EN_Pos)    /*!< 0x00000002 */
#define RCC_APB2ENR_TIM8EN                 RCC_APB2ENR_TIM8EN_Msk
#define RCC_APB2ENR_USART1EN_Pos           (4U)
#define RCC_APB2ENR_USART1EN_Msk           (0x1UL << RCC_APB2ENR_USART1EN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_USART1EN               RCC_APB2ENR_USART1EN_Msk
#define RCC_APB2ENR_USART6EN_Pos           (5U)
#define RCC_APB2ENR_USART6EN_Msk           (0x1UL << RCC_APB2ENR_USART6EN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_USART6EN               RCC_APB2ENR_USART6EN_Msk
#define RCC_APB2ENR_ADC1EN_Pos             (8U)
#define RCC_APB2ENR_ADC1EN_Msk             (0x1UL << RCC_APB2ENR_ADC1EN_Pos)    /*!< 0x00000100 */
#define RCC_APB2ENR_ADC1EN                 RCC_APB2ENR_ADC1EN_Msk
#define RCC_APB2ENR_ADC2EN_Pos             (9U)
#define RCC_APB2ENR_ADC2EN_Msk             (0x1UL << RCC_APB2ENR_ADC2EN_Pos)    /*!< 0x00000200 */
#define RCC_APB2ENR_ADC2EN                 RCC_APB2ENR_ADC2EN_Msk
#define RCC_APB2ENR_ADC3EN_Pos             (10U)
#define RCC_APB2ENR_ADC3EN_Msk             (0x1UL << RCC_APB2ENR_ADC3EN_Pos)    /*!< 0x00000400 */
#define RCC_APB2ENR_ADC3EN                 RCC_APB2ENR_ADC3EN_Msk
#define RCC_APB2ENR_SDIOEN_Pos             (11U)
#define RCC_APB2ENR_SDIOEN_Msk             (0x1UL << RCC_APB2ENR_SDIOEN_Pos)    /*!< 0x00000800 */
#define RCC_APB2ENR_SDIOEN                 RCC_APB2ENR_SDIOEN_Msk
#define RCC_APB2ENR_SPI1EN_Pos             (12U)
#define RCC_APB2ENR_SPI1EN_Msk             (0x1UL << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                 RCC_APB2ENR_SPI1EN_Msk
#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)
#define RCC_APB2ENR_SYSCFGEN_Msk           (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */
#define RCC_APB2ENR_SYSCFGEN               RCC_APB2ENR_SYSCFGEN_Msk
#define RCC_APB2ENR_TIM9EN_Pos             (16U)
#define RCC_APB2ENR_TIM9EN_Msk             (0x1UL << RCC_APB2ENR_TIM9EN_Pos)    /*!< 0x00010000 */
#define RCC_APB2ENR_TIM9EN                 RCC_APB2ENR_TIM9EN_Msk
#define RCC_APB2ENR_TIM10EN_Pos            (17U)
#define RCC_APB2ENR_TIM10EN_Msk            (0x1UL << RCC_APB2ENR_TIM10EN_Pos)   /*!< 0x00020000 */
#define RCC_APB2ENR_TIM10EN                RCC_APB2ENR_TIM10EN_Msk
#define RCC_APB2ENR_TIM11EN_Pos            (18U)
#define RCC_APB2ENR_TIM11EN_Msk            (0x1UL << RCC_APB2ENR_TIM11EN_Pos)   /*!< 0x00040000 */
#define RCC_APB2ENR_TIM11EN                RCC_APB2ENR_TIM11EN_Msk

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos        (0U)
#define RCC_AHB1LPENR_GPIOALPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1LPENR_GPIOALPEN            RCC_AHB1LPENR_GPIOALPEN_Msk
#define RCC_AHB1LPENR_GPIOBLPEN_Pos        (1U)
#define RCC_AHB1LPENR_GPIOBLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1LPENR_GPIOBLPEN            RCC_AHB1LPENR_GPIOBLPEN_Msk
#define RCC_AHB1LPENR_GPIOCLPEN_Pos        (2U)
#define RCC_AHB1LPENR_GPIOCLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1LPENR_GPIOCLPEN            RCC_AHB1LPENR_GPIOCLPEN_Msk
#define RCC_AHB1LPENR_GPIODLPEN_Pos        (3U)
#define RCC_AHB1LPENR_GPIODLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1LPENR_GPIODLPEN            RCC_AHB1LPENR_GPIODLPEN_Msk
#define RCC_AHB1LPENR_GPIOELPEN_Pos        (4U)
#define RCC_AHB1LPENR_GPIOELPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1LPENR_GPIOELPEN            RCC_AHB1LPENR_GPIOELPEN_Msk
#define RCC_AHB1LPENR_GPIOFLPEN_Pos        (5U)
#define RCC_AHB1LPENR_GPIOFLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOFLPEN_Pos) /*!< 0x00000020 */
#define RCC_AHB1LPENR_GPIOFLPEN            RCC_AHB1LPENR_GPIOFLPEN_Msk
#define RCC_AHB1LPENR_GPIOGLPEN_Pos        (6U)
#define RCC_AHB1LPENR_GPIOGLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB1LPENR_GPIOGLPEN            RCC_AHB1LPENR_GPIOGLPEN_Msk
#define RCC_AHB1LPENR_GPIOHLPEN_Pos        (7U)
#define RCC_AHB1LPENR_GPIOHLPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1LPENR_GPIOHLPEN            RCC_AHB1LPENR_GPIOHLPEN_Msk
#define RCC_AHB1LPENR_GPIOILPEN_Pos        (8U)
#define RCC_AHB1LPENR_GPIOILPEN_Msk        (0x1UL << RCC_AHB1LPENR_GPIOILPEN_Pos) /*!< 0x00000100 */
#define RCC_AHB1LPENR_GPIOILPEN            RCC_AHB1LPENR_GPIOILPEN_Msk
#define RCC_AHB1LPENR_CRCLPEN_Pos          (12U)
#define RCC_AHB1LPENR_CRCLPEN_Msk          (0x1UL << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1LPENR_CRCLPEN              RCC_AHB1LPENR_CRCLPEN_Msk
#define RCC_AHB1LPENR_FLITFLPEN_Pos        (15U)
#define RCC_AHB1LPENR_FLITFLPEN_Msk        (0x1UL << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */
#define RCC_AHB1LPENR_FLITFLPEN            RCC_AHB1LPENR_FLITFLPEN_Msk
#define RCC_AHB1LPENR_SRAM1LPEN_Pos        (16U)
#define RCC_AHB1LPENR_SRAM1LPEN_Msk        (0x1UL << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */
#define RCC_AHB1LPENR_SRAM1LPEN            RCC_AHB1LPENR_SRAM1LPEN_Msk
#define RCC_AHB1LPENR_SRAM2LPEN_Pos        (17U)
#define RCC_AHB1LPENR_SRAM2LPEN_Msk        (0x1UL << RCC_AHB1LPENR_SRAM2LPEN_Pos) /*!< 0x00020000 */
#define RCC_AHB1LPENR_SRAM2LPEN            RCC_AHB1LPENR_SRAM2LPEN_Msk
#define RCC_AHB1LPENR_BKPSRAMLPEN_Pos      (18U)
#define RCC_AHB1LPENR_BKPSRAMLPEN_Msk      (0x1UL << RCC_AHB1LPENR_BKPSRAMLPEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1LPENR_BKPSRAMLPEN          RCC_AHB1LPENR_BKPSRAMLPEN_Msk
#define RCC_AHB1LPENR_DMA1LPEN_Pos         (21U)
#define RCC_AHB1LPENR_DMA1LPEN_Msk         (0x1UL << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */
#define RCC_AHB1LPENR_DMA1LPEN             RCC_AHB1LPENR_DMA1LPEN_Msk
#define RCC_AHB1LPENR_DMA2LPEN_Pos         (22U)
#define RCC_AHB1LPENR_DMA2LPEN_Msk         (0x1UL << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */
#define RCC_AHB1LPENR_DMA2LPEN             RCC_AHB1LPENR_DMA2LPEN_Msk

#define RCC_AHB1LPENR_ETHMACLPEN_Pos       (25U)
#define RCC_AHB1LPENR_ETHMACLPEN_Msk       (0x1UL << RCC_AHB1LPENR_ETHMACLPEN_Pos) /*!< 0x02000000 */
#define RCC_AHB1LPENR_ETHMACLPEN           RCC_AHB1LPENR_ETHMACLPEN_Msk
#define RCC_AHB1LPENR_ETHMACTXLPEN_Pos     (26U)
#define RCC_AHB1LPENR_ETHMACTXLPEN_Msk     (0x1UL << RCC_AHB1LPENR_ETHMACTXLPEN_Pos) /*!< 0x04000000 */
#define RCC_AHB1LPENR_ETHMACTXLPEN         RCC_AHB1LPENR_ETHMACTXLPEN_Msk
#define RCC_AHB1LPENR_ETHMACRXLPEN_Pos     (27U)
#define RCC_AHB1LPENR_ETHMACRXLPEN_Msk     (0x1UL << RCC_AHB1LPENR_ETHMACRXLPEN_Pos) /*!< 0x08000000 */
#define RCC_AHB1LPENR_ETHMACRXLPEN         RCC_AHB1LPENR_ETHMACRXLPEN_Msk
#define RCC_AHB1LPENR_ETHMACPTPLPEN_Pos    (28U)
#define RCC_AHB1LPENR_ETHMACPTPLPEN_Msk    (0x1UL << RCC_AHB1LPENR_ETHMACPTPLPEN_Pos) /*!< 0x10000000 */
#define RCC_AHB1LPENR_ETHMACPTPLPEN        RCC_AHB1LPENR_ETHMACPTPLPEN_Msk
#define RCC_AHB1LPENR_OTGHSLPEN_Pos        (29U)
#define RCC_AHB1LPENR_OTGHSLPEN_Msk        (0x1UL << RCC_AHB1LPENR_OTGHSLPEN_Pos) /*!< 0x20000000 */
#define RCC_AHB1LPENR_OTGHSLPEN            RCC_AHB1LPENR_OTGHSLPEN_Msk
#define RCC_AHB1LPENR_OTGHSULPILPEN_Pos    (30U)
#define RCC_AHB1LPENR_OTGHSULPILPEN_Msk    (0x1UL << RCC_AHB1LPENR_OTGHSULPILPEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1LPENR_OTGHSULPILPEN        RCC_AHB1LPENR_OTGHSULPILPEN_Msk

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_DCMILPEN_Pos         (0U)
#define RCC_AHB2LPENR_DCMILPEN_Msk         (0x1UL << RCC_AHB2LPENR_DCMILPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB2LPENR_DCMILPEN             RCC_AHB2LPENR_DCMILPEN_Msk
#define RCC_AHB2LPENR_RNGLPEN_Pos          (6U)
#define RCC_AHB2LPENR_RNGLPEN_Msk          (0x1UL << RCC_AHB2LPENR_RNGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB2LPENR_RNGLPEN              RCC_AHB2LPENR_RNGLPEN_Msk
#define RCC_AHB2LPENR_OTGFSLPEN_Pos        (7U)
#define RCC_AHB2LPENR_OTGFSLPEN_Msk        (0x1UL << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB2LPENR_OTGFSLPEN            RCC_AHB2LPENR_OTGFSLPEN_Msk

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define RCC_AHB3LPENR_FSMCLPEN_Pos         (0U)
#define RCC_AHB3LPENR_FSMCLPEN_Msk         (0x1UL << RCC_AHB3LPENR_FSMCLPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB3LPENR_FSMCLPEN             RCC_AHB3LPENR_FSMCLPEN_Msk

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos         (0U)
#define RCC_APB1LPENR_TIM2LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB1LPENR_TIM2LPEN             RCC_APB1LPENR_TIM2LPEN_Msk
#define RCC_APB1LPENR_TIM3LPEN_Pos         (1U)
#define RCC_APB1LPENR_TIM3LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB1LPENR_TIM3LPEN             RCC_APB1LPENR_TIM3LPEN_Msk
#define RCC_APB1LPENR_TIM4LPEN_Pos         (2U)
#define RCC_APB1LPENR_TIM4LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */
#define RCC_APB1LPENR_TIM4LPEN             RCC_APB1LPENR_TIM4LPEN_Msk
#define RCC_APB1LPENR_TIM5LPEN_Pos         (3U)
#define RCC_APB1LPENR_TIM5LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */
#define RCC_APB1LPENR_TIM5LPEN             RCC_APB1LPENR_TIM5LPEN_Msk
#define RCC_APB1LPENR_TIM6LPEN_Pos         (4U)
#define RCC_APB1LPENR_TIM6LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM6LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB1LPENR_TIM6LPEN             RCC_APB1LPENR_TIM6LPEN_Msk
#define RCC_APB1LPENR_TIM7LPEN_Pos         (5U)
#define RCC_APB1LPENR_TIM7LPEN_Msk         (0x1UL << RCC_APB1LPENR_TIM7LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB1LPENR_TIM7LPEN             RCC_APB1LPENR_TIM7LPEN_Msk
#define RCC_APB1LPENR_TIM12LPEN_Pos        (6U)
#define RCC_APB1LPENR_TIM12LPEN_Msk        (0x1UL << RCC_APB1LPENR_TIM12LPEN_Pos) /*!< 0x00000040 */
#define RCC_APB1LPENR_TIM12LPEN            RCC_APB1LPENR_TIM12LPEN_Msk
#define RCC_APB1LPENR_TIM13LPEN_Pos        (7U)
#define RCC_APB1LPENR_TIM13LPEN_Msk        (0x1UL << RCC_APB1LPENR_TIM13LPEN_Pos) /*!< 0x00000080 */
#define RCC_APB1LPENR_TIM13LPEN            RCC_APB1LPENR_TIM13LPEN_Msk
#define RCC_APB1LPENR_TIM14LPEN_Pos        (8U)
#define RCC_APB1LPENR_TIM14LPEN_Msk        (0x1UL << RCC_APB1LPENR_TIM14LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB1LPENR_TIM14LPEN            RCC_APB1LPENR_TIM14LPEN_Msk
#define RCC_APB1LPENR_WWDGLPEN_Pos         (11U)
#define RCC_APB1LPENR_WWDGLPEN_Msk         (0x1UL << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB1LPENR_WWDGLPEN             RCC_APB1LPENR_WWDGLPEN_Msk
#define RCC_APB1LPENR_SPI2LPEN_Pos         (14U)
#define RCC_APB1LPENR_SPI2LPEN_Msk         (0x1UL << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */
#define RCC_APB1LPENR_SPI2LPEN             RCC_APB1LPENR_SPI2LPEN_Msk
#define RCC_APB1LPENR_SPI3LPEN_Pos         (15U)
#define RCC_APB1LPENR_SPI3LPEN_Msk         (0x1UL << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */
#define RCC_APB1LPENR_SPI3LPEN             RCC_APB1LPENR_SPI3LPEN_Msk
#define RCC_APB1LPENR_USART2LPEN_Pos       (17U)
#define RCC_APB1LPENR_USART2LPEN_Msk       (0x1UL << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB1LPENR_USART2LPEN           RCC_APB1LPENR_USART2LPEN_Msk
#define RCC_APB1LPENR_USART3LPEN_Pos       (18U)
#define RCC_APB1LPENR_USART3LPEN_Msk       (0x1UL << RCC_APB1LPENR_USART3LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB1LPENR_USART3LPEN           RCC_APB1LPENR_USART3LPEN_Msk
#define RCC_APB1LPENR_UART4LPEN_Pos        (19U)
#define RCC_APB1LPENR_UART4LPEN_Msk        (0x1UL << RCC_APB1LPENR_UART4LPEN_Pos) /*!< 0x00080000 */
#define RCC_APB1LPENR_UART4LPEN            RCC_APB1LPENR_UART4LPEN_Msk
#define RCC_APB1LPENR_UART5LPEN_Pos        (20U)
#define RCC_APB1LPENR_UART5LPEN_Msk        (0x1UL << RCC_APB1LPENR_UART5LPEN_Pos) /*!< 0x00100000 */
#define RCC_APB1LPENR_UART5LPEN            RCC_APB1LPENR_UART5LPEN_Msk
#define RCC_APB1LPENR_I2C1LPEN_Pos         (21U)
#define RCC_APB1LPENR_I2C1LPEN_Msk         (0x1UL << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB1LPENR_I2C1LPEN             RCC_APB1LPENR_I2C1LPEN_Msk
#define RCC_APB1LPENR_I2C2LPEN_Pos         (22U)
#define RCC_APB1LPENR_I2C2LPEN_Msk         (0x1UL << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB1LPENR_I2C2LPEN             RCC_APB1LPENR_I2C2LPEN_Msk
#define RCC_APB1LPENR_I2C3LPEN_Pos         (23U)
#define RCC_APB1LPENR_I2C3LPEN_Msk         (0x1UL << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB1LPENR_I2C3LPEN             RCC_APB1LPENR_I2C3LPEN_Msk
#define RCC_APB1LPENR_CAN1LPEN_Pos         (25U)
#define RCC_APB1LPENR_CAN1LPEN_Msk         (0x1UL << RCC_APB1LPENR_CAN1LPEN_Pos) /*!< 0x02000000 */
#define RCC_APB1LPENR_CAN1LPEN             RCC_APB1LPENR_CAN1LPEN_Msk
#define RCC_APB1LPENR_CAN2LPEN_Pos         (26U)
#define RCC_APB1LPENR_CAN2LPEN_Msk         (0x1UL << RCC_APB1LPENR_CAN2LPEN_Pos) /*!< 0x04000000 */
#define RCC_APB1LPENR_CAN2LPEN             RCC_APB1LPENR_CAN2LPEN_Msk
#define RCC_APB1LPENR_PWRLPEN_Pos          (28U)
#define RCC_APB1LPENR_PWRLPEN_Msk          (0x1UL << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */
#define RCC_APB1LPENR_PWRLPEN              RCC_APB1LPENR_PWRLPEN_Msk
#define RCC_APB1LPENR_DACLPEN_Pos          (29U)
#define RCC_APB1LPENR_DACLPEN_Msk          (0x1UL << RCC_APB1LPENR_DACLPEN_Pos) /*!< 0x20000000 */
#define RCC_APB1LPENR_DACLPEN              RCC_APB1LPENR_DACLPEN_Msk

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos         (0U)
#define RCC_APB2LPENR_TIM1LPEN_Msk         (0x1UL << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2LPENR_TIM1LPEN             RCC_APB2LPENR_TIM1LPEN_Msk
#define RCC_APB2LPENR_TIM8LPEN_Pos         (1U)
#define RCC_APB2LPENR_TIM8LPEN_Msk         (0x1UL << RCC_APB2LPENR_TIM8LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB2LPENR_TIM8LPEN             RCC_APB2LPENR_TIM8LPEN_Msk
#define RCC_APB2LPENR_USART1LPEN_Pos       (4U)
#define RCC_APB2LPENR_USART1LPEN_Msk       (0x1UL << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB2LPENR_USART1LPEN           RCC_APB2LPENR_USART1LPEN_Msk
#define RCC_APB2LPENR_USART6LPEN_Pos       (5U)
#define RCC_APB2LPENR_USART6LPEN_Msk       (0x1UL << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB2LPENR_USART6LPEN           RCC_APB2LPENR_USART6LPEN_Msk
#define RCC_APB2LPENR_ADC1LPEN_Pos         (8U)
#define RCC_APB2LPENR_ADC1LPEN_Msk         (0x1UL << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB2LPENR_ADC1LPEN             RCC_APB2LPENR_ADC1LPEN_Msk
#define RCC_APB2LPENR_ADC2LPEN_Pos         (9U)
#define RCC_APB2LPENR_ADC2LPEN_Msk         (0x1UL << RCC_APB2LPENR_ADC2LPEN_Pos) /*!< 0x00000200 */
#define RCC_APB2LPENR_ADC2LPEN             RCC_APB2LPENR_ADC2LPEN_Msk
#define RCC_APB2LPENR_ADC3LPEN_Pos         (10U)
#define RCC_APB2LPENR_ADC3LPEN_Msk         (0x1UL << RCC_APB2LPENR_ADC3LPEN_Pos) /*!< 0x00000400 */
#define RCC_APB2LPENR_ADC3LPEN             RCC_APB2LPENR_ADC3LPEN_Msk
#define RCC_APB2LPENR_SDIOLPEN_Pos         (11U)
#define RCC_APB2LPENR_SDIOLPEN_Msk         (0x1UL << RCC_APB2LPENR_SDIOLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB2LPENR_SDIOLPEN             RCC_APB2LPENR_SDIOLPEN_Msk
#define RCC_APB2LPENR_SPI1LPEN_Pos         (12U)
#define RCC_APB2LPENR_SPI1LPEN_Msk         (0x1UL << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */
#define RCC_APB2LPENR_SPI1LPEN             RCC_APB2LPENR_SPI1LPEN_Msk
#define RCC_APB2LPENR_SYSCFGLPEN_Pos       (14U)
#define RCC_APB2LPENR_SYSCFGLPEN_Msk       (0x1UL << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */
#define RCC_APB2LPENR_SYSCFGLPEN           RCC_APB2LPENR_SYSCFGLPEN_Msk
#define RCC_APB2LPENR_TIM9LPEN_Pos         (16U)
#define RCC_APB2LPENR_TIM9LPEN_Msk         (0x1UL << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */
#define RCC_APB2LPENR_TIM9LPEN             RCC_APB2LPENR_TIM9LPEN_Msk
#define RCC_APB2LPENR_TIM10LPEN_Pos        (17U)
#define RCC_APB2LPENR_TIM10LPEN_Msk        (0x1UL << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB2LPENR_TIM10LPEN            RCC_APB2LPENR_TIM10LPEN_Msk
#define RCC_APB2LPENR_TIM11LPEN_Pos        (18U)
#define RCC_APB2LPENR_TIM11LPEN_Msk        (0x1UL << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB2LPENR_TIM11LPEN            RCC_APB2LPENR_TIM11LPEN_Msk

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos                 (0U)
#define RCC_BDCR_LSEON_Msk                 (0x1UL << RCC_BDCR_LSEON_Pos)        /*!< 0x00000001 */
#define RCC_BDCR_LSEON                     RCC_BDCR_LSEON_Msk
#define RCC_BDCR_LSERDY_Pos                (1U)
#define RCC_BDCR_LSERDY_Msk                (0x1UL << RCC_BDCR_LSERDY_Pos)       /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                    RCC_BDCR_LSERDY_Msk
#define RCC_BDCR_LSEBYP_Pos                (2U)
#define RCC_BDCR_LSEBYP_Msk                (0x1UL << RCC_BDCR_LSEBYP_Pos)       /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                    RCC_BDCR_LSEBYP_Msk

#define RCC_BDCR_RTCSEL_Pos                (8U)
#define RCC_BDCR_RTCSEL_Msk                (0x3UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                    RCC_BDCR_RTCSEL_Msk
#define RCC_BDCR_RTCSEL_0                  (0x1UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                  (0x2UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000200 */

#define RCC_BDCR_RTCEN_Pos                 (15U)
#define RCC_BDCR_RTCEN_Msk                 (0x1UL << RCC_BDCR_RTCEN_Pos)        /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                     RCC_BDCR_RTCEN_Msk
#define RCC_BDCR_BDRST_Pos                 (16U)
#define RCC_BDCR_BDRST_Msk                 (0x1UL << RCC_BDCR_BDRST_Pos)        /*!< 0x00010000 */
#define RCC_BDCR_BDRST                     RCC_BDCR_BDRST_Msk

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                  (0U)
#define RCC_CSR_LSION_Msk                  (0x1UL << RCC_CSR_LSION_Pos)         /*!< 0x00000001 */
#define RCC_CSR_LSION                      RCC_CSR_LSION_Msk
#define RCC_CSR_LSIRDY_Pos                 (1U)
#define RCC_CSR_LSIRDY_Msk                 (0x1UL << RCC_CSR_LSIRDY_Pos)        /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                     RCC_CSR_LSIRDY_Msk
#define RCC_CSR_RMVF_Pos                   (24U)
#define RCC_CSR_RMVF_Msk                   (0x1UL << RCC_CSR_RMVF_Pos)          /*!< 0x01000000 */
#define RCC_CSR_RMVF                       RCC_CSR_RMVF_Msk
#define RCC_CSR_BORRSTF_Pos                (25U)
#define RCC_CSR_BORRSTF_Msk                (0x1UL << RCC_CSR_BORRSTF_Pos)       /*!< 0x02000000 */
#define RCC_CSR_BORRSTF                    RCC_CSR_BORRSTF_Msk
#define RCC_CSR_PINRSTF_Pos                (26U)
#define RCC_CSR_PINRSTF_Msk                (0x1UL << RCC_CSR_PINRSTF_Pos)       /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                    RCC_CSR_PINRSTF_Msk
#define RCC_CSR_PORRSTF_Pos                (27U)
#define RCC_CSR_PORRSTF_Msk                (0x1UL << RCC_CSR_PORRSTF_Pos)       /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                    RCC_CSR_PORRSTF_Msk
#define RCC_CSR_SFTRSTF_Pos                (28U)
#define RCC_CSR_SFTRSTF_Msk                (0x1UL << RCC_CSR_SFTRSTF_Pos)       /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                    RCC_CSR_SFTRSTF_Msk
#define RCC_CSR_IWDGRSTF_Pos               (29U)
#define RCC_CSR_IWDGRSTF_Msk               (0x1UL << RCC_CSR_IWDGRSTF_Pos)      /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                   RCC_CSR_IWDGRSTF_Msk
#define RCC_CSR_WWDGRSTF_Pos               (30U)
#define RCC_CSR_WWDGRSTF_Msk               (0x1UL << RCC_CSR_WWDGRSTF_Pos)      /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                   RCC_CSR_WWDGRSTF_Msk
#define RCC_CSR_LPWRRSTF_Pos               (31U)
#define RCC_CSR_LPWRRSTF_Msk               (0x1UL << RCC_CSR_LPWRRSTF_Pos)      /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                   RCC_CSR_LPWRRSTF_Msk
/* Legacy defines */
#define RCC_CSR_PADRSTF                    RCC_CSR_PINRSTF
#define RCC_CSR_WDGRSTF                    RCC_CSR_IWDGRSTF

/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos               (0U)
#define RCC_SSCGR_MODPER_Msk               (0x1FFFUL << RCC_SSCGR_MODPER_Pos)   /*!< 0x00001FFF */
#define RCC_SSCGR_MODPER                   RCC_SSCGR_MODPER_Msk
#define RCC_SSCGR_INCSTEP_Pos              (13U)
#define RCC_SSCGR_INCSTEP_Msk              (0x7FFFUL << RCC_SSCGR_INCSTEP_Pos)  /*!< 0x0FFFE000 */
#define RCC_SSCGR_INCSTEP                  RCC_SSCGR_INCSTEP_Msk
#define RCC_SSCGR_SPREADSEL_Pos            (30U)
#define RCC_SSCGR_SPREADSEL_Msk            (0x1UL << RCC_SSCGR_SPREADSEL_Pos)   /*!< 0x40000000 */
#define RCC_SSCGR_SPREADSEL                RCC_SSCGR_SPREADSEL_Msk
#define RCC_SSCGR_SSCGEN_Pos               (31U)
#define RCC_SSCGR_SSCGEN_Msk               (0x1UL << RCC_SSCGR_SSCGEN_Pos)      /*!< 0x80000000 */
#define RCC_SSCGR_SSCGEN                   RCC_SSCGR_SSCGEN_Msk

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SR_Pos         (28U)
#define RCC_PLLI2SCFGR_PLLI2SR_Msk         (0x7UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR             RCC_PLLI2SCFGR_PLLI2SR_Msk
#define RCC_PLLI2SCFGR_PLLI2SR_0           (0x1UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x10000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_1           (0x2UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x20000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_2           (0x4UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x40000000 */

#define RCC_PLLI2SCFGR_PLLI2SN_Pos         (6U)
#define RCC_PLLI2SCFGR_PLLI2SN_Msk         (0x1FFUL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN             RCC_PLLI2SCFGR_PLLI2SN_Msk
#define RCC_PLLI2SCFGR_PLLI2SN_0           (0x001UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000040 */
#define RCC_PLLI2SCFGR_PLLI2SN_1           (0x002UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000080 */
#define RCC_PLLI2SCFGR_PLLI2SN_2           (0x004UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000100 */
#define RCC_PLLI2SCFGR_PLLI2SN_3           (0x008UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000200 */
#define RCC_PLLI2SCFGR_PLLI2SN_4           (0x010UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000400 */
#define RCC_PLLI2SCFGR_PLLI2SN_5           (0x020UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000800 */
#define RCC_PLLI2SCFGR_PLLI2SN_6           (0x040UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00001000 */
#define RCC_PLLI2SCFGR_PLLI2SN_7           (0x080UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00002000 */
#define RCC_PLLI2SCFGR_PLLI2SN_8           (0x100UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00004000 */

#define RCC_MCODIV_4	((uint32_t)(0x2UL << (24U)) | (0x4UL << (24U)))

#define RCC_HCLK_DIV1                    0x00000000U
#define RCC_HCLK_DIV2                    0x00001000U
#define RCC_HCLK_DIV4                    0x00001400U
#define RCC_HCLK_DIV8                    0x00001800U
#define RCC_HCLK_DIV16                   0x00001C00U

#define RCC_HSE_OFF                      0x00000000U
#define RCC_HSE_ON                       (0x1UL << 16U)
#define RCC_HSE_BYPASS                   ((uint32_t)((0x1UL << (18U)) | (0x1UL << (16U))))

#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U

#define RCC_HSI_OFF                      ((uint8_t)0x00)
#define RCC_HSI_ON                       ((uint8_t)0x01)

#define RCC_LSI_OFF                      ((uint8_t)0x00)
#define RCC_LSI_ON                       ((uint8_t)0x01)

#define RCC_LSE_OFF                    0x00000000U
#define RCC_LSE_ON                     RCC_BDCR_LSEON
#define RCC_LSE_BYPASS                 ((uint32_t)((0x1UL << (2U)) | (0x1UL << (0U))))

#define RCC_PLL_NONE                      ((uint8_t)0x00)
#define RCC_PLL_OFF                       ((uint8_t)0x01)
#define RCC_PLL_ON                        ((uint8_t)0x02)

#define RCC_CLOCKTYPE_SYSCLK             0x00000001U
#define RCC_CLOCKTYPE_HCLK               0x00000002U
#define RCC_CLOCKTYPE_PCLK1              0x00000004U
#define RCC_CLOCKTYPE_PCLK2              0x00000008U

#define RCC_MCO1                         0x00000000U
#define RCC_MCO2                         0x00000001U

#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512



/******************************************************************************/
/*                                                                            */
/*                                    DCMI                                    */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DCMI_CR register  ******************/
#define DCMI_CR_CAPTURE_Pos        (0U)
#define DCMI_CR_CAPTURE_Msk        (0x1UL << DCMI_CR_CAPTURE_Pos)               /*!< 0x00000001 */
#define DCMI_CR_CAPTURE            DCMI_CR_CAPTURE_Msk
#define DCMI_CR_CM_Pos             (1U)
#define DCMI_CR_CM_Msk             (0x1UL << DCMI_CR_CM_Pos)                    /*!< 0x00000002 */
#define DCMI_CR_CM                 DCMI_CR_CM_Msk
#define DCMI_CR_CROP_Pos           (2U)
#define DCMI_CR_CROP_Msk           (0x1UL << DCMI_CR_CROP_Pos)                  /*!< 0x00000004 */
#define DCMI_CR_CROP               DCMI_CR_CROP_Msk
#define DCMI_CR_JPEG_Pos           (3U)
#define DCMI_CR_JPEG_Msk           (0x1UL << DCMI_CR_JPEG_Pos)                  /*!< 0x00000008 */
#define DCMI_CR_JPEG               DCMI_CR_JPEG_Msk
#define DCMI_CR_ESS_Pos            (4U)
#define DCMI_CR_ESS_Msk            (0x1UL << DCMI_CR_ESS_Pos)                   /*!< 0x00000010 */
#define DCMI_CR_ESS                DCMI_CR_ESS_Msk
#define DCMI_CR_PCKPOL_Pos         (5U)
#define DCMI_CR_PCKPOL_Msk         (0x1UL << DCMI_CR_PCKPOL_Pos)                /*!< 0x00000020 */
#define DCMI_CR_PCKPOL             DCMI_CR_PCKPOL_Msk
#define DCMI_CR_HSPOL_Pos          (6U)
#define DCMI_CR_HSPOL_Msk          (0x1UL << DCMI_CR_HSPOL_Pos)                 /*!< 0x00000040 */
#define DCMI_CR_HSPOL              DCMI_CR_HSPOL_Msk
#define DCMI_CR_VSPOL_Pos          (7U)
#define DCMI_CR_VSPOL_Msk          (0x1UL << DCMI_CR_VSPOL_Pos)                 /*!< 0x00000080 */
#define DCMI_CR_VSPOL              DCMI_CR_VSPOL_Msk
#define DCMI_CR_FCRC_0             0x00000100U
#define DCMI_CR_FCRC_1             0x00000200U
#define DCMI_CR_EDM_0              0x00000400U
#define DCMI_CR_EDM_1              0x00000800U
#define DCMI_CR_CRE_Pos            (12U)
#define DCMI_CR_CRE_Msk            (0x1UL << DCMI_CR_CRE_Pos)                   /*!< 0x00001000 */
#define DCMI_CR_CRE                DCMI_CR_CRE_Msk
#define DCMI_CR_ENABLE_Pos         (14U)
#define DCMI_CR_ENABLE_Msk         (0x1UL << DCMI_CR_ENABLE_Pos)                /*!< 0x00004000 */
#define DCMI_CR_ENABLE             DCMI_CR_ENABLE_Msk

/********************  Bits definition for DCMI_SR register  ******************/
#define DCMI_SR_HSYNC_Pos          (0U)
#define DCMI_SR_HSYNC_Msk          (0x1UL << DCMI_SR_HSYNC_Pos)                 /*!< 0x00000001 */
#define DCMI_SR_HSYNC              DCMI_SR_HSYNC_Msk
#define DCMI_SR_VSYNC_Pos          (1U)
#define DCMI_SR_VSYNC_Msk          (0x1UL << DCMI_SR_VSYNC_Pos)                 /*!< 0x00000002 */
#define DCMI_SR_VSYNC              DCMI_SR_VSYNC_Msk
#define DCMI_SR_FNE_Pos            (2U)
#define DCMI_SR_FNE_Msk            (0x1UL << DCMI_SR_FNE_Pos)                   /*!< 0x00000004 */
#define DCMI_SR_FNE                DCMI_SR_FNE_Msk

/********************  Bits definition for DCMI_RIS register  *****************/
#define DCMI_RIS_FRAME_RIS_Pos     (0U)
#define DCMI_RIS_FRAME_RIS_Msk     (0x1UL << DCMI_RIS_FRAME_RIS_Pos)            /*!< 0x00000001 */
#define DCMI_RIS_FRAME_RIS         DCMI_RIS_FRAME_RIS_Msk
#define DCMI_RIS_OVR_RIS_Pos       (1U)
#define DCMI_RIS_OVR_RIS_Msk       (0x1UL << DCMI_RIS_OVR_RIS_Pos)              /*!< 0x00000002 */
#define DCMI_RIS_OVR_RIS           DCMI_RIS_OVR_RIS_Msk
#define DCMI_RIS_ERR_RIS_Pos       (2U)
#define DCMI_RIS_ERR_RIS_Msk       (0x1UL << DCMI_RIS_ERR_RIS_Pos)              /*!< 0x00000004 */
#define DCMI_RIS_ERR_RIS           DCMI_RIS_ERR_RIS_Msk
#define DCMI_RIS_VSYNC_RIS_Pos     (3U)
#define DCMI_RIS_VSYNC_RIS_Msk     (0x1UL << DCMI_RIS_VSYNC_RIS_Pos)            /*!< 0x00000008 */
#define DCMI_RIS_VSYNC_RIS         DCMI_RIS_VSYNC_RIS_Msk
#define DCMI_RIS_LINE_RIS_Pos      (4U)
#define DCMI_RIS_LINE_RIS_Msk      (0x1UL << DCMI_RIS_LINE_RIS_Pos)             /*!< 0x00000010 */
#define DCMI_RIS_LINE_RIS          DCMI_RIS_LINE_RIS_Msk
/* Legacy defines */
#define DCMI_RISR_FRAME_RIS                  DCMI_RIS_FRAME_RIS
#define DCMI_RISR_OVR_RIS                    DCMI_RIS_OVR_RIS
#define DCMI_RISR_ERR_RIS                    DCMI_RIS_ERR_RIS
#define DCMI_RISR_VSYNC_RIS                  DCMI_RIS_VSYNC_RIS
#define DCMI_RISR_LINE_RIS                   DCMI_RIS_LINE_RIS
#define DCMI_RISR_OVF_RIS                    DCMI_RIS_OVR_RIS

/********************  Bits definition for DCMI_IER register  *****************/
#define DCMI_IER_FRAME_IE_Pos      (0U)
#define DCMI_IER_FRAME_IE_Msk      (0x1UL << DCMI_IER_FRAME_IE_Pos)             /*!< 0x00000001 */
#define DCMI_IER_FRAME_IE          DCMI_IER_FRAME_IE_Msk
#define DCMI_IER_OVR_IE_Pos        (1U)
#define DCMI_IER_OVR_IE_Msk        (0x1UL << DCMI_IER_OVR_IE_Pos)               /*!< 0x00000002 */
#define DCMI_IER_OVR_IE            DCMI_IER_OVR_IE_Msk
#define DCMI_IER_ERR_IE_Pos        (2U)
#define DCMI_IER_ERR_IE_Msk        (0x1UL << DCMI_IER_ERR_IE_Pos)               /*!< 0x00000004 */
#define DCMI_IER_ERR_IE            DCMI_IER_ERR_IE_Msk
#define DCMI_IER_VSYNC_IE_Pos      (3U)
#define DCMI_IER_VSYNC_IE_Msk      (0x1UL << DCMI_IER_VSYNC_IE_Pos)             /*!< 0x00000008 */
#define DCMI_IER_VSYNC_IE          DCMI_IER_VSYNC_IE_Msk
#define DCMI_IER_LINE_IE_Pos       (4U)
#define DCMI_IER_LINE_IE_Msk       (0x1UL << DCMI_IER_LINE_IE_Pos)              /*!< 0x00000010 */
#define DCMI_IER_LINE_IE           DCMI_IER_LINE_IE_Msk
/* Legacy defines */
#define DCMI_IER_OVF_IE                      DCMI_IER_OVR_IE

/********************  Bits definition for DCMI_MIS register  *****************/
#define DCMI_MIS_FRAME_MIS_Pos     (0U)
#define DCMI_MIS_FRAME_MIS_Msk     (0x1UL << DCMI_MIS_FRAME_MIS_Pos)            /*!< 0x00000001 */
#define DCMI_MIS_FRAME_MIS         DCMI_MIS_FRAME_MIS_Msk
#define DCMI_MIS_OVR_MIS_Pos       (1U)
#define DCMI_MIS_OVR_MIS_Msk       (0x1UL << DCMI_MIS_OVR_MIS_Pos)              /*!< 0x00000002 */
#define DCMI_MIS_OVR_MIS           DCMI_MIS_OVR_MIS_Msk
#define DCMI_MIS_ERR_MIS_Pos       (2U)
#define DCMI_MIS_ERR_MIS_Msk       (0x1UL << DCMI_MIS_ERR_MIS_Pos)              /*!< 0x00000004 */
#define DCMI_MIS_ERR_MIS           DCMI_MIS_ERR_MIS_Msk
#define DCMI_MIS_VSYNC_MIS_Pos     (3U)
#define DCMI_MIS_VSYNC_MIS_Msk     (0x1UL << DCMI_MIS_VSYNC_MIS_Pos)            /*!< 0x00000008 */
#define DCMI_MIS_VSYNC_MIS         DCMI_MIS_VSYNC_MIS_Msk
#define DCMI_MIS_LINE_MIS_Pos      (4U)
#define DCMI_MIS_LINE_MIS_Msk      (0x1UL << DCMI_MIS_LINE_MIS_Pos)             /*!< 0x00000010 */
#define DCMI_MIS_LINE_MIS          DCMI_MIS_LINE_MIS_Msk

/* Legacy defines */
#define DCMI_MISR_FRAME_MIS                  DCMI_MIS_FRAME_MIS
#define DCMI_MISR_OVF_MIS                    DCMI_MIS_OVR_MIS
#define DCMI_MISR_ERR_MIS                    DCMI_MIS_ERR_MIS
#define DCMI_MISR_VSYNC_MIS                  DCMI_MIS_VSYNC_MIS
#define DCMI_MISR_LINE_MIS                   DCMI_MIS_LINE_MIS

/********************  Bits definition for DCMI_ICR register  *****************/
#define DCMI_ICR_FRAME_ISC_Pos     (0U)
#define DCMI_ICR_FRAME_ISC_Msk     (0x1UL << DCMI_ICR_FRAME_ISC_Pos)            /*!< 0x00000001 */
#define DCMI_ICR_FRAME_ISC         DCMI_ICR_FRAME_ISC_Msk
#define DCMI_ICR_OVR_ISC_Pos       (1U)
#define DCMI_ICR_OVR_ISC_Msk       (0x1UL << DCMI_ICR_OVR_ISC_Pos)              /*!< 0x00000002 */
#define DCMI_ICR_OVR_ISC           DCMI_ICR_OVR_ISC_Msk
#define DCMI_ICR_ERR_ISC_Pos       (2U)
#define DCMI_ICR_ERR_ISC_Msk       (0x1UL << DCMI_ICR_ERR_ISC_Pos)              /*!< 0x00000004 */
#define DCMI_ICR_ERR_ISC           DCMI_ICR_ERR_ISC_Msk
#define DCMI_ICR_VSYNC_ISC_Pos     (3U)
#define DCMI_ICR_VSYNC_ISC_Msk     (0x1UL << DCMI_ICR_VSYNC_ISC_Pos)            /*!< 0x00000008 */
#define DCMI_ICR_VSYNC_ISC         DCMI_ICR_VSYNC_ISC_Msk
#define DCMI_ICR_LINE_ISC_Pos      (4U)
#define DCMI_ICR_LINE_ISC_Msk      (0x1UL << DCMI_ICR_LINE_ISC_Pos)             /*!< 0x00000010 */
#define DCMI_ICR_LINE_ISC          DCMI_ICR_LINE_ISC_Msk

/* Legacy defines */
#define DCMI_ICR_OVF_ISC                     DCMI_ICR_OVR_ISC

/********************  Bits definition for DCMI_ESCR register  ******************/
#define DCMI_ESCR_FSC_Pos          (0U)
#define DCMI_ESCR_FSC_Msk          (0xFFUL << DCMI_ESCR_FSC_Pos)                /*!< 0x000000FF */
#define DCMI_ESCR_FSC              DCMI_ESCR_FSC_Msk
#define DCMI_ESCR_LSC_Pos          (8U)
#define DCMI_ESCR_LSC_Msk          (0xFFUL << DCMI_ESCR_LSC_Pos)                /*!< 0x0000FF00 */
#define DCMI_ESCR_LSC              DCMI_ESCR_LSC_Msk
#define DCMI_ESCR_LEC_Pos          (16U)
#define DCMI_ESCR_LEC_Msk          (0xFFUL << DCMI_ESCR_LEC_Pos)                /*!< 0x00FF0000 */
#define DCMI_ESCR_LEC              DCMI_ESCR_LEC_Msk
#define DCMI_ESCR_FEC_Pos          (24U)
#define DCMI_ESCR_FEC_Msk          (0xFFUL << DCMI_ESCR_FEC_Pos)                /*!< 0xFF000000 */
#define DCMI_ESCR_FEC              DCMI_ESCR_FEC_Msk

/********************  Bits definition for DCMI_ESUR register  ******************/
#define DCMI_ESUR_FSU_Pos          (0U)
#define DCMI_ESUR_FSU_Msk          (0xFFUL << DCMI_ESUR_FSU_Pos)                /*!< 0x000000FF */
#define DCMI_ESUR_FSU              DCMI_ESUR_FSU_Msk
#define DCMI_ESUR_LSU_Pos          (8U)
#define DCMI_ESUR_LSU_Msk          (0xFFUL << DCMI_ESUR_LSU_Pos)                /*!< 0x0000FF00 */
#define DCMI_ESUR_LSU              DCMI_ESUR_LSU_Msk
#define DCMI_ESUR_LEU_Pos          (16U)
#define DCMI_ESUR_LEU_Msk          (0xFFUL << DCMI_ESUR_LEU_Pos)                /*!< 0x00FF0000 */
#define DCMI_ESUR_LEU              DCMI_ESUR_LEU_Msk
#define DCMI_ESUR_FEU_Pos          (24U)
#define DCMI_ESUR_FEU_Msk          (0xFFUL << DCMI_ESUR_FEU_Pos)                /*!< 0xFF000000 */
#define DCMI_ESUR_FEU              DCMI_ESUR_FEU_Msk

/********************  Bits definition for DCMI_CWSTRT register  ******************/
#define DCMI_CWSTRT_HOFFCNT_Pos    (0U)
#define DCMI_CWSTRT_HOFFCNT_Msk    (0x3FFFUL << DCMI_CWSTRT_HOFFCNT_Pos)        /*!< 0x00003FFF */
#define DCMI_CWSTRT_HOFFCNT        DCMI_CWSTRT_HOFFCNT_Msk
#define DCMI_CWSTRT_VST_Pos        (16U)
#define DCMI_CWSTRT_VST_Msk        (0x1FFFUL << DCMI_CWSTRT_VST_Pos)            /*!< 0x1FFF0000 */
#define DCMI_CWSTRT_VST            DCMI_CWSTRT_VST_Msk

/********************  Bits definition for DCMI_CWSIZE register  ******************/
#define DCMI_CWSIZE_CAPCNT_Pos     (0U)
#define DCMI_CWSIZE_CAPCNT_Msk     (0x3FFFUL << DCMI_CWSIZE_CAPCNT_Pos)         /*!< 0x00003FFF */
#define DCMI_CWSIZE_CAPCNT         DCMI_CWSIZE_CAPCNT_Msk
#define DCMI_CWSIZE_VLINE_Pos      (16U)
#define DCMI_CWSIZE_VLINE_Msk      (0x3FFFUL << DCMI_CWSIZE_VLINE_Pos)          /*!< 0x3FFF0000 */
#define DCMI_CWSIZE_VLINE          DCMI_CWSIZE_VLINE_Msk

/********************  Bits definition for DCMI_DR register  *********************/
#define DCMI_DR_BYTE0_Pos          (0U)
#define DCMI_DR_BYTE0_Msk          (0xFFUL << DCMI_DR_BYTE0_Pos)                /*!< 0x000000FF */
#define DCMI_DR_BYTE0              DCMI_DR_BYTE0_Msk
#define DCMI_DR_BYTE1_Pos          (8U)
#define DCMI_DR_BYTE1_Msk          (0xFFUL << DCMI_DR_BYTE1_Pos)                /*!< 0x0000FF00 */
#define DCMI_DR_BYTE1              DCMI_DR_BYTE1_Msk
#define DCMI_DR_BYTE2_Pos          (16U)
#define DCMI_DR_BYTE2_Msk          (0xFFUL << DCMI_DR_BYTE2_Pos)                /*!< 0x00FF0000 */
#define DCMI_DR_BYTE2              DCMI_DR_BYTE2_Msk
#define DCMI_DR_BYTE3_Pos          (24U)
#define DCMI_DR_BYTE3_Msk          (0xFFUL << DCMI_DR_BYTE3_Pos)                /*!< 0xFF000000 */
#define DCMI_DR_BYTE3              DCMI_DR_BYTE3_Msk


/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY_Pos          (0U)
#define FLASH_ACR_LATENCY_Msk          (0x7UL << FLASH_ACR_LATENCY_Pos)         /*!< 0x00000007 */
#define FLASH_ACR_LATENCY              FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_0WS          0x00000000U
#define FLASH_ACR_LATENCY_1WS          0x00000001U
#define FLASH_ACR_LATENCY_2WS          0x00000002U
#define FLASH_ACR_LATENCY_3WS          0x00000003U
#define FLASH_ACR_LATENCY_4WS          0x00000004U
#define FLASH_ACR_LATENCY_5WS          0x00000005U
#define FLASH_ACR_LATENCY_6WS          0x00000006U
#define FLASH_ACR_LATENCY_7WS          0x00000007U


#define FLASH_ACR_PRFTEN_Pos           (8U)
#define FLASH_ACR_PRFTEN_Msk           (0x1UL << FLASH_ACR_PRFTEN_Pos)          /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN               FLASH_ACR_PRFTEN_Msk
#define FLASH_ACR_ICEN_Pos             (9U)
#define FLASH_ACR_ICEN_Msk             (0x1UL << FLASH_ACR_ICEN_Pos)            /*!< 0x00000200 */
#define FLASH_ACR_ICEN                 FLASH_ACR_ICEN_Msk
#define FLASH_ACR_DCEN_Pos             (10U)
#define FLASH_ACR_DCEN_Msk             (0x1UL << FLASH_ACR_DCEN_Pos)            /*!< 0x00000400 */
#define FLASH_ACR_DCEN                 FLASH_ACR_DCEN_Msk
#define FLASH_ACR_ICRST_Pos            (11U)
#define FLASH_ACR_ICRST_Msk            (0x1UL << FLASH_ACR_ICRST_Pos)           /*!< 0x00000800 */
#define FLASH_ACR_ICRST                FLASH_ACR_ICRST_Msk
#define FLASH_ACR_DCRST_Pos            (12U)
#define FLASH_ACR_DCRST_Msk            (0x1UL << FLASH_ACR_DCRST_Pos)           /*!< 0x00001000 */
#define FLASH_ACR_DCRST                FLASH_ACR_DCRST_Msk
#define FLASH_ACR_BYTE0_ADDRESS_Pos    (10U)
#define FLASH_ACR_BYTE0_ADDRESS_Msk    (0x10008FUL << FLASH_ACR_BYTE0_ADDRESS_Pos) /*!< 0x40023C00 */
#define FLASH_ACR_BYTE0_ADDRESS        FLASH_ACR_BYTE0_ADDRESS_Msk
#define FLASH_ACR_BYTE2_ADDRESS_Pos    (0U)
#define FLASH_ACR_BYTE2_ADDRESS_Msk    (0x40023C03UL << FLASH_ACR_BYTE2_ADDRESS_Pos) /*!< 0x40023C03 */
#define FLASH_ACR_BYTE2_ADDRESS        FLASH_ACR_BYTE2_ADDRESS_Msk

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos               (0U)
#define FLASH_SR_EOP_Msk               (0x1UL << FLASH_SR_EOP_Pos)              /*!< 0x00000001 */
#define FLASH_SR_EOP                   FLASH_SR_EOP_Msk
#define FLASH_SR_SOP_Pos               (1U)
#define FLASH_SR_SOP_Msk               (0x1UL << FLASH_SR_SOP_Pos)              /*!< 0x00000002 */
#define FLASH_SR_SOP                   FLASH_SR_SOP_Msk
#define FLASH_SR_WRPERR_Pos            (4U)
#define FLASH_SR_WRPERR_Msk            (0x1UL << FLASH_SR_WRPERR_Pos)           /*!< 0x00000010 */
#define FLASH_SR_WRPERR                FLASH_SR_WRPERR_Msk
#define FLASH_SR_PGAERR_Pos            (5U)
#define FLASH_SR_PGAERR_Msk            (0x1UL << FLASH_SR_PGAERR_Pos)           /*!< 0x00000020 */
#define FLASH_SR_PGAERR                FLASH_SR_PGAERR_Msk
#define FLASH_SR_PGPERR_Pos            (6U)
#define FLASH_SR_PGPERR_Msk            (0x1UL << FLASH_SR_PGPERR_Pos)           /*!< 0x00000040 */
#define FLASH_SR_PGPERR                FLASH_SR_PGPERR_Msk
#define FLASH_SR_PGSERR_Pos            (7U)
#define FLASH_SR_PGSERR_Msk            (0x1UL << FLASH_SR_PGSERR_Pos)           /*!< 0x00000080 */
#define FLASH_SR_PGSERR                FLASH_SR_PGSERR_Msk
#define FLASH_SR_BSY_Pos               (16U)
#define FLASH_SR_BSY_Msk               (0x1UL << FLASH_SR_BSY_Pos)              /*!< 0x00010000 */
#define FLASH_SR_BSY                   FLASH_SR_BSY_Msk

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG_Pos                (0U)
#define FLASH_CR_PG_Msk                (0x1UL << FLASH_CR_PG_Pos)               /*!< 0x00000001 */
#define FLASH_CR_PG                    FLASH_CR_PG_Msk
#define FLASH_CR_SER_Pos               (1U)
#define FLASH_CR_SER_Msk               (0x1UL << FLASH_CR_SER_Pos)              /*!< 0x00000002 */
#define FLASH_CR_SER                   FLASH_CR_SER_Msk
#define FLASH_CR_MER_Pos               (2U)
#define FLASH_CR_MER_Msk               (0x1UL << FLASH_CR_MER_Pos)              /*!< 0x00000004 */
#define FLASH_CR_MER                   FLASH_CR_MER_Msk
#define FLASH_CR_SNB_Pos               (3U)
#define FLASH_CR_SNB_Msk               (0x1FUL << FLASH_CR_SNB_Pos)             /*!< 0x000000F8 */
#define FLASH_CR_SNB                   FLASH_CR_SNB_Msk
#define FLASH_CR_SNB_0                 (0x01UL << FLASH_CR_SNB_Pos)             /*!< 0x00000008 */
#define FLASH_CR_SNB_1                 (0x02UL << FLASH_CR_SNB_Pos)             /*!< 0x00000010 */
#define FLASH_CR_SNB_2                 (0x04UL << FLASH_CR_SNB_Pos)             /*!< 0x00000020 */
#define FLASH_CR_SNB_3                 (0x08UL << FLASH_CR_SNB_Pos)             /*!< 0x00000040 */
#define FLASH_CR_SNB_4                 (0x10UL << FLASH_CR_SNB_Pos)             /*!< 0x00000080 */
#define FLASH_CR_PSIZE_Pos             (8U)
#define FLASH_CR_PSIZE_Msk             (0x3UL << FLASH_CR_PSIZE_Pos)            /*!< 0x00000300 */
#define FLASH_CR_PSIZE                 FLASH_CR_PSIZE_Msk
#define FLASH_CR_PSIZE_0               (0x1UL << FLASH_CR_PSIZE_Pos)            /*!< 0x00000100 */
#define FLASH_CR_PSIZE_1               (0x2UL << FLASH_CR_PSIZE_Pos)            /*!< 0x00000200 */
#define FLASH_CR_STRT_Pos              (16U)
#define FLASH_CR_STRT_Msk              (0x1UL << FLASH_CR_STRT_Pos)             /*!< 0x00010000 */
#define FLASH_CR_STRT                  FLASH_CR_STRT_Msk
#define FLASH_CR_EOPIE_Pos             (24U)
#define FLASH_CR_EOPIE_Msk             (0x1UL << FLASH_CR_EOPIE_Pos)            /*!< 0x01000000 */
#define FLASH_CR_EOPIE                 FLASH_CR_EOPIE_Msk
#define FLASH_CR_ERRIE_Pos             (25U)
#define FLASH_CR_ERRIE_Msk             (0x1UL << FLASH_CR_ERRIE_Pos)
#define FLASH_CR_ERRIE                 FLASH_CR_ERRIE_Msk
#define FLASH_CR_LOCK_Pos              (31U)
#define FLASH_CR_LOCK_Msk              (0x1UL << FLASH_CR_LOCK_Pos)             /*!< 0x80000000 */
#define FLASH_CR_LOCK                  FLASH_CR_LOCK_Msk

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK_Pos        (0U)
#define FLASH_OPTCR_OPTLOCK_Msk        (0x1UL << FLASH_OPTCR_OPTLOCK_Pos)       /*!< 0x00000001 */
#define FLASH_OPTCR_OPTLOCK            FLASH_OPTCR_OPTLOCK_Msk
#define FLASH_OPTCR_OPTSTRT_Pos        (1U)
#define FLASH_OPTCR_OPTSTRT_Msk        (0x1UL << FLASH_OPTCR_OPTSTRT_Pos)       /*!< 0x00000002 */
#define FLASH_OPTCR_OPTSTRT            FLASH_OPTCR_OPTSTRT_Msk

#define FLASH_OPTCR_BOR_LEV_0          0x00000004U
#define FLASH_OPTCR_BOR_LEV_1          0x00000008U
#define FLASH_OPTCR_BOR_LEV_Pos        (2U)
#define FLASH_OPTCR_BOR_LEV_Msk        (0x3UL << FLASH_OPTCR_BOR_LEV_Pos)       /*!< 0x0000000C */
#define FLASH_OPTCR_BOR_LEV            FLASH_OPTCR_BOR_LEV_Msk
#define FLASH_OPTCR_WDG_SW_Pos         (5U)
#define FLASH_OPTCR_WDG_SW_Msk         (0x1UL << FLASH_OPTCR_WDG_SW_Pos)        /*!< 0x00000020 */
#define FLASH_OPTCR_WDG_SW             FLASH_OPTCR_WDG_SW_Msk
#define FLASH_OPTCR_nRST_STOP_Pos      (6U)
#define FLASH_OPTCR_nRST_STOP_Msk      (0x1UL << FLASH_OPTCR_nRST_STOP_Pos)     /*!< 0x00000040 */
#define FLASH_OPTCR_nRST_STOP          FLASH_OPTCR_nRST_STOP_Msk
#define FLASH_OPTCR_nRST_STDBY_Pos     (7U)
#define FLASH_OPTCR_nRST_STDBY_Msk     (0x1UL << FLASH_OPTCR_nRST_STDBY_Pos)    /*!< 0x00000080 */
#define FLASH_OPTCR_nRST_STDBY         FLASH_OPTCR_nRST_STDBY_Msk
#define FLASH_OPTCR_RDP_Pos            (8U)
#define FLASH_OPTCR_RDP_Msk            (0xFFUL << FLASH_OPTCR_RDP_Pos)          /*!< 0x0000FF00 */
#define FLASH_OPTCR_RDP                FLASH_OPTCR_RDP_Msk
#define FLASH_OPTCR_RDP_0              (0x01UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000100 */
#define FLASH_OPTCR_RDP_1              (0x02UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000200 */
#define FLASH_OPTCR_RDP_2              (0x04UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000400 */
#define FLASH_OPTCR_RDP_3              (0x08UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000800 */
#define FLASH_OPTCR_RDP_4              (0x10UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00001000 */
#define FLASH_OPTCR_RDP_5              (0x20UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00002000 */
#define FLASH_OPTCR_RDP_6              (0x40UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00004000 */
#define FLASH_OPTCR_RDP_7              (0x80UL << FLASH_OPTCR_RDP_Pos)          /*!< 0x00008000 */
#define FLASH_OPTCR_nWRP_Pos           (16U)
#define FLASH_OPTCR_nWRP_Msk           (0xFFFUL << FLASH_OPTCR_nWRP_Pos)        /*!< 0x0FFF0000 */
#define FLASH_OPTCR_nWRP               FLASH_OPTCR_nWRP_Msk
#define FLASH_OPTCR_nWRP_0             0x00010000U
#define FLASH_OPTCR_nWRP_1             0x00020000U
#define FLASH_OPTCR_nWRP_2             0x00040000U
#define FLASH_OPTCR_nWRP_3             0x00080000U
#define FLASH_OPTCR_nWRP_4             0x00100000U
#define FLASH_OPTCR_nWRP_5             0x00200000U
#define FLASH_OPTCR_nWRP_6             0x00400000U
#define FLASH_OPTCR_nWRP_7             0x00800000U
#define FLASH_OPTCR_nWRP_8             0x01000000U
#define FLASH_OPTCR_nWRP_9             0x02000000U
#define FLASH_OPTCR_nWRP_10            0x04000000U
#define FLASH_OPTCR_nWRP_11            0x08000000U

/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP_Pos          (16U)
#define FLASH_OPTCR1_nWRP_Msk          (0xFFFUL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x0FFF0000 */
#define FLASH_OPTCR1_nWRP              FLASH_OPTCR1_nWRP_Msk
#define FLASH_OPTCR1_nWRP_0            (0x001UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00010000 */
#define FLASH_OPTCR1_nWRP_1            (0x002UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00020000 */
#define FLASH_OPTCR1_nWRP_2            (0x004UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00040000 */
#define FLASH_OPTCR1_nWRP_3            (0x008UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00080000 */
#define FLASH_OPTCR1_nWRP_4            (0x010UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00100000 */
#define FLASH_OPTCR1_nWRP_5            (0x020UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00200000 */
#define FLASH_OPTCR1_nWRP_6            (0x040UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00400000 */
#define FLASH_OPTCR1_nWRP_7            (0x080UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00800000 */
#define FLASH_OPTCR1_nWRP_8            (0x100UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x01000000 */
#define FLASH_OPTCR1_nWRP_9            (0x200UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x02000000 */
#define FLASH_OPTCR1_nWRP_10           (0x400UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x04000000 */
#define FLASH_OPTCR1_nWRP_11           (0x800UL << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x08000000 */


/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL_Pos       (25U)
#define DMA_SxCR_CHSEL_Msk       (0x7UL << DMA_SxCR_CHSEL_Pos)                  /*!< 0x0E000000 */
#define DMA_SxCR_CHSEL           DMA_SxCR_CHSEL_Msk
#define DMA_SxCR_CHSEL_0         0x02000000U
#define DMA_SxCR_CHSEL_1         0x04000000U
#define DMA_SxCR_CHSEL_2         0x08000000U
#define DMA_SxCR_MBURST_Pos      (23U)
#define DMA_SxCR_MBURST_Msk      (0x3UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01800000 */
#define DMA_SxCR_MBURST          DMA_SxCR_MBURST_Msk
#define DMA_SxCR_MBURST_0        (0x1UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x00800000 */
#define DMA_SxCR_MBURST_1        (0x2UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01000000 */
#define DMA_SxCR_PBURST_Pos      (21U)
#define DMA_SxCR_PBURST_Msk      (0x3UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00600000 */
#define DMA_SxCR_PBURST          DMA_SxCR_PBURST_Msk
#define DMA_SxCR_PBURST_0        (0x1UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00200000 */
#define DMA_SxCR_PBURST_1        (0x2UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00400000 */
#define DMA_SxCR_CT_Pos          (19U)
#define DMA_SxCR_CT_Msk          (0x1UL << DMA_SxCR_CT_Pos)                     /*!< 0x00080000 */
#define DMA_SxCR_CT              DMA_SxCR_CT_Msk
#define DMA_SxCR_DBM_Pos         (18U)
#define DMA_SxCR_DBM_Msk         (0x1UL << DMA_SxCR_DBM_Pos)                    /*!< 0x00040000 */
#define DMA_SxCR_DBM             DMA_SxCR_DBM_Msk
#define DMA_SxCR_PL_Pos          (16U)
#define DMA_SxCR_PL_Msk          (0x3UL << DMA_SxCR_PL_Pos)                     /*!< 0x00030000 */
#define DMA_SxCR_PL              DMA_SxCR_PL_Msk
#define DMA_SxCR_PL_0            (0x1UL << DMA_SxCR_PL_Pos)                     /*!< 0x00010000 */
#define DMA_SxCR_PL_1            (0x2UL << DMA_SxCR_PL_Pos)                     /*!< 0x00020000 */
#define DMA_SxCR_PINCOS_Pos      (15U)
#define DMA_SxCR_PINCOS_Msk      (0x1UL << DMA_SxCR_PINCOS_Pos)                 /*!< 0x00008000 */
#define DMA_SxCR_PINCOS          DMA_SxCR_PINCOS_Msk
#define DMA_SxCR_MSIZE_Pos       (13U)
#define DMA_SxCR_MSIZE_Msk       (0x3UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00006000 */
#define DMA_SxCR_MSIZE           DMA_SxCR_MSIZE_Msk
#define DMA_SxCR_MSIZE_0         (0x1UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00002000 */
#define DMA_SxCR_MSIZE_1         (0x2UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00004000 */
#define DMA_SxCR_PSIZE_Pos       (11U)
#define DMA_SxCR_PSIZE_Msk       (0x3UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001800 */
#define DMA_SxCR_PSIZE           DMA_SxCR_PSIZE_Msk
#define DMA_SxCR_PSIZE_0         (0x1UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00000800 */
#define DMA_SxCR_PSIZE_1         (0x2UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001000 */
#define DMA_SxCR_MINC_Pos        (10U)
#define DMA_SxCR_MINC_Msk        (0x1UL << DMA_SxCR_MINC_Pos)                   /*!< 0x00000400 */
#define DMA_SxCR_MINC            DMA_SxCR_MINC_Msk
#define DMA_SxCR_PINC_Pos        (9U)
#define DMA_SxCR_PINC_Msk        (0x1UL << DMA_SxCR_PINC_Pos)                   /*!< 0x00000200 */
#define DMA_SxCR_PINC            DMA_SxCR_PINC_Msk
#define DMA_SxCR_CIRC_Pos        (8U)
#define DMA_SxCR_CIRC_Msk        (0x1UL << DMA_SxCR_CIRC_Pos)                   /*!< 0x00000100 */
#define DMA_SxCR_CIRC            DMA_SxCR_CIRC_Msk
#define DMA_SxCR_DIR_Pos         (6U)
#define DMA_SxCR_DIR_Msk         (0x3UL << DMA_SxCR_DIR_Pos)                    /*!< 0x000000C0 */
#define DMA_SxCR_DIR             DMA_SxCR_DIR_Msk
#define DMA_SxCR_DIR_0           (0x1UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000040 */
#define DMA_SxCR_DIR_1           (0x2UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000080 */
#define DMA_SxCR_PFCTRL_Pos      (5U)
#define DMA_SxCR_PFCTRL_Msk      (0x1UL << DMA_SxCR_PFCTRL_Pos)                 /*!< 0x00000020 */
#define DMA_SxCR_PFCTRL          DMA_SxCR_PFCTRL_Msk
#define DMA_SxCR_TCIE_Pos        (4U)
#define DMA_SxCR_TCIE_Msk        (0x1UL << DMA_SxCR_TCIE_Pos)                   /*!< 0x00000010 */
#define DMA_SxCR_TCIE            DMA_SxCR_TCIE_Msk
#define DMA_SxCR_HTIE_Pos        (3U)
#define DMA_SxCR_HTIE_Msk        (0x1UL << DMA_SxCR_HTIE_Pos)                   /*!< 0x00000008 */
#define DMA_SxCR_HTIE            DMA_SxCR_HTIE_Msk
#define DMA_SxCR_TEIE_Pos        (2U)
#define DMA_SxCR_TEIE_Msk        (0x1UL << DMA_SxCR_TEIE_Pos)                   /*!< 0x00000004 */
#define DMA_SxCR_TEIE            DMA_SxCR_TEIE_Msk
#define DMA_SxCR_DMEIE_Pos       (1U)
#define DMA_SxCR_DMEIE_Msk       (0x1UL << DMA_SxCR_DMEIE_Pos)                  /*!< 0x00000002 */
#define DMA_SxCR_DMEIE           DMA_SxCR_DMEIE_Msk
#define DMA_SxCR_EN_Pos          (0U)
#define DMA_SxCR_EN_Msk          (0x1UL << DMA_SxCR_EN_Pos)                     /*!< 0x00000001 */
#define DMA_SxCR_EN              DMA_SxCR_EN_Msk

/* Legacy defines */
#define DMA_SxCR_ACK_Pos         (20U)
#define DMA_SxCR_ACK_Msk         (0x1UL << DMA_SxCR_ACK_Pos)                    /*!< 0x00100000 */
#define DMA_SxCR_ACK             DMA_SxCR_ACK_Msk

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT_Pos            (0U)
#define DMA_SxNDT_Msk            (0xFFFFUL << DMA_SxNDT_Pos)                    /*!< 0x0000FFFF */
#define DMA_SxNDT                DMA_SxNDT_Msk
#define DMA_SxNDT_0              (0x0001UL << DMA_SxNDT_Pos)                    /*!< 0x00000001 */
#define DMA_SxNDT_1              (0x0002UL << DMA_SxNDT_Pos)                    /*!< 0x00000002 */
#define DMA_SxNDT_2              (0x0004UL << DMA_SxNDT_Pos)                    /*!< 0x00000004 */
#define DMA_SxNDT_3              (0x0008UL << DMA_SxNDT_Pos)                    /*!< 0x00000008 */
#define DMA_SxNDT_4              (0x0010UL << DMA_SxNDT_Pos)                    /*!< 0x00000010 */
#define DMA_SxNDT_5              (0x0020UL << DMA_SxNDT_Pos)                    /*!< 0x00000020 */
#define DMA_SxNDT_6              (0x0040UL << DMA_SxNDT_Pos)                    /*!< 0x00000040 */
#define DMA_SxNDT_7              (0x0080UL << DMA_SxNDT_Pos)                    /*!< 0x00000080 */
#define DMA_SxNDT_8              (0x0100UL << DMA_SxNDT_Pos)                    /*!< 0x00000100 */
#define DMA_SxNDT_9              (0x0200UL << DMA_SxNDT_Pos)                    /*!< 0x00000200 */
#define DMA_SxNDT_10             (0x0400UL << DMA_SxNDT_Pos)                    /*!< 0x00000400 */
#define DMA_SxNDT_11             (0x0800UL << DMA_SxNDT_Pos)                    /*!< 0x00000800 */
#define DMA_SxNDT_12             (0x1000UL << DMA_SxNDT_Pos)                    /*!< 0x00001000 */
#define DMA_SxNDT_13             (0x2000UL << DMA_SxNDT_Pos)                    /*!< 0x00002000 */
#define DMA_SxNDT_14             (0x4000UL << DMA_SxNDT_Pos)                    /*!< 0x00004000 */
#define DMA_SxNDT_15             (0x8000UL << DMA_SxNDT_Pos)                    /*!< 0x00008000 */

/********************  Bits definition for DMA_SxFCR register  ****************/
#define DMA_SxFCR_FEIE_Pos       (7U)
#define DMA_SxFCR_FEIE_Msk       (0x1UL << DMA_SxFCR_FEIE_Pos)                  /*!< 0x00000080 */
#define DMA_SxFCR_FEIE           DMA_SxFCR_FEIE_Msk
#define DMA_SxFCR_FS_Pos         (3U)
#define DMA_SxFCR_FS_Msk         (0x7UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000038 */
#define DMA_SxFCR_FS             DMA_SxFCR_FS_Msk
#define DMA_SxFCR_FS_0           (0x1UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000008 */
#define DMA_SxFCR_FS_1           (0x2UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000010 */
#define DMA_SxFCR_FS_2           (0x4UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000020 */
#define DMA_SxFCR_DMDIS_Pos      (2U)
#define DMA_SxFCR_DMDIS_Msk      (0x1UL << DMA_SxFCR_DMDIS_Pos)                 /*!< 0x00000004 */
#define DMA_SxFCR_DMDIS          DMA_SxFCR_DMDIS_Msk
#define DMA_SxFCR_FTH_Pos        (0U)
#define DMA_SxFCR_FTH_Msk        (0x3UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000003 */
#define DMA_SxFCR_FTH            DMA_SxFCR_FTH_Msk
#define DMA_SxFCR_FTH_0          (0x1UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000001 */
#define DMA_SxFCR_FTH_1          (0x2UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000002 */

/********************  Bits definition for DMA_LISR register  *****************/
#define DMA_LISR_TCIF3_Pos       (27U)
#define DMA_LISR_TCIF3_Msk       (0x1UL << DMA_LISR_TCIF3_Pos)                  /*!< 0x08000000 */
#define DMA_LISR_TCIF3           DMA_LISR_TCIF3_Msk
#define DMA_LISR_HTIF3_Pos       (26U)
#define DMA_LISR_HTIF3_Msk       (0x1UL << DMA_LISR_HTIF3_Pos)                  /*!< 0x04000000 */
#define DMA_LISR_HTIF3           DMA_LISR_HTIF3_Msk
#define DMA_LISR_TEIF3_Pos       (25U)
#define DMA_LISR_TEIF3_Msk       (0x1UL << DMA_LISR_TEIF3_Pos)                  /*!< 0x02000000 */
#define DMA_LISR_TEIF3           DMA_LISR_TEIF3_Msk
#define DMA_LISR_DMEIF3_Pos      (24U)
#define DMA_LISR_DMEIF3_Msk      (0x1UL << DMA_LISR_DMEIF3_Pos)                 /*!< 0x01000000 */
#define DMA_LISR_DMEIF3          DMA_LISR_DMEIF3_Msk
#define DMA_LISR_FEIF3_Pos       (22U)
#define DMA_LISR_FEIF3_Msk       (0x1UL << DMA_LISR_FEIF3_Pos)                  /*!< 0x00400000 */
#define DMA_LISR_FEIF3           DMA_LISR_FEIF3_Msk
#define DMA_LISR_TCIF2_Pos       (21U)
#define DMA_LISR_TCIF2_Msk       (0x1UL << DMA_LISR_TCIF2_Pos)                  /*!< 0x00200000 */
#define DMA_LISR_TCIF2           DMA_LISR_TCIF2_Msk
#define DMA_LISR_HTIF2_Pos       (20U)
#define DMA_LISR_HTIF2_Msk       (0x1UL << DMA_LISR_HTIF2_Pos)                  /*!< 0x00100000 */
#define DMA_LISR_HTIF2           DMA_LISR_HTIF2_Msk
#define DMA_LISR_TEIF2_Pos       (19U)
#define DMA_LISR_TEIF2_Msk       (0x1UL << DMA_LISR_TEIF2_Pos)                  /*!< 0x00080000 */
#define DMA_LISR_TEIF2           DMA_LISR_TEIF2_Msk
#define DMA_LISR_DMEIF2_Pos      (18U)
#define DMA_LISR_DMEIF2_Msk      (0x1UL << DMA_LISR_DMEIF2_Pos)                 /*!< 0x00040000 */
#define DMA_LISR_DMEIF2          DMA_LISR_DMEIF2_Msk
#define DMA_LISR_FEIF2_Pos       (16U)
#define DMA_LISR_FEIF2_Msk       (0x1UL << DMA_LISR_FEIF2_Pos)                  /*!< 0x00010000 */
#define DMA_LISR_FEIF2           DMA_LISR_FEIF2_Msk
#define DMA_LISR_TCIF1_Pos       (11U)
#define DMA_LISR_TCIF1_Msk       (0x1UL << DMA_LISR_TCIF1_Pos)                  /*!< 0x00000800 */
#define DMA_LISR_TCIF1           DMA_LISR_TCIF1_Msk
#define DMA_LISR_HTIF1_Pos       (10U)
#define DMA_LISR_HTIF1_Msk       (0x1UL << DMA_LISR_HTIF1_Pos)                  /*!< 0x00000400 */
#define DMA_LISR_HTIF1           DMA_LISR_HTIF1_Msk
#define DMA_LISR_TEIF1_Pos       (9U)
#define DMA_LISR_TEIF1_Msk       (0x1UL << DMA_LISR_TEIF1_Pos)                  /*!< 0x00000200 */
#define DMA_LISR_TEIF1           DMA_LISR_TEIF1_Msk
#define DMA_LISR_DMEIF1_Pos      (8U)
#define DMA_LISR_DMEIF1_Msk      (0x1UL << DMA_LISR_DMEIF1_Pos)                 /*!< 0x00000100 */
#define DMA_LISR_DMEIF1          DMA_LISR_DMEIF1_Msk
#define DMA_LISR_FEIF1_Pos       (6U)
#define DMA_LISR_FEIF1_Msk       (0x1UL << DMA_LISR_FEIF1_Pos)                  /*!< 0x00000040 */
#define DMA_LISR_FEIF1           DMA_LISR_FEIF1_Msk
#define DMA_LISR_TCIF0_Pos       (5U)
#define DMA_LISR_TCIF0_Msk       (0x1UL << DMA_LISR_TCIF0_Pos)                  /*!< 0x00000020 */
#define DMA_LISR_TCIF0           DMA_LISR_TCIF0_Msk
#define DMA_LISR_HTIF0_Pos       (4U)
#define DMA_LISR_HTIF0_Msk       (0x1UL << DMA_LISR_HTIF0_Pos)                  /*!< 0x00000010 */
#define DMA_LISR_HTIF0           DMA_LISR_HTIF0_Msk
#define DMA_LISR_TEIF0_Pos       (3U)
#define DMA_LISR_TEIF0_Msk       (0x1UL << DMA_LISR_TEIF0_Pos)                  /*!< 0x00000008 */
#define DMA_LISR_TEIF0           DMA_LISR_TEIF0_Msk
#define DMA_LISR_DMEIF0_Pos      (2U)
#define DMA_LISR_DMEIF0_Msk      (0x1UL << DMA_LISR_DMEIF0_Pos)                 /*!< 0x00000004 */
#define DMA_LISR_DMEIF0          DMA_LISR_DMEIF0_Msk
#define DMA_LISR_FEIF0_Pos       (0U)
#define DMA_LISR_FEIF0_Msk       (0x1UL << DMA_LISR_FEIF0_Pos)                  /*!< 0x00000001 */
#define DMA_LISR_FEIF0           DMA_LISR_FEIF0_Msk

/********************  Bits definition for DMA_HISR register  *****************/
#define DMA_HISR_TCIF7_Pos       (27U)
#define DMA_HISR_TCIF7_Msk       (0x1UL << DMA_HISR_TCIF7_Pos)                  /*!< 0x08000000 */
#define DMA_HISR_TCIF7           DMA_HISR_TCIF7_Msk
#define DMA_HISR_HTIF7_Pos       (26U)
#define DMA_HISR_HTIF7_Msk       (0x1UL << DMA_HISR_HTIF7_Pos)                  /*!< 0x04000000 */
#define DMA_HISR_HTIF7           DMA_HISR_HTIF7_Msk
#define DMA_HISR_TEIF7_Pos       (25U)
#define DMA_HISR_TEIF7_Msk       (0x1UL << DMA_HISR_TEIF7_Pos)                  /*!< 0x02000000 */
#define DMA_HISR_TEIF7           DMA_HISR_TEIF7_Msk
#define DMA_HISR_DMEIF7_Pos      (24U)
#define DMA_HISR_DMEIF7_Msk      (0x1UL << DMA_HISR_DMEIF7_Pos)                 /*!< 0x01000000 */
#define DMA_HISR_DMEIF7          DMA_HISR_DMEIF7_Msk
#define DMA_HISR_FEIF7_Pos       (22U)
#define DMA_HISR_FEIF7_Msk       (0x1UL << DMA_HISR_FEIF7_Pos)                  /*!< 0x00400000 */
#define DMA_HISR_FEIF7           DMA_HISR_FEIF7_Msk
#define DMA_HISR_TCIF6_Pos       (21U)
#define DMA_HISR_TCIF6_Msk       (0x1UL << DMA_HISR_TCIF6_Pos)                  /*!< 0x00200000 */
#define DMA_HISR_TCIF6           DMA_HISR_TCIF6_Msk
#define DMA_HISR_HTIF6_Pos       (20U)
#define DMA_HISR_HTIF6_Msk       (0x1UL << DMA_HISR_HTIF6_Pos)                  /*!< 0x00100000 */
#define DMA_HISR_HTIF6           DMA_HISR_HTIF6_Msk
#define DMA_HISR_TEIF6_Pos       (19U)
#define DMA_HISR_TEIF6_Msk       (0x1UL << DMA_HISR_TEIF6_Pos)                  /*!< 0x00080000 */
#define DMA_HISR_TEIF6           DMA_HISR_TEIF6_Msk
#define DMA_HISR_DMEIF6_Pos      (18U)
#define DMA_HISR_DMEIF6_Msk      (0x1UL << DMA_HISR_DMEIF6_Pos)                 /*!< 0x00040000 */
#define DMA_HISR_DMEIF6          DMA_HISR_DMEIF6_Msk
#define DMA_HISR_FEIF6_Pos       (16U)
#define DMA_HISR_FEIF6_Msk       (0x1UL << DMA_HISR_FEIF6_Pos)                  /*!< 0x00010000 */
#define DMA_HISR_FEIF6           DMA_HISR_FEIF6_Msk
#define DMA_HISR_TCIF5_Pos       (11U)
#define DMA_HISR_TCIF5_Msk       (0x1UL << DMA_HISR_TCIF5_Pos)                  /*!< 0x00000800 */
#define DMA_HISR_TCIF5           DMA_HISR_TCIF5_Msk
#define DMA_HISR_HTIF5_Pos       (10U)
#define DMA_HISR_HTIF5_Msk       (0x1UL << DMA_HISR_HTIF5_Pos)                  /*!< 0x00000400 */
#define DMA_HISR_HTIF5           DMA_HISR_HTIF5_Msk
#define DMA_HISR_TEIF5_Pos       (9U)
#define DMA_HISR_TEIF5_Msk       (0x1UL << DMA_HISR_TEIF5_Pos)                  /*!< 0x00000200 */
#define DMA_HISR_TEIF5           DMA_HISR_TEIF5_Msk
#define DMA_HISR_DMEIF5_Pos      (8U)
#define DMA_HISR_DMEIF5_Msk      (0x1UL << DMA_HISR_DMEIF5_Pos)                 /*!< 0x00000100 */
#define DMA_HISR_DMEIF5          DMA_HISR_DMEIF5_Msk
#define DMA_HISR_FEIF5_Pos       (6U)
#define DMA_HISR_FEIF5_Msk       (0x1UL << DMA_HISR_FEIF5_Pos)                  /*!< 0x00000040 */
#define DMA_HISR_FEIF5           DMA_HISR_FEIF5_Msk
#define DMA_HISR_TCIF4_Pos       (5U)
#define DMA_HISR_TCIF4_Msk       (0x1UL << DMA_HISR_TCIF4_Pos)                  /*!< 0x00000020 */
#define DMA_HISR_TCIF4           DMA_HISR_TCIF4_Msk
#define DMA_HISR_HTIF4_Pos       (4U)
#define DMA_HISR_HTIF4_Msk       (0x1UL << DMA_HISR_HTIF4_Pos)                  /*!< 0x00000010 */
#define DMA_HISR_HTIF4           DMA_HISR_HTIF4_Msk
#define DMA_HISR_TEIF4_Pos       (3U)
#define DMA_HISR_TEIF4_Msk       (0x1UL << DMA_HISR_TEIF4_Pos)                  /*!< 0x00000008 */
#define DMA_HISR_TEIF4           DMA_HISR_TEIF4_Msk
#define DMA_HISR_DMEIF4_Pos      (2U)
#define DMA_HISR_DMEIF4_Msk      (0x1UL << DMA_HISR_DMEIF4_Pos)                 /*!< 0x00000004 */
#define DMA_HISR_DMEIF4          DMA_HISR_DMEIF4_Msk
#define DMA_HISR_FEIF4_Pos       (0U)
#define DMA_HISR_FEIF4_Msk       (0x1UL << DMA_HISR_FEIF4_Pos)                  /*!< 0x00000001 */
#define DMA_HISR_FEIF4           DMA_HISR_FEIF4_Msk

/********************  Bits definition for DMA_LIFCR register  ****************/
#define DMA_LIFCR_CTCIF3_Pos     (27U)
#define DMA_LIFCR_CTCIF3_Msk     (0x1UL << DMA_LIFCR_CTCIF3_Pos)                /*!< 0x08000000 */
#define DMA_LIFCR_CTCIF3         DMA_LIFCR_CTCIF3_Msk
#define DMA_LIFCR_CHTIF3_Pos     (26U)
#define DMA_LIFCR_CHTIF3_Msk     (0x1UL << DMA_LIFCR_CHTIF3_Pos)                /*!< 0x04000000 */
#define DMA_LIFCR_CHTIF3         DMA_LIFCR_CHTIF3_Msk
#define DMA_LIFCR_CTEIF3_Pos     (25U)
#define DMA_LIFCR_CTEIF3_Msk     (0x1UL << DMA_LIFCR_CTEIF3_Pos)                /*!< 0x02000000 */
#define DMA_LIFCR_CTEIF3         DMA_LIFCR_CTEIF3_Msk
#define DMA_LIFCR_CDMEIF3_Pos    (24U)
#define DMA_LIFCR_CDMEIF3_Msk    (0x1UL << DMA_LIFCR_CDMEIF3_Pos)               /*!< 0x01000000 */
#define DMA_LIFCR_CDMEIF3        DMA_LIFCR_CDMEIF3_Msk
#define DMA_LIFCR_CFEIF3_Pos     (22U)
#define DMA_LIFCR_CFEIF3_Msk     (0x1UL << DMA_LIFCR_CFEIF3_Pos)                /*!< 0x00400000 */
#define DMA_LIFCR_CFEIF3         DMA_LIFCR_CFEIF3_Msk
#define DMA_LIFCR_CTCIF2_Pos     (21U)
#define DMA_LIFCR_CTCIF2_Msk     (0x1UL << DMA_LIFCR_CTCIF2_Pos)                /*!< 0x00200000 */
#define DMA_LIFCR_CTCIF2         DMA_LIFCR_CTCIF2_Msk
#define DMA_LIFCR_CHTIF2_Pos     (20U)
#define DMA_LIFCR_CHTIF2_Msk     (0x1UL << DMA_LIFCR_CHTIF2_Pos)                /*!< 0x00100000 */
#define DMA_LIFCR_CHTIF2         DMA_LIFCR_CHTIF2_Msk
#define DMA_LIFCR_CTEIF2_Pos     (19U)
#define DMA_LIFCR_CTEIF2_Msk     (0x1UL << DMA_LIFCR_CTEIF2_Pos)                /*!< 0x00080000 */
#define DMA_LIFCR_CTEIF2         DMA_LIFCR_CTEIF2_Msk
#define DMA_LIFCR_CDMEIF2_Pos    (18U)
#define DMA_LIFCR_CDMEIF2_Msk    (0x1UL << DMA_LIFCR_CDMEIF2_Pos)               /*!< 0x00040000 */
#define DMA_LIFCR_CDMEIF2        DMA_LIFCR_CDMEIF2_Msk
#define DMA_LIFCR_CFEIF2_Pos     (16U)
#define DMA_LIFCR_CFEIF2_Msk     (0x1UL << DMA_LIFCR_CFEIF2_Pos)                /*!< 0x00010000 */
#define DMA_LIFCR_CFEIF2         DMA_LIFCR_CFEIF2_Msk
#define DMA_LIFCR_CTCIF1_Pos     (11U)
#define DMA_LIFCR_CTCIF1_Msk     (0x1UL << DMA_LIFCR_CTCIF1_Pos)                /*!< 0x00000800 */
#define DMA_LIFCR_CTCIF1         DMA_LIFCR_CTCIF1_Msk
#define DMA_LIFCR_CHTIF1_Pos     (10U)
#define DMA_LIFCR_CHTIF1_Msk     (0x1UL << DMA_LIFCR_CHTIF1_Pos)                /*!< 0x00000400 */
#define DMA_LIFCR_CHTIF1         DMA_LIFCR_CHTIF1_Msk
#define DMA_LIFCR_CTEIF1_Pos     (9U)
#define DMA_LIFCR_CTEIF1_Msk     (0x1UL << DMA_LIFCR_CTEIF1_Pos)                /*!< 0x00000200 */
#define DMA_LIFCR_CTEIF1         DMA_LIFCR_CTEIF1_Msk
#define DMA_LIFCR_CDMEIF1_Pos    (8U)
#define DMA_LIFCR_CDMEIF1_Msk    (0x1UL << DMA_LIFCR_CDMEIF1_Pos)               /*!< 0x00000100 */
#define DMA_LIFCR_CDMEIF1        DMA_LIFCR_CDMEIF1_Msk
#define DMA_LIFCR_CFEIF1_Pos     (6U)
#define DMA_LIFCR_CFEIF1_Msk     (0x1UL << DMA_LIFCR_CFEIF1_Pos)                /*!< 0x00000040 */
#define DMA_LIFCR_CFEIF1         DMA_LIFCR_CFEIF1_Msk
#define DMA_LIFCR_CTCIF0_Pos     (5U)
#define DMA_LIFCR_CTCIF0_Msk     (0x1UL << DMA_LIFCR_CTCIF0_Pos)                /*!< 0x00000020 */
#define DMA_LIFCR_CTCIF0         DMA_LIFCR_CTCIF0_Msk
#define DMA_LIFCR_CHTIF0_Pos     (4U)
#define DMA_LIFCR_CHTIF0_Msk     (0x1UL << DMA_LIFCR_CHTIF0_Pos)                /*!< 0x00000010 */
#define DMA_LIFCR_CHTIF0         DMA_LIFCR_CHTIF0_Msk
#define DMA_LIFCR_CTEIF0_Pos     (3U)
#define DMA_LIFCR_CTEIF0_Msk     (0x1UL << DMA_LIFCR_CTEIF0_Pos)                /*!< 0x00000008 */
#define DMA_LIFCR_CTEIF0         DMA_LIFCR_CTEIF0_Msk
#define DMA_LIFCR_CDMEIF0_Pos    (2U)
#define DMA_LIFCR_CDMEIF0_Msk    (0x1UL << DMA_LIFCR_CDMEIF0_Pos)               /*!< 0x00000004 */
#define DMA_LIFCR_CDMEIF0        DMA_LIFCR_CDMEIF0_Msk
#define DMA_LIFCR_CFEIF0_Pos     (0U)
#define DMA_LIFCR_CFEIF0_Msk     (0x1UL << DMA_LIFCR_CFEIF0_Pos)                /*!< 0x00000001 */
#define DMA_LIFCR_CFEIF0         DMA_LIFCR_CFEIF0_Msk

/********************  Bits definition for DMA_HIFCR  register  ****************/
#define DMA_HIFCR_CTCIF7_Pos     (27U)
#define DMA_HIFCR_CTCIF7_Msk     (0x1UL << DMA_HIFCR_CTCIF7_Pos)                /*!< 0x08000000 */
#define DMA_HIFCR_CTCIF7         DMA_HIFCR_CTCIF7_Msk
#define DMA_HIFCR_CHTIF7_Pos     (26U)
#define DMA_HIFCR_CHTIF7_Msk     (0x1UL << DMA_HIFCR_CHTIF7_Pos)                /*!< 0x04000000 */
#define DMA_HIFCR_CHTIF7         DMA_HIFCR_CHTIF7_Msk
#define DMA_HIFCR_CTEIF7_Pos     (25U)
#define DMA_HIFCR_CTEIF7_Msk     (0x1UL << DMA_HIFCR_CTEIF7_Pos)                /*!< 0x02000000 */
#define DMA_HIFCR_CTEIF7         DMA_HIFCR_CTEIF7_Msk
#define DMA_HIFCR_CDMEIF7_Pos    (24U)
#define DMA_HIFCR_CDMEIF7_Msk    (0x1UL << DMA_HIFCR_CDMEIF7_Pos)               /*!< 0x01000000 */
#define DMA_HIFCR_CDMEIF7        DMA_HIFCR_CDMEIF7_Msk
#define DMA_HIFCR_CFEIF7_Pos     (22U)
#define DMA_HIFCR_CFEIF7_Msk     (0x1UL << DMA_HIFCR_CFEIF7_Pos)                /*!< 0x00400000 */
#define DMA_HIFCR_CFEIF7         DMA_HIFCR_CFEIF7_Msk
#define DMA_HIFCR_CTCIF6_Pos     (21U)
#define DMA_HIFCR_CTCIF6_Msk     (0x1UL << DMA_HIFCR_CTCIF6_Pos)                /*!< 0x00200000 */
#define DMA_HIFCR_CTCIF6         DMA_HIFCR_CTCIF6_Msk
#define DMA_HIFCR_CHTIF6_Pos     (20U)
#define DMA_HIFCR_CHTIF6_Msk     (0x1UL << DMA_HIFCR_CHTIF6_Pos)                /*!< 0x00100000 */
#define DMA_HIFCR_CHTIF6         DMA_HIFCR_CHTIF6_Msk
#define DMA_HIFCR_CTEIF6_Pos     (19U)
#define DMA_HIFCR_CTEIF6_Msk     (0x1UL << DMA_HIFCR_CTEIF6_Pos)                /*!< 0x00080000 */
#define DMA_HIFCR_CTEIF6         DMA_HIFCR_CTEIF6_Msk
#define DMA_HIFCR_CDMEIF6_Pos    (18U)
#define DMA_HIFCR_CDMEIF6_Msk    (0x1UL << DMA_HIFCR_CDMEIF6_Pos)               /*!< 0x00040000 */
#define DMA_HIFCR_CDMEIF6        DMA_HIFCR_CDMEIF6_Msk
#define DMA_HIFCR_CFEIF6_Pos     (16U)
#define DMA_HIFCR_CFEIF6_Msk     (0x1UL << DMA_HIFCR_CFEIF6_Pos)                /*!< 0x00010000 */
#define DMA_HIFCR_CFEIF6         DMA_HIFCR_CFEIF6_Msk
#define DMA_HIFCR_CTCIF5_Pos     (11U)
#define DMA_HIFCR_CTCIF5_Msk     (0x1UL << DMA_HIFCR_CTCIF5_Pos)                /*!< 0x00000800 */
#define DMA_HIFCR_CTCIF5         DMA_HIFCR_CTCIF5_Msk
#define DMA_HIFCR_CHTIF5_Pos     (10U)
#define DMA_HIFCR_CHTIF5_Msk     (0x1UL << DMA_HIFCR_CHTIF5_Pos)                /*!< 0x00000400 */
#define DMA_HIFCR_CHTIF5         DMA_HIFCR_CHTIF5_Msk
#define DMA_HIFCR_CTEIF5_Pos     (9U)
#define DMA_HIFCR_CTEIF5_Msk     (0x1UL << DMA_HIFCR_CTEIF5_Pos)                /*!< 0x00000200 */
#define DMA_HIFCR_CTEIF5         DMA_HIFCR_CTEIF5_Msk
#define DMA_HIFCR_CDMEIF5_Pos    (8U)
#define DMA_HIFCR_CDMEIF5_Msk    (0x1UL << DMA_HIFCR_CDMEIF5_Pos)               /*!< 0x00000100 */
#define DMA_HIFCR_CDMEIF5        DMA_HIFCR_CDMEIF5_Msk
#define DMA_HIFCR_CFEIF5_Pos     (6U)
#define DMA_HIFCR_CFEIF5_Msk     (0x1UL << DMA_HIFCR_CFEIF5_Pos)                /*!< 0x00000040 */
#define DMA_HIFCR_CFEIF5         DMA_HIFCR_CFEIF5_Msk
#define DMA_HIFCR_CTCIF4_Pos     (5U)
#define DMA_HIFCR_CTCIF4_Msk     (0x1UL << DMA_HIFCR_CTCIF4_Pos)                /*!< 0x00000020 */
#define DMA_HIFCR_CTCIF4         DMA_HIFCR_CTCIF4_Msk
#define DMA_HIFCR_CHTIF4_Pos     (4U)
#define DMA_HIFCR_CHTIF4_Msk     (0x1UL << DMA_HIFCR_CHTIF4_Pos)                /*!< 0x00000010 */
#define DMA_HIFCR_CHTIF4         DMA_HIFCR_CHTIF4_Msk
#define DMA_HIFCR_CTEIF4_Pos     (3U)
#define DMA_HIFCR_CTEIF4_Msk     (0x1UL << DMA_HIFCR_CTEIF4_Pos)                /*!< 0x00000008 */
#define DMA_HIFCR_CTEIF4         DMA_HIFCR_CTEIF4_Msk
#define DMA_HIFCR_CDMEIF4_Pos    (2U)
#define DMA_HIFCR_CDMEIF4_Msk    (0x1UL << DMA_HIFCR_CDMEIF4_Pos)               /*!< 0x00000004 */
#define DMA_HIFCR_CDMEIF4        DMA_HIFCR_CDMEIF4_Msk
#define DMA_HIFCR_CFEIF4_Pos     (0U)
#define DMA_HIFCR_CFEIF4_Msk     (0x1UL << DMA_HIFCR_CFEIF4_Pos)                /*!< 0x00000001 */
#define DMA_HIFCR_CFEIF4         DMA_HIFCR_CFEIF4_Msk

/******************  Bit definition for DMA_SxPAR register  ********************/
#define DMA_SxPAR_PA_Pos         (0U)
#define DMA_SxPAR_PA_Msk         (0xFFFFFFFFUL << DMA_SxPAR_PA_Pos)             /*!< 0xFFFFFFFF */
#define DMA_SxPAR_PA             DMA_SxPAR_PA_Msk                              /*!< Peripheral Address */

/******************  Bit definition for DMA_SxM0AR register  ********************/
#define DMA_SxM0AR_M0A_Pos       (0U)
#define DMA_SxM0AR_M0A_Msk       (0xFFFFFFFFUL << DMA_SxM0AR_M0A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM0AR_M0A           DMA_SxM0AR_M0A_Msk                            /*!< Memory Address */

/******************  Bit definition for DMA_SxM1AR register  ********************/
#define DMA_SxM1AR_M1A_Pos       (0U)
#define DMA_SxM1AR_M1A_Msk       (0xFFFFFFFFUL << DMA_SxM1AR_M1A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM1AR_M1A           DMA_SxM1AR_M1A_Msk                            /*!< Memory Address */


/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE_Pos           (0U)
#define SYSCFG_MEMRMP_MEM_MODE_Msk           (0x3UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000003 */
#define SYSCFG_MEMRMP_MEM_MODE               SYSCFG_MEMRMP_MEM_MODE_Msk        /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0             (0x1UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000001 */
#define SYSCFG_MEMRMP_MEM_MODE_1             (0x2UL << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000002 */
/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_MII_RMII_SEL_Pos          (23U)
#define SYSCFG_PMC_MII_RMII_SEL_Msk          (0x1UL << SYSCFG_PMC_MII_RMII_SEL_Pos) /*!< 0x00800000 */
#define SYSCFG_PMC_MII_RMII_SEL              SYSCFG_PMC_MII_RMII_SEL_Msk       /*!<Ethernet PHY interface selection */
/* Old MII_RMII_SEL bit definition, maintained for legacy purpose */
#define SYSCFG_PMC_MII_RMII             SYSCFG_PMC_MII_RMII_SEL

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0_Pos             (0U)
#define SYSCFG_EXTICR1_EXTI0_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI0_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI0                 SYSCFG_EXTICR1_EXTI0_Msk          /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1_Pos             (4U)
#define SYSCFG_EXTICR1_EXTI1_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI1_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI1                 SYSCFG_EXTICR1_EXTI1_Msk          /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2_Pos             (8U)
#define SYSCFG_EXTICR1_EXTI2_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI2_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI2                 SYSCFG_EXTICR1_EXTI2_Msk          /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3_Pos             (12U)
#define SYSCFG_EXTICR1_EXTI3_Msk             (0xFUL << SYSCFG_EXTICR1_EXTI3_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR1_EXTI3                 SYSCFG_EXTICR1_EXTI3_Msk          /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA              0x0000U                           /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB              0x0001U                           /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC              0x0002U                           /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD              0x0003U                           /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE              0x0004U                           /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF              0x0005U                           /*!<PF[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PG              0x0006U                           /*!<PG[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH              0x0007U                           /*!<PH[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PI              0x0008U                           /*!<PI[0] pin */

/**
  * @brief   EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA              0x0000U                           /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB              0x0010U                           /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC              0x0020U                           /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD              0x0030U                           /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE              0x0040U                           /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF              0x0050U                           /*!<PF[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PG              0x0060U                           /*!<PG[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH              0x0070U                           /*!<PH[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PI              0x0080U                           /*!<PI[1] pin */

/**
  * @brief   EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA              0x0000U                           /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB              0x0100U                           /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC              0x0200U                           /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD              0x0300U                           /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE              0x0400U                           /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF              0x0500U                           /*!<PF[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PG              0x0600U                           /*!<PG[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH              0x0700U                           /*!<PH[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PI              0x0800U                           /*!<PI[2] pin */

/**
  * @brief   EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA              0x0000U                           /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB              0x1000U                           /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC              0x2000U                           /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD              0x3000U                           /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE              0x4000U                           /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF              0x5000U                           /*!<PF[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG              0x6000U                           /*!<PG[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH              0x7000U                           /*!<PH[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PI              0x8000U                           /*!<PI[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4_Pos             (0U)
#define SYSCFG_EXTICR2_EXTI4_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI4_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI4                 SYSCFG_EXTICR2_EXTI4_Msk          /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5_Pos             (4U)
#define SYSCFG_EXTICR2_EXTI5_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI5_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI5                 SYSCFG_EXTICR2_EXTI5_Msk          /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6_Pos             (8U)
#define SYSCFG_EXTICR2_EXTI6_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI6_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI6                 SYSCFG_EXTICR2_EXTI6_Msk          /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7_Pos             (12U)
#define SYSCFG_EXTICR2_EXTI7_Msk             (0xFUL << SYSCFG_EXTICR2_EXTI7_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR2_EXTI7                 SYSCFG_EXTICR2_EXTI7_Msk          /*!<EXTI 7 configuration */

/**
  * @brief   EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA              0x0000U                           /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB              0x0001U                           /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC              0x0002U                           /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD              0x0003U                           /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE              0x0004U                           /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF              0x0005U                           /*!<PF[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PG              0x0006U                           /*!<PG[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH              0x0007U                           /*!<PH[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PI              0x0008U                           /*!<PI[4] pin */

/**
  * @brief   EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA              0x0000U                           /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB              0x0010U                           /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC              0x0020U                           /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD              0x0030U                           /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE              0x0040U                           /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF              0x0050U                           /*!<PF[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PG              0x0060U                           /*!<PG[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH              0x0070U                           /*!<PH[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PI              0x0080U                           /*!<PI[5] pin */

/**
  * @brief   EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA              0x0000U                           /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB              0x0100U                           /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC              0x0200U                           /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD              0x0300U                           /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE              0x0400U                           /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF              0x0500U                           /*!<PF[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PG              0x0600U                           /*!<PG[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH              0x0700U                           /*!<PH[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PI              0x0800U                           /*!<PI[6] pin */

/**
  * @brief   EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA              0x0000U                           /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB              0x1000U                           /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC              0x2000U                           /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD              0x3000U                           /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE              0x4000U                           /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF              0x5000U                           /*!<PF[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PG              0x6000U                           /*!<PG[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH              0x7000U                           /*!<PH[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PI              0x8000U                           /*!<PI[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8_Pos             (0U)
#define SYSCFG_EXTICR3_EXTI8_Msk             (0xFUL << SYSCFG_EXTICR3_EXTI8_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR3_EXTI8                 SYSCFG_EXTICR3_EXTI8_Msk          /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9_Pos             (4U)
#define SYSCFG_EXTICR3_EXTI9_Msk             (0xFUL << SYSCFG_EXTICR3_EXTI9_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR3_EXTI9                 SYSCFG_EXTICR3_EXTI9_Msk          /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10_Pos            (8U)
#define SYSCFG_EXTICR3_EXTI10_Msk            (0xFUL << SYSCFG_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR3_EXTI10                SYSCFG_EXTICR3_EXTI10_Msk         /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11_Pos            (12U)
#define SYSCFG_EXTICR3_EXTI11_Msk            (0xFUL << SYSCFG_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR3_EXTI11                SYSCFG_EXTICR3_EXTI11_Msk         /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA              0x0000U                           /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB              0x0001U                           /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC              0x0002U                           /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD              0x0003U                           /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE              0x0004U                           /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PF              0x0005U                           /*!<PF[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PG              0x0006U                           /*!<PG[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH              0x0007U                           /*!<PH[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PI              0x0008U                           /*!<PI[8] pin */

/**
  * @brief   EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA              0x0000U                           /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB              0x0010U                           /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC              0x0020U                           /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD              0x0030U                           /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE              0x0040U                           /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF              0x0050U                           /*!<PF[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PG              0x0060U                           /*!<PG[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH              0x0070U                           /*!<PH[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PI              0x0080U                           /*!<PI[9] pin */

/**
  * @brief   EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA             0x0000U                           /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB             0x0100U                           /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC             0x0200U                           /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD             0x0300U                           /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE             0x0400U                           /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF             0x0500U                           /*!<PF[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PG             0x0600U                           /*!<PG[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH             0x0700U                           /*!<PH[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PI             0x0800U                           /*!<PI[10] pin */

/**
  * @brief   EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA             0x0000U                           /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB             0x1000U                           /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC             0x2000U                           /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD             0x3000U                           /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE             0x4000U                           /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PF             0x5000U                           /*!<PF[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PG             0x6000U                           /*!<PG[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH             0x7000U                           /*!<PH[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PI             0x8000U                           /*!<PI[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12_Pos            (0U)
#define SYSCFG_EXTICR4_EXTI12_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR4_EXTI12                SYSCFG_EXTICR4_EXTI12_Msk         /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13_Pos            (4U)
#define SYSCFG_EXTICR4_EXTI13_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR4_EXTI13                SYSCFG_EXTICR4_EXTI13_Msk         /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14_Pos            (8U)
#define SYSCFG_EXTICR4_EXTI14_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR4_EXTI14                SYSCFG_EXTICR4_EXTI14_Msk         /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15_Pos            (12U)
#define SYSCFG_EXTICR4_EXTI15_Msk            (0xFUL << SYSCFG_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR4_EXTI15                SYSCFG_EXTICR4_EXTI15_Msk         /*!<EXTI 15 configuration */

/**
  * @brief   EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA             0x0000U                           /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB             0x0001U                           /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC             0x0002U                           /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD             0x0003U                           /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE             0x0004U                           /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PF             0x0005U                           /*!<PF[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PG             0x0006U                           /*!<PG[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH             0x0007U                           /*!<PH[12] pin */

/**
  * @brief   EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA             0x0000U                           /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB             0x0010U                           /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC             0x0020U                           /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD             0x0030U                           /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE             0x0040U                           /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PF             0x0050U                           /*!<PF[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PG             0x0060U                           /*!<PG[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH             0x0070U                           /*!<PH[13] pin */

/**
  * @brief   EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA             0x0000U                           /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB             0x0100U                           /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC             0x0200U                           /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD             0x0300U                           /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE             0x0400U                           /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PF             0x0500U                           /*!<PF[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PG             0x0600U                           /*!<PG[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH             0x0700U                           /*!<PH[14] pin */

/**
  * @brief   EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA             0x0000U                           /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB             0x1000U                           /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC             0x2000U                           /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD             0x3000U                           /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE             0x4000U                           /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PF             0x5000U                           /*!<PF[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PG             0x6000U                           /*!<PG[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH             0x7000U                           /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD_Pos              (0U)
#define SYSCFG_CMPCR_CMP_PD_Msk              (0x1UL << SYSCFG_CMPCR_CMP_PD_Pos) /*!< 0x00000001 */
#define SYSCFG_CMPCR_CMP_PD                  SYSCFG_CMPCR_CMP_PD_Msk           /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY_Pos               (8U)
#define SYSCFG_CMPCR_READY_Msk               (0x1UL << SYSCFG_CMPCR_READY_Pos)  /*!< 0x00000100 */
#define SYSCFG_CMPCR_READY                   SYSCFG_CMPCR_READY_Msk            /*!<Compensation cell power-down */


/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define DBGMCU_IDCODE_DEV_ID_Pos                     (0U)
#define DBGMCU_IDCODE_DEV_ID_Msk                     (0xFFFUL << DBGMCU_IDCODE_DEV_ID_Pos) /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID                         DBGMCU_IDCODE_DEV_ID_Msk
#define DBGMCU_IDCODE_REV_ID_Pos                     (16U)
#define DBGMCU_IDCODE_REV_ID_Msk                     (0xFFFFUL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID                         DBGMCU_IDCODE_REV_ID_Msk

/********************  Bit definition for DBGMCU_CR register  *****************/
#define DBGMCU_CR_DBG_SLEEP_Pos                      (0U)
#define DBGMCU_CR_DBG_SLEEP_Msk                      (0x1UL << DBGMCU_CR_DBG_SLEEP_Pos) /*!< 0x00000001 */
#define DBGMCU_CR_DBG_SLEEP                          DBGMCU_CR_DBG_SLEEP_Msk
#define DBGMCU_CR_DBG_STOP_Pos                       (1U)
#define DBGMCU_CR_DBG_STOP_Msk                       (0x1UL << DBGMCU_CR_DBG_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                           DBGMCU_CR_DBG_STOP_Msk
#define DBGMCU_CR_DBG_STANDBY_Pos                    (2U)
#define DBGMCU_CR_DBG_STANDBY_Msk                    (0x1UL << DBGMCU_CR_DBG_STANDBY_Pos) /*!< 0x00000004 */
#define DBGMCU_CR_DBG_STANDBY                        DBGMCU_CR_DBG_STANDBY_Msk
#define DBGMCU_CR_TRACE_IOEN_Pos                     (5U)
#define DBGMCU_CR_TRACE_IOEN_Msk                     (0x1UL << DBGMCU_CR_TRACE_IOEN_Pos) /*!< 0x00000020 */
#define DBGMCU_CR_TRACE_IOEN                         DBGMCU_CR_TRACE_IOEN_Msk

#define DBGMCU_CR_TRACE_MODE_Pos                     (6U)
#define DBGMCU_CR_TRACE_MODE_Msk                     (0x3UL << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x000000C0 */
#define DBGMCU_CR_TRACE_MODE                         DBGMCU_CR_TRACE_MODE_Msk
#define DBGMCU_CR_TRACE_MODE_0                       (0x1UL << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x00000040 */
#define DBGMCU_CR_TRACE_MODE_1                       (0x2UL << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x00000080 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos             (0U)
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP                 DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos             (1U)
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP                 DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM4_STOP_Pos             (2U)
#define DBGMCU_APB1_FZ_DBG_TIM4_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM4_STOP_Pos) /*!< 0x00000004 */
#define DBGMCU_APB1_FZ_DBG_TIM4_STOP                 DBGMCU_APB1_FZ_DBG_TIM4_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos             (3U)
#define DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos) /*!< 0x00000008 */
#define DBGMCU_APB1_FZ_DBG_TIM5_STOP                 DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos             (4U)
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos) /*!< 0x00000010 */
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP                 DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM7_STOP_Pos             (5U)
#define DBGMCU_APB1_FZ_DBG_TIM7_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_TIM7_STOP_Pos) /*!< 0x00000020 */
#define DBGMCU_APB1_FZ_DBG_TIM7_STOP                 DBGMCU_APB1_FZ_DBG_TIM7_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM12_STOP_Pos            (6U)
#define DBGMCU_APB1_FZ_DBG_TIM12_STOP_Msk            (0x1UL << DBGMCU_APB1_FZ_DBG_TIM12_STOP_Pos) /*!< 0x00000040 */
#define DBGMCU_APB1_FZ_DBG_TIM12_STOP                DBGMCU_APB1_FZ_DBG_TIM12_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM13_STOP_Pos            (7U)
#define DBGMCU_APB1_FZ_DBG_TIM13_STOP_Msk            (0x1UL << DBGMCU_APB1_FZ_DBG_TIM13_STOP_Pos) /*!< 0x00000080 */
#define DBGMCU_APB1_FZ_DBG_TIM13_STOP                DBGMCU_APB1_FZ_DBG_TIM13_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos            (8U)
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk            (0x1UL << DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos) /*!< 0x00000100 */
#define DBGMCU_APB1_FZ_DBG_TIM14_STOP                DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos              (10U)
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk              (0x1UL << DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_APB1_FZ_DBG_RTC_STOP                  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos             (11U)
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP                 DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos             (12U)
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP                 DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos    (21U)
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk    (0x1UL << DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) /*!< 0x00200000 */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk
#define DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos    (22U)
#define DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk    (0x1UL << DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos) /*!< 0x00400000 */
#define DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos    (23U)
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk    (0x1UL << DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos) /*!< 0x00800000 */
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk
#define DBGMCU_APB1_FZ_DBG_CAN1_STOP_Pos             (25U)
#define DBGMCU_APB1_FZ_DBG_CAN1_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_CAN1_STOP_Pos) /*!< 0x02000000 */
#define DBGMCU_APB1_FZ_DBG_CAN1_STOP                 DBGMCU_APB1_FZ_DBG_CAN1_STOP_Msk
#define DBGMCU_APB1_FZ_DBG_CAN2_STOP_Pos             (26U)
#define DBGMCU_APB1_FZ_DBG_CAN2_STOP_Msk             (0x1UL << DBGMCU_APB1_FZ_DBG_CAN2_STOP_Pos) /*!< 0x04000000 */
#define DBGMCU_APB1_FZ_DBG_CAN2_STOP                 DBGMCU_APB1_FZ_DBG_CAN2_STOP_Msk
/* Old IWDGSTOP bit definition, maintained for legacy purpose */
#define  DBGMCU_APB1_FZ_DBG_IWDEG_STOP           DBGMCU_APB1_FZ_DBG_IWDG_STOP

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos             (0U)
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk             (0x1UL << DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP                 DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk
#define DBGMCU_APB2_FZ_DBG_TIM8_STOP_Pos             (1U)
#define DBGMCU_APB2_FZ_DBG_TIM8_STOP_Msk             (0x1UL << DBGMCU_APB2_FZ_DBG_TIM8_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_APB2_FZ_DBG_TIM8_STOP                 DBGMCU_APB2_FZ_DBG_TIM8_STOP_Msk
#define DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos             (16U)
#define DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk             (0x1UL << DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos) /*!< 0x00010000 */
#define DBGMCU_APB2_FZ_DBG_TIM9_STOP                 DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk
#define DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos            (17U)
#define DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk            (0x1UL << DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos) /*!< 0x00020000 */
#define DBGMCU_APB2_FZ_DBG_TIM10_STOP                DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk
#define DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos            (18U)
#define DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk            (0x1UL << DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos) /*!< 0x00040000 */
#define DBGMCU_APB2_FZ_DBG_TIM11_STOP                DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk

typedef struct
{
  volatile uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  volatile uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
}SYSCFG_TypeDef;


#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)


/******************************************************************************/
/*                                                                            */
/*                                       USB_OTG                              */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for USB_OTG_GOTGCTL register  ***********/
#define USB_OTG_GOTGCTL_SRQSCS_Pos               (0U)
#define USB_OTG_GOTGCTL_SRQSCS_Msk               (0x1UL << USB_OTG_GOTGCTL_SRQSCS_Pos) /*!< 0x00000001 */
#define USB_OTG_GOTGCTL_SRQSCS                   USB_OTG_GOTGCTL_SRQSCS_Msk    /*!< Session request success */
#define USB_OTG_GOTGCTL_SRQ_Pos                  (1U)
#define USB_OTG_GOTGCTL_SRQ_Msk                  (0x1UL << USB_OTG_GOTGCTL_SRQ_Pos) /*!< 0x00000002 */
#define USB_OTG_GOTGCTL_SRQ                      USB_OTG_GOTGCTL_SRQ_Msk       /*!< Session request */
#define USB_OTG_GOTGCTL_HNGSCS_Pos               (8U)
#define USB_OTG_GOTGCTL_HNGSCS_Msk               (0x1UL << USB_OTG_GOTGCTL_HNGSCS_Pos) /*!< 0x00000100 */
#define USB_OTG_GOTGCTL_HNGSCS                   USB_OTG_GOTGCTL_HNGSCS_Msk    /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_HNPRQ_Pos                (9U)
#define USB_OTG_GOTGCTL_HNPRQ_Msk                (0x1UL << USB_OTG_GOTGCTL_HNPRQ_Pos) /*!< 0x00000200 */
#define USB_OTG_GOTGCTL_HNPRQ                    USB_OTG_GOTGCTL_HNPRQ_Msk     /*!< HNP request */
#define USB_OTG_GOTGCTL_HSHNPEN_Pos              (10U)
#define USB_OTG_GOTGCTL_HSHNPEN_Msk              (0x1UL << USB_OTG_GOTGCTL_HSHNPEN_Pos) /*!< 0x00000400 */
#define USB_OTG_GOTGCTL_HSHNPEN                  USB_OTG_GOTGCTL_HSHNPEN_Msk   /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_DHNPEN_Pos               (11U)
#define USB_OTG_GOTGCTL_DHNPEN_Msk               (0x1UL << USB_OTG_GOTGCTL_DHNPEN_Pos) /*!< 0x00000800 */
#define USB_OTG_GOTGCTL_DHNPEN                   USB_OTG_GOTGCTL_DHNPEN_Msk    /*!< Device HNP enabled */
#define USB_OTG_GOTGCTL_CIDSTS_Pos               (16U)
#define USB_OTG_GOTGCTL_CIDSTS_Msk               (0x1UL << USB_OTG_GOTGCTL_CIDSTS_Pos) /*!< 0x00010000 */
#define USB_OTG_GOTGCTL_CIDSTS                   USB_OTG_GOTGCTL_CIDSTS_Msk    /*!< Connector ID status */
#define USB_OTG_GOTGCTL_DBCT_Pos                 (17U)
#define USB_OTG_GOTGCTL_DBCT_Msk                 (0x1UL << USB_OTG_GOTGCTL_DBCT_Pos) /*!< 0x00020000 */
#define USB_OTG_GOTGCTL_DBCT                     USB_OTG_GOTGCTL_DBCT_Msk      /*!< Long/short debounce time */
#define USB_OTG_GOTGCTL_ASVLD_Pos                (18U)
#define USB_OTG_GOTGCTL_ASVLD_Msk                (0x1UL << USB_OTG_GOTGCTL_ASVLD_Pos) /*!< 0x00040000 */
#define USB_OTG_GOTGCTL_ASVLD                    USB_OTG_GOTGCTL_ASVLD_Msk     /*!< A-session valid  */
#define USB_OTG_GOTGCTL_BSVLD_Pos                (19U)
#define USB_OTG_GOTGCTL_BSVLD_Msk                (0x1UL << USB_OTG_GOTGCTL_BSVLD_Pos) /*!< 0x00080000 */
#define USB_OTG_GOTGCTL_BSVLD                    USB_OTG_GOTGCTL_BSVLD_Msk     /*!< B-session valid */

/********************  Bit definition forUSB_OTG_HCFG register  ********************/

#define USB_OTG_HCFG_FSLSPCS_Pos                 (0U)
#define USB_OTG_HCFG_FSLSPCS_Msk                 (0x3UL << USB_OTG_HCFG_FSLSPCS_Pos) /*!< 0x00000003 */
#define USB_OTG_HCFG_FSLSPCS                     USB_OTG_HCFG_FSLSPCS_Msk      /*!< FS/LS PHY clock select  */
#define USB_OTG_HCFG_FSLSPCS_0                   (0x1UL << USB_OTG_HCFG_FSLSPCS_Pos) /*!< 0x00000001 */
#define USB_OTG_HCFG_FSLSPCS_1                   (0x2UL << USB_OTG_HCFG_FSLSPCS_Pos) /*!< 0x00000002 */
#define USB_OTG_HCFG_FSLSS_Pos                   (2U)
#define USB_OTG_HCFG_FSLSS_Msk                   (0x1UL << USB_OTG_HCFG_FSLSS_Pos) /*!< 0x00000004 */
#define USB_OTG_HCFG_FSLSS                       USB_OTG_HCFG_FSLSS_Msk        /*!< FS- and LS-only support */

/********************  Bit definition for USB_OTG_DCFG register  ********************/

#define USB_OTG_DCFG_DSPD_Pos                    (0U)
#define USB_OTG_DCFG_DSPD_Msk                    (0x3UL << USB_OTG_DCFG_DSPD_Pos) /*!< 0x00000003 */
#define USB_OTG_DCFG_DSPD                        USB_OTG_DCFG_DSPD_Msk         /*!< Device speed */
#define USB_OTG_DCFG_DSPD_0                      (0x1UL << USB_OTG_DCFG_DSPD_Pos) /*!< 0x00000001 */
#define USB_OTG_DCFG_DSPD_1                      (0x2UL << USB_OTG_DCFG_DSPD_Pos) /*!< 0x00000002 */
#define USB_OTG_DCFG_NZLSOHSK_Pos                (2U)
#define USB_OTG_DCFG_NZLSOHSK_Msk                (0x1UL << USB_OTG_DCFG_NZLSOHSK_Pos) /*!< 0x00000004 */
#define USB_OTG_DCFG_NZLSOHSK                    USB_OTG_DCFG_NZLSOHSK_Msk     /*!< Nonzero-length status OUT handshake */

#define USB_OTG_DCFG_DAD_Pos                     (4U)
#define USB_OTG_DCFG_DAD_Msk                     (0x7FUL << USB_OTG_DCFG_DAD_Pos) /*!< 0x000007F0 */
#define USB_OTG_DCFG_DAD                         USB_OTG_DCFG_DAD_Msk          /*!< Device address */
#define USB_OTG_DCFG_DAD_0                       (0x01UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000010 */
#define USB_OTG_DCFG_DAD_1                       (0x02UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000020 */
#define USB_OTG_DCFG_DAD_2                       (0x04UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000040 */
#define USB_OTG_DCFG_DAD_3                       (0x08UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000080 */
#define USB_OTG_DCFG_DAD_4                       (0x10UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000100 */
#define USB_OTG_DCFG_DAD_5                       (0x20UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000200 */
#define USB_OTG_DCFG_DAD_6                       (0x40UL << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000400 */

#define USB_OTG_DCFG_PFIVL_Pos                   (11U)
#define USB_OTG_DCFG_PFIVL_Msk                   (0x3UL << USB_OTG_DCFG_PFIVL_Pos) /*!< 0x00001800 */
#define USB_OTG_DCFG_PFIVL                       USB_OTG_DCFG_PFIVL_Msk        /*!< Periodic (micro)frame interval */
#define USB_OTG_DCFG_PFIVL_0                     (0x1UL << USB_OTG_DCFG_PFIVL_Pos) /*!< 0x00000800 */
#define USB_OTG_DCFG_PFIVL_1                     (0x2UL << USB_OTG_DCFG_PFIVL_Pos) /*!< 0x00001000 */

#define USB_OTG_DCFG_XCVRDLY_Pos                 (14U)
#define USB_OTG_DCFG_XCVRDLY_Msk                 (0x1UL << USB_OTG_DCFG_XCVRDLY_Pos) /*!< 0x00004000 */
#define USB_OTG_DCFG_XCVRDLY                     USB_OTG_DCFG_XCVRDLY_Msk        /*!< Transceiver delay */

#define USB_OTG_DCFG_ERRATIM_Pos                 (15U)
#define USB_OTG_DCFG_ERRATIM_Msk                 (0x1UL << USB_OTG_DCFG_ERRATIM_Pos) /*!< 0x00008000 */
#define USB_OTG_DCFG_ERRATIM                     USB_OTG_DCFG_ERRATIM_Msk        /*!< Erratic error interrupt mask */

#define USB_OTG_DCFG_PERSCHIVL_Pos               (24U)
#define USB_OTG_DCFG_PERSCHIVL_Msk               (0x3UL << USB_OTG_DCFG_PERSCHIVL_Pos) /*!< 0x03000000 */
#define USB_OTG_DCFG_PERSCHIVL                   USB_OTG_DCFG_PERSCHIVL_Msk    /*!< Periodic scheduling interval */
#define USB_OTG_DCFG_PERSCHIVL_0                 (0x1UL << USB_OTG_DCFG_PERSCHIVL_Pos) /*!< 0x01000000 */
#define USB_OTG_DCFG_PERSCHIVL_1                 (0x2UL << USB_OTG_DCFG_PERSCHIVL_Pos) /*!< 0x02000000 */

/********************  Bit definition for USB_OTG_PCGCR register  ********************/
#define USB_OTG_PCGCR_STPPCLK_Pos                (0U)
#define USB_OTG_PCGCR_STPPCLK_Msk                (0x1UL << USB_OTG_PCGCR_STPPCLK_Pos) /*!< 0x00000001 */
#define USB_OTG_PCGCR_STPPCLK                    USB_OTG_PCGCR_STPPCLK_Msk     /*!< Stop PHY clock */
#define USB_OTG_PCGCR_GATEHCLK_Pos               (1U)
#define USB_OTG_PCGCR_GATEHCLK_Msk               (0x1UL << USB_OTG_PCGCR_GATEHCLK_Pos) /*!< 0x00000002 */
#define USB_OTG_PCGCR_GATEHCLK                   USB_OTG_PCGCR_GATEHCLK_Msk    /*!< Gate HCLK */
#define USB_OTG_PCGCR_PHYSUSP_Pos                (4U)
#define USB_OTG_PCGCR_PHYSUSP_Msk                (0x1UL << USB_OTG_PCGCR_PHYSUSP_Pos) /*!< 0x00000010 */
#define USB_OTG_PCGCR_PHYSUSP                    USB_OTG_PCGCR_PHYSUSP_Msk     /*!< PHY suspended */

/********************  Bit definition for USB_OTG_GOTGINT register  ********************/
#define USB_OTG_GOTGINT_SEDET_Pos                (2U)
#define USB_OTG_GOTGINT_SEDET_Msk                (0x1UL << USB_OTG_GOTGINT_SEDET_Pos) /*!< 0x00000004 */
#define USB_OTG_GOTGINT_SEDET                    USB_OTG_GOTGINT_SEDET_Msk     /*!< Session end detected                   */
#define USB_OTG_GOTGINT_SRSSCHG_Pos              (8U)
#define USB_OTG_GOTGINT_SRSSCHG_Msk              (0x1UL << USB_OTG_GOTGINT_SRSSCHG_Pos) /*!< 0x00000100 */
#define USB_OTG_GOTGINT_SRSSCHG                  USB_OTG_GOTGINT_SRSSCHG_Msk   /*!< Session request success status change  */
#define USB_OTG_GOTGINT_HNSSCHG_Pos              (9U)
#define USB_OTG_GOTGINT_HNSSCHG_Msk              (0x1UL << USB_OTG_GOTGINT_HNSSCHG_Pos) /*!< 0x00000200 */
#define USB_OTG_GOTGINT_HNSSCHG                  USB_OTG_GOTGINT_HNSSCHG_Msk   /*!< Host negotiation success status change */
#define USB_OTG_GOTGINT_HNGDET_Pos               (17U)
#define USB_OTG_GOTGINT_HNGDET_Msk               (0x1UL << USB_OTG_GOTGINT_HNGDET_Pos) /*!< 0x00020000 */
#define USB_OTG_GOTGINT_HNGDET                   USB_OTG_GOTGINT_HNGDET_Msk    /*!< Host negotiation detected              */
#define USB_OTG_GOTGINT_ADTOCHG_Pos              (18U)
#define USB_OTG_GOTGINT_ADTOCHG_Msk              (0x1UL << USB_OTG_GOTGINT_ADTOCHG_Pos) /*!< 0x00040000 */
#define USB_OTG_GOTGINT_ADTOCHG                  USB_OTG_GOTGINT_ADTOCHG_Msk   /*!< A-device timeout change                */
#define USB_OTG_GOTGINT_DBCDNE_Pos               (19U)
#define USB_OTG_GOTGINT_DBCDNE_Msk               (0x1UL << USB_OTG_GOTGINT_DBCDNE_Pos) /*!< 0x00080000 */
#define USB_OTG_GOTGINT_DBCDNE                   USB_OTG_GOTGINT_DBCDNE_Msk    /*!< Debounce done                          */

/********************  Bit definition for USB_OTG_DCTL register  ********************/
#define USB_OTG_DCTL_RWUSIG_Pos                  (0U)
#define USB_OTG_DCTL_RWUSIG_Msk                  (0x1UL << USB_OTG_DCTL_RWUSIG_Pos) /*!< 0x00000001 */
#define USB_OTG_DCTL_RWUSIG                      USB_OTG_DCTL_RWUSIG_Msk       /*!< Remote wakeup signaling */
#define USB_OTG_DCTL_SDIS_Pos                    (1U)
#define USB_OTG_DCTL_SDIS_Msk                    (0x1UL << USB_OTG_DCTL_SDIS_Pos) /*!< 0x00000002 */
#define USB_OTG_DCTL_SDIS                        USB_OTG_DCTL_SDIS_Msk         /*!< Soft disconnect         */
#define USB_OTG_DCTL_GINSTS_Pos                  (2U)
#define USB_OTG_DCTL_GINSTS_Msk                  (0x1UL << USB_OTG_DCTL_GINSTS_Pos) /*!< 0x00000004 */
#define USB_OTG_DCTL_GINSTS                      USB_OTG_DCTL_GINSTS_Msk       /*!< Global IN NAK status    */
#define USB_OTG_DCTL_GONSTS_Pos                  (3U)
#define USB_OTG_DCTL_GONSTS_Msk                  (0x1UL << USB_OTG_DCTL_GONSTS_Pos) /*!< 0x00000008 */
#define USB_OTG_DCTL_GONSTS                      USB_OTG_DCTL_GONSTS_Msk       /*!< Global OUT NAK status   */

#define USB_OTG_DCTL_TCTL_Pos                    (4U)
#define USB_OTG_DCTL_TCTL_Msk                    (0x7UL << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000070 */
#define USB_OTG_DCTL_TCTL                        USB_OTG_DCTL_TCTL_Msk         /*!< Test control */
#define USB_OTG_DCTL_TCTL_0                      (0x1UL << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000010 */
#define USB_OTG_DCTL_TCTL_1                      (0x2UL << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000020 */
#define USB_OTG_DCTL_TCTL_2                      (0x4UL << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000040 */
#define USB_OTG_DCTL_SGINAK_Pos                  (7U)
#define USB_OTG_DCTL_SGINAK_Msk                  (0x1UL << USB_OTG_DCTL_SGINAK_Pos) /*!< 0x00000080 */
#define USB_OTG_DCTL_SGINAK                      USB_OTG_DCTL_SGINAK_Msk       /*!< Set global IN NAK         */
#define USB_OTG_DCTL_CGINAK_Pos                  (8U)
#define USB_OTG_DCTL_CGINAK_Msk                  (0x1UL << USB_OTG_DCTL_CGINAK_Pos) /*!< 0x00000100 */
#define USB_OTG_DCTL_CGINAK                      USB_OTG_DCTL_CGINAK_Msk       /*!< Clear global IN NAK       */
#define USB_OTG_DCTL_SGONAK_Pos                  (9U)
#define USB_OTG_DCTL_SGONAK_Msk                  (0x1UL << USB_OTG_DCTL_SGONAK_Pos) /*!< 0x00000200 */
#define USB_OTG_DCTL_SGONAK                      USB_OTG_DCTL_SGONAK_Msk       /*!< Set global OUT NAK        */
#define USB_OTG_DCTL_CGONAK_Pos                  (10U)
#define USB_OTG_DCTL_CGONAK_Msk                  (0x1UL << USB_OTG_DCTL_CGONAK_Pos) /*!< 0x00000400 */
#define USB_OTG_DCTL_CGONAK                      USB_OTG_DCTL_CGONAK_Msk       /*!< Clear global OUT NAK      */
#define USB_OTG_DCTL_POPRGDNE_Pos                (11U)
#define USB_OTG_DCTL_POPRGDNE_Msk                (0x1UL << USB_OTG_DCTL_POPRGDNE_Pos) /*!< 0x00000800 */
#define USB_OTG_DCTL_POPRGDNE                    USB_OTG_DCTL_POPRGDNE_Msk     /*!< Power-on programming done */

/********************  Bit definition for USB_OTG_HFIR register  ********************/
#define USB_OTG_HFIR_FRIVL_Pos                   (0U)
#define USB_OTG_HFIR_FRIVL_Msk                   (0xFFFFUL << USB_OTG_HFIR_FRIVL_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HFIR_FRIVL                       USB_OTG_HFIR_FRIVL_Msk        /*!< Frame interval */

/********************  Bit definition for USB_OTG_HFNUM register  ********************/
#define USB_OTG_HFNUM_FRNUM_Pos                  (0U)
#define USB_OTG_HFNUM_FRNUM_Msk                  (0xFFFFUL << USB_OTG_HFNUM_FRNUM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HFNUM_FRNUM                      USB_OTG_HFNUM_FRNUM_Msk       /*!< Frame number         */
#define USB_OTG_HFNUM_FTREM_Pos                  (16U)
#define USB_OTG_HFNUM_FTREM_Msk                  (0xFFFFUL << USB_OTG_HFNUM_FTREM_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_HFNUM_FTREM                      USB_OTG_HFNUM_FTREM_Msk       /*!< Frame time remaining */

/********************  Bit definition for USB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_SUSPSTS_Pos                 (0U)
#define USB_OTG_DSTS_SUSPSTS_Msk                 (0x1UL << USB_OTG_DSTS_SUSPSTS_Pos) /*!< 0x00000001 */
#define USB_OTG_DSTS_SUSPSTS                     USB_OTG_DSTS_SUSPSTS_Msk      /*!< Suspend status   */

#define USB_OTG_DSTS_ENUMSPD_Pos                 (1U)
#define USB_OTG_DSTS_ENUMSPD_Msk                 (0x3UL << USB_OTG_DSTS_ENUMSPD_Pos) /*!< 0x00000006 */
#define USB_OTG_DSTS_ENUMSPD                     USB_OTG_DSTS_ENUMSPD_Msk      /*!< Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD_0                   (0x1UL << USB_OTG_DSTS_ENUMSPD_Pos) /*!< 0x00000002 */
#define USB_OTG_DSTS_ENUMSPD_1                   (0x2UL << USB_OTG_DSTS_ENUMSPD_Pos) /*!< 0x00000004 */
#define USB_OTG_DSTS_EERR_Pos                    (3U)
#define USB_OTG_DSTS_EERR_Msk                    (0x1UL << USB_OTG_DSTS_EERR_Pos) /*!< 0x00000008 */
#define USB_OTG_DSTS_EERR                        USB_OTG_DSTS_EERR_Msk         /*!< Erratic error     */
#define USB_OTG_DSTS_FNSOF_Pos                   (8U)
#define USB_OTG_DSTS_FNSOF_Msk                   (0x3FFFUL << USB_OTG_DSTS_FNSOF_Pos) /*!< 0x003FFF00 */
#define USB_OTG_DSTS_FNSOF                       USB_OTG_DSTS_FNSOF_Msk        /*!< Frame number of the received SOF */

/********************  Bit definition for USB_OTG_GAHBCFG register  ********************/
#define USB_OTG_GAHBCFG_GINT_Pos                 (0U)
#define USB_OTG_GAHBCFG_GINT_Msk                 (0x1UL << USB_OTG_GAHBCFG_GINT_Pos) /*!< 0x00000001 */
#define USB_OTG_GAHBCFG_GINT                     USB_OTG_GAHBCFG_GINT_Msk      /*!< Global interrupt mask */
#define USB_OTG_GAHBCFG_HBSTLEN_Pos              (1U)
#define USB_OTG_GAHBCFG_HBSTLEN_Msk              (0xFUL << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< 0x0000001E */
#define USB_OTG_GAHBCFG_HBSTLEN                  USB_OTG_GAHBCFG_HBSTLEN_Msk   /*!< Burst length/type */
#define USB_OTG_GAHBCFG_HBSTLEN_0                (0x0UL << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< Single */
#define USB_OTG_GAHBCFG_HBSTLEN_1                (0x1UL << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR */
#define USB_OTG_GAHBCFG_HBSTLEN_2                (0x3UL << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR4 */
#define USB_OTG_GAHBCFG_HBSTLEN_3                (0x5UL << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR8 */
#define USB_OTG_GAHBCFG_HBSTLEN_4                (0x7UL << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR16 */
#define USB_OTG_GAHBCFG_DMAEN_Pos                (5U)
#define USB_OTG_GAHBCFG_DMAEN_Msk                (0x1UL << USB_OTG_GAHBCFG_DMAEN_Pos) /*!< 0x00000020 */
#define USB_OTG_GAHBCFG_DMAEN                    USB_OTG_GAHBCFG_DMAEN_Msk     /*!< DMA enable */
#define USB_OTG_GAHBCFG_TXFELVL_Pos              (7U)
#define USB_OTG_GAHBCFG_TXFELVL_Msk              (0x1UL << USB_OTG_GAHBCFG_TXFELVL_Pos) /*!< 0x00000080 */
#define USB_OTG_GAHBCFG_TXFELVL                  USB_OTG_GAHBCFG_TXFELVL_Msk   /*!< TxFIFO empty level */
#define USB_OTG_GAHBCFG_PTXFELVL_Pos             (8U)
#define USB_OTG_GAHBCFG_PTXFELVL_Msk             (0x1UL << USB_OTG_GAHBCFG_PTXFELVL_Pos) /*!< 0x00000100 */
#define USB_OTG_GAHBCFG_PTXFELVL                 USB_OTG_GAHBCFG_PTXFELVL_Msk  /*!< Periodic TxFIFO empty level */

/********************  Bit definition for USB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL_Pos                (0U)
#define USB_OTG_GUSBCFG_TOCAL_Msk                (0x7UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000007 */
#define USB_OTG_GUSBCFG_TOCAL                    USB_OTG_GUSBCFG_TOCAL_Msk     /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                  (0x1UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000001 */
#define USB_OTG_GUSBCFG_TOCAL_1                  (0x2UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000002 */
#define USB_OTG_GUSBCFG_TOCAL_2                  (0x4UL << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000004 */
#define USB_OTG_GUSBCFG_PHYSEL_Pos               (6U)
#define USB_OTG_GUSBCFG_PHYSEL_Msk               (0x1UL << USB_OTG_GUSBCFG_PHYSEL_Pos) /*!< 0x00000040 */
#define USB_OTG_GUSBCFG_PHYSEL                   USB_OTG_GUSBCFG_PHYSEL_Msk    /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP_Pos               (8U)
#define USB_OTG_GUSBCFG_SRPCAP_Msk               (0x1UL << USB_OTG_GUSBCFG_SRPCAP_Pos) /*!< 0x00000100 */
#define USB_OTG_GUSBCFG_SRPCAP                   USB_OTG_GUSBCFG_SRPCAP_Msk    /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP_Pos               (9U)
#define USB_OTG_GUSBCFG_HNPCAP_Msk               (0x1UL << USB_OTG_GUSBCFG_HNPCAP_Pos) /*!< 0x00000200 */
#define USB_OTG_GUSBCFG_HNPCAP                   USB_OTG_GUSBCFG_HNPCAP_Msk    /*!< HNP-capable */
#define USB_OTG_GUSBCFG_TRDT_Pos                 (10U)
#define USB_OTG_GUSBCFG_TRDT_Msk                 (0xFUL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00003C00 */
#define USB_OTG_GUSBCFG_TRDT                     USB_OTG_GUSBCFG_TRDT_Msk      /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                   (0x1UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00000400 */
#define USB_OTG_GUSBCFG_TRDT_1                   (0x2UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00000800 */
#define USB_OTG_GUSBCFG_TRDT_2                   (0x4UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00001000 */
#define USB_OTG_GUSBCFG_TRDT_3                   (0x8UL << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00002000 */
#define USB_OTG_GUSBCFG_PHYLPCS_Pos              (15U)
#define USB_OTG_GUSBCFG_PHYLPCS_Msk              (0x1UL << USB_OTG_GUSBCFG_PHYLPCS_Pos) /*!< 0x00008000 */
#define USB_OTG_GUSBCFG_PHYLPCS                  USB_OTG_GUSBCFG_PHYLPCS_Msk   /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS_Pos             (17U)
#define USB_OTG_GUSBCFG_ULPIFSLS_Msk             (0x1UL << USB_OTG_GUSBCFG_ULPIFSLS_Pos) /*!< 0x00020000 */
#define USB_OTG_GUSBCFG_ULPIFSLS                 USB_OTG_GUSBCFG_ULPIFSLS_Msk  /*!< ULPI FS/LS select               */
#define USB_OTG_GUSBCFG_ULPIAR_Pos               (18U)
#define USB_OTG_GUSBCFG_ULPIAR_Msk               (0x1UL << USB_OTG_GUSBCFG_ULPIAR_Pos) /*!< 0x00040000 */
#define USB_OTG_GUSBCFG_ULPIAR                   USB_OTG_GUSBCFG_ULPIAR_Msk    /*!< ULPI Auto-resume                */
#define USB_OTG_GUSBCFG_ULPICSM_Pos              (19U)
#define USB_OTG_GUSBCFG_ULPICSM_Msk              (0x1UL << USB_OTG_GUSBCFG_ULPICSM_Pos) /*!< 0x00080000 */
#define USB_OTG_GUSBCFG_ULPICSM                  USB_OTG_GUSBCFG_ULPICSM_Msk   /*!< ULPI Clock SuspendM             */
#define USB_OTG_GUSBCFG_ULPIEVBUSD_Pos           (20U)
#define USB_OTG_GUSBCFG_ULPIEVBUSD_Msk           (0x1UL << USB_OTG_GUSBCFG_ULPIEVBUSD_Pos) /*!< 0x00100000 */
#define USB_OTG_GUSBCFG_ULPIEVBUSD               USB_OTG_GUSBCFG_ULPIEVBUSD_Msk /*!< ULPI External VBUS Drive        */
#define USB_OTG_GUSBCFG_ULPIEVBUSI_Pos           (21U)
#define USB_OTG_GUSBCFG_ULPIEVBUSI_Msk           (0x1UL << USB_OTG_GUSBCFG_ULPIEVBUSI_Pos) /*!< 0x00200000 */
#define USB_OTG_GUSBCFG_ULPIEVBUSI               USB_OTG_GUSBCFG_ULPIEVBUSI_Msk /*!< ULPI external VBUS indicator    */
#define USB_OTG_GUSBCFG_TSDPS_Pos                (22U)
#define USB_OTG_GUSBCFG_TSDPS_Msk                (0x1UL << USB_OTG_GUSBCFG_TSDPS_Pos) /*!< 0x00400000 */
#define USB_OTG_GUSBCFG_TSDPS                    USB_OTG_GUSBCFG_TSDPS_Msk     /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI_Pos                 (23U)
#define USB_OTG_GUSBCFG_PCCI_Msk                 (0x1UL << USB_OTG_GUSBCFG_PCCI_Pos) /*!< 0x00800000 */
#define USB_OTG_GUSBCFG_PCCI                     USB_OTG_GUSBCFG_PCCI_Msk      /*!< Indicator complement            */
#define USB_OTG_GUSBCFG_PTCI_Pos                 (24U)
#define USB_OTG_GUSBCFG_PTCI_Msk                 (0x1UL << USB_OTG_GUSBCFG_PTCI_Pos) /*!< 0x01000000 */
#define USB_OTG_GUSBCFG_PTCI                     USB_OTG_GUSBCFG_PTCI_Msk      /*!< Indicator pass through          */
#define USB_OTG_GUSBCFG_ULPIIPD_Pos              (25U)
#define USB_OTG_GUSBCFG_ULPIIPD_Msk              (0x1UL << USB_OTG_GUSBCFG_ULPIIPD_Pos) /*!< 0x02000000 */
#define USB_OTG_GUSBCFG_ULPIIPD                  USB_OTG_GUSBCFG_ULPIIPD_Msk   /*!< ULPI interface protect disable  */
#define USB_OTG_GUSBCFG_FHMOD_Pos                (29U)
#define USB_OTG_GUSBCFG_FHMOD_Msk                (0x1UL << USB_OTG_GUSBCFG_FHMOD_Pos) /*!< 0x20000000 */
#define USB_OTG_GUSBCFG_FHMOD                    USB_OTG_GUSBCFG_FHMOD_Msk     /*!< Forced host mode                */
#define USB_OTG_GUSBCFG_FDMOD_Pos                (30U)
#define USB_OTG_GUSBCFG_FDMOD_Msk                (0x1UL << USB_OTG_GUSBCFG_FDMOD_Pos) /*!< 0x40000000 */
#define USB_OTG_GUSBCFG_FDMOD                    USB_OTG_GUSBCFG_FDMOD_Msk     /*!< Forced peripheral mode          */
#define USB_OTG_GUSBCFG_CTXPKT_Pos               (31U)
#define USB_OTG_GUSBCFG_CTXPKT_Msk               (0x1UL << USB_OTG_GUSBCFG_CTXPKT_Pos) /*!< 0x80000000 */
#define USB_OTG_GUSBCFG_CTXPKT                   USB_OTG_GUSBCFG_CTXPKT_Msk    /*!< Corrupt Tx packet               */

/********************  Bit definition for USB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_CSRST_Pos                (0U)
#define USB_OTG_GRSTCTL_CSRST_Msk                (0x1UL << USB_OTG_GRSTCTL_CSRST_Pos) /*!< 0x00000001 */
#define USB_OTG_GRSTCTL_CSRST                    USB_OTG_GRSTCTL_CSRST_Msk     /*!< Core soft reset          */
#define USB_OTG_GRSTCTL_HSRST_Pos                (1U)
#define USB_OTG_GRSTCTL_HSRST_Msk                (0x1UL << USB_OTG_GRSTCTL_HSRST_Pos) /*!< 0x00000002 */
#define USB_OTG_GRSTCTL_HSRST                    USB_OTG_GRSTCTL_HSRST_Msk     /*!< HCLK soft reset          */
#define USB_OTG_GRSTCTL_FCRST_Pos                (2U)
#define USB_OTG_GRSTCTL_FCRST_Msk                (0x1UL << USB_OTG_GRSTCTL_FCRST_Pos) /*!< 0x00000004 */
#define USB_OTG_GRSTCTL_FCRST                    USB_OTG_GRSTCTL_FCRST_Msk     /*!< Host frame counter reset */
#define USB_OTG_GRSTCTL_RXFFLSH_Pos              (4U)
#define USB_OTG_GRSTCTL_RXFFLSH_Msk              (0x1UL << USB_OTG_GRSTCTL_RXFFLSH_Pos) /*!< 0x00000010 */
#define USB_OTG_GRSTCTL_RXFFLSH                  USB_OTG_GRSTCTL_RXFFLSH_Msk   /*!< RxFIFO flush             */
#define USB_OTG_GRSTCTL_TXFFLSH_Pos              (5U)
#define USB_OTG_GRSTCTL_TXFFLSH_Msk              (0x1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos) /*!< 0x00000020 */
#define USB_OTG_GRSTCTL_TXFFLSH                  USB_OTG_GRSTCTL_TXFFLSH_Msk   /*!< TxFIFO flush             */


#define USB_OTG_GRSTCTL_TXFNUM_Pos               (6U)
#define USB_OTG_GRSTCTL_TXFNUM_Msk               (0x1FUL << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x000007C0 */
#define USB_OTG_GRSTCTL_TXFNUM                   USB_OTG_GRSTCTL_TXFNUM_Msk    /*!< TxFIFO number */
#define USB_OTG_GRSTCTL_TXFNUM_0                 (0x01UL << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000040 */
#define USB_OTG_GRSTCTL_TXFNUM_1                 (0x02UL << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000080 */
#define USB_OTG_GRSTCTL_TXFNUM_2                 (0x04UL << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000100 */
#define USB_OTG_GRSTCTL_TXFNUM_3                 (0x08UL << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000200 */
#define USB_OTG_GRSTCTL_TXFNUM_4                 (0x10UL << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000400 */
#define USB_OTG_GRSTCTL_DMAREQ_Pos               (30U)
#define USB_OTG_GRSTCTL_DMAREQ_Msk               (0x1UL << USB_OTG_GRSTCTL_DMAREQ_Pos) /*!< 0x40000000 */
#define USB_OTG_GRSTCTL_DMAREQ                   USB_OTG_GRSTCTL_DMAREQ_Msk    /*!< DMA request signal */
#define USB_OTG_GRSTCTL_AHBIDL_Pos               (31U)
#define USB_OTG_GRSTCTL_AHBIDL_Msk               (0x1UL << USB_OTG_GRSTCTL_AHBIDL_Pos) /*!< 0x80000000 */
#define USB_OTG_GRSTCTL_AHBIDL                   USB_OTG_GRSTCTL_AHBIDL_Msk    /*!< AHB master idle */

/********************  Bit definition for USB_OTG_DIEPMSK register  ********************/
#define USB_OTG_DIEPMSK_XFRCM_Pos                (0U)
#define USB_OTG_DIEPMSK_XFRCM_Msk                (0x1UL << USB_OTG_DIEPMSK_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DIEPMSK_XFRCM                    USB_OTG_DIEPMSK_XFRCM_Msk     /*!< Transfer completed interrupt mask                 */
#define USB_OTG_DIEPMSK_EPDM_Pos                 (1U)
#define USB_OTG_DIEPMSK_EPDM_Msk                 (0x1UL << USB_OTG_DIEPMSK_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DIEPMSK_EPDM                     USB_OTG_DIEPMSK_EPDM_Msk      /*!< Endpoint disabled interrupt mask                  */
#define USB_OTG_DIEPMSK_TOM_Pos                  (3U)
#define USB_OTG_DIEPMSK_TOM_Msk                  (0x1UL << USB_OTG_DIEPMSK_TOM_Pos) /*!< 0x00000008 */
#define USB_OTG_DIEPMSK_TOM                      USB_OTG_DIEPMSK_TOM_Msk       /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPMSK_ITTXFEMSK_Pos            (4U)
#define USB_OTG_DIEPMSK_ITTXFEMSK_Msk            (0x1UL << USB_OTG_DIEPMSK_ITTXFEMSK_Pos) /*!< 0x00000010 */
#define USB_OTG_DIEPMSK_ITTXFEMSK                USB_OTG_DIEPMSK_ITTXFEMSK_Msk /*!< IN token received when TxFIFO empty mask          */
#define USB_OTG_DIEPMSK_INEPNMM_Pos              (5U)
#define USB_OTG_DIEPMSK_INEPNMM_Msk              (0x1UL << USB_OTG_DIEPMSK_INEPNMM_Pos) /*!< 0x00000020 */
#define USB_OTG_DIEPMSK_INEPNMM                  USB_OTG_DIEPMSK_INEPNMM_Msk   /*!< IN token received with EP mismatch mask           */
#define USB_OTG_DIEPMSK_INEPNEM_Pos              (6U)
#define USB_OTG_DIEPMSK_INEPNEM_Msk              (0x1UL << USB_OTG_DIEPMSK_INEPNEM_Pos) /*!< 0x00000040 */
#define USB_OTG_DIEPMSK_INEPNEM                  USB_OTG_DIEPMSK_INEPNEM_Msk   /*!< IN endpoint NAK effective mask                    */
#define USB_OTG_DIEPMSK_TXFURM_Pos               (8U)
#define USB_OTG_DIEPMSK_TXFURM_Msk               (0x1UL << USB_OTG_DIEPMSK_TXFURM_Pos) /*!< 0x00000100 */
#define USB_OTG_DIEPMSK_TXFURM                   USB_OTG_DIEPMSK_TXFURM_Msk    /*!< FIFO underrun mask                                */
#define USB_OTG_DIEPMSK_BIM_Pos                  (9U)
#define USB_OTG_DIEPMSK_BIM_Msk                  (0x1UL << USB_OTG_DIEPMSK_BIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DIEPMSK_BIM                      USB_OTG_DIEPMSK_BIM_Msk       /*!< BNA interrupt mask                                */

/********************  Bit definition for USB_OTG_HPTXSTS register  ********************/
#define USB_OTG_HPTXSTS_PTXFSAVL_Pos             (0U)
#define USB_OTG_HPTXSTS_PTXFSAVL_Msk             (0xFFFFUL << USB_OTG_HPTXSTS_PTXFSAVL_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HPTXSTS_PTXFSAVL                 USB_OTG_HPTXSTS_PTXFSAVL_Msk  /*!< Periodic transmit data FIFO space available     */
#define USB_OTG_HPTXSTS_PTXQSAV_Pos              (16U)
#define USB_OTG_HPTXSTS_PTXQSAV_Msk              (0xFFUL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00FF0000 */
#define USB_OTG_HPTXSTS_PTXQSAV                  USB_OTG_HPTXSTS_PTXQSAV_Msk   /*!< Periodic transmit request queue space available */
#define USB_OTG_HPTXSTS_PTXQSAV_0                (0x01UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00010000 */
#define USB_OTG_HPTXSTS_PTXQSAV_1                (0x02UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00020000 */
#define USB_OTG_HPTXSTS_PTXQSAV_2                (0x04UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00040000 */
#define USB_OTG_HPTXSTS_PTXQSAV_3                (0x08UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00080000 */
#define USB_OTG_HPTXSTS_PTXQSAV_4                (0x10UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00100000 */
#define USB_OTG_HPTXSTS_PTXQSAV_5                (0x20UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00200000 */
#define USB_OTG_HPTXSTS_PTXQSAV_6                (0x40UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00400000 */
#define USB_OTG_HPTXSTS_PTXQSAV_7                (0x80UL << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00800000 */

#define USB_OTG_HPTXSTS_PTXQTOP_Pos              (24U)
#define USB_OTG_HPTXSTS_PTXQTOP_Msk              (0xFFUL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0xFF000000 */
#define USB_OTG_HPTXSTS_PTXQTOP                  USB_OTG_HPTXSTS_PTXQTOP_Msk   /*!< Top of the periodic transmit request queue */
#define USB_OTG_HPTXSTS_PTXQTOP_0                (0x01UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x01000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_1                (0x02UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x02000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_2                (0x04UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x04000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_3                (0x08UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x08000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_4                (0x10UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x10000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_5                (0x20UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x20000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_6                (0x40UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x40000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_7                (0x80UL << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x80000000 */

/********************  Bit definition for USB_OTG_HAINT register  ********************/
#define USB_OTG_HAINT_HAINT_Pos                  (0U)
#define USB_OTG_HAINT_HAINT_Msk                  (0xFFFFUL << USB_OTG_HAINT_HAINT_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HAINT_HAINT                      USB_OTG_HAINT_HAINT_Msk       /*!< Channel interrupts */

/********************  Bit definition for USB_OTG_DOEPMSK register  ********************/
#define USB_OTG_DOEPMSK_XFRCM_Pos                (0U)
#define USB_OTG_DOEPMSK_XFRCM_Msk                (0x1UL << USB_OTG_DOEPMSK_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DOEPMSK_XFRCM                    USB_OTG_DOEPMSK_XFRCM_Msk     /*!< Transfer completed interrupt mask              */
#define USB_OTG_DOEPMSK_EPDM_Pos                 (1U)
#define USB_OTG_DOEPMSK_EPDM_Msk                 (0x1UL << USB_OTG_DOEPMSK_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DOEPMSK_EPDM                     USB_OTG_DOEPMSK_EPDM_Msk      /*!< Endpoint disabled interrupt mask               */
#define USB_OTG_DOEPMSK_AHBERRM_Pos              (2U)
#define USB_OTG_DOEPMSK_AHBERRM_Msk              (0x1UL << USB_OTG_DOEPMSK_AHBERRM_Pos) /*!< 0x00000004 */
#define USB_OTG_DOEPMSK_AHBERRM                  USB_OTG_DOEPMSK_AHBERRM_Msk   /*!< OUT transaction AHB Error interrupt mask       */
#define USB_OTG_DOEPMSK_STUPM_Pos                (3U)
#define USB_OTG_DOEPMSK_STUPM_Msk                (0x1UL << USB_OTG_DOEPMSK_STUPM_Pos) /*!< 0x00000008 */
#define USB_OTG_DOEPMSK_STUPM                    USB_OTG_DOEPMSK_STUPM_Msk     /*!< SETUP phase done mask                          */
#define USB_OTG_DOEPMSK_OTEPDM_Pos               (4U)
#define USB_OTG_DOEPMSK_OTEPDM_Msk               (0x1UL << USB_OTG_DOEPMSK_OTEPDM_Pos) /*!< 0x00000010 */
#define USB_OTG_DOEPMSK_OTEPDM                   USB_OTG_DOEPMSK_OTEPDM_Msk    /*!< OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_OTEPSPRM_Pos             (5U)
#define USB_OTG_DOEPMSK_OTEPSPRM_Msk             (0x1UL << USB_OTG_DOEPMSK_OTEPSPRM_Pos) /*!< 0x00000020 */
#define USB_OTG_DOEPMSK_OTEPSPRM                 USB_OTG_DOEPMSK_OTEPSPRM_Msk  /*!< Status Phase Received mask                     */
#define USB_OTG_DOEPMSK_B2BSTUP_Pos              (6U)
#define USB_OTG_DOEPMSK_B2BSTUP_Msk              (0x1UL << USB_OTG_DOEPMSK_B2BSTUP_Pos) /*!< 0x00000040 */
#define USB_OTG_DOEPMSK_B2BSTUP                  USB_OTG_DOEPMSK_B2BSTUP_Msk   /*!< Back-to-back SETUP packets received mask       */
#define USB_OTG_DOEPMSK_OPEM_Pos                 (8U)
#define USB_OTG_DOEPMSK_OPEM_Msk                 (0x1UL << USB_OTG_DOEPMSK_OPEM_Pos) /*!< 0x00000100 */
#define USB_OTG_DOEPMSK_OPEM                     USB_OTG_DOEPMSK_OPEM_Msk      /*!< OUT packet error mask                          */
#define USB_OTG_DOEPMSK_BOIM_Pos                 (9U)
#define USB_OTG_DOEPMSK_BOIM_Msk                 (0x1UL << USB_OTG_DOEPMSK_BOIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DOEPMSK_BOIM                     USB_OTG_DOEPMSK_BOIM_Msk      /*!< BNA interrupt mask                             */
#define USB_OTG_DOEPMSK_BERRM_Pos                (12U)
#define USB_OTG_DOEPMSK_BERRM_Msk                (0x1UL << USB_OTG_DOEPMSK_BERRM_Pos) /*!< 0x00001000 */
#define USB_OTG_DOEPMSK_BERRM                    USB_OTG_DOEPMSK_BERRM_Msk      /*!< Babble error interrupt mask                   */
#define USB_OTG_DOEPMSK_NAKM_Pos                 (13U)
#define USB_OTG_DOEPMSK_NAKM_Msk                 (0x1UL << USB_OTG_DOEPMSK_NAKM_Pos) /*!< 0x00002000 */
#define USB_OTG_DOEPMSK_NAKM                     USB_OTG_DOEPMSK_NAKM_Msk      /*!< OUT Packet NAK interrupt mask                  */
#define USB_OTG_DOEPMSK_NYETM_Pos                (14U)
#define USB_OTG_DOEPMSK_NYETM_Msk                (0x1UL << USB_OTG_DOEPMSK_NYETM_Pos) /*!< 0x00004000 */
#define USB_OTG_DOEPMSK_NYETM                    USB_OTG_DOEPMSK_NYETM_Msk     /*!< NYET interrupt mask                            */
/********************  Bit definition for USB_OTG_GINTSTS register  ********************/
#define USB_OTG_GINTSTS_CMOD_Pos                 (0U)
#define USB_OTG_GINTSTS_CMOD_Msk                 (0x1UL << USB_OTG_GINTSTS_CMOD_Pos) /*!< 0x00000001 */
#define USB_OTG_GINTSTS_CMOD                     USB_OTG_GINTSTS_CMOD_Msk      /*!< Current mode of operation                      */
#define USB_OTG_GINTSTS_MMIS_Pos                 (1U)
#define USB_OTG_GINTSTS_MMIS_Msk                 (0x1UL << USB_OTG_GINTSTS_MMIS_Pos) /*!< 0x00000002 */
#define USB_OTG_GINTSTS_MMIS                     USB_OTG_GINTSTS_MMIS_Msk      /*!< Mode mismatch interrupt                        */
#define USB_OTG_GINTSTS_OTGINT_Pos               (2U)
#define USB_OTG_GINTSTS_OTGINT_Msk               (0x1UL << USB_OTG_GINTSTS_OTGINT_Pos) /*!< 0x00000004 */
#define USB_OTG_GINTSTS_OTGINT                   USB_OTG_GINTSTS_OTGINT_Msk    /*!< OTG interrupt                                  */
#define USB_OTG_GINTSTS_SOF_Pos                  (3U)
#define USB_OTG_GINTSTS_SOF_Msk                  (0x1UL << USB_OTG_GINTSTS_SOF_Pos) /*!< 0x00000008 */
#define USB_OTG_GINTSTS_SOF                      USB_OTG_GINTSTS_SOF_Msk       /*!< Start of frame                                 */
#define USB_OTG_GINTSTS_RXFLVL_Pos               (4U)
#define USB_OTG_GINTSTS_RXFLVL_Msk               (0x1UL << USB_OTG_GINTSTS_RXFLVL_Pos) /*!< 0x00000010 */
#define USB_OTG_GINTSTS_RXFLVL                   USB_OTG_GINTSTS_RXFLVL_Msk    /*!< RxFIFO nonempty                                */
#define USB_OTG_GINTSTS_NPTXFE_Pos               (5U)
#define USB_OTG_GINTSTS_NPTXFE_Msk               (0x1UL << USB_OTG_GINTSTS_NPTXFE_Pos) /*!< 0x00000020 */
#define USB_OTG_GINTSTS_NPTXFE                   USB_OTG_GINTSTS_NPTXFE_Msk    /*!< Nonperiodic TxFIFO empty                       */
#define USB_OTG_GINTSTS_GINAKEFF_Pos             (6U)
#define USB_OTG_GINTSTS_GINAKEFF_Msk             (0x1UL << USB_OTG_GINTSTS_GINAKEFF_Pos) /*!< 0x00000040 */
#define USB_OTG_GINTSTS_GINAKEFF                 USB_OTG_GINTSTS_GINAKEFF_Msk  /*!< Global IN nonperiodic NAK effective            */
#define USB_OTG_GINTSTS_BOUTNAKEFF_Pos           (7U)
#define USB_OTG_GINTSTS_BOUTNAKEFF_Msk           (0x1UL << USB_OTG_GINTSTS_BOUTNAKEFF_Pos) /*!< 0x00000080 */
#define USB_OTG_GINTSTS_BOUTNAKEFF               USB_OTG_GINTSTS_BOUTNAKEFF_Msk /*!< Global OUT NAK effective                       */
#define USB_OTG_GINTSTS_ESUSP_Pos                (10U)
#define USB_OTG_GINTSTS_ESUSP_Msk                (0x1UL << USB_OTG_GINTSTS_ESUSP_Pos) /*!< 0x00000400 */
#define USB_OTG_GINTSTS_ESUSP                    USB_OTG_GINTSTS_ESUSP_Msk     /*!< Early suspend                                  */
#define USB_OTG_GINTSTS_USBSUSP_Pos              (11U)
#define USB_OTG_GINTSTS_USBSUSP_Msk              (0x1UL << USB_OTG_GINTSTS_USBSUSP_Pos) /*!< 0x00000800 */
#define USB_OTG_GINTSTS_USBSUSP                  USB_OTG_GINTSTS_USBSUSP_Msk   /*!< USB suspend                                    */
#define USB_OTG_GINTSTS_USBRST_Pos               (12U)
#define USB_OTG_GINTSTS_USBRST_Msk               (0x1UL << USB_OTG_GINTSTS_USBRST_Pos) /*!< 0x00001000 */
#define USB_OTG_GINTSTS_USBRST                   USB_OTG_GINTSTS_USBRST_Msk    /*!< USB reset                                      */
#define USB_OTG_GINTSTS_ENUMDNE_Pos              (13U)
#define USB_OTG_GINTSTS_ENUMDNE_Msk              (0x1UL << USB_OTG_GINTSTS_ENUMDNE_Pos) /*!< 0x00002000 */
#define USB_OTG_GINTSTS_ENUMDNE                  USB_OTG_GINTSTS_ENUMDNE_Msk   /*!< Enumeration done                               */
#define USB_OTG_GINTSTS_ISOODRP_Pos              (14U)
#define USB_OTG_GINTSTS_ISOODRP_Msk              (0x1UL << USB_OTG_GINTSTS_ISOODRP_Pos) /*!< 0x00004000 */
#define USB_OTG_GINTSTS_ISOODRP                  USB_OTG_GINTSTS_ISOODRP_Msk   /*!< Isochronous OUT packet dropped interrupt       */
#define USB_OTG_GINTSTS_EOPF_Pos                 (15U)
#define USB_OTG_GINTSTS_EOPF_Msk                 (0x1UL << USB_OTG_GINTSTS_EOPF_Pos) /*!< 0x00008000 */
#define USB_OTG_GINTSTS_EOPF                     USB_OTG_GINTSTS_EOPF_Msk      /*!< End of periodic frame interrupt                */
#define USB_OTG_GINTSTS_IEPINT_Pos               (18U)
#define USB_OTG_GINTSTS_IEPINT_Msk               (0x1UL << USB_OTG_GINTSTS_IEPINT_Pos) /*!< 0x00040000 */
#define USB_OTG_GINTSTS_IEPINT                   USB_OTG_GINTSTS_IEPINT_Msk    /*!< IN endpoint interrupt                          */
#define USB_OTG_GINTSTS_OEPINT_Pos               (19U)
#define USB_OTG_GINTSTS_OEPINT_Msk               (0x1UL << USB_OTG_GINTSTS_OEPINT_Pos) /*!< 0x00080000 */
#define USB_OTG_GINTSTS_OEPINT                   USB_OTG_GINTSTS_OEPINT_Msk    /*!< OUT endpoint interrupt                         */
#define USB_OTG_GINTSTS_IISOIXFR_Pos             (20U)
#define USB_OTG_GINTSTS_IISOIXFR_Msk             (0x1UL << USB_OTG_GINTSTS_IISOIXFR_Pos) /*!< 0x00100000 */
#define USB_OTG_GINTSTS_IISOIXFR                 USB_OTG_GINTSTS_IISOIXFR_Msk  /*!< Incomplete isochronous IN transfer             */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos    (21U)
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk    (0x1UL << USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos) /*!< 0x00200000 */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT        USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk /*!< Incomplete periodic transfer                   */
#define USB_OTG_GINTSTS_DATAFSUSP_Pos            (22U)
#define USB_OTG_GINTSTS_DATAFSUSP_Msk            (0x1UL << USB_OTG_GINTSTS_DATAFSUSP_Pos) /*!< 0x00400000 */
#define USB_OTG_GINTSTS_DATAFSUSP                USB_OTG_GINTSTS_DATAFSUSP_Msk /*!< Data fetch suspended                           */
#define USB_OTG_GINTSTS_HPRTINT_Pos              (24U)
#define USB_OTG_GINTSTS_HPRTINT_Msk              (0x1UL << USB_OTG_GINTSTS_HPRTINT_Pos) /*!< 0x01000000 */
#define USB_OTG_GINTSTS_HPRTINT                  USB_OTG_GINTSTS_HPRTINT_Msk   /*!< Host port interrupt                            */
#define USB_OTG_GINTSTS_HCINT_Pos                (25U)
#define USB_OTG_GINTSTS_HCINT_Msk                (0x1UL << USB_OTG_GINTSTS_HCINT_Pos) /*!< 0x02000000 */
#define USB_OTG_GINTSTS_HCINT                    USB_OTG_GINTSTS_HCINT_Msk     /*!< Host channels interrupt                        */
#define USB_OTG_GINTSTS_PTXFE_Pos                (26U)
#define USB_OTG_GINTSTS_PTXFE_Msk                (0x1UL << USB_OTG_GINTSTS_PTXFE_Pos) /*!< 0x04000000 */
#define USB_OTG_GINTSTS_PTXFE                    USB_OTG_GINTSTS_PTXFE_Msk     /*!< Periodic TxFIFO empty                          */
#define USB_OTG_GINTSTS_CIDSCHG_Pos              (28U)
#define USB_OTG_GINTSTS_CIDSCHG_Msk              (0x1UL << USB_OTG_GINTSTS_CIDSCHG_Pos) /*!< 0x10000000 */
#define USB_OTG_GINTSTS_CIDSCHG                  USB_OTG_GINTSTS_CIDSCHG_Msk   /*!< Connector ID status change                     */
#define USB_OTG_GINTSTS_DISCINT_Pos              (29U)
#define USB_OTG_GINTSTS_DISCINT_Msk              (0x1UL << USB_OTG_GINTSTS_DISCINT_Pos) /*!< 0x20000000 */
#define USB_OTG_GINTSTS_DISCINT                  USB_OTG_GINTSTS_DISCINT_Msk   /*!< Disconnect detected interrupt                  */
#define USB_OTG_GINTSTS_SRQINT_Pos               (30U)
#define USB_OTG_GINTSTS_SRQINT_Msk               (0x1UL << USB_OTG_GINTSTS_SRQINT_Pos) /*!< 0x40000000 */
#define USB_OTG_GINTSTS_SRQINT                   USB_OTG_GINTSTS_SRQINT_Msk    /*!< Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_WKUINT_Pos               (31U)
#define USB_OTG_GINTSTS_WKUINT_Msk               (0x1UL << USB_OTG_GINTSTS_WKUINT_Pos) /*!< 0x80000000 */
#define USB_OTG_GINTSTS_WKUINT                   USB_OTG_GINTSTS_WKUINT_Msk    /*!< Resume/remote wakeup detected interrupt        */

/********************  Bit definition for USB_OTG_GINTMSK register  ********************/
#define USB_OTG_GINTMSK_MMISM_Pos                (1U)
#define USB_OTG_GINTMSK_MMISM_Msk                (0x1UL << USB_OTG_GINTMSK_MMISM_Pos) /*!< 0x00000002 */
#define USB_OTG_GINTMSK_MMISM                    USB_OTG_GINTMSK_MMISM_Msk     /*!< Mode mismatch interrupt mask                        */
#define USB_OTG_GINTMSK_OTGINT_Pos               (2U)
#define USB_OTG_GINTMSK_OTGINT_Msk               (0x1UL << USB_OTG_GINTMSK_OTGINT_Pos) /*!< 0x00000004 */
#define USB_OTG_GINTMSK_OTGINT                   USB_OTG_GINTMSK_OTGINT_Msk    /*!< OTG interrupt mask                                  */
#define USB_OTG_GINTMSK_SOFM_Pos                 (3U)
#define USB_OTG_GINTMSK_SOFM_Msk                 (0x1UL << USB_OTG_GINTMSK_SOFM_Pos) /*!< 0x00000008 */
#define USB_OTG_GINTMSK_SOFM                     USB_OTG_GINTMSK_SOFM_Msk      /*!< Start of frame mask                                 */
#define USB_OTG_GINTMSK_RXFLVLM_Pos              (4U)
#define USB_OTG_GINTMSK_RXFLVLM_Msk              (0x1UL << USB_OTG_GINTMSK_RXFLVLM_Pos) /*!< 0x00000010 */
#define USB_OTG_GINTMSK_RXFLVLM                  USB_OTG_GINTMSK_RXFLVLM_Msk   /*!< Receive FIFO nonempty mask                          */
#define USB_OTG_GINTMSK_NPTXFEM_Pos              (5U)
#define USB_OTG_GINTMSK_NPTXFEM_Msk              (0x1UL << USB_OTG_GINTMSK_NPTXFEM_Pos) /*!< 0x00000020 */
#define USB_OTG_GINTMSK_NPTXFEM                  USB_OTG_GINTMSK_NPTXFEM_Msk   /*!< Nonperiodic TxFIFO empty mask                       */
#define USB_OTG_GINTMSK_GINAKEFFM_Pos            (6U)
#define USB_OTG_GINTMSK_GINAKEFFM_Msk            (0x1UL << USB_OTG_GINTMSK_GINAKEFFM_Pos) /*!< 0x00000040 */
#define USB_OTG_GINTMSK_GINAKEFFM                USB_OTG_GINTMSK_GINAKEFFM_Msk /*!< Global nonperiodic IN NAK effective mask            */
#define USB_OTG_GINTMSK_GONAKEFFM_Pos            (7U)
#define USB_OTG_GINTMSK_GONAKEFFM_Msk            (0x1UL << USB_OTG_GINTMSK_GONAKEFFM_Pos) /*!< 0x00000080 */
#define USB_OTG_GINTMSK_GONAKEFFM                USB_OTG_GINTMSK_GONAKEFFM_Msk /*!< Global OUT NAK effective mask                       */
#define USB_OTG_GINTMSK_ESUSPM_Pos               (10U)
#define USB_OTG_GINTMSK_ESUSPM_Msk               (0x1UL << USB_OTG_GINTMSK_ESUSPM_Pos) /*!< 0x00000400 */
#define USB_OTG_GINTMSK_ESUSPM                   USB_OTG_GINTMSK_ESUSPM_Msk    /*!< Early suspend mask                                  */
#define USB_OTG_GINTMSK_USBSUSPM_Pos             (11U)
#define USB_OTG_GINTMSK_USBSUSPM_Msk             (0x1UL << USB_OTG_GINTMSK_USBSUSPM_Pos) /*!< 0x00000800 */
#define USB_OTG_GINTMSK_USBSUSPM                 USB_OTG_GINTMSK_USBSUSPM_Msk  /*!< USB suspend mask                                    */
#define USB_OTG_GINTMSK_USBRST_Pos               (12U)
#define USB_OTG_GINTMSK_USBRST_Msk               (0x1UL << USB_OTG_GINTMSK_USBRST_Pos) /*!< 0x00001000 */
#define USB_OTG_GINTMSK_USBRST                   USB_OTG_GINTMSK_USBRST_Msk    /*!< USB reset mask                                      */
#define USB_OTG_GINTMSK_ENUMDNEM_Pos             (13U)
#define USB_OTG_GINTMSK_ENUMDNEM_Msk             (0x1UL << USB_OTG_GINTMSK_ENUMDNEM_Pos) /*!< 0x00002000 */
#define USB_OTG_GINTMSK_ENUMDNEM                 USB_OTG_GINTMSK_ENUMDNEM_Msk  /*!< Enumeration done mask                               */
#define USB_OTG_GINTMSK_ISOODRPM_Pos             (14U)
#define USB_OTG_GINTMSK_ISOODRPM_Msk             (0x1UL << USB_OTG_GINTMSK_ISOODRPM_Pos) /*!< 0x00004000 */
#define USB_OTG_GINTMSK_ISOODRPM                 USB_OTG_GINTMSK_ISOODRPM_Msk  /*!< Isochronous OUT packet dropped interrupt mask       */
#define USB_OTG_GINTMSK_EOPFM_Pos                (15U)
#define USB_OTG_GINTMSK_EOPFM_Msk                (0x1UL << USB_OTG_GINTMSK_EOPFM_Pos) /*!< 0x00008000 */
#define USB_OTG_GINTMSK_EOPFM                    USB_OTG_GINTMSK_EOPFM_Msk     /*!< End of periodic frame interrupt mask                */
#define USB_OTG_GINTMSK_EPMISM_Pos               (17U)
#define USB_OTG_GINTMSK_EPMISM_Msk               (0x1UL << USB_OTG_GINTMSK_EPMISM_Pos) /*!< 0x00020000 */
#define USB_OTG_GINTMSK_EPMISM                   USB_OTG_GINTMSK_EPMISM_Msk    /*!< Endpoint mismatch interrupt mask                    */
#define USB_OTG_GINTMSK_IEPINT_Pos               (18U)
#define USB_OTG_GINTMSK_IEPINT_Msk               (0x1UL << USB_OTG_GINTMSK_IEPINT_Pos) /*!< 0x00040000 */
#define USB_OTG_GINTMSK_IEPINT                   USB_OTG_GINTMSK_IEPINT_Msk    /*!< IN endpoints interrupt mask                         */
#define USB_OTG_GINTMSK_OEPINT_Pos               (19U)
#define USB_OTG_GINTMSK_OEPINT_Msk               (0x1UL << USB_OTG_GINTMSK_OEPINT_Pos) /*!< 0x00080000 */
#define USB_OTG_GINTMSK_OEPINT                   USB_OTG_GINTMSK_OEPINT_Msk    /*!< OUT endpoints interrupt mask                        */
#define USB_OTG_GINTMSK_IISOIXFRM_Pos            (20U)
#define USB_OTG_GINTMSK_IISOIXFRM_Msk            (0x1UL << USB_OTG_GINTMSK_IISOIXFRM_Pos) /*!< 0x00100000 */
#define USB_OTG_GINTMSK_IISOIXFRM                USB_OTG_GINTMSK_IISOIXFRM_Msk /*!< Incomplete isochronous IN transfer mask             */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Pos      (21U)
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Msk      (0x1UL << USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Pos) /*!< 0x00200000 */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM          USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Msk /*!< Incomplete periodic transfer mask                   */
#define USB_OTG_GINTMSK_FSUSPM_Pos               (22U)
#define USB_OTG_GINTMSK_FSUSPM_Msk               (0x1UL << USB_OTG_GINTMSK_FSUSPM_Pos) /*!< 0x00400000 */
#define USB_OTG_GINTMSK_FSUSPM                   USB_OTG_GINTMSK_FSUSPM_Msk    /*!< Data fetch suspended mask                           */
#define USB_OTG_GINTMSK_PRTIM_Pos                (24U)
#define USB_OTG_GINTMSK_PRTIM_Msk                (0x1UL << USB_OTG_GINTMSK_PRTIM_Pos) /*!< 0x01000000 */
#define USB_OTG_GINTMSK_PRTIM                    USB_OTG_GINTMSK_PRTIM_Msk     /*!< Host port interrupt mask                            */
#define USB_OTG_GINTMSK_HCIM_Pos                 (25U)
#define USB_OTG_GINTMSK_HCIM_Msk                 (0x1UL << USB_OTG_GINTMSK_HCIM_Pos) /*!< 0x02000000 */
#define USB_OTG_GINTMSK_HCIM                     USB_OTG_GINTMSK_HCIM_Msk      /*!< Host channels interrupt mask                        */
#define USB_OTG_GINTMSK_PTXFEM_Pos               (26U)
#define USB_OTG_GINTMSK_PTXFEM_Msk               (0x1UL << USB_OTG_GINTMSK_PTXFEM_Pos) /*!< 0x04000000 */
#define USB_OTG_GINTMSK_PTXFEM                   USB_OTG_GINTMSK_PTXFEM_Msk    /*!< Periodic TxFIFO empty mask                          */
#define USB_OTG_GINTMSK_CIDSCHGM_Pos             (28U)
#define USB_OTG_GINTMSK_CIDSCHGM_Msk             (0x1UL << USB_OTG_GINTMSK_CIDSCHGM_Pos) /*!< 0x10000000 */
#define USB_OTG_GINTMSK_CIDSCHGM                 USB_OTG_GINTMSK_CIDSCHGM_Msk  /*!< Connector ID status change mask                     */
#define USB_OTG_GINTMSK_DISCINT_Pos              (29U)
#define USB_OTG_GINTMSK_DISCINT_Msk              (0x1UL << USB_OTG_GINTMSK_DISCINT_Pos) /*!< 0x20000000 */
#define USB_OTG_GINTMSK_DISCINT                  USB_OTG_GINTMSK_DISCINT_Msk   /*!< Disconnect detected interrupt mask                  */
#define USB_OTG_GINTMSK_SRQIM_Pos                (30U)
#define USB_OTG_GINTMSK_SRQIM_Msk                (0x1UL << USB_OTG_GINTMSK_SRQIM_Pos) /*!< 0x40000000 */
#define USB_OTG_GINTMSK_SRQIM                    USB_OTG_GINTMSK_SRQIM_Msk     /*!< Session request/new session detected interrupt mask */
#define USB_OTG_GINTMSK_WUIM_Pos                 (31U)
#define USB_OTG_GINTMSK_WUIM_Msk                 (0x1UL << USB_OTG_GINTMSK_WUIM_Pos) /*!< 0x80000000 */
#define USB_OTG_GINTMSK_WUIM                     USB_OTG_GINTMSK_WUIM_Msk      /*!< Resume/remote wakeup detected interrupt mask        */

/********************  Bit definition for USB_OTG_DAINT register  ********************/
#define USB_OTG_DAINT_IEPINT_Pos                 (0U)
#define USB_OTG_DAINT_IEPINT_Msk                 (0xFFFFUL << USB_OTG_DAINT_IEPINT_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DAINT_IEPINT                     USB_OTG_DAINT_IEPINT_Msk      /*!< IN endpoint interrupt bits  */
#define USB_OTG_DAINT_OEPINT_Pos                 (16U)
#define USB_OTG_DAINT_OEPINT_Msk                 (0xFFFFUL << USB_OTG_DAINT_OEPINT_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_DAINT_OEPINT                     USB_OTG_DAINT_OEPINT_Msk      /*!< OUT endpoint interrupt bits */

/********************  Bit definition for USB_OTG_HAINTMSK register  ********************/
#define USB_OTG_HAINTMSK_HAINTM_Pos              (0U)
#define USB_OTG_HAINTMSK_HAINTM_Msk              (0xFFFFUL << USB_OTG_HAINTMSK_HAINTM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HAINTMSK_HAINTM                  USB_OTG_HAINTMSK_HAINTM_Msk   /*!< Channel interrupt mask */

/********************  Bit definition for USB_OTG_GRXSTSP register  ********************/
#define USB_OTG_GRXSTSP_EPNUM_Pos                (0U)
#define USB_OTG_GRXSTSP_EPNUM_Msk                (0xFUL << USB_OTG_GRXSTSP_EPNUM_Pos) /*!< 0x0000000F */
#define USB_OTG_GRXSTSP_EPNUM                    USB_OTG_GRXSTSP_EPNUM_Msk     /*!< IN EP interrupt mask bits  */
#define USB_OTG_GRXSTSP_BCNT_Pos                 (4U)
#define USB_OTG_GRXSTSP_BCNT_Msk                 (0x7FFUL << USB_OTG_GRXSTSP_BCNT_Pos) /*!< 0x00007FF0 */
#define USB_OTG_GRXSTSP_BCNT                     USB_OTG_GRXSTSP_BCNT_Msk      /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID_Pos                 (15U)
#define USB_OTG_GRXSTSP_DPID_Msk                 (0x3UL << USB_OTG_GRXSTSP_DPID_Pos) /*!< 0x00018000 */
#define USB_OTG_GRXSTSP_DPID                     USB_OTG_GRXSTSP_DPID_Msk      /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS_Pos               (17U)
#define USB_OTG_GRXSTSP_PKTSTS_Msk               (0xFUL << USB_OTG_GRXSTSP_PKTSTS_Pos) /*!< 0x001E0000 */
#define USB_OTG_GRXSTSP_PKTSTS                   USB_OTG_GRXSTSP_PKTSTS_Msk    /*!< OUT EP interrupt mask bits */

/********************  Bit definition for USB_OTG_DAINTMSK register  ********************/
#define USB_OTG_DAINTMSK_IEPM_Pos                (0U)
#define USB_OTG_DAINTMSK_IEPM_Msk                (0xFFFFUL << USB_OTG_DAINTMSK_IEPM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DAINTMSK_IEPM                    USB_OTG_DAINTMSK_IEPM_Msk     /*!< IN EP interrupt mask bits */
#define USB_OTG_DAINTMSK_OEPM_Pos                (16U)
#define USB_OTG_DAINTMSK_OEPM_Msk                (0xFFFFUL << USB_OTG_DAINTMSK_OEPM_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_DAINTMSK_OEPM                    USB_OTG_DAINTMSK_OEPM_Msk     /*!< OUT EP interrupt mask bits */

/********************  Bit definition for USB_OTG_GRXFSIZ register  ********************/
#define USB_OTG_GRXFSIZ_RXFD_Pos                 (0U)
#define USB_OTG_GRXFSIZ_RXFD_Msk                 (0xFFFFUL << USB_OTG_GRXFSIZ_RXFD_Pos) /*!< 0x0000FFFF */
#define USB_OTG_GRXFSIZ_RXFD                     USB_OTG_GRXFSIZ_RXFD_Msk      /*!< RxFIFO depth */

/********************  Bit definition for USB_OTG_DVBUSDIS register  ********************/
#define USB_OTG_DVBUSDIS_VBUSDT_Pos              (0U)
#define USB_OTG_DVBUSDIS_VBUSDT_Msk              (0xFFFFUL << USB_OTG_DVBUSDIS_VBUSDT_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DVBUSDIS_VBUSDT                  USB_OTG_DVBUSDIS_VBUSDT_Msk   /*!< Device VBUS discharge time */

/********************  Bit definition for OTG register  ********************/
#define USB_OTG_NPTXFSA_Pos                      (0U)
#define USB_OTG_NPTXFSA_Msk                      (0xFFFFUL << USB_OTG_NPTXFSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_NPTXFSA                          USB_OTG_NPTXFSA_Msk           /*!< Nonperiodic transmit RAM start address */
#define USB_OTG_NPTXFD_Pos                       (16U)
#define USB_OTG_NPTXFD_Msk                       (0xFFFFUL << USB_OTG_NPTXFD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_NPTXFD                           USB_OTG_NPTXFD_Msk            /*!< Nonperiodic TxFIFO depth               */
#define USB_OTG_TX0FSA_Pos                       (0U)
#define USB_OTG_TX0FSA_Msk                       (0xFFFFUL << USB_OTG_TX0FSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_TX0FSA                           USB_OTG_TX0FSA_Msk            /*!< Endpoint 0 transmit RAM start address  */
#define USB_OTG_TX0FD_Pos                        (16U)
#define USB_OTG_TX0FD_Msk                        (0xFFFFUL << USB_OTG_TX0FD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_TX0FD                            USB_OTG_TX0FD_Msk             /*!< Endpoint 0 TxFIFO depth                */

/********************  Bit definition forUSB_OTG_DVBUSPULSE register  ********************/
#define USB_OTG_DVBUSPULSE_DVBUSP_Pos            (0U)
#define USB_OTG_DVBUSPULSE_DVBUSP_Msk            (0xFFFUL << USB_OTG_DVBUSPULSE_DVBUSP_Pos) /*!< 0x00000FFF */
#define USB_OTG_DVBUSPULSE_DVBUSP                USB_OTG_DVBUSPULSE_DVBUSP_Msk /*!< Device VBUS pulsing time */

/********************  Bit definition for USB_OTG_GNPTXSTS register  ********************/
#define USB_OTG_GNPTXSTS_NPTXFSAV_Pos            (0U)
#define USB_OTG_GNPTXSTS_NPTXFSAV_Msk            (0xFFFFUL << USB_OTG_GNPTXSTS_NPTXFSAV_Pos) /*!< 0x0000FFFF */
#define USB_OTG_GNPTXSTS_NPTXFSAV                USB_OTG_GNPTXSTS_NPTXFSAV_Msk /*!< Nonperiodic TxFIFO space available */

#define USB_OTG_GNPTXSTS_NPTQXSAV_Pos            (16U)
#define USB_OTG_GNPTXSTS_NPTQXSAV_Msk            (0xFFUL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00FF0000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV                USB_OTG_GNPTXSTS_NPTQXSAV_Msk /*!< Nonperiodic transmit request queue space available */
#define USB_OTG_GNPTXSTS_NPTQXSAV_0              (0x01UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00010000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_1              (0x02UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00020000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_2              (0x04UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00040000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_3              (0x08UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00080000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_4              (0x10UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00100000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_5              (0x20UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00200000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_6              (0x40UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00400000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_7              (0x80UL << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00800000 */

#define USB_OTG_GNPTXSTS_NPTXQTOP_Pos            (24U)
#define USB_OTG_GNPTXSTS_NPTXQTOP_Msk            (0x7FUL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x7F000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP                USB_OTG_GNPTXSTS_NPTXQTOP_Msk /*!< Top of the nonperiodic transmit request queue */
#define USB_OTG_GNPTXSTS_NPTXQTOP_0              (0x01UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x01000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_1              (0x02UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x02000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_2              (0x04UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x04000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_3              (0x08UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x08000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_4              (0x10UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x10000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_5              (0x20UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x20000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_6              (0x40UL << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x40000000 */

/********************  Bit definition for USB_OTG_DTHRCTL register  ********************/
#define USB_OTG_DTHRCTL_NONISOTHREN_Pos          (0U)
#define USB_OTG_DTHRCTL_NONISOTHREN_Msk          (0x1UL << USB_OTG_DTHRCTL_NONISOTHREN_Pos) /*!< 0x00000001 */
#define USB_OTG_DTHRCTL_NONISOTHREN              USB_OTG_DTHRCTL_NONISOTHREN_Msk /*!< Nonisochronous IN endpoints threshold enable */
#define USB_OTG_DTHRCTL_ISOTHREN_Pos             (1U)
#define USB_OTG_DTHRCTL_ISOTHREN_Msk             (0x1UL << USB_OTG_DTHRCTL_ISOTHREN_Pos) /*!< 0x00000002 */
#define USB_OTG_DTHRCTL_ISOTHREN                 USB_OTG_DTHRCTL_ISOTHREN_Msk  /*!< ISO IN endpoint threshold enable */

#define USB_OTG_DTHRCTL_TXTHRLEN_Pos             (2U)
#define USB_OTG_DTHRCTL_TXTHRLEN_Msk             (0x1FFUL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x000007FC */
#define USB_OTG_DTHRCTL_TXTHRLEN                 USB_OTG_DTHRCTL_TXTHRLEN_Msk  /*!< Transmit threshold length */
#define USB_OTG_DTHRCTL_TXTHRLEN_0               (0x001UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000004 */
#define USB_OTG_DTHRCTL_TXTHRLEN_1               (0x002UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000008 */
#define USB_OTG_DTHRCTL_TXTHRLEN_2               (0x004UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000010 */
#define USB_OTG_DTHRCTL_TXTHRLEN_3               (0x008UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000020 */
#define USB_OTG_DTHRCTL_TXTHRLEN_4               (0x010UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000040 */
#define USB_OTG_DTHRCTL_TXTHRLEN_5               (0x020UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000080 */
#define USB_OTG_DTHRCTL_TXTHRLEN_6               (0x040UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000100 */
#define USB_OTG_DTHRCTL_TXTHRLEN_7               (0x080UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000200 */
#define USB_OTG_DTHRCTL_TXTHRLEN_8               (0x100UL << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000400 */
#define USB_OTG_DTHRCTL_RXTHREN_Pos              (16U)
#define USB_OTG_DTHRCTL_RXTHREN_Msk              (0x1UL << USB_OTG_DTHRCTL_RXTHREN_Pos) /*!< 0x00010000 */
#define USB_OTG_DTHRCTL_RXTHREN                  USB_OTG_DTHRCTL_RXTHREN_Msk   /*!< Receive threshold enable */

#define USB_OTG_DTHRCTL_RXTHRLEN_Pos             (17U)
#define USB_OTG_DTHRCTL_RXTHRLEN_Msk             (0x1FFUL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x03FE0000 */
#define USB_OTG_DTHRCTL_RXTHRLEN                 USB_OTG_DTHRCTL_RXTHRLEN_Msk  /*!< Receive threshold length */
#define USB_OTG_DTHRCTL_RXTHRLEN_0               (0x001UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00020000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_1               (0x002UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00040000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_2               (0x004UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00080000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_3               (0x008UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00100000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_4               (0x010UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00200000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_5               (0x020UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00400000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_6               (0x040UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00800000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_7               (0x080UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x01000000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_8               (0x100UL << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x02000000 */
#define USB_OTG_DTHRCTL_ARPEN_Pos                (27U)
#define USB_OTG_DTHRCTL_ARPEN_Msk                (0x1UL << USB_OTG_DTHRCTL_ARPEN_Pos) /*!< 0x08000000 */
#define USB_OTG_DTHRCTL_ARPEN                    USB_OTG_DTHRCTL_ARPEN_Msk     /*!< Arbiter parking enable */

/********************  Bit definition for USB_OTG_DIEPEMPMSK register  ********************/
#define USB_OTG_DIEPEMPMSK_INEPTXFEM_Pos         (0U)
#define USB_OTG_DIEPEMPMSK_INEPTXFEM_Msk         (0xFFFFUL << USB_OTG_DIEPEMPMSK_INEPTXFEM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DIEPEMPMSK_INEPTXFEM             USB_OTG_DIEPEMPMSK_INEPTXFEM_Msk /*!< IN EP Tx FIFO empty interrupt mask bits */

/********************  Bit definition for USB_OTG_DEACHINT register  ********************/
#define USB_OTG_DEACHINT_IEP1INT_Pos             (1U)
#define USB_OTG_DEACHINT_IEP1INT_Msk             (0x1UL << USB_OTG_DEACHINT_IEP1INT_Pos) /*!< 0x00000002 */
#define USB_OTG_DEACHINT_IEP1INT                 USB_OTG_DEACHINT_IEP1INT_Msk  /*!< IN endpoint 1interrupt bit   */
#define USB_OTG_DEACHINT_OEP1INT_Pos             (17U)
#define USB_OTG_DEACHINT_OEP1INT_Msk             (0x1UL << USB_OTG_DEACHINT_OEP1INT_Pos) /*!< 0x00020000 */
#define USB_OTG_DEACHINT_OEP1INT                 USB_OTG_DEACHINT_OEP1INT_Msk  /*!< OUT endpoint 1 interrupt bit */

/********************  Bit definition for USB_OTG_GCCFG register  ********************/
#define USB_OTG_GCCFG_PWRDWN_Pos                 (16U)
#define USB_OTG_GCCFG_PWRDWN_Msk                 (0x1UL << USB_OTG_GCCFG_PWRDWN_Pos) /*!< 0x00010000 */
#define USB_OTG_GCCFG_PWRDWN                     USB_OTG_GCCFG_PWRDWN_Msk      /*!< Power down */
#define USB_OTG_GCCFG_I2CPADEN_Pos               (17U)
#define USB_OTG_GCCFG_I2CPADEN_Msk               (0x1UL << USB_OTG_GCCFG_I2CPADEN_Pos) /*!< 0x00020000 */
#define USB_OTG_GCCFG_I2CPADEN                   USB_OTG_GCCFG_I2CPADEN_Msk    /*!< Enable I2C bus connection for the external I2C PHY interface*/
#define USB_OTG_GCCFG_VBUSASEN_Pos               (18U)
#define USB_OTG_GCCFG_VBUSASEN_Msk               (0x1UL << USB_OTG_GCCFG_VBUSASEN_Pos) /*!< 0x00040000 */
#define USB_OTG_GCCFG_VBUSASEN                   USB_OTG_GCCFG_VBUSASEN_Msk    /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_VBUSBSEN_Pos               (19U)
#define USB_OTG_GCCFG_VBUSBSEN_Msk               (0x1UL << USB_OTG_GCCFG_VBUSBSEN_Pos) /*!< 0x00080000 */
#define USB_OTG_GCCFG_VBUSBSEN                   USB_OTG_GCCFG_VBUSBSEN_Msk    /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_SOFOUTEN_Pos               (20U)
#define USB_OTG_GCCFG_SOFOUTEN_Msk               (0x1UL << USB_OTG_GCCFG_SOFOUTEN_Pos) /*!< 0x00100000 */
#define USB_OTG_GCCFG_SOFOUTEN                   USB_OTG_GCCFG_SOFOUTEN_Msk    /*!< SOF output enable */
#define USB_OTG_GCCFG_NOVBUSSENS_Pos             (21U)
#define USB_OTG_GCCFG_NOVBUSSENS_Msk             (0x1UL << USB_OTG_GCCFG_NOVBUSSENS_Pos) /*!< 0x00200000 */
#define USB_OTG_GCCFG_NOVBUSSENS                 USB_OTG_GCCFG_NOVBUSSENS_Msk  /*!< VBUS sensing disable option*/

/********************  Bit definition forUSB_OTG_DEACHINTMSK register  ********************/
#define USB_OTG_DEACHINTMSK_IEP1INTM_Pos         (1U)
#define USB_OTG_DEACHINTMSK_IEP1INTM_Msk         (0x1UL << USB_OTG_DEACHINTMSK_IEP1INTM_Pos) /*!< 0x00000002 */
#define USB_OTG_DEACHINTMSK_IEP1INTM             USB_OTG_DEACHINTMSK_IEP1INTM_Msk /*!< IN Endpoint 1 interrupt mask bit  */
#define USB_OTG_DEACHINTMSK_OEP1INTM_Pos         (17U)
#define USB_OTG_DEACHINTMSK_OEP1INTM_Msk         (0x1UL << USB_OTG_DEACHINTMSK_OEP1INTM_Pos) /*!< 0x00020000 */
#define USB_OTG_DEACHINTMSK_OEP1INTM             USB_OTG_DEACHINTMSK_OEP1INTM_Msk /*!< OUT Endpoint 1 interrupt mask bit */

/********************  Bit definition for USB_OTG_CID register  ********************/
#define USB_OTG_CID_PRODUCT_ID_Pos               (0U)
#define USB_OTG_CID_PRODUCT_ID_Msk               (0xFFFFFFFFUL << USB_OTG_CID_PRODUCT_ID_Pos) /*!< 0xFFFFFFFF */
#define USB_OTG_CID_PRODUCT_ID                   USB_OTG_CID_PRODUCT_ID_Msk    /*!< Product ID field */

/********************  Bit definition for USB_OTG_DIEPEACHMSK1 register  ********************/
#define USB_OTG_DIEPEACHMSK1_XFRCM_Pos           (0U)
#define USB_OTG_DIEPEACHMSK1_XFRCM_Msk           (0x1UL << USB_OTG_DIEPEACHMSK1_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DIEPEACHMSK1_XFRCM               USB_OTG_DIEPEACHMSK1_XFRCM_Msk /*!< Transfer completed interrupt mask                 */
#define USB_OTG_DIEPEACHMSK1_EPDM_Pos            (1U)
#define USB_OTG_DIEPEACHMSK1_EPDM_Msk            (0x1UL << USB_OTG_DIEPEACHMSK1_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DIEPEACHMSK1_EPDM                USB_OTG_DIEPEACHMSK1_EPDM_Msk /*!< Endpoint disabled interrupt mask                  */
#define USB_OTG_DIEPEACHMSK1_TOM_Pos             (3U)
#define USB_OTG_DIEPEACHMSK1_TOM_Msk             (0x1UL << USB_OTG_DIEPEACHMSK1_TOM_Pos) /*!< 0x00000008 */
#define USB_OTG_DIEPEACHMSK1_TOM                 USB_OTG_DIEPEACHMSK1_TOM_Msk  /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Pos       (4U)
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Msk       (0x1UL << USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Pos) /*!< 0x00000010 */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK           USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Msk /*!< IN token received when TxFIFO empty mask          */
#define USB_OTG_DIEPEACHMSK1_INEPNMM_Pos         (5U)
#define USB_OTG_DIEPEACHMSK1_INEPNMM_Msk         (0x1UL << USB_OTG_DIEPEACHMSK1_INEPNMM_Pos) /*!< 0x00000020 */
#define USB_OTG_DIEPEACHMSK1_INEPNMM             USB_OTG_DIEPEACHMSK1_INEPNMM_Msk /*!< IN token received with EP mismatch mask           */
#define USB_OTG_DIEPEACHMSK1_INEPNEM_Pos         (6U)
#define USB_OTG_DIEPEACHMSK1_INEPNEM_Msk         (0x1UL << USB_OTG_DIEPEACHMSK1_INEPNEM_Pos) /*!< 0x00000040 */
#define USB_OTG_DIEPEACHMSK1_INEPNEM             USB_OTG_DIEPEACHMSK1_INEPNEM_Msk /*!< IN endpoint NAK effective mask                    */
#define USB_OTG_DIEPEACHMSK1_TXFURM_Pos          (8U)
#define USB_OTG_DIEPEACHMSK1_TXFURM_Msk          (0x1UL << USB_OTG_DIEPEACHMSK1_TXFURM_Pos) /*!< 0x00000100 */
#define USB_OTG_DIEPEACHMSK1_TXFURM              USB_OTG_DIEPEACHMSK1_TXFURM_Msk /*!< FIFO underrun mask                                */
#define USB_OTG_DIEPEACHMSK1_BIM_Pos             (9U)
#define USB_OTG_DIEPEACHMSK1_BIM_Msk             (0x1UL << USB_OTG_DIEPEACHMSK1_BIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DIEPEACHMSK1_BIM                 USB_OTG_DIEPEACHMSK1_BIM_Msk  /*!< BNA interrupt mask                                */
#define USB_OTG_DIEPEACHMSK1_NAKM_Pos            (13U)
#define USB_OTG_DIEPEACHMSK1_NAKM_Msk            (0x1UL << USB_OTG_DIEPEACHMSK1_NAKM_Pos) /*!< 0x00002000 */
#define USB_OTG_DIEPEACHMSK1_NAKM                USB_OTG_DIEPEACHMSK1_NAKM_Msk /*!< NAK interrupt mask                                */

/********************  Bit definition for USB_OTG_HPRT register  ********************/
#define USB_OTG_HPRT_PCSTS_Pos                   (0U)
#define USB_OTG_HPRT_PCSTS_Msk                   (0x1UL << USB_OTG_HPRT_PCSTS_Pos) /*!< 0x00000001 */
#define USB_OTG_HPRT_PCSTS                       USB_OTG_HPRT_PCSTS_Msk        /*!< Port connect status        */
#define USB_OTG_HPRT_PCDET_Pos                   (1U)
#define USB_OTG_HPRT_PCDET_Msk                   (0x1UL << USB_OTG_HPRT_PCDET_Pos) /*!< 0x00000002 */
#define USB_OTG_HPRT_PCDET                       USB_OTG_HPRT_PCDET_Msk        /*!< Port connect detected      */
#define USB_OTG_HPRT_PENA_Pos                    (2U)
#define USB_OTG_HPRT_PENA_Msk                    (0x1UL << USB_OTG_HPRT_PENA_Pos) /*!< 0x00000004 */
#define USB_OTG_HPRT_PENA                        USB_OTG_HPRT_PENA_Msk         /*!< Port enable                */
#define USB_OTG_HPRT_PENCHNG_Pos                 (3U)
#define USB_OTG_HPRT_PENCHNG_Msk                 (0x1UL << USB_OTG_HPRT_PENCHNG_Pos) /*!< 0x00000008 */
#define USB_OTG_HPRT_PENCHNG                     USB_OTG_HPRT_PENCHNG_Msk      /*!< Port enable/disable change */
#define USB_OTG_HPRT_POCA_Pos                    (4U)
#define USB_OTG_HPRT_POCA_Msk                    (0x1UL << USB_OTG_HPRT_POCA_Pos) /*!< 0x00000010 */
#define USB_OTG_HPRT_POCA                        USB_OTG_HPRT_POCA_Msk         /*!< Port overcurrent active    */
#define USB_OTG_HPRT_POCCHNG_Pos                 (5U)
#define USB_OTG_HPRT_POCCHNG_Msk                 (0x1UL << USB_OTG_HPRT_POCCHNG_Pos) /*!< 0x00000020 */
#define USB_OTG_HPRT_POCCHNG                     USB_OTG_HPRT_POCCHNG_Msk      /*!< Port overcurrent change    */
#define USB_OTG_HPRT_PRES_Pos                    (6U)
#define USB_OTG_HPRT_PRES_Msk                    (0x1UL << USB_OTG_HPRT_PRES_Pos) /*!< 0x00000040 */
#define USB_OTG_HPRT_PRES                        USB_OTG_HPRT_PRES_Msk         /*!< Port resume                */
#define USB_OTG_HPRT_PSUSP_Pos                   (7U)
#define USB_OTG_HPRT_PSUSP_Msk                   (0x1UL << USB_OTG_HPRT_PSUSP_Pos) /*!< 0x00000080 */
#define USB_OTG_HPRT_PSUSP                       USB_OTG_HPRT_PSUSP_Msk        /*!< Port suspend               */
#define USB_OTG_HPRT_PRST_Pos                    (8U)
#define USB_OTG_HPRT_PRST_Msk                    (0x1UL << USB_OTG_HPRT_PRST_Pos) /*!< 0x00000100 */
#define USB_OTG_HPRT_PRST                        USB_OTG_HPRT_PRST_Msk         /*!< Port reset                 */

#define USB_OTG_HPRT_PLSTS_Pos                   (10U)
#define USB_OTG_HPRT_PLSTS_Msk                   (0x3UL << USB_OTG_HPRT_PLSTS_Pos) /*!< 0x00000C00 */
#define USB_OTG_HPRT_PLSTS                       USB_OTG_HPRT_PLSTS_Msk        /*!< Port line status           */
#define USB_OTG_HPRT_PLSTS_0                     (0x1UL << USB_OTG_HPRT_PLSTS_Pos) /*!< 0x00000400 */
#define USB_OTG_HPRT_PLSTS_1                     (0x2UL << USB_OTG_HPRT_PLSTS_Pos) /*!< 0x00000800 */
#define USB_OTG_HPRT_PPWR_Pos                    (12U)
#define USB_OTG_HPRT_PPWR_Msk                    (0x1UL << USB_OTG_HPRT_PPWR_Pos) /*!< 0x00001000 */
#define USB_OTG_HPRT_PPWR                        USB_OTG_HPRT_PPWR_Msk         /*!< Port power                 */

#define USB_OTG_HPRT_PTCTL_Pos                   (13U)
#define USB_OTG_HPRT_PTCTL_Msk                   (0xFUL << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x0001E000 */
#define USB_OTG_HPRT_PTCTL                       USB_OTG_HPRT_PTCTL_Msk        /*!< Port test control          */
#define USB_OTG_HPRT_PTCTL_0                     (0x1UL << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00002000 */
#define USB_OTG_HPRT_PTCTL_1                     (0x2UL << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00004000 */
#define USB_OTG_HPRT_PTCTL_2                     (0x4UL << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00008000 */
#define USB_OTG_HPRT_PTCTL_3                     (0x8UL << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00010000 */

#define USB_OTG_HPRT_PSPD_Pos                    (17U)
#define USB_OTG_HPRT_PSPD_Msk                    (0x3UL << USB_OTG_HPRT_PSPD_Pos) /*!< 0x00060000 */
#define USB_OTG_HPRT_PSPD                        USB_OTG_HPRT_PSPD_Msk         /*!< Port speed                 */
#define USB_OTG_HPRT_PSPD_0                      (0x1UL << USB_OTG_HPRT_PSPD_Pos) /*!< 0x00020000 */
#define USB_OTG_HPRT_PSPD_1                      (0x2UL << USB_OTG_HPRT_PSPD_Pos) /*!< 0x00040000 */

/********************  Bit definition for USB_OTG_DOEPEACHMSK1 register  ********************/
#define USB_OTG_DOEPEACHMSK1_XFRCM_Pos           (0U)
#define USB_OTG_DOEPEACHMSK1_XFRCM_Msk           (0x1UL << USB_OTG_DOEPEACHMSK1_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DOEPEACHMSK1_XFRCM               USB_OTG_DOEPEACHMSK1_XFRCM_Msk /*!< Transfer completed interrupt mask         */
#define USB_OTG_DOEPEACHMSK1_EPDM_Pos            (1U)
#define USB_OTG_DOEPEACHMSK1_EPDM_Msk            (0x1UL << USB_OTG_DOEPEACHMSK1_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DOEPEACHMSK1_EPDM                USB_OTG_DOEPEACHMSK1_EPDM_Msk /*!< Endpoint disabled interrupt mask          */
#define USB_OTG_DOEPEACHMSK1_TOM_Pos             (3U)
#define USB_OTG_DOEPEACHMSK1_TOM_Msk             (0x1UL << USB_OTG_DOEPEACHMSK1_TOM_Pos) /*!< 0x00000008 */
#define USB_OTG_DOEPEACHMSK1_TOM                 USB_OTG_DOEPEACHMSK1_TOM_Msk  /*!< Timeout condition mask                    */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Pos       (4U)
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Msk       (0x1UL << USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Pos) /*!< 0x00000010 */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK           USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Msk /*!< IN token received when TxFIFO empty mask  */
#define USB_OTG_DOEPEACHMSK1_INEPNMM_Pos         (5U)
#define USB_OTG_DOEPEACHMSK1_INEPNMM_Msk         (0x1UL << USB_OTG_DOEPEACHMSK1_INEPNMM_Pos) /*!< 0x00000020 */
#define USB_OTG_DOEPEACHMSK1_INEPNMM             USB_OTG_DOEPEACHMSK1_INEPNMM_Msk /*!< IN token received with EP mismatch mask   */
#define USB_OTG_DOEPEACHMSK1_INEPNEM_Pos         (6U)
#define USB_OTG_DOEPEACHMSK1_INEPNEM_Msk         (0x1UL << USB_OTG_DOEPEACHMSK1_INEPNEM_Pos) /*!< 0x00000040 */
#define USB_OTG_DOEPEACHMSK1_INEPNEM             USB_OTG_DOEPEACHMSK1_INEPNEM_Msk /*!< IN endpoint NAK effective mask            */
#define USB_OTG_DOEPEACHMSK1_TXFURM_Pos          (8U)
#define USB_OTG_DOEPEACHMSK1_TXFURM_Msk          (0x1UL << USB_OTG_DOEPEACHMSK1_TXFURM_Pos) /*!< 0x00000100 */
#define USB_OTG_DOEPEACHMSK1_TXFURM              USB_OTG_DOEPEACHMSK1_TXFURM_Msk /*!< OUT packet error mask                     */
#define USB_OTG_DOEPEACHMSK1_BIM_Pos             (9U)
#define USB_OTG_DOEPEACHMSK1_BIM_Msk             (0x1UL << USB_OTG_DOEPEACHMSK1_BIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DOEPEACHMSK1_BIM                 USB_OTG_DOEPEACHMSK1_BIM_Msk  /*!< BNA interrupt mask                        */
#define USB_OTG_DOEPEACHMSK1_BERRM_Pos           (12U)
#define USB_OTG_DOEPEACHMSK1_BERRM_Msk           (0x1UL << USB_OTG_DOEPEACHMSK1_BERRM_Pos) /*!< 0x00001000 */
#define USB_OTG_DOEPEACHMSK1_BERRM               USB_OTG_DOEPEACHMSK1_BERRM_Msk /*!< Bubble error interrupt mask               */
#define USB_OTG_DOEPEACHMSK1_NAKM_Pos            (13U)
#define USB_OTG_DOEPEACHMSK1_NAKM_Msk            (0x1UL << USB_OTG_DOEPEACHMSK1_NAKM_Pos) /*!< 0x00002000 */
#define USB_OTG_DOEPEACHMSK1_NAKM                USB_OTG_DOEPEACHMSK1_NAKM_Msk /*!< NAK interrupt mask                        */
#define USB_OTG_DOEPEACHMSK1_NYETM_Pos           (14U)
#define USB_OTG_DOEPEACHMSK1_NYETM_Msk           (0x1UL << USB_OTG_DOEPEACHMSK1_NYETM_Pos) /*!< 0x00004000 */
#define USB_OTG_DOEPEACHMSK1_NYETM               USB_OTG_DOEPEACHMSK1_NYETM_Msk /*!< NYET interrupt mask                       */

/********************  Bit definition for USB_OTG_HPTXFSIZ register  ********************/
#define USB_OTG_HPTXFSIZ_PTXSA_Pos               (0U)
#define USB_OTG_HPTXFSIZ_PTXSA_Msk               (0xFFFFUL << USB_OTG_HPTXFSIZ_PTXSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HPTXFSIZ_PTXSA                   USB_OTG_HPTXFSIZ_PTXSA_Msk    /*!< Host periodic TxFIFO start address            */
#define USB_OTG_HPTXFSIZ_PTXFD_Pos               (16U)
#define USB_OTG_HPTXFSIZ_PTXFD_Msk               (0xFFFFUL << USB_OTG_HPTXFSIZ_PTXFD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_HPTXFSIZ_PTXFD                   USB_OTG_HPTXFSIZ_PTXFD_Msk    /*!< Host periodic TxFIFO depth                    */

/********************  Bit definition for USB_OTG_DIEPCTL register  ********************/
#define USB_OTG_DIEPCTL_MPSIZ_Pos                (0U)
#define USB_OTG_DIEPCTL_MPSIZ_Msk                (0x7FFUL << USB_OTG_DIEPCTL_MPSIZ_Pos) /*!< 0x000007FF */
#define USB_OTG_DIEPCTL_MPSIZ                    USB_OTG_DIEPCTL_MPSIZ_Msk     /*!< Maximum packet size              */
#define USB_OTG_DIEPCTL_USBAEP_Pos               (15U)
#define USB_OTG_DIEPCTL_USBAEP_Msk               (0x1UL << USB_OTG_DIEPCTL_USBAEP_Pos) /*!< 0x00008000 */
#define USB_OTG_DIEPCTL_USBAEP                   USB_OTG_DIEPCTL_USBAEP_Msk    /*!< USB active endpoint              */
#define USB_OTG_DIEPCTL_EONUM_DPID_Pos           (16U)
#define USB_OTG_DIEPCTL_EONUM_DPID_Msk           (0x1UL << USB_OTG_DIEPCTL_EONUM_DPID_Pos) /*!< 0x00010000 */
#define USB_OTG_DIEPCTL_EONUM_DPID               USB_OTG_DIEPCTL_EONUM_DPID_Msk /*!< Even/odd frame                   */
#define USB_OTG_DIEPCTL_NAKSTS_Pos               (17U)
#define USB_OTG_DIEPCTL_NAKSTS_Msk               (0x1UL << USB_OTG_DIEPCTL_NAKSTS_Pos) /*!< 0x00020000 */
#define USB_OTG_DIEPCTL_NAKSTS                   USB_OTG_DIEPCTL_NAKSTS_Msk    /*!< NAK status                       */

#define USB_OTG_DIEPCTL_EPTYP_Pos                (18U)
#define USB_OTG_DIEPCTL_EPTYP_Msk                (0x3UL << USB_OTG_DIEPCTL_EPTYP_Pos) /*!< 0x000C0000 */
#define USB_OTG_DIEPCTL_EPTYP                    USB_OTG_DIEPCTL_EPTYP_Msk     /*!< Endpoint type                    */
#define USB_OTG_DIEPCTL_EPTYP_0                  (0x1UL << USB_OTG_DIEPCTL_EPTYP_Pos) /*!< 0x00040000 */
#define USB_OTG_DIEPCTL_EPTYP_1                  (0x2UL << USB_OTG_DIEPCTL_EPTYP_Pos) /*!< 0x00080000 */
#define USB_OTG_DIEPCTL_STALL_Pos                (21U)
#define USB_OTG_DIEPCTL_STALL_Msk                (0x1UL << USB_OTG_DIEPCTL_STALL_Pos) /*!< 0x00200000 */
#define USB_OTG_DIEPCTL_STALL                    USB_OTG_DIEPCTL_STALL_Msk     /*!< STALL handshake                  */

#define USB_OTG_DIEPCTL_TXFNUM_Pos               (22U)
#define USB_OTG_DIEPCTL_TXFNUM_Msk               (0xFUL << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x03C00000 */
#define USB_OTG_DIEPCTL_TXFNUM                   USB_OTG_DIEPCTL_TXFNUM_Msk    /*!< TxFIFO number                    */
#define USB_OTG_DIEPCTL_TXFNUM_0                 (0x1UL << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x00400000 */
#define USB_OTG_DIEPCTL_TXFNUM_1                 (0x2UL << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x00800000 */
#define USB_OTG_DIEPCTL_TXFNUM_2                 (0x4UL << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x01000000 */
#define USB_OTG_DIEPCTL_TXFNUM_3                 (0x8UL << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x02000000 */
#define USB_OTG_DIEPCTL_CNAK_Pos                 (26U)
#define USB_OTG_DIEPCTL_CNAK_Msk                 (0x1UL << USB_OTG_DIEPCTL_CNAK_Pos) /*!< 0x04000000 */
#define USB_OTG_DIEPCTL_CNAK                     USB_OTG_DIEPCTL_CNAK_Msk      /*!< Clear NAK                        */
#define USB_OTG_DIEPCTL_SNAK_Pos                 (27U)
#define USB_OTG_DIEPCTL_SNAK_Msk                 (0x1UL << USB_OTG_DIEPCTL_SNAK_Pos) /*!< 0x08000000 */
#define USB_OTG_DIEPCTL_SNAK                     USB_OTG_DIEPCTL_SNAK_Msk      /*!< Set NAK */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Pos       (28U)
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk       (0x1UL << USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Pos) /*!< 0x10000000 */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM           USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk /*!< Set DATA0 PID                    */
#define USB_OTG_DIEPCTL_SODDFRM_Pos              (29U)
#define USB_OTG_DIEPCTL_SODDFRM_Msk              (0x1UL << USB_OTG_DIEPCTL_SODDFRM_Pos) /*!< 0x20000000 */
#define USB_OTG_DIEPCTL_SODDFRM                  USB_OTG_DIEPCTL_SODDFRM_Msk   /*!< Set odd frame                    */
#define USB_OTG_DIEPCTL_EPDIS_Pos                (30U)
#define USB_OTG_DIEPCTL_EPDIS_Msk                (0x1UL << USB_OTG_DIEPCTL_EPDIS_Pos) /*!< 0x40000000 */
#define USB_OTG_DIEPCTL_EPDIS                    USB_OTG_DIEPCTL_EPDIS_Msk     /*!< Endpoint disable                 */
#define USB_OTG_DIEPCTL_EPENA_Pos                (31U)
#define USB_OTG_DIEPCTL_EPENA_Msk                (0x1UL << USB_OTG_DIEPCTL_EPENA_Pos) /*!< 0x80000000 */
#define USB_OTG_DIEPCTL_EPENA                    USB_OTG_DIEPCTL_EPENA_Msk     /*!< Endpoint enable                  */

/********************  Bit definition for USB_OTG_HCCHAR register  ********************/
#define USB_OTG_HCCHAR_MPSIZ_Pos                 (0U)
#define USB_OTG_HCCHAR_MPSIZ_Msk                 (0x7FFUL << USB_OTG_HCCHAR_MPSIZ_Pos) /*!< 0x000007FF */
#define USB_OTG_HCCHAR_MPSIZ                     USB_OTG_HCCHAR_MPSIZ_Msk      /*!< Maximum packet size */

#define USB_OTG_HCCHAR_EPNUM_Pos                 (11U)
#define USB_OTG_HCCHAR_EPNUM_Msk                 (0xFUL << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00007800 */
#define USB_OTG_HCCHAR_EPNUM                     USB_OTG_HCCHAR_EPNUM_Msk      /*!< Endpoint number */
#define USB_OTG_HCCHAR_EPNUM_0                   (0x1UL << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00000800 */
#define USB_OTG_HCCHAR_EPNUM_1                   (0x2UL << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00001000 */
#define USB_OTG_HCCHAR_EPNUM_2                   (0x4UL << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00002000 */
#define USB_OTG_HCCHAR_EPNUM_3                   (0x8UL << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00004000 */
#define USB_OTG_HCCHAR_EPDIR_Pos                 (15U)
#define USB_OTG_HCCHAR_EPDIR_Msk                 (0x1UL << USB_OTG_HCCHAR_EPDIR_Pos) /*!< 0x00008000 */
#define USB_OTG_HCCHAR_EPDIR                     USB_OTG_HCCHAR_EPDIR_Msk      /*!< Endpoint direction */
#define USB_OTG_HCCHAR_LSDEV_Pos                 (17U)
#define USB_OTG_HCCHAR_LSDEV_Msk                 (0x1UL << USB_OTG_HCCHAR_LSDEV_Pos) /*!< 0x00020000 */
#define USB_OTG_HCCHAR_LSDEV                     USB_OTG_HCCHAR_LSDEV_Msk      /*!< Low-speed device */

#define USB_OTG_HCCHAR_EPTYP_Pos                 (18U)
#define USB_OTG_HCCHAR_EPTYP_Msk                 (0x3UL << USB_OTG_HCCHAR_EPTYP_Pos) /*!< 0x000C0000 */
#define USB_OTG_HCCHAR_EPTYP                     USB_OTG_HCCHAR_EPTYP_Msk      /*!< Endpoint type */
#define USB_OTG_HCCHAR_EPTYP_0                   (0x1UL << USB_OTG_HCCHAR_EPTYP_Pos) /*!< 0x00040000 */
#define USB_OTG_HCCHAR_EPTYP_1                   (0x2UL << USB_OTG_HCCHAR_EPTYP_Pos) /*!< 0x00080000 */

#define USB_OTG_HCCHAR_MC_Pos                    (20U)
#define USB_OTG_HCCHAR_MC_Msk                    (0x3UL << USB_OTG_HCCHAR_MC_Pos) /*!< 0x00300000 */
#define USB_OTG_HCCHAR_MC                        USB_OTG_HCCHAR_MC_Msk         /*!< Multi Count (MC) / Error Count (EC) */
#define USB_OTG_HCCHAR_MC_0                      (0x1UL << USB_OTG_HCCHAR_MC_Pos) /*!< 0x00100000 */
#define USB_OTG_HCCHAR_MC_1                      (0x2UL << USB_OTG_HCCHAR_MC_Pos) /*!< 0x00200000 */

#define USB_OTG_HCCHAR_DAD_Pos                   (22U)
#define USB_OTG_HCCHAR_DAD_Msk                   (0x7FUL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x1FC00000 */
#define USB_OTG_HCCHAR_DAD                       USB_OTG_HCCHAR_DAD_Msk        /*!< Device address */
#define USB_OTG_HCCHAR_DAD_0                     (0x01UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x00400000 */
#define USB_OTG_HCCHAR_DAD_1                     (0x02UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x00800000 */
#define USB_OTG_HCCHAR_DAD_2                     (0x04UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x01000000 */
#define USB_OTG_HCCHAR_DAD_3                     (0x08UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x02000000 */
#define USB_OTG_HCCHAR_DAD_4                     (0x10UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x04000000 */
#define USB_OTG_HCCHAR_DAD_5                     (0x20UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x08000000 */
#define USB_OTG_HCCHAR_DAD_6                     (0x40UL << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x10000000 */
#define USB_OTG_HCCHAR_ODDFRM_Pos                (29U)
#define USB_OTG_HCCHAR_ODDFRM_Msk                (0x1UL << USB_OTG_HCCHAR_ODDFRM_Pos) /*!< 0x20000000 */
#define USB_OTG_HCCHAR_ODDFRM                    USB_OTG_HCCHAR_ODDFRM_Msk     /*!< Odd frame */
#define USB_OTG_HCCHAR_CHDIS_Pos                 (30U)
#define USB_OTG_HCCHAR_CHDIS_Msk                 (0x1UL << USB_OTG_HCCHAR_CHDIS_Pos) /*!< 0x40000000 */
#define USB_OTG_HCCHAR_CHDIS                     USB_OTG_HCCHAR_CHDIS_Msk      /*!< Channel disable */
#define USB_OTG_HCCHAR_CHENA_Pos                 (31U)
#define USB_OTG_HCCHAR_CHENA_Msk                 (0x1UL << USB_OTG_HCCHAR_CHENA_Pos) /*!< 0x80000000 */
#define USB_OTG_HCCHAR_CHENA                     USB_OTG_HCCHAR_CHENA_Msk      /*!< Channel enable */

/********************  Bit definition for USB_OTG_HCSPLT register  ********************/

#define USB_OTG_HCSPLT_PRTADDR_Pos               (0U)
#define USB_OTG_HCSPLT_PRTADDR_Msk               (0x7FUL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x0000007F */
#define USB_OTG_HCSPLT_PRTADDR                   USB_OTG_HCSPLT_PRTADDR_Msk    /*!< Port address */
#define USB_OTG_HCSPLT_PRTADDR_0                 (0x01UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000001 */
#define USB_OTG_HCSPLT_PRTADDR_1                 (0x02UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000002 */
#define USB_OTG_HCSPLT_PRTADDR_2                 (0x04UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000004 */
#define USB_OTG_HCSPLT_PRTADDR_3                 (0x08UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000008 */
#define USB_OTG_HCSPLT_PRTADDR_4                 (0x10UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000010 */
#define USB_OTG_HCSPLT_PRTADDR_5                 (0x20UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000020 */
#define USB_OTG_HCSPLT_PRTADDR_6                 (0x40UL << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000040 */

#define USB_OTG_HCSPLT_HUBADDR_Pos               (7U)
#define USB_OTG_HCSPLT_HUBADDR_Msk               (0x7FUL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00003F80 */
#define USB_OTG_HCSPLT_HUBADDR                   USB_OTG_HCSPLT_HUBADDR_Msk    /*!< Hub address */
#define USB_OTG_HCSPLT_HUBADDR_0                 (0x01UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000080 */
#define USB_OTG_HCSPLT_HUBADDR_1                 (0x02UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000100 */
#define USB_OTG_HCSPLT_HUBADDR_2                 (0x04UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000200 */
#define USB_OTG_HCSPLT_HUBADDR_3                 (0x08UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000400 */
#define USB_OTG_HCSPLT_HUBADDR_4                 (0x10UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000800 */
#define USB_OTG_HCSPLT_HUBADDR_5                 (0x20UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00001000 */
#define USB_OTG_HCSPLT_HUBADDR_6                 (0x40UL << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00002000 */

#define USB_OTG_HCSPLT_XACTPOS_Pos               (14U)
#define USB_OTG_HCSPLT_XACTPOS_Msk               (0x3UL << USB_OTG_HCSPLT_XACTPOS_Pos) /*!< 0x0000C000 */
#define USB_OTG_HCSPLT_XACTPOS                   USB_OTG_HCSPLT_XACTPOS_Msk    /*!< XACTPOS */
#define USB_OTG_HCSPLT_XACTPOS_0                 (0x1UL << USB_OTG_HCSPLT_XACTPOS_Pos) /*!< 0x00004000 */
#define USB_OTG_HCSPLT_XACTPOS_1                 (0x2UL << USB_OTG_HCSPLT_XACTPOS_Pos) /*!< 0x00008000 */
#define USB_OTG_HCSPLT_COMPLSPLT_Pos             (16U)
#define USB_OTG_HCSPLT_COMPLSPLT_Msk             (0x1UL << USB_OTG_HCSPLT_COMPLSPLT_Pos) /*!< 0x00010000 */
#define USB_OTG_HCSPLT_COMPLSPLT                 USB_OTG_HCSPLT_COMPLSPLT_Msk  /*!< Do complete split */
#define USB_OTG_HCSPLT_SPLITEN_Pos               (31U)
#define USB_OTG_HCSPLT_SPLITEN_Msk               (0x1UL << USB_OTG_HCSPLT_SPLITEN_Pos) /*!< 0x80000000 */
#define USB_OTG_HCSPLT_SPLITEN                   USB_OTG_HCSPLT_SPLITEN_Msk    /*!< Split enable */

/********************  Bit definition for USB_OTG_HCINT register  ********************/
#define USB_OTG_HCINT_XFRC_Pos                   (0U)
#define USB_OTG_HCINT_XFRC_Msk                   (0x1UL << USB_OTG_HCINT_XFRC_Pos) /*!< 0x00000001 */
#define USB_OTG_HCINT_XFRC                       USB_OTG_HCINT_XFRC_Msk        /*!< Transfer completed */
#define USB_OTG_HCINT_CHH_Pos                    (1U)
#define USB_OTG_HCINT_CHH_Msk                    (0x1UL << USB_OTG_HCINT_CHH_Pos) /*!< 0x00000002 */
#define USB_OTG_HCINT_CHH                        USB_OTG_HCINT_CHH_Msk         /*!< Channel halted */
#define USB_OTG_HCINT_AHBERR_Pos                 (2U)
#define USB_OTG_HCINT_AHBERR_Msk                 (0x1UL << USB_OTG_HCINT_AHBERR_Pos) /*!< 0x00000004 */
#define USB_OTG_HCINT_AHBERR                     USB_OTG_HCINT_AHBERR_Msk      /*!< AHB error */
#define USB_OTG_HCINT_STALL_Pos                  (3U)
#define USB_OTG_HCINT_STALL_Msk                  (0x1UL << USB_OTG_HCINT_STALL_Pos) /*!< 0x00000008 */
#define USB_OTG_HCINT_STALL                      USB_OTG_HCINT_STALL_Msk       /*!< STALL response received interrupt */
#define USB_OTG_HCINT_NAK_Pos                    (4U)
#define USB_OTG_HCINT_NAK_Msk                    (0x1UL << USB_OTG_HCINT_NAK_Pos) /*!< 0x00000010 */
#define USB_OTG_HCINT_NAK                        USB_OTG_HCINT_NAK_Msk         /*!< NAK response received interrupt */
#define USB_OTG_HCINT_ACK_Pos                    (5U)
#define USB_OTG_HCINT_ACK_Msk                    (0x1UL << USB_OTG_HCINT_ACK_Pos) /*!< 0x00000020 */
#define USB_OTG_HCINT_ACK                        USB_OTG_HCINT_ACK_Msk         /*!< ACK response received/transmitted interrupt */
#define USB_OTG_HCINT_NYET_Pos                   (6U)
#define USB_OTG_HCINT_NYET_Msk                   (0x1UL << USB_OTG_HCINT_NYET_Pos) /*!< 0x00000040 */
#define USB_OTG_HCINT_NYET                       USB_OTG_HCINT_NYET_Msk        /*!< Response received interrupt */
#define USB_OTG_HCINT_TXERR_Pos                  (7U)
#define USB_OTG_HCINT_TXERR_Msk                  (0x1UL << USB_OTG_HCINT_TXERR_Pos) /*!< 0x00000080 */
#define USB_OTG_HCINT_TXERR                      USB_OTG_HCINT_TXERR_Msk       /*!< Transaction error */
#define USB_OTG_HCINT_BBERR_Pos                  (8U)
#define USB_OTG_HCINT_BBERR_Msk                  (0x1UL << USB_OTG_HCINT_BBERR_Pos) /*!< 0x00000100 */
#define USB_OTG_HCINT_BBERR                      USB_OTG_HCINT_BBERR_Msk       /*!< Babble error */
#define USB_OTG_HCINT_FRMOR_Pos                  (9U)
#define USB_OTG_HCINT_FRMOR_Msk                  (0x1UL << USB_OTG_HCINT_FRMOR_Pos) /*!< 0x00000200 */
#define USB_OTG_HCINT_FRMOR                      USB_OTG_HCINT_FRMOR_Msk       /*!< Frame overrun */
#define USB_OTG_HCINT_DTERR_Pos                  (10U)
#define USB_OTG_HCINT_DTERR_Msk                  (0x1UL << USB_OTG_HCINT_DTERR_Pos) /*!< 0x00000400 */
#define USB_OTG_HCINT_DTERR                      USB_OTG_HCINT_DTERR_Msk       /*!< Data toggle error */

/********************  Bit definition for USB_OTG_DIEPINT register  ********************/
#define USB_OTG_DIEPINT_XFRC_Pos                 (0U)
#define USB_OTG_DIEPINT_XFRC_Msk                 (0x1UL << USB_OTG_DIEPINT_XFRC_Pos) /*!< 0x00000001 */
#define USB_OTG_DIEPINT_XFRC                     USB_OTG_DIEPINT_XFRC_Msk      /*!< Transfer completed interrupt */
#define USB_OTG_DIEPINT_EPDISD_Pos               (1U)
#define USB_OTG_DIEPINT_EPDISD_Msk               (0x1UL << USB_OTG_DIEPINT_EPDISD_Pos) /*!< 0x00000002 */
#define USB_OTG_DIEPINT_EPDISD                   USB_OTG_DIEPINT_EPDISD_Msk    /*!< Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_AHBERR_Pos               (2U)
#define USB_OTG_DIEPINT_AHBERR_Msk               (0x1UL << USB_OTG_DIEPINT_AHBERR_Pos) /*!< 0x00000004 */
#define USB_OTG_DIEPINT_AHBERR                   USB_OTG_DIEPINT_AHBERR_Msk   /*!< AHB Error (AHBErr) during an IN transaction */
#define USB_OTG_DIEPINT_TOC_Pos                  (3U)
#define USB_OTG_DIEPINT_TOC_Msk                  (0x1UL << USB_OTG_DIEPINT_TOC_Pos) /*!< 0x00000008 */
#define USB_OTG_DIEPINT_TOC                      USB_OTG_DIEPINT_TOC_Msk       /*!< Timeout condition */
#define USB_OTG_DIEPINT_ITTXFE_Pos               (4U)
#define USB_OTG_DIEPINT_ITTXFE_Msk               (0x1UL << USB_OTG_DIEPINT_ITTXFE_Pos) /*!< 0x00000010 */
#define USB_OTG_DIEPINT_ITTXFE                   USB_OTG_DIEPINT_ITTXFE_Msk    /*!< IN token received when TxFIFO is empty */
#define USB_OTG_DIEPINT_INEPNM_Pos               (5U)
#define USB_OTG_DIEPINT_INEPNM_Msk               (0x1UL << USB_OTG_DIEPINT_INEPNM_Pos) /*!< 0x00000004 */
#define USB_OTG_DIEPINT_INEPNM                   USB_OTG_DIEPINT_INEPNM_Msk   /*!< IN token received with EP mismatch */
#define USB_OTG_DIEPINT_INEPNE_Pos               (6U)
#define USB_OTG_DIEPINT_INEPNE_Msk               (0x1UL << USB_OTG_DIEPINT_INEPNE_Pos) /*!< 0x00000040 */
#define USB_OTG_DIEPINT_INEPNE                   USB_OTG_DIEPINT_INEPNE_Msk    /*!< IN endpoint NAK effective */
#define USB_OTG_DIEPINT_TXFE_Pos                 (7U)
#define USB_OTG_DIEPINT_TXFE_Msk                 (0x1UL << USB_OTG_DIEPINT_TXFE_Pos) /*!< 0x00000080 */
#define USB_OTG_DIEPINT_TXFE                     USB_OTG_DIEPINT_TXFE_Msk      /*!< Transmit FIFO empty */
#define USB_OTG_DIEPINT_TXFIFOUDRN_Pos           (8U)
#define USB_OTG_DIEPINT_TXFIFOUDRN_Msk           (0x1UL << USB_OTG_DIEPINT_TXFIFOUDRN_Pos) /*!< 0x00000100 */
#define USB_OTG_DIEPINT_TXFIFOUDRN               USB_OTG_DIEPINT_TXFIFOUDRN_Msk /*!< Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_BNA_Pos                  (9U)
#define USB_OTG_DIEPINT_BNA_Msk                  (0x1UL << USB_OTG_DIEPINT_BNA_Pos) /*!< 0x00000200 */
#define USB_OTG_DIEPINT_BNA                      USB_OTG_DIEPINT_BNA_Msk       /*!< Buffer not available interrupt */
#define USB_OTG_DIEPINT_PKTDRPSTS_Pos            (11U)
#define USB_OTG_DIEPINT_PKTDRPSTS_Msk            (0x1UL << USB_OTG_DIEPINT_PKTDRPSTS_Pos) /*!< 0x00000800 */
#define USB_OTG_DIEPINT_PKTDRPSTS                USB_OTG_DIEPINT_PKTDRPSTS_Msk /*!< Packet dropped status */
#define USB_OTG_DIEPINT_BERR_Pos                 (12U)
#define USB_OTG_DIEPINT_BERR_Msk                 (0x1UL << USB_OTG_DIEPINT_BERR_Pos) /*!< 0x00001000 */
#define USB_OTG_DIEPINT_BERR                     USB_OTG_DIEPINT_BERR_Msk      /*!< Babble error interrupt */
#define USB_OTG_DIEPINT_NAK_Pos                  (13U)
#define USB_OTG_DIEPINT_NAK_Msk                  (0x1UL << USB_OTG_DIEPINT_NAK_Pos) /*!< 0x00002000 */
#define USB_OTG_DIEPINT_NAK                      USB_OTG_DIEPINT_NAK_Msk       /*!< NAK interrupt */

/********************  Bit definition forUSB_OTG_HCINTMSK register  ********************/
#define USB_OTG_HCINTMSK_XFRCM_Pos               (0U)
#define USB_OTG_HCINTMSK_XFRCM_Msk               (0x1UL << USB_OTG_HCINTMSK_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_HCINTMSK_XFRCM                   USB_OTG_HCINTMSK_XFRCM_Msk    /*!< Transfer completed mask */
#define USB_OTG_HCINTMSK_CHHM_Pos                (1U)
#define USB_OTG_HCINTMSK_CHHM_Msk                (0x1UL << USB_OTG_HCINTMSK_CHHM_Pos) /*!< 0x00000002 */
#define USB_OTG_HCINTMSK_CHHM                    USB_OTG_HCINTMSK_CHHM_Msk     /*!< Channel halted mask */
#define USB_OTG_HCINTMSK_AHBERR_Pos              (2U)
#define USB_OTG_HCINTMSK_AHBERR_Msk              (0x1UL << USB_OTG_HCINTMSK_AHBERR_Pos) /*!< 0x00000004 */
#define USB_OTG_HCINTMSK_AHBERR                  USB_OTG_HCINTMSK_AHBERR_Msk   /*!< AHB error */
#define USB_OTG_HCINTMSK_STALLM_Pos              (3U)
#define USB_OTG_HCINTMSK_STALLM_Msk              (0x1UL << USB_OTG_HCINTMSK_STALLM_Pos) /*!< 0x00000008 */
#define USB_OTG_HCINTMSK_STALLM                  USB_OTG_HCINTMSK_STALLM_Msk   /*!< STALL response received interrupt mask */
#define USB_OTG_HCINTMSK_NAKM_Pos                (4U)
#define USB_OTG_HCINTMSK_NAKM_Msk                (0x1UL << USB_OTG_HCINTMSK_NAKM_Pos) /*!< 0x00000010 */
#define USB_OTG_HCINTMSK_NAKM                    USB_OTG_HCINTMSK_NAKM_Msk     /*!< NAK response received interrupt mask */
#define USB_OTG_HCINTMSK_ACKM_Pos                (5U)
#define USB_OTG_HCINTMSK_ACKM_Msk                (0x1UL << USB_OTG_HCINTMSK_ACKM_Pos) /*!< 0x00000020 */
#define USB_OTG_HCINTMSK_ACKM                    USB_OTG_HCINTMSK_ACKM_Msk     /*!< ACK response received/transmitted interrupt mask */
#define USB_OTG_HCINTMSK_NYET_Pos                (6U)
#define USB_OTG_HCINTMSK_NYET_Msk                (0x1UL << USB_OTG_HCINTMSK_NYET_Pos) /*!< 0x00000040 */
#define USB_OTG_HCINTMSK_NYET                    USB_OTG_HCINTMSK_NYET_Msk     /*!< response received interrupt mask */
#define USB_OTG_HCINTMSK_TXERRM_Pos              (7U)
#define USB_OTG_HCINTMSK_TXERRM_Msk              (0x1UL << USB_OTG_HCINTMSK_TXERRM_Pos) /*!< 0x00000080 */
#define USB_OTG_HCINTMSK_TXERRM                  USB_OTG_HCINTMSK_TXERRM_Msk   /*!< Transaction error mask */
#define USB_OTG_HCINTMSK_BBERRM_Pos              (8U)
#define USB_OTG_HCINTMSK_BBERRM_Msk              (0x1UL << USB_OTG_HCINTMSK_BBERRM_Pos) /*!< 0x00000100 */
#define USB_OTG_HCINTMSK_BBERRM                  USB_OTG_HCINTMSK_BBERRM_Msk   /*!< Babble error mask */
#define USB_OTG_HCINTMSK_FRMORM_Pos              (9U)
#define USB_OTG_HCINTMSK_FRMORM_Msk              (0x1UL << USB_OTG_HCINTMSK_FRMORM_Pos) /*!< 0x00000200 */
#define USB_OTG_HCINTMSK_FRMORM                  USB_OTG_HCINTMSK_FRMORM_Msk   /*!< Frame overrun mask */
#define USB_OTG_HCINTMSK_DTERRM_Pos              (10U)
#define USB_OTG_HCINTMSK_DTERRM_Msk              (0x1UL << USB_OTG_HCINTMSK_DTERRM_Pos) /*!< 0x00000400 */
#define USB_OTG_HCINTMSK_DTERRM                  USB_OTG_HCINTMSK_DTERRM_Msk   /*!< Data toggle error mask */

/********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************/

#define USB_OTG_DIEPTSIZ_XFRSIZ_Pos              (0U)
#define USB_OTG_DIEPTSIZ_XFRSIZ_Msk              (0x7FFFFUL << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) /*!< 0x0007FFFF */
#define USB_OTG_DIEPTSIZ_XFRSIZ                  USB_OTG_DIEPTSIZ_XFRSIZ_Msk   /*!< Transfer size */
#define USB_OTG_DIEPTSIZ_PKTCNT_Pos              (19U)
#define USB_OTG_DIEPTSIZ_PKTCNT_Msk              (0x3FFUL << USB_OTG_DIEPTSIZ_PKTCNT_Pos) /*!< 0x1FF80000 */
#define USB_OTG_DIEPTSIZ_PKTCNT                  USB_OTG_DIEPTSIZ_PKTCNT_Msk   /*!< Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT_Pos              (29U)
#define USB_OTG_DIEPTSIZ_MULCNT_Msk              (0x3UL << USB_OTG_DIEPTSIZ_MULCNT_Pos) /*!< 0x60000000 */
#define USB_OTG_DIEPTSIZ_MULCNT                  USB_OTG_DIEPTSIZ_MULCNT_Msk   /*!< Packet count */
/********************  Bit definition for USB_OTG_HCTSIZ register  ********************/
#define USB_OTG_HCTSIZ_XFRSIZ_Pos                (0U)
#define USB_OTG_HCTSIZ_XFRSIZ_Msk                (0x7FFFFUL << USB_OTG_HCTSIZ_XFRSIZ_Pos) /*!< 0x0007FFFF */
#define USB_OTG_HCTSIZ_XFRSIZ                    USB_OTG_HCTSIZ_XFRSIZ_Msk     /*!< Transfer size */
#define USB_OTG_HCTSIZ_PKTCNT_Pos                (19U)
#define USB_OTG_HCTSIZ_PKTCNT_Msk                (0x3FFUL << USB_OTG_HCTSIZ_PKTCNT_Pos) /*!< 0x1FF80000 */
#define USB_OTG_HCTSIZ_PKTCNT                    USB_OTG_HCTSIZ_PKTCNT_Msk     /*!< Packet count */
#define USB_OTG_HCTSIZ_DOPING_Pos                (31U)
#define USB_OTG_HCTSIZ_DOPING_Msk                (0x1UL << USB_OTG_HCTSIZ_DOPING_Pos) /*!< 0x80000000 */
#define USB_OTG_HCTSIZ_DOPING                    USB_OTG_HCTSIZ_DOPING_Msk     /*!< Do PING */
#define USB_OTG_HCTSIZ_DPID_Pos                  (29U)
#define USB_OTG_HCTSIZ_DPID_Msk                  (0x3UL << USB_OTG_HCTSIZ_DPID_Pos) /*!< 0x60000000 */
#define USB_OTG_HCTSIZ_DPID                      USB_OTG_HCTSIZ_DPID_Msk       /*!< Data PID */
#define USB_OTG_HCTSIZ_DPID_0                    (0x1UL << USB_OTG_HCTSIZ_DPID_Pos) /*!< 0x20000000 */
#define USB_OTG_HCTSIZ_DPID_1                    (0x2UL << USB_OTG_HCTSIZ_DPID_Pos) /*!< 0x40000000 */

/********************  Bit definition for USB_OTG_DIEPDMA register  ********************/
#define USB_OTG_DIEPDMA_DMAADDR_Pos              (0U)
#define USB_OTG_DIEPDMA_DMAADDR_Msk              (0xFFFFFFFFUL << USB_OTG_DIEPDMA_DMAADDR_Pos) /*!< 0xFFFFFFFF */
#define USB_OTG_DIEPDMA_DMAADDR                  USB_OTG_DIEPDMA_DMAADDR_Msk   /*!< DMA address */

/********************  Bit definition for USB_OTG_HCDMA register  ********************/
#define USB_OTG_HCDMA_DMAADDR_Pos                (0U)
#define USB_OTG_HCDMA_DMAADDR_Msk                (0xFFFFFFFFUL << USB_OTG_HCDMA_DMAADDR_Pos) /*!< 0xFFFFFFFF */
#define USB_OTG_HCDMA_DMAADDR                    USB_OTG_HCDMA_DMAADDR_Msk     /*!< DMA address */

/********************  Bit definition for USB_OTG_DTXFSTS register  ********************/
#define USB_OTG_DTXFSTS_INEPTFSAV_Pos            (0U)
#define USB_OTG_DTXFSTS_INEPTFSAV_Msk            (0xFFFFUL << USB_OTG_DTXFSTS_INEPTFSAV_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DTXFSTS_INEPTFSAV                USB_OTG_DTXFSTS_INEPTFSAV_Msk /*!< IN endpoint TxFIFO space available */

/********************  Bit definition for USB_OTG_DIEPTXF register  ********************/
#define USB_OTG_DIEPTXF_INEPTXSA_Pos             (0U)
#define USB_OTG_DIEPTXF_INEPTXSA_Msk             (0xFFFFUL << USB_OTG_DIEPTXF_INEPTXSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DIEPTXF_INEPTXSA                 USB_OTG_DIEPTXF_INEPTXSA_Msk  /*!< IN endpoint FIFOx transmit RAM start address */
#define USB_OTG_DIEPTXF_INEPTXFD_Pos             (16U)
#define USB_OTG_DIEPTXF_INEPTXFD_Msk             (0xFFFFUL << USB_OTG_DIEPTXF_INEPTXFD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_DIEPTXF_INEPTXFD                 USB_OTG_DIEPTXF_INEPTXFD_Msk  /*!< IN endpoint TxFIFO depth */

/********************  Bit definition for USB_OTG_DOEPCTL register  ********************/

#define USB_OTG_DOEPCTL_MPSIZ_Pos                (0U)
#define USB_OTG_DOEPCTL_MPSIZ_Msk                (0x7FFUL << USB_OTG_DOEPCTL_MPSIZ_Pos) /*!< 0x000007FF */
#define USB_OTG_DOEPCTL_MPSIZ                    USB_OTG_DOEPCTL_MPSIZ_Msk     /*!< Maximum packet size */          /*!<Bit 1 */
#define USB_OTG_DOEPCTL_USBAEP_Pos               (15U)
#define USB_OTG_DOEPCTL_USBAEP_Msk               (0x1UL << USB_OTG_DOEPCTL_USBAEP_Pos) /*!< 0x00008000 */
#define USB_OTG_DOEPCTL_USBAEP                   USB_OTG_DOEPCTL_USBAEP_Msk    /*!< USB active endpoint */
#define USB_OTG_DOEPCTL_NAKSTS_Pos               (17U)
#define USB_OTG_DOEPCTL_NAKSTS_Msk               (0x1UL << USB_OTG_DOEPCTL_NAKSTS_Pos) /*!< 0x00020000 */
#define USB_OTG_DOEPCTL_NAKSTS                   USB_OTG_DOEPCTL_NAKSTS_Msk    /*!< NAK status */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Pos       (28U)
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk       (0x1UL << USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Pos) /*!< 0x10000000 */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM           USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk /*!< Set DATA0 PID */
#define USB_OTG_DOEPCTL_SODDFRM_Pos              (29U)
#define USB_OTG_DOEPCTL_SODDFRM_Msk              (0x1UL << USB_OTG_DOEPCTL_SODDFRM_Pos) /*!< 0x20000000 */
#define USB_OTG_DOEPCTL_SODDFRM                  USB_OTG_DOEPCTL_SODDFRM_Msk   /*!< Set odd frame */
#define USB_OTG_DOEPCTL_EPTYP_Pos                (18U)
#define USB_OTG_DOEPCTL_EPTYP_Msk                (0x3UL << USB_OTG_DOEPCTL_EPTYP_Pos) /*!< 0x000C0000 */
#define USB_OTG_DOEPCTL_EPTYP                    USB_OTG_DOEPCTL_EPTYP_Msk     /*!< Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP_0                  (0x1UL << USB_OTG_DOEPCTL_EPTYP_Pos) /*!< 0x00040000 */
#define USB_OTG_DOEPCTL_EPTYP_1                  (0x2UL << USB_OTG_DOEPCTL_EPTYP_Pos) /*!< 0x00080000 */
#define USB_OTG_DOEPCTL_SNPM_Pos                 (20U)
#define USB_OTG_DOEPCTL_SNPM_Msk                 (0x1UL << USB_OTG_DOEPCTL_SNPM_Pos) /*!< 0x00100000 */
#define USB_OTG_DOEPCTL_SNPM                     USB_OTG_DOEPCTL_SNPM_Msk      /*!< Snoop mode */
#define USB_OTG_DOEPCTL_STALL_Pos                (21U)
#define USB_OTG_DOEPCTL_STALL_Msk                (0x1UL << USB_OTG_DOEPCTL_STALL_Pos) /*!< 0x00200000 */
#define USB_OTG_DOEPCTL_STALL                    USB_OTG_DOEPCTL_STALL_Msk     /*!< STALL handshake */
#define USB_OTG_DOEPCTL_CNAK_Pos                 (26U)
#define USB_OTG_DOEPCTL_CNAK_Msk                 (0x1UL << USB_OTG_DOEPCTL_CNAK_Pos) /*!< 0x04000000 */
#define USB_OTG_DOEPCTL_CNAK                     USB_OTG_DOEPCTL_CNAK_Msk      /*!< Clear NAK */
#define USB_OTG_DOEPCTL_SNAK_Pos                 (27U)
#define USB_OTG_DOEPCTL_SNAK_Msk                 (0x1UL << USB_OTG_DOEPCTL_SNAK_Pos) /*!< 0x08000000 */
#define USB_OTG_DOEPCTL_SNAK                     USB_OTG_DOEPCTL_SNAK_Msk      /*!< Set NAK */
#define USB_OTG_DOEPCTL_EPDIS_Pos                (30U)
#define USB_OTG_DOEPCTL_EPDIS_Msk                (0x1UL << USB_OTG_DOEPCTL_EPDIS_Pos) /*!< 0x40000000 */
#define USB_OTG_DOEPCTL_EPDIS                    USB_OTG_DOEPCTL_EPDIS_Msk     /*!< Endpoint disable */
#define USB_OTG_DOEPCTL_EPENA_Pos                (31U)
#define USB_OTG_DOEPCTL_EPENA_Msk                (0x1UL << USB_OTG_DOEPCTL_EPENA_Pos) /*!< 0x80000000 */
#define USB_OTG_DOEPCTL_EPENA                    USB_OTG_DOEPCTL_EPENA_Msk     /*!< Endpoint enable */

/********************  Bit definition for USB_OTG_DOEPINT register  ********************/
#define USB_OTG_DOEPINT_XFRC_Pos                 (0U)
#define USB_OTG_DOEPINT_XFRC_Msk                 (0x1UL << USB_OTG_DOEPINT_XFRC_Pos) /*!< 0x00000001 */
#define USB_OTG_DOEPINT_XFRC                     USB_OTG_DOEPINT_XFRC_Msk      /*!< Transfer completed interrupt */
#define USB_OTG_DOEPINT_EPDISD_Pos               (1U)
#define USB_OTG_DOEPINT_EPDISD_Msk               (0x1UL << USB_OTG_DOEPINT_EPDISD_Pos) /*!< 0x00000002 */
#define USB_OTG_DOEPINT_EPDISD                   USB_OTG_DOEPINT_EPDISD_Msk    /*!< Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_AHBERR_Pos               (2U)
#define USB_OTG_DOEPINT_AHBERR_Msk               (0x1UL << USB_OTG_DOEPINT_AHBERR_Pos) /*!< 0x00000004 */
#define USB_OTG_DOEPINT_AHBERR                   USB_OTG_DOEPINT_AHBERR_Msk   /*!< AHB Error (AHBErr) during an OUT transaction */
#define USB_OTG_DOEPINT_STUP_Pos                 (3U)
#define USB_OTG_DOEPINT_STUP_Msk                 (0x1UL << USB_OTG_DOEPINT_STUP_Pos) /*!< 0x00000008 */
#define USB_OTG_DOEPINT_STUP                     USB_OTG_DOEPINT_STUP_Msk      /*!< SETUP phase done */
#define USB_OTG_DOEPINT_OTEPDIS_Pos              (4U)
#define USB_OTG_DOEPINT_OTEPDIS_Msk              (0x1UL << USB_OTG_DOEPINT_OTEPDIS_Pos) /*!< 0x00000010 */
#define USB_OTG_DOEPINT_OTEPDIS                  USB_OTG_DOEPINT_OTEPDIS_Msk   /*!< OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_OTEPSPR_Pos              (5U)
#define USB_OTG_DOEPINT_OTEPSPR_Msk              (0x1UL << USB_OTG_DOEPINT_OTEPSPR_Pos) /*!< 0x00000020 */
#define USB_OTG_DOEPINT_OTEPSPR                  USB_OTG_DOEPINT_OTEPSPR_Msk   /*!< Status Phase Received For Control Write */
#define USB_OTG_DOEPINT_B2BSTUP_Pos              (6U)
#define USB_OTG_DOEPINT_B2BSTUP_Msk              (0x1UL << USB_OTG_DOEPINT_B2BSTUP_Pos) /*!< 0x00000040 */
#define USB_OTG_DOEPINT_B2BSTUP                  USB_OTG_DOEPINT_B2BSTUP_Msk   /*!< Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_OUTPKTERR_Pos            (8U)
#define USB_OTG_DOEPINT_OUTPKTERR_Msk            (0x1UL << USB_OTG_DOEPINT_OUTPKTERR_Pos) /*!< 0x00000100 */
#define USB_OTG_DOEPINT_OUTPKTERR                USB_OTG_DOEPINT_OUTPKTERR_Msk   /*!< OUT packet error */
#define USB_OTG_DOEPINT_NAK_Pos                  (13U)
#define USB_OTG_DOEPINT_NAK_Msk                  (0x1UL << USB_OTG_DOEPINT_NAK_Pos) /*!< 0x00002000 */
#define USB_OTG_DOEPINT_NAK                      USB_OTG_DOEPINT_NAK_Msk   /*!< NAK Packet is transmitted by the device */
#define USB_OTG_DOEPINT_NYET_Pos                 (14U)
#define USB_OTG_DOEPINT_NYET_Msk                 (0x1UL << USB_OTG_DOEPINT_NYET_Pos) /*!< 0x00004000 */
#define USB_OTG_DOEPINT_NYET                     USB_OTG_DOEPINT_NYET_Msk      /*!< NYET interrupt */
#define USB_OTG_DOEPINT_STPKTRX_Pos              (15U)
#define USB_OTG_DOEPINT_STPKTRX_Msk              (0x1UL << USB_OTG_DOEPINT_STPKTRX_Pos) /*!< 0x00008000 */
#define USB_OTG_DOEPINT_STPKTRX                  USB_OTG_DOEPINT_STPKTRX_Msk   /*!< Setup Packet Received */
/********************  Bit definition for USB_OTG_DOEPTSIZ register  ********************/

#define USB_OTG_DOEPTSIZ_XFRSIZ_Pos              (0U)
#define USB_OTG_DOEPTSIZ_XFRSIZ_Msk              (0x7FFFFUL << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) /*!< 0x0007FFFF */
#define USB_OTG_DOEPTSIZ_XFRSIZ                  USB_OTG_DOEPTSIZ_XFRSIZ_Msk   /*!< Transfer size */
#define USB_OTG_DOEPTSIZ_PKTCNT_Pos              (19U)
#define USB_OTG_DOEPTSIZ_PKTCNT_Msk              (0x3FFUL << USB_OTG_DOEPTSIZ_PKTCNT_Pos) /*!< 0x1FF80000 */
#define USB_OTG_DOEPTSIZ_PKTCNT                  USB_OTG_DOEPTSIZ_PKTCNT_Msk   /*!< Packet count */

#define USB_OTG_DOEPTSIZ_STUPCNT_Pos             (29U)
#define USB_OTG_DOEPTSIZ_STUPCNT_Msk             (0x3UL << USB_OTG_DOEPTSIZ_STUPCNT_Pos) /*!< 0x60000000 */
#define USB_OTG_DOEPTSIZ_STUPCNT                 USB_OTG_DOEPTSIZ_STUPCNT_Msk  /*!< SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT_0               (0x1UL << USB_OTG_DOEPTSIZ_STUPCNT_Pos) /*!< 0x20000000 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1               (0x2UL << USB_OTG_DOEPTSIZ_STUPCNT_Pos) /*!< 0x40000000 */

/********************  Bit definition for PCGCCTL register  ********************/
#define USB_OTG_PCGCCTL_STOPCLK_Pos              (0U)
#define USB_OTG_PCGCCTL_STOPCLK_Msk              (0x1UL << USB_OTG_PCGCCTL_STOPCLK_Pos) /*!< 0x00000001 */
#define USB_OTG_PCGCCTL_STOPCLK                  USB_OTG_PCGCCTL_STOPCLK_Msk   /*!< SETUP packet count */
#define USB_OTG_PCGCCTL_GATECLK_Pos              (1U)
#define USB_OTG_PCGCCTL_GATECLK_Msk              (0x1UL << USB_OTG_PCGCCTL_GATECLK_Pos) /*!< 0x00000002 */
#define USB_OTG_PCGCCTL_GATECLK                  USB_OTG_PCGCCTL_GATECLK_Msk   /*!<Bit 0 */
#define USB_OTG_PCGCCTL_PHYSUSP_Pos              (4U)
#define USB_OTG_PCGCCTL_PHYSUSP_Msk              (0x1UL << USB_OTG_PCGCCTL_PHYSUSP_Pos) /*!< 0x00000010 */
#define USB_OTG_PCGCCTL_PHYSUSP                  USB_OTG_PCGCCTL_PHYSUSP_Msk   /*!<Bit 1 */

/* Legacy define */
/********************  Bit definition for OTG register  ********************/
#define USB_OTG_CHNUM_Pos                        (0U)
#define USB_OTG_CHNUM_Msk                        (0xFUL << USB_OTG_CHNUM_Pos)   /*!< 0x0000000F */
#define USB_OTG_CHNUM                            USB_OTG_CHNUM_Msk             /*!< Channel number */
#define USB_OTG_CHNUM_0                          (0x1UL << USB_OTG_CHNUM_Pos)   /*!< 0x00000001 */
#define USB_OTG_CHNUM_1                          (0x2UL << USB_OTG_CHNUM_Pos)   /*!< 0x00000002 */
#define USB_OTG_CHNUM_2                          (0x4UL << USB_OTG_CHNUM_Pos)   /*!< 0x00000004 */
#define USB_OTG_CHNUM_3                          (0x8UL << USB_OTG_CHNUM_Pos)   /*!< 0x00000008 */
#define USB_OTG_BCNT_Pos                         (4U)
#define USB_OTG_BCNT_Msk                         (0x7FFUL << USB_OTG_BCNT_Pos)  /*!< 0x00007FF0 */
#define USB_OTG_BCNT                             USB_OTG_BCNT_Msk              /*!< Byte count */

#define USB_OTG_DPID_Pos                         (15U)
#define USB_OTG_DPID_Msk                         (0x3UL << USB_OTG_DPID_Pos)    /*!< 0x00018000 */
#define USB_OTG_DPID                             USB_OTG_DPID_Msk              /*!< Data PID */
#define USB_OTG_DPID_0                           (0x1UL << USB_OTG_DPID_Pos)    /*!< 0x00008000 */
#define USB_OTG_DPID_1                           (0x2UL << USB_OTG_DPID_Pos)    /*!< 0x00010000 */

#define USB_OTG_PKTSTS_Pos                       (17U)
#define USB_OTG_PKTSTS_Msk                       (0xFUL << USB_OTG_PKTSTS_Pos)  /*!< 0x001E0000 */
#define USB_OTG_PKTSTS                           USB_OTG_PKTSTS_Msk            /*!< Packet status */
#define USB_OTG_PKTSTS_0                         (0x1UL << USB_OTG_PKTSTS_Pos)  /*!< 0x00020000 */
#define USB_OTG_PKTSTS_1                         (0x2UL << USB_OTG_PKTSTS_Pos)  /*!< 0x00040000 */
#define USB_OTG_PKTSTS_2                         (0x4UL << USB_OTG_PKTSTS_Pos)  /*!< 0x00080000 */
#define USB_OTG_PKTSTS_3                         (0x8UL << USB_OTG_PKTSTS_Pos)  /*!< 0x00100000 */

#define USB_OTG_EPNUM_Pos                        (0U)
#define USB_OTG_EPNUM_Msk                        (0xFUL << USB_OTG_EPNUM_Pos)   /*!< 0x0000000F */
#define USB_OTG_EPNUM                            USB_OTG_EPNUM_Msk             /*!< Endpoint number */
#define USB_OTG_EPNUM_0                          (0x1UL << USB_OTG_EPNUM_Pos)   /*!< 0x00000001 */
#define USB_OTG_EPNUM_1                          (0x2UL << USB_OTG_EPNUM_Pos)   /*!< 0x00000002 */
#define USB_OTG_EPNUM_2                          (0x4UL << USB_OTG_EPNUM_Pos)   /*!< 0x00000004 */
#define USB_OTG_EPNUM_3                          (0x8UL << USB_OTG_EPNUM_Pos)   /*!< 0x00000008 */

#define USB_OTG_FRMNUM_Pos                       (21U)
#define USB_OTG_FRMNUM_Msk                       (0xFUL << USB_OTG_FRMNUM_Pos)  /*!< 0x01E00000 */
#define USB_OTG_FRMNUM                           USB_OTG_FRMNUM_Msk            /*!< Frame number */
#define USB_OTG_FRMNUM_0                         (0x1UL << USB_OTG_FRMNUM_Pos)  /*!< 0x00200000 */
#define USB_OTG_FRMNUM_1                         (0x2UL << USB_OTG_FRMNUM_Pos)  /*!< 0x00400000 */
#define USB_OTG_FRMNUM_2                         (0x4UL << USB_OTG_FRMNUM_Pos)  /*!< 0x00800000 */
#define USB_OTG_FRMNUM_3                         (0x8UL << USB_OTG_FRMNUM_Pos)  /*!< 0x01000000 */


typedef struct
{
  volatile uint32_t HCFG;             /*!< Host Configuration Register          400h */
  volatile uint32_t HFIR;             /*!< Host Frame Interval Register         404h */
  volatile uint32_t HFNUM;            /*!< Host Frame Nbr/Frame Remaining       408h */
  uint32_t Reserved40C;           /*!< Reserved                             40Ch */
  volatile uint32_t HPTXSTS;          /*!< Host Periodic Tx FIFO/ Queue Status  410h */
  volatile uint32_t HAINT;            /*!< Host All Channels Interrupt Register 414h */
  volatile uint32_t HAINTMSK;         /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;

typedef struct
{
  volatile uint32_t HCCHAR;           /*!< Host Channel Characteristics Register    500h */
  volatile uint32_t HCSPLT;           /*!< Host Channel Split Control Register      504h */
  volatile uint32_t HCINT;            /*!< Host Channel Interrupt Register          508h */
  volatile uint32_t HCINTMSK;         /*!< Host Channel Interrupt Mask Register     50Ch */
  volatile uint32_t HCTSIZ;           /*!< Host Channel Transfer Size Register      510h */
  volatile uint32_t HCDMA;            /*!< Host Channel DMA Address Register        514h */
  uint32_t Reserved[2];           /*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;


/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos            (0U)
#define GPIO_MODER_MODER0_Msk            (0x3UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk
#define GPIO_MODER_MODER0_0              (0x1UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1              (0x2UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos            (2U)
#define GPIO_MODER_MODER1_Msk            (0x3UL << GPIO_MODER_MODER1_Pos)       /*!< 0x0000000C */
#define GPIO_MODER_MODER1                GPIO_MODER_MODER1_Msk
#define GPIO_MODER_MODER1_0              (0x1UL << GPIO_MODER_MODER1_Pos)       /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1              (0x2UL << GPIO_MODER_MODER1_Pos)       /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos            (4U)
#define GPIO_MODER_MODER2_Msk            (0x3UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000030 */
#define GPIO_MODER_MODER2                GPIO_MODER_MODER2_Msk
#define GPIO_MODER_MODER2_0              (0x1UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1              (0x2UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos            (6U)
#define GPIO_MODER_MODER3_Msk            (0x3UL << GPIO_MODER_MODER3_Pos)       /*!< 0x000000C0 */
#define GPIO_MODER_MODER3                GPIO_MODER_MODER3_Msk
#define GPIO_MODER_MODER3_0              (0x1UL << GPIO_MODER_MODER3_Pos)       /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1              (0x2UL << GPIO_MODER_MODER3_Pos)       /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos            (8U)
#define GPIO_MODER_MODER4_Msk            (0x3UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000300 */
#define GPIO_MODER_MODER4                GPIO_MODER_MODER4_Msk
#define GPIO_MODER_MODER4_0              (0x1UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1              (0x2UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos            (10U)
#define GPIO_MODER_MODER5_Msk            (0x3UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000C00 */
#define GPIO_MODER_MODER5                GPIO_MODER_MODER5_Msk
#define GPIO_MODER_MODER5_0              (0x1UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1              (0x2UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos            (12U)
#define GPIO_MODER_MODER6_Msk            (0x3UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00003000 */
#define GPIO_MODER_MODER6                GPIO_MODER_MODER6_Msk
#define GPIO_MODER_MODER6_0              (0x1UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1              (0x2UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos            (14U)
#define GPIO_MODER_MODER7_Msk            (0x3UL << GPIO_MODER_MODER7_Pos)       /*!< 0x0000C000 */
#define GPIO_MODER_MODER7                GPIO_MODER_MODER7_Msk
#define GPIO_MODER_MODER7_0              (0x1UL << GPIO_MODER_MODER7_Pos)       /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1              (0x2UL << GPIO_MODER_MODER7_Pos)       /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos            (16U)
#define GPIO_MODER_MODER8_Msk            (0x3UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */
#define GPIO_MODER_MODER8                GPIO_MODER_MODER8_Msk
#define GPIO_MODER_MODER8_0              (0x1UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1              (0x2UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos            (18U)
#define GPIO_MODER_MODER9_Msk            (0x3UL << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */
#define GPIO_MODER_MODER9                GPIO_MODER_MODER9_Msk
#define GPIO_MODER_MODER9_0              (0x1UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1              (0x2UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos           (20U)
#define GPIO_MODER_MODER10_Msk           (0x3UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00300000 */
#define GPIO_MODER_MODER10               GPIO_MODER_MODER10_Msk
#define GPIO_MODER_MODER10_0             (0x1UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1             (0x2UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos           (22U)
#define GPIO_MODER_MODER11_Msk           (0x3UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00C00000 */
#define GPIO_MODER_MODER11               GPIO_MODER_MODER11_Msk
#define GPIO_MODER_MODER11_0             (0x1UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1             (0x2UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos           (24U)
#define GPIO_MODER_MODER12_Msk           (0x3UL << GPIO_MODER_MODER12_Pos)      /*!< 0x03000000 */
#define GPIO_MODER_MODER12               GPIO_MODER_MODER12_Msk
#define GPIO_MODER_MODER12_0             (0x1UL << GPIO_MODER_MODER12_Pos)      /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1             (0x2UL << GPIO_MODER_MODER12_Pos)      /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos           (26U)
#define GPIO_MODER_MODER13_Msk           (0x3UL << GPIO_MODER_MODER13_Pos)      /*!< 0x0C000000 */
#define GPIO_MODER_MODER13               GPIO_MODER_MODER13_Msk
#define GPIO_MODER_MODER13_0             (0x1UL << GPIO_MODER_MODER13_Pos)      /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1             (0x2UL << GPIO_MODER_MODER13_Pos)      /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos           (28U)
#define GPIO_MODER_MODER14_Msk           (0x3UL << GPIO_MODER_MODER14_Pos)      /*!< 0x30000000 */
#define GPIO_MODER_MODER14               GPIO_MODER_MODER14_Msk
#define GPIO_MODER_MODER14_0             (0x1UL << GPIO_MODER_MODER14_Pos)      /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1             (0x2UL << GPIO_MODER_MODER14_Pos)      /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos           (30U)
#define GPIO_MODER_MODER15_Msk           (0x3UL << GPIO_MODER_MODER15_Pos)      /*!< 0xC0000000 */
#define GPIO_MODER_MODER15               GPIO_MODER_MODER15_Msk
#define GPIO_MODER_MODER15_0             (0x1UL << GPIO_MODER_MODER15_Pos)      /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1             (0x2UL << GPIO_MODER_MODER15_Pos)      /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_MODER_MODE0_Pos             GPIO_MODER_MODER0_Pos
#define GPIO_MODER_MODE0_Msk             GPIO_MODER_MODER0_Msk
#define GPIO_MODER_MODE0                 GPIO_MODER_MODER0
#define GPIO_MODER_MODE0_0               GPIO_MODER_MODER0_0
#define GPIO_MODER_MODE0_1               GPIO_MODER_MODER0_1
#define GPIO_MODER_MODE1_Pos             GPIO_MODER_MODER1_Pos
#define GPIO_MODER_MODE1_Msk             GPIO_MODER_MODER1_Msk
#define GPIO_MODER_MODE1                 GPIO_MODER_MODER1
#define GPIO_MODER_MODE1_0               GPIO_MODER_MODER1_0
#define GPIO_MODER_MODE1_1               GPIO_MODER_MODER1_1
#define GPIO_MODER_MODE2_Pos             GPIO_MODER_MODER2_Pos
#define GPIO_MODER_MODE2_Msk             GPIO_MODER_MODER2_Msk
#define GPIO_MODER_MODE2                 GPIO_MODER_MODER2
#define GPIO_MODER_MODE2_0               GPIO_MODER_MODER2_0
#define GPIO_MODER_MODE2_1               GPIO_MODER_MODER2_1
#define GPIO_MODER_MODE3_Pos             GPIO_MODER_MODER3_Pos
#define GPIO_MODER_MODE3_Msk             GPIO_MODER_MODER3_Msk
#define GPIO_MODER_MODE3                 GPIO_MODER_MODER3
#define GPIO_MODER_MODE3_0               GPIO_MODER_MODER3_0
#define GPIO_MODER_MODE3_1               GPIO_MODER_MODER3_1
#define GPIO_MODER_MODE4_Pos             GPIO_MODER_MODER4_Pos
#define GPIO_MODER_MODE4_Msk             GPIO_MODER_MODER4_Msk
#define GPIO_MODER_MODE4                 GPIO_MODER_MODER4
#define GPIO_MODER_MODE4_0               GPIO_MODER_MODER4_0
#define GPIO_MODER_MODE4_1               GPIO_MODER_MODER4_1
#define GPIO_MODER_MODE5_Pos             GPIO_MODER_MODER5_Pos
#define GPIO_MODER_MODE5_Msk             GPIO_MODER_MODER5_Msk
#define GPIO_MODER_MODE5                 GPIO_MODER_MODER5
#define GPIO_MODER_MODE5_0               GPIO_MODER_MODER5_0
#define GPIO_MODER_MODE5_1               GPIO_MODER_MODER5_1
#define GPIO_MODER_MODE6_Pos             GPIO_MODER_MODER6_Pos
#define GPIO_MODER_MODE6_Msk             GPIO_MODER_MODER6_Msk
#define GPIO_MODER_MODE6                 GPIO_MODER_MODER6
#define GPIO_MODER_MODE6_0               GPIO_MODER_MODER6_0
#define GPIO_MODER_MODE6_1               GPIO_MODER_MODER6_1
#define GPIO_MODER_MODE7_Pos             GPIO_MODER_MODER7_Pos
#define GPIO_MODER_MODE7_Msk             GPIO_MODER_MODER7_Msk
#define GPIO_MODER_MODE7                 GPIO_MODER_MODER7
#define GPIO_MODER_MODE7_0               GPIO_MODER_MODER7_0
#define GPIO_MODER_MODE7_1               GPIO_MODER_MODER7_1
#define GPIO_MODER_MODE8_Pos             GPIO_MODER_MODER8_Pos
#define GPIO_MODER_MODE8_Msk             GPIO_MODER_MODER8_Msk
#define GPIO_MODER_MODE8                 GPIO_MODER_MODER8
#define GPIO_MODER_MODE8_0               GPIO_MODER_MODER8_0
#define GPIO_MODER_MODE8_1               GPIO_MODER_MODER8_1
#define GPIO_MODER_MODE9_Pos             GPIO_MODER_MODER9_Pos
#define GPIO_MODER_MODE9_Msk             GPIO_MODER_MODER9_Msk
#define GPIO_MODER_MODE9                 GPIO_MODER_MODER9
#define GPIO_MODER_MODE9_0               GPIO_MODER_MODER9_0
#define GPIO_MODER_MODE9_1               GPIO_MODER_MODER9_1
#define GPIO_MODER_MODE10_Pos            GPIO_MODER_MODER10_Pos
#define GPIO_MODER_MODE10_Msk            GPIO_MODER_MODER10_Msk
#define GPIO_MODER_MODE10                GPIO_MODER_MODER10
#define GPIO_MODER_MODE10_0              GPIO_MODER_MODER10_0
#define GPIO_MODER_MODE10_1              GPIO_MODER_MODER10_1
#define GPIO_MODER_MODE11_Pos            GPIO_MODER_MODER11_Pos
#define GPIO_MODER_MODE11_Msk            GPIO_MODER_MODER11_Msk
#define GPIO_MODER_MODE11                GPIO_MODER_MODER11
#define GPIO_MODER_MODE11_0              GPIO_MODER_MODER11_0
#define GPIO_MODER_MODE11_1              GPIO_MODER_MODER11_1
#define GPIO_MODER_MODE12_Pos            GPIO_MODER_MODER12_Pos
#define GPIO_MODER_MODE12_Msk            GPIO_MODER_MODER12_Msk
#define GPIO_MODER_MODE12                GPIO_MODER_MODER12
#define GPIO_MODER_MODE12_0              GPIO_MODER_MODER12_0
#define GPIO_MODER_MODE12_1              GPIO_MODER_MODER12_1
#define GPIO_MODER_MODE13_Pos            GPIO_MODER_MODER13_Pos
#define GPIO_MODER_MODE13_Msk            GPIO_MODER_MODER13_Msk
#define GPIO_MODER_MODE13                GPIO_MODER_MODER13
#define GPIO_MODER_MODE13_0              GPIO_MODER_MODER13_0
#define GPIO_MODER_MODE13_1              GPIO_MODER_MODER13_1
#define GPIO_MODER_MODE14_Pos            GPIO_MODER_MODER14_Pos
#define GPIO_MODER_MODE14_Msk            GPIO_MODER_MODER14_Msk
#define GPIO_MODER_MODE14                GPIO_MODER_MODER14
#define GPIO_MODER_MODE14_0              GPIO_MODER_MODER14_0
#define GPIO_MODER_MODE14_1              GPIO_MODER_MODER14_1
#define GPIO_MODER_MODE15_Pos            GPIO_MODER_MODER15_Pos
#define GPIO_MODER_MODE15_Msk            GPIO_MODER_MODER15_Msk
#define GPIO_MODER_MODE15                GPIO_MODER_MODER15
#define GPIO_MODER_MODE15_0              GPIO_MODER_MODER15_0
#define GPIO_MODER_MODE15_1              GPIO_MODER_MODER15_1

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos              (0U)
#define GPIO_OTYPER_OT0_Msk              (0x1UL << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                  GPIO_OTYPER_OT0_Msk
#define GPIO_OTYPER_OT1_Pos              (1U)
#define GPIO_OTYPER_OT1_Msk              (0x1UL << GPIO_OTYPER_OT1_Pos)         /*!< 0x00000002 */
#define GPIO_OTYPER_OT1                  GPIO_OTYPER_OT1_Msk
#define GPIO_OTYPER_OT2_Pos              (2U)
#define GPIO_OTYPER_OT2_Msk              (0x1UL << GPIO_OTYPER_OT2_Pos)         /*!< 0x00000004 */
#define GPIO_OTYPER_OT2                  GPIO_OTYPER_OT2_Msk
#define GPIO_OTYPER_OT3_Pos              (3U)
#define GPIO_OTYPER_OT3_Msk              (0x1UL << GPIO_OTYPER_OT3_Pos)         /*!< 0x00000008 */
#define GPIO_OTYPER_OT3                  GPIO_OTYPER_OT3_Msk
#define GPIO_OTYPER_OT4_Pos              (4U)
#define GPIO_OTYPER_OT4_Msk              (0x1UL << GPIO_OTYPER_OT4_Pos)         /*!< 0x00000010 */
#define GPIO_OTYPER_OT4                  GPIO_OTYPER_OT4_Msk
#define GPIO_OTYPER_OT5_Pos              (5U)
#define GPIO_OTYPER_OT5_Msk              (0x1UL << GPIO_OTYPER_OT5_Pos)         /*!< 0x00000020 */
#define GPIO_OTYPER_OT5                  GPIO_OTYPER_OT5_Msk
#define GPIO_OTYPER_OT6_Pos              (6U)
#define GPIO_OTYPER_OT6_Msk              (0x1UL << GPIO_OTYPER_OT6_Pos)         /*!< 0x00000040 */
#define GPIO_OTYPER_OT6                  GPIO_OTYPER_OT6_Msk
#define GPIO_OTYPER_OT7_Pos              (7U)
#define GPIO_OTYPER_OT7_Msk              (0x1UL << GPIO_OTYPER_OT7_Pos)         /*!< 0x00000080 */
#define GPIO_OTYPER_OT7                  GPIO_OTYPER_OT7_Msk
#define GPIO_OTYPER_OT8_Pos              (8U)
#define GPIO_OTYPER_OT8_Msk              (0x1UL << GPIO_OTYPER_OT8_Pos)         /*!< 0x00000100 */
#define GPIO_OTYPER_OT8                  GPIO_OTYPER_OT8_Msk
#define GPIO_OTYPER_OT9_Pos              (9U)
#define GPIO_OTYPER_OT9_Msk              (0x1UL << GPIO_OTYPER_OT9_Pos)         /*!< 0x00000200 */
#define GPIO_OTYPER_OT9                  GPIO_OTYPER_OT9_Msk
#define GPIO_OTYPER_OT10_Pos             (10U)
#define GPIO_OTYPER_OT10_Msk             (0x1UL << GPIO_OTYPER_OT10_Pos)        /*!< 0x00000400 */
#define GPIO_OTYPER_OT10                 GPIO_OTYPER_OT10_Msk
#define GPIO_OTYPER_OT11_Pos             (11U)
#define GPIO_OTYPER_OT11_Msk             (0x1UL << GPIO_OTYPER_OT11_Pos)        /*!< 0x00000800 */
#define GPIO_OTYPER_OT11                 GPIO_OTYPER_OT11_Msk
#define GPIO_OTYPER_OT12_Pos             (12U)
#define GPIO_OTYPER_OT12_Msk             (0x1UL << GPIO_OTYPER_OT12_Pos)        /*!< 0x00001000 */
#define GPIO_OTYPER_OT12                 GPIO_OTYPER_OT12_Msk
#define GPIO_OTYPER_OT13_Pos             (13U)
#define GPIO_OTYPER_OT13_Msk             (0x1UL << GPIO_OTYPER_OT13_Pos)        /*!< 0x00002000 */
#define GPIO_OTYPER_OT13                 GPIO_OTYPER_OT13_Msk
#define GPIO_OTYPER_OT14_Pos             (14U)
#define GPIO_OTYPER_OT14_Msk             (0x1UL << GPIO_OTYPER_OT14_Pos)        /*!< 0x00004000 */
#define GPIO_OTYPER_OT14                 GPIO_OTYPER_OT14_Msk
#define GPIO_OTYPER_OT15_Pos             (15U)
#define GPIO_OTYPER_OT15_Msk             (0x1UL << GPIO_OTYPER_OT15_Pos)        /*!< 0x00008000 */
#define GPIO_OTYPER_OT15                 GPIO_OTYPER_OT15_Msk

/* Legacy defines */
#define GPIO_OTYPER_OT_0                 GPIO_OTYPER_OT0
#define GPIO_OTYPER_OT_1                 GPIO_OTYPER_OT1
#define GPIO_OTYPER_OT_2                 GPIO_OTYPER_OT2
#define GPIO_OTYPER_OT_3                 GPIO_OTYPER_OT3
#define GPIO_OTYPER_OT_4                 GPIO_OTYPER_OT4
#define GPIO_OTYPER_OT_5                 GPIO_OTYPER_OT5
#define GPIO_OTYPER_OT_6                 GPIO_OTYPER_OT6
#define GPIO_OTYPER_OT_7                 GPIO_OTYPER_OT7
#define GPIO_OTYPER_OT_8                 GPIO_OTYPER_OT8
#define GPIO_OTYPER_OT_9                 GPIO_OTYPER_OT9
#define GPIO_OTYPER_OT_10                GPIO_OTYPER_OT10
#define GPIO_OTYPER_OT_11                GPIO_OTYPER_OT11
#define GPIO_OTYPER_OT_12                GPIO_OTYPER_OT12
#define GPIO_OTYPER_OT_13                GPIO_OTYPER_OT13
#define GPIO_OTYPER_OT_14                GPIO_OTYPER_OT14
#define GPIO_OTYPER_OT_15                GPIO_OTYPER_OT15

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)
#define GPIO_OSPEEDR_OSPEED0_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0             GPIO_OSPEEDR_OSPEED0_Msk
#define GPIO_OSPEEDR_OSPEED0_0           (0x1UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1           (0x2UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000002 */
#define GPIO_OSPEEDR_OSPEED1_Pos         (2U)
#define GPIO_OSPEEDR_OSPEED1_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEED1             GPIO_OSPEEDR_OSPEED1_Msk
#define GPIO_OSPEEDR_OSPEED1_0           (0x1UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEED1_1           (0x2UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000008 */
#define GPIO_OSPEEDR_OSPEED2_Pos         (4U)
#define GPIO_OSPEEDR_OSPEED2_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEED2             GPIO_OSPEEDR_OSPEED2_Msk
#define GPIO_OSPEEDR_OSPEED2_0           (0x1UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEED2_1           (0x2UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000020 */
#define GPIO_OSPEEDR_OSPEED3_Pos         (6U)
#define GPIO_OSPEEDR_OSPEED3_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEED3             GPIO_OSPEEDR_OSPEED3_Msk
#define GPIO_OSPEEDR_OSPEED3_0           (0x1UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEED3_1           (0x2UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000080 */
#define GPIO_OSPEEDR_OSPEED4_Pos         (8U)
#define GPIO_OSPEEDR_OSPEED4_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEED4             GPIO_OSPEEDR_OSPEED4_Msk
#define GPIO_OSPEEDR_OSPEED4_0           (0x1UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEED4_1           (0x2UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000200 */
#define GPIO_OSPEEDR_OSPEED5_Pos         (10U)
#define GPIO_OSPEEDR_OSPEED5_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEED5             GPIO_OSPEEDR_OSPEED5_Msk
#define GPIO_OSPEEDR_OSPEED5_0           (0x1UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEED5_1           (0x2UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000800 */
#define GPIO_OSPEEDR_OSPEED6_Pos         (12U)
#define GPIO_OSPEEDR_OSPEED6_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEED6             GPIO_OSPEEDR_OSPEED6_Msk
#define GPIO_OSPEEDR_OSPEED6_0           (0x1UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEED6_1           (0x2UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00002000 */
#define GPIO_OSPEEDR_OSPEED7_Pos         (14U)
#define GPIO_OSPEEDR_OSPEED7_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEED7             GPIO_OSPEEDR_OSPEED7_Msk
#define GPIO_OSPEEDR_OSPEED7_0           (0x1UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEED7_1           (0x2UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00008000 */
#define GPIO_OSPEEDR_OSPEED8_Pos         (16U)
#define GPIO_OSPEEDR_OSPEED8_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEED8             GPIO_OSPEEDR_OSPEED8_Msk
#define GPIO_OSPEEDR_OSPEED8_0           (0x1UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEED8_1           (0x2UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00020000 */
#define GPIO_OSPEEDR_OSPEED9_Pos         (18U)
#define GPIO_OSPEEDR_OSPEED9_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEED9             GPIO_OSPEEDR_OSPEED9_Msk
#define GPIO_OSPEEDR_OSPEED9_0           (0x1UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEED9_1           (0x2UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00080000 */
#define GPIO_OSPEEDR_OSPEED10_Pos        (20U)
#define GPIO_OSPEEDR_OSPEED10_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEED10            GPIO_OSPEEDR_OSPEED10_Msk
#define GPIO_OSPEEDR_OSPEED10_0          (0x1UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEED10_1          (0x2UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00200000 */
#define GPIO_OSPEEDR_OSPEED11_Pos        (22U)
#define GPIO_OSPEEDR_OSPEED11_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEED11            GPIO_OSPEEDR_OSPEED11_Msk
#define GPIO_OSPEEDR_OSPEED11_0          (0x1UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEED11_1          (0x2UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00800000 */
#define GPIO_OSPEEDR_OSPEED12_Pos        (24U)
#define GPIO_OSPEEDR_OSPEED12_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEED12            GPIO_OSPEEDR_OSPEED12_Msk
#define GPIO_OSPEEDR_OSPEED12_0          (0x1UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEED12_1          (0x2UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x02000000 */
#define GPIO_OSPEEDR_OSPEED13_Pos        (26U)
#define GPIO_OSPEEDR_OSPEED13_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEED13            GPIO_OSPEEDR_OSPEED13_Msk
#define GPIO_OSPEEDR_OSPEED13_0          (0x1UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEED13_1          (0x2UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x08000000 */
#define GPIO_OSPEEDR_OSPEED14_Pos        (28U)
#define GPIO_OSPEEDR_OSPEED14_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEED14            GPIO_OSPEEDR_OSPEED14_Msk
#define GPIO_OSPEEDR_OSPEED14_0          (0x1UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEED14_1          (0x2UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x20000000 */
#define GPIO_OSPEEDR_OSPEED15_Pos        (30U)
#define GPIO_OSPEEDR_OSPEED15_Msk        (0x3UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEED15            GPIO_OSPEEDR_OSPEED15_Msk
#define GPIO_OSPEEDR_OSPEED15_0          (0x1UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEED15_1          (0x2UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDR_OSPEED0
#define GPIO_OSPEEDER_OSPEEDR0_0         GPIO_OSPEEDR_OSPEED0_0
#define GPIO_OSPEEDER_OSPEEDR0_1         GPIO_OSPEEDR_OSPEED0_1
#define GPIO_OSPEEDER_OSPEEDR1           GPIO_OSPEEDR_OSPEED1
#define GPIO_OSPEEDER_OSPEEDR1_0         GPIO_OSPEEDR_OSPEED1_0
#define GPIO_OSPEEDER_OSPEEDR1_1         GPIO_OSPEEDR_OSPEED1_1
#define GPIO_OSPEEDER_OSPEEDR2           GPIO_OSPEEDR_OSPEED2
#define GPIO_OSPEEDER_OSPEEDR2_0         GPIO_OSPEEDR_OSPEED2_0
#define GPIO_OSPEEDER_OSPEEDR2_1         GPIO_OSPEEDR_OSPEED2_1
#define GPIO_OSPEEDER_OSPEEDR3           GPIO_OSPEEDR_OSPEED3
#define GPIO_OSPEEDER_OSPEEDR3_0         GPIO_OSPEEDR_OSPEED3_0
#define GPIO_OSPEEDER_OSPEEDR3_1         GPIO_OSPEEDR_OSPEED3_1
#define GPIO_OSPEEDER_OSPEEDR4           GPIO_OSPEEDR_OSPEED4
#define GPIO_OSPEEDER_OSPEEDR4_0         GPIO_OSPEEDR_OSPEED4_0
#define GPIO_OSPEEDER_OSPEEDR4_1         GPIO_OSPEEDR_OSPEED4_1
#define GPIO_OSPEEDER_OSPEEDR5           GPIO_OSPEEDR_OSPEED5
#define GPIO_OSPEEDER_OSPEEDR5_0         GPIO_OSPEEDR_OSPEED5_0
#define GPIO_OSPEEDER_OSPEEDR5_1         GPIO_OSPEEDR_OSPEED5_1
#define GPIO_OSPEEDER_OSPEEDR6           GPIO_OSPEEDR_OSPEED6
#define GPIO_OSPEEDER_OSPEEDR6_0         GPIO_OSPEEDR_OSPEED6_0
#define GPIO_OSPEEDER_OSPEEDR6_1         GPIO_OSPEEDR_OSPEED6_1
#define GPIO_OSPEEDER_OSPEEDR7           GPIO_OSPEEDR_OSPEED7
#define GPIO_OSPEEDER_OSPEEDR7_0         GPIO_OSPEEDR_OSPEED7_0
#define GPIO_OSPEEDER_OSPEEDR7_1         GPIO_OSPEEDR_OSPEED7_1
#define GPIO_OSPEEDER_OSPEEDR8           GPIO_OSPEEDR_OSPEED8
#define GPIO_OSPEEDER_OSPEEDR8_0         GPIO_OSPEEDR_OSPEED8_0
#define GPIO_OSPEEDER_OSPEEDR8_1         GPIO_OSPEEDR_OSPEED8_1
#define GPIO_OSPEEDER_OSPEEDR9           GPIO_OSPEEDR_OSPEED9
#define GPIO_OSPEEDER_OSPEEDR9_0         GPIO_OSPEEDR_OSPEED9_0
#define GPIO_OSPEEDER_OSPEEDR9_1         GPIO_OSPEEDR_OSPEED9_1
#define GPIO_OSPEEDER_OSPEEDR10          GPIO_OSPEEDR_OSPEED10
#define GPIO_OSPEEDER_OSPEEDR10_0        GPIO_OSPEEDR_OSPEED10_0
#define GPIO_OSPEEDER_OSPEEDR10_1        GPIO_OSPEEDR_OSPEED10_1
#define GPIO_OSPEEDER_OSPEEDR11          GPIO_OSPEEDR_OSPEED11
#define GPIO_OSPEEDER_OSPEEDR11_0        GPIO_OSPEEDR_OSPEED11_0
#define GPIO_OSPEEDER_OSPEEDR11_1        GPIO_OSPEEDR_OSPEED11_1
#define GPIO_OSPEEDER_OSPEEDR12          GPIO_OSPEEDR_OSPEED12
#define GPIO_OSPEEDER_OSPEEDR12_0        GPIO_OSPEEDR_OSPEED12_0
#define GPIO_OSPEEDER_OSPEEDR12_1        GPIO_OSPEEDR_OSPEED12_1
#define GPIO_OSPEEDER_OSPEEDR13          GPIO_OSPEEDR_OSPEED13
#define GPIO_OSPEEDER_OSPEEDR13_0        GPIO_OSPEEDR_OSPEED13_0
#define GPIO_OSPEEDER_OSPEEDR13_1        GPIO_OSPEEDR_OSPEED13_1
#define GPIO_OSPEEDER_OSPEEDR14          GPIO_OSPEEDR_OSPEED14
#define GPIO_OSPEEDER_OSPEEDR14_0        GPIO_OSPEEDR_OSPEED14_0
#define GPIO_OSPEEDER_OSPEEDR14_1        GPIO_OSPEEDR_OSPEED14_1
#define GPIO_OSPEEDER_OSPEEDR15          GPIO_OSPEEDR_OSPEED15
#define GPIO_OSPEEDER_OSPEEDR15_0        GPIO_OSPEEDR_OSPEED15_0
#define GPIO_OSPEEDER_OSPEEDR15_1        GPIO_OSPEEDR_OSPEED15_1

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos             (0U)
#define GPIO_PUPDR_PUPD0_Msk             (0x3UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0                 GPIO_PUPDR_PUPD0_Msk
#define GPIO_PUPDR_PUPD0_0               (0x1UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1               (0x2UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPD1_Pos             (2U)
#define GPIO_PUPDR_PUPD1_Msk             (0x3UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPD1                 GPIO_PUPDR_PUPD1_Msk
#define GPIO_PUPDR_PUPD1_0               (0x1UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPD1_1               (0x2UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPD2_Pos             (4U)
#define GPIO_PUPDR_PUPD2_Msk             (0x3UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPD2                 GPIO_PUPDR_PUPD2_Msk
#define GPIO_PUPDR_PUPD2_0               (0x1UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPD2_1               (0x2UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPD3_Pos             (6U)
#define GPIO_PUPDR_PUPD3_Msk             (0x3UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPD3                 GPIO_PUPDR_PUPD3_Msk
#define GPIO_PUPDR_PUPD3_0               (0x1UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPD3_1               (0x2UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPD4_Pos             (8U)
#define GPIO_PUPDR_PUPD4_Msk             (0x3UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPD4                 GPIO_PUPDR_PUPD4_Msk
#define GPIO_PUPDR_PUPD4_0               (0x1UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPD4_1               (0x2UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPD5_Pos             (10U)
#define GPIO_PUPDR_PUPD5_Msk             (0x3UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPD5                 GPIO_PUPDR_PUPD5_Msk
#define GPIO_PUPDR_PUPD5_0               (0x1UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPD5_1               (0x2UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPD6_Pos             (12U)
#define GPIO_PUPDR_PUPD6_Msk             (0x3UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPD6                 GPIO_PUPDR_PUPD6_Msk
#define GPIO_PUPDR_PUPD6_0               (0x1UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPD6_1               (0x2UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPD7_Pos             (14U)
#define GPIO_PUPDR_PUPD7_Msk             (0x3UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPD7                 GPIO_PUPDR_PUPD7_Msk
#define GPIO_PUPDR_PUPD7_0               (0x1UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPD7_1               (0x2UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPD8_Pos             (16U)
#define GPIO_PUPDR_PUPD8_Msk             (0x3UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPD8                 GPIO_PUPDR_PUPD8_Msk
#define GPIO_PUPDR_PUPD8_0               (0x1UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPD8_1               (0x2UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPD9_Pos             (18U)
#define GPIO_PUPDR_PUPD9_Msk             (0x3UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPD9                 GPIO_PUPDR_PUPD9_Msk
#define GPIO_PUPDR_PUPD9_0               (0x1UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPD9_1               (0x2UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPD10_Pos            (20U)
#define GPIO_PUPDR_PUPD10_Msk            (0x3UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPD10                GPIO_PUPDR_PUPD10_Msk
#define GPIO_PUPDR_PUPD10_0              (0x1UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPD10_1              (0x2UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPD11_Pos            (22U)
#define GPIO_PUPDR_PUPD11_Msk            (0x3UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPD11                GPIO_PUPDR_PUPD11_Msk
#define GPIO_PUPDR_PUPD11_0              (0x1UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPD11_1              (0x2UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPD12_Pos            (24U)
#define GPIO_PUPDR_PUPD12_Msk            (0x3UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPD12                GPIO_PUPDR_PUPD12_Msk
#define GPIO_PUPDR_PUPD12_0              (0x1UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPD12_1              (0x2UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPD13_Pos            (26U)
#define GPIO_PUPDR_PUPD13_Msk            (0x3UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPD13                GPIO_PUPDR_PUPD13_Msk
#define GPIO_PUPDR_PUPD13_0              (0x1UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPD13_1              (0x2UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPD14_Pos            (28U)
#define GPIO_PUPDR_PUPD14_Msk            (0x3UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPD14                GPIO_PUPDR_PUPD14_Msk
#define GPIO_PUPDR_PUPD14_0              (0x1UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPD14_1              (0x2UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPD15_Pos            (30U)
#define GPIO_PUPDR_PUPD15_Msk            (0x3UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPD15                GPIO_PUPDR_PUPD15_Msk
#define GPIO_PUPDR_PUPD15_0              (0x1UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPD15_1              (0x2UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPD0
#define GPIO_PUPDR_PUPDR0_0              GPIO_PUPDR_PUPD0_0
#define GPIO_PUPDR_PUPDR0_1              GPIO_PUPDR_PUPD0_1
#define GPIO_PUPDR_PUPDR1                GPIO_PUPDR_PUPD1
#define GPIO_PUPDR_PUPDR1_0              GPIO_PUPDR_PUPD1_0
#define GPIO_PUPDR_PUPDR1_1              GPIO_PUPDR_PUPD1_1
#define GPIO_PUPDR_PUPDR2                GPIO_PUPDR_PUPD2
#define GPIO_PUPDR_PUPDR2_0              GPIO_PUPDR_PUPD2_0
#define GPIO_PUPDR_PUPDR2_1              GPIO_PUPDR_PUPD2_1
#define GPIO_PUPDR_PUPDR3                GPIO_PUPDR_PUPD3
#define GPIO_PUPDR_PUPDR3_0              GPIO_PUPDR_PUPD3_0
#define GPIO_PUPDR_PUPDR3_1              GPIO_PUPDR_PUPD3_1
#define GPIO_PUPDR_PUPDR4                GPIO_PUPDR_PUPD4
#define GPIO_PUPDR_PUPDR4_0              GPIO_PUPDR_PUPD4_0
#define GPIO_PUPDR_PUPDR4_1              GPIO_PUPDR_PUPD4_1
#define GPIO_PUPDR_PUPDR5                GPIO_PUPDR_PUPD5
#define GPIO_PUPDR_PUPDR5_0              GPIO_PUPDR_PUPD5_0
#define GPIO_PUPDR_PUPDR5_1              GPIO_PUPDR_PUPD5_1
#define GPIO_PUPDR_PUPDR6                GPIO_PUPDR_PUPD6
#define GPIO_PUPDR_PUPDR6_0              GPIO_PUPDR_PUPD6_0
#define GPIO_PUPDR_PUPDR6_1              GPIO_PUPDR_PUPD6_1
#define GPIO_PUPDR_PUPDR7                GPIO_PUPDR_PUPD7
#define GPIO_PUPDR_PUPDR7_0              GPIO_PUPDR_PUPD7_0
#define GPIO_PUPDR_PUPDR7_1              GPIO_PUPDR_PUPD7_1
#define GPIO_PUPDR_PUPDR8                GPIO_PUPDR_PUPD8
#define GPIO_PUPDR_PUPDR8_0              GPIO_PUPDR_PUPD8_0
#define GPIO_PUPDR_PUPDR8_1              GPIO_PUPDR_PUPD8_1
#define GPIO_PUPDR_PUPDR9                GPIO_PUPDR_PUPD9
#define GPIO_PUPDR_PUPDR9_0              GPIO_PUPDR_PUPD9_0
#define GPIO_PUPDR_PUPDR9_1              GPIO_PUPDR_PUPD9_1
#define GPIO_PUPDR_PUPDR10               GPIO_PUPDR_PUPD10
#define GPIO_PUPDR_PUPDR10_0             GPIO_PUPDR_PUPD10_0
#define GPIO_PUPDR_PUPDR10_1             GPIO_PUPDR_PUPD10_1
#define GPIO_PUPDR_PUPDR11               GPIO_PUPDR_PUPD11
#define GPIO_PUPDR_PUPDR11_0             GPIO_PUPDR_PUPD11_0
#define GPIO_PUPDR_PUPDR11_1             GPIO_PUPDR_PUPD11_1
#define GPIO_PUPDR_PUPDR12               GPIO_PUPDR_PUPD12
#define GPIO_PUPDR_PUPDR12_0             GPIO_PUPDR_PUPD12_0
#define GPIO_PUPDR_PUPDR12_1             GPIO_PUPDR_PUPD12_1
#define GPIO_PUPDR_PUPDR13               GPIO_PUPDR_PUPD13
#define GPIO_PUPDR_PUPDR13_0             GPIO_PUPDR_PUPD13_0
#define GPIO_PUPDR_PUPDR13_1             GPIO_PUPDR_PUPD13_1
#define GPIO_PUPDR_PUPDR14               GPIO_PUPDR_PUPD14
#define GPIO_PUPDR_PUPDR14_0             GPIO_PUPDR_PUPD14_0
#define GPIO_PUPDR_PUPDR14_1             GPIO_PUPDR_PUPD14_1
#define GPIO_PUPDR_PUPDR15               GPIO_PUPDR_PUPD15
#define GPIO_PUPDR_PUPDR15_0             GPIO_PUPDR_PUPD15_0
#define GPIO_PUPDR_PUPDR15_1             GPIO_PUPDR_PUPD15_1

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos                 (0U)
#define GPIO_IDR_ID0_Msk                 (0x1UL << GPIO_IDR_ID0_Pos)            /*!< 0x00000001 */
#define GPIO_IDR_ID0                     GPIO_IDR_ID0_Msk
#define GPIO_IDR_ID1_Pos                 (1U)
#define GPIO_IDR_ID1_Msk                 (0x1UL << GPIO_IDR_ID1_Pos)            /*!< 0x00000002 */
#define GPIO_IDR_ID1                     GPIO_IDR_ID1_Msk
#define GPIO_IDR_ID2_Pos                 (2U)
#define GPIO_IDR_ID2_Msk                 (0x1UL << GPIO_IDR_ID2_Pos)            /*!< 0x00000004 */
#define GPIO_IDR_ID2                     GPIO_IDR_ID2_Msk
#define GPIO_IDR_ID3_Pos                 (3U)
#define GPIO_IDR_ID3_Msk                 (0x1UL << GPIO_IDR_ID3_Pos)            /*!< 0x00000008 */
#define GPIO_IDR_ID3                     GPIO_IDR_ID3_Msk
#define GPIO_IDR_ID4_Pos                 (4U)
#define GPIO_IDR_ID4_Msk                 (0x1UL << GPIO_IDR_ID4_Pos)            /*!< 0x00000010 */
#define GPIO_IDR_ID4                     GPIO_IDR_ID4_Msk
#define GPIO_IDR_ID5_Pos                 (5U)
#define GPIO_IDR_ID5_Msk                 (0x1UL << GPIO_IDR_ID5_Pos)            /*!< 0x00000020 */
#define GPIO_IDR_ID5                     GPIO_IDR_ID5_Msk
#define GPIO_IDR_ID6_Pos                 (6U)
#define GPIO_IDR_ID6_Msk                 (0x1UL << GPIO_IDR_ID6_Pos)            /*!< 0x00000040 */
#define GPIO_IDR_ID6                     GPIO_IDR_ID6_Msk
#define GPIO_IDR_ID7_Pos                 (7U)
#define GPIO_IDR_ID7_Msk                 (0x1UL << GPIO_IDR_ID7_Pos)            /*!< 0x00000080 */
#define GPIO_IDR_ID7                     GPIO_IDR_ID7_Msk
#define GPIO_IDR_ID8_Pos                 (8U)
#define GPIO_IDR_ID8_Msk                 (0x1UL << GPIO_IDR_ID8_Pos)            /*!< 0x00000100 */
#define GPIO_IDR_ID8                     GPIO_IDR_ID8_Msk
#define GPIO_IDR_ID9_Pos                 (9U)
#define GPIO_IDR_ID9_Msk                 (0x1UL << GPIO_IDR_ID9_Pos)            /*!< 0x00000200 */
#define GPIO_IDR_ID9                     GPIO_IDR_ID9_Msk
#define GPIO_IDR_ID10_Pos                (10U)
#define GPIO_IDR_ID10_Msk                (0x1UL << GPIO_IDR_ID10_Pos)           /*!< 0x00000400 */
#define GPIO_IDR_ID10                    GPIO_IDR_ID10_Msk
#define GPIO_IDR_ID11_Pos                (11U)
#define GPIO_IDR_ID11_Msk                (0x1UL << GPIO_IDR_ID11_Pos)           /*!< 0x00000800 */
#define GPIO_IDR_ID11                    GPIO_IDR_ID11_Msk
#define GPIO_IDR_ID12_Pos                (12U)
#define GPIO_IDR_ID12_Msk                (0x1UL << GPIO_IDR_ID12_Pos)           /*!< 0x00001000 */
#define GPIO_IDR_ID12                    GPIO_IDR_ID12_Msk
#define GPIO_IDR_ID13_Pos                (13U)
#define GPIO_IDR_ID13_Msk                (0x1UL << GPIO_IDR_ID13_Pos)           /*!< 0x00002000 */
#define GPIO_IDR_ID13                    GPIO_IDR_ID13_Msk
#define GPIO_IDR_ID14_Pos                (14U)
#define GPIO_IDR_ID14_Msk                (0x1UL << GPIO_IDR_ID14_Pos)           /*!< 0x00004000 */
#define GPIO_IDR_ID14                    GPIO_IDR_ID14_Msk
#define GPIO_IDR_ID15_Pos                (15U)
#define GPIO_IDR_ID15_Msk                (0x1UL << GPIO_IDR_ID15_Pos)           /*!< 0x00008000 */
#define GPIO_IDR_ID15                    GPIO_IDR_ID15_Msk

/* Legacy defines */
#define GPIO_IDR_IDR_0                   GPIO_IDR_ID0
#define GPIO_IDR_IDR_1                   GPIO_IDR_ID1
#define GPIO_IDR_IDR_2                   GPIO_IDR_ID2
#define GPIO_IDR_IDR_3                   GPIO_IDR_ID3
#define GPIO_IDR_IDR_4                   GPIO_IDR_ID4
#define GPIO_IDR_IDR_5                   GPIO_IDR_ID5
#define GPIO_IDR_IDR_6                   GPIO_IDR_ID6
#define GPIO_IDR_IDR_7                   GPIO_IDR_ID7
#define GPIO_IDR_IDR_8                   GPIO_IDR_ID8
#define GPIO_IDR_IDR_9                   GPIO_IDR_ID9
#define GPIO_IDR_IDR_10                  GPIO_IDR_ID10
#define GPIO_IDR_IDR_11                  GPIO_IDR_ID11
#define GPIO_IDR_IDR_12                  GPIO_IDR_ID12
#define GPIO_IDR_IDR_13                  GPIO_IDR_ID13
#define GPIO_IDR_IDR_14                  GPIO_IDR_ID14
#define GPIO_IDR_IDR_15                  GPIO_IDR_ID15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos                 (0U)
#define GPIO_ODR_OD0_Msk                 (0x1UL << GPIO_ODR_OD0_Pos)            /*!< 0x00000001 */
#define GPIO_ODR_OD0                     GPIO_ODR_OD0_Msk
#define GPIO_ODR_OD1_Pos                 (1U)
#define GPIO_ODR_OD1_Msk                 (0x1UL << GPIO_ODR_OD1_Pos)            /*!< 0x00000002 */
#define GPIO_ODR_OD1                     GPIO_ODR_OD1_Msk
#define GPIO_ODR_OD2_Pos                 (2U)
#define GPIO_ODR_OD2_Msk                 (0x1UL << GPIO_ODR_OD2_Pos)            /*!< 0x00000004 */
#define GPIO_ODR_OD2                     GPIO_ODR_OD2_Msk
#define GPIO_ODR_OD3_Pos                 (3U)
#define GPIO_ODR_OD3_Msk                 (0x1UL << GPIO_ODR_OD3_Pos)            /*!< 0x00000008 */
#define GPIO_ODR_OD3                     GPIO_ODR_OD3_Msk
#define GPIO_ODR_OD4_Pos                 (4U)
#define GPIO_ODR_OD4_Msk                 (0x1UL << GPIO_ODR_OD4_Pos)            /*!< 0x00000010 */
#define GPIO_ODR_OD4                     GPIO_ODR_OD4_Msk
#define GPIO_ODR_OD5_Pos                 (5U)
#define GPIO_ODR_OD5_Msk                 (0x1UL << GPIO_ODR_OD5_Pos)            /*!< 0x00000020 */
#define GPIO_ODR_OD5                     GPIO_ODR_OD5_Msk
#define GPIO_ODR_OD6_Pos                 (6U)
#define GPIO_ODR_OD6_Msk                 (0x1UL << GPIO_ODR_OD6_Pos)            /*!< 0x00000040 */
#define GPIO_ODR_OD6                     GPIO_ODR_OD6_Msk
#define GPIO_ODR_OD7_Pos                 (7U)
#define GPIO_ODR_OD7_Msk                 (0x1UL << GPIO_ODR_OD7_Pos)            /*!< 0x00000080 */
#define GPIO_ODR_OD7                     GPIO_ODR_OD7_Msk
#define GPIO_ODR_OD8_Pos                 (8U)
#define GPIO_ODR_OD8_Msk                 (0x1UL << GPIO_ODR_OD8_Pos)            /*!< 0x00000100 */
#define GPIO_ODR_OD8                     GPIO_ODR_OD8_Msk
#define GPIO_ODR_OD9_Pos                 (9U)
#define GPIO_ODR_OD9_Msk                 (0x1UL << GPIO_ODR_OD9_Pos)            /*!< 0x00000200 */
#define GPIO_ODR_OD9                     GPIO_ODR_OD9_Msk
#define GPIO_ODR_OD10_Pos                (10U)
#define GPIO_ODR_OD10_Msk                (0x1UL << GPIO_ODR_OD10_Pos)           /*!< 0x00000400 */
#define GPIO_ODR_OD10                    GPIO_ODR_OD10_Msk
#define GPIO_ODR_OD11_Pos                (11U)
#define GPIO_ODR_OD11_Msk                (0x1UL << GPIO_ODR_OD11_Pos)           /*!< 0x00000800 */
#define GPIO_ODR_OD11                    GPIO_ODR_OD11_Msk
#define GPIO_ODR_OD12_Pos                (12U)
#define GPIO_ODR_OD12_Msk                (0x1UL << GPIO_ODR_OD12_Pos)           /*!< 0x00001000 */
#define GPIO_ODR_OD12                    GPIO_ODR_OD12_Msk
#define GPIO_ODR_OD13_Pos                (13U)
#define GPIO_ODR_OD13_Msk                (0x1UL << GPIO_ODR_OD13_Pos)           /*!< 0x00002000 */
#define GPIO_ODR_OD13                    GPIO_ODR_OD13_Msk
#define GPIO_ODR_OD14_Pos                (14U)
#define GPIO_ODR_OD14_Msk                (0x1UL << GPIO_ODR_OD14_Pos)           /*!< 0x00004000 */
#define GPIO_ODR_OD14                    GPIO_ODR_OD14_Msk
#define GPIO_ODR_OD15_Pos                (15U)
#define GPIO_ODR_OD15_Msk                (0x1UL << GPIO_ODR_OD15_Pos)           /*!< 0x00008000 */
#define GPIO_ODR_OD15                    GPIO_ODR_OD15_Msk
/* Legacy defines */
#define GPIO_ODR_ODR_0                   GPIO_ODR_OD0
#define GPIO_ODR_ODR_1                   GPIO_ODR_OD1
#define GPIO_ODR_ODR_2                   GPIO_ODR_OD2
#define GPIO_ODR_ODR_3                   GPIO_ODR_OD3
#define GPIO_ODR_ODR_4                   GPIO_ODR_OD4
#define GPIO_ODR_ODR_5                   GPIO_ODR_OD5
#define GPIO_ODR_ODR_6                   GPIO_ODR_OD6
#define GPIO_ODR_ODR_7                   GPIO_ODR_OD7
#define GPIO_ODR_ODR_8                   GPIO_ODR_OD8
#define GPIO_ODR_ODR_9                   GPIO_ODR_OD9
#define GPIO_ODR_ODR_10                  GPIO_ODR_OD10
#define GPIO_ODR_ODR_11                  GPIO_ODR_OD11
#define GPIO_ODR_ODR_12                  GPIO_ODR_OD12
#define GPIO_ODR_ODR_13                  GPIO_ODR_OD13
#define GPIO_ODR_ODR_14                  GPIO_ODR_OD14
#define GPIO_ODR_ODR_15                  GPIO_ODR_OD15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos                (0U)
#define GPIO_BSRR_BS0_Msk                (0x1UL << GPIO_BSRR_BS0_Pos)           /*!< 0x00000001 */
#define GPIO_BSRR_BS0                    GPIO_BSRR_BS0_Msk
#define GPIO_BSRR_BS1_Pos                (1U)
#define GPIO_BSRR_BS1_Msk                (0x1UL << GPIO_BSRR_BS1_Pos)           /*!< 0x00000002 */
#define GPIO_BSRR_BS1                    GPIO_BSRR_BS1_Msk
#define GPIO_BSRR_BS2_Pos                (2U)
#define GPIO_BSRR_BS2_Msk                (0x1UL << GPIO_BSRR_BS2_Pos)           /*!< 0x00000004 */
#define GPIO_BSRR_BS2                    GPIO_BSRR_BS2_Msk
#define GPIO_BSRR_BS3_Pos                (3U)
#define GPIO_BSRR_BS3_Msk                (0x1UL << GPIO_BSRR_BS3_Pos)           /*!< 0x00000008 */
#define GPIO_BSRR_BS3                    GPIO_BSRR_BS3_Msk
#define GPIO_BSRR_BS4_Pos                (4U)
#define GPIO_BSRR_BS4_Msk                (0x1UL << GPIO_BSRR_BS4_Pos)           /*!< 0x00000010 */
#define GPIO_BSRR_BS4                    GPIO_BSRR_BS4_Msk
#define GPIO_BSRR_BS5_Pos                (5U)
#define GPIO_BSRR_BS5_Msk                (0x1UL << GPIO_BSRR_BS5_Pos)           /*!< 0x00000020 */
#define GPIO_BSRR_BS5                    GPIO_BSRR_BS5_Msk
#define GPIO_BSRR_BS6_Pos                (6U)
#define GPIO_BSRR_BS6_Msk                (0x1UL << GPIO_BSRR_BS6_Pos)           /*!< 0x00000040 */
#define GPIO_BSRR_BS6                    GPIO_BSRR_BS6_Msk
#define GPIO_BSRR_BS7_Pos                (7U)
#define GPIO_BSRR_BS7_Msk                (0x1UL << GPIO_BSRR_BS7_Pos)           /*!< 0x00000080 */
#define GPIO_BSRR_BS7                    GPIO_BSRR_BS7_Msk
#define GPIO_BSRR_BS8_Pos                (8U)
#define GPIO_BSRR_BS8_Msk                (0x1UL << GPIO_BSRR_BS8_Pos)           /*!< 0x00000100 */
#define GPIO_BSRR_BS8                    GPIO_BSRR_BS8_Msk
#define GPIO_BSRR_BS9_Pos                (9U)
#define GPIO_BSRR_BS9_Msk                (0x1UL << GPIO_BSRR_BS9_Pos)           /*!< 0x00000200 */
#define GPIO_BSRR_BS9                    GPIO_BSRR_BS9_Msk
#define GPIO_BSRR_BS10_Pos               (10U)
#define GPIO_BSRR_BS10_Msk               (0x1UL << GPIO_BSRR_BS10_Pos)          /*!< 0x00000400 */
#define GPIO_BSRR_BS10                   GPIO_BSRR_BS10_Msk
#define GPIO_BSRR_BS11_Pos               (11U)
#define GPIO_BSRR_BS11_Msk               (0x1UL << GPIO_BSRR_BS11_Pos)          /*!< 0x00000800 */
#define GPIO_BSRR_BS11                   GPIO_BSRR_BS11_Msk
#define GPIO_BSRR_BS12_Pos               (12U)
#define GPIO_BSRR_BS12_Msk               (0x1UL << GPIO_BSRR_BS12_Pos)          /*!< 0x00001000 */
#define GPIO_BSRR_BS12                   GPIO_BSRR_BS12_Msk
#define GPIO_BSRR_BS13_Pos               (13U)
#define GPIO_BSRR_BS13_Msk               (0x1UL << GPIO_BSRR_BS13_Pos)          /*!< 0x00002000 */
#define GPIO_BSRR_BS13                   GPIO_BSRR_BS13_Msk
#define GPIO_BSRR_BS14_Pos               (14U)
#define GPIO_BSRR_BS14_Msk               (0x1UL << GPIO_BSRR_BS14_Pos)          /*!< 0x00004000 */
#define GPIO_BSRR_BS14                   GPIO_BSRR_BS14_Msk
#define GPIO_BSRR_BS15_Pos               (15U)
#define GPIO_BSRR_BS15_Msk               (0x1UL << GPIO_BSRR_BS15_Pos)          /*!< 0x00008000 */
#define GPIO_BSRR_BS15                   GPIO_BSRR_BS15_Msk
#define GPIO_BSRR_BR0_Pos                (16U)
#define GPIO_BSRR_BR0_Msk                (0x1UL << GPIO_BSRR_BR0_Pos)           /*!< 0x00010000 */
#define GPIO_BSRR_BR0                    GPIO_BSRR_BR0_Msk
#define GPIO_BSRR_BR1_Pos                (17U)
#define GPIO_BSRR_BR1_Msk                (0x1UL << GPIO_BSRR_BR1_Pos)           /*!< 0x00020000 */
#define GPIO_BSRR_BR1                    GPIO_BSRR_BR1_Msk
#define GPIO_BSRR_BR2_Pos                (18U)
#define GPIO_BSRR_BR2_Msk                (0x1UL << GPIO_BSRR_BR2_Pos)           /*!< 0x00040000 */
#define GPIO_BSRR_BR2                    GPIO_BSRR_BR2_Msk
#define GPIO_BSRR_BR3_Pos                (19U)
#define GPIO_BSRR_BR3_Msk                (0x1UL << GPIO_BSRR_BR3_Pos)           /*!< 0x00080000 */
#define GPIO_BSRR_BR3                    GPIO_BSRR_BR3_Msk
#define GPIO_BSRR_BR4_Pos                (20U)
#define GPIO_BSRR_BR4_Msk                (0x1UL << GPIO_BSRR_BR4_Pos)           /*!< 0x00100000 */
#define GPIO_BSRR_BR4                    GPIO_BSRR_BR4_Msk
#define GPIO_BSRR_BR5_Pos                (21U)
#define GPIO_BSRR_BR5_Msk                (0x1UL << GPIO_BSRR_BR5_Pos)           /*!< 0x00200000 */
#define GPIO_BSRR_BR5                    GPIO_BSRR_BR5_Msk
#define GPIO_BSRR_BR6_Pos                (22U)
#define GPIO_BSRR_BR6_Msk                (0x1UL << GPIO_BSRR_BR6_Pos)           /*!< 0x00400000 */
#define GPIO_BSRR_BR6                    GPIO_BSRR_BR6_Msk
#define GPIO_BSRR_BR7_Pos                (23U)
#define GPIO_BSRR_BR7_Msk                (0x1UL << GPIO_BSRR_BR7_Pos)           /*!< 0x00800000 */
#define GPIO_BSRR_BR7                    GPIO_BSRR_BR7_Msk
#define GPIO_BSRR_BR8_Pos                (24U)
#define GPIO_BSRR_BR8_Msk                (0x1UL << GPIO_BSRR_BR8_Pos)           /*!< 0x01000000 */
#define GPIO_BSRR_BR8                    GPIO_BSRR_BR8_Msk
#define GPIO_BSRR_BR9_Pos                (25U)
#define GPIO_BSRR_BR9_Msk                (0x1UL << GPIO_BSRR_BR9_Pos)           /*!< 0x02000000 */
#define GPIO_BSRR_BR9                    GPIO_BSRR_BR9_Msk
#define GPIO_BSRR_BR10_Pos               (26U)
#define GPIO_BSRR_BR10_Msk               (0x1UL << GPIO_BSRR_BR10_Pos)          /*!< 0x04000000 */
#define GPIO_BSRR_BR10                   GPIO_BSRR_BR10_Msk
#define GPIO_BSRR_BR11_Pos               (27U)
#define GPIO_BSRR_BR11_Msk               (0x1UL << GPIO_BSRR_BR11_Pos)          /*!< 0x08000000 */
#define GPIO_BSRR_BR11                   GPIO_BSRR_BR11_Msk
#define GPIO_BSRR_BR12_Pos               (28U)
#define GPIO_BSRR_BR12_Msk               (0x1UL << GPIO_BSRR_BR12_Pos)          /*!< 0x10000000 */
#define GPIO_BSRR_BR12                   GPIO_BSRR_BR12_Msk
#define GPIO_BSRR_BR13_Pos               (29U)
#define GPIO_BSRR_BR13_Msk               (0x1UL << GPIO_BSRR_BR13_Pos)          /*!< 0x20000000 */
#define GPIO_BSRR_BR13                   GPIO_BSRR_BR13_Msk
#define GPIO_BSRR_BR14_Pos               (30U)
#define GPIO_BSRR_BR14_Msk               (0x1UL << GPIO_BSRR_BR14_Pos)          /*!< 0x40000000 */
#define GPIO_BSRR_BR14                   GPIO_BSRR_BR14_Msk
#define GPIO_BSRR_BR15_Pos               (31U)
#define GPIO_BSRR_BR15_Msk               (0x1UL << GPIO_BSRR_BR15_Pos)          /*!< 0x80000000 */
#define GPIO_BSRR_BR15                   GPIO_BSRR_BR15_Msk

/* Legacy defines */
#define GPIO_BSRR_BS_0                   GPIO_BSRR_BS0
#define GPIO_BSRR_BS_1                   GPIO_BSRR_BS1
#define GPIO_BSRR_BS_2                   GPIO_BSRR_BS2
#define GPIO_BSRR_BS_3                   GPIO_BSRR_BS3
#define GPIO_BSRR_BS_4                   GPIO_BSRR_BS4
#define GPIO_BSRR_BS_5                   GPIO_BSRR_BS5
#define GPIO_BSRR_BS_6                   GPIO_BSRR_BS6
#define GPIO_BSRR_BS_7                   GPIO_BSRR_BS7
#define GPIO_BSRR_BS_8                   GPIO_BSRR_BS8
#define GPIO_BSRR_BS_9                   GPIO_BSRR_BS9
#define GPIO_BSRR_BS_10                  GPIO_BSRR_BS10
#define GPIO_BSRR_BS_11                  GPIO_BSRR_BS11
#define GPIO_BSRR_BS_12                  GPIO_BSRR_BS12
#define GPIO_BSRR_BS_13                  GPIO_BSRR_BS13
#define GPIO_BSRR_BS_14                  GPIO_BSRR_BS14
#define GPIO_BSRR_BS_15                  GPIO_BSRR_BS15
#define GPIO_BSRR_BR_0                   GPIO_BSRR_BR0
#define GPIO_BSRR_BR_1                   GPIO_BSRR_BR1
#define GPIO_BSRR_BR_2                   GPIO_BSRR_BR2
#define GPIO_BSRR_BR_3                   GPIO_BSRR_BR3
#define GPIO_BSRR_BR_4                   GPIO_BSRR_BR4
#define GPIO_BSRR_BR_5                   GPIO_BSRR_BR5
#define GPIO_BSRR_BR_6                   GPIO_BSRR_BR6
#define GPIO_BSRR_BR_7                   GPIO_BSRR_BR7
#define GPIO_BSRR_BR_8                   GPIO_BSRR_BR8
#define GPIO_BSRR_BR_9                   GPIO_BSRR_BR9
#define GPIO_BSRR_BR_10                  GPIO_BSRR_BR10
#define GPIO_BSRR_BR_11                  GPIO_BSRR_BR11
#define GPIO_BSRR_BR_12                  GPIO_BSRR_BR12
#define GPIO_BSRR_BR_13                  GPIO_BSRR_BR13
#define GPIO_BSRR_BR_14                  GPIO_BSRR_BR14
#define GPIO_BSRR_BR_15                  GPIO_BSRR_BR15
#define GPIO_BRR_BR0                     GPIO_BSRR_BR0
#define GPIO_BRR_BR0_Pos                 GPIO_BSRR_BR0_Pos
#define GPIO_BRR_BR0_Msk                 GPIO_BSRR_BR0_Msk
#define GPIO_BRR_BR1                     GPIO_BSRR_BR1
#define GPIO_BRR_BR1_Pos                 GPIO_BSRR_BR1_Pos
#define GPIO_BRR_BR1_Msk                 GPIO_BSRR_BR1_Msk
#define GPIO_BRR_BR2                     GPIO_BSRR_BR2
#define GPIO_BRR_BR2_Pos                 GPIO_BSRR_BR2_Pos
#define GPIO_BRR_BR2_Msk                 GPIO_BSRR_BR2_Msk
#define GPIO_BRR_BR3                     GPIO_BSRR_BR3
#define GPIO_BRR_BR3_Pos                 GPIO_BSRR_BR3_Pos
#define GPIO_BRR_BR3_Msk                 GPIO_BSRR_BR3_Msk
#define GPIO_BRR_BR4                     GPIO_BSRR_BR4
#define GPIO_BRR_BR4_Pos                 GPIO_BSRR_BR4_Pos
#define GPIO_BRR_BR4_Msk                 GPIO_BSRR_BR4_Msk
#define GPIO_BRR_BR5                     GPIO_BSRR_BR5
#define GPIO_BRR_BR5_Pos                 GPIO_BSRR_BR5_Pos
#define GPIO_BRR_BR5_Msk                 GPIO_BSRR_BR5_Msk
#define GPIO_BRR_BR6                     GPIO_BSRR_BR6
#define GPIO_BRR_BR6_Pos                 GPIO_BSRR_BR6_Pos
#define GPIO_BRR_BR6_Msk                 GPIO_BSRR_BR6_Msk
#define GPIO_BRR_BR7                     GPIO_BSRR_BR7
#define GPIO_BRR_BR7_Pos                 GPIO_BSRR_BR7_Pos
#define GPIO_BRR_BR7_Msk                 GPIO_BSRR_BR7_Msk
#define GPIO_BRR_BR8                     GPIO_BSRR_BR8
#define GPIO_BRR_BR8_Pos                 GPIO_BSRR_BR8_Pos
#define GPIO_BRR_BR8_Msk                 GPIO_BSRR_BR8_Msk
#define GPIO_BRR_BR9                     GPIO_BSRR_BR9
#define GPIO_BRR_BR9_Pos                 GPIO_BSRR_BR9_Pos
#define GPIO_BRR_BR9_Msk                 GPIO_BSRR_BR9_Msk
#define GPIO_BRR_BR10                    GPIO_BSRR_BR10
#define GPIO_BRR_BR10_Pos                GPIO_BSRR_BR10_Pos
#define GPIO_BRR_BR10_Msk                GPIO_BSRR_BR10_Msk
#define GPIO_BRR_BR11                    GPIO_BSRR_BR11
#define GPIO_BRR_BR11_Pos                GPIO_BSRR_BR11_Pos
#define GPIO_BRR_BR11_Msk                GPIO_BSRR_BR11_Msk
#define GPIO_BRR_BR12                    GPIO_BSRR_BR12
#define GPIO_BRR_BR12_Pos                GPIO_BSRR_BR12_Pos
#define GPIO_BRR_BR12_Msk                GPIO_BSRR_BR12_Msk
#define GPIO_BRR_BR13                    GPIO_BSRR_BR13
#define GPIO_BRR_BR13_Pos                GPIO_BSRR_BR13_Pos
#define GPIO_BRR_BR13_Msk                GPIO_BSRR_BR13_Msk
#define GPIO_BRR_BR14                    GPIO_BSRR_BR14
#define GPIO_BRR_BR14_Pos                GPIO_BSRR_BR14_Pos
#define GPIO_BRR_BR14_Msk                GPIO_BSRR_BR14_Msk
#define GPIO_BRR_BR15                    GPIO_BSRR_BR15
#define GPIO_BRR_BR15_Pos                GPIO_BSRR_BR15_Pos
#define GPIO_BRR_BR15_Msk                GPIO_BSRR_BR15_Msk
/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos               (0U)
#define GPIO_LCKR_LCK0_Msk               (0x1UL << GPIO_LCKR_LCK0_Pos)          /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                   GPIO_LCKR_LCK0_Msk
#define GPIO_LCKR_LCK1_Pos               (1U)
#define GPIO_LCKR_LCK1_Msk               (0x1UL << GPIO_LCKR_LCK1_Pos)          /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                   GPIO_LCKR_LCK1_Msk
#define GPIO_LCKR_LCK2_Pos               (2U)
#define GPIO_LCKR_LCK2_Msk               (0x1UL << GPIO_LCKR_LCK2_Pos)          /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                   GPIO_LCKR_LCK2_Msk
#define GPIO_LCKR_LCK3_Pos               (3U)
#define GPIO_LCKR_LCK3_Msk               (0x1UL << GPIO_LCKR_LCK3_Pos)          /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                   GPIO_LCKR_LCK3_Msk
#define GPIO_LCKR_LCK4_Pos               (4U)
#define GPIO_LCKR_LCK4_Msk               (0x1UL << GPIO_LCKR_LCK4_Pos)          /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                   GPIO_LCKR_LCK4_Msk
#define GPIO_LCKR_LCK5_Pos               (5U)
#define GPIO_LCKR_LCK5_Msk               (0x1UL << GPIO_LCKR_LCK5_Pos)          /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                   GPIO_LCKR_LCK5_Msk
#define GPIO_LCKR_LCK6_Pos               (6U)
#define GPIO_LCKR_LCK6_Msk               (0x1UL << GPIO_LCKR_LCK6_Pos)          /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                   GPIO_LCKR_LCK6_Msk
#define GPIO_LCKR_LCK7_Pos               (7U)
#define GPIO_LCKR_LCK7_Msk               (0x1UL << GPIO_LCKR_LCK7_Pos)          /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                   GPIO_LCKR_LCK7_Msk
#define GPIO_LCKR_LCK8_Pos               (8U)
#define GPIO_LCKR_LCK8_Msk               (0x1UL << GPIO_LCKR_LCK8_Pos)          /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                   GPIO_LCKR_LCK8_Msk
#define GPIO_LCKR_LCK9_Pos               (9U)
#define GPIO_LCKR_LCK9_Msk               (0x1UL << GPIO_LCKR_LCK9_Pos)          /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                   GPIO_LCKR_LCK9_Msk
#define GPIO_LCKR_LCK10_Pos              (10U)
#define GPIO_LCKR_LCK10_Msk              (0x1UL << GPIO_LCKR_LCK10_Pos)         /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                  GPIO_LCKR_LCK10_Msk
#define GPIO_LCKR_LCK11_Pos              (11U)
#define GPIO_LCKR_LCK11_Msk              (0x1UL << GPIO_LCKR_LCK11_Pos)         /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                  GPIO_LCKR_LCK11_Msk
#define GPIO_LCKR_LCK12_Pos              (12U)
#define GPIO_LCKR_LCK12_Msk              (0x1UL << GPIO_LCKR_LCK12_Pos)         /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                  GPIO_LCKR_LCK12_Msk
#define GPIO_LCKR_LCK13_Pos              (13U)
#define GPIO_LCKR_LCK13_Msk              (0x1UL << GPIO_LCKR_LCK13_Pos)         /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                  GPIO_LCKR_LCK13_Msk
#define GPIO_LCKR_LCK14_Pos              (14U)
#define GPIO_LCKR_LCK14_Msk              (0x1UL << GPIO_LCKR_LCK14_Pos)         /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                  GPIO_LCKR_LCK14_Msk
#define GPIO_LCKR_LCK15_Pos              (15U)
#define GPIO_LCKR_LCK15_Msk              (0x1UL << GPIO_LCKR_LCK15_Pos)         /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                  GPIO_LCKR_LCK15_Msk
#define GPIO_LCKR_LCKK_Pos               (16U)
#define GPIO_LCKR_LCKK_Msk               (0x1UL << GPIO_LCKR_LCKK_Pos)          /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                   GPIO_LCKR_LCKK_Msk
/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0_Pos             (0U)
#define GPIO_AFRL_AFSEL0_Msk             (0xFUL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0                 GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFSEL0_0               (0x1UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000001 */
#define GPIO_AFRL_AFSEL0_1               (0x2UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000002 */
#define GPIO_AFRL_AFSEL0_2               (0x4UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000004 */
#define GPIO_AFRL_AFSEL0_3               (0x8UL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000008 */
#define GPIO_AFRL_AFSEL1_Pos             (4U)
#define GPIO_AFRL_AFSEL1_Msk             (0xFUL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1                 GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFSEL1_0               (0x1UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000010 */
#define GPIO_AFRL_AFSEL1_1               (0x2UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000020 */
#define GPIO_AFRL_AFSEL1_2               (0x4UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000040 */
#define GPIO_AFRL_AFSEL1_3               (0x8UL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000080 */
#define GPIO_AFRL_AFSEL2_Pos             (8U)
#define GPIO_AFRL_AFSEL2_Msk             (0xFUL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2                 GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFSEL2_0               (0x1UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000100 */
#define GPIO_AFRL_AFSEL2_1               (0x2UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000200 */
#define GPIO_AFRL_AFSEL2_2               (0x4UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000400 */
#define GPIO_AFRL_AFSEL2_3               (0x8UL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000800 */
#define GPIO_AFRL_AFSEL3_Pos             (12U)
#define GPIO_AFRL_AFSEL3_Msk             (0xFUL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3                 GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFSEL3_0               (0x1UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00001000 */
#define GPIO_AFRL_AFSEL3_1               (0x2UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00002000 */
#define GPIO_AFRL_AFSEL3_2               (0x4UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00004000 */
#define GPIO_AFRL_AFSEL3_3               (0x8UL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00008000 */
#define GPIO_AFRL_AFSEL4_Pos             (16U)
#define GPIO_AFRL_AFSEL4_Msk             (0xFUL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4                 GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFSEL4_0               (0x1UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00010000 */
#define GPIO_AFRL_AFSEL4_1               (0x2UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00020000 */
#define GPIO_AFRL_AFSEL4_2               (0x4UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00040000 */
#define GPIO_AFRL_AFSEL4_3               (0x8UL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00080000 */
#define GPIO_AFRL_AFSEL5_Pos             (20U)
#define GPIO_AFRL_AFSEL5_Msk             (0xFUL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5                 GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFSEL5_0               (0x1UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00100000 */
#define GPIO_AFRL_AFSEL5_1               (0x2UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00200000 */
#define GPIO_AFRL_AFSEL5_2               (0x4UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00400000 */
#define GPIO_AFRL_AFSEL5_3               (0x8UL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00800000 */
#define GPIO_AFRL_AFSEL6_Pos             (24U)
#define GPIO_AFRL_AFSEL6_Msk             (0xFUL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6                 GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFSEL6_0               (0x1UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x01000000 */
#define GPIO_AFRL_AFSEL6_1               (0x2UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x02000000 */
#define GPIO_AFRL_AFSEL6_2               (0x4UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x04000000 */
#define GPIO_AFRL_AFSEL6_3               (0x8UL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x08000000 */
#define GPIO_AFRL_AFSEL7_Pos             (28U)
#define GPIO_AFRL_AFSEL7_Msk             (0xFUL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7                 GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFSEL7_0               (0x1UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x10000000 */
#define GPIO_AFRL_AFSEL7_1               (0x2UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x20000000 */
#define GPIO_AFRL_AFSEL7_2               (0x4UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x40000000 */
#define GPIO_AFRL_AFSEL7_3               (0x8UL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRL_AFRL0                  GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL0_0                GPIO_AFRL_AFSEL0_0
#define GPIO_AFRL_AFRL0_1                GPIO_AFRL_AFSEL0_1
#define GPIO_AFRL_AFRL0_2                GPIO_AFRL_AFSEL0_2
#define GPIO_AFRL_AFRL0_3                GPIO_AFRL_AFSEL0_3
#define GPIO_AFRL_AFRL1                  GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL1_0                GPIO_AFRL_AFSEL1_0
#define GPIO_AFRL_AFRL1_1                GPIO_AFRL_AFSEL1_1
#define GPIO_AFRL_AFRL1_2                GPIO_AFRL_AFSEL1_2
#define GPIO_AFRL_AFRL1_3                GPIO_AFRL_AFSEL1_3
#define GPIO_AFRL_AFRL2                  GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL2_0                GPIO_AFRL_AFSEL2_0
#define GPIO_AFRL_AFRL2_1                GPIO_AFRL_AFSEL2_1
#define GPIO_AFRL_AFRL2_2                GPIO_AFRL_AFSEL2_2
#define GPIO_AFRL_AFRL2_3                GPIO_AFRL_AFSEL2_3
#define GPIO_AFRL_AFRL3                  GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL3_0                GPIO_AFRL_AFSEL3_0
#define GPIO_AFRL_AFRL3_1                GPIO_AFRL_AFSEL3_1
#define GPIO_AFRL_AFRL3_2                GPIO_AFRL_AFSEL3_2
#define GPIO_AFRL_AFRL3_3                GPIO_AFRL_AFSEL3_3
#define GPIO_AFRL_AFRL4                  GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL4_0                GPIO_AFRL_AFSEL4_0
#define GPIO_AFRL_AFRL4_1                GPIO_AFRL_AFSEL4_1
#define GPIO_AFRL_AFRL4_2                GPIO_AFRL_AFSEL4_2
#define GPIO_AFRL_AFRL4_3                GPIO_AFRL_AFSEL4_3
#define GPIO_AFRL_AFRL5                  GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL5_0                GPIO_AFRL_AFSEL5_0
#define GPIO_AFRL_AFRL5_1                GPIO_AFRL_AFSEL5_1
#define GPIO_AFRL_AFRL5_2                GPIO_AFRL_AFSEL5_2
#define GPIO_AFRL_AFRL5_3                GPIO_AFRL_AFSEL5_3
#define GPIO_AFRL_AFRL6                  GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL6_0                GPIO_AFRL_AFSEL6_0
#define GPIO_AFRL_AFRL6_1                GPIO_AFRL_AFSEL6_1
#define GPIO_AFRL_AFRL6_2                GPIO_AFRL_AFSEL6_2
#define GPIO_AFRL_AFRL6_3                GPIO_AFRL_AFSEL6_3
#define GPIO_AFRL_AFRL7                  GPIO_AFRL_AFSEL7
#define GPIO_AFRL_AFRL7_0                GPIO_AFRL_AFSEL7_0
#define GPIO_AFRL_AFRL7_1                GPIO_AFRL_AFSEL7_1
#define GPIO_AFRL_AFRL7_2                GPIO_AFRL_AFSEL7_2
#define GPIO_AFRL_AFRL7_3                GPIO_AFRL_AFSEL7_3

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8_Pos             (0U)
#define GPIO_AFRH_AFSEL8_Msk             (0xFUL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8                 GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFSEL8_0               (0x1UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000001 */
#define GPIO_AFRH_AFSEL8_1               (0x2UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000002 */
#define GPIO_AFRH_AFSEL8_2               (0x4UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000004 */
#define GPIO_AFRH_AFSEL8_3               (0x8UL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000008 */
#define GPIO_AFRH_AFSEL9_Pos             (4U)
#define GPIO_AFRH_AFSEL9_Msk             (0xFUL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9                 GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFSEL9_0               (0x1UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000010 */
#define GPIO_AFRH_AFSEL9_1               (0x2UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000020 */
#define GPIO_AFRH_AFSEL9_2               (0x4UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000040 */
#define GPIO_AFRH_AFSEL9_3               (0x8UL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000080 */
#define GPIO_AFRH_AFSEL10_Pos            (8U)
#define GPIO_AFRH_AFSEL10_Msk            (0xFUL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10                GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFSEL10_0              (0x1UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000100 */
#define GPIO_AFRH_AFSEL10_1              (0x2UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000200 */
#define GPIO_AFRH_AFSEL10_2              (0x4UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000400 */
#define GPIO_AFRH_AFSEL10_3              (0x8UL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000800 */
#define GPIO_AFRH_AFSEL11_Pos            (12U)
#define GPIO_AFRH_AFSEL11_Msk            (0xFUL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11                GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFSEL11_0              (0x1UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00001000 */
#define GPIO_AFRH_AFSEL11_1              (0x2UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00002000 */
#define GPIO_AFRH_AFSEL11_2              (0x4UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00004000 */
#define GPIO_AFRH_AFSEL11_3              (0x8UL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00008000 */
#define GPIO_AFRH_AFSEL12_Pos            (16U)
#define GPIO_AFRH_AFSEL12_Msk            (0xFUL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12                GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFSEL12_0              (0x1UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00010000 */
#define GPIO_AFRH_AFSEL12_1              (0x2UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00020000 */
#define GPIO_AFRH_AFSEL12_2              (0x4UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00040000 */
#define GPIO_AFRH_AFSEL12_3              (0x8UL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00080000 */
#define GPIO_AFRH_AFSEL13_Pos            (20U)
#define GPIO_AFRH_AFSEL13_Msk            (0xFUL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13                GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFSEL13_0              (0x1UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00100000 */
#define GPIO_AFRH_AFSEL13_1              (0x2UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00200000 */
#define GPIO_AFRH_AFSEL13_2              (0x4UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00400000 */
#define GPIO_AFRH_AFSEL13_3              (0x8UL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00800000 */
#define GPIO_AFRH_AFSEL14_Pos            (24U)
#define GPIO_AFRH_AFSEL14_Msk            (0xFUL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14                GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFSEL14_0              (0x1UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x01000000 */
#define GPIO_AFRH_AFSEL14_1              (0x2UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x02000000 */
#define GPIO_AFRH_AFSEL14_2              (0x4UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x04000000 */
#define GPIO_AFRH_AFSEL14_3              (0x8UL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x08000000 */
#define GPIO_AFRH_AFSEL15_Pos            (28U)
#define GPIO_AFRH_AFSEL15_Msk            (0xFUL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15                GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFSEL15_0              (0x1UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x10000000 */
#define GPIO_AFRH_AFSEL15_1              (0x2UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x20000000 */
#define GPIO_AFRH_AFSEL15_2              (0x4UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x40000000 */
#define GPIO_AFRH_AFSEL15_3              (0x8UL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRH_AFRH0                  GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH0_0                GPIO_AFRH_AFSEL8_0
#define GPIO_AFRH_AFRH0_1                GPIO_AFRH_AFSEL8_1
#define GPIO_AFRH_AFRH0_2                GPIO_AFRH_AFSEL8_2
#define GPIO_AFRH_AFRH0_3                GPIO_AFRH_AFSEL8_3
#define GPIO_AFRH_AFRH1                  GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH1_0                GPIO_AFRH_AFSEL9_0
#define GPIO_AFRH_AFRH1_1                GPIO_AFRH_AFSEL9_1
#define GPIO_AFRH_AFRH1_2                GPIO_AFRH_AFSEL9_2
#define GPIO_AFRH_AFRH1_3                GPIO_AFRH_AFSEL9_3
#define GPIO_AFRH_AFRH2                  GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH2_0                GPIO_AFRH_AFSEL10_0
#define GPIO_AFRH_AFRH2_1                GPIO_AFRH_AFSEL10_1
#define GPIO_AFRH_AFRH2_2                GPIO_AFRH_AFSEL10_2
#define GPIO_AFRH_AFRH2_3                GPIO_AFRH_AFSEL10_3
#define GPIO_AFRH_AFRH3                  GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH3_0                GPIO_AFRH_AFSEL11_0
#define GPIO_AFRH_AFRH3_1                GPIO_AFRH_AFSEL11_1
#define GPIO_AFRH_AFRH3_2                GPIO_AFRH_AFSEL11_2
#define GPIO_AFRH_AFRH3_3                GPIO_AFRH_AFSEL11_3
#define GPIO_AFRH_AFRH4                  GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH4_0                GPIO_AFRH_AFSEL12_0
#define GPIO_AFRH_AFRH4_1                GPIO_AFRH_AFSEL12_1
#define GPIO_AFRH_AFRH4_2                GPIO_AFRH_AFSEL12_2
#define GPIO_AFRH_AFRH4_3                GPIO_AFRH_AFSEL12_3
#define GPIO_AFRH_AFRH5                  GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH5_0                GPIO_AFRH_AFSEL13_0
#define GPIO_AFRH_AFRH5_1                GPIO_AFRH_AFSEL13_1
#define GPIO_AFRH_AFRH5_2                GPIO_AFRH_AFSEL13_2
#define GPIO_AFRH_AFRH5_3                GPIO_AFRH_AFSEL13_3
#define GPIO_AFRH_AFRH6                  GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH6_0                GPIO_AFRH_AFSEL14_0
#define GPIO_AFRH_AFRH6_1                GPIO_AFRH_AFSEL14_1
#define GPIO_AFRH_AFRH6_2                GPIO_AFRH_AFSEL14_2
#define GPIO_AFRH_AFRH6_3                GPIO_AFRH_AFSEL14_3
#define GPIO_AFRH_AFRH7                  GPIO_AFRH_AFSEL15
#define GPIO_AFRH_AFRH7_0                GPIO_AFRH_AFSEL15_0
#define GPIO_AFRH_AFRH7_1                GPIO_AFRH_AFSEL15_1
#define GPIO_AFRH_AFRH7_2                GPIO_AFRH_AFSEL15_2
#define GPIO_AFRH_AFRH7_3                GPIO_AFRH_AFSEL15_3



/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_LPDS_Pos        (0U)
#define PWR_CR_LPDS_Msk        (0x1UL << PWR_CR_LPDS_Pos)                       /*!< 0x00000001 */
#define PWR_CR_LPDS            PWR_CR_LPDS_Msk                                 /*!< Low-Power Deepsleep                 */
#define PWR_CR_PDDS_Pos        (1U)
#define PWR_CR_PDDS_Msk        (0x1UL << PWR_CR_PDDS_Pos)                       /*!< 0x00000002 */
#define PWR_CR_PDDS            PWR_CR_PDDS_Msk                                 /*!< Power Down Deepsleep                */
#define PWR_CR_CWUF_Pos        (2U)
#define PWR_CR_CWUF_Msk        (0x1UL << PWR_CR_CWUF_Pos)                       /*!< 0x00000004 */
#define PWR_CR_CWUF            PWR_CR_CWUF_Msk                                 /*!< Clear Wakeup Flag                   */
#define PWR_CR_CSBF_Pos        (3U)
#define PWR_CR_CSBF_Msk        (0x1UL << PWR_CR_CSBF_Pos)                       /*!< 0x00000008 */
#define PWR_CR_CSBF            PWR_CR_CSBF_Msk                                 /*!< Clear Standby Flag                  */
#define PWR_CR_PVDE_Pos        (4U)
#define PWR_CR_PVDE_Msk        (0x1UL << PWR_CR_PVDE_Pos)                       /*!< 0x00000010 */
#define PWR_CR_PVDE            PWR_CR_PVDE_Msk                                 /*!< Power Voltage Detector Enable       */

#define PWR_CR_PLS_Pos         (5U)
#define PWR_CR_PLS_Msk         (0x7UL << PWR_CR_PLS_Pos)                        /*!< 0x000000E0 */
#define PWR_CR_PLS             PWR_CR_PLS_Msk                                  /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0           (0x1UL << PWR_CR_PLS_Pos)                        /*!< 0x00000020 */
#define PWR_CR_PLS_1           (0x2UL << PWR_CR_PLS_Pos)                        /*!< 0x00000040 */
#define PWR_CR_PLS_2           (0x4UL << PWR_CR_PLS_Pos)                        /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0        0x00000000U                                     /*!< PVD level 0 */
#define PWR_CR_PLS_LEV1        0x00000020U                                     /*!< PVD level 1 */
#define PWR_CR_PLS_LEV2        0x00000040U                                     /*!< PVD level 2 */
#define PWR_CR_PLS_LEV3        0x00000060U                                     /*!< PVD level 3 */
#define PWR_CR_PLS_LEV4        0x00000080U                                     /*!< PVD level 4 */
#define PWR_CR_PLS_LEV5        0x000000A0U                                     /*!< PVD level 5 */
#define PWR_CR_PLS_LEV6        0x000000C0U                                     /*!< PVD level 6 */
#define PWR_CR_PLS_LEV7        0x000000E0U                                     /*!< PVD level 7 */
#define PWR_CR_DBP_Pos         (8U)
#define PWR_CR_DBP_Msk         (0x1UL << PWR_CR_DBP_Pos)                        /*!< 0x00000100 */
#define PWR_CR_DBP             PWR_CR_DBP_Msk                                  /*!< Disable Backup Domain write protection                     */
#define PWR_CR_FPDS_Pos        (9U)
#define PWR_CR_FPDS_Msk        (0x1UL << PWR_CR_FPDS_Pos)                       /*!< 0x00000200 */
#define PWR_CR_FPDS            PWR_CR_FPDS_Msk                                 /*!< Flash power down in Stop mode                              */
#define PWR_CR_VOS_Pos         (14U)
#define PWR_CR_VOS_Msk         (0x1UL << PWR_CR_VOS_Pos)                        /*!< 0x00004000 */
#define PWR_CR_VOS             PWR_CR_VOS_Msk                                  /*!< VOS bit (Regulator voltage scaling output selection) */

/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_WUF_Pos        (0U)
#define PWR_CSR_WUF_Msk        (0x1UL << PWR_CSR_WUF_Pos)                       /*!< 0x00000001 */
#define PWR_CSR_WUF            PWR_CSR_WUF_Msk                                 /*!< Wakeup Flag                                      */
#define PWR_CSR_SBF_Pos        (1U)
#define PWR_CSR_SBF_Msk        (0x1UL << PWR_CSR_SBF_Pos)                       /*!< 0x00000002 */
#define PWR_CSR_SBF            PWR_CSR_SBF_Msk                                 /*!< Standby Flag                                     */
#define PWR_CSR_PVDO_Pos       (2U)
#define PWR_CSR_PVDO_Msk       (0x1UL << PWR_CSR_PVDO_Pos)                      /*!< 0x00000004 */
#define PWR_CSR_PVDO           PWR_CSR_PVDO_Msk                                /*!< PVD Output                                       */
#define PWR_CSR_BRR_Pos        (3U)
#define PWR_CSR_BRR_Msk        (0x1UL << PWR_CSR_BRR_Pos)                       /*!< 0x00000008 */
#define PWR_CSR_BRR            PWR_CSR_BRR_Msk                                 /*!< Backup regulator ready                           */
#define PWR_CSR_EWUP_Pos       (8U)
#define PWR_CSR_EWUP_Msk       (0x1UL << PWR_CSR_EWUP_Pos)                      /*!< 0x00000100 */
#define PWR_CSR_EWUP           PWR_CSR_EWUP_Msk                                /*!< Enable WKUP pin                                  */
#define PWR_CSR_BRE_Pos        (9U)
#define PWR_CSR_BRE_Msk        (0x1UL << PWR_CSR_BRE_Pos)                       /*!< 0x00000200 */
#define PWR_CSR_BRE            PWR_CSR_BRE_Msk                                 /*!< Backup regulator enable                          */
#define PWR_CSR_VOSRDY_Pos     (14U)
#define PWR_CSR_VOSRDY_Msk     (0x1UL << PWR_CSR_VOSRDY_Pos)                    /*!< 0x00004000 */
#define PWR_CSR_VOSRDY         PWR_CSR_VOSRDY_Msk                              /*!< Regulator voltage scaling output selection ready */



/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)
#define EXTI_IMR_MR0_Msk          (0x1UL << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0 */
#define EXTI_IMR_MR1_Pos          (1U)
#define EXTI_IMR_MR1_Msk          (0x1UL << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR1              EXTI_IMR_MR1_Msk                             /*!< Interrupt Mask on line 1 */
#define EXTI_IMR_MR2_Pos          (2U)
#define EXTI_IMR_MR2_Msk          (0x1UL << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR2              EXTI_IMR_MR2_Msk                             /*!< Interrupt Mask on line 2 */
#define EXTI_IMR_MR3_Pos          (3U)
#define EXTI_IMR_MR3_Msk          (0x1UL << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR3              EXTI_IMR_MR3_Msk                             /*!< Interrupt Mask on line 3 */
#define EXTI_IMR_MR4_Pos          (4U)
#define EXTI_IMR_MR4_Msk          (0x1UL << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR4              EXTI_IMR_MR4_Msk                             /*!< Interrupt Mask on line 4 */
#define EXTI_IMR_MR5_Pos          (5U)
#define EXTI_IMR_MR5_Msk          (0x1UL << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR5              EXTI_IMR_MR5_Msk                             /*!< Interrupt Mask on line 5 */
#define EXTI_IMR_MR6_Pos          (6U)
#define EXTI_IMR_MR6_Msk          (0x1UL << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR6              EXTI_IMR_MR6_Msk                             /*!< Interrupt Mask on line 6 */
#define EXTI_IMR_MR7_Pos          (7U)
#define EXTI_IMR_MR7_Msk          (0x1UL << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR7              EXTI_IMR_MR7_Msk                             /*!< Interrupt Mask on line 7 */
#define EXTI_IMR_MR8_Pos          (8U)
#define EXTI_IMR_MR8_Msk          (0x1UL << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR8              EXTI_IMR_MR8_Msk                             /*!< Interrupt Mask on line 8 */
#define EXTI_IMR_MR9_Pos          (9U)
#define EXTI_IMR_MR9_Msk          (0x1UL << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR9              EXTI_IMR_MR9_Msk                             /*!< Interrupt Mask on line 9 */
#define EXTI_IMR_MR10_Pos         (10U)
#define EXTI_IMR_MR10_Msk         (0x1UL << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR10             EXTI_IMR_MR10_Msk                            /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos         (11U)
#define EXTI_IMR_MR11_Msk         (0x1UL << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR11             EXTI_IMR_MR11_Msk                            /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos         (12U)
#define EXTI_IMR_MR12_Msk         (0x1UL << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR12             EXTI_IMR_MR12_Msk                            /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos         (13U)
#define EXTI_IMR_MR13_Msk         (0x1UL << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR13             EXTI_IMR_MR13_Msk                            /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos         (14U)
#define EXTI_IMR_MR14_Msk         (0x1UL << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR14             EXTI_IMR_MR14_Msk                            /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos         (15U)
#define EXTI_IMR_MR15_Msk         (0x1UL << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR15             EXTI_IMR_MR15_Msk                            /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos         (16U)
#define EXTI_IMR_MR16_Msk         (0x1UL << EXTI_IMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_IMR_MR16             EXTI_IMR_MR16_Msk                            /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos         (17U)
#define EXTI_IMR_MR17_Msk         (0x1UL << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR17             EXTI_IMR_MR17_Msk                            /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos         (18U)
#define EXTI_IMR_MR18_Msk         (0x1UL << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR18             EXTI_IMR_MR18_Msk                            /*!< Interrupt Mask on line 18 */
#define EXTI_IMR_MR19_Pos         (19U)
#define EXTI_IMR_MR19_Msk         (0x1UL << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR19             EXTI_IMR_MR19_Msk                            /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR20_Pos         (20U)
#define EXTI_IMR_MR20_Msk         (0x1UL << EXTI_IMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_IMR_MR20             EXTI_IMR_MR20_Msk                            /*!< Interrupt Mask on line 20 */
#define EXTI_IMR_MR21_Pos         (21U)
#define EXTI_IMR_MR21_Msk         (0x1UL << EXTI_IMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_IMR_MR21             EXTI_IMR_MR21_Msk                            /*!< Interrupt Mask on line 21 */
#define EXTI_IMR_MR22_Pos         (22U)
#define EXTI_IMR_MR22_Msk         (0x1UL << EXTI_IMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_IMR_MR22             EXTI_IMR_MR22_Msk                            /*!< Interrupt Mask on line 22 */

/* Reference Defines */
#define  EXTI_IMR_IM0                        EXTI_IMR_MR0
#define  EXTI_IMR_IM1                        EXTI_IMR_MR1
#define  EXTI_IMR_IM2                        EXTI_IMR_MR2
#define  EXTI_IMR_IM3                        EXTI_IMR_MR3
#define  EXTI_IMR_IM4                        EXTI_IMR_MR4
#define  EXTI_IMR_IM5                        EXTI_IMR_MR5
#define  EXTI_IMR_IM6                        EXTI_IMR_MR6
#define  EXTI_IMR_IM7                        EXTI_IMR_MR7
#define  EXTI_IMR_IM8                        EXTI_IMR_MR8
#define  EXTI_IMR_IM9                        EXTI_IMR_MR9
#define  EXTI_IMR_IM10                       EXTI_IMR_MR10
#define  EXTI_IMR_IM11                       EXTI_IMR_MR11
#define  EXTI_IMR_IM12                       EXTI_IMR_MR12
#define  EXTI_IMR_IM13                       EXTI_IMR_MR13
#define  EXTI_IMR_IM14                       EXTI_IMR_MR14
#define  EXTI_IMR_IM15                       EXTI_IMR_MR15
#define  EXTI_IMR_IM16                       EXTI_IMR_MR16
#define  EXTI_IMR_IM17                       EXTI_IMR_MR17
#define  EXTI_IMR_IM18                       EXTI_IMR_MR18
#define  EXTI_IMR_IM19                       EXTI_IMR_MR19
#define  EXTI_IMR_IM20                       EXTI_IMR_MR20
#define  EXTI_IMR_IM21                       EXTI_IMR_MR21
#define  EXTI_IMR_IM22                       EXTI_IMR_MR22
#define EXTI_IMR_IM_Pos           (0U)
#define EXTI_IMR_IM_Msk           (0x7FFFFFUL << EXTI_IMR_IM_Pos)               /*!< 0x007FFFFF */
#define EXTI_IMR_IM               EXTI_IMR_IM_Msk                              /*!< Interrupt Mask All */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMR_MR0_Pos          (0U)
#define EXTI_EMR_MR0_Msk          (0x1UL << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR0              EXTI_EMR_MR0_Msk                             /*!< Event Mask on line 0 */
#define EXTI_EMR_MR1_Pos          (1U)
#define EXTI_EMR_MR1_Msk          (0x1UL << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR1              EXTI_EMR_MR1_Msk                             /*!< Event Mask on line 1 */
#define EXTI_EMR_MR2_Pos          (2U)
#define EXTI_EMR_MR2_Msk          (0x1UL << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR2              EXTI_EMR_MR2_Msk                             /*!< Event Mask on line 2 */
#define EXTI_EMR_MR3_Pos          (3U)
#define EXTI_EMR_MR3_Msk          (0x1UL << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR3              EXTI_EMR_MR3_Msk                             /*!< Event Mask on line 3 */
#define EXTI_EMR_MR4_Pos          (4U)
#define EXTI_EMR_MR4_Msk          (0x1UL << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR4              EXTI_EMR_MR4_Msk                             /*!< Event Mask on line 4 */
#define EXTI_EMR_MR5_Pos          (5U)
#define EXTI_EMR_MR5_Msk          (0x1UL << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR5              EXTI_EMR_MR5_Msk                             /*!< Event Mask on line 5 */
#define EXTI_EMR_MR6_Pos          (6U)
#define EXTI_EMR_MR6_Msk          (0x1UL << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR6              EXTI_EMR_MR6_Msk                             /*!< Event Mask on line 6 */
#define EXTI_EMR_MR7_Pos          (7U)
#define EXTI_EMR_MR7_Msk          (0x1UL << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR7              EXTI_EMR_MR7_Msk                             /*!< Event Mask on line 7 */
#define EXTI_EMR_MR8_Pos          (8U)
#define EXTI_EMR_MR8_Msk          (0x1UL << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR8              EXTI_EMR_MR8_Msk                             /*!< Event Mask on line 8 */
#define EXTI_EMR_MR9_Pos          (9U)
#define EXTI_EMR_MR9_Msk          (0x1UL << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR9              EXTI_EMR_MR9_Msk                             /*!< Event Mask on line 9 */
#define EXTI_EMR_MR10_Pos         (10U)
#define EXTI_EMR_MR10_Msk         (0x1UL << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR10             EXTI_EMR_MR10_Msk                            /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos         (11U)
#define EXTI_EMR_MR11_Msk         (0x1UL << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR11             EXTI_EMR_MR11_Msk                            /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos         (12U)
#define EXTI_EMR_MR12_Msk         (0x1UL << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR12             EXTI_EMR_MR12_Msk                            /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos         (13U)
#define EXTI_EMR_MR13_Msk         (0x1UL << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR13             EXTI_EMR_MR13_Msk                            /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos         (14U)
#define EXTI_EMR_MR14_Msk         (0x1UL << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR14             EXTI_EMR_MR14_Msk                            /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos         (15U)
#define EXTI_EMR_MR15_Msk         (0x1UL << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR15             EXTI_EMR_MR15_Msk                            /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos         (16U)
#define EXTI_EMR_MR16_Msk         (0x1UL << EXTI_EMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_EMR_MR16             EXTI_EMR_MR16_Msk                            /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos         (17U)
#define EXTI_EMR_MR17_Msk         (0x1UL << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR17             EXTI_EMR_MR17_Msk                            /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos         (18U)
#define EXTI_EMR_MR18_Msk         (0x1UL << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR18             EXTI_EMR_MR18_Msk                            /*!< Event Mask on line 18 */
#define EXTI_EMR_MR19_Pos         (19U)
#define EXTI_EMR_MR19_Msk         (0x1UL << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR19             EXTI_EMR_MR19_Msk                            /*!< Event Mask on line 19 */
#define EXTI_EMR_MR20_Pos         (20U)
#define EXTI_EMR_MR20_Msk         (0x1UL << EXTI_EMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_EMR_MR20             EXTI_EMR_MR20_Msk                            /*!< Event Mask on line 20 */
#define EXTI_EMR_MR21_Pos         (21U)
#define EXTI_EMR_MR21_Msk         (0x1UL << EXTI_EMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_EMR_MR21             EXTI_EMR_MR21_Msk                            /*!< Event Mask on line 21 */
#define EXTI_EMR_MR22_Pos         (22U)
#define EXTI_EMR_MR22_Msk         (0x1UL << EXTI_EMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_EMR_MR22             EXTI_EMR_MR22_Msk                            /*!< Event Mask on line 22 */

/* Reference Defines */
#define  EXTI_EMR_EM0                        EXTI_EMR_MR0
#define  EXTI_EMR_EM1                        EXTI_EMR_MR1
#define  EXTI_EMR_EM2                        EXTI_EMR_MR2
#define  EXTI_EMR_EM3                        EXTI_EMR_MR3
#define  EXTI_EMR_EM4                        EXTI_EMR_MR4
#define  EXTI_EMR_EM5                        EXTI_EMR_MR5
#define  EXTI_EMR_EM6                        EXTI_EMR_MR6
#define  EXTI_EMR_EM7                        EXTI_EMR_MR7
#define  EXTI_EMR_EM8                        EXTI_EMR_MR8
#define  EXTI_EMR_EM9                        EXTI_EMR_MR9
#define  EXTI_EMR_EM10                       EXTI_EMR_MR10
#define  EXTI_EMR_EM11                       EXTI_EMR_MR11
#define  EXTI_EMR_EM12                       EXTI_EMR_MR12
#define  EXTI_EMR_EM13                       EXTI_EMR_MR13
#define  EXTI_EMR_EM14                       EXTI_EMR_MR14
#define  EXTI_EMR_EM15                       EXTI_EMR_MR15
#define  EXTI_EMR_EM16                       EXTI_EMR_MR16
#define  EXTI_EMR_EM17                       EXTI_EMR_MR17
#define  EXTI_EMR_EM18                       EXTI_EMR_MR18
#define  EXTI_EMR_EM19                       EXTI_EMR_MR19
#define  EXTI_EMR_EM20                       EXTI_EMR_MR20
#define  EXTI_EMR_EM21                       EXTI_EMR_MR21
#define  EXTI_EMR_EM22                       EXTI_EMR_MR22

/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos         (0U)
#define EXTI_RTSR_TR0_Msk         (0x1UL << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos         (1U)
#define EXTI_RTSR_TR1_Msk         (0x1UL << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR1             EXTI_RTSR_TR1_Msk                            /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos         (2U)
#define EXTI_RTSR_TR2_Msk         (0x1UL << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR2             EXTI_RTSR_TR2_Msk                            /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos         (3U)
#define EXTI_RTSR_TR3_Msk         (0x1UL << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR3             EXTI_RTSR_TR3_Msk                            /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos         (4U)
#define EXTI_RTSR_TR4_Msk         (0x1UL << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR4             EXTI_RTSR_TR4_Msk                            /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos         (5U)
#define EXTI_RTSR_TR5_Msk         (0x1UL << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR5             EXTI_RTSR_TR5_Msk                            /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos         (6U)
#define EXTI_RTSR_TR6_Msk         (0x1UL << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR6             EXTI_RTSR_TR6_Msk                            /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos         (7U)
#define EXTI_RTSR_TR7_Msk         (0x1UL << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR7             EXTI_RTSR_TR7_Msk                            /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos         (8U)
#define EXTI_RTSR_TR8_Msk         (0x1UL << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR8             EXTI_RTSR_TR8_Msk                            /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos         (9U)
#define EXTI_RTSR_TR9_Msk         (0x1UL << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR9             EXTI_RTSR_TR9_Msk                            /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos        (10U)
#define EXTI_RTSR_TR10_Msk        (0x1UL << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR10            EXTI_RTSR_TR10_Msk                           /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos        (11U)
#define EXTI_RTSR_TR11_Msk        (0x1UL << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR11            EXTI_RTSR_TR11_Msk                           /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos        (12U)
#define EXTI_RTSR_TR12_Msk        (0x1UL << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR12            EXTI_RTSR_TR12_Msk                           /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos        (13U)
#define EXTI_RTSR_TR13_Msk        (0x1UL << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR13            EXTI_RTSR_TR13_Msk                           /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos        (14U)
#define EXTI_RTSR_TR14_Msk        (0x1UL << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR14            EXTI_RTSR_TR14_Msk                           /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos        (15U)
#define EXTI_RTSR_TR15_Msk        (0x1UL << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR15            EXTI_RTSR_TR15_Msk                           /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos        (16U)
#define EXTI_RTSR_TR16_Msk        (0x1UL << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR16            EXTI_RTSR_TR16_Msk                           /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos        (17U)
#define EXTI_RTSR_TR17_Msk        (0x1UL << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR17            EXTI_RTSR_TR17_Msk                           /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR18_Pos        (18U)
#define EXTI_RTSR_TR18_Msk        (0x1UL << EXTI_RTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_RTSR_TR18            EXTI_RTSR_TR18_Msk                           /*!< Rising trigger event configuration bit of line 18 */
#define EXTI_RTSR_TR19_Pos        (19U)
#define EXTI_RTSR_TR19_Msk        (0x1UL << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR19            EXTI_RTSR_TR19_Msk                           /*!< Rising trigger event configuration bit of line 19 */
#define EXTI_RTSR_TR20_Pos        (20U)
#define EXTI_RTSR_TR20_Msk        (0x1UL << EXTI_RTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_RTSR_TR20            EXTI_RTSR_TR20_Msk                           /*!< Rising trigger event configuration bit of line 20 */
#define EXTI_RTSR_TR21_Pos        (21U)
#define EXTI_RTSR_TR21_Msk        (0x1UL << EXTI_RTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_RTSR_TR21            EXTI_RTSR_TR21_Msk                           /*!< Rising trigger event configuration bit of line 21 */
#define EXTI_RTSR_TR22_Pos        (22U)
#define EXTI_RTSR_TR22_Msk        (0x1UL << EXTI_RTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_RTSR_TR22            EXTI_RTSR_TR22_Msk                           /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos         (0U)
#define EXTI_FTSR_TR0_Msk         (0x1UL << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos         (1U)
#define EXTI_FTSR_TR1_Msk         (0x1UL << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR1             EXTI_FTSR_TR1_Msk                            /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos         (2U)
#define EXTI_FTSR_TR2_Msk         (0x1UL << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR2             EXTI_FTSR_TR2_Msk                            /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos         (3U)
#define EXTI_FTSR_TR3_Msk         (0x1UL << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR3             EXTI_FTSR_TR3_Msk                            /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos         (4U)
#define EXTI_FTSR_TR4_Msk         (0x1UL << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR4             EXTI_FTSR_TR4_Msk                            /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos         (5U)
#define EXTI_FTSR_TR5_Msk         (0x1UL << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR5             EXTI_FTSR_TR5_Msk                            /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos         (6U)
#define EXTI_FTSR_TR6_Msk         (0x1UL << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR6             EXTI_FTSR_TR6_Msk                            /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos         (7U)
#define EXTI_FTSR_TR7_Msk         (0x1UL << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR7             EXTI_FTSR_TR7_Msk                            /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos         (8U)
#define EXTI_FTSR_TR8_Msk         (0x1UL << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR8             EXTI_FTSR_TR8_Msk                            /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos         (9U)
#define EXTI_FTSR_TR9_Msk         (0x1UL << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR9             EXTI_FTSR_TR9_Msk                            /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos        (10U)
#define EXTI_FTSR_TR10_Msk        (0x1UL << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR10            EXTI_FTSR_TR10_Msk                           /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos        (11U)
#define EXTI_FTSR_TR11_Msk        (0x1UL << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR11            EXTI_FTSR_TR11_Msk                           /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos        (12U)
#define EXTI_FTSR_TR12_Msk        (0x1UL << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR12            EXTI_FTSR_TR12_Msk                           /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos        (13U)
#define EXTI_FTSR_TR13_Msk        (0x1UL << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR13            EXTI_FTSR_TR13_Msk                           /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos        (14U)
#define EXTI_FTSR_TR14_Msk        (0x1UL << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR14            EXTI_FTSR_TR14_Msk                           /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos        (15U)
#define EXTI_FTSR_TR15_Msk        (0x1UL << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR15            EXTI_FTSR_TR15_Msk                           /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos        (16U)
#define EXTI_FTSR_TR16_Msk        (0x1UL << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR16            EXTI_FTSR_TR16_Msk                           /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos        (17U)
#define EXTI_FTSR_TR17_Msk        (0x1UL << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR17            EXTI_FTSR_TR17_Msk                           /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR18_Pos        (18U)
#define EXTI_FTSR_TR18_Msk        (0x1UL << EXTI_FTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_FTSR_TR18            EXTI_FTSR_TR18_Msk                           /*!< Falling trigger event configuration bit of line 18 */
#define EXTI_FTSR_TR19_Pos        (19U)
#define EXTI_FTSR_TR19_Msk        (0x1UL << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR19            EXTI_FTSR_TR19_Msk                           /*!< Falling trigger event configuration bit of line 19 */
#define EXTI_FTSR_TR20_Pos        (20U)
#define EXTI_FTSR_TR20_Msk        (0x1UL << EXTI_FTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_FTSR_TR20            EXTI_FTSR_TR20_Msk                           /*!< Falling trigger event configuration bit of line 20 */
#define EXTI_FTSR_TR21_Pos        (21U)
#define EXTI_FTSR_TR21_Msk        (0x1UL << EXTI_FTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_FTSR_TR21            EXTI_FTSR_TR21_Msk                           /*!< Falling trigger event configuration bit of line 21 */
#define EXTI_FTSR_TR22_Pos        (22U)
#define EXTI_FTSR_TR22_Msk        (0x1UL << EXTI_FTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_FTSR_TR22            EXTI_FTSR_TR22_Msk                           /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)
#define EXTI_SWIER_SWIER0_Msk     (0x1UL << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0 */
#define EXTI_SWIER_SWIER1_Pos     (1U)
#define EXTI_SWIER_SWIER1_Msk     (0x1UL << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1         EXTI_SWIER_SWIER1_Msk                        /*!< Software Interrupt on line 1 */
#define EXTI_SWIER_SWIER2_Pos     (2U)
#define EXTI_SWIER_SWIER2_Msk     (0x1UL << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2         EXTI_SWIER_SWIER2_Msk                        /*!< Software Interrupt on line 2 */
#define EXTI_SWIER_SWIER3_Pos     (3U)
#define EXTI_SWIER_SWIER3_Msk     (0x1UL << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3         EXTI_SWIER_SWIER3_Msk                        /*!< Software Interrupt on line 3 */
#define EXTI_SWIER_SWIER4_Pos     (4U)
#define EXTI_SWIER_SWIER4_Msk     (0x1UL << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4         EXTI_SWIER_SWIER4_Msk                        /*!< Software Interrupt on line 4 */
#define EXTI_SWIER_SWIER5_Pos     (5U)
#define EXTI_SWIER_SWIER5_Msk     (0x1UL << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5         EXTI_SWIER_SWIER5_Msk                        /*!< Software Interrupt on line 5 */
#define EXTI_SWIER_SWIER6_Pos     (6U)
#define EXTI_SWIER_SWIER6_Msk     (0x1UL << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6         EXTI_SWIER_SWIER6_Msk                        /*!< Software Interrupt on line 6 */
#define EXTI_SWIER_SWIER7_Pos     (7U)
#define EXTI_SWIER_SWIER7_Msk     (0x1UL << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7         EXTI_SWIER_SWIER7_Msk                        /*!< Software Interrupt on line 7 */
#define EXTI_SWIER_SWIER8_Pos     (8U)
#define EXTI_SWIER_SWIER8_Msk     (0x1UL << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8         EXTI_SWIER_SWIER8_Msk                        /*!< Software Interrupt on line 8 */
#define EXTI_SWIER_SWIER9_Pos     (9U)
#define EXTI_SWIER_SWIER9_Msk     (0x1UL << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9         EXTI_SWIER_SWIER9_Msk                        /*!< Software Interrupt on line 9 */
#define EXTI_SWIER_SWIER10_Pos    (10U)
#define EXTI_SWIER_SWIER10_Msk    (0x1UL << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10        EXTI_SWIER_SWIER10_Msk                       /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos    (11U)
#define EXTI_SWIER_SWIER11_Msk    (0x1UL << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11        EXTI_SWIER_SWIER11_Msk                       /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos    (12U)
#define EXTI_SWIER_SWIER12_Msk    (0x1UL << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12        EXTI_SWIER_SWIER12_Msk                       /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos    (13U)
#define EXTI_SWIER_SWIER13_Msk    (0x1UL << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13        EXTI_SWIER_SWIER13_Msk                       /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos    (14U)
#define EXTI_SWIER_SWIER14_Msk    (0x1UL << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14        EXTI_SWIER_SWIER14_Msk                       /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos    (15U)
#define EXTI_SWIER_SWIER15_Msk    (0x1UL << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15        EXTI_SWIER_SWIER15_Msk                       /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos    (16U)
#define EXTI_SWIER_SWIER16_Msk    (0x1UL << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16        EXTI_SWIER_SWIER16_Msk                       /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos    (17U)
#define EXTI_SWIER_SWIER17_Msk    (0x1UL << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17        EXTI_SWIER_SWIER17_Msk                       /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER18_Pos    (18U)
#define EXTI_SWIER_SWIER18_Msk    (0x1UL << EXTI_SWIER_SWIER18_Pos)             /*!< 0x00040000 */
#define EXTI_SWIER_SWIER18        EXTI_SWIER_SWIER18_Msk                       /*!< Software Interrupt on line 18 */
#define EXTI_SWIER_SWIER19_Pos    (19U)
#define EXTI_SWIER_SWIER19_Msk    (0x1UL << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19        EXTI_SWIER_SWIER19_Msk                       /*!< Software Interrupt on line 19 */
#define EXTI_SWIER_SWIER20_Pos    (20U)
#define EXTI_SWIER_SWIER20_Msk    (0x1UL << EXTI_SWIER_SWIER20_Pos)             /*!< 0x00100000 */
#define EXTI_SWIER_SWIER20        EXTI_SWIER_SWIER20_Msk                       /*!< Software Interrupt on line 20 */
#define EXTI_SWIER_SWIER21_Pos    (21U)
#define EXTI_SWIER_SWIER21_Msk    (0x1UL << EXTI_SWIER_SWIER21_Pos)             /*!< 0x00200000 */
#define EXTI_SWIER_SWIER21        EXTI_SWIER_SWIER21_Msk                       /*!< Software Interrupt on line 21 */
#define EXTI_SWIER_SWIER22_Pos    (22U)
#define EXTI_SWIER_SWIER22_Msk    (0x1UL << EXTI_SWIER_SWIER22_Pos)             /*!< 0x00400000 */
#define EXTI_SWIER_SWIER22        EXTI_SWIER_SWIER22_Msk                       /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos           (0U)
#define EXTI_PR_PR0_Msk           (0x1UL << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit for line 0 */
#define EXTI_PR_PR1_Pos           (1U)
#define EXTI_PR_PR1_Msk           (0x1UL << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR1               EXTI_PR_PR1_Msk                              /*!< Pending bit for line 1 */
#define EXTI_PR_PR2_Pos           (2U)
#define EXTI_PR_PR2_Msk           (0x1UL << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR2               EXTI_PR_PR2_Msk                              /*!< Pending bit for line 2 */
#define EXTI_PR_PR3_Pos           (3U)
#define EXTI_PR_PR3_Msk           (0x1UL << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR3               EXTI_PR_PR3_Msk                              /*!< Pending bit for line 3 */
#define EXTI_PR_PR4_Pos           (4U)
#define EXTI_PR_PR4_Msk           (0x1UL << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR4               EXTI_PR_PR4_Msk                              /*!< Pending bit for line 4 */
#define EXTI_PR_PR5_Pos           (5U)
#define EXTI_PR_PR5_Msk           (0x1UL << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR5               EXTI_PR_PR5_Msk                              /*!< Pending bit for line 5 */
#define EXTI_PR_PR6_Pos           (6U)
#define EXTI_PR_PR6_Msk           (0x1UL << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR6               EXTI_PR_PR6_Msk                              /*!< Pending bit for line 6 */
#define EXTI_PR_PR7_Pos           (7U)
#define EXTI_PR_PR7_Msk           (0x1UL << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR7               EXTI_PR_PR7_Msk                              /*!< Pending bit for line 7 */
#define EXTI_PR_PR8_Pos           (8U)
#define EXTI_PR_PR8_Msk           (0x1UL << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR8               EXTI_PR_PR8_Msk                              /*!< Pending bit for line 8 */
#define EXTI_PR_PR9_Pos           (9U)
#define EXTI_PR_PR9_Msk           (0x1UL << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR9               EXTI_PR_PR9_Msk                              /*!< Pending bit for line 9 */
#define EXTI_PR_PR10_Pos          (10U)
#define EXTI_PR_PR10_Msk          (0x1UL << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR10              EXTI_PR_PR10_Msk                             /*!< Pending bit for line 10 */
#define EXTI_PR_PR11_Pos          (11U)
#define EXTI_PR_PR11_Msk          (0x1UL << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR11              EXTI_PR_PR11_Msk                             /*!< Pending bit for line 11 */
#define EXTI_PR_PR12_Pos          (12U)
#define EXTI_PR_PR12_Msk          (0x1UL << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR12              EXTI_PR_PR12_Msk                             /*!< Pending bit for line 12 */
#define EXTI_PR_PR13_Pos          (13U)
#define EXTI_PR_PR13_Msk          (0x1UL << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR13              EXTI_PR_PR13_Msk                             /*!< Pending bit for line 13 */
#define EXTI_PR_PR14_Pos          (14U)
#define EXTI_PR_PR14_Msk          (0x1UL << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR14              EXTI_PR_PR14_Msk                             /*!< Pending bit for line 14 */
#define EXTI_PR_PR15_Pos          (15U)
#define EXTI_PR_PR15_Msk          (0x1UL << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR15              EXTI_PR_PR15_Msk                             /*!< Pending bit for line 15 */
#define EXTI_PR_PR16_Pos          (16U)
#define EXTI_PR_PR16_Msk          (0x1UL << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR16              EXTI_PR_PR16_Msk                             /*!< Pending bit for line 16 */
#define EXTI_PR_PR17_Pos          (17U)
#define EXTI_PR_PR17_Msk          (0x1UL << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR17              EXTI_PR_PR17_Msk                             /*!< Pending bit for line 17 */
#define EXTI_PR_PR18_Pos          (18U)
#define EXTI_PR_PR18_Msk          (0x1UL << EXTI_PR_PR18_Pos)                   /*!< 0x00040000 */
#define EXTI_PR_PR18              EXTI_PR_PR18_Msk                             /*!< Pending bit for line 18 */
#define EXTI_PR_PR19_Pos          (19U)
#define EXTI_PR_PR19_Msk          (0x1UL << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR19              EXTI_PR_PR19_Msk                             /*!< Pending bit for line 19 */
#define EXTI_PR_PR20_Pos          (20U)
#define EXTI_PR_PR20_Msk          (0x1UL << EXTI_PR_PR20_Pos)                   /*!< 0x00100000 */
#define EXTI_PR_PR20              EXTI_PR_PR20_Msk                             /*!< Pending bit for line 20 */
#define EXTI_PR_PR21_Pos          (21U)
#define EXTI_PR_PR21_Msk          (0x1UL << EXTI_PR_PR21_Pos)                   /*!< 0x00200000 */
#define EXTI_PR_PR21              EXTI_PR_PR21_Msk                             /*!< Pending bit for line 21 */
#define EXTI_PR_PR22_Pos          (22U)
#define EXTI_PR_PR22_Msk          (0x1UL << EXTI_PR_PR22_Pos)                   /*!< 0x00400000 */
#define EXTI_PR_PR22              EXTI_PR_PR22_Msk                             /*!< Pending bit for line 22 */

#define RCC_MCO2SOURCE_PLLI2SCLK	(0x1UL << 30U)

typedef struct
{
  volatile uint32_t DCFG;            /*!< dev Configuration Register   800h */
  volatile uint32_t DCTL;            /*!< dev Control Register         804h */
  volatile uint32_t DSTS;            /*!< dev Status Register (RO)     808h */
  uint32_t Reserved0C;           /*!< Reserved                     80Ch */
  volatile uint32_t DIEPMSK;         /*!< dev IN Endpoint Mask         810h */
  volatile uint32_t DOEPMSK;         /*!< dev OUT Endpoint Mask        814h */
  volatile uint32_t DAINT;           /*!< dev All Endpoints Itr Reg    818h */
  volatile uint32_t DAINTMSK;        /*!< dev All Endpoints Itr Mask   81Ch */
  uint32_t  Reserved20;          /*!< Reserved                     820h */
  uint32_t Reserved9;            /*!< Reserved                     824h */
  volatile uint32_t DVBUSDIS;        /*!< dev VBUS discharge Register  828h */
  volatile uint32_t DVBUSPULSE;      /*!< dev VBUS Pulse Register      82Ch */
  volatile uint32_t DTHRCTL;         /*!< dev threshold                830h */
  volatile uint32_t DIEPEMPMSK;      /*!< dev empty msk                834h */
  volatile uint32_t DEACHINT;        /*!< dedicated EP interrupt       838h */
  volatile uint32_t DEACHMSK;        /*!< dedicated EP msk             83Ch */
  uint32_t Reserved40;           /*!< dedicated EP mask            840h */
  volatile uint32_t DINEP1MSK;       /*!< dedicated EP mask            844h */
  uint32_t  Reserved44[15];      /*!< Reserved                 844-87Ch */
  volatile uint32_t DOUTEP1MSK;      /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;


typedef struct
{
  volatile uint32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  volatile uint32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  volatile uint32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  volatile uint32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  volatile uint32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;


typedef struct
{
  volatile uint32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  volatile uint32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  volatile uint32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  volatile uint32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;


/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/** 
  * @brief  HAL Lock structures definition  
  */
typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

/* Exported macro ------------------------------------------------------------*/

#if !defined(UNUSED)
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#endif /* UNUSED */

#define HAL_MAX_DELAY      0xFFFFFFFFU

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__)               \
                        do{                                                      \
                              (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
                              (__DMA_HANDLE__).Parent = (__HANDLE__);             \
                          } while(0U)

/** @brief Reset the Handle's State field.
  * @param __HANDLE__ specifies the Peripheral Handle.
  * @note  This macro can be used for the following purpose: 
  *          - When the Handle is declared as local variable; before passing it as parameter
  *            to HAL_PPP_Init() for the first time, it is mandatory to use this macro 
  *            to set to 0 the Handle's "State" field.
  *            Otherwise, "State" field may have any random value and the first time the function 
  *            HAL_PPP_Init() is called, the low level hardware initialization will be missed
  *            (i.e. HAL_PPP_MspInit() will not be executed).
  *          - When there is a need to reconfigure the low level hardware: instead of calling
  *            HAL_PPP_DeInit() then HAL_PPP_Init(), user can make a call to this macro then HAL_PPP_Init().
  *            In this later function, when the Handle's "State" field is set to 0, it will execute the function
  *            HAL_PPP_MspInit() which will reconfigure the low level hardware.
  * @retval None
  */
#define __HAL_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = 0U)

#if (USE_RTOS == 1U)
  /* Reserved for future use */
  #error "USE_RTOS should be 0 in the current HAL release"
#else
  #define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0U)

  #define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0U)
#endif /* USE_RTOS */

#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* ARM Compiler V6 */
  #ifndef __weak
    #define __weak  __attribute__((weak))
  #endif
  #ifndef __packed
    #define __packed  __attribute__((packed))
  #endif
#elif defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
  #ifndef __packed
    #define __packed __attribute__((__packed__))
  #endif /* __packed */
#endif /* __GNUC__ */


/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* ARM Compiler V6 */
  #ifndef __ALIGN_BEGIN
    #define __ALIGN_BEGIN
  #endif
  #ifndef __ALIGN_END
    #define __ALIGN_END      __attribute__ ((aligned (4)))
  #endif
#elif defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
  #ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
  #endif /* __ALIGN_END */
  #ifndef __ALIGN_BEGIN  
    #define __ALIGN_BEGIN
  #endif /* __ALIGN_BEGIN */
#else
  #ifndef __ALIGN_END
    #define __ALIGN_END
  #endif /* __ALIGN_END */
  #ifndef __ALIGN_BEGIN      
    #if defined   (__CC_ARM)      /* ARM Compiler V5*/
#define __ALIGN_BEGIN    __align(4)
    #elif defined (__ICCARM__)    /* IAR Compiler */
      #define __ALIGN_BEGIN 
    #endif /* __CC_ARM */
  #endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */


/** 
  * @brief  __RAM_FUNC definition
  */ 
#if defined ( __CC_ARM   ) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
/* ARM Compiler V4/V5 and V6
   --------------------------
   RAM functions are defined using the toolchain options. 
   Functions that are executed in RAM should reside in a separate source module.
   Using the 'Options for File' dialog you can simply change the 'Code / Const' 
   area of a module to a memory space in physical RAM.
   Available memory areas are declared in the 'Target' tab of the 'Options for Target'
   dialog. 
*/
#define __RAM_FUNC

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
   RAM functions are defined using a specific toolchain keyword "__ramfunc". 
*/
#define __RAM_FUNC __ramfunc

#elif defined   (  __GNUC__  )
/* GNU Compiler
   ------------
  RAM functions are defined using a specific toolchain attribute 
   "__attribute__((section(".RamFunc")))".
*/
#define __RAM_FUNC __attribute__((section(".RamFunc")))

#endif

/** 
  * @brief  __NOINLINE definition
  */ 
#if defined ( __CC_ARM   ) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) || defined   (  __GNUC__  )
/* ARM V4/V5 and V6 & GNU Compiler
   -------------------------------
*/
#define __NOINLINE __attribute__ ( (noinline) )

#elif defined ( __ICCARM__ )
/* ICCARM Compiler
   ---------------
*/
#define __NOINLINE _Pragma("optimize = no_inline")

#endif

#ifdef __cplusplus
}
#endif

#endif /* ___STM32F4xx_HAL_DEF */


