// ****************************************************************************
/// \file      main.h
///
/// \brief     Firmware for the ethernet to rs485 bridge.
///
/// \details   Bi-directional ethernet to rs485 bridge. Firmware uses a
///            ringbuffer with defined size. Use of interrupts and dma with a
///            configured cpu clock of 400 MHz shall speed up this application
///            so it can hold on with the ethernet and rs485 speed.
///
/// \author    Nico Korn
///
/// \version   0.1
///
/// \date      20190430
/// 
/// \copyright Copyright (C) 2019  by "Reichle & De-Massari AG", 
///            all rights reserved.
///
/// \pre       
///
/// \bug       
///
/// \warning   
///
/// \todo      
///
// ****************************************************************************

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
// UART BUS
typedef enum
{
   RECEIVE = 0U,
   TRANSMIT             
} uart_cmd_t;

typedef enum
{
   NOT_INITIALIZED,
   UART_TO_ETH,
   ETH_TO_UART             
} message_direction_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Exported defines ----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
