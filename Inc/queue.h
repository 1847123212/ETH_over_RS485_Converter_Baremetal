// ****************************************************************************
/// \file      queue.h
///
/// \brief     queue Module
///
/// \details   Module which manages the list for transmitting 
///            data between communication interfaces. 
///
/// \author    Nico Korn
///
/// \version   0.0.1.0
///
/// \date      18062021
/// 
/// \copyright Copyright 20210 Reichle & De-Massari AG
///            
///            Permission is hereby granted, free of charge, to any person 
///            obtaining a copy of this software and associated documentation 
///            files (the "Software"), to deal in the Software without 
///            restriction, including without limitation the rights to use, 
///            copy, modify, merge, publish, distribute, sublicense, and/or sell
///            copies of the Software, and to permit persons to whom the 
///            Software is furnished to do so, subject to the following 
///            conditions:
///            
///            The above copyright notice and this permission notice shall be 
///            included in all copies or substantial portions of the Software.
///            
///            THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
///            EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
///            OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
///            NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
///            HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
///            WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
///            FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
///            OTHER DEALINGS IN THE SOFTWARE.
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
#ifndef __QUEUE_H
#define __QUEUE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

// Exported defines ***********************************************************
#define BUFFERLENGTH                       ( 1558u )
#define QUEUELENGTH                        ( 40u )

// Exported types *************************************************************
typedef enum
{
   NOT_INITIALIZED,
   UART_TO_ETH,
   ETH_TO_UART             
} message_direction_t;

typedef enum
{
   EMPTY,
   TAIL_UNBLOCKED,
   TAIL_BLOCKED
} queue_status_t;

typedef enum
{
   EMPTY_TX = 0,
   READY_FOR_TX,
   PROCESSING_TX
} message_status_t;

typedef struct queue_obj{
    uint8_t             data[BUFFERLENGTH];
    uint8_t*            dataStart;
    uint16_t            dataLength;
    message_status_t    messageStatus;
} queue_obj_t;

typedef struct queue 
{
   queue_status_t       queueStatus;
   uint32_t             dataPacketsIN; 
   uint32_t             bytesIN;
   uint32_t             dataPacketsOUT;
   uint32_t             bytesOUT;
   uint32_t             frameCounter;
   uint32_t             queueFull;
   uint32_t             queueLength;
   uint32_t             queueLengthPeak;
   message_direction_t  messageDirection;   
   queue_obj_t          queue[QUEUELENGTH];
   uint32_t             headIndex;
   uint32_t             tailIndex;
   uint32_t             tailError;
   uint32_t             spuriousError;
   uint8_t              (*output)(uint8_t*, uint16_t);
} queue_handle_t;

// Exported functions *********************************************************
void     queue_init              ( queue_handle_t *queueHandle );
void     queue_manager           ( queue_handle_t *queueHandle );
void     queue_dequeue           ( queue_handle_t *queueHandle );
uint8_t* queue_queue             ( uint8_t* dataStart, uint16_t dataLength, queue_handle_t *queueHandle );
uint8_t* queue_getHeadBuffer     ( queue_handle_t *queueHandle );
uint8_t* queue_getTailBuffer     ( queue_handle_t *queueHandle );

#endif /* __QUEUE_H */
