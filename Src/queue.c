// ****************************************************************************
/// \file      queue.c
///
/// \brief     queue Module
///
/// \details   Module which manages the queue for transmitting 
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


// Include ********************************************************************
#include "queue.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Private define *************************************************************

// Private types     **********************************************************

// Private variables **********************************************************

// Private functions **********************************************************

// ----------------------------------------------------------------------------
/// \brief     Queue init.
///
/// \param     none
///
/// \return    none
void queue_init( queue_handle_t *queueHandle )
{   
   // init statistics to 0
   queueHandle->dataPacketsIN          = 0;
   queueHandle->bytesIN                = 0;
   queueHandle->dataPacketsOUT         = 0;
   queueHandle->bytesOUT               = 0;
   queueHandle->queueFull              = 0;
   queueHandle->queueLengthPeak        = 0;
   queueHandle->queueLength            = 0;
   queueHandle->headIndex              = QUEUELENGTH;
   queueHandle->tailIndex              = QUEUELENGTH;
   
   // message status of the message that's need to be send
   queueHandle->queueStatus            = TAIL_UNBLOCKED;
}

// ----------------------------------------------------------------------------
/// \brief     The queue manager checks for available data to send, and calls
///            the out interface.
///
/// \param     none
///
/// \return    none
inline void queue_manager( queue_handle_t *queueHandle )
{      
   if( queueHandle->queueStatus != TAIL_UNBLOCKED )
   {
      return;
   }
   
   if( queueHandle->tailIndex < queueHandle->headIndex 
      && queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].messageStatus == READY_FOR_TX )
   {
      // set queue and message status need to be set at this point to avoid transmission complete
      // race conditions
      queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].messageStatus = PROCESSING_TX;
      queueHandle->queueStatus = TAIL_BLOCKED;
      
      // setup the next frame pointed by the tx pointer and send it hrough the interface
      if( queueHandle->output( queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].dataStart, queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].dataLength ) != 1 )
      {
         // peripheral occupied, set back states
         queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].messageStatus = READY_FOR_TX;
         queueHandle->queueStatus = TAIL_UNBLOCKED;
      }
   }
}

// ----------------------------------------------------------------------------
/// \brief     The queue manager checks for available data to send, and calls
///            the out interface.
///
/// \param     none
///
/// \return    none
inline void queue_dequeue( queue_handle_t *queueHandle )
{   
   // transmission complete unblock the tail
   queueHandle->queueStatus = TAIL_UNBLOCKED;
   
   if( queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].messageStatus != PROCESSING_TX )
   {
      // spurious
      queueHandle->spuriousError++;
      return;
   }
   
   if( queueHandle->tailIndex < queueHandle->headIndex )
   {
      // decrement queueHandle length
      queueHandle->dataPacketsOUT++;
      queueHandle->queueLength--;
      queueHandle->bytesOUT += queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].dataLength; // note: this are the frame bytes without preamble and crc value
      
      // set message status
      //queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].data[0] = 0x00;
      queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].messageStatus = EMPTY_TX;
      
      // set tail number
      queueHandle->tailIndex++;
   }
   else
   {
      queueHandle->tailError++;
   }
}

// ----------------------------------------------------------------------------
/// \brief     Set the queue object as ready to process and return a pointer 
///            to an empty queue object.
///
/// \param     [in]  queue_handle_t *queueHandle
///
/// \return    none
inline uint8_t* queue_enqueue( uint8_t* dataStart, uint16_t dataLength, queue_handle_t *queueHandle )
{
   // integer type wrap arround check and set for head and tail index
   if( queueHandle->headIndex < QUEUELENGTH )
   {
      queueHandle->headIndex += QUEUELENGTH;
      queueHandle->tailIndex += QUEUELENGTH;
   }
   
   // fifo not full?
   if( (queueHandle->headIndex - queueHandle->tailIndex) < QUEUELENGTH )
   {
      // the flag is valid, so this slot on the fifo is ready to be computed
      uint8_t arrayIndex = queueHandle->headIndex%QUEUELENGTH;
      
      // set data length
      queueHandle->queue[arrayIndex].dataLength = dataLength;
      
      // set data start in the databuffer
      queueHandle->queue[arrayIndex].dataStart = dataStart;
      
      // set message status
      queueHandle->queue[arrayIndex].messageStatus = READY_FOR_TX;
      
      // increment fifo frame counter
      queueHandle->frameCounter++;
      
      // increment the queue length
      queueHandle->queueLength++;
      queueHandle->dataPacketsIN++;
      queueHandle->bytesIN += dataLength;
      
      // set the peak if necessary
      if( queueHandle->queueLength > queueHandle->queueLengthPeak )
      {
         queueHandle->queueLengthPeak = queueHandle->queueLength;
      }
      
      // increment head index
      queueHandle->headIndex++;
      
      // set receiving state on the queue object
      //queueHandle->queue[queueHandle->headIndex%QUEUELENGTH].messageStatus = RECEIVING_RX;

      // return new pointer
      return queueHandle->queue[queueHandle->headIndex%QUEUELENGTH].data;
   }

   // queue is full, return old pointer
   queueHandle->queueFull++;
   return queueHandle->queue[queueHandle->headIndex%QUEUELENGTH].data;
}

// ----------------------------------------------------------------------------
/// \brief     Returns pointer to the head buffer of the queue.
///
/// \param     none
///
/// \return    none
uint8_t* queue_getHeadBuffer( queue_handle_t *queueHandle )
{
   return queueHandle->queue[queueHandle->headIndex%QUEUELENGTH].data;
}

// ----------------------------------------------------------------------------
/// \brief     Returns pointer to the tail buffer of the queue.
///
/// \param     none
///
/// \return    none
uint8_t* queue_getTailBuffer( queue_handle_t *queueHandle )
{
   return queueHandle->queue[queueHandle->tailIndex%QUEUELENGTH].data;
}

/********************** (C) COPYRIGHT Reichle & De-Massari *****END OF FILE****/