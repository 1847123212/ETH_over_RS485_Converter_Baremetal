// ****************************************************************************
/// \file      buffer.c
///
/// \brief     buffer Module
///
/// \details   Module which manages the bufferslots and the queue for transmitting 
///            data between UART (RS485) and ETH. 
///
/// \author    Nico Korn
///
/// \version   0.1
///
/// \date      20190422
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


// Include ********************************************************************
#include "main.h"
#include "uart.h"
#include "eth.h"
#include "buffer.h"

// Private define *************************************************************
#define STACKLENGTH     150

// Private types     **********************************************************

// Private variables **********************************************************

// Private function prototypes ************************************************
#pragma location=0x24000000
static BufferSlot          buffer[STACKLENGTH];
static BufferSlot*         bufferTxPointer;
static BufferSlot*         bufferRxPointer;
static uint16_t            bufferTxPointerCnt;
static uint16_t            bufferRxPointerCnt;
static uint32_t            topAddress;
static uint8_t             bufferLockFlag;

// Private functions **********************************************************

// Global variables ***********************************************************
extern UART_HandleTypeDef  huart2;

// ----------------------------------------------------------------------------
/// \brief     Stack init
///
/// \return    -
void buffer_init( void )
{
   // reset the pointers
   bufferTxPointer = buffer;
   bufferRxPointer = buffer;
   // reset the pointer counter
   bufferTxPointerCnt = 0;
   bufferRxPointerCnt = 0;
   // top address limit of the array
   topAddress = sizeof(buffer)+(uint32_t)&buffer;
   // reset the buffer lock flag
   bufferLockFlag = 0;
}

// ----------------------------------------------------------------------------
/// \brief     Get next buffer from queue through a pointer
///
/// \return    BufferSlot
uint8_t buffer_setNextSlotTx( void )
{
   if( bufferTxPointerCnt < bufferRxPointerCnt )
   {
      // found a slot which is ready to be send
      buffer_incrTxPointer();
      bufferTxPointerCnt++;
      // if tx pointer counter has reached the same counter value as the rx pointer counter
      // reset booth to zero
      if( bufferTxPointerCnt == bufferRxPointerCnt )
      {
         bufferTxPointerCnt = 0;
         bufferRxPointerCnt = 0;
      }
      return 1;
   }
   else
   {
      // Error, in this situation in the logical view the tx pointer would be ahead 
      // of the rx pointer, a situation that cannot be. So reset here.
      // buffer_init();
      return 0;
   }
}

// ----------------------------------------------------------------------------
/// \brief     Set next bufferslot in the array of bufferlsot structs
///
/// \return    1 success, 0 error
uint8_t buffer_setNextSlotRx( void )
{
   if( bufferRxPointerCnt < STACKLENGTH-1 )
   {
      // found a free slot
      buffer_incrRxPointer();
      bufferRxPointerCnt++;
      // reset flags on the new pointed struct
      bufferRxPointer->dataLengthInBuffer = 0;
      bufferRxPointer->messageDirection = NOT_INITIALIZED;
      return 1;
   }
   else
   {
      // if the counter value is bigger than the STACKLENGTH, it means that the tx pointer
      // points on the next bufferslot in the array, sending a message. So return a 0.
      return 0;
   }
}

// ----------------------------------------------------------------------------
/// \brief     Send if there is something to send
///
/// \return    -
void buffer_output( BufferSlot* output )
{
   // if toETHFlag is one, it means that the current pointed message is dedicadet for the eth interface
   if( output->messageDirection == UART_TO_ETH )
   {
      // setup the next frame pointed by the tx pointer and send it hrough the eth interface
      eth_output( output->bufferSlot, output->dataLengthInBuffer );
   }
   else if( output->messageDirection == ETH_TO_UART )
   {
      // setup the next frame pointed by the tx pointer and send it hrough the uart interface
      uart_output( output->bufferSlot, output->dataLengthInBuffer );
   }
}

// ----------------------------------------------------------------------------
/// \brief     Increment the tx pointer to the array of message buffer
///
/// \return    -
BufferSlot* buffer_incrTxPointer( void )
{
   // check if the pointer needs to be set to the beginning
   if( (uint32_t)bufferTxPointer+sizeof(BufferSlot) < topAddress)
   {
      bufferTxPointer++;
   }
   else
   { 
      bufferTxPointer = buffer;
   }
   return bufferTxPointer;
}

// ----------------------------------------------------------------------------
/// \brief     Increment the rx pointer to the array of message buffer
///
/// \return    -
void buffer_incrRxPointer( void )
{
   // check if the pointer needs to be set to the beginning
   if( (uint32_t)bufferRxPointer+sizeof(BufferSlot) < topAddress)
   {
      bufferRxPointer++;
   }
   else
   { 
      bufferRxPointer = buffer;
   }
}

// ----------------------------------------------------------------------------
/// \brief     Sets message size on current pointed bufferslot
///
/// \return    -
void buffer_setMessageSize( uint16_t messagesize )
{
   bufferRxPointer->dataLengthInBuffer = messagesize;
}

// ----------------------------------------------------------------------------
/// \brief     Sets message direction either the message needs to be forwarded to
///            uart or eth.
///
/// \return    -
void buffer_setMessageDirection( message_direction_t direction )
{
   bufferRxPointer->messageDirection = direction;
}

// ----------------------------------------------------------------------------
/// \brief     Sets message size on current pointed bufferslot
///
/// \return    -
BufferSlot* buffer_getRxPointer( void )
{
   return bufferRxPointer;
}

// ----------------------------------------------------------------------------
/// \brief     Sets message size on current pointed bufferslot
///
/// \return    -
uint8_t* buffer_getBufferslotPointer( void )
{
   return bufferRxPointer->bufferSlot;
}

void buffer_manager( void )
{
   //is there data to send?
   if( bufferTxPointer->dataLengthInBuffer != 0 )
   {
      // start to send data
      buffer_output( bufferTxPointer );
      // increment pointer to set it to the next bufferslot
      buffer_setNextSlotTx();
   }
}

// ----------------------------------------------------------------------------
/// \brief     locks the ringbuffer access as shared source
///
/// \return    -
void buffer_lock( void )
{
   bufferLockFlag = 1;
}

// ----------------------------------------------------------------------------
/// \brief     unlocks the ringbuffer access as shared source
///
/// \return    -
void buffer_unlock( void )
{
   bufferLockFlag = 0;
}

// ----------------------------------------------------------------------------
/// \brief     unlocks the ringbuffer access as shared source
///
/// \return    -
uint8_t buffer_getLockStatus( void )
{
   return bufferLockFlag;
}