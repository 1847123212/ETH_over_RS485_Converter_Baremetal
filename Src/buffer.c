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
#define STACKLENGTH     50

// Private types     **********************************************************

// Private variables **********************************************************

// Private function prototypes ************************************************
static BufferSlot          buffer[STACKLENGTH];
static BufferSlot*         bufferTxPointer;
static BufferSlot*         bufferRxPointer;
static size_t              size;
static uint32_t            topAddress;
static FlagStatus          rxPointerIncrFlag;
static FlagStatus          txPointerIncrFlag;

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
   // reset the flags
   rxPointerIncrFlag = RESET;
   txPointerIncrFlag = RESET;
   // calculate size of the array of structs
   size = sizeof(buffer);
   // top address limit of the array
   topAddress = (uint32_t)(&buffer + size);
   // start to receive uart(rs485)
   bus_uart_receive( &huart2, (uint8_t*)bufferRxPointer, BUFFERLENGTH );
   // start to receive eth
}

// ----------------------------------------------------------------------------
/// \brief     Get next buffer from queue through a pointer
///
/// \return    BufferSlot
uint8_t buffer_setNextSlotTx( void )
{
   if( bufferTxPointer+1 <= bufferRxPointer )
   {
      // found a slot which is ready to be send
      buffer_incrTxPointer();
      return 1;
   }
   else
   {
      // no slot available with data to send
      return 0;
   }
}

// ----------------------------------------------------------------------------
/// \brief     Set next bufferslot in the array of bufferlsot structs
///
/// \return    1 success, 0 error
uint8_t buffer_setNextSlotRx( void )
{
   if( bufferRxPointer+1 != bufferTxPointer )
   {
      // found a free slot
      buffer_incrRxPointer();
      // reset flags on the new pointed struct
      bufferRxPointer->dataLengthInBuffer = 0;
      bufferRxPointer->toETHFlag = 0xff;
      return 1;
   }
   else
   {
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
   if( output->toETHFlag )
   {
      // setup the next frame pointed by the tx pointer and send it hrough the eth interface
      eth_output( output->buffer, output->dataLengthInBuffer );
   }
   else
   {
      // setup the next frame pointed by the tx pointer and send it hrough the uart interface
      uart_output( output->buffer, output->dataLengthInBuffer );
   }
}

// ----------------------------------------------------------------------------
/// \brief     Increment the tx pointer to the array of message buffer
///
/// \return    -
BufferSlot* buffer_incrTxPointer( void )
{
   // check if the pointer needs to be set to the beginning
   if( (uint32_t)(bufferTxPointer+sizeof(BufferSlot)) < topAddress)
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
   if( (uint32_t)(bufferTxPointer+sizeof(BufferSlot)) < topAddress)
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
/// \brief     Sets message size on current pointed bufferslot
///
/// \return    -
BufferSlot* buffer_getRxPointer( void )
{
   return bufferRxPointer;
}

void buffer_setRxIncrReq( void )
{
   rxPointerIncrFlag = SET;
}

void buffer_setTxIncrReq( void )
{
   txPointerIncrFlag = SET;
}

void buffer_manager( void )
{
   // check for the tx flag to increment the pointer on the array of messagebuffers
   if( txPointerIncrFlag == SET )
   {
      // try to increment the pointer by setting the next slot
      if( buffer_setNextSlotTx() )
      {
         // incrementation was successful
         txPointerIncrFlag = RESET;
      }
      else
      {
         // incrementation wasn't successfull, because the tx pointer can't overtake the rx pointer
         // try to increment another time
         txPointerIncrFlag = SET;
      }
   }
   
   // check for the rx flag to increment the pointer on the array of messagebuffers
   if( rxPointerIncrFlag == SET )
   {
      // try to increment the pointer by setting the next slot
      if( buffer_setNextSlotRx() )
      {
         // incrementation was successful
         rxPointerIncrFlag = RESET;
         // start to receive on the new slot
         bus_uart_receive( &huart2, bufferRxPointer->buffer, BUFFERLENGTH );
      }
      else
      {
         // incrementation wasn't successfull, because there is the tx pointer on the next slot
         // try to increment another time
         rxPointerIncrFlag = SET;
      }
   }
   
   //is there data to send?
   if( bufferTxPointer->dataLengthInBuffer != 0 )
   {
      // start to send data
      buffer_output( bufferTxPointer );
   }
}