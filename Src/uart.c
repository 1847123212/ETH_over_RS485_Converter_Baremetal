// ****************************************************************************
/// \file      pcu_bus_uart.c
///
/// \brief     bus uart module
///
/// \details   Module used to initialise bus uart peripherals completed with 
///            functions
///
/// \author    Nico Korn
///
/// \version   0.1
///
/// \date      20190124
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
#include <string.h>
#include <stdio.h>
#include "buffer.h"
#include "uart.h"

// Private types     **********************************************************

// Private variables **********************************************************
static BUS_UART_RX_t          busuartRx;
static const uint8_t          preAmbleSFD[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};

// Global variables ***********************************************************
extern UART_HandleTypeDef     huart2;
extern DMA_HandleTypeDef      hdma_usart2_rx;
extern CRC_HandleTypeDef      hcrc;

// Private function prototypes ************************************************


//------------------------------------------------------------------------------
/// \brief     Function to set RTS/CTS pins on rs485 driver                  
///
/// \param     [in]  uart_cmd_t setter
///
/// \return    none
void bus_uart_setRs485( uart_cmd_t setter )
{
   if( setter != RECEIVE )
   {
      HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_SET);
   }
   else
   {
      HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_RESET);
   }   
}

//------------------------------------------------------------------------------
/// \brief     Function returns handler of the bus peripheral             
///
/// \param     -
///
/// \return    none
UART_HandleTypeDef* bus_uart_getHandler( void )
{
   return &huart2;
}

//------------------------------------------------------------------------------
/// \brief     Returns size of the last received ethernet frame                 
///
/// \param     - 
///
/// \return    uint16_t size
uint16_t bus_uart_getSize( void )
{
   return busuartRx.frameSize;
}

//------------------------------------------------------------------------------
/// \brief     Returns pointer to the last received ethernet frame data              
///
/// \param     - 
///
/// \return    uint16_t busuartRx.buffer
uint8_t* bus_uart_getBufferpointer( void )
{
   return busuartRx.buffer;
}

//------------------------------------------------------------------------------
/// \brief     Returns pointer to the buffer length              
///
/// \param     - 
///
/// \return    uint16_t busuartRx.buffer
uint16_t bus_uart_getBuffersize( void )
{
   return busuartRx.bufferSize;
}

//-----------------------------------------------------------------------------
/// \brief      This function resets rx dependend flags and 
///             variables
///
/// \return     
void bus_uart_resetRx( void )
{
   // reset flags and variables
   busuartRx.uartState = 0;
   // reset command information
   busuartRx.frameSize = 0;
}

uint8_t bus_uart_frameCheck( uint8_t* framePointer, uint16_t frameLength )
{  
   // check for the preamble and start frame delimiter
   if(( memcmp( ( void * ) framePointer, ( void * ) preAmbleSFD, PREAMBLESFDLENGTH) != 0 ))
   {
      return 0;
   }
   
   // check the frames crc
   if( bus_uart_calcCRC( (uint32_t*)(framePointer+MACDSTFIELD), (uint32_t)(frameLength-PREAMBLESFDLENGTH) ) != 0 )
   {
      return 0;
   }
   
   // both preamble/sfd and crc check has been successfull
   return 1;
}

void uart_output( uint8_t* buffer, uint16_t length )
{
   static     uint8_t*  crcFragment;
   static     uint8_t   txBuffer[1600];
   static     uint32_t  crc32;
   
   // set preamble (could be avoided, by setting the preamble+sfd at the beginning)
   for( uint8_t i=0; i<PREAMBLESFDLENGTH; i++ )
   {
      txBuffer[i] = preAmbleSFD[i];
   }
   
   // copy data into tx output buffer
   memcpy( &txBuffer[MACDSTFIELD], buffer, length );
   
   // calculate crc32 value
   crc32 = bus_uart_calcCRC( (uint32_t*)&txBuffer[MACDSTFIELD], (uint32_t)length );
   
   // append crc to the outputbuffer
   crcFragment = (uint8_t*)&crc32;
   
   for( uint8_t i=0, j=3; i<4; i++,j-- )
   {
      *(txBuffer+MACDSTFIELD+length+i) = *(crcFragment+j);
   }
   length += CRC32LENGTH;
   
   // add preamble/sfd length 
   length += PREAMBLESFDLENGTH;
   
   // send the data in the buffer
   bus_uart_send(&huart2, txBuffer, length);
}

//------------------------------------------------------------------------------
/// \brief     Calculates CRC Value of given data and length             
///
/// \param     [in] data pointer 
///            [in] data length
///
/// \return    checksum value
uint32_t bus_uart_calcCRC( uint32_t* dataPointer, uint32_t dataLength )
{
   return HAL_CRC_Calculate(&hcrc, dataPointer, dataLength);
}

//------------------------------------------------------------------------------
/// \brief     Function used to send data over uart           
///
/// \param     -
///
/// \return    none
void bus_uart_send( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size )
{
   // wait until tx/rx is completed
   while(huart2.gState != HAL_UART_STATE_READY){}
   // switch the RS485 transceiver into transmit mode
   bus_uart_setRs485( TRANSMIT );
   // start transmitting in interrupt mode
   HAL_UART_Transmit_DMA(huart, pData, Size);
}

//------------------------------------------------------------------------------
/// \brief     Function to receive data over uart
///
/// \param     -
///
/// \return    none
void bus_uart_receive( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size )
{
   // wait until tx/rx is completed
   while(huart2.gState != HAL_UART_STATE_READY){}   
   // switch the RS485 transceiver into receive mode
   bus_uart_setRs485( RECEIVE );
   // enable idle line interrupt
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
   // start receiving in interrupt mode
   HAL_UART_Receive_DMA(huart, pData, Size);
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback                   
///
/// \param     - 
///
/// \return    none
void bus_uart_TxCpltCallback( void )
{
   // set the tx pointer increment flag to set it to the next bufferslot
   buffer_setTxIncrReq();
   
   // start to receive data
   bus_uart_receive( &huart2, (uint8_t*)buffer_getRxPointer(), BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     Rx Transfer completed callback (not needed, because idle line
///            detection is being used.                   
///
/// \param     - 
///
/// \return    none
void bus_uart_RxCpltCallback( void )
{
}

//------------------------------------------------------------------------------
/// \brief     Rx idle line detection callback                   
///
/// \param     - 
///
/// \return    none
void bus_uart_IdleLnCallback( void )
{
   const uint16_t bytesLeftDmaBuffer = ((BDMA_Channel_TypeDef*)hdma_usart2_rx.Instance)->CNDTR;
   static uint16_t frameSize;
   
   // stop irq 
   HAL_UART_DMAStop(&huart2);
   HAL_UART_Abort_IT(&huart2);
   
   // get message length
   frameSize = BUFFERLENGTH - bytesLeftDmaBuffer;
   buffer_setMessageSize( frameSize );
   
   // abort if input data is 0 bytes in length or too long or too short for a ethernet frame
   if( frameSize == 0 || frameSize > ETHSIZE+PREAMBLESFDLENGTH || frameSize < MINSIZE)
   {
      // start receiving with new flags
      bus_uart_receive( &huart2, (uint8_t*)buffer_getRxPointer(), BUFFERLENGTH );
      return;
   }
   
   // abort if the message doesn't have a preamble, sfd and valid fcs
   if( bus_uart_frameCheck( (uint8_t*)buffer_getRxPointer(), frameSize ) != 1 )
   {
      // start receiving with new flags
      bus_uart_receive( &huart2, (uint8_t*)buffer_getRxPointer(), BUFFERLENGTH );
      return;
   }
  
   // set the rx pointer increment flag to set it to the next bufferslot
   buffer_setRxIncrReq();
}