// ****************************************************************************
/// \file      pcu_uart.c
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
static const uint8_t          preAmbleSFD[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};
#pragma location=0x2403B000
ALIGN_32BYTES(uint8_t         rxBuffer[BUFFERLENGTH]);
#pragma location=0x2403B700
ALIGN_32BYTES(uint8_t         txBuffer[BUFFERLENGTH]);

// Global variables ***********************************************************
UART_HandleTypeDef            huart2;
DMA_HandleTypeDef             hdma_usart2_rx;
DMA_HandleTypeDef             hdma_usart2_tx;
extern CRC_HandleTypeDef      hcrc;

// Private function prototypes ************************************************
static void      crc_init          (void);
static void      mpu_uart_config   (void);
static uint32_t  uart_calcCRC      ( uint32_t* dataPointer, uint32_t dataLength );
static void      uart_send         ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
static void      uart_receive      ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );

//------------------------------------------------------------------------------
/// \brief     USART2 Initialization Function          
///
/// \param     none
///
/// \return    none
void uart_init( void )
{
   // variables
   GPIO_InitTypeDef GPIO_InitStruct;
   
   // enable clocks
   __HAL_RCC_DMA1_CLK_ENABLE();
   __HAL_RCC_USART2_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   
   // setup hardware crc
   crc_init();
   
   // set mpu for cache reading and writing
   mpu_uart_config();

   // RS485 CTS RTS GPIO Configuration
   // PD3     ------> CTS 
   // PD4     ------> RTS 
   GPIO_InitStruct.Pin                       = UART_PIN_BUS_RTS|UART_PIN_BUS_CTS;
   GPIO_InitStruct.Mode                      = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull                      = GPIO_NOPULL;
   GPIO_InitStruct.Speed                     = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
   
   // USART2 GPIO Configuration    
   // PD5     ------> USART2_TX
   // PD6     ------> USART2_RX 
   GPIO_InitStruct.Pin                       = GPIO_PIN_5|GPIO_PIN_6;
   GPIO_InitStruct.Mode                      = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull                      = GPIO_PULLUP;
   GPIO_InitStruct.Speed                     = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate                 = GPIO_AF7_USART2;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
   
   // USER CODE END USART2_Init 1
   huart2.Instance                           = USART2;
   huart2.Init.BaudRate                      = 5000000;
   huart2.Init.WordLength                    = UART_WORDLENGTH_8B;
   huart2.Init.StopBits                      = UART_STOPBITS_1;
   huart2.Init.Parity                        = UART_PARITY_NONE;
   huart2.Init.Mode                          = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl                     = UART_HWCONTROL_NONE;
   //huart2.Init.OverSampling                  = UART_OVERSAMPLING_16;
   huart2.Init.OverSampling                  = UART_OVERSAMPLING_16;
   huart2.Init.OneBitSampling                = UART_ONE_BIT_SAMPLE_DISABLE;
   huart2.Init.Prescaler                     = UART_PRESCALER_DIV1;
   huart2.Init.FIFOMode                      = UART_FIFOMODE_DISABLE;
   huart2.Init.TXFIFOThreshold               = UART_TXFIFO_THRESHOLD_1_8;
   huart2.Init.RXFIFOThreshold               = UART_RXFIFO_THRESHOLD_1_8;
   huart2.AdvancedInit.AdvFeatureInit        = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart2) != HAL_OK)
   {
      Error_Handler();
   }

   // USART2 DMA Init
   // USART2_RX Init
   hdma_usart2_rx.Instance                  = DMA1_Stream0;
   hdma_usart2_rx.Init.Request              = DMA_REQUEST_USART2_RX;
   hdma_usart2_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
   hdma_usart2_rx.Init.PeriphInc            = DMA_PINC_DISABLE;
   hdma_usart2_rx.Init.MemInc               = DMA_MINC_ENABLE;
   hdma_usart2_rx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
   hdma_usart2_rx.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
   hdma_usart2_rx.Init.Mode                 = DMA_NORMAL;
   hdma_usart2_rx.Init.Priority             = DMA_PRIORITY_LOW;
   hdma_usart2_rx.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;
   if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
   {
     Error_Handler();
   }
   __HAL_LINKDMA(&huart2,hdmarx,hdma_usart2_rx);

   // USART2_TX Init
   hdma_usart2_tx.Instance                  = DMA1_Stream1;
   hdma_usart2_tx.Init.Request              = DMA_REQUEST_USART2_TX;
   hdma_usart2_tx.Init.Direction            = DMA_MEMORY_TO_PERIPH;
   hdma_usart2_tx.Init.PeriphInc            = DMA_PINC_DISABLE;
   hdma_usart2_tx.Init.MemInc               = DMA_MINC_ENABLE;
   hdma_usart2_tx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
   hdma_usart2_tx.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
   hdma_usart2_tx.Init.Mode                 = DMA_NORMAL;
   hdma_usart2_tx.Init.Priority             = DMA_PRIORITY_LOW;
   hdma_usart2_tx.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;
   if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
   {
     Error_Handler();
   }
   __HAL_LINKDMA(&huart2,hdmatx,hdma_usart2_tx);

   // clear it pending bit to avoid trigger at beginning
   __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF);

   // set irq
   HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(USART2_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
   
   // set preamble in the tx buffer
   memcpy(txBuffer, preAmbleSFD, PREAMBLESFDLENGTH);
   
   // start to receive uart(rs485)
   uart_receive( &huart2, (uint8_t*)rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     Configure the MPU attributes 
///
/// \param     none
///
/// \return    none
static void mpu_uart_config(void)
{
   MPU_Region_InitTypeDef MPU_InitStruct;
   
   // Disable the MPU
   HAL_MPU_Disable();
   
   // Configure the MPU attributes as Cacheable write through 
   MPU_InitStruct.Enable = MPU_REGION_ENABLE;
   MPU_InitStruct.BaseAddress = 0x24000000;
   MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
   MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
   MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
   MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
   MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
   MPU_InitStruct.Number = MPU_REGION_NUMBER1;
   MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
   MPU_InitStruct.SubRegionDisable = 0x00;
   MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
   
   HAL_MPU_ConfigRegion(&MPU_InitStruct);
   
   // Enable the MPU
   HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

//------------------------------------------------------------------------------
/// \brief     CRC Initialization Function             
///
/// \param     none
///
/// \return    none
static void crc_init(void)
{
   hcrc.Instance = CRC;
   hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
   hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
   hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
   hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
   hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
   if (HAL_CRC_Init(&hcrc) != HAL_OK)
   {
      Error_Handler();
   }
}

void uart_output( uint8_t* buffer, uint16_t length )
{
   static     uint8_t*  crcFragment;
   static     uint32_t  crc32;
   
   // Invalidate data cache for uart tx Buffers
   SCB_InvalidateDCache_by_Addr((uint32_t *)txBuffer, BUFFERLENGTH);
      
   // copy data into tx output buffer
   memcpy( &txBuffer[MACDSTFIELD], buffer, length );
   
   // calculate crc32 value
   crc32 = uart_calcCRC( (uint32_t*)&txBuffer[MACDSTFIELD], (uint32_t)length );
   
   // append crc to the outputbuffer
   crcFragment = (uint8_t*)&crc32;
   
   for( uint8_t i=0, j=3; i<4; i++,j-- )
   {
      *(txBuffer+MACDSTFIELD+length+i) = *(crcFragment+j);
   }
   
   // add crc32 length to the total length
   length += CRC32LENGTH;
   
   // add preamble/sfd length 
   length += PREAMBLESFDLENGTH;
   
   // send the data in the buffer
   uart_send(&huart2, txBuffer, length);
}

//------------------------------------------------------------------------------
/// \brief     Calculates CRC Value of given data and length             
///
/// \param     [in] data pointer 
///            [in] data length
///
/// \return    checksum value
static uint32_t uart_calcCRC( uint32_t* dataPointer, uint32_t dataLength )
{
   return HAL_CRC_Calculate(&hcrc, dataPointer, dataLength);
}

//------------------------------------------------------------------------------
/// \brief     Function used to send data over uart           
///
/// \param     -
///
/// \return    none
static void uart_send( UART_HandleTypeDef *huart2, uint8_t *pData, uint16_t Size )
{
   // wait until uart peripheral is ready
   while(huart2->gState != HAL_UART_STATE_READY){}
   // RS485 drive enable
   GPIOD->BSRRL = UART_PIN_BUS_RTS|UART_PIN_BUS_CTS;
   // Clean and Invalidate data cache
   SCB_CleanInvalidateDCache();
   // start transmitting in interrupt mode
   HAL_UART_Transmit_DMA(huart2, (uint8_t*)pData, Size);
}

//------------------------------------------------------------------------------
/// \brief     Function to receive data over uart
///
/// \param     -
///
/// \return    none
static void uart_receive( UART_HandleTypeDef *huart2, uint8_t *pData, uint16_t Size )
{
   // wait until uart peripheral is ready
   while(huart2->gState != HAL_UART_STATE_READY){}   
   // RS485 drive disable
   GPIOD->BSRRH = UART_PIN_BUS_RTS|UART_PIN_BUS_CTS;
   // enable idle line interrupt
   __HAL_UART_ENABLE_IT(huart2, UART_IT_IDLE);
   // DEBUG
   memset(pData, 0 , BUFFERLENGTH);
   // start receiving in interrupt mode
   HAL_UART_Receive_DMA(huart2, pData, Size);
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart2 )
{
   // Invalidate data cache for uart tx Buffers
   SCB_CleanInvalidateDCache();
   // start to receive data
   uart_receive( huart2, (uint8_t*)rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     Rx Transfer completed callback (not needed, because idle line
///            detection is being used.                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart2 )
{
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback (not needed)                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
}

//------------------------------------------------------------------------------
/// \brief     Rx idle line detection callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_IdleLnCallback( UART_HandleTypeDef *huart2 )
{
   volatile uint16_t  bytesLeft = __HAL_DMA_GET_COUNTER(huart2->hdmarx);
   volatile uint16_t  frameSize = 0;
   volatile uint8_t*  preAmblePointer = NULL;
   
   // stop irq to reset values, disable idle line interrupt
   HAL_UART_DMAStop(huart2);
   HAL_UART_Abort_IT(huart2);
   __HAL_UART_DISABLE_IT(huart2, UART_IT_IDLE);
   
   // get message length
   frameSize = BUFFERLENGTH - bytesLeft;
   
   // abort if input data is 0 bytes in length or too long or too short for a ethernet frame
   if( frameSize == 0 || frameSize > ETHSIZE+PREAMBLESFDLENGTH || frameSize < MINSIZE)
   {
      // start receiving with new flags
      // Invalidate data cache for UART Rx Buffers
      SCB_CleanInvalidateDCache();
      uart_receive( huart2, rxBuffer, BUFFERLENGTH );
      return;
   }
   
   // preamble check
   preAmblePointer = rxBuffer;
   for(uint8_t i=0; i<4; i++)
   {
      if(*(preAmblePointer) == 0xAA)
      {
         if( memcmp( ( void * ) preAmblePointer, ( void * ) preAmbleSFD, PREAMBLESFDLENGTH) != 0 )
         {
            // Invalidate data cache for UART Rx Buffers
            SCB_CleanInvalidateDCache();
            uart_receive( huart2, rxBuffer, BUFFERLENGTH );
            return ;
         }
      }
      else
      {
         preAmblePointer++;
         frameSize--;    
      }  
   }
   
   // crc check
   if( uart_calcCRC( (uint32_t*)(preAmblePointer+MACDSTFIELD), (uint32_t)(frameSize-PREAMBLESFDLENGTH) ) != 0 )
   {
      // Invalidate data cache for UART Rx Buffers
      SCB_CleanInvalidateDCache();
      uart_receive( huart2, rxBuffer, BUFFERLENGTH );
      return ;
   }
   
   // check if bufferslot access is ready, if not wait
   while(buffer_getLockStatus() == 1);
   // access is now ready, lock the buffer access
   buffer_lock();

   // copy data into the current bufferslot
   memcpy((void*)buffer_getBufferslotPointer(), (void const*)preAmblePointer, frameSize);
   
   // set message direction
   buffer_setMessageDirection( UART_TO_ETH );

   // set message size
   buffer_setMessageSize( frameSize );
   
   // increment the bufferslot rx pointer
   buffer_setNextSlotRx();
   
   // unlock access
   buffer_unlock();
   
   // Invalidate data cache for UART Rx Buffers
   SCB_CleanInvalidateDCache();
   
   // start to receive again
   uart_receive( huart2, (uint8_t*)rxBuffer, BUFFERLENGTH );
}