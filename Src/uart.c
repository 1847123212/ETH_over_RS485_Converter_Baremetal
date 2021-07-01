// ****************************************************************************
/// \file      uart.c
///
/// \brief     bus uart module
///
/// \details   Module used to initialise bus uart peripherals completed with 
///            functions for receiving and transmitting data.
///
/// \author    Nico Korn
///
/// \version   1.0
///
/// \date      27102020
/// 
/// \copyright Copyright 2020 Reichle & De-Massari AG
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
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include "list.h"
#include "queue.h"
#include "uart.h"

// Private defines ************************************************************
#define FRAMEGAPTIME   ( 1000u ) // 1=0.1 us (1000 for 6mbit), (2000 for 3mbit)

// Private types     **********************************************************

// Private variables **********************************************************
static const uint8_t          preAmbleSFD[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};
static uint8_t                *rxBuffer;

//#pragma data_alignment = 4
//static uint8_t                rxBuffer2[BUFFERLENGTH];

static volatile FlagStatus    randomTimeoutFlag = SET;  
static volatile FlagStatus    framegapTimeoutFlag = SET;  

// Global variables ***********************************************************
UART_HandleTypeDef            huart2;
DMA_HandleTypeDef             hdma_usart2_rx;
DMA_HandleTypeDef             hdma_usart2_tx;
CRC_HandleTypeDef             hcrc;
extern ETH_HandleTypeDef      heth;
TIM_HandleTypeDef             BusTimHandleTx;
TIM_HandleTypeDef             BusTimHandleRx;
RNG_HandleTypeDef             RngHandle;
static queue_handle_t         *uartQueue;
static queue_handle_t         *ethQueue;

// Private function prototypes ************************************************
static void      crc_init                       ( void );
static uint32_t  uart_calcCRC                   ( uint32_t* dataPointer, uint32_t dataLength );
static uint8_t   uart_send                      ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length );
static void      uart_receive                   ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length );
static void      bus_randomTimer_init           ( void );
static void      bus_uart_startRandomTimeout    ( void );
static void      bus_framegapTimer_init         ( void );
static void      bus_uart_rng_init              ( void );
static uint32_t  bus_uart_getRandomNumber       ( void );

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
   
   // init random number generator
   bus_uart_rng_init();

   // Timer for bus access
   bus_randomTimer_init();
   bus_framegapTimer_init();

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
   
   // USART2 2 init
   huart2.Instance                           = USART2;
   huart2.Init.BaudRate                      = 6000000u;
   huart2.Init.WordLength                    = UART_WORDLENGTH_8B;
   huart2.Init.StopBits                      = UART_STOPBITS_1;
   huart2.Init.Parity                        = UART_PARITY_NONE;
   huart2.Init.Mode                          = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl                     = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling                  = UART_OVERSAMPLING_16;
   huart2.Init.OneBitSampling                = UART_ONE_BIT_SAMPLE_DISABLE;
   huart2.Init.ClockPrescaler                = UART_PRESCALER_DIV1;
   huart2.AdvancedInit.AdvFeatureInit        = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart2) != HAL_OK)
   {
      Error_Handler();
   }
   if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
   {
      Error_Handler();
   }
   if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
   {
      Error_Handler();
   }
   if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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

   // set irq
   HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
   HAL_NVIC_EnableIRQ(USART2_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
   
   // start to receive uart(rs485)
   ethQueue = get_ethQueue();
   uartQueue = get_uartQueue();
   rxBuffer = queue_getHeadBuffer( uartQueue );
   //rxBuffer = rxBuffer2;
   uart_receive( &huart2, (uint8_t*)rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     CRC Initialization Function             
///
/// \param     none
///
/// \return    none
static void crc_init(void)
{
   // init the clock
   __HAL_RCC_CRC_CLK_ENABLE();
   
   // init the crc peripheral
   hcrc.Instance                       = CRC;
   hcrc.Init.DefaultPolynomialUse      = DEFAULT_POLYNOMIAL_ENABLE;
   hcrc.Init.DefaultInitValueUse       = DEFAULT_INIT_VALUE_ENABLE;
   hcrc.Init.InputDataInversionMode    = CRC_INPUTDATA_INVERSION_NONE;
   hcrc.Init.OutputDataInversionMode   = CRC_OUTPUTDATA_INVERSION_DISABLE;
   hcrc.InputDataFormat                = CRC_INPUTDATA_FORMAT_BYTES;
   if (HAL_CRC_Init(&hcrc) != HAL_OK)
   {
      Error_Handler();
   }
}

uint8_t uart_output( uint8_t* buffer, uint16_t length )
{
   return 1;
   // check for the peripheral and bus access to be ready
   if( huart2.gState != HAL_UART_STATE_READY || randomTimeoutFlag != SET || framegapTimeoutFlag != SET )
   {
      return 0;
   }
   
   uint32_t   crc32;
   uint8_t*   crcFragment;
   
   //if( memcmp( buffer, preAmbleSFD, 8u ) != 0 )
   //{
    // copy preamble into buffer, caution this is dangerous but since the
    // usb data header is bigger than 8 this is no problem for this case.
    memcpy( buffer, preAmbleSFD, 8u );
    
    // calculate crc32 value
    crc32 = uart_calcCRC( (uint32_t*)&buffer[MACDSTFIELD], (uint32_t)length );
    
    // append crc to the outputbuffer
    crcFragment = (uint8_t*)&crc32;
    
    for( uint8_t i=0, j=3; i<4; i++,j-- )
    {
       buffer[MACDSTFIELD+length+i] = crcFragment[j];
    }
   //}
   
   // check for the peripheral and bus access to be ready
   if( huart2.gState != HAL_UART_STATE_READY || randomTimeoutFlag != SET || framegapTimeoutFlag != SET )
   {
      return 0;
   }

   // switch the RS485 transceiver into transmit mode
   HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_SET);
   
   // start transmitting in interrupt mode
   __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);         // disable rx interrupt

   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buffer, BUFFERLENGTH);
   
   // send the data
   if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, length+(uint16_t)PREAMBLESFDLENGTH+(uint16_t)CRC32LENGTH ) != HAL_OK )
   {
      return 0;
   }
   
   return 1;
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
/// \param     [in] uart handler
/// \param     [in] pointer to data buffer
/// \param     [in] length of the buffer
///
/// \return    none
static uint8_t uart_send( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length )
{
   return 0;
}

//------------------------------------------------------------------------------
/// \brief     Function to receive data over uart
///
/// \param     [in]  UART_HandleTypeDef *huart
/// \param     [in]  uint8_t *buffer
/// \param     [in]  uint16_t length
///
/// \return    none
static void uart_receive( UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t length )
{
   // Error counter for debugging purposes
   static uint32_t   uart_rx_err_counter;

   // RS485 set to listening
   HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_RESET);
   
   // enable idle line and rx interrupt
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
   __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
   
   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buffer, BUFFERLENGTH);
   
   // start receiving in interrupt mode
   if(HAL_UART_Receive_DMA(huart, buffer, length) != HAL_OK)
   {
      uart_rx_err_counter++;
   }
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
   //while(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != SET);
   //__HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_TCF);

   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   
   // dequeue the tail
   queue_dequeue( ethQueue );
   
   // start to receive data
   uart_receive( huart, rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     Rx Transfer completed callback (not needed, because idle line
///            detection is being used.                   
///
/// \param     [in]  UART_HandleTypeDef *huart
///
/// \return    none
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
}

//------------------------------------------------------------------------------
/// \brief     Error callback of the uart peripheral                   
///
/// \param     [in]  UART_HandleTypeDef *UartHandle
///
/// \return    none
void HAL_UART_ErrorCallback( UART_HandleTypeDef *UartHandle )
{
   static uint16_t errorCallbackCounter;
   errorCallbackCounter++;
}

//------------------------------------------------------------------------------
/// \brief     Error abort callback of the uart peripheral                   
///
/// \param     [in]  UART_HandleTypeDef *UartHandle
///
/// \return    none
void HAL_UART_AbortCpltCallback( UART_HandleTypeDef *UartHandle )
{
   static uint16_t abortCallbackCounter;
   abortCallbackCounter++;
}

//------------------------------------------------------------------------------
/// \brief     Rx idle line detection callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_IdleLnCallback( UART_HandleTypeDef *huart )
{
   static uint32_t   framelength;
   static uint32_t   framelengthError;
   static uint32_t   preAmbleError;
   static uint32_t   crcError;
   static uint32_t   validBusFrame;
   
   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rxBuffer, BUFFERLENGTH);
      
   // take action on the peripherals
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   
   // get message length
   framelength = BUFFERLENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);
   
   // abort if input data is 0 bytes in length or too long or too short for a ethernet frame
   if( (framelength > (uint16_t)(ETHSIZE+PREAMBLESFDLENGTH)) || (framelength < (uint16_t)MINSIZE))
   {
      // start receive irq
      framelengthError++;
      uart_receive( huart, rxBuffer, BUFFERLENGTH );
      return;
   }
   
   // preamble check
   if(( memcmp( ( void * ) (rxBuffer), ( void * ) (preAmbleSFD), (PREAMBLESFDLENGTH)) != 0 ))
   {
      // start receive irq
      preAmbleError++;
      uart_receive( huart, rxBuffer, BUFFERLENGTH );
      return ;
   }
   
   // crc check
   if( uart_calcCRC( (uint32_t*)(rxBuffer+MACDSTFIELD), (uint32_t)(framelength-PREAMBLESFDLENGTH) ) != 0 )
   {
      // start receive irq
      crcError++;
      uart_receive( huart, rxBuffer, BUFFERLENGTH );
      return ;
   }

   // create a new node in the list, with the received data
   validBusFrame++;
   
   queue_enqueue( rxBuffer+MACDSTFIELD, (uint16_t)(framelength-PREAMBLESFDLENGTH-CRC32LENGTH), uartQueue );
   
   // start to receive again
   rxBuffer = queue_getHeadBuffer( uartQueue );
   //rxBuffer = rxBuffer2;
   
   // start to receive again
   uart_receive( huart, (uint8_t*)rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     bus access timer initialisation   
///
/// \param     none
///
/// \return    none
static void bus_randomTimer_init( void )
{
   // set prescaler to 0.1 us ticks
   uint32_t uwPrescalerValue = (uint32_t)(120000000 / (10000000)) - 1;
   
   // clock (APB1)
   __HAL_RCC_TIM3_CLK_ENABLE();
   
   // configuration
   BusTimHandleTx.Instance               = TIM3;
   BusTimHandleTx.Init.Period            = 0; 
   BusTimHandleTx.Init.Prescaler         = uwPrescalerValue;
   BusTimHandleTx.Init.ClockDivision     = 0;
   BusTimHandleTx.Init.CounterMode       = TIM_COUNTERMODE_UP;
   BusTimHandleTx.Init.RepetitionCounter = 0;
   
   HAL_TIM_Base_Init(&BusTimHandleTx);
   
   // Set the TIM3 priority
   HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Starts the timer with a random value
///
/// \param     none
///
/// \return    none
inline static void bus_uart_startRandomTimeout( void )
{
   // reset the flag
   randomTimeoutFlag = RESET;
   // set a random number for the auto reload register
   TIM3->ARR = (uint32_t)(bus_uart_getRandomNumber() % 1000)+1000;//3mbit = (uint32_t)(bus_uart_getRandomNumber() % 200)*20+200; // 6mbit = (uint32_t)(bus_uart_getRandomNumber() % 100)*10+100;
   // set counter value to 0
   TIM3->CNT = 0;
   // start the timer
   HAL_TIM_Base_Start_IT(&BusTimHandleTx);
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3, which sets the bus access flag.
///
/// \param     none
///
/// \return    none
inline void bus_uart_randomTimeoutCallback( void )
{
   // set the timeout flag to 1
   randomTimeoutFlag = SET;
   // stop the timer
   HAL_TIM_Base_Stop_IT(&BusTimHandleTx);
}

//------------------------------------------------------------------------------
/// \brief     bus access timer initialisation   
///
/// \param     none
///
/// \return    none
static void bus_framegapTimer_init( void )
{
   // set prescaler to 0.1 us ticks
   uint32_t uwPrescalerValue = (uint32_t)(120000000 / (10000000)) - 1;
   
   // clock (APB1)
   __HAL_RCC_TIM5_CLK_ENABLE();
   
   // configuration
   BusTimHandleRx.Instance               = TIM5;
   BusTimHandleRx.Init.Period            = FRAMEGAPTIME; 
   BusTimHandleRx.Init.Prescaler         = uwPrescalerValue;
   BusTimHandleRx.Init.ClockDivision     = 0;
   BusTimHandleRx.Init.CounterMode       = TIM_COUNTERMODE_UP;
   BusTimHandleRx.Init.RepetitionCounter = 0;
   
   HAL_TIM_Base_Init(&BusTimHandleRx);
   
   // Set the TIM3 priority
   HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Starts the timer with a defined value
///
/// \param     none
///
/// \return    none
inline void bus_uart_startFramegap( void )
{
   framegapTimeoutFlag = RESET;
  __HAL_TIM_DISABLE_IT(&BusTimHandleRx, TIM_IT_UPDATE);
   TIM5->CNT = 0;
   __HAL_TIM_ENABLE_IT(&BusTimHandleRx, TIM_IT_UPDATE);
   __HAL_TIM_ENABLE(&BusTimHandleRx);
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3, which sets the bus access flag.
///
/// \param     none
///
/// \return    none
inline void bus_uart_framegapTimeoutCallback( void )
{
   // set the timeout flag to 1
   framegapTimeoutFlag = SET;
   
   // stop the timer
   HAL_TIM_Base_Stop_IT(&BusTimHandleRx);
   
   // start random timeout
   bus_uart_startRandomTimeout();
}

// ----------------------------------------------------------------------------
/// \brief     random number generator init
///
/// \param     none
///
/// \return    none
static void bus_uart_rng_init( void )
{
   __HAL_RCC_RNG_CLK_ENABLE();
   RngHandle.Instance = RNG;
   if (HAL_RNG_Init(&RngHandle) != HAL_OK)
   {
      // Initialization Error
      while(1);
   }
}

// ----------------------------------------------------------------------------
/// \brief     returns a random number generated number
///
/// \param     none
///
/// \return    rand number
static uint32_t bus_uart_getRandomNumber( void )
{
   uint32_t rand;
   HAL_RNG_GenerateRandomNumber(&RngHandle,&rand);
	return rand;
}

/********************** (C) COPYRIGHT Reichle & De-Massari *****END OF FILE****/