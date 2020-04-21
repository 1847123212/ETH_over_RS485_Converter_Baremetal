// ****************************************************************************
/// \file      uart.c
///
/// \brief     bus uart module
///
/// \details   Module used to initialise bus uart peripherals completed with 
///            functions
///
/// \author    Nico Korn
///
/// \version   0.2
///
/// \date      10042019
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
#include <stdlib.h>
#include "list.h"
#include "uart.h"

// Private defines ************************************************************
#define FRAMEGAPTIME   ( 1000u ) // 1=0.1 us

// Private types     **********************************************************

// Private variables **********************************************************
static const uint8_t          preAmbleSFD[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};
static uint8_t                rxBuffer[BUFFERLENGTH];
static uint8_t                txBuffer[BUFFERLENGTH];

static volatile FlagStatus    randomTimeoutFlag = SET;  
static volatile FlagStatus    framegapTimeoutFlag = SET;  
static volatile FlagStatus    txCplt = SET;

// Global variables ***********************************************************
UART_HandleTypeDef            huart2;
DMA_HandleTypeDef             hdma_usart2_rx;
DMA_HandleTypeDef             hdma_usart2_tx;
CRC_HandleTypeDef             hcrc;
extern ETH_HandleTypeDef      heth;
TIM_HandleTypeDef             BusTimHandleTx;
TIM_HandleTypeDef             BusTimHandleRx;
RNG_HandleTypeDef             RngHandle;

// Private function prototypes ************************************************
static void      crc_init                       ( void );
static uint32_t  uart_calcCRC                   ( uint32_t* dataPointer, uint32_t dataLength );
static void      uart_send                      ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length );
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
   huart2.Init.BaudRate                      = 6000000;
   huart2.Init.WordLength                    = UART_WORDLENGTH_8B;
   huart2.Init.StopBits                      = UART_STOPBITS_1;
   huart2.Init.Parity                        = UART_PARITY_NONE;
   huart2.Init.Mode                          = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl                     = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling                  = UART_OVERSAMPLING_8;
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

   // clear it pending bit to avoid trigger at beginning
   __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF);

   // set irq
   HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(USART2_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
   
   // set preamble in the tx buffer
   memcpy(txBuffer, preAmbleSFD, PREAMBLESFDLENGTH);
   
   // start to receive uart(rs485)
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

void uart_output( uint8_t* buffer, uint16_t length )
{
   // wait for the peripheral to be ready
   while( txCplt != SET );
   txCplt = RESET;
       
   // copy data into tx output buffer
   memcpy( &txBuffer[MACDSTFIELD], buffer, length );
   
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
/// \param     [in] uart handler
/// \param     [in] pointer to data buffer
/// \param     [in] length of the buffer
///
/// \return    none
static void uart_send( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length )
{
   // Error counter for debugging purposes
   static uint32_t   uart_tx_err_counter;
   static uint32_t   crc32;
   static uint8_t*   crcFragment;
   
   // calculate crc32 value
   crc32 = uart_calcCRC( (uint32_t*)&pData[MACDSTFIELD], (uint32_t)length );
   
   // append crc to the outputbuffer
   crcFragment = (uint8_t*)&crc32;
   
   for( uint8_t i=0, j=3; i<4; i++,j-- )
   {
      pData[MACDSTFIELD+length+i] = crcFragment[j];
   }
   
   // if necessary, wait for interframegap end
   while( framegapTimeoutFlag != SET );
   
   // start the random countdown to check if the bus is not occupied
   do
   {
      bus_uart_startRandomTimeout();
      while( randomTimeoutFlag != SET );
   }while( framegapTimeoutFlag != SET );
   
   // switch the RS485 transceiver into transmit mode
   HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_SET);
   
   // start transmitting in interrupt mode
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   
   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pData, BUFFERLENGTH);
   
   // send the data
   if(HAL_UART_Transmit_DMA(huart, (uint8_t*)pData, length+(uint16_t)PREAMBLESFDLENGTH+(uint16_t)CRC32LENGTH ) != HAL_OK )
   {
      uart_tx_err_counter++;
   }
}

//------------------------------------------------------------------------------
/// \brief     Function to receive data over uart
///
/// \param     -
///
/// \return    none
static void uart_receive( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length )
{
   // Error counter for debugging purposes
   static uint32_t   uart_rx_err_counter;

   // wait until uart peripheral is ready
   while(huart->gState != HAL_UART_STATE_READY);
   
   // RS485 set to listening
   HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_RESET);
   
   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pData, BUFFERLENGTH);
   
   // enable idle line and rx interrupt
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
   __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
   
   // start receiving in interrupt mode
   if(HAL_UART_Receive_DMA(huart, pData, length) != HAL_OK)
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
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   txCplt = SET;
   
   // start to receive data
   uart_receive( huart, (uint8_t*)rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     Rx Transfer completed callback (not needed, because idle line
///            detection is being used.                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
}

//------------------------------------------------------------------------------
/// \brief     Error callback of the uart peripheral                   
///
/// \param     - 
///
/// \return    none
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
   static uint16_t errorCallbackCounter;
   errorCallbackCounter++;
}

//------------------------------------------------------------------------------
/// \brief     Error abort callback of the uart peripheral                   
///
/// \param     - 
///
/// \return    none
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *UartHandle)
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
   static uint16_t    framelength;
   
   static uint16_t   framelengthError;
   static uint16_t   preAmbleError;
   static uint16_t   crcError;
   
   // take action on the peripherals
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   
   // get message length
   framelength = BUFFERLENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);
   
   // abort if input data is 0 bytes in length or too long or too short for a ethernet frame
   if( (framelength == (uint16_t)0) || (framelength > (uint16_t)(ETHSIZE+PREAMBLESFDLENGTH)) || (framelength < (uint16_t)MINSIZE))
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
      //uart_receive( huart, rxBuffer, BUFFERLENGTH );
      //return;
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
   list_insertData( rxBuffer, framelength, UART_TO_ETH );
   
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
   HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Starts the timer with a random value
///
/// \param     -
///
/// \return    none
inline static void bus_uart_startRandomTimeout( void )
{
   // reset the flag
   randomTimeoutFlag = RESET;
   // set a random number for the auto reload register
   TIM3->ARR = (uint32_t)(bus_uart_getRandomNumber() % 1000)+10;
   // set counter value to 0
   TIM3->CNT = 0;
   // start the timer
   HAL_TIM_Base_Start_IT(&BusTimHandleTx);
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3, which sets the bus access flag.
///
/// \param     -
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
   HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Starts the timer with a defined value
///
/// \param     [in] timer value for the auto reload register
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
/// \param     -
///
/// \return    none
inline void bus_uart_framegapTimeoutCallback( void )
{
   // set the timeout flag to 1
   framegapTimeoutFlag = SET;
   // stop the timer
   HAL_TIM_Base_Stop_IT(&BusTimHandleRx);
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
      /* Initialization Error */
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
	/* Get a 32-bit Random number */
	return rand;
}

/********************** (C) COPYRIGHT Reichle & De-Massari *****END OF FILE****/