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
#include <stdlib.h>
#include "list.h"
#include "uart.h"

// Private defines ************************************************************
#define BYTETIMOUTUS   ( 10 )

// Private types     **********************************************************

// Private variables **********************************************************
static const uint8_t          preAmbleSFD[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};
static uint8_t                rxBuffer[BUFFERLENGTH];
static uint8_t                txBuffer[BUFFERLENGTH];

static uint32_t               colCounter;

static volatile uint8_t       timeoutFlag = 0;  
static uint8_t                busIdleFlag = 1;

// Global variables ***********************************************************
UART_HandleTypeDef            huart2;
DMA_HandleTypeDef             hdma_usart2_rx;
DMA_HandleTypeDef             hdma_usart2_tx;
CRC_HandleTypeDef             hcrc;
extern ETH_HandleTypeDef      heth;
TIM_HandleTypeDef             LedTimHandle;
TIM_HandleTypeDef             BusTimHandle;
TIM_HandleTypeDef             BTimeoutTimHandle;

// Private function prototypes ************************************************
static void      crc_init          (void);
static uint32_t  uart_calcCRC      ( uint32_t* dataPointer, uint32_t dataLength );
static void      uart_send         ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
static void      uart_receive      ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
static void      bus_timer_init    ( void );
static void      setRandomWait     ( void );
static void      resetRandomWait   ( void );
static void      bus_uart_startRandomTimout( void );

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
   colCounter = 0;

   // LED for Collision indication
   //led_gpio_init();
   
   // Timer for Collision LED Configuration
   //led_timer_init();
   
   // Timer for bus access
   bus_timer_init();
   
   // Timer for bytetimout
   //bus_bytetimeout_timer_init();
   
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
   huart2.Instance = USART2;
   huart2.Init.BaudRate = 6000000;
   huart2.Init.WordLength = UART_WORDLENGTH_8B;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling = UART_OVERSAMPLING_8;
   huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
   huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
   static     uint8_t*  crcFragment;
   static     uint32_t  crc32;
   
   // as long as the buffer is accessed by the peripheral, wait here!
   while((&huart2)->gState != HAL_UART_STATE_READY);
       
   // copy data into tx output buffer
   memcpy( &txBuffer[MACDSTFIELD], buffer, length );
   
   // calculate crc32 value
   crc32 = uart_calcCRC( (uint32_t*)&txBuffer[MACDSTFIELD], (uint32_t)length );
   
   // append crc to the outputbuffer
   crcFragment = (uint8_t*)&crc32;
   
   for( uint8_t i=0, j=3; i<4; i++,j-- )
   {
      txBuffer[MACDSTFIELD+length+i] = crcFragment[j];
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
static void uart_send( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size )
{
   // generate variable allready at the beginning of program run
   static uint32_t   uart_tx_err_counter = 0;

   // start the random countdown to access the bus
   do
   {
      bus_uart_startRandomTimout();
      while( timeoutFlag != (uint8_t)1  );
      timeoutFlag = 0;
   }
   while( huart->gState != HAL_UART_STATE_READY && busIdleFlag != 1 );


   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pData, BUFFERLENGTH);
   // start transmitting in interrupt mode
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   //disable rx receive interrupt
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   // RS485 drive enable -> write to the bus (listen not anymore possible at this moment)
   HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_SET);
   // send the data
   if(HAL_UART_Transmit_DMA(huart, (uint8_t*)pData, Size) != HAL_OK )
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
static void uart_receive( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size )
{
   // wait until uart peripheral is ready
   while(huart->gState != HAL_UART_STATE_READY);
   //disable rx transmit interrupt
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   // RS485 set to listening
   HAL_GPIO_WritePin(GPIOD, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_RESET);
   // Clean & invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pData, BUFFERLENGTH);
   // start receiving in interrupt mode
   HAL_UART_Receive_DMA(huart, pData, Size);
   // enable idle line and rx interrupt
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
   __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
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
/// \brief     Error callback       
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   
   // reset errors
   if (huart->Instance->ISR & USART_ISR_ORE) // Overrun Error
   {
      huart->Instance->ICR = USART_ICR_ORECF;
   }
   if (huart->Instance->ISR & USART_ISR_NE) // Noise Error
   {
      //huart->Instance->ICR = USART_ICR_NCF;
   }
   if (huart->Instance->ISR & USART_ISR_FE) // Framing Error
   {    
      huart->Instance->ICR = USART_ICR_FECF;
   }
   
   // take action on the peripherals
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   
   // start receive irq
   uart_receive( huart, rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     Rx idle line detection callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_IdleLnCallback( UART_HandleTypeDef *huart )
{
   static uint16_t    bytesLeft;
   static uint16_t    frameSize;
   static uint8_t*    preAmblePointer;
   
   // set variables
   bytesLeft = __HAL_DMA_GET_COUNTER(huart->hdmarx);
   
   // take action on the peripherals
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   
   // get message length
   frameSize = BUFFERLENGTH - bytesLeft;
   
   // abort if input data is 0 bytes in length or too long or too short for a ethernet frame
   if( (frameSize == (uint16_t)0) || (frameSize > (uint16_t)(ETHSIZE+PREAMBLESFDLENGTH)) || (frameSize < (uint16_t)MINSIZE))
   {
      // start receive irq
      uart_receive( huart, rxBuffer, BUFFERLENGTH );
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
            // start receive irq
            uart_receive( huart, rxBuffer, BUFFERLENGTH );
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
      // start receive irq
      uart_receive( huart, rxBuffer, BUFFERLENGTH );
      colCounter++;
      return ;
   }

   // create a new node in the list, with the received data
   list_insertData( preAmblePointer, frameSize, UART_TO_ETH );
   
   // start to receive again
   uart_receive( huart, (uint8_t*)rxBuffer, BUFFERLENGTH );
}

//------------------------------------------------------------------------------
/// \brief     bus access timer initialisation   
///
/// \param     none
///
/// \return    none
static void bus_timer_init( void )
{
   // Compute the prescaler value to have TIM3 counter clock equal to 1000000 Hz = 1 us
   // (24 MHz / 2 M)-1 = 11
   // Prescaler = (TIM3CLK / TIM3 counter clock) - 1
   // TIM3CLK = 120 MHz
   // TIM3 counter clock = us = 1000000
   //uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / (2*1000000)) - 1;
   uint32_t uwPrescalerValue = (uint32_t)(120000000 / (10000000)) - 1; // div with 2 because apb1 runs on 120 MHz
   
   // clock (APB1)
   __HAL_RCC_TIM3_CLK_ENABLE();
   
   // configuration
   BusTimHandle.Instance               = TIM3;
   BusTimHandle.Init.Period            = 0; 
   BusTimHandle.Init.Prescaler         = uwPrescalerValue;
   BusTimHandle.Init.ClockDivision     = 0;
   BusTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
   BusTimHandle.Init.RepetitionCounter = 0;
   
   HAL_TIM_Base_Init(&BusTimHandle);
   
   // Set the TIM3 priority
   HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3, which sets the bus access flag.
///
/// \param     -
///
/// \return    none
static void bus_uart_startRandomTimout( void )
{
   // set a random number for the auto reload register
   TIM3->ARR = (uint32_t)(rand() % 10000)+500; // default for 10 mbit 1000+300
   // set counter value to 0
   TIM3->CNT = 0;
   // start the timer
   HAL_TIM_Base_Start_IT(&BusTimHandle);
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3, which sets the bus access flag.
///
/// \param     -
///
/// \return    none
void bus_uart_timeoutCallback( void )
{
   // set the timeout flag to 1
   timeoutFlag = 1;
   // stop the timer
   HAL_TIM_Base_Stop_IT(&BusTimHandle);
}

//------------------------------------------------------------------------------
/// \brief     Set the rx idle flag    
///
/// \param     [in] 1 = idle, 0 = not idle
///
/// \return    none
void bus_uart_setRxIdleFlag( uint8_t value )
{
   busIdleFlag = value;
}

//------------------------------------------------------------------------------
/// \brief     Get the rx idle flag    
///
/// \param     [in] 1 = idle, 0 = not idle
///
/// \return    idle flag value
uint8_t bus_uart_getRxbusIdleFlag( void )
{
   return busIdleFlag;
}