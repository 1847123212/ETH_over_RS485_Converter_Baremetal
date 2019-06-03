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
#include <stdlib.h>
#include "list.h"
#include "uart.h"

// Private defines ************************************************************
#define BYTETIMOUTUS   ( 10 )

// Private types     **********************************************************

// Private variables **********************************************************
static const uint8_t          preAmbleSFD[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};
#pragma location=0x2403B000
ALIGN_32BYTES(uint8_t         rxBuffer[BUFFERLENGTH]);
#pragma location=0x2403B700
ALIGN_32BYTES(uint8_t         txBuffer[BUFFERLENGTH]);

uint8_t                       uartBusAccessFlag;
uint8_t                       uartBusActiveFlag;
uint8_t                       txActiveFlag = 0;
uint8_t                       rxActiveFlag = 0;
TIM_HandleTypeDef             LedTimHandle;
TIM_HandleTypeDef             BusTimHandle;
TIM_HandleTypeDef             BTimeoutTimHandle;
uint32_t                      colCounter = 0;

// Global variables ***********************************************************
UART_HandleTypeDef            huart2;
DMA_HandleTypeDef             hdma_usart2_rx;
DMA_HandleTypeDef             hdma_usart2_tx;
CRC_HandleTypeDef             hcrc;
extern ETH_HandleTypeDef      heth;

// Private function prototypes ************************************************
static void      crc_init          (void);
static void      mpu_uart_config   (void);
static uint32_t  uart_calcCRC      ( uint32_t* dataPointer, uint32_t dataLength );
static void      uart_send         ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
static void      uart_receive      ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
static void      led_timer_init    ( void );
static void      bus_timer_init    ( void );
static void      led_gpio_init     ( void );
static void      setRandomWait     ( void );
static void      resetRandomWait   ( void );
static void      bus_bytetimeout_timer_init( void );

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

   // LED for Collision indication
   led_gpio_init();
   
   // Timer for Collision LED Configuration
   led_timer_init();
   
   // Timer for bus access
   bus_timer_init();
   
   // Timer for bytetimout
   bus_bytetimeout_timer_init();
   
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
   HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
   HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
   
   // set preamble in the tx buffer
   memcpy(txBuffer, preAmbleSFD, PREAMBLESFDLENGTH);
   
   uartBusAccessFlag = 1;
   uartBusActiveFlag = 0;
   
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
   MPU_InitStruct.BaseAddress = 0x2403B000;
   MPU_InitStruct.Size = MPU_REGION_SIZE_4KB;
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
static void uart_send( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size )
{
   volatile uint8_t goFlag = 0;

   // wait for the bus to become idle and send then data
   while(goFlag != 1)
   {
      // wait until uart peripheral is ready and line is idle
      while(huart->gState != HAL_UART_STATE_READY && uartBusActiveFlag != 1){}
      // wait a random short time to avoid possible collisions at the beginning
      uartBusAccessFlag = 0;
      setRandomWait();
      while(uartBusAccessFlag!=1);
      resetRandomWait();
      // check again if the bus is still idle
      if( uartBusActiveFlag == 0 )
      {
         goFlag = 1;
      }
   }
   // RS485 drive enable -> write to the bus (listen not anymore possible at this moment)
   GPIOD->BSRRL = UART_PIN_BUS_RTS|UART_PIN_BUS_CTS;
   txActiveFlag = 1;
   rxActiveFlag = 0;
   // Clean data cache
   SCB_CleanDCache_by_Addr((uint32_t*)pData, BUFFERLENGTH);
   // start transmitting in interrupt mode
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   HAL_UART_Transmit_DMA(huart, (uint8_t*)pData, Size);
}

//------------------------------------------------------------------------------
/// \brief     Function to receive data over uart
///
/// \param     -
///
/// \return    none
static void uart_receive( UART_HandleTypeDef *huart2, uint8_t *pData, uint16_t Size )
{
   // set buffer to 0
   //memset(&rxBuffer, 0, BUFFERLENGTH);
   // wait until uart peripheral is ready
   while(txActiveFlag != 0){}  
   // RS485 set to listening
   GPIOD->BSRRH = UART_PIN_BUS_RTS|UART_PIN_BUS_CTS;
   rxActiveFlag = 1;
   // enable idle line and rx interrupt
   __HAL_UART_ENABLE_IT(huart2, UART_IT_IDLE);
   __HAL_UART_ENABLE_IT(huart2, UART_IT_RXNE);
   // Clean and Invalidate data cache
   SCB_CleanDCache_by_Addr((uint32_t*)pData, BUFFERLENGTH);
   // start receiving in interrupt mode
   HAL_UART_Receive_DMA(huart2, pData, Size);
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback                   
///
/// \param     [in] UART_HandleTypeDef
///
/// \return    none
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
   // invalidate data cache (deletes data in the cache itself)
   SCB_InvalidateDCache_by_Addr((uint32_t *)txBuffer, BUFFERLENGTH);
   txActiveFlag = 0;
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
   // take action on the peripherals
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);         // disable rx interrupt
   
   // invalidate data cache (deletes data in the cache itself)
   SCB_InvalidateDCache_by_Addr((uint32_t *)rxBuffer, BUFFERLENGTH);
   
   // start receive irq
   uart_receive( huart, rxBuffer, BUFFERLENGTH );
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
      huart->Instance->ICR = USART_ICR_NCF;
   }
   if (huart->Instance->ISR & USART_ISR_FE) // Framing Error
   {    
      huart->Instance->ICR = USART_ICR_FECF;
   }
   
   // take action on the peripherals
   HAL_UART_DMAStop(huart);
   HAL_UART_Abort_IT(huart);
   
   // invalidate data cache (deletes data in the cache itself)
   SCB_InvalidateDCache_by_Addr((uint32_t *)rxBuffer, BUFFERLENGTH);
   SCB_InvalidateDCache_by_Addr((uint32_t *)txBuffer, BUFFERLENGTH);
   
   // finish tx state
   if( txActiveFlag == 1 )
   {
      txActiveFlag = 0;
   }
   
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
   //uartBusActiveFlag = 0;
   
   // invalidate data cache (deletes data in the cache itself)
   SCB_InvalidateDCache_by_Addr((uint32_t *)rxBuffer, BUFFERLENGTH);
   
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
      // set collision led
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
      // start timer to reset collision led
      HAL_TIM_Base_Start_IT(&LedTimHandle);
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
/// \brief     set uart bus access flag           
///
/// \param     none
///
/// \return    none
void uart_setUartAccessFlag( void )
{
   uartBusAccessFlag = 1;
}

//------------------------------------------------------------------------------
/// \brief     reset uart bus access flag           
///
/// \param     none
///
/// \return    none
void uart_resetUartAccessFlag( void )
{
   uartBusAccessFlag = 0;
}

//------------------------------------------------------------------------------
/// \brief     led timer initialisation   
///
/// \param     none
///
/// \return    none
static void led_timer_init( void )
{
  // Compute the prescaler value to have TIMx counter clock equal to 1000 Hz = 1 ms
  uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / (2*1000)) - 1;

  // clock
  __HAL_RCC_TIM2_CLK_ENABLE();
  
  // configuration
  LedTimHandle.Instance = TIM2;
  LedTimHandle.Init.Period            = 100-1;
  LedTimHandle.Init.Prescaler         = uwPrescalerValue;
  LedTimHandle.Init.ClockDivision     = 0;
  LedTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  LedTimHandle.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&LedTimHandle);

  // Set the TIM2 priority
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);

  // Enable the TIMx global Interrupt
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
   uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / (2*1000000)) - 1; // div with 2 because apb1 runs on 200MHz
   
   // clock
   __HAL_RCC_TIM3_CLK_ENABLE();
   
   // configuration
   BusTimHandle.Instance               = TIM3;
   BusTimHandle.Init.Period            = 0; // us
   BusTimHandle.Init.Prescaler         = uwPrescalerValue;
   BusTimHandle.Init.ClockDivision     = 0;
   BusTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
   BusTimHandle.Init.RepetitionCounter = 0;
   
   HAL_TIM_Base_Init(&BusTimHandle);
   
   // Set the TIM3 priority
   HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     led gpio initialisation   
///
/// \param     none
///
/// \return    none
static void led_gpio_init( void )
{
   // variables
   GPIO_InitTypeDef GPIO_InitStruct;
   
   // clock
   __HAL_RCC_GPIOB_CLK_ENABLE();
   
   // gpio configuration
   GPIO_InitStruct.Pin                       = GPIO_PIN_14;
   GPIO_InitStruct.Mode                      = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed                     = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//------------------------------------------------------------------------------
/// \brief     timer callback function
///
/// \param     none
///
/// \return    none
void uart_ledTimerCallback( void )
{
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
   HAL_TIM_Base_Stop_IT(&LedTimHandle);
}

//------------------------------------------------------------------------------
/// \brief     Sets random timer period
///
/// \param     none
///
/// \return    none
static void setRandomWait( void )
{
   /* Set the Autoreload value */
  TIM3->ARR = (uint32_t)(rand() % 100) + 1;
  /* set counter value to 0 */
  TIM3->CNT = 0;
  /* start the timer */
  HAL_TIM_Base_Start_IT(&BusTimHandle);
}

//------------------------------------------------------------------------------
/// \brief     Stops the timer
///
/// \param     none
///
/// \return    none
static void resetRandomWait( void )
{
   HAL_TIM_Base_Stop_IT(&BusTimHandle);
}

//------------------------------------------------------------------------------
/// \brief     bus access timer initialisation   
///
/// \param     none
///
/// \return    none
static void bus_bytetimeout_timer_init( void )
{
  // Compute the prescaler value to have TIM3 counter clock equal to 1000000 Hz = 1 us
  uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / (2*1000000)) - 1; // div with 2 because apb1 runs on 200MHz

  // clock
  __HAL_RCC_TIM5_CLK_ENABLE();
  
  // configuration
  BTimeoutTimHandle.Instance               = TIM5;
  BTimeoutTimHandle.Init.Period            = BYTETIMOUTUS; // us
  BTimeoutTimHandle.Init.Prescaler         = uwPrescalerValue;
  BTimeoutTimHandle.Init.ClockDivision     = 0;
  BTimeoutTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  BTimeoutTimHandle.Init.RepetitionCounter = 0;
  
  HAL_TIM_Base_Init(&BTimeoutTimHandle);

  // Set the TIM5 priority
  HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);

  // Enable the TIMx global Interrupt
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     timer callback function for bytetimout timeout event
///
/// \param     none
///
/// \return    none
void uart_bytetimeoutTimerCallback( void )
{
   // reset bus active flag
   uartBusActiveFlag = 0;
   // stop timer interrupt
   HAL_TIM_Base_Stop_IT(&BTimeoutTimHandle);
}

void uart_startBytetimeout( void )
{
   // set bus active flag
   uartBusActiveFlag = 1;
   // set counter value to 0
   TIM5->CNT = 0;
   // start the timer
   HAL_TIM_Base_Start_IT(&BTimeoutTimHandle);
}

void uart_resetBytetimeout( void )
{
   // set bus active flag
   uartBusActiveFlag = 1;
   // set counter value to 0
   TIM5->CNT = 0;
}

//------------------------------------------------------------------------------
/// \brief     ....        
///
/// \param     none
///
/// \return    none
void uart_customCallback( void )
{
   // no clean and invalidate data cache, it will be done in HAL_UART_IdleLnCallback
   //uartBusActiveFlag = 1;
   if( uartBusActiveFlag == 0 )
   {
      // start bytetimeout
      uart_startBytetimeout();
   }
   else
   {
      //reset bytetimeout
      uart_resetBytetimeout();
   }
}