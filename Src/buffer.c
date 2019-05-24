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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Private define *************************************************************

// Private types     **********************************************************
typedef struct node {
    uint8_t*               data;
    uint16_t               dataLength;
    message_direction_t    messageDirection;   
    struct node *          next;
} node_t;

// Private variables **********************************************************
static node_t              *head;
static node_t              *oldhead;
static uint32_t            listLength;
static uint32_t            mallocFailCounter;
static uint8_t             sourceLockFlag;
static node_t              *nextNode;

// Private functions **********************************************************
static void led_gpio_init( void );
static void led_timer_init( void );

// Global variables ***********************************************************
extern UART_HandleTypeDef  huart2;
TIM_HandleTypeDef    LedMallocTimHandle;

// ----------------------------------------------------------------------------
/// \brief     Buffer init
///
/// \param     none
///
/// \return    -
void buffer_init( void )
{
   // reset the source lock flag
   sourceLockFlag = 0;
   
   // init dummy header
   head = malloc(sizeof(node_t));
   if (head == NULL) {
      while(1);
   }

   // prepare the new node
   head->data              = 0;
   head->dataLength        = 0;
   head->messageDirection  = 0;
   head->next              = NULL;
   
   listLength = 0;
   
   // init malloc fail led
   led_timer_init();
   led_gpio_init();
}

// ----------------------------------------------------------------------------
/// \brief     Buffermanager checks if there is something in the ringbuffer
///            to send and gives the command to do so if necessary
///
/// \param     none
///
/// \return    -
void buffer_manager( void )
{
   nextNode = head->next;
   
   // check if there is data ready to send on the header node through its next member
   if( nextNode != NULL )
   {
      // head shift!
      oldhead = head;
      head = head->next;
      // release the old head
      free(oldhead->data);
      free(oldhead);
      
      // decrement list length
      listLength--;
      
      // if toETHFlag is one, it means that the current pointed message is dedicadet for the eth interface
      if( head->messageDirection == UART_TO_ETH )
      {
         // setup the next frame pointed by the tx pointer and send it hrough the eth interface
         eth_output( head->data, head->dataLength );
      }
      else if( head->messageDirection == ETH_TO_UART )
      {
         // setup the next frame pointed by the tx pointer and send it hrough the uart interface
         uart_output( head->data, head->dataLength );
      }
   }
}

// ----------------------------------------------------------------------------
/// \brief     locks the ringbuffer access as shared source
///
/// \param     none
///
/// \return    -
void buffer_lock( void )
{
   sourceLockFlag = 1;
}

// ----------------------------------------------------------------------------
/// \brief     unlocks the ringbuffer access as shared source
///
/// \param     none
///
/// \return    -
void buffer_unlock( void )
{
   sourceLockFlag = 0;
}

// ----------------------------------------------------------------------------
/// \brief     unlocks the ringbuffer access as shared source
///
/// \param     none
///
/// \return    -
uint8_t buffer_getLockStatus( void )
{
   return sourceLockFlag;
}

// ----------------------------------------------------------------------------
/// \brief     unlocks the ringbuffer access as shared source
///
/// \param     [in] data pointer
/// \param     [in] data length
/// \param     [in] direction
///
/// \return    -
void buffer_insertData( uint8_t* data, uint16_t dataLength, message_direction_t messageDirection )
{
   node_t         *newNode   = malloc(sizeof(node_t));
   node_t         *tmp       = head;
   node_t         *tailNode;
   
   if( newNode != NULL )
   {
      newNode->data = malloc(dataLength*sizeof(uint8_t));
   }
   else
   {
      // set collision led
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
      // start timer to reset collision led
      HAL_TIM_Base_Start_IT(&LedMallocTimHandle);
      mallocFailCounter++;
      return;
   }
   

   // check if allocation was successfull
   if( newNode->data != NULL )
   {
      // copy data to the allocated heap
      memcpy((void*)(newNode->data), (void const*)data, dataLength);
      newNode->dataLength        = dataLength;
      newNode->messageDirection  = messageDirection;
      newNode->next              = NULL;
   }
   else
   {
      // set collision led
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
      // start timer to reset collision led
      HAL_TIM_Base_Start_IT(&LedMallocTimHandle);
      mallocFailCounter++;
      return;
   }
   
   // get the tail of the list
	do 
   {
		tailNode = tmp;
		tmp = tmp->next;
	} 
   while (tmp); // tailNode has now the tail of the list where the member next is null and tmp equals null
   
   // tail node needs address of the new tailnode "newNode"
   // old_node
   tailNode->next         = newNode;

   // increment list length
   listLength++;
}

//------------------------------------------------------------------------------
/// \brief     bus access timer initialisation   
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
  LedMallocTimHandle.Instance = TIM4;
  LedMallocTimHandle.Init.Period            = 100-1;
  LedMallocTimHandle.Init.Prescaler         = uwPrescalerValue;
  LedMallocTimHandle.Init.ClockDivision     = 0;
  LedMallocTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  LedMallocTimHandle.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&LedMallocTimHandle);

  // Set the TIM2 priority
  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);

  // Enable the TIMx global Interrupt
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
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
   __HAL_RCC_GPIOE_CLK_ENABLE();
   
   // gpio configuration
   GPIO_InitStruct.Pin                       = GPIO_PIN_1;
   GPIO_InitStruct.Mode                      = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed                     = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

//------------------------------------------------------------------------------
/// \brief     timer callback function
///
/// \param     none
///
/// \return    none
void buffer_ledTimerCallback( void )
{
   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
   HAL_TIM_Base_Stop_IT(&LedMallocTimHandle);
}