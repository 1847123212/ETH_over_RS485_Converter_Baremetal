// ****************************************************************************
/// \file      list.c
///
/// \brief     list Module
///
/// \details   Module which manages the list for transmitting 
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
#include "list.h"
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
static uint32_t            mallocFailCounter;
static uint32_t            dataPacketsIN;
static uint32_t            bytesIN;
static uint32_t            dataPacketsOUT;
static uint32_t            bytesOUT;
static uint32_t            listLengthPeak;
static uint32_t            listLength;

// Private functions **********************************************************
static void led_gpio_init( void );
static void led_timer_init( void );

// Global variables ***********************************************************
TIM_HandleTypeDef    LedMallocTimHandle;

// ----------------------------------------------------------------------------
/// \brief     Buffer init
///
/// \param     none
///
/// \return    -
void list_init( void )
{
   // init dummy header
   head = malloc(sizeof(node_t));
   if (head == NULL) {
     // set the orange user led
     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
     // stay here
      while(1);
   }

   // prepare the new node
   head->data              = 0;
   head->dataLength        = 0;
   head->messageDirection  = NOT_INITIALIZED;
   head->next              = NULL;
   
   // init statistics to 0
   dataPacketsIN     = 0;
   bytesIN           = 0;
   dataPacketsOUT    = 0;
   bytesOUT          = 0;
   mallocFailCounter = 0;
   listLengthPeak    = 0;
   listLength        = 0;
   
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
void list_manager( void )
{
   static node_t *next;
   next = head->next;
   
   // check if there is data ready to send on the header node through its next member
   if( next != NULL )
   {
      // head shift
      oldhead = head;
      head = next;
      
      // release the old head
      __disable_irq();
      free(oldhead->data);
      free(oldhead);
      __enable_irq();
       
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
      // decrement list length
      dataPacketsOUT++;
      listLength--;
      bytesOUT += head->dataLength;
   }
}

// ----------------------------------------------------------------------------
/// \brief     unlocks the ringbuffer access as shared source
///
/// \param     [in] data pointer
/// \param     [in] data length
/// \param     [in] direction
///
/// \return    -
inline void list_insertData( uint8_t* data, uint16_t dataLength, message_direction_t messageDirection )
{
   // disable all interrupts, as malloc is used in an isr
   __disable_irq();
   
   // variables
   node_t *newNode   = malloc(sizeof(node_t));
   static node_t *tmp;       
   static node_t *tailNode;
   
   if( newNode != NULL )
   {
      newNode->data = malloc(dataLength*sizeof(uint8_t));
      __enable_irq();
   }
   else
   {
      // increment malloc fail counter
      mallocFailCounter++;
      // enable all interrupts again
      __enable_irq();
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
      // increment malloc fail counter
      mallocFailCounter++;
      // free allready allocated data
      free(newNode);
      // enable all interrupts again
      __enable_irq();
      return;
   }
   
   // get the tail of the list
   tmp = head;
   do 
   {
      tailNode = tmp;
      tmp = tmp->next;
   } 
   while(tmp); // tailNode has now the tail of the list where the member next is null and tmp equals null
   
   // tail node needs address of the new tailnode "newNode"
   tailNode->next = newNode;
   
   // enable all interrupts again
   //__enable_irq();
   
   // increment list length
   dataPacketsIN++;
   listLength++;
   bytesIN += dataLength;
   if( listLength > listLengthPeak )
   {
      listLengthPeak = listLength;
   }
}

//------------------------------------------------------------------------------
/// \brief     led timer initialisation   
///
/// \param     none
///
/// \return    none
static void led_timer_init( void )
{
    // Compute the prescaler value to have TIM3 counter clock equal to 10000 Hz = 10 us
    uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / (2*10000)) - 1;
   
   // clock
   __HAL_RCC_TIM4_CLK_ENABLE();
   
   // configuration
   LedMallocTimHandle.Instance               = TIM4;
   LedMallocTimHandle.Init.Period            = 9999;
   LedMallocTimHandle.Init.Prescaler         = uwPrescalerValue;
   LedMallocTimHandle.Init.ClockDivision     = 0;
   LedMallocTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
   LedMallocTimHandle.Init.RepetitionCounter = 0;
   HAL_TIM_Base_Init(&LedMallocTimHandle);
   
   // Set the TIM2 priority
   HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM4_IRQn);
  
   // temporary led it blink
   HAL_TIM_Base_Start_IT(&LedMallocTimHandle);
}

//------------------------------------------------------------------------------
/// \brief     led gpio initialisation for firmware operation led  
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
   GPIO_InitStruct.Pin                       = GPIO_PIN_0;
   GPIO_InitStruct.Mode                      = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed                     = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//------------------------------------------------------------------------------
/// \brief     firmware operation led callback
///
/// \param     none
///
/// \return    none
void list_ledTimerCallback( void )
{
   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}