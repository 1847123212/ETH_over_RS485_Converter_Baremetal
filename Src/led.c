// ****************************************************************************
/// \file      led.c
///
/// \brief     Led module to init and provide control functions to the status 
///            led. The status led toggles its light regulary, 
///
/// \details   -
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

// Global variables ***********************************************************
TIM_HandleTypeDef    LedTimHandle;

// Private types     **********************************************************

// Private variables **********************************************************

// Private function prototypes ************************************************
static void led_timer_init    ( void );
static void led_gpio_init     ( void );


//------------------------------------------------------------------------------
/// \brief     init function for the toggling status led
///
/// \param     none
///
/// \return    none
void led_init( void )
{
   led_gpio_init();
   led_timer_init();
   
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
   LedTimHandle.Instance               = TIM4;
   LedTimHandle.Init.Period            = 9999;
   LedTimHandle.Init.Prescaler         = uwPrescalerValue;
   LedTimHandle.Init.ClockDivision     = 0;
   LedTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
   LedTimHandle.Init.RepetitionCounter = 0;
   HAL_TIM_Base_Init(&LedTimHandle);
   
   // Set the TIM2 priority
   HAL_NVIC_SetPriority(TIM4_IRQn, 9, 0);
   
   // Enable the TIMx global Interrupt
   HAL_NVIC_EnableIRQ(TIM4_IRQn);
  
   // temporary led it blink
   HAL_TIM_Base_Start_IT(&LedTimHandle);
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

/********************** (C) COPYRIGHT Reichle & De-Massari *****END OF FILE****/