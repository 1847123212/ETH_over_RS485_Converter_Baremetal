// ****************************************************************************
/// \file      led.c
///
/// \brief     Led module to init and provide control functions to the status 
///            led. The status led toggles its light regulary, to show if the 
///            gateway has not stuck somewhere.
///
/// \details   -
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
    // Compute the prescaler value to have TIM4 counter clock equal to 10000 Hz = 10 us
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
   
   // Set the TIM4 priority
   HAL_NVIC_SetPriority(TIM4_IRQn, 9, 0);
   
   // Enable the TIM4 global Interrupt
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
void led_ledTimerCallback( void )
{
   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}

/********************** (C) COPYRIGHT Reichle & De-Massari *****END OF FILE****/