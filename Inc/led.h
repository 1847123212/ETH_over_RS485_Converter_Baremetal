// ****************************************************************************
/// \file      led.h
///
/// \brief     led module to init and provide controle functions to the status 
///            led
///
/// \details   -
///
/// \author    Nico Korn
///
/// \version   0.3
///
/// \date      02122019
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

// Define to prevent recursive inclusion **************************************
#ifndef __LED_H
#define __LED_H

// Include ********************************************************************

// Exported defines ***********************************************************

// Exported types *************************************************************

// Exported functions *********************************************************
void  led_ledTimerCallback   ( void );
void  led_init                ( void );

#endif // __LED_H