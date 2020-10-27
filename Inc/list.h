// ****************************************************************************
/// \file      list.h
///
/// \brief     list Module
///
/// \details   Module which manages the list for transmitting 
///            data between UART (RS485) and ETH. 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIST_H
#define __LIST_H

#ifdef __cplusplus
extern "C" {
#endif

// Exported defines ***********************************************************
#define BUFFERLENGTH                       ( 1600 )

// Exported types *************************************************************

// Exported functions *********************************************************
void            list_init                  ( void );
void            list_manager               ( void );
void            list_insertData            ( uint8_t* data, uint16_t dataLength, message_direction_t messageDirection );
#ifdef __cplusplus
}
#endif

#endif /* __LIST_H */
