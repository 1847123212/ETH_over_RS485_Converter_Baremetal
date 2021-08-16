// ****************************************************************************
/// \file      eth.h
///
/// \brief     eth module
///
/// \details   Initialisation of ethernet peripheral and function for receving
///            and transmitting ethernet frames.
///
/// \author    Nico Korn
///
/// \version   1.0.0.2
///
/// \date      17082021
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

// Define to prevent recursive inclusion **************************************
#ifndef __ETH_H
#define __ETH_H

// Include ********************************************************************

// Exported defines ***********************************************************

// Exported types *************************************************************

// Exported functions *********************************************************
void     eth_init               ( void );
uint8_t  eth_output             ( uint8_t* buffer, uint16_t length );
void     eth_link_update        ( void );
//void     pbuf_free_custom(struct pbuf *p);
#endif // __ETH_H