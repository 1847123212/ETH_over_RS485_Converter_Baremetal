// ****************************************************************************
/// \file      uart.h
///
/// \brief     uart module
///
/// \details   -
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
#ifndef __PCU_BUS_UART_H
#define __PCU_BUS_UART_H

// Include ********************************************************************

// Exported defines ***********************************************************
#define MACDSTFIELD              ( 8 )
#define TOTALLENGTHLSBFIELD      ( 17 )
#define TOTALLENGTHMSBFIELD      ( 16 )
#define MACSRCADRLENGTH          ( 6 )
#define MACDSTADRLENGTH          ( 6 )
#define ETHTYPELENGTH            ( 2 )
#define PREAMBLESFDLENGTH        ( 8 )
#define CRC32LENGTH              ( 4 )
#define ETHSIZE                  ( 1522 )    // The original Ethernet IEEE 802.3 
                                             // standard defined the minimum 
                                             // Ethernet frame size as 64 bytes 
                                             // and the maximum as 1518 bytes. 
                                             // The maximum was later increased 
                                             // to 1522 bytes to allow for VLAN 
                                             // tagging. The minimum size of an 
                                             // Ethernet frame that carries an 
                                             // ICMP packet is 74 bytes.
                                             // +8 bytes for preamble & sfd
#define MINSIZE                  ( 25 )
#define TYPEARP                  ( 0806 )
#define ETHTYPEFIELDMSB          ( 12 )
#define ETHTYPEFIELDLSB          ( 13 )
#define MINPAYLOAD               ( 46 )

#define UART_PIN_BUS_RTS         GPIO_PIN_4
#define UART_PIN_BUS_CTS         GPIO_PIN_3

// Exported types *************************************************************
typedef struct BUS_UART_RX_s
{
   uint8_t        uartState;                       ///< state of the uart communication    0 = idle, 1 = busy           
   uint8_t*       buffer;                          ///< pointer to buffer
   uint16_t       bufferSize;                      ///< size to the pointed buffer
   uint16_t       frameSize;                       ///< size of the complete eth frame
   uint16_t       byteCounter;                     ///< size of the data field in the message                             
} UART_RX_t;

// Exported functions *********************************************************
void                 uart_init                        ( void );
void                 HAL_UART_IdleLnCallback          ( UART_HandleTypeDef *huart );
uint8_t              uart_output                      ( uint8_t* buffer, uint16_t length );
void                 uart_setUartAccessFlag           ( void );
void                 uart_resetUartAccessFlag         ( void );
void                 uart_ledTimerCallback            ( void );
void                 uart_startBytetimeout            ( void );
void                 uart_bytetimeoutTimerCallback    ( void );
void                 uart_customCallback              ( void );
void                 uart_resetBytetimeout            ( void );
void                 bus_uart_setRxIdleFlag           ( FlagStatus value );
FlagStatus           bus_uart_getRxIdleFlag           ( void );
void                 bus_uart_randomTimeoutCallback   ( void );
void                 bus_uart_framegapTimeoutCallback ( void );
void                 bus_uart_startFramegap           ( void );
void send( void );
#endif // __PCU_BUS_UART_H