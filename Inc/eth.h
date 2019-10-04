// ****************************************************************************
/// \file      eth.h
///
/// \brief     eth module
///
/// \details   -
///
/// \author    Nico Korn
///
/// \author    Nico Korn
///
/// \version   0.2
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
#ifndef __ETH_H
#define __ETH_H

// Include ********************************************************************

// Exported defines ***********************************************************

// Exported types *************************************************************

// Exported functions *********************************************************
void     eth_init               ( void );
void     eth_output             ( uint8_t* buffer, uint16_t length );
void     eth_link_update        ( void );
//void     pbuf_free_custom(struct pbuf *p);
#endif // __ETH_H