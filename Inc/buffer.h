/**
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUFFER_H
#define __BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

// Exported defines ***********************************************************
#define BUFFERLENGTH    ( 1600 )

// Exported types *************************************************************
typedef struct {
   uint8_t              bufferSlot[BUFFERLENGTH];     // array with data and max length BUFFERLENGTH
   message_direction_t  messageDirection;             // direction flag: either to rs485 or ethernet
   uint16_t             dataLengthInBuffer;           // length of data in the buffer - 0 means empty
} BufferSlot;

// Exported functions *********************************************************
void            buffer_init                  ( void );
uint8_t         buffer_setNextSlotTx         ( void );
uint8_t         buffer_setNextSlotRx         ( void );
BufferSlot*     buffer_incrTxPointer         ( void );
void            buffer_incrRxPointer         ( void );
BufferSlot*     buffer_getRxPointer          ( void );
void            buffer_manager               ( void );
void            buffer_setRxIncrReq          ( void );
void            buffer_setTxIncrReq          ( void );
void            buffer_setMessageSize        ( uint16_t messagesize );
void            buffer_setMessageDirection   ( message_direction_t direction );
uint8_t*        buffer_getBufferslotPointer  ( void );
void            buffer_lock                  ( void );
void            buffer_unlock                ( void );
uint8_t         buffer_getLockStatus         ( void );
#ifdef __cplusplus
}
#endif

#endif /* __BUFFER_H */
