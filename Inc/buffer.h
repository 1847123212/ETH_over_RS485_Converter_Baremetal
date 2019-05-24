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
   uint16_t             bytesToSend;                  // length of data in the buffer - 0 means empty
   uint32_t             tick;                         // system tick
} BufferSlot;

// Exported functions *********************************************************
void            buffer_init                  ( void );
void            buffer_manager               ( void );
void            buffer_lock                  ( void );
void            buffer_unlock                ( void );
uint8_t         buffer_getLockStatus         ( void );
void            buffer_insertData            ( uint8_t* data, uint16_t dataLength, message_direction_t messageDirection );
void            buffer_ledTimerCallback      ( void );
#ifdef __cplusplus
}
#endif

#endif /* __BUFFER_H */
