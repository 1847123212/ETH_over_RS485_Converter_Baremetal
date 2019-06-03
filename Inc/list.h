/**
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */

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
void            list_lock                  ( void );
void            list_unlock                ( void );
uint8_t         list_getLockStatus         ( void );
void            list_insertData            ( uint8_t* data, uint16_t dataLength, message_direction_t messageDirection );
void            list_ledTimerCallback      ( void );
#ifdef __cplusplus
}
#endif

#endif /* __LIST_H */
