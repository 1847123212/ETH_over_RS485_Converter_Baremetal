// ****************************************************************************
/// \file      eth.c
///
/// \brief     eth module
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
#include <string.h>
#include <stdio.h>
#include "list.h"
#include "lan8742.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "eth.h"

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];     /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];     /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */
#pragma location=0x30044000
uint8_t Tx_Buff[ETH_TX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Transmit Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

//struct pbuf_custom rx_pbuf[ETH_RX_DESC_CNT];
uint32_t current_pbuf_idx =0;

#define ETH_MAC_ADDR0           ((uint8_t)0x02)
#define ETH_MAC_ADDR1           ((uint8_t)0x00)
#define ETH_MAC_ADDR2           ((uint8_t)0x00)
#define ETH_MAC_ADDR3           ((uint8_t)0x00)
#define ETH_MAC_ADDR4           ((uint8_t)0x00)
#define ETH_MAC_ADDR5           ((uint8_t)0x00)
#define ETH_RX_BUFFER_SIZE      (1536UL)
#define ETH_TX_BUFFER_SIZE      (1536UL)

// Private types     **********************************************************

// Private variables **********************************************************
static   uint8_t  macaddress[6] = {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};

// Global variables ***********************************************************
extern UART_HandleTypeDef     huart2;
extern DMA_HandleTypeDef      hdma_usart2_rx;
extern CRC_HandleTypeDef      hcrc;

ETH_HandleTypeDef             heth;
ETH_TxPacketConfig            TxConfig; 
lan8742_Object_t              LAN8742;

// Private function prototypes ************************************************
int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit(void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);
static void mpu_eth_config(void);


lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                               ETH_PHY_IO_DeInit,
                               ETH_PHY_IO_WriteReg,
                               ETH_PHY_IO_ReadReg,
                               ETH_PHY_IO_GetTick};


//------------------------------------------------------------------------------
/// \brief     Configure the MPU attributes 
///
/// \param     none
///
/// \return    none
static void mpu_eth_config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as Device not cacheable 
     for ETH DMA descriptors */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Configure the MPU attributes as Normal Non Cacheable
     for LwIP RAM heap which contains the Tx buffers */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

//------------------------------------------------------------------------------
/// \brief     ETH Initialization Function
///
/// \param     none
///
/// \return    none
void eth_init( void )
{
   uint32_t idx, duplex, speed = 0;
   int32_t PHYLinkState;
   ETH_MACConfigTypeDef MACConf;
   ETH_MACFilterConfigTypeDef MACFilter;
   
   mpu_eth_config();
  
   heth.Instance              = ETH;  
   heth.Init.MACAddr          = macaddress;
   heth.Init.MediaInterface   = HAL_ETH_RMII_MODE;
   heth.Init.RxDesc           = DMARxDscrTab;
   heth.Init.TxDesc           = DMATxDscrTab;
   heth.Init.RxBuffLen        = ETH_RX_BUFFER_SIZE;
   
   //tx descriptor
   // testwise set backupaddress
   heth.Init.TxDesc->BackupAddr0 = (uint32_t)&Tx_Buff;
   heth.Init.TxDesc->BackupAddr1 = (uint32_t)&Tx_Buff;
   
   if (HAL_ETH_Init(&heth) != HAL_OK)
   {
     Error_Handler();
   }
   
   for(idx = 0; idx < ETH_RX_DESC_CNT; idx ++)
   {
      HAL_ETH_DescAssignMemory(&heth, idx, Rx_Buff[idx], NULL);
   }
   
   memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
   TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
   TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
   TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
   
   /* Set PHY IO functions */
   LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
   
   /* Initialize the LAN8742 ETH PHY */
   LAN8742_Init(&LAN8742);
   //LAN8742_SetLinkState(&LAN8742, LAN8742_STATUS_10MBITS_FULLDUPLEX);
   //HAL_Delay(3000);
   PHYLinkState = LAN8742_GetLinkState(&LAN8742);
   
   /* Get link state */  
   //if(PHYLinkState <= LAN8742_STATUS_LINK_DOWN)
   //{
   //}
   //else 
   //{
      switch (PHYLinkState)
      {
      case LAN8742_STATUS_100MBITS_FULLDUPLEX:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        break;
      case LAN8742_STATUS_100MBITS_HALFDUPLEX:
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        break;
      case LAN8742_STATUS_10MBITS_FULLDUPLEX:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        break;
      case LAN8742_STATUS_10MBITS_HALFDUPLEX:
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        break;
      default:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        break;      
      }
      
      /* Get MAC Config MAC */
      HAL_ETH_GetMACConfig(&heth, &MACConf); 
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_ETH_SetMACConfig(&heth, &MACConf);
      
      /* Get MAC filter Config MAC */
      HAL_ETH_GetMACFilterConfig(&heth, &MACFilter);
      MACFilter.ReceiveAllMode = ENABLE;
      HAL_ETH_SetMACFilterConfig(&heth, &MACFilter);
      
      /* start eth reception */
      HAL_ETH_Start_IT(&heth);
   //}
}

//------------------------------------------------------------------------------
/// \brief     ETH MSP Initialization
///
/// \param     [in]  ETH handle pointer
///
/// \return    none
void HAL_ETH_MspInit(ETH_HandleTypeDef* heth)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Ethernett MSP init: RMII Mode */
  
  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

/* Ethernet pins configuration ************************************************/
  /*
        RMII_REF_CLK ----------------------> PA1
        RMII_MDIO -------------------------> PA2
        RMII_MDC --------------------------> PC1
        RMII_MII_CRS_DV -------------------> PA7
        RMII_MII_RXD0 ---------------------> PC4
        RMII_MII_RXD1 ---------------------> PC5
        RMII_MII_RXER ---------------------> PG2
        RMII_MII_TX_EN --------------------> PG11
        RMII_MII_TXD0 ---------------------> PG13
        RMII_MII_TXD1 ---------------------> PB13
  */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Speed       = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode        = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull        = GPIO_NOPULL; 
  GPIO_InitStructure.Alternate   = GPIO_AF11_ETH;
  GPIO_InitStructure.Pin         = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure PB13 */
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PG2, PG11, PG13 and PG14 */
  GPIO_InitStructure.Pin =  GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);	
  
  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
  
  /* Enable Ethernet clocks */
  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();
}

//------------------------------------------------------------------------------
/// \brief     ETH MSP De-Initialization 
///
/// \param     [in]  ETH handle pointer
///
/// \return    none
void HAL_ETH_MspDeInit(ETH_HandleTypeDef* heth)
{
  if(heth->Instance==ETH)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1RX_CLK_DISABLE();
  
    /**ETH GPIO Configuration    
    PE2     ------> ETH_TXD3
    PC1     ------> ETH_MDC
    PC2_C     ------> ETH_TXD2
    PC3_C     ------> ETH_TX_CLK
    PA0     ------> ETH_CRS
    PA1     ------> ETH_RX_CLK
    PA2     ------> ETH_MDIO
    PA3     ------> ETH_COL
    PA7     ------> ETH_RX_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB0     ------> ETH_RXD2
    PB1     ------> ETH_RXD3
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13);
  }
}

//------------------------------------------------------------------------------
/// \brief      send the message to the eth peripherals                 
///
/// \param     [in]  BufferSlot
///
/// \return    none
void eth_output( uint8_t* buffer, uint16_t length )
{   
   static ETH_BufferTypeDef   Txbuffer;
   static uint32_t            eth_tx_err_counter = 0;
   
   //while(heth.gState != HAL_ETH_STATE_READY);
   
   // Clean and Invalidate data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t*)Tx_Buff, (ETH_TX_DESC_CNT*ETH_TX_BUFFER_SIZE));

   length = length-PREAMBLESFDLENGTH-CRC32LENGTH;
   buffer = buffer+PREAMBLESFDLENGTH;

   memcpy((void*)Tx_Buff, buffer, length);   
   
   Txbuffer.buffer = (uint8_t*)Tx_Buff;
   Txbuffer.len = length;
   Txbuffer.next = NULL;
   
   TxConfig.Length = length;
   TxConfig.TxBuffer = &Txbuffer;
   
   // set mac address
   macaddress[0] = *(buffer+MACDSTADRLENGTH);
   macaddress[1] = *(buffer+MACDSTADRLENGTH+1);
   macaddress[2] = *(buffer+MACDSTADRLENGTH+2);
   macaddress[3] = *(buffer+MACDSTADRLENGTH+3);
   macaddress[4] = *(buffer+MACDSTADRLENGTH+4);
   macaddress[5] = *(buffer+MACDSTADRLENGTH+5);
   heth.Init.MACAddr = macaddress;
   
   // Set MAC addr bits 32 to 47
   heth.Instance->MACA0HR = ((heth.Init.MACAddr[5] << 8) | heth.Init.MACAddr[4]);
   
   // Set MAC addr bits 0 to 31
   heth.Instance->MACA0LR = ((heth.Init.MACAddr[3] << 24) | (heth.Init.MACAddr[2] << 16) | (heth.Init.MACAddr[1] << 8) | heth.Init.MACAddr[0]);
   
   // send the data
   if(HAL_ETH_Transmit_IT(&heth, &TxConfig) != HAL_OK)
   {
      eth_tx_err_counter++;
   }
}

//------------------------------------------------------------------------------
/// \brief     Initializes the MDIO interface GPIO and clocks.
///
/// \param     none
///
/// \return    0 if OK -1 if Error
int32_t ETH_PHY_IO_Init(void)
{  
   // Configure the MDIO Clock
   HAL_ETH_SetMDIOClockRange(&heth);
  
   return 0;
}

//------------------------------------------------------------------------------
/// \brief     De-Initializes the MDIO interface .       
///
/// \param     none
///
/// \return    0 if OK -1 if Error
int32_t ETH_PHY_IO_DeInit (void)
{
   return 0;
}

//------------------------------------------------------------------------------
/// \brief     Read a PHY register through the MDIO interface.        
///
/// \param     [in]  DevAddr: PHY port address
/// \param     [in]  RegAddr: PHY register address
/// \param     [in]  pRegVal: pointer to hold the register value 
///
/// \return    0 if OK -1 if Error
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
   if(HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
   {
      return -1;
   }
  
   return 0;
}

//------------------------------------------------------------------------------
/// \brief     Write a value to a PHY register through the MDIO interface.         
///
/// \param     [in]  DevAddr: PHY port address
/// \param     [in]  RegAddr: PHY register address
/// \param     [in]  RegVal: Value to be written 
///
/// \return    0 if OK -1 if Error
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
   if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
   {
      return -1;
   }
  
   return 0;
}

//------------------------------------------------------------------------------
/// \brief     Get the time in millisecons used for internal PHY driver process.                 
///
/// \param     none
///
/// \return    Time value
int32_t ETH_PHY_IO_GetTick(void)
{
   return HAL_GetTick();
}

//------------------------------------------------------------------------------
/// \brief     Check the ETH link state and update netif accordingly.                 
///
/// \param     none
///
/// \return    none
void eth_link_update( void )
{
   ETH_MACConfigTypeDef MACConf;
   int32_t PHYLinkState;
   uint32_t linkchanged = 0, speed = 0, duplex =0;
  
   PHYLinkState = LAN8742_GetLinkState(&LAN8742);
   
   if( PHYLinkState <= LAN8742_STATUS_LINK_DOWN )
   {
     HAL_ETH_Stop_IT(&heth);
   }
   else if( PHYLinkState > LAN8742_STATUS_LINK_DOWN )
   {
     switch (PHYLinkState)
     {
     case LAN8742_STATUS_100MBITS_FULLDUPLEX:
       duplex = ETH_FULLDUPLEX_MODE;
       speed = ETH_SPEED_100M;
       linkchanged = 1;
       break;
     case LAN8742_STATUS_100MBITS_HALFDUPLEX:
       duplex = ETH_HALFDUPLEX_MODE;
       speed = ETH_SPEED_100M;
       linkchanged = 1;
       break;
     case LAN8742_STATUS_10MBITS_FULLDUPLEX:
       duplex = ETH_FULLDUPLEX_MODE;
       speed = ETH_SPEED_10M;
       linkchanged = 1;
       break;
     case LAN8742_STATUS_10MBITS_HALFDUPLEX:
       duplex = ETH_HALFDUPLEX_MODE;
       speed = ETH_SPEED_10M;
       linkchanged = 1;
       break;
     default:
       break;      
     }
     
     if(linkchanged)
     {
       /* Get MAC Config MAC */
       HAL_ETH_GetMACConfig(&heth, &MACConf); 
       MACConf.DuplexMode = duplex;
       MACConf.Speed = speed;
       HAL_ETH_SetMACConfig(&heth, &MACConf);
       HAL_ETH_Start_IT(&heth);
     }
   }
}

//------------------------------------------------------------------------------
/// \brief     Receive complete callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
   static ETH_BufferTypeDef RxBuff;

   // invalidate and clean data cache
   SCB_CleanInvalidateDCache_by_Addr((uint32_t *)Rx_Buff, (ETH_RX_DESC_CNT*ETH_RX_BUFFER_SIZE));
   
   // get data from the buffer
   HAL_ETH_GetRxDataBuffer(heth, &RxBuff);
   
   // create a new node in the list, with the received data
   list_insertData( RxBuff.buffer, RxBuff.len, ETH_TO_UART );

   // set the RX descriptor for next receive
   HAL_ETH_BuildRxDescriptors(heth);
}

//------------------------------------------------------------------------------
/// \brief     Transfer complete callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
}

//------------------------------------------------------------------------------
/// \brief     DMA error callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_DMAErrorCallback(ETH_HandleTypeDef *heth)
{
}

//------------------------------------------------------------------------------
/// \brief     MAC error callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_MACErrorCallback(ETH_HandleTypeDef *heth)
{
}

//------------------------------------------------------------------------------
/// \brief     PMT callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_PMTCallback(ETH_HandleTypeDef *heth)
{
}

//------------------------------------------------------------------------------
/// \brief     EEE callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_EEECallback(ETH_HandleTypeDef *heth)
{
}

//------------------------------------------------------------------------------
/// \brief     Wakeup callback                   
///
/// \param     [in] ETH_HandleTypeDef
///
/// \return    none
void HAL_ETH_WakeUpCallback(ETH_HandleTypeDef *heth)
{
}

/********************** (C) COPYRIGHT Reichle & De-Massari *****END OF FILE****/