// ****************************************************************************
/// \file      eth.c
///
/// \brief     eth module
///
/// \details   -
///
/// \author    Nico Korn
///
/// \version   0.1
///
/// \date      20190124
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
#include "buffer.h"
#include "lan8742.h"
#include "eth.h"

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

// Private types     **********************************************************

// Private variables **********************************************************

// Global variables ***********************************************************
extern UART_HandleTypeDef     huart2;
extern DMA_HandleTypeDef      hdma_usart2_rx;
extern CRC_HandleTypeDef      hcrc;

ETH_HandleTypeDef             heth;
ETH_TxPacketConfig            TxConfig; 
lan8742_Object_t              LAN8742;

// Private function prototypes ************************************************
int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);


lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                               ETH_PHY_IO_DeInit,
                               ETH_PHY_IO_WriteReg,
                               ETH_PHY_IO_ReadReg,
                               ETH_PHY_IO_GetTick};

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void eth_init( void )
{
   uint32_t idx, duplex, speed = 0;
   int32_t PHYLinkState;
   ETH_MACConfigTypeDef MACConf;
   ETH_MACFilterConfigTypeDef MACFilter;
   uint8_t MACAddr[6] ;

   heth.Instance = ETH;
   MACAddr[0] = 0x00;
   MACAddr[1] = 0x00;
   MACAddr[2] = 0x00;
   MACAddr[3] = 0x00;
   MACAddr[4] = 0x00;
   MACAddr[5] = 0x00;
   heth.Init.MACAddr = &MACAddr[0];
   heth.Init.MediaInterface = HAL_ETH_MII_MODE;
   heth.Init.TxDesc = DMATxDscrTab;
   heth.Init.RxDesc = DMARxDscrTab;
   heth.Init.RxBuffLen = 1524;
   
   /* USER CODE BEGIN MACADDRESS */
     
   /* USER CODE END MACADDRESS */
   
   if (HAL_ETH_Init(&heth) != HAL_OK)
   {
     Error_Handler();
   }
   
   memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
   TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
   TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
   TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
   TxConfig.SrcAddrCtrl = ETH_SRC_ADDR_CONTROL_DISABLE;
   
   /* Set PHY IO functions */
   LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
   
   /* Initialize the LAN8742 ETH PHY */
   LAN8742_Init(&LAN8742);
   
   PHYLinkState = LAN8742_GetLinkState(&LAN8742);
   
   /* Get link state */  
   if(PHYLinkState <= LAN8742_STATUS_LINK_DOWN)
   {
      //netif_set_link_down(netif);
      //netif_set_down(netif);
   }
   else 
   {
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
      HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(ETH_IRQn);
      HAL_ETH_Start_IT(&heth);
      //netif_set_up(netif);
      //netif_set_link_up(netif);
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
   // set mac address
   //heth.Init.MACAddr = buffer+6;
   
   // init eth with mac address
   /*
   if (HAL_ETH_Init(&heth) != HAL_OK)
   {
     Error_Handler();
   }
*/
   
   TxConfig.Length = length;
   TxConfig.TxBuffer->len = length;
   TxConfig.TxBuffer->buffer = buffer;
   
   HAL_ETH_Transmit_IT(&heth, &TxConfig);
}

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{  
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here 
  */
  
  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&heth);
  
  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief  Check the ETH link state and update netif accordingly.
  * @param  argument: netif
  * @retval None
  */
void eth_link_update( void )
{
   ETH_MACConfigTypeDef MACConf;
   int32_t PHYLinkState;
   uint32_t linkchanged = 0, speed = 0, duplex =0;
  
   PHYLinkState = LAN8742_GetLinkState(&LAN8742);
   
   if( PHYLinkState <= LAN8742_STATUS_LINK_DOWN )
   {
     HAL_ETH_Stop_IT(&heth);
     //netif_set_down(netif);
     //netif_set_link_down(netif);
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
       //netif_set_up(netif);
       //netif_set_link_up(netif);
     }
   }
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
}
