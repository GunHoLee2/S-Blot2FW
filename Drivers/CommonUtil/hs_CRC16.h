/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  *
  ******************************************************************************
  */  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HS_CRC16_H
#define __HS_CRC16_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

// Device include -----------------------------------------------------------
//#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal.h"
//---------------------------------------------------------------------------
      


   

   

   

   


uint16_t wHsCRC16_GetPacket(uint8_t * p_uPacket, uint16_t wPacket_Len);
uint16_t wHsCRC16_PutPacket(uint8_t * p_uPacket, uint16_t wPacket_Len);

#ifdef __cplusplus
}
#endif


#endif /* __HS_CRC16_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
