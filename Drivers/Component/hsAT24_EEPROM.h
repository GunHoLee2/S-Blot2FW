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
#ifndef __HS_AT24_EEPROM_H
#define __HS_AT24_EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

   
typedef struct
{
  uint16_t wPageSize;
  uint16_t wDev_Address;
  uint16_t wMemStart_Adrress;
  uint16_t wMemAddress_Size;
  uint16_t wMemMaxSize;
}hsAT24xx_ConfigType;



void hsAT24xx_Config(hsAT24xx_ConfigType *p_tMemConfig, uint16_t wPageSize, uint16_t wDev_Address, uint16_t wMemStart_Adrress, uint16_t wMemAddress_Size, uint16_t wMemMaxSize);
int32_t hsAT24xx_Write(hsAT24xx_ConfigType *p_tMemConfig, uint8_t *p_uData, uint16_t wStartAddress, uint32_t dwWriteDataSize);
int32_t hsAT24xx_Read(hsAT24xx_ConfigType *p_tMemConfig, uint8_t *p_uData, uint16_t wStartAddress, uint32_t dwReadDataSize);


  


#ifdef __cplusplus
}
#endif


#endif /* __HS_AT24_EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
