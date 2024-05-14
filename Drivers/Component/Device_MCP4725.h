/**
  ******************************************************************************
  * @file    main.h 
  * @author  KMAC Electronic Controll Team
  * @version V1.0.0
  * @date    2014 10 14
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEV_MCP4725_H
#define __DEV_MCP4725_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include <stdio.h>



/* Exported types ------------------------------------------------------------*/
typedef union
{
  struct
  {

    __IO uint16_t   RESERV_1     :4; //Low
    __IO uint16_t   DAC_DATA     :12;
    
    __IO uint16_t   RESERV_2     :1;    
    __IO uint16_t   PW_DN        :2;
    __IO uint16_t   RESERV_3     :2;
    __IO uint16_t   CMD          :3;
    
    __IO uint16_t   RW           :1;
    __IO uint16_t   ADDRESS      :3;
    __IO uint16_t   DEV_COD      :4; //High
    

  }MCP4725_DAC_BitREG;
  __IO uint8_t uMCP4725_WR_8REG[4];
  __IO uint32_t dwMCP4725_WR_32REG;

}MCP4725_WR_REG;


typedef union
{
  struct
  {

    __IO uint16_t   RESERV_1     :4; //Low
    __IO uint16_t   DAC_DATA     :12;
    
    __IO uint16_t   RESERV_2     :1;    
    __IO uint16_t   PW_DN        :2;
    __IO uint16_t   RESERV_3     :3;
    __IO uint16_t   POR          :1;
    __IO uint16_t   RDY_BSY      :1;
    
    
    __IO uint16_t   RW           :1;
    __IO uint16_t   ADDRESS      :3;
    __IO uint16_t   DEV_COD      :4; //High
    

  }MCP4725_DAC_BitREG;
  __IO uint8_t uMCP4725_RD_8REG[4];
  __IO uint32_t dwMCP4725_RD_32REG;

}MCP4725_RD_REG;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void hs_MCP4725_I2C_DAC_Init(void);
int32_t hsMCP4725_PowerDown_EEPROM(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr);
int32_t hsMCP4725_PowerDown_Reg(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr);

int32_t hsMCP4725_Power_On(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr);

int32_t hsMCP4725_DAC_CountSetEEPROM(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr, uint16_t wDAC_Count);
int32_t hsMCP4725_DAC_CountSetReg(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr, uint16_t wDAC_Count);



#endif /* __DEV_MCP4725_H */

/* Includes ------------------------------------------------------------------*/
