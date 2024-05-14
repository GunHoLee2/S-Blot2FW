/**
  ******************************************************************************
  * @file    main.c 
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
Vref/4096 = LSB
3.3/4096  = 0.000805 v, Current Ref 3 Ohm 0.000805/3  = 0.00027V

1A  =  1V  = x * 0.00027 --> x = 3704 Cnt

0.7 = 2593

  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Device_MCP4725.h"
#include "hs_TRF_MCU_Config.h"


#include "hs_Common_Utill.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define DEV_COD_MCP4725         0xC


//Write Cmd
#define nCMD_WR_FAST_MD         0
#define nCMD_WR_DAC_REG         2
#define nCMD_WR_DAC_REG_EEPROM  3

// Power Down
#define nPD_NORMAL_MD           0
#define nPD_1KR_GND_MD          1
#define nPD_100KR_GND_MD        2
#define nPD_500KR_GND_MD        3



/* Private macro -------------------------------------------------------------*/
#define __HS_MCP_DAC_DELAY_us(x)   for(uint32_t tempCnt = 0; tempCnt < x ; tempCnt++){for(uint32_t tempCnt1 = 0; tempCnt1 < 24 ; tempCnt1++){asm("NOP");}} 

/* Private variables ---------------------------------------------------------*/
int32_t (*MCP4725_I2C_Write)(uint16_t, uint8_t *, uint16_t , uint32_t);
int32_t (*MCP4725_I2C_Read)(uint16_t, uint8_t *, uint16_t , uint32_t);
//int32_t BSP_TRF_MasterI2C_Read(uint16_t wDev_Address, uint8_t *p_ReadBuff, uint16_t wBuffSize, uint32_t dwTimeOut_msec);




/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/** 
  * @brief  
  * @param  None
  * @retval None
  */
void hs_MCP4725_I2C_DAC_Init(void)
{
  MCP4725_I2C_Write = BSP_TRF_MasterI2C_Write;
  MCP4725_I2C_Read = BSP_TRF_MasterI2C_Read;
}


/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_Write(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr, uint16_t wCmd, uint16_t wPw_Dn, uint16_t wData)
{
  uint8_t uBody[3] = {0};
  uint8_t uHeader = 0;
  
  p_tDAC_IC->MCP4725_DAC_BitREG.DEV_COD = DEV_COD_MCP4725;
  p_tDAC_IC->MCP4725_DAC_BitREG.ADDRESS = wDevAddr;
  p_tDAC_IC->MCP4725_DAC_BitREG.RW = 0;
  
  p_tDAC_IC->MCP4725_DAC_BitREG.CMD = wCmd;
  p_tDAC_IC->MCP4725_DAC_BitREG.PW_DN = wPw_Dn;
  p_tDAC_IC->MCP4725_DAC_BitREG.DAC_DATA = wData;
  
  uHeader = p_tDAC_IC->uMCP4725_WR_8REG[3];
  uBody[0] = p_tDAC_IC->uMCP4725_WR_8REG[2];
  uBody[1] = p_tDAC_IC->uMCP4725_WR_8REG[1];
  uBody[2] = p_tDAC_IC->uMCP4725_WR_8REG[0];
  
  MCP4725_I2C_Write(uHeader, uBody, 3, 1000);
  
  return 0;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_Read(MCP4725_WR_REG *p_tReadData, uint16_t wDevAddr)
{
  uint8_t uBody[3] = {0};
  uint8_t uHeader = 0;
  
  p_tReadData->MCP4725_DAC_BitREG.DEV_COD = DEV_COD_MCP4725;
  p_tReadData->MCP4725_DAC_BitREG.ADDRESS = wDevAddr;
  p_tReadData->MCP4725_DAC_BitREG.RW = 0;
  
  p_tReadData->MCP4725_DAC_BitREG.CMD = 0;
  p_tReadData->MCP4725_DAC_BitREG.PW_DN = 0;
  p_tReadData->MCP4725_DAC_BitREG.DAC_DATA = 0;
  
  uHeader = p_tReadData->uMCP4725_WR_8REG[3];
  
  MCP4725_I2C_Read(uHeader, uBody, 3, 1000);

  p_tReadData->uMCP4725_WR_8REG[2] = uBody[0];
  p_tReadData->uMCP4725_WR_8REG[1] = uBody[1];
  p_tReadData->uMCP4725_WR_8REG[0] = uBody[2];
  
  return 0;
}


/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_PowerDown_EEPROM(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr)
{
  hsMCP4725_Write(p_tDAC_IC, wDevAddr, nCMD_WR_DAC_REG_EEPROM, nPD_1KR_GND_MD, p_tDAC_IC->MCP4725_DAC_BitREG.DAC_DATA);
  
  return 0;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_PowerDown_Reg(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr)
{
  hsMCP4725_Write(p_tDAC_IC, wDevAddr, nCMD_WR_DAC_REG, nPD_1KR_GND_MD, p_tDAC_IC->MCP4725_DAC_BitREG.DAC_DATA);
  
  return 0;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_Power_On(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr)
{
  hsMCP4725_Write(p_tDAC_IC, wDevAddr, nCMD_WR_DAC_REG, nPD_NORMAL_MD, p_tDAC_IC->MCP4725_DAC_BitREG.DAC_DATA);
  
  return 0;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_DAC_CountSetEEPROM(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr, uint16_t wDAC_Count)
{
  hsMCP4725_Write(p_tDAC_IC, wDevAddr, nCMD_WR_DAC_REG_EEPROM, nPD_NORMAL_MD, wDAC_Count);
  
  return 0;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t hsMCP4725_DAC_CountSetReg(MCP4725_WR_REG *p_tDAC_IC, uint16_t wDevAddr, uint16_t wDAC_Count)
{
  hsMCP4725_Write(p_tDAC_IC, wDevAddr, nCMD_WR_DAC_REG, nPD_NORMAL_MD, wDAC_Count);
  
  return 0;
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
