/**

  ******************************************************************************


  ******************************************************************************


  ******************************************************************************


  ******************************************************************************
  */  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HS_BSP_MCU_CONFIG_H
#define __HS_BSP_MCU_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

//----------------------------------------------------------------
//Callback Function Index
#define nWORK_TIME_CALLBACK             0

#define nFLASHER_RX_CALLBACK            1
#define nLCD_RX_CALLBACK                2
#define nRS485_RX_CALLBACK              3
#define nUSB_RX_CALLBACK                4

#define nFLASHER_TX_CALLBACK            5
#define nLCD_TX_CALLBACK                6
#define nRS485_TX_CALLBACK              7
#define nUSB_TX_CALLBACK                8

#define nCALLBACK_n                     9
   
   
#define nHW_SYSTEM_TIMER                0

//----------------------------------------------------------------
   
   
void BSP_MCU_Init(uint32_t *p_FP);
uint32_t BSP_Version(void);
void BSP_IndcatorLED_OnOff(uint32_t dwDev_num, uint32_t dwPinState);

void BSP_System_Error(void);

void BSP_TB6617_Enable(uint32_t dwDev_num);
void BSP_TB6617_Diasble(uint32_t dwDev_num);
void BSP_TB6617_Break(uint32_t dwDev_num);
void BSP_TB6617_Stop(uint32_t dwDev_num);
void BSP_TB6617_CW(uint32_t dwDev_num);
void BSP_TB6617_CCW(uint32_t dwDev_num);

void BSP_DRV8825_StandBy(uint32_t dwDev_num);
void BSP_DRV8825_ENABLE(uint32_t dwDev_num, uint32_t dwEable);
void BSP_DRV8825_DIR(uint32_t dwDev_num, uint32_t dwCCW_nCW);
void BSP_DRV8825_DECAY(uint32_t dwDev_num, uint32_t dwFast_nLow);
void BSP_DRV8825_RESET(uint32_t dwDev_num);
void BSP_DRV8825_STEP(uint32_t dwDev_num, uint32_t dwPulse_us);
int32_t BSP_DRV8825_Fault_Check(uint32_t dwDev_num);
int32_t BSP_DRV8825_Home_Check(uint32_t dwDev_num);


int32_t BSP_AT24_I2C_Write(uint16_t wDevAddr, uint16_t wMemAddr, uint16_t wMemAddrSize, uint8_t *p_uData, uint16_t wSize, uint32_t dwTimeout);
int32_t BSP_AT24_I2C_Read(uint16_t wDevAddr, uint16_t wMemAddr, uint16_t wMemAddrSize, uint8_t *p_uData, uint16_t wSize, uint32_t dwTimeout);


void BSP_ETC_OnOff_Device(uint32_t dwDev_num, uint32_t dwPinState);
int32_t BSP_ETC_Input_Check(uint32_t dwDev_num);

void BSP_UART_GetIT_Byte(uint32_t dwDev_num, uint8_t *p_uGetData);
int32_t BSP_UART_PutIT_Byte(uint32_t dwDev_num, uint8_t *p_uPutData, uint16_t wLen);


void BSP_SystickTimerSet(uint32_t TimerBank_num);
uint32_t BSP_SystickTimerCurrentValue(uint32_t TimerBank_num);
void BSP_SystemHW_Delay_msec(uint32_t dwWaitTime_msec);
void BSP_SystemHW_DelayBreakerHandler(uint32_t dwSetting);
uint32_t BSP_SystickTimCountGet(void);


void BSP_MCU_Check_LEDOnOff(uint32_t dwOn_nOff);


unsigned char eeprom_write_test(void);
unsigned char eeprom_read_test(void);


void Time2_It_Stop();

void Time2_It_Start();
void servo_mv(unsigned int pos);
void Servo_MT_init();
void Servo_MT_stop();
void USB_IT_Disable(void);
void USB_IT_Enable(void);

extern const UART_HandleTypeDef *gp_UART_Handle[];
#ifdef __cplusplus
}
#endif


#endif /* __HS_BSP_MCU_CONFIG_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
