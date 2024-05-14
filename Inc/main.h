#ifndef	MAIN_H
#define	MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"
#include "hsBSP_MCU_Config.h"
#include "hsTB6617_PeristalticPumpDrive.h"
#include "hs_Common_Utill.h"
#include "hsAT24_EEPROM.h"
#include "17HS3410_Dspin.h"
#include "17HS3410_Dspin_Config.h"
#include "spu.h"
#include "macro.h"
#include "event.h"
#include "cntrl.h"
#include "timer.h"
#include "hsDRV88xx_StepmotorDrive.h"
#include "usb_uart.h"
#include "error.h"
#include "k_blot_pr.h"
#include "motor.h"
#include "eeprom.h"
#include "pump.h"
#include "adc.h"
#include "sq.h"
#include "pp_cal.h"
#include "QProc.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
extern void main_root();
extern void sub_root();
#define AirFan_Off   HAL_GPIO_WritePin(M1_HOME_GPIO_Port, M1_HOME_Pin, GPIO_PIN_SET)
#define AirFan_On    HAL_GPIO_WritePin(M1_HOME_GPIO_Port, M1_HOME_Pin, GPIO_PIN_RESET)

//start QUEUE
extern void setUsbTxQueue(uint32_t dwDev_num, byte *data, uint16_t length);
extern void setRxQueue(QID id, byte data);

int txProc();
int rxProc(QID id);
void tempProc();


//end QUEUE

#endif
