/**
  ******************************************************************************
  * @file    dspin.h 
  * @author  IPC Rennes
  * @version V2.0
  * @date    October 4, 2013
  * @brief   Header for dspin.c module
  * @note    (C) COPYRIGHT 2013 STMicroelectronics
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DSPIN_H
#define __DSPIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "17HS3410_Dspin_Config.h"

/* Exported constants --------------------------------------------------------*/

/* Select one of the following two evalboard options, comment the other one --*/
/* #define STEVAL_PCC009V2 */
/* #define STM32_VL_Discovery */
 #define ST_DSPIN_6470H_DISCOVERY

/** @defgroup  dSPIN FW library interface
  *
  * {
  */

#define FALSE (0)
#define TRUE  (1)


#ifdef STEVAL_PCC009V2
	#define dSPIN_SPI			SPI2
	
	#define dSPIN_SCK_Pin		GPIO_Pin_13
	#define dSPIN_SCK_Port		GPIOB
	
	#define dSPIN_MOSI_Pin		GPIO_Pin_15
	#define dSPIN_MOSI_Port		GPIOB
	
	#define dSPIN_MISO_Pin		GPIO_Pin_14
	#define dSPIN_MISO_Port		GPIOB
	
	#define dSPIN_nSS_Pin		GPIO_Pin_12
	#define dSPIN_nSS_Port		GPIOB
	
	#define dSPIN_BUSY_Pin		GPIO_Pin_10
	#define dSPIN_BUSY_Port		GPIOB
	
	#define dSPIN_FLAG_Pin		GPIO_Pin_11
	#define dSPIN_FLAG_Port		GPIOB
	
	#define dSPIN_PWM1_Pin		GPIO_Pin_0
	#define dSPIN_PWM1_Port		GPIOB
	
	#define dSPIN_PWM2_Pin		GPIO_Pin_1
	#define dSPIN_PWM2_Port		GPIOB

	#define POWER_LED_Pin		GPIO_Pin_4
	#define POWER_LED_Port		GPIOC
    
        #define STATUS_LED_Pin		GPIO_Pin_8
	#define STATUS_LED_Port		GPIOC

	#define dSPIN_STBY_RESET_Pin    GPIO_Pin_1
	#define dSPIN_STBY_RESET_Port   GPIOB
    
        #define TIM_PWM                 TIM3
	
	/* List all the peripherals, which CLKs have to be enabled! */
	#define dSPIN_PERIPHERAL_CLKs_APB1	(RCC_APB1Periph_SPI2|RCC_APB1Periph_TIM3)
	#define dSPIN_PERIPHERAL_CLKs_APB2	(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO)
        /* Note : RCC_APB2Periph_AFIO is mandatory for interrupt enabling */
#endif

        /* dSPIN maximum number of bytes of command and arguments to set a parameter */
        #define dSPIN_CMD_ARG_MAX_NB_BYTES (4)
        /* dSPIN command + argument bytes number */
        #define dSPIN_CMD_ARG_NB_BYTES_NOP              (1)
        #define dSPIN_CMD_ARG_NB_BYTES_RUN              (4)
        #define dSPIN_CMD_ARG_NB_BYTES_STEP_CLOCK       (1)
        #define dSPIN_CMD_ARG_NB_BYTES_MOVE             (4)
        #define dSPIN_CMD_ARG_NB_BYTES_GO_TO            (4)
        #define dSPIN_CMD_ARG_NB_BYTES_GO_TO_DIR        (4)
        #define dSPIN_CMD_ARG_NB_BYTES_GO_UNTIL         (4)
        #define dSPIN_CMD_ARG_NB_BYTES_RELEASE_SW       (1)
        #define dSPIN_CMD_ARG_NB_BYTES_GO_HOME          (1)
        #define dSPIN_CMD_ARG_NB_BYTES_GO_MARK          (1)
        #define dSPIN_CMD_ARG_NB_BYTES_RESET_POS        (1)
        #define dSPIN_CMD_ARG_NB_BYTES_RESET_DEVICE     (1)
        #define dSPIN_CMD_ARG_NB_BYTES_SOFT_STOP        (1)
        #define dSPIN_CMD_ARG_NB_BYTES_HARD_STOP        (1)
        #define dSPIN_CMD_ARG_NB_BYTES_SOFT_HIZ         (1)
        #define dSPIN_CMD_ARG_NB_BYTES_HARD_HIZ         (1)
        #define dSPIN_CMD_ARG_NB_BYTES_GET_STATUS       (1)
        /* dSPIN response bytes number */
        #define dSPIN_RSP_NB_BYTES_GET_STATUS           (2)    
        /* Daisy chain command mask */
        #define DAISY_CHAIN_COMMAND_MASK (0xFA)

/**  dSPIN signals - Used for connection with STM32 discovery kit
  *  If another control board is used, please change the following settings
  *  according to the application board wiring diagram.
  *  Do not touch if STEVAL-PCC009V2 is used, see defs below for that evalboard.
  * 
  */
#ifdef STM32_VL_Discovery
	#define dSPIN_SPI			SPI1
	
	#define dSPIN_SCK_Pin		GPIO_Pin_5
	#define dSPIN_SCK_Port		GPIOA
	
	#define dSPIN_MOSI_Pin		GPIO_Pin_7
	#define dSPIN_MOSI_Port		GPIOA
	
	#define dSPIN_MISO_Pin		GPIO_Pin_6
	#define dSPIN_MISO_Port		GPIOA
	
	#define dSPIN_nSS_Pin		GPIO_Pin_4
	#define dSPIN_nSS_Port		GPIOA
	
	#define dSPIN_BUSY_Pin		GPIO_Pin_4
	#define dSPIN_BUSY_Port		GPIOC
	
	#define dSPIN_FLAG_Pin		GPIO_Pin_5
	#define dSPIN_FLAG_Port		GPIOC
	
	#define dSPIN_PWM1_Pin		GPIO_Pin_1
	#define dSPIN_PWM1_Port		GPIOA
	
	#define dSPIN_PWM2_Pin		GPIO_Pin_0
	#define dSPIN_PWM2_Port		GPIOB
	
	/* List all the peripherals, which CLKs have to be enabled! */
	#define dSPIN_PERIPHERAL_CLKs_APB1	(0x00)
	#define dSPIN_PERIPHERAL_CLKs_APB2	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1)
#endif

/**  dSPIN signals - Used for connection with ST_DSPIN_6470H_DISCOVERY 
  */

//#ifdef ST_DSPIN_6470H_DISCOVERY
//	#define dSPIN_SPI		SPI1
//	
//	#define dSPIN_SCK_Pin		GPIO_Pin_5
//	#define dSPIN_SCK_Port		GPIOA
//	
//	#define dSPIN_MOSI_Pin		GPIO_Pin_7
//	#define dSPIN_MOSI_Port		GPIOA
//	
//	#define dSPIN_MISO_Pin		GPIO_Pin_6
//	#define dSPIN_MISO_Port		GPIOA
//	
//	#define dSPIN_nSS_Pin		GPIO_Pin_4
//	#define dSPIN_nSS_Port		GPIOA
//	
//	#define dSPIN_BUSY_Pin		GPIO_Pin_11
//	#define dSPIN_BUSY_Port		GPIOB
//
//	#define dSPIN_FLAG_Pin		GPIO_Pin_10
//	#define dSPIN_FLAG_Port		GPIOB
//
//        /* LED_READY, GREEN*/
//	#define POWER_LED_Pin		GPIO_Pin_3
//	#define POWER_LED_Port		GPIOC
//
//        /* LED_BUSY, ORANGE */
//        #define LED_BUSY_Pin		GPIO_Pin_2
//	#define LED_BUSY_Port		GPIOC
//
//        /* LED_ERROR, RED */
//	#define STATUS_LED_Pin		GPIO_Pin_1
//	#define STATUS_LED_Port		GPIOC
//
//        /* LED_SPARE, YELLOW */
//	#define LED_SPARE_Pin		GPIO_Pin_0
//	#define LED_SPARE_Port		GPIOC
//
//	#define dSPIN_PWM1_Pin		GPIO_Pin_3
//	#define dSPIN_PWM1_Port		GPIOA
//
//	#define SW_MOTOR_Pin            GPIO_Pin_0
//	#define SW_MOTOR_Port	        GPIOA
//
//	#define dSPIN_SW_Pin            GPIO_Pin_4
//	#define dSPIN_SW_Port	        GPIOC
//
//	#define BUTTON_A_Pin		GPIO_Pin_1
//	#define BUTTON_A_Port		GPIOA
//
//	#define BUTTON_B_Pin		GPIO_Pin_2
//	#define BUTTON_B_Port		GPIOA
//    
//	#define dSPIN_STBY_RESET_Pin    GPIO_Pin_5
//	#define dSPIN_STBY_RESET_Port   GPIOC
//    
//        #define TIM_PWM                 TIM5
//   
//        /* List all the peripherals, which CLKs have to be enabled! */
//        /* Note : RCC_APB2Periph_AFIO is mandatory for interrupt enabling */
//        #define dSPIN_PERIPHERAL_CLKs_APB1      (RCC_APB1Periph_TIM5)
//	#define dSPIN_PERIPHERAL_CLKs_APB2	(RCC_APB2Periph_SPI1 | \
//                                                 RCC_APB2Periph_GPIOA | \
//                                                 RCC_APB2Periph_GPIOB | \
//                                                 RCC_APB2Periph_GPIOC | \
//                                                 RCC_APB2Periph_AFIO)
//#endif

/** dSPIN parameter min and max values
  */
#define dSPIN_CONF_PARAM_STALL_TH_MA_MAX           ((uint16_t)(4000)) /* current in mA */

/** Register bits / masks
 */
    
/* dSPIN electrical position register masks */
#define dSPIN_ELPOS_STEP_MASK			((uint8_t)0xC0)
#define dSPIN_ELPOS_MICROSTEP_MASK		((uint8_t)0x3F)

/* dSPIN min speed register bit / mask */
#define dSPIN_LSPD_OPT			((uint16_t)0x1000)
#define dSPIN_MIN_SPEED_MASK	((uint16_t)0x0FFF)

/* dSPIN Sync Output frequency enabling bit */
#define dSPIN_SYNC_EN		0x80

#if defined(L6472)
/* dSPIN step mode bit3 must be 1 */
#define dSPIN_STEP_MODE_BIT3	0x08
#endif /* defined(L6472) */

/* Exported types ------------------------------------------------------------*/

//typedef unsigned char  bool;

#if defined(L6470)
/** 
  * @brief dSPIN Init structure definition
  */
typedef struct
{
  uint32_t ABS_POS;
  uint16_t EL_POS;
  uint32_t MARK;
  uint32_t SPEED;
  uint16_t ACC;
  uint16_t DEC;
  uint16_t MAX_SPEED;
  uint16_t MIN_SPEED;
  uint16_t FS_SPD;
  uint8_t  KVAL_HOLD;
  uint8_t  KVAL_RUN;
  uint8_t  KVAL_ACC;
  uint8_t  KVAL_DEC;
  uint16_t INT_SPD;
  uint8_t  ST_SLP;
  uint8_t  FN_SLP_ACC;
  uint8_t  FN_SLP_DEC;
  uint8_t  K_THERM;
  uint8_t  ADC_OUT;
  uint8_t  OCD_TH;
  uint8_t  STALL_TH;
  uint8_t  STEP_MODE;
  uint8_t  ALARM_EN;
  uint16_t CONFIG;
  uint16_t STATUS;
}dSPIN_RegsStruct_TypeDef;

/* dSPIN Low speed optimization */
typedef enum {
	dSPIN_LSPD_OPT_OFF		=((uint16_t)0x0000),
	dSPIN_LSPD_OPT_ON		=((uint16_t)dSPIN_LSPD_OPT)
} dSPIN_LSPD_OPT_TypeDef;

/* dSPIN overcurrent threshold options */
typedef enum {
	dSPIN_OCD_TH_375mA		=((uint8_t)0x00),
	dSPIN_OCD_TH_750mA		=((uint8_t)0x01),
	dSPIN_OCD_TH_1125mA		=((uint8_t)0x02),
	dSPIN_OCD_TH_1500mA		=((uint8_t)0x03),
	dSPIN_OCD_TH_1875mA		=((uint8_t)0x04),
	dSPIN_OCD_TH_2250mA		=((uint8_t)0x05),
	dSPIN_OCD_TH_2625mA		=((uint8_t)0x06),
	dSPIN_OCD_TH_3000mA		=((uint8_t)0x07),
	dSPIN_OCD_TH_3375mA		=((uint8_t)0x08),
	dSPIN_OCD_TH_3750mA		=((uint8_t)0x09),
	dSPIN_OCD_TH_4125mA		=((uint8_t)0x0A),
	dSPIN_OCD_TH_4500mA		=((uint8_t)0x0B),
	dSPIN_OCD_TH_4875mA		=((uint8_t)0x0C),
	dSPIN_OCD_TH_5250mA		=((uint8_t)0x0D),
	dSPIN_OCD_TH_5625mA		=((uint8_t)0x0E),
	dSPIN_OCD_TH_6000mA		=((uint8_t)0x0F)
} dSPIN_OCD_TH_TypeDef;

/* dSPIN STEP_MODE register masks */
typedef enum {
	dSPIN_STEP_MODE_STEP_SEL		=((uint8_t)0x07),
	dSPIN_STEP_MODE_SYNC_SEL		=((uint8_t)0x70),
	dSPIN_STEP_MODE_SYNC_EN			=((uint8_t)0x80)
} dSPIN_STEP_MODE_Masks_TypeDef;

 /* dSPIN STEP_MODE register options */
/* dSPIN STEP_SEL options */
typedef enum {
	dSPIN_STEP_SEL_1		=((uint8_t)0x00),
	dSPIN_STEP_SEL_1_2		=((uint8_t)0x01),
	dSPIN_STEP_SEL_1_4		=((uint8_t)0x02),
	dSPIN_STEP_SEL_1_8		=((uint8_t)0x03),
	dSPIN_STEP_SEL_1_16		=((uint8_t)0x04),
	dSPIN_STEP_SEL_1_32		=((uint8_t)0x05),
	dSPIN_STEP_SEL_1_64		=((uint8_t)0x06),
	dSPIN_STEP_SEL_1_128	=((uint8_t)0x07)
} dSPIN_STEP_SEL_TypeDef;

/* dSPIN SYNC_SEL options */
typedef enum {
	dSPIN_SYNC_SEL_DISABLED		=((uint8_t)0x00),
        dSPIN_SYNC_SEL_1_2		=((uint8_t)(dSPIN_SYNC_EN|0x00)),
	dSPIN_SYNC_SEL_1		=((uint8_t)(dSPIN_SYNC_EN|0x10)),
	dSPIN_SYNC_SEL_2		=((uint8_t)(dSPIN_SYNC_EN|0x20)),
	dSPIN_SYNC_SEL_4		=((uint8_t)(dSPIN_SYNC_EN|0x30)),
	dSPIN_SYNC_SEL_8		=((uint8_t)(dSPIN_SYNC_EN|0x40)),
	dSPIN_SYNC_SEL_16		=((uint8_t)(dSPIN_SYNC_EN|0x50)),
	dSPIN_SYNC_SEL_32		=((uint8_t)(dSPIN_SYNC_EN|0x60)),
	dSPIN_SYNC_SEL_64		=((uint8_t)(dSPIN_SYNC_EN|0x70))
} dSPIN_SYNC_SEL_TypeDef;

/* dSPIN ALARM_EN register options */
typedef enum {
	dSPIN_ALARM_EN_OVERCURRENT			=((uint8_t)0x01),
	dSPIN_ALARM_EN_THERMAL_SHUTDOWN		=((uint8_t)0x02),
	dSPIN_ALARM_EN_THERMAL_WARNING		=((uint8_t)0x04),
	dSPIN_ALARM_EN_UNDER_VOLTAGE		=((uint8_t)0x08),
	dSPIN_ALARM_EN_STALL_DET_A			=((uint8_t)0x10),
	dSPIN_ALARM_EN_STALL_DET_B			=((uint8_t)0x20),
	dSPIN_ALARM_EN_SW_TURN_ON			=((uint8_t)0x40),
	dSPIN_ALARM_EN_WRONG_NPERF_CMD		=((uint8_t)0x80)
} dSPIN_ALARM_EN_TypeDef;

/* dSPIN Config register masks */
typedef enum {
	dSPIN_CONFIG_OSC_SEL					=((uint16_t)0x0007),
	dSPIN_CONFIG_EXT_CLK					=((uint16_t)0x0008),
	dSPIN_CONFIG_SW_MODE					=((uint16_t)0x0010),
	dSPIN_CONFIG_EN_VSCOMP					=((uint16_t)0x0020),
	dSPIN_CONFIG_OC_SD						=((uint16_t)0x0080),
	dSPIN_CONFIG_POW_SR						=((uint16_t)0x0300),
	dSPIN_CONFIG_F_PWM_DEC					=((uint16_t)0x1C00),
	dSPIN_CONFIG_F_PWM_INT					=((uint16_t)0xE000)
} dSPIN_CONFIG_Masks_TypeDef;

/* dSPIN Config register options */
typedef enum {
	dSPIN_CONFIG_INT_16MHZ					=((uint16_t)0x0000),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ		=((uint16_t)0x0008),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ		=((uint16_t)0x0009),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ		=((uint16_t)0x000A),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ		=((uint16_t)0x000B),
	dSPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE		=((uint16_t)0x0004),
	dSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE		=((uint16_t)0x0005),
	dSPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE		=((uint16_t)0x0006),
	dSPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE		=((uint16_t)0x0007),
	dSPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT		=((uint16_t)0x000C),
	dSPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT	=((uint16_t)0x000D),
	dSPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT	=((uint16_t)0x000E),
	dSPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT	=((uint16_t)0x000F)
} dSPIN_CONFIG_OSC_MGMT_TypeDef;

typedef enum {
	dSPIN_CONFIG_SW_HARD_STOP		=((uint16_t)0x0000),
	dSPIN_CONFIG_SW_USER			=((uint16_t)0x0010)
} dSPIN_CONFIG_SW_MODE_TypeDef;

typedef enum {
	dSPIN_CONFIG_VS_COMP_DISABLE	=((uint16_t)0x0000),
	dSPIN_CONFIG_VS_COMP_ENABLE		=((uint16_t)0x0020)
} dSPIN_CONFIG_EN_VSCOMP_TypeDef;

typedef enum {
	dSPIN_CONFIG_OC_SD_DISABLE		=((uint16_t)0x0000),
	dSPIN_CONFIG_OC_SD_ENABLE		=((uint16_t)0x0080)
} dSPIN_CONFIG_OC_SD_TypeDef;

typedef enum {
	dSPIN_CONFIG_SR_320V_us		=((uint16_t)0x0000),
        dSPIN_CONFIG_SR_075V_us		=((uint16_t)0x0100),
	dSPIN_CONFIG_SR_110V_us		=((uint16_t)0x0200),
	dSPIN_CONFIG_SR_260V_us		=((uint16_t)0x0300)
} dSPIN_CONFIG_POW_SR_TypeDef;

typedef enum {
	dSPIN_CONFIG_PWM_DIV_1		=(((uint16_t)0x00)<<13),
	dSPIN_CONFIG_PWM_DIV_2		=(((uint16_t)0x01)<<13),
	dSPIN_CONFIG_PWM_DIV_3		=(((uint16_t)0x02)<<13),
	dSPIN_CONFIG_PWM_DIV_4		=(((uint16_t)0x03)<<13),
	dSPIN_CONFIG_PWM_DIV_5		=(((uint16_t)0x04)<<13),
	dSPIN_CONFIG_PWM_DIV_6		=(((uint16_t)0x05)<<13),
	dSPIN_CONFIG_PWM_DIV_7		=(((uint16_t)0x06)<<13)
} dSPIN_CONFIG_F_PWM_INT_TypeDef;

typedef enum {
	dSPIN_CONFIG_PWM_MUL_0_625		=(((uint16_t)0x00)<<10),
	dSPIN_CONFIG_PWM_MUL_0_75		=(((uint16_t)0x01)<<10),
	dSPIN_CONFIG_PWM_MUL_0_875		=(((uint16_t)0x02)<<10),
	dSPIN_CONFIG_PWM_MUL_1			=(((uint16_t)0x03)<<10),
	dSPIN_CONFIG_PWM_MUL_1_25		=(((uint16_t)0x04)<<10),
	dSPIN_CONFIG_PWM_MUL_1_5		=(((uint16_t)0x05)<<10),
	dSPIN_CONFIG_PWM_MUL_1_75		=(((uint16_t)0x06)<<10),
	dSPIN_CONFIG_PWM_MUL_2			=(((uint16_t)0x07)<<10)
} dSPIN_CONFIG_F_PWM_DEC_TypeDef;

/* Status Register bit masks */
typedef enum {
	dSPIN_STATUS_HIZ			=(((uint16_t)0x0001)),
	dSPIN_STATUS_BUSY			=(((uint16_t)0x0002)),
	dSPIN_STATUS_SW_F			=(((uint16_t)0x0004)),
	dSPIN_STATUS_SW_EVN			=(((uint16_t)0x0008)),
	dSPIN_STATUS_DIR			=(((uint16_t)0x0010)),
	dSPIN_STATUS_MOT_STATUS		=(((uint16_t)0x0060)),
	dSPIN_STATUS_NOTPERF_CMD	=(((uint16_t)0x0080)),
	dSPIN_STATUS_WRONG_CMD		=(((uint16_t)0x0100)),
	dSPIN_STATUS_UVLO			=(((uint16_t)0x0200)),
	dSPIN_STATUS_TH_WRN			=(((uint16_t)0x0400)),
	dSPIN_STATUS_TH_SD			=(((uint16_t)0x0800)),
	dSPIN_STATUS_OCD			=(((uint16_t)0x1000)),
	dSPIN_STATUS_STEP_LOSS_A	=(((uint16_t)0x2000)),
	dSPIN_STATUS_STEP_LOSS_B	=(((uint16_t)0x4000)),
	dSPIN_STATUS_SCK_MOD		=(((uint16_t)0x8000))
} dSPIN_STATUS_Masks_TypeDef;

/* Status Register options */
typedef enum {
	dSPIN_STATUS_MOT_STATUS_STOPPED			=(((uint16_t)0x0000)<<5),
	dSPIN_STATUS_MOT_STATUS_ACCELERATION	=(((uint16_t)0x0001)<<5),
	dSPIN_STATUS_MOT_STATUS_DECELERATION	=(((uint16_t)0x0002)<<5),
	dSPIN_STATUS_MOT_STATUS_CONST_SPD		=(((uint16_t)0x0003)<<5)
} dSPIN_STATUS_TypeDef;

/* dSPIN internal register addresses */
typedef enum {
	dSPIN_ABS_POS			=((uint8_t)0x01),
	dSPIN_EL_POS			=((uint8_t)0x02),
	dSPIN_MARK				=((uint8_t)0x03),
	dSPIN_SPEED				=((uint8_t)0x04),
	dSPIN_ACC				=((uint8_t)0x05),
	dSPIN_DEC				=((uint8_t)0x06),
	dSPIN_MAX_SPEED			=((uint8_t)0x07),
	dSPIN_MIN_SPEED			=((uint8_t)0x08),
	dSPIN_FS_SPD			=((uint8_t)0x15),
	dSPIN_KVAL_HOLD			=((uint8_t)0x09),
	dSPIN_KVAL_RUN			=((uint8_t)0x0A),
	dSPIN_KVAL_ACC			=((uint8_t)0x0B),
	dSPIN_KVAL_DEC			=((uint8_t)0x0C),
	dSPIN_INT_SPD			=((uint8_t)0x0D),
	dSPIN_ST_SLP			=((uint8_t)0x0E),
	dSPIN_FN_SLP_ACC		=((uint8_t)0x0F),
	dSPIN_FN_SLP_DEC		=((uint8_t)0x10),
	dSPIN_K_THERM			=((uint8_t)0x11),
	dSPIN_ADC_OUT			=((uint8_t)0x12),
	dSPIN_OCD_TH			=((uint8_t)0x13),
	dSPIN_STALL_TH			=((uint8_t)0x14),
	dSPIN_STEP_MODE			=((uint8_t)0x16),
	dSPIN_ALARM_EN			=((uint8_t)0x17),
	dSPIN_CONFIG			=((uint8_t)0x18),
	dSPIN_STATUS			=((uint8_t)0x19),
	dSPIN_RESERVED_REG2		=((uint8_t)0x1A),
	dSPIN_RESERVED_REG1		=((uint8_t)0x1B)
} dSPIN_Registers_TypeDef;

/* dSPIN command set */
typedef enum {
	dSPIN_NOP			=((uint8_t)0x00),
	dSPIN_SET_PARAM		=((uint8_t)0x00),
	dSPIN_GET_PARAM		=((uint8_t)0x20),
	dSPIN_RUN			=((uint8_t)0x50),
	dSPIN_STEP_CLOCK	=((uint8_t)0x58),
	dSPIN_MOVE			=((uint8_t)0x40),
	dSPIN_GO_TO			=((uint8_t)0x60),
	dSPIN_GO_TO_DIR		=((uint8_t)0x68),
	dSPIN_GO_UNTIL		=((uint8_t)0x82),
        dSPIN_GO_UNTIL_ACT_CPY  =((uint8_t)0x8A),
	dSPIN_RELEASE_SW	=((uint8_t)0x92),
	dSPIN_GO_HOME		=((uint8_t)0x70),
	dSPIN_GO_MARK		=((uint8_t)0x78),
	dSPIN_RESET_POS		=((uint8_t)0xD8),
	dSPIN_RESET_DEVICE	=((uint8_t)0xC0),
	dSPIN_SOFT_STOP		=((uint8_t)0xB0),
	dSPIN_HARD_STOP		=((uint8_t)0xB8),
	dSPIN_SOFT_HIZ		=((uint8_t)0xA0),
	dSPIN_HARD_HIZ		=((uint8_t)0xA8),
	dSPIN_GET_STATUS	=((uint8_t)0xD0),
	dSPIN_RESERVED_CMD2	=((uint8_t)0xEB),
	dSPIN_RESERVED_CMD1	=((uint8_t)0xF8)
} dSPIN_Commands_TypeDef;

/* dSPIN direction options */
typedef enum {
	FWD		=((uint8_t)0x01),
	REV		=((uint8_t)0x00)
} dSPIN_Direction_TypeDef;

/* dSPIN action options */
typedef enum {
	ACTION_RESET	=((uint8_t)0x00),
	ACTION_COPY	=((uint8_t)0x08)
} dSPIN_Action_TypeDef;
/**
  * @}
  */
#endif /* defined(L6470) */

#if defined(L6472)
/** 
  * @brief dSPIN Init structure definition
  */
typedef struct
{
  uint32_t ABS_POS;
  uint16_t EL_POS;
  uint32_t MARK;
  uint32_t SPEED;
  uint16_t ACC;
  uint16_t DEC;
  uint16_t MAX_SPEED;
  uint16_t MIN_SPEED;
  uint16_t FS_SPD;
  uint8_t  TVAL_HOLD;
  uint8_t  TVAL_RUN;
  uint8_t  TVAL_ACC;
  uint8_t  TVAL_DEC;
  uint16_t RESERVED_3;
  uint8_t  T_FAST;
  uint8_t  TON_MIN;
  uint8_t  TOFF_MIN;
  uint8_t  RESERVED_2;
  uint8_t  ADC_OUT;
  uint8_t  OCD_TH;
  uint8_t  RESERVED_1;
  uint8_t  STEP_MODE;
  uint8_t  ALARM_EN;
  uint16_t CONFIG;
}dSPIN_RegsStruct_TypeDef;

/* dSPIN maximum fall step times */
typedef enum {
	dSPIN_FAST_STEP_2us		=((uint8_t)0x00),
	dSPIN_FAST_STEP_4us		=((uint8_t)0x01),
	dSPIN_FAST_STEP_6us		=((uint8_t)0x02),
	dSPIN_FAST_STEP_8us		=((uint8_t)0x03),
	dSPIN_FAST_STEP_10us		=((uint8_t)0x04),
	dSPIN_FAST_STEP_12us		=((uint8_t)0x05),
	dSPIN_FAST_STEP_14us		=((uint8_t)0x06),
	dSPIN_FAST_STEP_16us		=((uint8_t)0x07),
	dSPIN_FAST_STEP_18us		=((uint8_t)0x08),
	dSPIN_FAST_STEP_20us		=((uint8_t)0x09),
	dSPIN_FAST_STEP_22us		=((uint8_t)0x0A),
	dSPIN_FAST_STEP_24s		=((uint8_t)0x0B),
	dSPIN_FAST_STEP_26us		=((uint8_t)0x0C),
	dSPIN_FAST_STEP_28us		=((uint8_t)0x0D),
	dSPIN_FAST_STEP_30us		=((uint8_t)0x0E),
	dSPIN_FAST_STEP_32us		=((uint8_t)0x0F)
} dSPIN_FAST_STEP_TypeDef;

/* dSPIN maximum fast decay times */
typedef enum {
	dSPIN_TOFF_FAST_2us		=(((uint8_t)0x00)<<4),
	dSPIN_TOFF_FAST_4us		=(((uint8_t)0x01)<<4),
	dSPIN_TOFF_FAST_6us		=(((uint8_t)0x02)<<4),
	dSPIN_TOFF_FAST_8us		=(((uint8_t)0x03)<<4),
	dSPIN_TOFF_FAST_10us		=(((uint8_t)0x04)<<4),
	dSPIN_TOFF_FAST_12us		=(((uint8_t)0x05)<<4),
	dSPIN_TOFF_FAST_14us		=(((uint8_t)0x06)<<4),
	dSPIN_TOFF_FAST_16us		=(((uint8_t)0x07)<<4),
	dSPIN_TOFF_FAST_18us		=(((uint8_t)0x08)<<4),
	dSPIN_TOFF_FAST_20us		=(((uint8_t)0x09)<<4),
	dSPIN_TOFF_FAST_22us		=(((uint8_t)0x0A)<<4),
	dSPIN_TOFF_FAST_24us		=(((uint8_t)0x0B)<<4),
	dSPIN_TOFF_FAST_26us		=(((uint8_t)0x0C)<<4),
	dSPIN_TOFF_FAST_28us		=(((uint8_t)0x0D)<<4),
	dSPIN_TOFF_FAST_30us		=(((uint8_t)0x0E)<<4),
	dSPIN_TOFF_FAST_32us		=(((uint8_t)0x0F)<<4)
} dSPIN_TOFF_FAST_TypeDef;

/* dSPIN overcurrent threshold options */
typedef enum {
	dSPIN_OCD_TH_375mA		=((uint8_t)0x00),
	dSPIN_OCD_TH_750mA		=((uint8_t)0x01),
	dSPIN_OCD_TH_1125mA		=((uint8_t)0x02),
	dSPIN_OCD_TH_1500mA		=((uint8_t)0x03),
	dSPIN_OCD_TH_1875mA		=((uint8_t)0x04),
	dSPIN_OCD_TH_2250mA		=((uint8_t)0x05),
	dSPIN_OCD_TH_2625mA		=((uint8_t)0x06),
	dSPIN_OCD_TH_3000mA		=((uint8_t)0x07),
	dSPIN_OCD_TH_3375mA		=((uint8_t)0x08),
	dSPIN_OCD_TH_3750mA		=((uint8_t)0x09),
	dSPIN_OCD_TH_4125mA		=((uint8_t)0x0A),
	dSPIN_OCD_TH_4500mA		=((uint8_t)0x0B),
	dSPIN_OCD_TH_4875mA		=((uint8_t)0x0C),
	dSPIN_OCD_TH_5250mA		=((uint8_t)0x0D),
	dSPIN_OCD_TH_5625mA		=((uint8_t)0x0E),
	dSPIN_OCD_TH_6000mA		=((uint8_t)0x0F)
} dSPIN_OCD_TH_TypeDef;

/* dSPIN STEP_MODE register masks */
typedef enum {
	dSPIN_STEP_MODE_STEP_SEL		=((uint8_t)0x07),
	dSPIN_STEP_MODE_SYNC_SEL		=((uint8_t)0x70),
	dSPIN_STEP_MODE_SYNC_EN			=((uint8_t)0x80)
} dSPIN_STEP_MODE_Masks_TypeDef;

 /* dSPIN STEP_MODE register options */
/* dSPIN STEP_SEL options */
typedef enum {
	dSPIN_STEP_SEL_1		=((uint8_t)(dSPIN_STEP_MODE_BIT3|0x00)),
	dSPIN_STEP_SEL_1_2		=((uint8_t)(dSPIN_STEP_MODE_BIT3|0x01)),
	dSPIN_STEP_SEL_1_4		=((uint8_t)(dSPIN_STEP_MODE_BIT3|0x02)),
	dSPIN_STEP_SEL_1_8		=((uint8_t)(dSPIN_STEP_MODE_BIT3|0x03)),
	dSPIN_STEP_SEL_1_16		=((uint8_t)(dSPIN_STEP_MODE_BIT3|0x04))
} dSPIN_STEP_SEL_TypeDef;

/* dSPIN SYNC_SEL options */
typedef enum {
	dSPIN_SYNC_SEL_DISABLED		=((uint8_t)0x00),
        dSPIN_SYNC_SEL_1_2		=((uint8_t)(dSPIN_SYNC_EN|0x00)),
	dSPIN_SYNC_SEL_1		=((uint8_t)(dSPIN_SYNC_EN|0x10)),
	dSPIN_SYNC_SEL_2		=((uint8_t)(dSPIN_SYNC_EN|0x20)),
	dSPIN_SYNC_SEL_4		=((uint8_t)(dSPIN_SYNC_EN|0x30)),
	dSPIN_SYNC_SEL_8		=((uint8_t)(dSPIN_SYNC_EN|0x40))
} dSPIN_SYNC_SEL_TypeDef;

/* dSPIN ALARM_EN register options */
typedef enum {
	dSPIN_ALARM_EN_OVERCURRENT		=((uint8_t)0x01),
	dSPIN_ALARM_EN_THERMAL_SHUTDOWN		=((uint8_t)0x02),
	dSPIN_ALARM_EN_THERMAL_WARNING		=((uint8_t)0x04),
	dSPIN_ALARM_EN_UNDER_VOLTAGE		=((uint8_t)0x08),
	dSPIN_ALARM_EN_SW_TURN_ON		=((uint8_t)0x40),
	dSPIN_ALARM_EN_WRONG_NPERF_CMD		=((uint8_t)0x80)
} dSPIN_ALARM_EN_TypeDef;

/* dSPIN Config register masks */
typedef enum {
	dSPIN_CONFIG_OSC_SEL					=((uint16_t)0x0007),
	dSPIN_CONFIG_EXT_CLK					=((uint16_t)0x0008),
	dSPIN_CONFIG_SW_MODE					=((uint16_t)0x0010),
	dSPIN_CONFIG_EN_VSCOMP					=((uint16_t)0x0020),
	dSPIN_CONFIG_OC_SD					=((uint16_t)0x0080),
	dSPIN_CONFIG_POW_SR					=((uint16_t)0x0300),
	dSPIN_CONFIG_TSW					=((uint16_t)0x7C00),
	dSPIN_CONFIG_PRED_EN					=((uint16_t)0x8000)
} dSPIN_CONFIG_Masks_TypeDef;

/* dSPIN Config register options */
typedef enum {
	dSPIN_CONFIG_INT_16MHZ				=((uint16_t)0x0000),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ		=((uint16_t)0x0008),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ		=((uint16_t)0x0009),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ		=((uint16_t)0x000A),
	dSPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ		=((uint16_t)0x000B),
	dSPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE		=((uint16_t)0x0004),
	dSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE		=((uint16_t)0x0005),
	dSPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE		=((uint16_t)0x0006),
	dSPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE		=((uint16_t)0x0007),
	dSPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT		=((uint16_t)0x000C),
	dSPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT	=((uint16_t)0x000D),
	dSPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT	=((uint16_t)0x000E),
	dSPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT	=((uint16_t)0x000F)
} dSPIN_CONFIG_OSC_MGMT_TypeDef;

typedef enum {
	dSPIN_CONFIG_SW_HARD_STOP		=((uint16_t)0x0000),
	dSPIN_CONFIG_SW_USER			=((uint16_t)0x0010)
} dSPIN_CONFIG_SW_MODE_TypeDef;

typedef enum {
	dSPIN_CONFIG_TQ_REG_TVAL_USED	        =((uint16_t)0x0000),
	dSPIN_CONFIG_TQ_REG_ADC_OUT		=((uint16_t)0x0020)
} dSPIN_CONFIG_EN_TQREG_TypeDef;

typedef enum {
	dSPIN_CONFIG_OC_SD_DISABLE		=((uint16_t)0x0000),
	dSPIN_CONFIG_OC_SD_ENABLE		=((uint16_t)0x0080)
} dSPIN_CONFIG_OC_SD_TypeDef;

typedef enum {
	dSPIN_CONFIG_SR_320V_us		=((uint16_t)0x0000),
        dSPIN_CONFIG_SR_075V_us		=((uint16_t)0x0100),
	dSPIN_CONFIG_SR_110V_us		=((uint16_t)0x0200),
	dSPIN_CONFIG_SR_260V_us		=((uint16_t)0x0300)
} dSPIN_CONFIG_POW_SR_TypeDef;

typedef enum {
	dSPIN_CONFIG_PRED_DISABLE	=((uint16_t)0x0000),
	dSPIN_CONFIG_PRED_ENABLE	=((uint16_t)0x8000)
} dSPIN_CONFIG_PRED_EN_TypeDef;

typedef enum {
	dSPIN_CONFIG_TSW_004us		=(((uint16_t)0x01)<<10),
	dSPIN_CONFIG_TSW_008us		=(((uint16_t)0x02)<<10),
	dSPIN_CONFIG_TSW_012us		=(((uint16_t)0x03)<<10),
	dSPIN_CONFIG_TSW_016us		=(((uint16_t)0x04)<<10),
	dSPIN_CONFIG_TSW_020us		=(((uint16_t)0x05)<<10),
	dSPIN_CONFIG_TSW_024us		=(((uint16_t)0x06)<<10),
	dSPIN_CONFIG_TSW_028us		=(((uint16_t)0x07)<<10),
	dSPIN_CONFIG_TSW_032us		=(((uint16_t)0x08)<<10),
	dSPIN_CONFIG_TSW_036us		=(((uint16_t)0x09)<<10),
	dSPIN_CONFIG_TSW_040us		=(((uint16_t)0x0A)<<10),
	dSPIN_CONFIG_TSW_044us		=(((uint16_t)0x0B)<<10),
	dSPIN_CONFIG_TSW_048us		=(((uint16_t)0x0C)<<10),
	dSPIN_CONFIG_TSW_052us		=(((uint16_t)0x0D)<<10),
	dSPIN_CONFIG_TSW_056us		=(((uint16_t)0x0E)<<10),
	dSPIN_CONFIG_TSW_060us		=(((uint16_t)0x0F)<<10),
	dSPIN_CONFIG_TSW_064us		=(((uint16_t)0x10)<<10),
	dSPIN_CONFIG_TSW_068us		=(((uint16_t)0x11)<<10),
	dSPIN_CONFIG_TSW_072us		=(((uint16_t)0x12)<<10),
	dSPIN_CONFIG_TSW_076us		=(((uint16_t)0x13)<<10),
	dSPIN_CONFIG_TSW_080us		=(((uint16_t)0x14)<<10),
	dSPIN_CONFIG_TSW_084us		=(((uint16_t)0x15)<<10),
	dSPIN_CONFIG_TSW_088us		=(((uint16_t)0x16)<<10),
	dSPIN_CONFIG_TSW_092us		=(((uint16_t)0x17)<<10),
	dSPIN_CONFIG_TSW_096us		=(((uint16_t)0x18)<<10),
	dSPIN_CONFIG_TSW_100us		=(((uint16_t)0x19)<<10),
	dSPIN_CONFIG_TSW_104us		=(((uint16_t)0x1A)<<10),
	dSPIN_CONFIG_TSW_108us		=(((uint16_t)0x1B)<<10),
	dSPIN_CONFIG_TSW_112us		=(((uint16_t)0x1C)<<10),
	dSPIN_CONFIG_TSW_116us		=(((uint16_t)0x1D)<<10),
	dSPIN_CONFIG_TSW_120us		=(((uint16_t)0x1E)<<10),
        dSPIN_CONFIG_TSW_124us		=(((uint16_t)0x1F)<<10)        
} dSPIN_CONFIG_TSW_TypeDef;

/* Status Register bit masks */
typedef enum {
	dSPIN_STATUS_HIZ			=(((uint16_t)0x0001)),
	dSPIN_STATUS_BUSY			=(((uint16_t)0x0002)),
	dSPIN_STATUS_SW_F			=(((uint16_t)0x0004)),
	dSPIN_STATUS_SW_EVN			=(((uint16_t)0x0008)),
	dSPIN_STATUS_DIR			=(((uint16_t)0x0010)),
	dSPIN_STATUS_MOT_STATUS		=(((uint16_t)0x0060)),
	dSPIN_STATUS_NOTPERF_CMD	=(((uint16_t)0x0080)),
	dSPIN_STATUS_WRONG_CMD		=(((uint16_t)0x0100)),
	dSPIN_STATUS_UVLO			=(((uint16_t)0x0200)),
	dSPIN_STATUS_TH_WRN			=(((uint16_t)0x0400)),
	dSPIN_STATUS_TH_SD			=(((uint16_t)0x0800)),
	dSPIN_STATUS_OCD			=(((uint16_t)0x1000)),
	dSPIN_STATUS_SCK_MOD		=(((uint16_t)0x8000))
} dSPIN_STATUS_Masks_TypeDef;

/* Status Register options */
typedef enum {
	dSPIN_STATUS_MOT_STATUS_STOPPED			=(((uint16_t)0x0000)<<5),
	dSPIN_STATUS_MOT_STATUS_ACCELERATION	=(((uint16_t)0x0001)<<5),
	dSPIN_STATUS_MOT_STATUS_DECELERATION	=(((uint16_t)0x0002)<<5),
	dSPIN_STATUS_MOT_STATUS_CONST_SPD		=(((uint16_t)0x0003)<<5)
} dSPIN_STATUS_TypeDef;

/* dSPIN internal register addresses */
typedef enum {
	dSPIN_ABS_POS			=((uint8_t)0x01),
	dSPIN_EL_POS			=((uint8_t)0x02),
	dSPIN_MARK			=((uint8_t)0x03),
	dSPIN_SPEED			=((uint8_t)0x04),
	dSPIN_ACC			=((uint8_t)0x05),
	dSPIN_DEC			=((uint8_t)0x06),
	dSPIN_MAX_SPEED			=((uint8_t)0x07),
	dSPIN_MIN_SPEED			=((uint8_t)0x08),
	dSPIN_FS_SPD			=((uint8_t)0x15),
	dSPIN_TVAL_HOLD			=((uint8_t)0x09),
	dSPIN_TVAL_RUN			=((uint8_t)0x0A),
	dSPIN_TVAL_ACC			=((uint8_t)0x0B),
	dSPIN_TVAL_DEC			=((uint8_t)0x0C),
	dSPIN_RESERVED_REG5		=((uint8_t)0x0D),
	dSPIN_T_FAST			=((uint8_t)0x0E),
	dSPIN_TON_MIN		        =((uint8_t)0x0F),
	dSPIN_TOFF_MIN		        =((uint8_t)0x10),
	dSPIN_RESERVED_REG4		=((uint8_t)0x11),
	dSPIN_ADC_OUT			=((uint8_t)0x12),
	dSPIN_OCD_TH			=((uint8_t)0x13),
	dSPIN_RESERVED_REG3		=((uint8_t)0x14),
	dSPIN_STEP_MODE			=((uint8_t)0x16),
	dSPIN_ALARM_EN			=((uint8_t)0x17),
	dSPIN_CONFIG			=((uint8_t)0x18),
	dSPIN_STATUS			=((uint8_t)0x19),
	dSPIN_RESERVED_REG2		=((uint8_t)0x1A),
	dSPIN_RESERVED_REG1		=((uint8_t)0x1B)
} dSPIN_Registers_TypeDef;

/* dSPIN command set */
typedef enum {
	dSPIN_NOP		=((uint8_t)0x00),
	dSPIN_SET_PARAM		=((uint8_t)0x00),
	dSPIN_GET_PARAM		=((uint8_t)0x20),
	dSPIN_RUN		=((uint8_t)0x50),
	dSPIN_STEP_CLOCK	=((uint8_t)0x58),
	dSPIN_MOVE		=((uint8_t)0x40),
	dSPIN_GO_TO		=((uint8_t)0x60),
	dSPIN_GO_TO_DIR		=((uint8_t)0x68),
	dSPIN_GO_UNTIL		=((uint8_t)0x82),
        dSPIN_GO_UNTIL_ACT_CPY  =((uint8_t)0x8A),
	dSPIN_RELEASE_SW	=((uint8_t)0x92),
	dSPIN_GO_HOME		=((uint8_t)0x70),
	dSPIN_GO_MARK		=((uint8_t)0x78),
	dSPIN_RESET_POS		=((uint8_t)0xD8),
	dSPIN_RESET_DEVICE	=((uint8_t)0xC0),
	dSPIN_SOFT_STOP		=((uint8_t)0xB0),
	dSPIN_HARD_STOP		=((uint8_t)0xB8),
	dSPIN_SOFT_HIZ		=((uint8_t)0xA0),
	dSPIN_HARD_HIZ		=((uint8_t)0xA8),
	dSPIN_GET_STATUS	=((uint8_t)0xD0),
	dSPIN_RESERVED_CMD2	=((uint8_t)0xEB),
	dSPIN_RESERVED_CMD1	=((uint8_t)0xF8)
} dSPIN_Commands_TypeDef;

/* dSPIN direction options */
typedef enum {
	FWD		=((uint8_t)0x01),
	REV		=((uint8_t)0x00)
} dSPIN_Direction_TypeDef;

/* dSPIN action options */
typedef enum {
	ACTION_RESET	=((uint8_t)0x00),
	ACTION_COPY	=((uint8_t)0x08)
} dSPIN_Action_TypeDef;
/**
  * @}
  */
#endif /* defined(L6472) */

/* Exported macro ------------------------------------------------------------*/
#define Speed_Steps_to_Par(steps) ((uint32_t)(((steps)*67.108864)+0.5))			/* Speed conversion, range 0 to 15625 steps/s */
#define AccDec_Steps_to_Par(steps) ((uint16_t)(((steps)*0.068719476736)+0.5))	/* Acc/Dec rates conversion, range 14.55 to 59590 steps/s2 */
#define MaxSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))			/* Max Speed conversion, range 15.25 to 15610 steps/s */
#define MinSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))			/* Min Speed conversion, range 0 to 976.3 steps/s */
#define FSSpd_Steps_to_Par(steps) ((uint16_t)((steps)*0.065536))				/* Full Step Speed conversion, range 7.63 to 15625 steps/s */
#if defined(L6470)
#define IntSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))			/* Intersect Speed conversion, range 0 to 3906 steps/s */
#define Kval_Perc_to_Par(perc) ((uint8_t)(((perc)/0.390625)+0.5))				/* KVAL conversions, range 0.4% to 99.6% */
#define BEMF_Slope_Perc_to_Par(perc) ((uint8_t)(((perc)/0.00156862745098)+0.5))	/* BEMF compensation slopes, range 0 to 0.4% s/step */
#define KTherm_to_Par(KTherm) ((uint8_t)(((KTherm - 1)/0.03125)+0.5))			/* K_THERM compensation conversion, range 1 to 1.46875 */
#define StallTh_to_Par(StallTh) ((uint8_t)(((StallTh - 31.25)/31.25)+0.5))		/* Stall Threshold conversion, range 31.25mA to 4000mA */
#endif /* defined(L6470) */
#if defined(L6472)
#define Tval_Current_to_Par(Tval) ((uint8_t)(((Tval - 31.25)/31.25)+0.5))       /* Torque regulation DAC current conversion, range 31.25mA to 4000mA */
#define Tmin_Time_to_Par(Tmin) ((uint8_t)(((Tmin - 0.5)*2)+0.5))                /* Minimum time conversion, range 0.5us to 64us */
#endif /* defined(L6472) */
#if defined(DEBUG)
#define CHECK_REG(reg, result) if (dSPIN_RegsStruct-> ## reg != dSPIN_Get_Param(dSPIN_ ## reg)) result |= 1 << ((uint8_t) dSPIN_ ## reg)
#define PRINT_REG(reg_name, read_reg, diff, reg_name_string)  strcpy(reg_name_string,"          "); \
  strncpy(reg_name_string, #reg_name, strlen(#reg_name)); \
  strcpy(diff," "); \
  read_reg = dSPIN_Get_Param(dSPIN_ ## reg_name); \
  if (read_reg != dSPIN_RegsStruct-> ## reg_name) strcpy(diff,"x"); \
  printf("%s %s mcu: 0x%-6X, dspin: 0x%-6X\n", diff, reg_name_string, dSPIN_RegsStruct-> ## reg_name, read_reg)
#endif /* defined(DEBUG) */
/* Exported functions ------------------------------------------------------- */
void dSPIN_Delay(__IO uint32_t nCount);
//void dSPIN_Reset_And_Standby(void);
void dSPIN_Reset_And_Standby(uint32_t dwDev_num);
void dSPIN_Gpio_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void dSPIN_Led_Check(void);
void dSPIN_Peripherals_Init(void);
void dSPIN_PWM_Enable(uint16_t Period);
void dSPIN_PWM_DISABLE(void); 
void dSPIN_Interrupt_Channel_Config(void);
void dSPIN_Flag_Interrupt_GPIO_Config(void);
void dSPIN_Busy_Interrupt_GPIO_Config(void);
void dSPIN_Busy_Interrupt_GPIO_DeConfig(void);
void dSPIN_Buttons_Interrupts_GPIO_Config(void);
void dSPIN_Switch_Motor_Interrupt_Config(void);
void dSPIN_Regs_Struct_Reset(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct);
//void dSPIN_Registers_Set(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct);
void dSPIN_Registers_Set(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct,uint32_t dwDev_num);
#if defined(DEBUG) 
uint32_t dSPIN_Registers_Check(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct);
void dSPIN_Registers_Get(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct);
#endif /* defined(DEBUG) */
//void dSPIN_Nop(void);
void dSPIN_Nop(uint32_t dwDev_num);
//void dSPIN_Set_Param(dSPIN_Registers_TypeDef param, uint32_t value);
void dSPIN_Set_Param(uint32_t dwDev_num, dSPIN_Registers_TypeDef param, uint32_t value);
//uint32_t dSPIN_Get_Param(dSPIN_Registers_TypeDef param);
uint32_t dSPIN_Get_Param(uint32_t dwDev_num,dSPIN_Registers_TypeDef param);
//void dSPIN_Run(dSPIN_Direction_TypeDef direction, uint32_t speed);
void dSPIN_Run(dSPIN_Direction_TypeDef direction, uint32_t speed,uint32_t dwDev_num);

//void dSPIN_Step_Clock(dSPIN_Direction_TypeDef direction);
void dSPIN_Step_Clock(dSPIN_Direction_TypeDef direction, uint32_t dwDev_num);

//void dSPIN_Move(dSPIN_Direction_TypeDef direction, uint32_t n_step);
void dSPIN_Move(dSPIN_Direction_TypeDef direction, uint32_t n_step, uint32_t dwDev_num );

//void dSPIN_Go_To(uint32_t abs_pos);
void dSPIN_Go_To(uint32_t abs_pos, uint32_t dwDev_num);
//void dSPIN_Go_To_Dir(dSPIN_Direction_TypeDef direction, uint32_t abs_pos);
void dSPIN_Go_To_Dir(dSPIN_Direction_TypeDef direction, int32_t abs_pos,uint32_t dwDev_num);

void dSPIN_Go_Until(dSPIN_Action_TypeDef action, dSPIN_Direction_TypeDef direction, uint32_t speed,uint32_t dwDev_num);
void dSPIN_Release_SW(dSPIN_Action_TypeDef action, dSPIN_Direction_TypeDef direction, uint32_t dwDev_num);
void dSPIN_Go_Home(uint32_t dwDev_num);
void dSPIN_Go_Mark(uint32_t dwDev_num);
void dSPIN_Reset_Pos(uint32_t dwDev_num);
void dSPIN_Reset_Device(uint32_t dwDev_num);
void dSPIN_Soft_Stop(uint32_t dwDev_num);
void dSPIN_Hard_Stop(uint32_t dwDev_num);
void dSPIN_Soft_HiZ(uint32_t dwDev_num);
void dSPIN_Hard_HiZ(uint32_t dwDev_num);
uint16_t dSPIN_Get_Status(uint32_t dwDev_num);
//uint8_t dSPIN_Busy_HW(void);
uint8_t dSPIN_Busy_HW(uint32_t dwDev_num);
uint8_t dSPIN_Busy_SW(uint32_t dwDev_num);
//uint8_t dSPIN_Flag(void);
uint8_t dSPIN_Flag(uint32_t dwDev_num);
//uint8_t dSPIN_Write_Byte(uint8_t byte);
uint8_t dSPIN_Write_Byte(uint32_t dwDev_num,uint8_t byte);
void dSPIN_Write_Daisy_Chain_Bytes(uint8_t *pTxByte, uint8_t *pRxByte, uint8_t nBytes,uint32_t dwDev_num);
void dSPIN_All_Slaves_Registers_Set(uint8_t slaves_number, dSPIN_RegsStruct_TypeDef *dSPIN_RegsStructArray,uint32_t dwDev_num);
void dSPIN_All_Slaves_Set_Param(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue,uint32_t dwDev_num);
void dSPIN_All_Slaves_Get_Param(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue,uint32_t dwDev_num);
void dSPIN_One_Slave_Move(uint8_t slaveNumber, uint8_t slaves_number, dSPIN_Direction_TypeDef direction, uint32_t n_step,uint32_t dwDev_num);
void dSPIN_One_Slave_Run(uint8_t slaveNumber, uint8_t slaves_number, dSPIN_Direction_TypeDef direction, uint32_t speed,uint32_t dwDev_num);
void dSPIN_One_Slave_Send_Command(uint8_t slaveNumber, uint8_t slaves_number, uint8_t param, uint32_t value,uint32_t dwDev_num);
void dSPIN_All_Slaves_Send_Command(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue,uint32_t dwDev_num);
void dSPIN_All_Slaves_Get_Status(uint8_t slaves_number, uint32_t *pValue,uint32_t dwDev_num);
uint8_t dSPIN_One_Or_More_Slaves_Busy_SW(uint8_t slaves_number,uint32_t dwDev_num);

/** @} */  

void set_spi_handle(SPI_HandleTypeDef *h_spi);

#endif /* __DSPIN_H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
