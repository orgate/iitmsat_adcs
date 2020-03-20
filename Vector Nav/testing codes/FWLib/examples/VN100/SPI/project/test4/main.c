#define sensorID 0
#define VN_BYTES2WORD(b1, b2, b3, b4) (((unsigned long)(b1) << 24) | ((unsigned long)(b2) << 16) | ((unsigned long)(b3) << 8) | (unsigned long)(b4))
/* #define RESET 0
#define SET 1 */
/* to be verified: struct and var inside if code */
#ifdef _SPI1
  SPI1 = (SPI_TypeDef *)  SPI1_BASE;
#endif /*_SPI1 */








#include "lib.h"




* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_GPIO_H
#define __STM32F10x_GPIO_H

/* Includes ------------------------------------------------------------------*/
/* #include "stm32f10x_map.h" */

/* Exported types ------------------------------------------------------------*/
#define IS_GPIO_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == GPIOA_BASE) || \
                                    ((*(u32*)&(PERIPH)) == GPIOB_BASE) || \
                                    ((*(u32*)&(PERIPH)) == GPIOC_BASE) || \
                                    ((*(u32*)&(PERIPH)) == GPIOD_BASE) || \
                                    ((*(u32*)&(PERIPH)) == GPIOE_BASE) || \
                                    ((*(u32*)&(PERIPH)) == GPIOF_BASE) || \
                                    ((*(u32*)&(PERIPH)) == GPIOG_BASE))
                                     
/* Output Maximum frequency selection ----------------------------------------*/
typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;

#define IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_Speed_10MHz) || ((SPEED) == GPIO_Speed_2MHz) || \
                              ((SPEED) == GPIO_Speed_50MHz))
                                         
/* Configuration Mode enumeration --------------------------------------------*/
typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;

#define IS_GPIO_MODE(MODE) (((MODE) == GPIO_Mode_AIN) || ((MODE) == GPIO_Mode_IN_FLOATING) || \
                            ((MODE) == GPIO_Mode_IPD) || ((MODE) == GPIO_Mode_IPU) || \
                            ((MODE) == GPIO_Mode_Out_OD) || ((MODE) == GPIO_Mode_Out_PP) || \
                            ((MODE) == GPIO_Mode_AF_OD) || ((MODE) == GPIO_Mode_AF_PP))
                              
/* GPIO Init structure definition */
typedef struct
{
  u16 GPIO_Pin;
  GPIOSpeed_TypeDef GPIO_Speed;
  GPIOMode_TypeDef GPIO_Mode;
}GPIO_InitTypeDef;

/* Bit_SET and Bit_RESET enumeration -----------------------------------------*/
typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;
#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET))

/* Exported constants --------------------------------------------------------*/
/* GPIO pins define ----------------------------------------------------------*/
#define GPIO_Pin_0                 ((u16)0x0001)  /* Pin 0 selected */
#define GPIO_Pin_1                 ((u16)0x0002)  /* Pin 1 selected */
#define GPIO_Pin_2                 ((u16)0x0004)  /* Pin 2 selected */
#define GPIO_Pin_3                 ((u16)0x0008)  /* Pin 3 selected */
#define GPIO_Pin_4                 ((u16)0x0010)  /* Pin 4 selected */
#define GPIO_Pin_5                 ((u16)0x0020)  /* Pin 5 selected */
#define GPIO_Pin_6                 ((u16)0x0040)  /* Pin 6 selected */
#define GPIO_Pin_7                 ((u16)0x0080)  /* Pin 7 selected */
#define GPIO_Pin_8                 ((u16)0x0100)  /* Pin 8 selected */
#define GPIO_Pin_9                 ((u16)0x0200)  /* Pin 9 selected */
#define GPIO_Pin_10                ((u16)0x0400)  /* Pin 10 selected */
#define GPIO_Pin_11                ((u16)0x0800)  /* Pin 11 selected */
#define GPIO_Pin_12                ((u16)0x1000)  /* Pin 12 selected */
#define GPIO_Pin_13                ((u16)0x2000)  /* Pin 13 selected */
#define GPIO_Pin_14                ((u16)0x4000)  /* Pin 14 selected */
#define GPIO_Pin_15                ((u16)0x8000)  /* Pin 15 selected */
#define GPIO_Pin_All               ((u16)0xFFFF)  /* All pins selected */

#define IS_GPIO_PIN(PIN) ((((PIN) & (u16)0x00) == 0x00) && ((PIN) != (u16)0x00))

#define IS_GET_GPIO_PIN(PIN) (((PIN) == GPIO_Pin_0) || \
                              ((PIN) == GPIO_Pin_1) || \
                              ((PIN) == GPIO_Pin_2) || \
                              ((PIN) == GPIO_Pin_3) || \
                              ((PIN) == GPIO_Pin_4) || \
                              ((PIN) == GPIO_Pin_5) || \
                              ((PIN) == GPIO_Pin_6) || \
                              ((PIN) == GPIO_Pin_7) || \
                              ((PIN) == GPIO_Pin_8) || \
                              ((PIN) == GPIO_Pin_9) || \
                              ((PIN) == GPIO_Pin_10) || \
                              ((PIN) == GPIO_Pin_11) || \
                              ((PIN) == GPIO_Pin_12) || \
                              ((PIN) == GPIO_Pin_13) || \
                              ((PIN) == GPIO_Pin_14) || \
                              ((PIN) == GPIO_Pin_15))
                            
/* GPIO Remap define ---------------------------------------------------------*/
#define GPIO_Remap_SPI1            ((u32)0x00000001)  /* SPI1 Alternate Function mapping */
#define GPIO_Remap_I2C1            ((u32)0x00000002)  /* I2C1 Alternate Function mapping */
#define GPIO_Remap_USART1          ((u32)0x00000004)  /* USART1 Alternate Function mapping */
#define GPIO_Remap_USART2          ((u32)0x00000008)  /* USART2 Alternate Function mapping */
#define GPIO_PartialRemap_USART3   ((u32)0x00140010)  /* USART3 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART3      ((u32)0x00140030)  /* USART3 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM1     ((u32)0x00160040)  /* TIM1 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM1        ((u32)0x001600C0)  /* TIM1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM2    ((u32)0x00180100)  /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2    ((u32)0x00180200)  /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM2        ((u32)0x00180300)  /* TIM2 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM3     ((u32)0x001A0800)  /* TIM3 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM3        ((u32)0x001A0C00)  /* TIM3 Full Alternate Function mapping */
#define GPIO_Remap_TIM4            ((u32)0x00001000)  /* TIM4 Alternate Function mapping */
#define GPIO_Remap1_CAN            ((u32)0x001D4000)  /* CAN Alternate Function mapping */
#define GPIO_Remap2_CAN            ((u32)0x001D6000)  /* CAN Alternate Function mapping */
#define GPIO_Remap_PD01            ((u32)0x00008000)  /* PD01 Alternate Function mapping */
#define GPIO_Remap_TIM5CH4_LSI     ((u32)0x00200001)  /* LSI connected to TIM5 Channel4 input capture for calibration */
#define GPIO_Remap_ADC1_ETRGINJ    ((u32)0x00200002)  /* ADC1 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC1_ETRGREG    ((u32)0x00200004)  /* ADC1 External Trigger Regular Conversion remapping */
#define GPIO_Remap_ADC2_ETRGINJ    ((u32)0x00200008)  /* ADC2 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC2_ETRGREG    ((u32)0x00200010)  /* ADC2 External Trigger Regular Conversion remapping */
#define GPIO_Remap_SWJ_NoJTRST     ((u32)0x00300100)  /* Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */
#define GPIO_Remap_SWJ_JTAGDisable ((u32)0x00300200)  /* JTAG-DP Disabled and SW-DP Enabled */
#define GPIO_Remap_SWJ_Disable     ((u32)0x00300400)  /* Full SWJ Disabled (JTAG-DP + SW-DP) */


#define IS_GPIO_REMAP(REMAP) (((REMAP) == GPIO_Remap_SPI1) || ((REMAP) == GPIO_Remap_I2C1) || \
                              ((REMAP) == GPIO_Remap_USART1) || ((REMAP) == GPIO_Remap_USART2) || \
                              ((REMAP) == GPIO_PartialRemap_USART3) || ((REMAP) == GPIO_FullRemap_USART3) || \
                              ((REMAP) == GPIO_PartialRemap_TIM1) || ((REMAP) == GPIO_FullRemap_TIM1) || \
                              ((REMAP) == GPIO_PartialRemap1_TIM2) || ((REMAP) == GPIO_PartialRemap2_TIM2) || \
                              ((REMAP) == GPIO_FullRemap_TIM2) || ((REMAP) == GPIO_PartialRemap_TIM3) || \
                              ((REMAP) == GPIO_FullRemap_TIM3) || ((REMAP) == GPIO_Remap_TIM4) || \
                              ((REMAP) == GPIO_Remap1_CAN) || ((REMAP) == GPIO_Remap2_CAN) || \
                              ((REMAP) == GPIO_Remap_PD01) || ((REMAP) == GPIO_Remap_TIM5CH4_LSI) || \
                              ((REMAP) == GPIO_Remap_ADC1_ETRGINJ) ||((REMAP) == GPIO_Remap_ADC1_ETRGREG) || \
                              ((REMAP) == GPIO_Remap_ADC2_ETRGINJ) ||((REMAP) == GPIO_Remap_ADC2_ETRGREG) || \
                              ((REMAP) == GPIO_Remap_SWJ_NoJTRST) || ((REMAP) == GPIO_Remap_SWJ_JTAGDisable)|| \
                              ((REMAP) == GPIO_Remap_SWJ_Disable))
                              
/* GPIO Port Sources ---------------------------------------------------------*/
#define GPIO_PortSourceGPIOA       ((u8)0x00)
#define GPIO_PortSourceGPIOB       ((u8)0x01)
#define GPIO_PortSourceGPIOC       ((u8)0x02)
#define GPIO_PortSourceGPIOD       ((u8)0x03)
#define GPIO_PortSourceGPIOE       ((u8)0x04)
#define GPIO_PortSourceGPIOF       ((u8)0x05)
#define GPIO_PortSourceGPIOG       ((u8)0x06)

#define IS_GPIO_EVENTOUT_PORT_SOURCE(PORTSOURCE) (((PORTSOURCE) == GPIO_PortSourceGPIOA) || \
                                                  ((PORTSOURCE) == GPIO_PortSourceGPIOB) || \
                                                  ((PORTSOURCE) == GPIO_PortSourceGPIOC) || \
                                                  ((PORTSOURCE) == GPIO_PortSourceGPIOD) || \
                                                  ((PORTSOURCE) == GPIO_PortSourceGPIOE))
                                         
#define IS_GPIO_EXTI_PORT_SOURCE(PORTSOURCE) (((PORTSOURCE) == GPIO_PortSourceGPIOA) || \
                                              ((PORTSOURCE) == GPIO_PortSourceGPIOB) || \
                                              ((PORTSOURCE) == GPIO_PortSourceGPIOC) || \
                                              ((PORTSOURCE) == GPIO_PortSourceGPIOD) || \
                                              ((PORTSOURCE) == GPIO_PortSourceGPIOE) || \
                                              ((PORTSOURCE) == GPIO_PortSourceGPIOF) || \
                                              ((PORTSOURCE) == GPIO_PortSourceGPIOG))
                                       
/* GPIO Pin sources ----------------------------------------------------------*/
#define GPIO_PinSource0            ((u8)0x00)
#define GPIO_PinSource1            ((u8)0x01)
#define GPIO_PinSource2            ((u8)0x02)
#define GPIO_PinSource3            ((u8)0x03)
#define GPIO_PinSource4            ((u8)0x04)
#define GPIO_PinSource5            ((u8)0x05)
#define GPIO_PinSource6            ((u8)0x06)
#define GPIO_PinSource7            ((u8)0x07)
#define GPIO_PinSource8            ((u8)0x08)
#define GPIO_PinSource9            ((u8)0x09)
#define GPIO_PinSource10           ((u8)0x0A)
#define GPIO_PinSource11           ((u8)0x0B)
#define GPIO_PinSource12           ((u8)0x0C)
#define GPIO_PinSource13           ((u8)0x0D)
#define GPIO_PinSource14           ((u8)0x0E)
#define GPIO_PinSource15           ((u8)0x0F)

#define IS_GPIO_PIN_SOURCE(PINSOURCE) (((PINSOURCE) == GPIO_PinSource0) || \
                                       ((PINSOURCE) == GPIO_PinSource1) || \
                                       ((PINSOURCE) == GPIO_PinSource2) || \
                                       ((PINSOURCE) == GPIO_PinSource3) || \
                                       ((PINSOURCE) == GPIO_PinSource4) || \
                                       ((PINSOURCE) == GPIO_PinSource5) || \
                                       ((PINSOURCE) == GPIO_PinSource6) || \
                                       ((PINSOURCE) == GPIO_PinSource7) || \
                                       ((PINSOURCE) == GPIO_PinSource8) || \
                                       ((PINSOURCE) == GPIO_PinSource9) || \
                                       ((PINSOURCE) == GPIO_PinSource10) || \
                                       ((PINSOURCE) == GPIO_PinSource11) || \
                                       ((PINSOURCE) == GPIO_PinSource12) || \
                                       ((PINSOURCE) == GPIO_PinSource13) || \
                                       ((PINSOURCE) == GPIO_PinSource14) || \
                                       ((PINSOURCE) == GPIO_PinSource15))
                          
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
u8 GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
u16 GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
u8 GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
u16 GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, u16 PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
void GPIO_EventOutputConfig(u8 GPIO_PortSource, u8 GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(u32 GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(u8 GPIO_PortSource, u8 GPIO_PinSource);

#endif /* __STM32F10x_GPIO_H */















#ifndef __STM32F10x_TYPE_H
#define __STM32F10x_TYPE_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;  /* Read Only */
typedef signed short const sc16;  /* Read Only */
typedef signed char  const sc8;   /* Read Only */

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;  /* Read Only */
typedef volatile signed short const vsc16;  /* Read Only */
typedef volatile signed char  const vsc8;   /* Read Only */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;  /* Read Only */
typedef unsigned short const uc16;  /* Read Only */
typedef unsigned char  const uc8;   /* Read Only */

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;  /* Read Only */
typedef volatile unsigned short const vuc16;  /* Read Only */
typedef volatile unsigned char  const vuc8;   /* Read Only */

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10x_TYPE_H */














/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_TYPE_H
#define __VN_TYPE_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Pin state */
typedef enum {VN_PIN_LOW = 0, VN_PIN_HIGH = !VN_PIN_LOW} VN_PinState;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __VN_TYPE_H */


















/* Bit mask to get the nth byte out of a 32-bit word where n=0 is most significant and n=3 is least significant */
#define VN_BYTE(word, n)   ((unsigned char)((word & (0x000000FF << (n*8))) >> (n*8)))



/* SPI_I2S flags definition */
#define SPI_I2S_FLAG_RXNE               ((u16)0x0001)
#define SPI_I2S_FLAG_TXE                ((u16)0x0002)
#define I2S_FLAG_CHSIDE                 ((u16)0x0004)
#define I2S_FLAG_UDR                    ((u16)0x0008)
#define SPI_FLAG_CRCERR                 ((u16)0x0010)
#define SPI_FLAG_MODF                   ((u16)0x0020)
#define SPI_I2S_FLAG_OVR                ((u16)0x0040)
#define SPI_I2S_FLAG_BSY                ((u16)0x0080)

/* #define IS_SPI_I2S_CLEAR_FLAG(FLAG) (((FLAG) == SPI_FLAG_CRCERR))

#define IS_SPI_I2S_GET_FLAG(FLAG) (((FLAG) == SPI_I2S_FLAG_BSY) || ((FLAG) == SPI_I2S_FLAG_OVR) || \
                                   ((FLAG) == SPI_FLAG_MODF) || ((FLAG) == SPI_FLAG_CRCERR) || \
                                   ((FLAG) == I2S_FLAG_UDR) || ((FLAG) == I2S_FLAG_CHSIDE) || \
                                   ((FLAG) == SPI_I2S_FLAG_TXE) || ((FLAG) == SPI_I2S_FLAG_RXNE)) */













/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SPI_H
#define __STM32F10x_SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* SPI Init structure definition */
typedef struct
{
  u16 SPI_Direction;
  u16 SPI_Mode;
  u16 SPI_DataSize;
  u16 SPI_CPOL;
  u16 SPI_CPHA;
  u16 SPI_NSS;
  u16 SPI_BaudRatePrescaler;
  u16 SPI_FirstBit;
  u16 SPI_CRCPolynomial;
}SPI_InitTypeDef;

/* I2S Init structure definition */
typedef struct
{
  u16 I2S_Mode;
  u16 I2S_Standard;
  u16 I2S_DataFormat;
  u16 I2S_MCLKOutput;
  u16 I2S_AudioFreq;
  u16 I2S_CPOL;
}I2S_InitTypeDef;

/* Exported constants --------------------------------------------------------*/

#define IS_SPI_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == SPI1_BASE) || \
                                   ((*(u32*)&(PERIPH)) == SPI2_BASE) || \
                                   ((*(u32*)&(PERIPH)) == SPI3_BASE))

#define IS_SPI_23_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == SPI2_BASE) || \
                                  ((*(u32*)&(PERIPH)) == SPI3_BASE))

/* SPI data direction mode */
#define SPI_Direction_2Lines_FullDuplex ((u16)0x0000)
#define SPI_Direction_2Lines_RxOnly     ((u16)0x0400)
#define SPI_Direction_1Line_Rx          ((u16)0x8000)
#define SPI_Direction_1Line_Tx          ((u16)0xC000)

#define IS_SPI_DIRECTION_MODE(MODE) (((MODE) == SPI_Direction_2Lines_FullDuplex) || \
                                     ((MODE) == SPI_Direction_2Lines_RxOnly) || \
                                     ((MODE) == SPI_Direction_1Line_Rx) || \
                                     ((MODE) == SPI_Direction_1Line_Tx))

/* SPI master/slave mode */
#define SPI_Mode_Master                 ((u16)0x0104)
#define SPI_Mode_Slave                  ((u16)0x0000)

#define IS_SPI_MODE(MODE) (((MODE) == SPI_Mode_Master) || \
                           ((MODE) == SPI_Mode_Slave))

/* SPI data size */
#define SPI_DataSize_16b                ((u16)0x0800)
#define SPI_DataSize_8b                 ((u16)0x0000)

#define IS_SPI_DATASIZE(DATASIZE) (((DATASIZE) == SPI_DataSize_16b) || \
                                   ((DATASIZE) == SPI_DataSize_8b))

/* SPI Clock Polarity */
#define SPI_CPOL_Low                    ((u16)0x0000)
#define SPI_CPOL_High                   ((u16)0x0002)

#define IS_SPI_CPOL(CPOL) (((CPOL) == SPI_CPOL_Low) || \
                           ((CPOL) == SPI_CPOL_High))

/* SPI Clock Phase */
#define SPI_CPHA_1Edge                  ((u16)0x0000)
#define SPI_CPHA_2Edge                  ((u16)0x0001)

#define IS_SPI_CPHA(CPHA) (((CPHA) == SPI_CPHA_1Edge) || \
                           ((CPHA) == SPI_CPHA_2Edge))

/* SPI Slave Select management */
#define SPI_NSS_Soft                    ((u16)0x0200)
#define SPI_NSS_Hard                    ((u16)0x0000)

#define IS_SPI_NSS(NSS) (((NSS) == SPI_NSS_Soft) || \
                         ((NSS) == SPI_NSS_Hard))                         

/* SPI BaudRate Prescaler  */
#define SPI_BaudRatePrescaler_2         ((u16)0x0000)
#define SPI_BaudRatePrescaler_4         ((u16)0x0008)
#define SPI_BaudRatePrescaler_8         ((u16)0x0010)
#define SPI_BaudRatePrescaler_16        ((u16)0x0018)
#define SPI_BaudRatePrescaler_32        ((u16)0x0020)
#define SPI_BaudRatePrescaler_64        ((u16)0x0028)
#define SPI_BaudRatePrescaler_128       ((u16)0x0030)
#define SPI_BaudRatePrescaler_256       ((u16)0x0038)

#define IS_SPI_BAUDRATE_PRESCALER(PRESCALER) (((PRESCALER) == SPI_BaudRatePrescaler_2) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_4) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_8) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_16) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_32) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_64) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_128) || \
                                              ((PRESCALER) == SPI_BaudRatePrescaler_256))

/* SPI MSB/LSB transmission */
#define SPI_FirstBit_MSB                ((u16)0x0000)
#define SPI_FirstBit_LSB                ((u16)0x0080)

#define IS_SPI_FIRST_BIT(BIT) (((BIT) == SPI_FirstBit_MSB) || \
                               ((BIT) == SPI_FirstBit_LSB))

/* I2S Mode */
#define I2S_Mode_SlaveTx                ((u16)0x0000)
#define I2S_Mode_SlaveRx                ((u16)0x0100)
#define I2S_Mode_MasterTx               ((u16)0x0200)
#define I2S_Mode_MasterRx               ((u16)0x0300)

#define IS_I2S_MODE(MODE) (((MODE) == I2S_Mode_SlaveTx) || \
                           ((MODE) == I2S_Mode_SlaveRx) || \
                           ((MODE) == I2S_Mode_MasterTx) || \
                           ((MODE) == I2S_Mode_MasterRx) )

/* I2S Standard */
#define I2S_Standard_Phillips           ((u16)0x0000)
#define I2S_Standard_MSB                ((u16)0x0010)
#define I2S_Standard_LSB                ((u16)0x0020)
#define I2S_Standard_PCMShort           ((u16)0x0030)
#define I2S_Standard_PCMLong            ((u16)0x00B0)

#define IS_I2S_STANDARD(STANDARD) (((STANDARD) == I2S_Standard_Phillips) || \
                                   ((STANDARD) == I2S_Standard_MSB) || \
                                   ((STANDARD) == I2S_Standard_LSB) || \
                                   ((STANDARD) == I2S_Standard_PCMShort) || \
                                   ((STANDARD) == I2S_Standard_PCMLong))

/* I2S Data Format */
#define I2S_DataFormat_16b              ((u16)0x0000)
#define I2S_DataFormat_16bextended      ((u16)0x0001)
#define I2S_DataFormat_24b              ((u16)0x0003)
#define I2S_DataFormat_32b              ((u16)0x0005)

#define IS_I2S_DATA_FORMAT(FORMAT) (((FORMAT) == I2S_DataFormat_16b) || \
                                    ((FORMAT) == I2S_DataFormat_16bextended) || \
                                    ((FORMAT) == I2S_DataFormat_24b) || \
                                    ((FORMAT) == I2S_DataFormat_32b))

/* I2S MCLK Output */ 
#define I2S_MCLKOutput_Enable           ((u16)0x0200)
#define I2S_MCLKOutput_Disable          ((u16)0x0000)

#define IS_I2S_MCLK_OUTPUT(OUTPUT) (((OUTPUT) == I2S_MCLKOutput_Enable) || \
                                    ((OUTPUT) == I2S_MCLKOutput_Disable))

/* I2S Audio Frequency */
#define I2S_AudioFreq_48k                ((u16)48000)
#define I2S_AudioFreq_44k                ((u16)44100)
#define I2S_AudioFreq_22k                ((u16)22050)
#define I2S_AudioFreq_16k                ((u16)16000)
#define I2S_AudioFreq_8k                 ((u16)8000)
#define I2S_AudioFreq_Default            ((u16)2)

#define IS_I2S_AUDIO_FREQ(FREQ) (((FREQ) == I2S_AudioFreq_48k) || \
                                 ((FREQ) == I2S_AudioFreq_44k) || \
                                 ((FREQ) == I2S_AudioFreq_22k) || \
                                 ((FREQ) == I2S_AudioFreq_16k) || \
                                 ((FREQ) == I2S_AudioFreq_8k)  || \
                                 ((FREQ) == I2S_AudioFreq_Default))

/* I2S Clock Polarity */
#define I2S_CPOL_Low                    ((u16)0x0000)
#define I2S_CPOL_High                   ((u16)0x0008)

#define IS_I2S_CPOL(CPOL) (((CPOL) == I2S_CPOL_Low) || \
                           ((CPOL) == I2S_CPOL_High))

/* SPI_I2S DMA transfer requests */
#define SPI_I2S_DMAReq_Tx               ((u16)0x0002)
#define SPI_I2S_DMAReq_Rx               ((u16)0x0001)

#define IS_SPI_I2S_DMAREQ(DMAREQ) ((((DMAREQ) & (u16)0xFFFC) == 0x00) && ((DMAREQ) != 0x00))

/* SPI NSS internal software mangement */
#define SPI_NSSInternalSoft_Set         ((u16)0x0100)
#define SPI_NSSInternalSoft_Reset       ((u16)0xFEFF)

#define IS_SPI_NSS_INTERNAL(INTERNAL) (((INTERNAL) == SPI_NSSInternalSoft_Set) || \
                                       ((INTERNAL) == SPI_NSSInternalSoft_Reset))

/* SPI CRC Transmit/Receive */
#define SPI_CRC_Tx                      ((u8)0x00)
#define SPI_CRC_Rx                      ((u8)0x01)

#define IS_SPI_CRC(CRC) (((CRC) == SPI_CRC_Tx) || ((CRC) == SPI_CRC_Rx))

/* SPI direction transmit/receive */
#define SPI_Direction_Rx                ((u16)0xBFFF)
#define SPI_Direction_Tx                ((u16)0x4000)

#define IS_SPI_DIRECTION(DIRECTION) (((DIRECTION) == SPI_Direction_Rx) || \
                                     ((DIRECTION) == SPI_Direction_Tx))

/* SPI_I2S interrupts definition */
#define SPI_I2S_IT_TXE                  ((u8)0x71)
#define SPI_I2S_IT_RXNE                 ((u8)0x60)
#define SPI_I2S_IT_ERR                  ((u8)0x50)

#define IS_SPI_I2S_CONFIG_IT(IT) (((IT) == SPI_I2S_IT_TXE) || \
                                 ((IT) == SPI_I2S_IT_RXNE) || \
                                 ((IT) == SPI_I2S_IT_ERR))

#define SPI_I2S_IT_OVR                  ((u8)0x56)
#define SPI_IT_MODF                     ((u8)0x55)
#define SPI_IT_CRCERR                   ((u8)0x54)
#define I2S_IT_UDR                      ((u8)0x53)

#define IS_SPI_I2S_CLEAR_IT(IT) (((IT) == SPI_IT_CRCERR))

#define IS_SPI_I2S_GET_IT(IT) (((IT) == SPI_I2S_IT_RXNE) || ((IT) == SPI_I2S_IT_TXE) || \
                               ((IT) == I2S_IT_UDR) || ((IT) == SPI_IT_CRCERR) || \
                               ((IT) == SPI_IT_MODF) || ((IT) == SPI_I2S_IT_OVR))

/* SPI_I2S flags definition */
#define SPI_I2S_FLAG_RXNE               ((u16)0x0001)
#define SPI_I2S_FLAG_TXE                ((u16)0x0002)
#define I2S_FLAG_CHSIDE                 ((u16)0x0004)
#define I2S_FLAG_UDR                    ((u16)0x0008)
#define SPI_FLAG_CRCERR                 ((u16)0x0010)
#define SPI_FLAG_MODF                   ((u16)0x0020)
#define SPI_I2S_FLAG_OVR                ((u16)0x0040)
#define SPI_I2S_FLAG_BSY                ((u16)0x0080)

#define IS_SPI_I2S_CLEAR_FLAG(FLAG) (((FLAG) == SPI_FLAG_CRCERR))

#define IS_SPI_I2S_GET_FLAG(FLAG) (((FLAG) == SPI_I2S_FLAG_BSY) || ((FLAG) == SPI_I2S_FLAG_OVR) || \
                                   ((FLAG) == SPI_FLAG_MODF) || ((FLAG) == SPI_FLAG_CRCERR) || \
                                   ((FLAG) == I2S_FLAG_UDR) || ((FLAG) == I2S_FLAG_CHSIDE) || \
                                   ((FLAG) == SPI_I2S_FLAG_TXE) || ((FLAG) == SPI_I2S_FLAG_RXNE))

/* SPI CRC polynomial --------------------------------------------------------*/
#define IS_SPI_CRC_POLYNOMIAL(POLYNOMIAL) ((POLYNOMIAL) >= 0x1)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, u8 SPI_I2S_IT, FunctionalState NewState);
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, u16 SPI_I2S_DMAReq, FunctionalState NewState);
void SPI_I2S_SendData(SPI_TypeDef* SPIx, u16 Data);
u16 SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, u16 SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, u16 SPI_DataSize);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
u16 SPI_GetCRC(SPI_TypeDef* SPIx, u8 SPI_CRC);
u16 SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, u16 SPI_Direction);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, u16 SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, u16 SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, u8 SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, u8 SPI_I2S_IT);

#endif /*__STM32F10x_SPI_H */
















/* verified for VN_SPI_SetSS function */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BRR = GPIO_Pin;
}










/* verified */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BSRR = GPIO_Pin;
}













typedef int VN_PIN_STATE;
/* typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef signed int s32;
typedef unsigned int u32;
typedef signed long s64;
typedef unsigned long u64;
typedef int FlagStatus;
typedef int ITStatus; */


/* 32-bit Parameter Type */
typedef union {
unsigned long UInt;
float Float;
} VN100_Param;


/* SPI Response Packet */
typedef struct {
unsigned char ZeroByte;
unsigned char CmdID;
unsigned char RegID;
unsigned char ErrID;
VN100_Param Data[VN100_SPI_BUFFER_SIZE];
} VN100_SPI_Packet;




/* verified for SendReceive function */
void SPI_I2S_SendData(SPI_TypeDef* SPIx, u16 Data)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Write in the DR register the data to be sent */
  SPIx->DR = Data;
}







/* verified for SendReceive function */
u16 SPI_I2S_ReceiveData(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Return the data in the DR register */
  return SPIx->DR;
}











/* verified */
void VN_Delay(unsigned long delay_uS){

unsigned long i;
  for(i=delay_uS*10; i--; );
}










/* to be verified for VN100_SPI_ReadRegister function */
void VN_SPI_SetSS(unsigned char sensorID, VN_PinState state){

/* User code to set SPI SS lines goes here. */   
  switch(sensorID){
  
    case 0:
      if(state == VN_PIN_LOW){
        /* Start SPI Transaction - Pull SPI CS line low */
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
      }else{
        /* End SPI transaction - Pull SPI CS line high */
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
      }
      break;
  }
}











/* verified for VN_SPI_SendReceive function */
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, u16 SPI_I2S_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_I2S_GET_FLAG(SPI_I2S_FLAG));

  /* Check the status of the specified SPI/I2S flag */
  if ((SPIx->SR & SPI_I2S_FLAG) != (u16)RESET)
  {
    /* SPI_I2S_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_I2S_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the SPI_I2S_FLAG status */
  return  bitstatus;
}












/* verified for readregister */
unsigned long VN_SPI_SendReceive(unsigned long data){

/* User code to send out 4 bytes over SPI goes here */
  unsigned long i;
  unsigned long ret = 0;
  
  for(i=0;i<4;i++){/* i don't know why, but the upper limit of i should be 8 for 64 bit operation(unsigned long is 64 bits in 64 bits OS) */
    /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
    /* Send SPI1 requests */
    SPI_I2S_SendData(SPI1, VN_BYTE(data, i));
  
    /* Wait for response from VN-100 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    /* Save received data in buffer */
    ret |= ((unsigned long)SPI_I2S_ReceiveData(SPI1) << (8*i));    
  }
  
  return ret;
}










/* verified for getting values from ypr register */
VN100_SPI_Packet* VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth){


/*  regID = 8; */
/*  regWidth = 3; */

  unsigned long i;

  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);/* function included; arguments defined */

  /* Send request */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_ReadRegister));
  VN_SPI_SendReceive(0);

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50us */
  VN_Delay(100);

  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  for(i=0;i<=regWidth;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);  


  /* Return Error code */
  return &VN_SPI_LastReceivedPacket;  
}













/* to be used in main */

int main()
{
/* Get the yaw, pitch, and roll*/
VN100_SPI_Packet *packet1;/* struct inc */
VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {0}}; /* initiation */
float yaw, pitch, roll;
packet1 = VN100_SPI_GetYPR(0, &yaw, &pitch, &roll);/* function included; arg def */



/* Read model number register */
VN100_SPI_Packet *packet2;/* struct inc */
packet2 = VN100_SPI_ReadRegister(0, 8, 3);/* fn inc; arg def */


getch();
return 0;
}
