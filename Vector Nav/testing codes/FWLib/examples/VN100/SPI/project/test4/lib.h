/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_lib.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file includes the device header files for the user
*                    : application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_LIB_H
#define __VN_LIB_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"
#include "VN_math.h"
#include "VN_user.h"

#ifdef _VN100
  #include "VN100.h"
#endif /*_VN100 */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Convert 4 bytes to a 32-bit word in the order given with b1 as the most significant byte*/
#define VN_BYTES2WORD(b1, b2, b3, b4) (((unsigned long)(b1) << 24) | ((unsigned long)(b2) << 16) | ((unsigned long)(b3) << 8) | (unsigned long)(b4))

/* Bit mask to get the 1st byte (most significant) out of a 32-bit word */
#define VN_BYTE1(word)    ((unsigned char)(((word) & 0xFF000000) >> 24))

/* Bit mask to get the 2nd byte out of a 32-bit word */
#define VN_BYTE2(word)    ((unsigned char)(((word) & 0x00FF0000) >> 16))

/* Bit mask to get the 3rd byte (most significant) out of a 32-bit word */
#define VN_BYTE3(word)    ((unsigned char)(((word) & 0x0000FF00) >> 8))

/* Bit mask to get the 1st byte (least significant) out of a 32-bit word */
#define VN_BYTE4(word)    ((unsigned char)((word) & 0x000000FF))

/* Bit mask to get the nth byte out of a 32-bit word where n=0 is most significant and n=3 is least significant */
#define VN_BYTE(word, n)   ((unsigned char)((word & (0x000000FF << (n*8))) >> (n*8)))

/* Exported functions ------------------------------------------------------- */





#endif /* __VN_LIB_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : cortexm3_macro.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Header file for cortexm3_macro.s.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CORTEXM3_MACRO_H
#define __CORTEXM3_MACRO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void __WFI(void);
void __WFE(void);
void __SEV(void);
void __ISB(void);
void __DSB(void);
void __DMB(void);
void __SVC(void);
u32 __MRS_CONTROL(void);
void __MSR_CONTROL(u32 Control);
u32 __MRS_PSP(void);
void __MSR_PSP(u32 TopOfProcessStack);
u32 __MRS_MSP(void);
void __MSR_MSP(u32 TopOfMainStack);
void __RESETPRIMASK(void);
void __SETPRIMASK(void);
u32 __READ_PRIMASK(void);
void __RESETFAULTMASK(void);
void __SETFAULTMASK(void);
u32 __READ_FAULTMASK(void);
void __BASEPRICONFIG(u32 NewPriority);
u32 __GetBASEPRI(void);
u16 __REV_HalfWord(u16 Data);
u32 __REV_Word(u32 Data);

#endif /* __CORTEXM3_MACRO_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_adc.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      ADC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_ADC_H
#define __STM32F10x_ADC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* ADC Init structure definition */
typedef struct
{
  u32 ADC_Mode;
  FunctionalState ADC_ScanConvMode; 
  FunctionalState ADC_ContinuousConvMode;
  u32 ADC_ExternalTrigConv;
  u32 ADC_DataAlign;
  u8 ADC_NbrOfChannel;
}ADC_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
#define IS_ADC_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == ADC1_BASE) || \
                                   ((*(u32*)&(PERIPH)) == ADC2_BASE) || \
                                   ((*(u32*)&(PERIPH)) == ADC3_BASE))
                                 
#define IS_ADC_DMA_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == ADC1_BASE) || \
                                   ((*(u32*)&(PERIPH)) == ADC3_BASE))

/* ADC dual mode -------------------------------------------------------------*/
#define ADC_Mode_Independent                       ((u32)0x00000000)
#define ADC_Mode_RegInjecSimult                    ((u32)0x00010000)
#define ADC_Mode_RegSimult_AlterTrig               ((u32)0x00020000)
#define ADC_Mode_InjecSimult_FastInterl            ((u32)0x00030000)
#define ADC_Mode_InjecSimult_SlowInterl            ((u32)0x00040000)
#define ADC_Mode_InjecSimult                       ((u32)0x00050000)
#define ADC_Mode_RegSimult                         ((u32)0x00060000)
#define ADC_Mode_FastInterl                        ((u32)0x00070000)
#define ADC_Mode_SlowInterl                        ((u32)0x00080000)
#define ADC_Mode_AlterTrig                         ((u32)0x00090000)

#define IS_ADC_MODE(MODE) (((MODE) == ADC_Mode_Independent) || \
                           ((MODE) == ADC_Mode_RegInjecSimult) || \
                           ((MODE) == ADC_Mode_RegSimult_AlterTrig) || \
                           ((MODE) == ADC_Mode_InjecSimult_FastInterl) || \
                           ((MODE) == ADC_Mode_InjecSimult_SlowInterl) || \
                           ((MODE) == ADC_Mode_InjecSimult) || \
                           ((MODE) == ADC_Mode_RegSimult) || \
                           ((MODE) == ADC_Mode_FastInterl) || \
                           ((MODE) == ADC_Mode_SlowInterl) || \
                           ((MODE) == ADC_Mode_AlterTrig))

/* ADC extrenal trigger sources for regular channels conversion --------------*/
/* for ADC1 and ADC2 */
#define ADC_ExternalTrigConv_T1_CC1                ((u32)0x00000000)
#define ADC_ExternalTrigConv_T1_CC2                ((u32)0x00020000)
#define ADC_ExternalTrigConv_T2_CC2                ((u32)0x00060000)
#define ADC_ExternalTrigConv_T3_TRGO               ((u32)0x00080000)
#define ADC_ExternalTrigConv_T4_CC4                ((u32)0x000A0000)
#define ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO    ((u32)0x000C0000)
/* for ADC1, ADC2 and ADC3 */
#define ADC_ExternalTrigConv_T1_CC3                ((u32)0x00040000)
#define ADC_ExternalTrigConv_None                  ((u32)0x000E0000)
/* for ADC3 */
#define ADC_ExternalTrigConv_T3_CC1                ((u32)0x00000000)
#define ADC_ExternalTrigConv_T2_CC3                ((u32)0x00020000)
#define ADC_ExternalTrigConv_T8_CC1                ((u32)0x00060000)
#define ADC_ExternalTrigConv_T8_TRGO               ((u32)0x00080000)
#define ADC_ExternalTrigConv_T5_CC1                ((u32)0x000A0000)
#define ADC_ExternalTrigConv_T5_CC3                ((u32)0x000C0000)

#define IS_ADC_EXT_TRIG(REGTRIG) (((REGTRIG) == ADC_ExternalTrigConv_T1_CC1) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T1_CC2) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T1_CC3) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T2_CC2) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T3_TRGO) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T4_CC4) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_None) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T3_CC1) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T2_CC3) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T8_CC1) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T8_TRGO) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T5_CC1) || \
                                  ((REGTRIG) == ADC_ExternalTrigConv_T5_CC3))

/* ADC data align ------------------------------------------------------------*/
#define ADC_DataAlign_Right                        ((u32)0x00000000)
#define ADC_DataAlign_Left                         ((u32)0x00000800)

#define IS_ADC_DATA_ALIGN(ALIGN) (((ALIGN) == ADC_DataAlign_Right) || \
                                  ((ALIGN) == ADC_DataAlign_Left))

/* ADC channels --------------------------------------------------------------*/
#define ADC_Channel_0                               ((u8)0x00)
#define ADC_Channel_1                               ((u8)0x01)
#define ADC_Channel_2                               ((u8)0x02)
#define ADC_Channel_3                               ((u8)0x03)
#define ADC_Channel_4                               ((u8)0x04)
#define ADC_Channel_5                               ((u8)0x05)
#define ADC_Channel_6                               ((u8)0x06)
#define ADC_Channel_7                               ((u8)0x07)
#define ADC_Channel_8                               ((u8)0x08)
#define ADC_Channel_9                               ((u8)0x09)
#define ADC_Channel_10                              ((u8)0x0A)
#define ADC_Channel_11                              ((u8)0x0B)
#define ADC_Channel_12                              ((u8)0x0C)
#define ADC_Channel_13                              ((u8)0x0D)
#define ADC_Channel_14                              ((u8)0x0E)
#define ADC_Channel_15                              ((u8)0x0F)
#define ADC_Channel_16                              ((u8)0x10)
#define ADC_Channel_17                              ((u8)0x11)

#define IS_ADC_CHANNEL(CHANNEL) (((CHANNEL) == ADC_Channel_0) || ((CHANNEL) == ADC_Channel_1) || \
                                 ((CHANNEL) == ADC_Channel_2) || ((CHANNEL) == ADC_Channel_3) || \
                                 ((CHANNEL) == ADC_Channel_4) || ((CHANNEL) == ADC_Channel_5) || \
                                 ((CHANNEL) == ADC_Channel_6) || ((CHANNEL) == ADC_Channel_7) || \
                                 ((CHANNEL) == ADC_Channel_8) || ((CHANNEL) == ADC_Channel_9) || \
                                 ((CHANNEL) == ADC_Channel_10) || ((CHANNEL) == ADC_Channel_11) || \
                                 ((CHANNEL) == ADC_Channel_12) || ((CHANNEL) == ADC_Channel_13) || \
                                 ((CHANNEL) == ADC_Channel_14) || ((CHANNEL) == ADC_Channel_15) || \
                                 ((CHANNEL) == ADC_Channel_16) || ((CHANNEL) == ADC_Channel_17))

/* ADC sampling times --------------------------------------------------------*/
#define ADC_SampleTime_1Cycles5                    ((u8)0x00)
#define ADC_SampleTime_7Cycles5                    ((u8)0x01)
#define ADC_SampleTime_13Cycles5                   ((u8)0x02)
#define ADC_SampleTime_28Cycles5                   ((u8)0x03)
#define ADC_SampleTime_41Cycles5                   ((u8)0x04)
#define ADC_SampleTime_55Cycles5                   ((u8)0x05)
#define ADC_SampleTime_71Cycles5                   ((u8)0x06)
#define ADC_SampleTime_239Cycles5                  ((u8)0x07)

#define IS_ADC_SAMPLE_TIME(TIME) (((TIME) == ADC_SampleTime_1Cycles5) || \
                                  ((TIME) == ADC_SampleTime_7Cycles5) || \
                                  ((TIME) == ADC_SampleTime_13Cycles5) || \
                                  ((TIME) == ADC_SampleTime_28Cycles5) || \
                                  ((TIME) == ADC_SampleTime_41Cycles5) || \
                                  ((TIME) == ADC_SampleTime_55Cycles5) || \
                                  ((TIME) == ADC_SampleTime_71Cycles5) || \
                                  ((TIME) == ADC_SampleTime_239Cycles5))

/* ADC extrenal trigger sources for injected channels conversion -------------*/
/* For ADC1 and ADC2 */
#define ADC_ExternalTrigInjecConv_T2_TRGO           ((u32)0x00002000)
#define ADC_ExternalTrigInjecConv_T2_CC1            ((u32)0x00003000)
#define ADC_ExternalTrigInjecConv_T3_CC4            ((u32)0x00004000)
#define ADC_ExternalTrigInjecConv_T4_TRGO           ((u32)0x00005000)
#define ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4 ((u32)0x00006000)
/* For ADC1, ADC2 and ADC3 */
#define ADC_ExternalTrigInjecConv_T1_TRGO           ((u32)0x00000000)
#define ADC_ExternalTrigInjecConv_T1_CC4            ((u32)0x00001000)
#define ADC_ExternalTrigInjecConv_None              ((u32)0x00007000)
/* For ADC3 */
#define ADC_ExternalTrigInjecConv_T4_CC3            ((u32)0x00002000)
#define ADC_ExternalTrigInjecConv_T8_CC2            ((u32)0x00003000)
#define ADC_ExternalTrigInjecConv_T8_CC4            ((u32)0x00004000)
#define ADC_ExternalTrigInjecConv_T5_TRGO           ((u32)0x00005000)
#define ADC_ExternalTrigInjecConv_T5_CC4            ((u32)0x00006000)

#define IS_ADC_EXT_INJEC_TRIG(INJTRIG) (((INJTRIG) == ADC_ExternalTrigInjecConv_T1_TRGO) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T1_CC4) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T2_TRGO) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T2_CC1) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T3_CC4) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T4_TRGO) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_None) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T4_CC3) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T8_CC2) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T8_CC4) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T5_TRGO) || \
                                        ((INJTRIG) == ADC_ExternalTrigInjecConv_T5_CC4))

/* ADC injected channel selection --------------------------------------------*/
#define ADC_InjectedChannel_1                       ((u8)0x14)
#define ADC_InjectedChannel_2                       ((u8)0x18)
#define ADC_InjectedChannel_3                       ((u8)0x1C)
#define ADC_InjectedChannel_4                       ((u8)0x20)

#define IS_ADC_INJECTED_CHANNEL(CHANNEL) (((CHANNEL) == ADC_InjectedChannel_1) || \
                                          ((CHANNEL) == ADC_InjectedChannel_2) || \
                                          ((CHANNEL) == ADC_InjectedChannel_3) || \
                                          ((CHANNEL) == ADC_InjectedChannel_4))

/* ADC analog watchdog selection ---------------------------------------------*/
#define ADC_AnalogWatchdog_SingleRegEnable         ((u32)0x00800200)
#define ADC_AnalogWatchdog_SingleInjecEnable       ((u32)0x00400200)
#define ADC_AnalogWatchdog_SingleRegOrInjecEnable  ((u32)0x00C00200)
#define ADC_AnalogWatchdog_AllRegEnable            ((u32)0x00800000)
#define ADC_AnalogWatchdog_AllInjecEnable          ((u32)0x00400000)
#define ADC_AnalogWatchdog_AllRegAllInjecEnable    ((u32)0x00C00000)
#define ADC_AnalogWatchdog_None                    ((u32)0x00000000)

#define IS_ADC_ANALOG_WATCHDOG(WATCHDOG) (((WATCHDOG) == ADC_AnalogWatchdog_SingleRegEnable) || \
                                          ((WATCHDOG) == ADC_AnalogWatchdog_SingleInjecEnable) || \
                                          ((WATCHDOG) == ADC_AnalogWatchdog_SingleRegOrInjecEnable) || \
                                          ((WATCHDOG) == ADC_AnalogWatchdog_AllRegEnable) || \
                                          ((WATCHDOG) == ADC_AnalogWatchdog_AllInjecEnable) || \
                                          ((WATCHDOG) == ADC_AnalogWatchdog_AllRegAllInjecEnable) || \
                                          ((WATCHDOG) == ADC_AnalogWatchdog_None))

/* ADC interrupts definition -------------------------------------------------*/
#define ADC_IT_EOC                                 ((u16)0x0220)
#define ADC_IT_AWD                                 ((u16)0x0140)
#define ADC_IT_JEOC                                ((u16)0x0480)

#define IS_ADC_IT(IT) ((((IT) & (u16)0xF81F) == 0x00) && ((IT) != 0x00))
#define IS_ADC_GET_IT(IT) (((IT) == ADC_IT_EOC) || ((IT) == ADC_IT_AWD) || \
                           ((IT) == ADC_IT_JEOC))

/* ADC flags definition ------------------------------------------------------*/
#define ADC_FLAG_AWD                               ((u8)0x01)
#define ADC_FLAG_EOC                               ((u8)0x02)
#define ADC_FLAG_JEOC                              ((u8)0x04)
#define ADC_FLAG_JSTRT                             ((u8)0x08)
#define ADC_FLAG_STRT                              ((u8)0x10)

#define IS_ADC_CLEAR_FLAG(FLAG) ((((FLAG) & (u8)0xE0) == 0x00) && ((FLAG) != 0x00))
#define IS_ADC_GET_FLAG(FLAG) (((FLAG) == ADC_FLAG_AWD) || ((FLAG) == ADC_FLAG_EOC) || \
                               ((FLAG) == ADC_FLAG_JEOC) || ((FLAG)== ADC_FLAG_JSTRT) || \
                               ((FLAG) == ADC_FLAG_STRT))

/* ADC thresholds ------------------------------------------------------------*/
#define IS_ADC_THRESHOLD(THRESHOLD) ((THRESHOLD) <= 0xFFF)

/* ADC injected offset -------------------------------------------------------*/
#define IS_ADC_OFFSET(OFFSET) ((OFFSET) <= 0xFFF)

/* ADC injected length -------------------------------------------------------*/
#define IS_ADC_INJECTED_LENGTH(LENGTH) (((LENGTH) >= 0x1) && ((LENGTH) <= 0x4))

/* ADC injected rank ---------------------------------------------------------*/
#define IS_ADC_INJECTED_RANK(RANK) (((RANK) >= 0x1) && ((RANK) <= 0x4))

/* ADC regular length --------------------------------------------------------*/
#define IS_ADC_REGULAR_LENGTH(LENGTH) (((LENGTH) >= 0x1) && ((LENGTH) <= 0x10))

/* ADC regular rank ----------------------------------------------------------*/
#define IS_ADC_REGULAR_RANK(RANK) (((RANK) >= 0x1) && ((RANK) <= 0x10))

/* ADC regular discontinuous mode number -------------------------------------*/
#define IS_ADC_REGULAR_DISC_NUMBER(NUMBER) (((NUMBER) >= 0x1) && ((NUMBER) <= 0x8))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, u16 ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, u8 Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, u8 ADC_Channel, u8 Rank, u8 ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
u16 ADC_GetConversionValue(ADC_TypeDef* ADCx);
u32 ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, u32 ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, u8 ADC_Channel, u8 Rank, u8 ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, u8 Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, u8 ADC_InjectedChannel, u16 Offset);
u16 ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, u8 ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, u32 ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, u16 HighThreshold, u16 LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, u8 ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, u8 ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, u8 ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, u16 ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, u16 ADC_IT);

#endif /*__STM32F10x_ADC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_bkp.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      BKP firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_BKP_H
#define __STM32F10x_BKP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Tamper Pin active level */
#define BKP_TamperPinLevel_High           ((u16)0x0000)
#define BKP_TamperPinLevel_Low            ((u16)0x0001)

#define IS_BKP_TAMPER_PIN_LEVEL(LEVEL) (((LEVEL) == BKP_TamperPinLevel_High) || \
                                        ((LEVEL) == BKP_TamperPinLevel_Low))

/* RTC output source to output on the Tamper pin */
#define BKP_RTCOutputSource_None          ((u16)0x0000)
#define BKP_RTCOutputSource_CalibClock    ((u16)0x0080)
#define BKP_RTCOutputSource_Alarm         ((u16)0x0100)
#define BKP_RTCOutputSource_Second        ((u16)0x0300)

#define IS_BKP_RTC_OUTPUT_SOURCE(SOURCE) (((SOURCE) == BKP_RTCOutputSource_None) || \
                                          ((SOURCE) == BKP_RTCOutputSource_CalibClock) || \
                                          ((SOURCE) == BKP_RTCOutputSource_Alarm) || \
                                          ((SOURCE) == BKP_RTCOutputSource_Second))

/* Data Backup Register */
#define BKP_DR1                           ((u16)0x0004)
#define BKP_DR2                           ((u16)0x0008)
#define BKP_DR3                           ((u16)0x000C)
#define BKP_DR4                           ((u16)0x0010)
#define BKP_DR5                           ((u16)0x0014)
#define BKP_DR6                           ((u16)0x0018)
#define BKP_DR7                           ((u16)0x001C)
#define BKP_DR8                           ((u16)0x0020)
#define BKP_DR9                           ((u16)0x0024)
#define BKP_DR10                          ((u16)0x0028)
#define BKP_DR11                          ((u16)0x0040)
#define BKP_DR12                          ((u16)0x0044)
#define BKP_DR13                          ((u16)0x0048)
#define BKP_DR14                          ((u16)0x004C)
#define BKP_DR15                          ((u16)0x0050)
#define BKP_DR16                          ((u16)0x0054)
#define BKP_DR17                          ((u16)0x0058)
#define BKP_DR18                          ((u16)0x005C)
#define BKP_DR19                          ((u16)0x0060)
#define BKP_DR20                          ((u16)0x0064)
#define BKP_DR21                          ((u16)0x0068)
#define BKP_DR22                          ((u16)0x006C)
#define BKP_DR23                          ((u16)0x0070)
#define BKP_DR24                          ((u16)0x0074)
#define BKP_DR25                          ((u16)0x0078)
#define BKP_DR26                          ((u16)0x007C)
#define BKP_DR27                          ((u16)0x0080)
#define BKP_DR28                          ((u16)0x0084)
#define BKP_DR29                          ((u16)0x0088)
#define BKP_DR30                          ((u16)0x008C)
#define BKP_DR31                          ((u16)0x0090)
#define BKP_DR32                          ((u16)0x0094)
#define BKP_DR33                          ((u16)0x0098)
#define BKP_DR34                          ((u16)0x009C)
#define BKP_DR35                          ((u16)0x00A0)
#define BKP_DR36                          ((u16)0x00A4)
#define BKP_DR37                          ((u16)0x00A8)
#define BKP_DR38                          ((u16)0x00AC)
#define BKP_DR39                          ((u16)0x00B0)
#define BKP_DR40                          ((u16)0x00B4)
#define BKP_DR41                          ((u16)0x00B8)
#define BKP_DR42                          ((u16)0x00BC)

#define IS_BKP_DR(DR) (((DR) == BKP_DR1)  || ((DR) == BKP_DR2)  || ((DR) == BKP_DR3)  || \
                       ((DR) == BKP_DR4)  || ((DR) == BKP_DR5)  || ((DR) == BKP_DR6)  || \
                       ((DR) == BKP_DR7)  || ((DR) == BKP_DR8)  || ((DR) == BKP_DR9)  || \
                       ((DR) == BKP_DR10) || ((DR) == BKP_DR11) || ((DR) == BKP_DR12) || \
                       ((DR) == BKP_DR13) || ((DR) == BKP_DR14) || ((DR) == BKP_DR15) || \
                       ((DR) == BKP_DR16) || ((DR) == BKP_DR17) || ((DR) == BKP_DR18) || \
                       ((DR) == BKP_DR19) || ((DR) == BKP_DR20) || ((DR) == BKP_DR21) || \
                       ((DR) == BKP_DR22) || ((DR) == BKP_DR23) || ((DR) == BKP_DR24) || \
                       ((DR) == BKP_DR25) || ((DR) == BKP_DR26) || ((DR) == BKP_DR27) || \
                       ((DR) == BKP_DR28) || ((DR) == BKP_DR29) || ((DR) == BKP_DR30) || \
                       ((DR) == BKP_DR31) || ((DR) == BKP_DR32) || ((DR) == BKP_DR33) || \
                       ((DR) == BKP_DR34) || ((DR) == BKP_DR35) || ((DR) == BKP_DR36) || \
                       ((DR) == BKP_DR37) || ((DR) == BKP_DR38) || ((DR) == BKP_DR39) || \
                       ((DR) == BKP_DR40) || ((DR) == BKP_DR41) || ((DR) == BKP_DR42))

#define IS_BKP_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x7F)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void BKP_DeInit(void);
void BKP_TamperPinLevelConfig(u16 BKP_TamperPinLevel);
void BKP_TamperPinCmd(FunctionalState NewState);
void BKP_ITConfig(FunctionalState NewState);
void BKP_RTCOutputConfig(u16 BKP_RTCOutputSource);
void BKP_SetRTCCalibrationValue(u8 CalibrationValue);
void BKP_WriteBackupRegister(u16 BKP_DR, u16 Data);
u16 BKP_ReadBackupRegister(u16 BKP_DR);
FlagStatus BKP_GetFlagStatus(void);
void BKP_ClearFlag(void);
ITStatus BKP_GetITStatus(void);
void BKP_ClearITPendingBit(void);

#endif /* __STM32F10x_BKP_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_can.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      CAN firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CAN_H
#define __STM32F10x_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* CAN init structure definition */
typedef struct
{
  FunctionalState CAN_TTCM;
  FunctionalState CAN_ABOM;
  FunctionalState CAN_AWUM;
  FunctionalState CAN_NART;
  FunctionalState CAN_RFLM;
  FunctionalState CAN_TXFP;
  u8 CAN_Mode;
  u8 CAN_SJW;
  u8 CAN_BS1;
  u8 CAN_BS2;
  u16 CAN_Prescaler;
} CAN_InitTypeDef;

/* CAN filter init structure definition */
typedef struct
{
  u8 CAN_FilterNumber;
  u8 CAN_FilterMode;
  u8 CAN_FilterScale;
  u16 CAN_FilterIdHigh;
  u16 CAN_FilterIdLow;
  u16 CAN_FilterMaskIdHigh;
  u16 CAN_FilterMaskIdLow;
  u16 CAN_FilterFIFOAssignment;
  FunctionalState CAN_FilterActivation;
} CAN_FilterInitTypeDef;

/* CAN Tx message structure definition */
typedef struct
{
  u32 StdId;
  u32 ExtId;
  u8 IDE;
  u8 RTR;
  u8 DLC;
  u8 Data[8];
} CanTxMsg;

/* CAN Rx message structure definition */
typedef struct
{
  u32 StdId;
  u32 ExtId;
  u8 IDE;
  u8 RTR;
  u8 DLC;
  u8 Data[8];
  u8 FMI;
} CanRxMsg;

/* Exported constants --------------------------------------------------------*/

/* CAN sleep constants */
#define CANINITFAILED              ((u8)0x00) /* CAN initialization failed */
#define CANINITOK                  ((u8)0x01) /* CAN initialization failed */

/* CAN operating mode */
#define CAN_Mode_Normal             ((u8)0x00)  /* normal mode */
#define CAN_Mode_LoopBack           ((u8)0x01)  /* loopback mode */
#define CAN_Mode_Silent             ((u8)0x02)  /* silent mode */
#define CAN_Mode_Silent_LoopBack    ((u8)0x03)  /* loopback combined with silent mode */

#define IS_CAN_MODE(MODE) (((MODE) == CAN_Mode_Normal) || ((MODE) == CAN_Mode_LoopBack)|| \
                           ((MODE) == CAN_Mode_Silent) || ((MODE) == CAN_Mode_Silent_LoopBack))

/* CAN synchronisation jump width */
#define CAN_SJW_1tq                 ((u8)0x00)  /* 1 time quantum */
#define CAN_SJW_2tq                 ((u8)0x01)  /* 2 time quantum */
#define CAN_SJW_3tq                 ((u8)0x02)  /* 3 time quantum */
#define CAN_SJW_4tq                 ((u8)0x03)  /* 4 time quantum */

#define IS_CAN_SJW(SJW) (((SJW) == CAN_SJW_1tq) || ((SJW) == CAN_SJW_2tq)|| \
                         ((SJW) == CAN_SJW_3tq) || ((SJW) == CAN_SJW_4tq))

/* time quantum in bit segment 1 */
#define CAN_BS1_1tq                 ((u8)0x00)  /* 1 time quantum */
#define CAN_BS1_2tq                 ((u8)0x01)  /* 2 time quantum */
#define CAN_BS1_3tq                 ((u8)0x02)  /* 3 time quantum */
#define CAN_BS1_4tq                 ((u8)0x03)  /* 4 time quantum */
#define CAN_BS1_5tq                 ((u8)0x04)  /* 5 time quantum */
#define CAN_BS1_6tq                 ((u8)0x05)  /* 6 time quantum */
#define CAN_BS1_7tq                 ((u8)0x06)  /* 7 time quantum */
#define CAN_BS1_8tq                 ((u8)0x07)  /* 8 time quantum */
#define CAN_BS1_9tq                 ((u8)0x08)  /* 9 time quantum */
#define CAN_BS1_10tq                ((u8)0x09)  /* 10 time quantum */
#define CAN_BS1_11tq                ((u8)0x0A)  /* 11 time quantum */
#define CAN_BS1_12tq                ((u8)0x0B)  /* 12 time quantum */
#define CAN_BS1_13tq                ((u8)0x0C)  /* 13 time quantum */
#define CAN_BS1_14tq                ((u8)0x0D)  /* 14 time quantum */
#define CAN_BS1_15tq                ((u8)0x0E)  /* 15 time quantum */
#define CAN_BS1_16tq                ((u8)0x0F)  /* 16 time quantum */

#define IS_CAN_BS1(BS1) ((BS1) <= CAN_BS1_16tq)

/* time quantum in bit segment 2 */
#define CAN_BS2_1tq                 ((u8)0x00)  /* 1 time quantum */
#define CAN_BS2_2tq                 ((u8)0x01)  /* 2 time quantum */
#define CAN_BS2_3tq                 ((u8)0x02)  /* 3 time quantum */
#define CAN_BS2_4tq                 ((u8)0x03)  /* 4 time quantum */
#define CAN_BS2_5tq                 ((u8)0x04)  /* 5 time quantum */
#define CAN_BS2_6tq                 ((u8)0x05)  /* 6 time quantum */
#define CAN_BS2_7tq                 ((u8)0x06)  /* 7 time quantum */
#define CAN_BS2_8tq                 ((u8)0x07)  /* 8 time quantum */

#define IS_CAN_BS2(BS2) ((BS2) <= CAN_BS2_8tq)

/* CAN clock prescaler */
#define IS_CAN_PRESCALER(PRESCALER) (((PRESCALER) >= 1) && ((PRESCALER) <= 1024))

/* CAN filter number */
#define IS_CAN_FILTER_NUMBER(NUMBER) ((NUMBER) <= 13)

/* CAN filter mode */
#define CAN_FilterMode_IdMask       ((u8)0x00)  /* id/mask mode */
#define CAN_FilterMode_IdList       ((u8)0x01)  /* identifier list mode */

#define IS_CAN_FILTER_MODE(MODE) (((MODE) == CAN_FilterMode_IdMask) || \
                                  ((MODE) == CAN_FilterMode_IdList))

/* CAN filter scale */
#define CAN_FilterScale_16bit       ((u8)0x00) /* 16-bit filter scale */
#define CAN_FilterScale_32bit       ((u8)0x01) /* 2-bit filter scale */

#define IS_CAN_FILTER_SCALE(SCALE) (((SCALE) == CAN_FilterScale_16bit) || \
                                    ((SCALE) == CAN_FilterScale_32bit))

/* CAN filter FIFO assignation */
#define CAN_FilterFIFO0             ((u8)0x00)  /* Filter FIFO 0 assignment for filter x */
#define CAN_FilterFIFO1             ((u8)0x01)  /* Filter FIFO 1 assignment for filter x */

#define IS_CAN_FILTER_FIFO(FIFO) (((FIFO) == CAN_FilterFIFO0) || \
                                  ((FIFO) == CAN_FilterFIFO1))

/* CAN Tx */
#define IS_CAN_TRANSMITMAILBOX(TRANSMITMAILBOX) ((TRANSMITMAILBOX) <= ((u8)0x02))
#define IS_CAN_STDID(STDID)   ((STDID) <= ((u32)0x7FF))
#define IS_CAN_EXTID(EXTID)   ((EXTID) <= ((u32)0x1FFFFFFF))
#define IS_CAN_DLC(DLC)       ((DLC) <= ((u8)0x08))

/* CAN identifier type */
#define CAN_ID_STD                 ((u32)0x00000000)  /* Standard Id */
#define CAN_ID_EXT                 ((u32)0x00000004)  /* Extended Id */

#define IS_CAN_IDTYPE(IDTYPE) (((IDTYPE) == CAN_ID_STD) || ((IDTYPE) == CAN_ID_EXT))

/* CAN remote transmission request */
#define CAN_RTR_DATA                ((u32)0x00000000)  /* Data frame */
#define CAN_RTR_REMOTE              ((u32)0x00000002)  /* Remote frame */

#define IS_CAN_RTR(RTR) (((RTR) == CAN_RTR_DATA) || ((RTR) == CAN_RTR_REMOTE))

/* CAN transmit constants */
#define CANTXFAILED                 ((u8)0x00) /* CAN transmission failed */
#define CANTXOK                     ((u8)0x01) /* CAN transmission succeeded */
#define CANTXPENDING                ((u8)0x02) /* CAN transmission pending */
#define CAN_NO_MB                   ((u8)0x04) /* CAN cell did not provide an empty mailbox */

/* CAN receive FIFO number constants */
#define CAN_FIFO0                 ((u8)0x00) /* CAN FIFO0 used to receive */
#define CAN_FIFO1                 ((u8)0x01) /* CAN FIFO1 used to receive */

#define IS_CAN_FIFO(FIFO) (((FIFO) == CAN_FIFO0) || ((FIFO) == CAN_FIFO1))

/* CAN sleep constants */
#define CANSLEEPFAILED              ((u8)0x00) /* CAN did not enter the sleep mode */
#define CANSLEEPOK                  ((u8)0x01) /* CAN entered the sleep mode */

/* CAN wake up constants */
#define CANWAKEUPFAILED             ((u8)0x00) /* CAN did not leave the sleep mode */
#define CANWAKEUPOK                 ((u8)0x01) /* CAN leaved the sleep mode */

/* CAN flags */
#define CAN_FLAG_EWG                ((u32)0x00000001) /* Error Warning Flag */
#define CAN_FLAG_EPV                ((u32)0x00000002) /* Error Passive Flag */
#define CAN_FLAG_BOF                ((u32)0x00000004) /* Bus-Off Flag */

#define IS_CAN_FLAG(FLAG) (((FLAG) == CAN_FLAG_EWG) || ((FLAG) == CAN_FLAG_EPV) ||\
                           ((FLAG) == CAN_FLAG_BOF))

/* CAN interrupts */
#define CAN_IT_RQCP0                ((u32)0x00000005) /* Request completed mailbox 0 */
#define CAN_IT_RQCP1                ((u32)0x00000006) /* Request completed mailbox 1 */
#define CAN_IT_RQCP2                ((u32)0x00000007) /* Request completed mailbox 2 */
#define CAN_IT_TME                  ((u32)0x00000001) /* Transmit mailbox empty */
#define CAN_IT_FMP0                 ((u32)0x00000002) /* FIFO 0 message pending */
#define CAN_IT_FF0                  ((u32)0x00000004) /* FIFO 0 full */
#define CAN_IT_FOV0                 ((u32)0x00000008) /* FIFO 0 overrun */
#define CAN_IT_FMP1                 ((u32)0x00000010) /* FIFO 1 message pending */
#define CAN_IT_FF1                  ((u32)0x00000020) /* FIFO 1 full */
#define CAN_IT_FOV1                 ((u32)0x00000040) /* FIFO 1 overrun */
#define CAN_IT_EWG                  ((u32)0x00000100) /* Error warning */
#define CAN_IT_EPV                  ((u32)0x00000200) /* Error passive */
#define CAN_IT_BOF                  ((u32)0x00000400) /* Bus-off */
#define CAN_IT_LEC                  ((u32)0x00000800) /* Last error code */
#define CAN_IT_ERR                  ((u32)0x00008000) /* Error */
#define CAN_IT_WKU                  ((u32)0x00010000) /* Wake-up */
#define CAN_IT_SLK                  ((u32)0x00020000) /* Sleep */

#define IS_CAN_ITConfig(IT) (((IT) == CAN_IT_TME)   || ((IT) == CAN_IT_FMP0)  ||\
                             ((IT) == CAN_IT_FF0)   || ((IT) == CAN_IT_FOV0)  ||\
                             ((IT) == CAN_IT_FMP1)  || ((IT) == CAN_IT_FF1)   ||\
                             ((IT) == CAN_IT_FOV1)  || ((IT) == CAN_IT_EWG)   ||\
                             ((IT) == CAN_IT_EPV)   || ((IT) == CAN_IT_BOF)   ||\
                             ((IT) == CAN_IT_LEC)   || ((IT) == CAN_IT_ERR)   ||\
                             ((IT) == CAN_IT_WKU)   || ((IT) == CAN_IT_SLK))

#define IS_CAN_ITStatus(IT) (((IT) == CAN_IT_RQCP0)  || ((IT) == CAN_IT_RQCP1)  ||\
                             ((IT) == CAN_IT_RQCP2)  || ((IT) == CAN_IT_FF0)    ||\
                             ((IT) == CAN_IT_FOV0)   || ((IT) == CAN_IT_FF1)    ||\
                             ((IT) == CAN_IT_FOV1)   || ((IT) == CAN_IT_EWG)    ||\
                             ((IT) == CAN_IT_EPV)    || ((IT) == CAN_IT_BOF)    ||\
                             ((IT) == CAN_IT_WKU)    || ((IT) == CAN_IT_SLK))

/* Exported macro ------------------------------------------------------------*/
/* Exported function protypes ----------------------------------------------- */
void CAN_DeInit(void);
u8 CAN_Init(CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_ITConfig(u32 CAN_IT, FunctionalState NewState);
u8 CAN_Transmit(CanTxMsg* TxMessage);
u8 CAN_TransmitStatus(u8 TransmitMailbox);
void CAN_CancelTransmit(u8 Mailbox);
void CAN_FIFORelease(u8 FIFONumber);
u8 CAN_MessagePending(u8 FIFONumber);
void CAN_Receive(u8 FIFONumber, CanRxMsg* RxMessage);
u8 CAN_Sleep(void);
u8 CAN_WakeUp(void);
FlagStatus CAN_GetFlagStatus(u32 CAN_FLAG);
void CAN_ClearFlag(u32 CAN_FLAG);
ITStatus CAN_GetITStatus(u32 CAN_IT);
void CAN_ClearITPendingBit(u32 CAN_IT);

#endif /* __STM32F10x_CAN_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_conf.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Library configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to compile the library in DEBUG mode, this will expanse
   the "assert_param" macro in the firmware library code (see "Exported macro"
   section below) */
#define DEBUG    1

/* Comment the line below to disable the specific peripheral inclusion */
/************************************* ADC ************************************/
/*
#define _ADC
#define _ADC1
#define _ADC2
#define _ADC3
*/
/************************************* BKP ************************************/
/*
#define _BKP
*/
/************************************* CAN ************************************/
/* 
#define _CAN
*/
/************************************* CRC ************************************/
/*
#define _CRC
*/
/************************************* DAC ************************************/
/*
#define _DAC
*/
/************************************* DBGMCU *********************************/
/*
#define _DBGMCU
*/
/************************************* DMA ************************************/
/*
#define _DMA
#define _DMA1_Channel1
#define _DMA1_Channel2
#define _DMA1_Channel3
#define _DMA1_Channel4
#define _DMA1_Channel5
#define _DMA1_Channel6
#define _DMA1_Channel7
#define _DMA2_Channel1
#define _DMA2_Channel2
#define _DMA2_Channel3
#define _DMA2_Channel4
#define _DMA2_Channel5
*/
/************************************* EXTI ***********************************/
/*
// #define _EXTI
*/
/************************************* FLASH and Option Bytes *****************/
#define _FLASH
/* Uncomment the line below to enable FLASH program/erase/protections functions,
   otherwise only FLASH configuration (latency, prefetch, half cycle) functions
   are enabled */
/* #define _FLASH_PROG */

/************************************* FSMC ***********************************/
/*
// #define _FSMC
*/
/************************************* GPIO ***********************************/
#define _GPIO
#define _GPIOA
#define _GPIOB
/*
#define _GPIOC
#define _GPIOD
#define _GPIOE
#define _GPIOF
#define _GPIOG
*/
#define _AFIO
/************************************* I2C ************************************/
/*
#define _I2C
#define _I2C1
#define _I2C2
*/
/************************************* IWDG ***********************************/
/*
#define _IWDG
*/
/************************************* NVIC ***********************************/
#define _NVIC
/************************************* PWR ************************************/
/*
#define _PWR
*/
/************************************* RCC ************************************/
#define _RCC
/************************************* RTC ************************************/
/*
#define _RTC
*/
/************************************* SDIO ***********************************/
/*
#define _SDIO
*/
/************************************* SPI ************************************/
#define _SPI
#define _SPI1
/*
#define _SPI2
#define _SPI3
*/
/************************************* SysTick ********************************/
#define _SysTick

/************************************* TIM ************************************/
/*
#define _TIM
#define _TIM1
#define _TIM2
#define _TIM3
#define _TIM4
#define _TIM5
#define _TIM6
#define _TIM7
#define _TIM8
*/
/************************************* USART **********************************/
#define _USART
#define _USART1
/*
#define _USART2
#define _USART3
#define _UART4
#define _UART5
*/
/************************************* WWDG ***********************************/
/*
#define _WWDG
*/
/* In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application */
#define HSE_Value    ((u32)8000000) /* Value of the External oscillator in Hz*/

/* In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value */
#define HSEStartUp_TimeOut    ((u16)0x0500) /* Time out for HSE start up */

/* Exported macro ------------------------------------------------------------*/
#ifdef  DEBUG
/*******************************************************************************
* Macro Name     : assert_param
* Description    : The assert_param macro is used for function's parameters check.
*                  It is used only if the library is compiled in DEBUG mode.
* Input          : - expr: If expr is false, it calls assert_failed function
*                    which reports the name of the source file and the source
*                    line number of the call that failed.
*                    If expr is true, it returns no value.
* Return         : None
*******************************************************************************/
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((u8 *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(u8* file, u32 line);
#else
  #define assert_param(expr) ((void)0)
#endif /* DEBUG */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_crc.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      CRC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CRC_H
#define __STM32F10x_CRC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void CRC_ResetDR(void);
u32 CRC_CalcCRC(u32 Data);
u32 CRC_CalcBlockCRC(u32 pBuffer[], u32 BufferLength);
u32 CRC_GetCRC(void);
void CRC_SetIDRegister(u8 IDValue);
u8 CRC_GetIDRegister(void);

#endif /* __STM32F10x_CRC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_dac.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      DAC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_DAC_H
#define __STM32F10x_DAC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* DAC Init structure definition */
typedef struct
{
  u32 DAC_Trigger;
  u32 DAC_WaveGeneration;
  u32 DAC_LFSRUnmask_TriangleAmplitude;
  u32 DAC_OutputBuffer; 
}DAC_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
/* DAC trigger selection */
#define DAC_Trigger_None                   ((u32)0x00000000)
#define DAC_Trigger_T6_TRGO                ((u32)0x00000004)
#define DAC_Trigger_T8_TRGO                ((u32)0x0000000C)
#define DAC_Trigger_T7_TRGO                ((u32)0x00000014)
#define DAC_Trigger_T5_TRGO                ((u32)0x0000001C)
#define DAC_Trigger_T2_TRGO                ((u32)0x00000024)
#define DAC_Trigger_T4_TRGO                ((u32)0x0000002C)
#define DAC_Trigger_Ext_IT9                ((u32)0x00000034)
#define DAC_Trigger_Software               ((u32)0x0000003C)

#define IS_DAC_TRIGGER(TRIGGER) (((TRIGGER) == DAC_Trigger_None) || \
                                 ((TRIGGER) == DAC_Trigger_T6_TRGO) || \
                                 ((TRIGGER) == DAC_Trigger_T8_TRGO) || \
                                 ((TRIGGER) == DAC_Trigger_T7_TRGO) || \
                                 ((TRIGGER) == DAC_Trigger_T5_TRGO) || \
                                 ((TRIGGER) == DAC_Trigger_T2_TRGO) || \
                                 ((TRIGGER) == DAC_Trigger_T4_TRGO) || \
                                 ((TRIGGER) == DAC_Trigger_Ext_IT9) || \
                                 ((TRIGGER) == DAC_Trigger_Software))

/* DAC wave generation */
#define DAC_WaveGeneration_None            ((u32)0x00000000)
#define DAC_WaveGeneration_Noise           ((u32)0x00000040)
#define DAC_WaveGeneration_Triangle        ((u32)0x00000080)

#define IS_DAC_GENERATE_WAVE(WAVE) (((WAVE) == DAC_WaveGeneration_None) || \
                                    ((WAVE) == DAC_WaveGeneration_Noise) || \
                                    ((WAVE) == DAC_WaveGeneration_Triangle))

/* DAC noise wave generation mask / triangle wave generation max amplitude */
#define DAC_LFSRUnmask_Bit0                ((u32)0x00000000)
#define DAC_LFSRUnmask_Bits1_0             ((u32)0x00000100)
#define DAC_LFSRUnmask_Bits2_0             ((u32)0x00000200)
#define DAC_LFSRUnmask_Bits3_0             ((u32)0x00000300)
#define DAC_LFSRUnmask_Bits4_0             ((u32)0x00000400)
#define DAC_LFSRUnmask_Bits5_0             ((u32)0x00000500)
#define DAC_LFSRUnmask_Bits6_0             ((u32)0x00000600)
#define DAC_LFSRUnmask_Bits7_0             ((u32)0x00000700)
#define DAC_LFSRUnmask_Bits8_0             ((u32)0x00000800)
#define DAC_LFSRUnmask_Bits9_0             ((u32)0x00000900)
#define DAC_LFSRUnmask_Bits10_0            ((u32)0x00000A00)
#define DAC_LFSRUnmask_Bits11_0            ((u32)0x00000B00)

#define DAC_TriangleAmplitude_1            ((u32)0x00000000)
#define DAC_TriangleAmplitude_3            ((u32)0x00000100)
#define DAC_TriangleAmplitude_7            ((u32)0x00000200)
#define DAC_TriangleAmplitude_15           ((u32)0x00000300)
#define DAC_TriangleAmplitude_31           ((u32)0x00000400)
#define DAC_TriangleAmplitude_63           ((u32)0x00000500)
#define DAC_TriangleAmplitude_127          ((u32)0x00000600)
#define DAC_TriangleAmplitude_255          ((u32)0x00000700)
#define DAC_TriangleAmplitude_511          ((u32)0x00000800)
#define DAC_TriangleAmplitude_1023         ((u32)0x00000900)
#define DAC_TriangleAmplitude_2047         ((u32)0x00000A00)
#define DAC_TriangleAmplitude_4095         ((u32)0x00000B00)

#define IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(VALUE) (((VALUE) == DAC_LFSRUnmask_Bit0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits1_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits2_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits3_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits4_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits5_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits6_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits7_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits8_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits9_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits10_0) || \
                                                      ((VALUE) == DAC_LFSRUnmask_Bits11_0) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_1) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_3) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_7) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_15) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_31) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_63) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_127) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_255) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_511) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_1023) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_2047) || \
                                                      ((VALUE) == DAC_TriangleAmplitude_4095))

/* DAC output buffer */
#define DAC_OutputBuffer_Enable            ((u32)0x00000000)
#define DAC_OutputBuffer_Disable           ((u32)0x00000002)

#define IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == DAC_OutputBuffer_Enable) || \
                                           ((STATE) == DAC_OutputBuffer_Disable))

/* DAC Channel selection */
#define DAC_Channel_1                      ((u32)0x00000000)
#define DAC_Channel_2                      ((u32)0x00000010)

#define IS_DAC_CHANNEL(CHANNEL) (((CHANNEL) == DAC_Channel_1) || \
                                 ((CHANNEL) == DAC_Channel_2))

/* DAC data alignement */
#define DAC_Align_12b_R                    ((u32)0x00000000)
#define DAC_Align_12b_L                    ((u32)0x00000004)
#define DAC_Align_8b_R                     ((u32)0x00000008)

#define IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_Align_12b_R) || \
                             ((ALIGN) == DAC_Align_12b_L) || \
                             ((ALIGN) == DAC_Align_8b_R))

/* DAC wave generation */
#define DAC_Wave_Noise                     ((u32)0x00000040)
#define DAC_Wave_Triangle                  ((u32)0x00000080)

#define IS_DAC_WAVE(WAVE) (((WAVE) == DAC_Wave_Noise) || \
                           ((WAVE) == DAC_Wave_Triangle))

/* DAC data ------------------------------------------------------------------*/
#define IS_DAC_DATA(DATA) ((DATA) <= 0xFFF0) 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DAC_DeInit(void);
void DAC_Init(u32 DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(u32 DAC_Channel, FunctionalState NewState);
void DAC_DMACmd(u32 DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(u32 DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(u32 DAC_Channel, u32 DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(u32 DAC_Align, u16 Data);
void DAC_SetChannel2Data(u32 DAC_Align, u16 Data);
void DAC_SetDualChannelData(u32 DAC_Align, u16 Data2, u16 Data1);
u16 DAC_GetDataOutputValue(u32 DAC_Channel);

#endif /*__STM32F10x_DAC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_dbgmcu.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      DBGMCU firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_DBGMCU_H
#define __STM32F10x_DBGMCU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define DBGMCU_SLEEP                 ((u32)0x00000001)
#define DBGMCU_STOP                  ((u32)0x00000002)
#define DBGMCU_STANDBY               ((u32)0x00000004)
#define DBGMCU_IWDG_STOP             ((u32)0x00000100)
#define DBGMCU_WWDG_STOP             ((u32)0x00000200)
#define DBGMCU_TIM1_STOP             ((u32)0x00000400)
#define DBGMCU_TIM2_STOP             ((u32)0x00000800)
#define DBGMCU_TIM3_STOP             ((u32)0x00001000)
#define DBGMCU_TIM4_STOP             ((u32)0x00002000)
#define DBGMCU_CAN_STOP              ((u32)0x00004000)
#define DBGMCU_I2C1_SMBUS_TIMEOUT    ((u32)0x00008000)
#define DBGMCU_I2C2_SMBUS_TIMEOUT    ((u32)0x00010000)
#define DBGMCU_TIM5_STOP             ((u32)0x00020000)
#define DBGMCU_TIM6_STOP             ((u32)0x00040000)
#define DBGMCU_TIM7_STOP             ((u32)0x00080000)
#define DBGMCU_TIM8_STOP             ((u32)0x00100000)
                                           
#define IS_DBGMCU_PERIPH(PERIPH) ((((PERIPH) & 0xFFE000F8) == 0x00) && ((PERIPH) != 0x00))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
u32 DBGMCU_GetREVID(void);
u32 DBGMCU_GetDEVID(void);
void DBGMCU_Config(u32 DBGMCU_Periph, FunctionalState NewState);

#endif /* __STM32F10x_DBGMCU_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_dma.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      DMA firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_DMA_H
#define __STM32F10x_DMA_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* DMA Init structure definition */
typedef struct
{
  u32 DMA_PeripheralBaseAddr;
  u32 DMA_MemoryBaseAddr;
  u32 DMA_DIR;
  u32 DMA_BufferSize;
  u32 DMA_PeripheralInc;
  u32 DMA_MemoryInc;
  u32 DMA_PeripheralDataSize;
  u32 DMA_MemoryDataSize;
  u32 DMA_Mode;
  u32 DMA_Priority;
  u32 DMA_M2M;
}DMA_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
#define IS_DMA_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == DMA1_Channel1_BASE) || \
                                   ((*(u32*)&(PERIPH)) == DMA1_Channel2_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA1_Channel3_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA1_Channel4_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA1_Channel5_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA1_Channel6_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA1_Channel7_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA2_Channel1_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA2_Channel2_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA2_Channel3_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA2_Channel4_BASE)  || \
                                   ((*(u32*)&(PERIPH)) == DMA2_Channel5_BASE))

/* DMA data transfer direction -----------------------------------------------*/
#define DMA_DIR_PeripheralDST              ((u32)0x00000010)
#define DMA_DIR_PeripheralSRC              ((u32)0x00000000)

#define IS_DMA_DIR(DIR) (((DIR) == DMA_DIR_PeripheralDST) || \
                         ((DIR) == DMA_DIR_PeripheralSRC))

/* DMA peripheral incremented mode -------------------------------------------*/
#define DMA_PeripheralInc_Enable           ((u32)0x00000040)
#define DMA_PeripheralInc_Disable          ((u32)0x00000000)

#define IS_DMA_PERIPHERAL_INC_STATE(STATE) (((STATE) == DMA_PeripheralInc_Enable) || \
                                            ((STATE) == DMA_PeripheralInc_Disable))

/* DMA memory incremented mode -----------------------------------------------*/
#define DMA_MemoryInc_Enable               ((u32)0x00000080)
#define DMA_MemoryInc_Disable              ((u32)0x00000000)

#define IS_DMA_MEMORY_INC_STATE(STATE) (((STATE) == DMA_MemoryInc_Enable) || \
                                        ((STATE) == DMA_MemoryInc_Disable))

/* DMA peripheral data size --------------------------------------------------*/
#define DMA_PeripheralDataSize_Byte        ((u32)0x00000000)
#define DMA_PeripheralDataSize_HalfWord    ((u32)0x00000100)
#define DMA_PeripheralDataSize_Word        ((u32)0x00000200)

#define IS_DMA_PERIPHERAL_DATA_SIZE(SIZE) (((SIZE) == DMA_PeripheralDataSize_Byte) || \
                                           ((SIZE) == DMA_PeripheralDataSize_HalfWord) || \
                                           ((SIZE) == DMA_PeripheralDataSize_Word))

/* DMA memory data size ------------------------------------------------------*/
#define DMA_MemoryDataSize_Byte            ((u32)0x00000000)
#define DMA_MemoryDataSize_HalfWord        ((u32)0x00000400)
#define DMA_MemoryDataSize_Word            ((u32)0x00000800)

#define IS_DMA_MEMORY_DATA_SIZE(SIZE) (((SIZE) == DMA_MemoryDataSize_Byte) || \
                                       ((SIZE) == DMA_MemoryDataSize_HalfWord) || \
                                       ((SIZE) == DMA_MemoryDataSize_Word))

/* DMA circular/normal mode --------------------------------------------------*/
#define DMA_Mode_Circular                  ((u32)0x00000020)
#define DMA_Mode_Normal                    ((u32)0x00000000)

#define IS_DMA_MODE(MODE) (((MODE) == DMA_Mode_Circular) || ((MODE) == DMA_Mode_Normal))

/* DMA priority level --------------------------------------------------------*/
#define DMA_Priority_VeryHigh              ((u32)0x00003000)
#define DMA_Priority_High                  ((u32)0x00002000)
#define DMA_Priority_Medium                ((u32)0x00001000)
#define DMA_Priority_Low                   ((u32)0x00000000)

#define IS_DMA_PRIORITY(PRIORITY) (((PRIORITY) == DMA_Priority_VeryHigh) || \
                                   ((PRIORITY) == DMA_Priority_High) || \
                                   ((PRIORITY) == DMA_Priority_Medium) || \
                                   ((PRIORITY) == DMA_Priority_Low))

/* DMA memory to memory ------------------------------------------------------*/
#define DMA_M2M_Enable                     ((u32)0x00004000)
#define DMA_M2M_Disable                    ((u32)0x00000000)

#define IS_DMA_M2M_STATE(STATE) (((STATE) == DMA_M2M_Enable) || ((STATE) == DMA_M2M_Disable))

/* DMA interrupts definition -------------------------------------------------*/
#define DMA_IT_TC                          ((u32)0x00000002)
#define DMA_IT_HT                          ((u32)0x00000004)
#define DMA_IT_TE                          ((u32)0x00000008)

#define IS_DMA_CONFIG_IT(IT) ((((IT) & 0xFFFFFFF1) == 0x00) && ((IT) != 0x00))

/* For DMA1 */
#define DMA1_IT_GL1                        ((u32)0x00000001)
#define DMA1_IT_TC1                        ((u32)0x00000002)
#define DMA1_IT_HT1                        ((u32)0x00000004)
#define DMA1_IT_TE1                        ((u32)0x00000008)
#define DMA1_IT_GL2                        ((u32)0x00000010)
#define DMA1_IT_TC2                        ((u32)0x00000020)
#define DMA1_IT_HT2                        ((u32)0x00000040)
#define DMA1_IT_TE2                        ((u32)0x00000080)
#define DMA1_IT_GL3                        ((u32)0x00000100)
#define DMA1_IT_TC3                        ((u32)0x00000200)
#define DMA1_IT_HT3                        ((u32)0x00000400)
#define DMA1_IT_TE3                        ((u32)0x00000800)
#define DMA1_IT_GL4                        ((u32)0x00001000)
#define DMA1_IT_TC4                        ((u32)0x00002000)
#define DMA1_IT_HT4                        ((u32)0x00004000)
#define DMA1_IT_TE4                        ((u32)0x00008000)
#define DMA1_IT_GL5                        ((u32)0x00010000)
#define DMA1_IT_TC5                        ((u32)0x00020000)
#define DMA1_IT_HT5                        ((u32)0x00040000)
#define DMA1_IT_TE5                        ((u32)0x00080000)
#define DMA1_IT_GL6                        ((u32)0x00100000)
#define DMA1_IT_TC6                        ((u32)0x00200000)
#define DMA1_IT_HT6                        ((u32)0x00400000)
#define DMA1_IT_TE6                        ((u32)0x00800000)
#define DMA1_IT_GL7                        ((u32)0x01000000)
#define DMA1_IT_TC7                        ((u32)0x02000000)
#define DMA1_IT_HT7                        ((u32)0x04000000)
#define DMA1_IT_TE7                        ((u32)0x08000000)
/* For DMA2 */
#define DMA2_IT_GL1                        ((u32)0x10000001)
#define DMA2_IT_TC1                        ((u32)0x10000002)
#define DMA2_IT_HT1                        ((u32)0x10000004)
#define DMA2_IT_TE1                        ((u32)0x10000008)
#define DMA2_IT_GL2                        ((u32)0x10000010)
#define DMA2_IT_TC2                        ((u32)0x10000020)
#define DMA2_IT_HT2                        ((u32)0x10000040)
#define DMA2_IT_TE2                        ((u32)0x10000080)
#define DMA2_IT_GL3                        ((u32)0x10000100)
#define DMA2_IT_TC3                        ((u32)0x10000200)
#define DMA2_IT_HT3                        ((u32)0x10000400)
#define DMA2_IT_TE3                        ((u32)0x10000800)
#define DMA2_IT_GL4                        ((u32)0x10001000)
#define DMA2_IT_TC4                        ((u32)0x10002000)
#define DMA2_IT_HT4                        ((u32)0x10004000)
#define DMA2_IT_TE4                        ((u32)0x10008000)
#define DMA2_IT_GL5                        ((u32)0x10010000)
#define DMA2_IT_TC5                        ((u32)0x10020000)
#define DMA2_IT_HT5                        ((u32)0x10040000)
#define DMA2_IT_TE5                        ((u32)0x10080000)

#define IS_DMA_CLEAR_IT(IT) (((((IT) & 0xF0000000) == 0x00) || (((IT) & 0xEFF00000) == 0x00)) && ((IT) != 0x00))
#define IS_DMA_GET_IT(IT) (((IT) == DMA1_IT_GL1) || ((IT) == DMA1_IT_TC1) || \
                           ((IT) == DMA1_IT_HT1) || ((IT) == DMA1_IT_TE1) || \
                           ((IT) == DMA1_IT_GL2) || ((IT) == DMA1_IT_TC2) || \
                           ((IT) == DMA1_IT_HT2) || ((IT) == DMA1_IT_TE2) || \
                           ((IT) == DMA1_IT_GL3) || ((IT) == DMA1_IT_TC3) || \
                           ((IT) == DMA1_IT_HT3) || ((IT) == DMA1_IT_TE3) || \
                           ((IT) == DMA1_IT_GL4) || ((IT) == DMA1_IT_TC4) || \
                           ((IT) == DMA1_IT_HT4) || ((IT) == DMA1_IT_TE4) || \
                           ((IT) == DMA1_IT_GL5) || ((IT) == DMA1_IT_TC5) || \
                           ((IT) == DMA1_IT_HT5) || ((IT) == DMA1_IT_TE5) || \
                           ((IT) == DMA1_IT_GL6) || ((IT) == DMA1_IT_TC6) || \
                           ((IT) == DMA1_IT_HT6) || ((IT) == DMA1_IT_TE6) || \
                           ((IT) == DMA1_IT_GL7) || ((IT) == DMA1_IT_TC7) || \
                           ((IT) == DMA1_IT_HT7) || ((IT) == DMA1_IT_TE7) || \
                           ((IT) == DMA2_IT_GL1) || ((IT) == DMA2_IT_TC1) || \
                           ((IT) == DMA2_IT_HT1) || ((IT) == DMA2_IT_TE1) || \
                           ((IT) == DMA2_IT_GL2) || ((IT) == DMA2_IT_TC2) || \
                           ((IT) == DMA2_IT_HT2) || ((IT) == DMA2_IT_TE2) || \
                           ((IT) == DMA2_IT_GL3) || ((IT) == DMA2_IT_TC3) || \
                           ((IT) == DMA2_IT_HT3) || ((IT) == DMA2_IT_TE3) || \
                           ((IT) == DMA2_IT_GL4) || ((IT) == DMA2_IT_TC4) || \
                           ((IT) == DMA2_IT_HT4) || ((IT) == DMA2_IT_TE4) || \
                           ((IT) == DMA2_IT_GL5) || ((IT) == DMA2_IT_TC5) || \
                           ((IT) == DMA2_IT_HT5) || ((IT) == DMA2_IT_TE5))

/* DMA flags definition ------------------------------------------------------*/
/* For DMA1 */
#define DMA1_FLAG_GL1                      ((u32)0x00000001)
#define DMA1_FLAG_TC1                      ((u32)0x00000002)
#define DMA1_FLAG_HT1                      ((u32)0x00000004)
#define DMA1_FLAG_TE1                      ((u32)0x00000008)
#define DMA1_FLAG_GL2                      ((u32)0x00000010)
#define DMA1_FLAG_TC2                      ((u32)0x00000020)
#define DMA1_FLAG_HT2                      ((u32)0x00000040)
#define DMA1_FLAG_TE2                      ((u32)0x00000080)
#define DMA1_FLAG_GL3                      ((u32)0x00000100)
#define DMA1_FLAG_TC3                      ((u32)0x00000200)
#define DMA1_FLAG_HT3                      ((u32)0x00000400)
#define DMA1_FLAG_TE3                      ((u32)0x00000800)
#define DMA1_FLAG_GL4                      ((u32)0x00001000)
#define DMA1_FLAG_TC4                      ((u32)0x00002000)
#define DMA1_FLAG_HT4                      ((u32)0x00004000)
#define DMA1_FLAG_TE4                      ((u32)0x00008000)
#define DMA1_FLAG_GL5                      ((u32)0x00010000)
#define DMA1_FLAG_TC5                      ((u32)0x00020000)
#define DMA1_FLAG_HT5                      ((u32)0x00040000)
#define DMA1_FLAG_TE5                      ((u32)0x00080000)
#define DMA1_FLAG_GL6                      ((u32)0x00100000)
#define DMA1_FLAG_TC6                      ((u32)0x00200000)
#define DMA1_FLAG_HT6                      ((u32)0x00400000)
#define DMA1_FLAG_TE6                      ((u32)0x00800000)
#define DMA1_FLAG_GL7                      ((u32)0x01000000)
#define DMA1_FLAG_TC7                      ((u32)0x02000000)
#define DMA1_FLAG_HT7                      ((u32)0x04000000)
#define DMA1_FLAG_TE7                      ((u32)0x08000000)
/* For DMA2 */
#define DMA2_FLAG_GL1                      ((u32)0x10000001)
#define DMA2_FLAG_TC1                      ((u32)0x10000002)
#define DMA2_FLAG_HT1                      ((u32)0x10000004)
#define DMA2_FLAG_TE1                      ((u32)0x10000008)
#define DMA2_FLAG_GL2                      ((u32)0x10000010)
#define DMA2_FLAG_TC2                      ((u32)0x10000020)
#define DMA2_FLAG_HT2                      ((u32)0x10000040)
#define DMA2_FLAG_TE2                      ((u32)0x10000080)
#define DMA2_FLAG_GL3                      ((u32)0x10000100)
#define DMA2_FLAG_TC3                      ((u32)0x10000200)
#define DMA2_FLAG_HT3                      ((u32)0x10000400)
#define DMA2_FLAG_TE3                      ((u32)0x10000800)
#define DMA2_FLAG_GL4                      ((u32)0x10001000)
#define DMA2_FLAG_TC4                      ((u32)0x10002000)
#define DMA2_FLAG_HT4                      ((u32)0x10004000)
#define DMA2_FLAG_TE4                      ((u32)0x10008000)
#define DMA2_FLAG_GL5                      ((u32)0x10010000)
#define DMA2_FLAG_TC5                      ((u32)0x10020000)
#define DMA2_FLAG_HT5                      ((u32)0x10040000)
#define DMA2_FLAG_TE5                      ((u32)0x10080000)

#define IS_DMA_CLEAR_FLAG(FLAG) (((((FLAG) & 0xF0000000) == 0x00) || (((FLAG) & 0xEFF00000) == 0x00)) && ((FLAG) != 0x00))
#define IS_DMA_GET_FLAG(FLAG) (((FLAG) == DMA1_FLAG_GL1) || ((FLAG) == DMA1_FLAG_TC1) || \
                               ((FLAG) == DMA1_FLAG_HT1) || ((FLAG) == DMA1_FLAG_TE1) || \
                               ((FLAG) == DMA1_FLAG_GL2) || ((FLAG) == DMA1_FLAG_TC2) || \
                               ((FLAG) == DMA1_FLAG_HT2) || ((FLAG) == DMA1_FLAG_TE2) || \
                               ((FLAG) == DMA1_FLAG_GL3) || ((FLAG) == DMA1_FLAG_TC3) || \
                               ((FLAG) == DMA1_FLAG_HT3) || ((FLAG) == DMA1_FLAG_TE3) || \
                               ((FLAG) == DMA1_FLAG_GL4) || ((FLAG) == DMA1_FLAG_TC4) || \
                               ((FLAG) == DMA1_FLAG_HT4) || ((FLAG) == DMA1_FLAG_TE4) || \
                               ((FLAG) == DMA1_FLAG_GL5) || ((FLAG) == DMA1_FLAG_TC5) || \
                               ((FLAG) == DMA1_FLAG_HT5) || ((FLAG) == DMA1_FLAG_TE5) || \
                               ((FLAG) == DMA1_FLAG_GL6) || ((FLAG) == DMA1_FLAG_TC6) || \
                               ((FLAG) == DMA1_FLAG_HT6) || ((FLAG) == DMA1_FLAG_TE6) || \
                               ((FLAG) == DMA1_FLAG_GL7) || ((FLAG) == DMA1_FLAG_TC7) || \
                               ((FLAG) == DMA1_FLAG_HT7) || ((FLAG) == DMA1_FLAG_TE7) || \
                               ((FLAG) == DMA2_FLAG_GL1) || ((FLAG) == DMA2_FLAG_TC1) || \
                               ((FLAG) == DMA2_FLAG_HT1) || ((FLAG) == DMA2_FLAG_TE1) || \
                               ((FLAG) == DMA2_FLAG_GL2) || ((FLAG) == DMA2_FLAG_TC2) || \
                               ((FLAG) == DMA2_FLAG_HT2) || ((FLAG) == DMA2_FLAG_TE2) || \
                               ((FLAG) == DMA2_FLAG_GL3) || ((FLAG) == DMA2_FLAG_TC3) || \
                               ((FLAG) == DMA2_FLAG_HT3) || ((FLAG) == DMA2_FLAG_TE3) || \
                               ((FLAG) == DMA2_FLAG_GL4) || ((FLAG) == DMA2_FLAG_TC4) || \
                               ((FLAG) == DMA2_FLAG_HT4) || ((FLAG) == DMA2_FLAG_TE4) || \
                               ((FLAG) == DMA2_FLAG_GL5) || ((FLAG) == DMA2_FLAG_TC5) || \
                               ((FLAG) == DMA2_FLAG_HT5) || ((FLAG) == DMA2_FLAG_TE5))

/* DMA Buffer Size -----------------------------------------------------------*/
#define IS_DMA_BUFFER_SIZE(SIZE) (((SIZE) >= 0x1) && ((SIZE) < 0x10000))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, u32 DMA_IT, FunctionalState NewState);
u16 DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(u32 DMA_FLAG);
void DMA_ClearFlag(u32 DMA_FLAG);
ITStatus DMA_GetITStatus(u32 DMA_IT);
void DMA_ClearITPendingBit(u32 DMA_IT);

#endif /*__STM32F10x_DMA_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_exti.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      EXTI firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_EXTI_H
#define __STM32F10x_EXTI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* EXTI mode enumeration -----------------------------------------------------*/
typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;

#define IS_EXTI_MODE(MODE) (((MODE) == EXTI_Mode_Interrupt) || ((MODE) == EXTI_Mode_Event))
                            
/* EXTI Trigger enumeration --------------------------------------------------*/
typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;

#define IS_EXTI_TRIGGER(TRIGGER) (((TRIGGER) == EXTI_Trigger_Rising) || \
                                  ((TRIGGER) == EXTI_Trigger_Falling) || \
                                  ((TRIGGER) == EXTI_Trigger_Rising_Falling))

/* EXTI Init Structure definition --------------------------------------------*/
typedef struct
{
  u32 EXTI_Line;
  EXTIMode_TypeDef EXTI_Mode;
  EXTITrigger_TypeDef EXTI_Trigger;
  FunctionalState EXTI_LineCmd;
}EXTI_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
/* EXTI Lines ----------------------------------------------------------------*/
#define EXTI_Line0       ((u32)0x00001)  /* External interrupt line 0 */
#define EXTI_Line1       ((u32)0x00002)  /* External interrupt line 1 */
#define EXTI_Line2       ((u32)0x00004)  /* External interrupt line 2 */
#define EXTI_Line3       ((u32)0x00008)  /* External interrupt line 3 */
#define EXTI_Line4       ((u32)0x00010)  /* External interrupt line 4 */
#define EXTI_Line5       ((u32)0x00020)  /* External interrupt line 5 */
#define EXTI_Line6       ((u32)0x00040)  /* External interrupt line 6 */
#define EXTI_Line7       ((u32)0x00080)  /* External interrupt line 7 */
#define EXTI_Line8       ((u32)0x00100)  /* External interrupt line 8 */
#define EXTI_Line9       ((u32)0x00200)  /* External interrupt line 9 */
#define EXTI_Line10      ((u32)0x00400)  /* External interrupt line 10 */
#define EXTI_Line11      ((u32)0x00800)  /* External interrupt line 11 */
#define EXTI_Line12      ((u32)0x01000)  /* External interrupt line 12 */
#define EXTI_Line13      ((u32)0x02000)  /* External interrupt line 13 */
#define EXTI_Line14      ((u32)0x04000)  /* External interrupt line 14 */
#define EXTI_Line15      ((u32)0x08000)  /* External interrupt line 15 */
#define EXTI_Line16      ((u32)0x10000)  /* External interrupt line 16
                                            Connected to the PVD Output */
#define EXTI_Line17      ((u32)0x20000)  /* External interrupt line 17 
                                            Connected to the RTC Alarm event */
#define EXTI_Line18      ((u32)0x40000)  /* External interrupt line 18 
                                            Connected to the USB Wakeup from 
                                            suspend event */

#define IS_EXTI_LINE(LINE) ((((LINE) & (u32)0xFFF80000) == 0x00) && ((LINE) != (u16)0x00))

#define IS_GET_EXTI_LINE(LINE) (((LINE) == EXTI_Line0) || ((LINE) == EXTI_Line1) || \
                            ((LINE) == EXTI_Line2) || ((LINE) == EXTI_Line3) || \
                            ((LINE) == EXTI_Line4) || ((LINE) == EXTI_Line5) || \
                            ((LINE) == EXTI_Line6) || ((LINE) == EXTI_Line7) || \
                            ((LINE) == EXTI_Line8) || ((LINE) == EXTI_Line9) || \
                            ((LINE) == EXTI_Line10) || ((LINE) == EXTI_Line11) || \
                            ((LINE) == EXTI_Line12) || ((LINE) == EXTI_Line13) || \
                            ((LINE) == EXTI_Line14) || ((LINE) == EXTI_Line15) || \
                            ((LINE) == EXTI_Line16) || ((LINE) == EXTI_Line17) || \
                            ((LINE) == EXTI_Line18))
                                 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void EXTI_DeInit(void);
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(u32 EXTI_Line);
FlagStatus EXTI_GetFlagStatus(u32 EXTI_Line);
void EXTI_ClearFlag(u32 EXTI_Line);
ITStatus EXTI_GetITStatus(u32 EXTI_Line);
void EXTI_ClearITPendingBit(u32 EXTI_Line);

#endif /* __STM32F10x_EXTI_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_flash.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      FLASH firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_FLASH_H
#define __STM32F10x_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
#ifdef _FLASH_PROG
/* FLASH Status */
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;
#endif

/* Flash Latency -------------------------------------------------------------*/
#define FLASH_Latency_0                ((u32)0x00000000)  /* FLASH Zero Latency cycle */
#define FLASH_Latency_1                ((u32)0x00000001)  /* FLASH One Latency cycle */
#define FLASH_Latency_2                ((u32)0x00000002)  /* FLASH Two Latency cycles */

#define IS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_Latency_0) || \
                                   ((LATENCY) == FLASH_Latency_1) || \
                                   ((LATENCY) == FLASH_Latency_2))

/* Half Cycle Enable/Disable -------------------------------------------------*/
#define FLASH_HalfCycleAccess_Enable   ((u32)0x00000008)  /* FLASH Half Cycle Enable */
#define FLASH_HalfCycleAccess_Disable  ((u32)0x00000000)  /* FLASH Half Cycle Disable */

#define IS_FLASH_HALFCYCLEACCESS_STATE(STATE) (((STATE) == FLASH_HalfCycleAccess_Enable) || \
                                               ((STATE) == FLASH_HalfCycleAccess_Disable)) 


/* Prefetch Buffer Enable/Disable --------------------------------------------*/
#define FLASH_PrefetchBuffer_Enable    ((u32)0x00000010)  /* FLASH Prefetch Buffer Enable */
#define FLASH_PrefetchBuffer_Disable   ((u32)0x00000000)  /* FLASH Prefetch Buffer Disable */

#define IS_FLASH_PREFETCHBUFFER_STATE(STATE) (((STATE) == FLASH_PrefetchBuffer_Enable) || \
                                              ((STATE) == FLASH_PrefetchBuffer_Disable)) 

#ifdef _FLASH_PROG
/* Option Bytes Write Protection ---------------------------------------------*/
/* Values to be used with STM32F10Xxx Medium-density devices: FLASH memory density
   ranges between 32 and 128 Kbytes with page size equal to 1 Kbytes */
#define FLASH_WRProt_Pages0to3         ((u32)0x00000001) /* Write protection of page 0 to 3 */
#define FLASH_WRProt_Pages4to7         ((u32)0x00000002) /* Write protection of page 4 to 7 */
#define FLASH_WRProt_Pages8to11        ((u32)0x00000004) /* Write protection of page 8 to 11 */
#define FLASH_WRProt_Pages12to15       ((u32)0x00000008) /* Write protection of page 12 to 15 */
#define FLASH_WRProt_Pages16to19       ((u32)0x00000010) /* Write protection of page 16 to 19 */
#define FLASH_WRProt_Pages20to23       ((u32)0x00000020) /* Write protection of page 20 to 23 */
#define FLASH_WRProt_Pages24to27       ((u32)0x00000040) /* Write protection of page 24 to 27 */
#define FLASH_WRProt_Pages28to31       ((u32)0x00000080) /* Write protection of page 28 to 31 */
#define FLASH_WRProt_Pages32to35       ((u32)0x00000100) /* Write protection of page 32 to 35 */
#define FLASH_WRProt_Pages36to39       ((u32)0x00000200) /* Write protection of page 36 to 39 */
#define FLASH_WRProt_Pages40to43       ((u32)0x00000400) /* Write protection of page 40 to 43 */
#define FLASH_WRProt_Pages44to47       ((u32)0x00000800) /* Write protection of page 44 to 47 */
#define FLASH_WRProt_Pages48to51       ((u32)0x00001000) /* Write protection of page 48 to 51 */
#define FLASH_WRProt_Pages52to55       ((u32)0x00002000) /* Write protection of page 52 to 55 */
#define FLASH_WRProt_Pages56to59       ((u32)0x00004000) /* Write protection of page 56 to 59 */
#define FLASH_WRProt_Pages60to63       ((u32)0x00008000) /* Write protection of page 60 to 63 */
#define FLASH_WRProt_Pages64to67       ((u32)0x00010000) /* Write protection of page 64 to 67 */
#define FLASH_WRProt_Pages68to71       ((u32)0x00020000) /* Write protection of page 68 to 71 */
#define FLASH_WRProt_Pages72to75       ((u32)0x00040000) /* Write protection of page 72 to 75 */
#define FLASH_WRProt_Pages76to79       ((u32)0x00080000) /* Write protection of page 76 to 79 */
#define FLASH_WRProt_Pages80to83       ((u32)0x00100000) /* Write protection of page 80 to 83 */
#define FLASH_WRProt_Pages84to87       ((u32)0x00200000) /* Write protection of page 84 to 87 */
#define FLASH_WRProt_Pages88to91       ((u32)0x00400000) /* Write protection of page 88 to 91 */
#define FLASH_WRProt_Pages92to95       ((u32)0x00800000) /* Write protection of page 92 to 95 */
#define FLASH_WRProt_Pages96to99       ((u32)0x01000000) /* Write protection of page 96 to 99 */
#define FLASH_WRProt_Pages100to103     ((u32)0x02000000) /* Write protection of page 100 to 103 */
#define FLASH_WRProt_Pages104to107     ((u32)0x04000000) /* Write protection of page 104 to 107 */
#define FLASH_WRProt_Pages108to111     ((u32)0x08000000) /* Write protection of page 108 to 111 */
#define FLASH_WRProt_Pages112to115     ((u32)0x10000000) /* Write protection of page 112 to 115 */
#define FLASH_WRProt_Pages116to119     ((u32)0x20000000) /* Write protection of page 115 to 119 */
#define FLASH_WRProt_Pages120to123     ((u32)0x40000000) /* Write protection of page 120 to 123 */
#define FLASH_WRProt_Pages124to127     ((u32)0x80000000) /* Write protection of page 124 to 127 */
/* Values to be used with STM32F10Xxx High-density devices: FLASH memory density
   ranges between 256 and 512 Kbytes with page size equal to 2 Kbytes */
#define FLASH_WRProt_Pages0to1         ((u32)0x00000001) /* Write protection of page 0 to 1 */
#define FLASH_WRProt_Pages2to3         ((u32)0x00000002) /* Write protection of page 2 to 3 */
#define FLASH_WRProt_Pages4to5         ((u32)0x00000004) /* Write protection of page 4 to 5 */
#define FLASH_WRProt_Pages6to7         ((u32)0x00000008) /* Write protection of page 6 to 7 */
#define FLASH_WRProt_Pages8to9         ((u32)0x00000010) /* Write protection of page 8 to 9 */
#define FLASH_WRProt_Pages10to11       ((u32)0x00000020) /* Write protection of page 10 to 11 */
#define FLASH_WRProt_Pages12to13       ((u32)0x00000040) /* Write protection of page 12 to 13 */
#define FLASH_WRProt_Pages14to15       ((u32)0x00000080) /* Write protection of page 14 to 15 */
#define FLASH_WRProt_Pages16to17       ((u32)0x00000100) /* Write protection of page 16 to 17 */
#define FLASH_WRProt_Pages18to19       ((u32)0x00000200) /* Write protection of page 18 to 19 */
#define FLASH_WRProt_Pages20to21       ((u32)0x00000400) /* Write protection of page 20 to 21 */
#define FLASH_WRProt_Pages22to23       ((u32)0x00000800) /* Write protection of page 22 to 23 */
#define FLASH_WRProt_Pages24to25       ((u32)0x00001000) /* Write protection of page 24 to 25 */
#define FLASH_WRProt_Pages26to27       ((u32)0x00002000) /* Write protection of page 26 to 27 */
#define FLASH_WRProt_Pages28to29       ((u32)0x00004000) /* Write protection of page 28 to 29 */
#define FLASH_WRProt_Pages30to31       ((u32)0x00008000) /* Write protection of page 30 to 31 */
#define FLASH_WRProt_Pages32to33       ((u32)0x00010000) /* Write protection of page 32 to 33 */
#define FLASH_WRProt_Pages34to35       ((u32)0x00020000) /* Write protection of page 34 to 35 */
#define FLASH_WRProt_Pages36to37       ((u32)0x00040000) /* Write protection of page 36 to 37 */
#define FLASH_WRProt_Pages38to39       ((u32)0x00080000) /* Write protection of page 38 to 39 */
#define FLASH_WRProt_Pages40to41       ((u32)0x00100000) /* Write protection of page 40 to 41 */
#define FLASH_WRProt_Pages42to43       ((u32)0x00200000) /* Write protection of page 42 to 43 */
#define FLASH_WRProt_Pages44to45       ((u32)0x00400000) /* Write protection of page 44 to 45 */
#define FLASH_WRProt_Pages46to47       ((u32)0x00800000) /* Write protection of page 46 to 47 */
#define FLASH_WRProt_Pages48to49       ((u32)0x01000000) /* Write protection of page 48 to 49 */
#define FLASH_WRProt_Pages50to51       ((u32)0x02000000) /* Write protection of page 50 to 51 */
#define FLASH_WRProt_Pages52to53       ((u32)0x04000000) /* Write protection of page 52 to 53 */
#define FLASH_WRProt_Pages54to55       ((u32)0x08000000) /* Write protection of page 54 to 55 */
#define FLASH_WRProt_Pages56to57       ((u32)0x10000000) /* Write protection of page 56 to 57 */
#define FLASH_WRProt_Pages58to59       ((u32)0x20000000) /* Write protection of page 58 to 59 */
#define FLASH_WRProt_Pages60to61       ((u32)0x40000000) /* Write protection of page 60 to 61 */
#define FLASH_WRProt_Pages62to255      ((u32)0x80000000) /* Write protection of page 62 to 255 */
#define FLASH_WRProt_AllPages          ((u32)0xFFFFFFFF) /* Write protection of all Pages */

#define IS_FLASH_WRPROT_PAGE(PAGE) (((PAGE) != 0x00000000))

#define IS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x0807FFFF))
#define IS_OB_DATA_ADDRESS(ADDRESS) (((ADDRESS) == 0x1FFFF804) || ((ADDRESS) == 0x1FFFF806))

/* Option Bytes IWatchdog ----------------------------------------------------*/
#define OB_IWDG_SW                     ((u16)0x0001)  /* Software IWDG selected */
#define OB_IWDG_HW                     ((u16)0x0000)  /* Hardware IWDG selected */

#define IS_OB_IWDG_SOURCE(SOURCE) (((SOURCE) == OB_IWDG_SW) || ((SOURCE) == OB_IWDG_HW))

/* Option Bytes nRST_STOP ----------------------------------------------------*/
#define OB_STOP_NoRST                  ((u16)0x0002) /* No reset generated when entering in STOP */
#define OB_STOP_RST                    ((u16)0x0000) /* Reset generated when entering in STOP */

#define IS_OB_STOP_SOURCE(SOURCE) (((SOURCE) == OB_STOP_NoRST) || ((SOURCE) == OB_STOP_RST))

/* Option Bytes nRST_STDBY ---------------------------------------------------*/
#define OB_STDBY_NoRST                 ((u16)0x0004) /* No reset generated when entering in STANDBY */
#define OB_STDBY_RST                   ((u16)0x0000) /* Reset generated when entering in STANDBY */

#define IS_OB_STDBY_SOURCE(SOURCE) (((SOURCE) == OB_STDBY_NoRST) || ((SOURCE) == OB_STDBY_RST))

/* FLASH Interrupts ----------------------------------------------------------*/
#define FLASH_IT_ERROR                 ((u32)0x00000400)  /* FPEC error interrupt source */
#define FLASH_IT_EOP                   ((u32)0x00001000)  /* End of FLASH Operation Interrupt source */

#define IS_FLASH_IT(IT) ((((IT) & (u32)0xFFFFEBFF) == 0x00000000) && (((IT) != 0x00000000)))

/* FLASH Flags ---------------------------------------------------------------*/
#define FLASH_FLAG_BSY                 ((u32)0x00000001)  /* FLASH Busy flag */
#define FLASH_FLAG_EOP                 ((u32)0x00000020)  /* FLASH End of Operation flag */
#define FLASH_FLAG_PGERR               ((u32)0x00000004)  /* FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR            ((u32)0x00000010)  /* FLASH Write protected error flag */
#define FLASH_FLAG_OPTERR              ((u32)0x00000001)  /* FLASH Option Byte error flag */
 
#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (u32)0xFFFFFFCA) == 0x00000000) && ((FLAG) != 0x00000000))

#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_BSY) || ((FLAG) == FLASH_FLAG_EOP) || \
                                  ((FLAG) == FLASH_FLAG_PGERR) || ((FLAG) == FLASH_FLAG_WRPRTERR) || \
                                  ((FLAG) == FLASH_FLAG_OPTERR))
#endif
								 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FLASH_SetLatency(u32 FLASH_Latency);
void FLASH_HalfCycleAccessCmd(u32 FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(u32 FLASH_PrefetchBuffer);

#ifdef _FLASH_PROG
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(u32 Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(u32 Address, u32 Data);
FLASH_Status FLASH_ProgramHalfWord(u32 Address, u16 Data);
FLASH_Status FLASH_ProgramOptionByteData(u32 Address, u8 Data);
FLASH_Status FLASH_EnableWriteProtection(u32 FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(u16 OB_IWDG, u16 OB_STOP, u16 OB_STDBY);
u32 FLASH_GetUserOptionByte(void);
u32 FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(u16 FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(u16 FLASH_FLAG);
void FLASH_ClearFlag(u16 FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(u32 Timeout);
#endif

#endif /* __STM32F10x_FLASH_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_fsmc.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      FSMC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_FSMC_H
#define __STM32F10x_FSMC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Timing parameters For NOR/SRAM Banks */
typedef struct
{
  u32 FSMC_AddressSetupTime;
  u32 FSMC_AddressHoldTime;
  u32 FSMC_DataSetupTime;
  u32 FSMC_BusTurnAroundDuration;
  u32 FSMC_CLKDivision;
  u32 FSMC_DataLatency;
  u32 FSMC_AccessMode;
}FSMC_NORSRAMTimingInitTypeDef;

/* FSMC NOR/SRAM Init structure definition */
typedef struct
{
  u32 FSMC_Bank;
  u32 FSMC_DataAddressMux;
  u32 FSMC_MemoryType;
  u32 FSMC_MemoryDataWidth;
  u32 FSMC_BurstAccessMode;
  u32 FSMC_WaitSignalPolarity;
  u32 FSMC_WrapMode;
  u32 FSMC_WaitSignalActive;
  u32 FSMC_WriteOperation;
  u32 FSMC_WaitSignal;
  u32 FSMC_ExtendedMode;
  u32 FSMC_WriteBurst;
  /* Timing Parameters for write and read access if the  ExtendedMode is not used*/
  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;
  /* Timing Parameters for write access if the  ExtendedMode is used*/
  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;
}FSMC_NORSRAMInitTypeDef;

/* Timing parameters For FSMC NAND and PCCARD Banks */
typedef struct
{
  u32 FSMC_SetupTime;
  u32 FSMC_WaitSetupTime;
  u32 FSMC_HoldSetupTime;
  u32 FSMC_HiZSetupTime;
}FSMC_NAND_PCCARDTimingInitTypeDef;

/* FSMC NAND Init structure definition */
typedef struct
{
  u32 FSMC_Bank;
  u32 FSMC_Waitfeature;
  u32 FSMC_MemoryDataWidth;
  u32 FSMC_ECC;
  u32 FSMC_ECCPageSize;
  u32 FSMC_AddressLowMapping;
  u32 FSMC_TCLRSetupTime;
  u32 FSMC_TARSetupTime;
  /* FSMC Common Space Timing */
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;
  /* FSMC Attribute Space Timing */
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;
}FSMC_NANDInitTypeDef;

/* FSMC PCCARD Init structure definition */
typedef struct
{
  u32 FSMC_Waitfeature;
  u32 FSMC_AddressLowMapping;
  u32 FSMC_TCLRSetupTime;
  u32 FSMC_TARSetupTime;
  /* FSMC Common Space Timing */
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;
  /* FSMC Attribute Space Timing */
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;
  /* FSMC IO Space Timing */
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;
}FSMC_PCCARDInitTypeDef;

/* Exported constants --------------------------------------------------------*/
/*-------------------------------FSMC Banks definitions ----------------------*/
#define FSMC_Bank1_NORSRAM1                             ((u32)0x00000000)
#define FSMC_Bank1_NORSRAM2                             ((u32)0x00000002)
#define FSMC_Bank1_NORSRAM3                             ((u32)0x00000004)
#define FSMC_Bank1_NORSRAM4                             ((u32)0x00000006)
#define FSMC_Bank2_NAND                                 ((u32)0x00000010)
#define FSMC_Bank3_NAND                                 ((u32)0x00000100)
#define FSMC_Bank4_PCCARD                               ((u32)0x00001000)

#define IS_FSMC_NORSRAM_BANK(BANK) (((BANK) == FSMC_Bank1_NORSRAM1) || \
                                    ((BANK) == FSMC_Bank1_NORSRAM2) || \
                                    ((BANK) == FSMC_Bank1_NORSRAM3) || \
                                    ((BANK) == FSMC_Bank1_NORSRAM4))                           


#define IS_FSMC_NAND_BANK(BANK) (((BANK) == FSMC_Bank2_NAND) || \
                                 ((BANK) == FSMC_Bank3_NAND))

#define IS_FSMC_GETFLAG_BANK(BANK) (((BANK) == FSMC_Bank2_NAND) || \
                                    ((BANK) == FSMC_Bank3_NAND) || \
                                    ((BANK) == FSMC_Bank4_PCCARD))
                                    
#define IS_FSMC_IT_BANK(BANK) (((BANK) == FSMC_Bank2_NAND) || \
                               ((BANK) == FSMC_Bank3_NAND) || \
                               ((BANK) == FSMC_Bank4_PCCARD))                                    


/*------------------------------- NOR/SRAM Banks -----------------------------*/
/* FSMC Data/Address Bus Multiplexing ----------------------------------------*/
#define FSMC_DataAddressMux_Disable                       ((u32)0x00000000)
#define FSMC_DataAddressMux_Enable                        ((u32)0x00000002)

#define IS_FSMC_MUX(MUX) (((MUX) == FSMC_DataAddressMux_Disable) || \
                          ((MUX) == FSMC_DataAddressMux_Enable))                           

/* FSMC Memory Type ----------------------------------------------------------*/
#define FSMC_MemoryType_SRAM                            ((u32)0x00000000)
#define FSMC_MemoryType_PSRAM                           ((u32)0x00000004)
#define FSMC_MemoryType_NOR                             ((u32)0x00000008)

#define IS_FSMC_MEMORY(MEMORY) (((MEMORY) == FSMC_MemoryType_SRAM) || \
                                ((MEMORY) == FSMC_MemoryType_PSRAM)|| \
                                ((MEMORY) == FSMC_MemoryType_NOR))
                                     
/* FSMC  Data Width ----------------------------------------------------------*/
#define FSMC_MemoryDataWidth_8b                         ((u32)0x00000000)
#define FSMC_MemoryDataWidth_16b                        ((u32)0x00000010)

#define IS_FSMC_MEMORY_WIDTH(WIDTH) (((WIDTH) == FSMC_MemoryDataWidth_8b) || \
                                     ((WIDTH) == FSMC_MemoryDataWidth_16b))
                                      
                               
/* FSMC Burst Access Mode ----------------------------------------------------*/
#define FSMC_BurstAccessMode_Disable                    ((u32)0x00000000) 
#define FSMC_BurstAccessMode_Enable                     ((u32)0x00000100)

#define IS_FSMC_BURSTMODE(STATE) (((STATE) == FSMC_BurstAccessMode_Disable) || \
                                  ((STATE) == FSMC_BurstAccessMode_Enable))

/* FSMC Wait Signal Polarity -------------------------------------------------*/                                  
#define FSMC_WaitSignalPolarity_Low                     ((u32)0x00000000)
#define FSMC_WaitSignalPolarity_High                    ((u32)0x00000200)

#define IS_FSMC_WAIT_POLARITY(POLARITY) (((POLARITY) == FSMC_WaitSignalPolarity_Low) || \
                                         ((POLARITY) == FSMC_WaitSignalPolarity_High)) 
                                        
/* FSMC Wrap Mode ------------------------------------------------------------*/ 
#define FSMC_WrapMode_Disable                           ((u32)0x00000000)
#define FSMC_WrapMode_Enable                            ((u32)0x00000400) 

#define IS_FSMC_WRAP_MODE(MODE) (((MODE) == FSMC_WrapMode_Disable) || \
                                 ((MODE) == FSMC_WrapMode_Enable))
                                 
/* FSMC Wait Timing ----------------------------------------------------------*/                                 
#define FSMC_WaitSignalActive_BeforeWaitState           ((u32)0x00000000)
#define FSMC_WaitSignalActive_DuringWaitState           ((u32)0x00000800) 

#define IS_FSMC_WAIT_SIGNAL_ACTIVE(ACTIVE) (((ACTIVE) == FSMC_WaitSignalActive_BeforeWaitState) || \
                                            ((ACTIVE) == FSMC_WaitSignalActive_DuringWaitState))
                                    
/* FSMC Write Operation ------------------------------------------------------*/
#define FSMC_WriteOperation_Disable                     ((u32)0x00000000)
#define FSMC_WriteOperation_Enable                      ((u32)0x00001000)

#define IS_FSMC_WRITE_OPERATION(OPERATION) (((OPERATION) == FSMC_WriteOperation_Disable) || \
                                            ((OPERATION) == FSMC_WriteOperation_Enable))
                              
/* FSMC Wait Signal ----------------------------------------------------------*/
#define FSMC_WaitSignal_Disable                         ((u32)0x00000000)
#define FSMC_WaitSignal_Enable                          ((u32)0x00002000) 

#define IS_FSMC_WAITE_SIGNAL(SIGNAL) (((SIGNAL) == FSMC_WaitSignal_Disable) || \
                                      ((SIGNAL) == FSMC_WaitSignal_Enable))

/* FSMC Extended Mode --------------------------------------------------------*/
#define FSMC_ExtendedMode_Disable                       ((u32)0x00000000)
#define FSMC_ExtendedMode_Enable                        ((u32)0x00004000)                                  

#define IS_FSMC_EXTENDED_MODE(MODE) (((MODE) == FSMC_ExtendedMode_Disable) || \
                                     ((MODE) == FSMC_ExtendedMode_Enable)) 
                                                                 
/* FSMC Write Burst ----------------------------------------------------------*/                                  
#define FSMC_WriteBurst_Disable                         ((u32)0x00000000)
#define FSMC_WriteBurst_Enable                          ((u32)0x00080000) 

#define IS_FSMC_WRITE_BURST(BURST) (((BURST) == FSMC_WriteBurst_Disable) || \
                                    ((BURST) == FSMC_WriteBurst_Enable))

/* FSMC Address Setup Time ---------------------------------------------------*/
#define IS_FSMC_ADDRESS_SETUP_TIME(TIME) ((TIME) <= 0xF)

/* FSMC Address Hold Time ----------------------------------------------------*/
#define IS_FSMC_ADDRESS_HOLD_TIME(TIME) ((TIME) <= 0xF)

/* FSMC Data Setup Time ------------------------------------------------------*/
#define IS_FSMC_DATASETUP_TIME(TIME) (((TIME) > 0) && ((TIME) <= 0xFF))

/* FSMC Bus Turn around Duration ---------------------------------------------*/
#define IS_FSMC_TURNAROUND_TIME(TIME) ((TIME) <= 0xF)

/* FSMC CLK Division ---------------------------------------------------------*/
#define IS_FSMC_CLK_DIV(DIV) ((DIV) <= 0xF)

/* FSMC Data Latency ---------------------------------------------------------*/
#define IS_FSMC_DATA_LATENCY(LATENCY) ((LATENCY) <= 0xF)

/* FSMC Access Mode ----------------------------------------------------------*/
#define FSMC_AccessMode_A                               ((u32)0x00000000)
#define FSMC_AccessMode_B                               ((u32)0x10000000) 
#define FSMC_AccessMode_C                               ((u32)0x20000000)
#define FSMC_AccessMode_D                               ((u32)0x30000000)

#define IS_FSMC_ACCESS_MODE(MODE) (((MODE) == FSMC_AccessMode_A) || \
                                   ((MODE) == FSMC_AccessMode_B) || \
                                   ((MODE) == FSMC_AccessMode_C) || \
                                   ((MODE) == FSMC_AccessMode_D)) 
                                  
/*----------------------------- NAND and PCCARD Banks ------------------------*/
/* FSMC Wait feature ---------------------------------------------------------*/
#define FSMC_Waitfeature_Disable                        ((u32)0x00000000)
#define FSMC_Waitfeature_Enable                         ((u32)0x00000002)

#define IS_FSMC_WAIT_FEATURE(FEATURE) (((FEATURE) == FSMC_Waitfeature_Disable) || \
                                       ((FEATURE) == FSMC_Waitfeature_Enable))
                                    
/* FSMC Memory Data Width ----------------------------------------------------*/
#define FSMC_MemoryDataWidth_8b                         ((u32)0x00000000)
#define FSMC_MemoryDataWidth_16b                        ((u32)0x00000010)

#define IS_FSMC_DATA_WIDTH(WIDTH) (((WIDTH) == FSMC_MemoryDataWidth_8b) || \
                                   ((WIDTH) == FSMC_MemoryDataWidth_16b))
                                    
/* FSMC ECC ------------------------------------------------------------------*/
#define FSMC_ECC_Disable                                ((u32)0x00000000)
#define FSMC_ECC_Enable                                 ((u32)0x00000040)

#define IS_FSMC_ECC_STATE(STATE) (((STATE) == FSMC_ECC_Disable) || \
                                  ((STATE) == FSMC_ECC_Enable))
                                            
/* FSMC ECC Page Size --------------------------------------------------------*/
#define FSMC_ECCPageSize_256Bytes                       ((u32)0x00000000)
#define FSMC_ECCPageSize_512Bytes                       ((u32)0x00020000)
#define FSMC_ECCPageSize_1024Bytes                      ((u32)0x00040000)
#define FSMC_ECCPageSize_2048Bytes                      ((u32)0x00060000)
#define FSMC_ECCPageSize_4096Bytes                      ((u32)0x00080000)
#define FSMC_ECCPageSize_8192Bytes                      ((u32)0x000A0000)

#define IS_FSMC_ECCPAGE_SIZE(SIZE) (((SIZE) == FSMC_ECCPageSize_256Bytes) || \
                                    ((SIZE) == FSMC_ECCPageSize_512Bytes) || \
                                    ((SIZE) == FSMC_ECCPageSize_1024Bytes) || \
                                    ((SIZE) == FSMC_ECCPageSize_2048Bytes) || \
                                    ((SIZE) == FSMC_ECCPageSize_4096Bytes) || \
                                    ((SIZE) == FSMC_ECCPageSize_8192Bytes))
                                                              
/* FSMC Address Low Mapping --------------------------------------------------*/
#define FSMC_AddressLowMapping_Direct                   ((u32)0x00000000)
#define FSMC_AddressLowMapping_InDirect                 ((u32)0x00000100)

#define IS_FSMC_ADDRESS_LOW_MAPPING(MAPPING) (((MAPPING) == FSMC_AddressLowMapping_Direct) || \
                                              ((MAPPING) == FSMC_AddressLowMapping_InDirect))
/* FSMC TCLR Setup Time ------------------------------------------------------*/
#define IS_FSMC_TCLR_TIME(TIME) ((TIME) <= 0xFF)

/* FSMC TAR Setup Time -------------------------------------------------------*/
#define IS_FSMC_TAR_TIME(TIME) ((TIME) <= 0xFF)

/* FSMC Setup Time ----------------------------------------------------*/
#define IS_FSMC_SETUP_TIME(TIME) ((TIME) <= 0xFF)

/* FSMC Wait Setup Time -----------------------------------------------*/
#define IS_FSMC_WAIT_TIME(TIME) ((TIME) <= 0xFF)

/* FSMC Hold Setup Time -----------------------------------------------*/
#define IS_FSMC_HOLD_TIME(TIME) ((TIME) <= 0xFF)

/* FSMC HiZ Setup Time ------------------------------------------------*/
#define IS_FSMC_HIZ_TIME(TIME) ((TIME) <= 0xFF)

/* FSMC Interrupt sources ----------------------------------------------------*/
#define FSMC_IT_RisingEdge                              ((u32)0x00000008)
#define FSMC_IT_Level                                   ((u32)0x00000010)
#define FSMC_IT_FallingEdge                             ((u32)0x00000020)

#define IS_FSMC_IT(IT) ((((IT) & (u32)0xFFFFFFC7) == 0x00000000) && ((IT) != 0x00000000))

#define IS_FSMC_GET_IT(IT) (((IT) == FSMC_IT_RisingEdge) || \
                            ((IT) == FSMC_IT_Level) || \
                            ((IT) == FSMC_IT_FallingEdge)) 

/* FSMC Flags ----------------------------------------------------------------*/
#define FSMC_FLAG_RisingEdge                            ((u32)0x00000001)
#define FSMC_FLAG_Level                                 ((u32)0x00000002)
#define FSMC_FLAG_FallingEdge                           ((u32)0x00000004)
#define FSMC_FLAG_FEMPT                                 ((u32)0x00000040)

#define IS_FSMC_GET_FLAG(FLAG) (((FLAG) == FSMC_FLAG_RisingEdge) || \
                                ((FLAG) == FSMC_FLAG_Level) || \
                                ((FLAG) == FSMC_FLAG_FallingEdge) || \
                                ((FLAG) == FSMC_FLAG_FEMPT))

#define IS_FSMC_CLEAR_FLAG(FLAG) ((((FLAG) & (u32)0xFFFFFFF8) == 0x00000000) && ((FLAG) != 0x00000000))                                                                                                                                                                                                                                                                                                                                  
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FSMC_NORSRAMDeInit(u32 FSMC_Bank);
void FSMC_NANDDeInit(u32 FSMC_Bank);
void FSMC_PCCARDDeInit(void);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMCmd(u32 FSMC_Bank, FunctionalState NewState);
void FSMC_NANDCmd(u32 FSMC_Bank, FunctionalState NewState);
void FSMC_PCCARDCmd(FunctionalState NewState);
void FSMC_NANDECCCmd(u32 FSMC_Bank, FunctionalState NewState);
u32 FSMC_GetECC(u32 FSMC_Bank);
void FSMC_ITConfig(u32 FSMC_Bank, u32 FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(u32 FSMC_Bank, u32 FSMC_FLAG);
void FSMC_ClearFlag(u32 FSMC_Bank, u32 FSMC_FLAG);
ITStatus FSMC_GetITStatus(u32 FSMC_Bank, u32 FSMC_IT);
void FSMC_ClearITPendingBit(u32 FSMC_Bank, u32 FSMC_IT);

#endif /*__STM32F10x_FSMC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/





















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_gpio.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      GPIO firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_GPIO_H
#define __STM32F10x_GPIO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

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

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_i2c.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      I2C firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_I2C_H    
#define __STM32F10x_I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* I2C Init structure definition */
typedef struct
{
  u16 I2C_Mode;
  u16 I2C_DutyCycle;
  u16 I2C_OwnAddress1;
  u16 I2C_Ack;
  u16 I2C_AcknowledgedAddress;
  u32 I2C_ClockSpeed;
}I2C_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
#define IS_I2C_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == I2C1_BASE) || \
                                   ((*(u32*)&(PERIPH)) == I2C2_BASE))

/* I2C modes */
#define I2C_Mode_I2C                    ((u16)0x0000)
#define I2C_Mode_SMBusDevice            ((u16)0x0002)
#define I2C_Mode_SMBusHost              ((u16)0x000A)

#define IS_I2C_MODE(MODE) (((MODE) == I2C_Mode_I2C) || \
                           ((MODE) == I2C_Mode_SMBusDevice) || \
                           ((MODE) == I2C_Mode_SMBusHost))
/* I2C duty cycle in fast mode */
#define I2C_DutyCycle_16_9              ((u16)0x4000)
#define I2C_DutyCycle_2                 ((u16)0xBFFF)

#define IS_I2C_DUTY_CYCLE(CYCLE) (((CYCLE) == I2C_DutyCycle_16_9) || \
                                  ((CYCLE) == I2C_DutyCycle_2))

/* I2C cknowledgementy */
#define I2C_Ack_Enable                  ((u16)0x0400)
#define I2C_Ack_Disable                 ((u16)0x0000)

#define IS_I2C_ACK_STATE(STATE) (((STATE) == I2C_Ack_Enable) || \
                                 ((STATE) == I2C_Ack_Disable))

/* I2C transfer direction */
#define  I2C_Direction_Transmitter      ((u8)0x00)
#define  I2C_Direction_Receiver         ((u8)0x01)

#define IS_I2C_DIRECTION(DIRECTION) (((DIRECTION) == I2C_Direction_Transmitter) || \
                                     ((DIRECTION) == I2C_Direction_Receiver))

/* I2C acknowledged address defines */
#define I2C_AcknowledgedAddress_7bit    ((u16)0x4000)
#define I2C_AcknowledgedAddress_10bit   ((u16)0xC000)

#define IS_I2C_ACKNOWLEDGE_ADDRESS(ADDRESS) (((ADDRESS) == I2C_AcknowledgedAddress_7bit) || \
                                             ((ADDRESS) == I2C_AcknowledgedAddress_10bit))

/* I2C registers */
#define I2C_Register_CR1                ((u8)0x00)
#define I2C_Register_CR2                ((u8)0x04)
#define I2C_Register_OAR1               ((u8)0x08)
#define I2C_Register_OAR2               ((u8)0x0C)
#define I2C_Register_DR                 ((u8)0x10)
#define I2C_Register_SR1                ((u8)0x14)
#define I2C_Register_SR2                ((u8)0x18)
#define I2C_Register_CCR                ((u8)0x1C)
#define I2C_Register_TRISE              ((u8)0x20)

#define IS_I2C_REGISTER(REGISTER) (((REGISTER) == I2C_Register_CR1) || \
                                   ((REGISTER) == I2C_Register_CR2) || \
                                   ((REGISTER) == I2C_Register_OAR1) || \
                                   ((REGISTER) == I2C_Register_OAR2) || \
                                   ((REGISTER) == I2C_Register_DR) || \
                                   ((REGISTER) == I2C_Register_SR1) || \
                                   ((REGISTER) == I2C_Register_SR2) || \
                                   ((REGISTER) == I2C_Register_CCR) || \
                                   ((REGISTER) == I2C_Register_TRISE))

/* I2C SMBus alert pin level */
#define I2C_SMBusAlert_Low              ((u16)0x2000)
#define I2C_SMBusAlert_High             ((u16)0xDFFF)

#define IS_I2C_SMBUS_ALERT(ALERT) (((ALERT) == I2C_SMBusAlert_Low) || \
                                   ((ALERT) == I2C_SMBusAlert_High))

/* I2C PEC position */
#define I2C_PECPosition_Next            ((u16)0x0800)
#define I2C_PECPosition_Current         ((u16)0xF7FF)

#define IS_I2C_PEC_POSITION(POSITION) (((POSITION) == I2C_PECPosition_Next) || \
                                       ((POSITION) == I2C_PECPosition_Current))

/* I2C interrupts definition */
#define I2C_IT_BUF                      ((u16)0x0400)
#define I2C_IT_EVT                      ((u16)0x0200)
#define I2C_IT_ERR                      ((u16)0x0100)

#define IS_I2C_CONFIG_IT(IT) ((((IT) & (u16)0xF8FF) == 0x00) && ((IT) != 0x00))

/* I2C interrupts definition */
#define I2C_IT_SMBALERT                 ((u32)0x01008000)
#define I2C_IT_TIMEOUT                  ((u32)0x01004000)
#define I2C_IT_PECERR                   ((u32)0x01001000)
#define I2C_IT_OVR                      ((u32)0x01000800)
#define I2C_IT_AF                       ((u32)0x01000400)
#define I2C_IT_ARLO                     ((u32)0x01000200)
#define I2C_IT_BERR                     ((u32)0x01000100)
#define I2C_IT_TXE                      ((u32)0x06000080)
#define I2C_IT_RXNE                     ((u32)0x06000040)
#define I2C_IT_STOPF                    ((u32)0x02000010)
#define I2C_IT_ADD10                    ((u32)0x02000008)
#define I2C_IT_BTF                      ((u32)0x02000004)
#define I2C_IT_ADDR                     ((u32)0x02000002)
#define I2C_IT_SB                       ((u32)0x02000001)

#define IS_I2C_CLEAR_IT(IT) ((((IT) & (u16)0x20FF) == 0x00) && ((IT) != (u16)0x00))                             

#define IS_I2C_GET_IT(IT) (((IT) == I2C_IT_SMBALERT) || ((IT) == I2C_IT_TIMEOUT) || \
                           ((IT) == I2C_IT_PECERR) || ((IT) == I2C_IT_OVR) || \
                           ((IT) == I2C_IT_AF) || ((IT) == I2C_IT_ARLO) || \
                           ((IT) == I2C_IT_BERR) || ((IT) == I2C_IT_TXE) || \
                           ((IT) == I2C_IT_RXNE) || ((IT) == I2C_IT_STOPF) || \
                           ((IT) == I2C_IT_ADD10) || ((IT) == I2C_IT_BTF) || \
                           ((IT) == I2C_IT_ADDR) || ((IT) == I2C_IT_SB))

/* I2C flags definition */
/* SR2 register flags */
#define I2C_FLAG_DUALF                  ((u32)0x00800000)
#define I2C_FLAG_SMBHOST                ((u32)0x00400000)
#define I2C_FLAG_SMBDEFAULT             ((u32)0x00200000)
#define I2C_FLAG_GENCALL                ((u32)0x00100000)
#define I2C_FLAG_TRA                    ((u32)0x00040000)
#define I2C_FLAG_BUSY                   ((u32)0x00020000)
#define I2C_FLAG_MSL                    ((u32)0x00010000)
/* SR1 register flags */
#define I2C_FLAG_SMBALERT               ((u32)0x10008000)
#define I2C_FLAG_TIMEOUT                ((u32)0x10004000)
#define I2C_FLAG_PECERR                 ((u32)0x10001000)
#define I2C_FLAG_OVR                    ((u32)0x10000800)
#define I2C_FLAG_AF                     ((u32)0x10000400)
#define I2C_FLAG_ARLO                   ((u32)0x10000200)
#define I2C_FLAG_BERR                   ((u32)0x10000100)
#define I2C_FLAG_TXE                    ((u32)0x10000080)
#define I2C_FLAG_RXNE                   ((u32)0x10000040)
#define I2C_FLAG_STOPF                  ((u32)0x10000010)
#define I2C_FLAG_ADD10                  ((u32)0x10000008)
#define I2C_FLAG_BTF                    ((u32)0x10000004)
#define I2C_FLAG_ADDR                   ((u32)0x10000002)
#define I2C_FLAG_SB                     ((u32)0x10000001)
                               
#define IS_I2C_CLEAR_FLAG(FLAG) ((((FLAG) & (u16)0x20FF) == 0x00) && ((FLAG) != (u16)0x00))                                  

#define IS_I2C_GET_FLAG(FLAG) (((FLAG) == I2C_FLAG_DUALF) || ((FLAG) == I2C_FLAG_SMBHOST) || \
                               ((FLAG) == I2C_FLAG_SMBDEFAULT) || ((FLAG) == I2C_FLAG_GENCALL) || \
                               ((FLAG) == I2C_FLAG_TRA) || ((FLAG) == I2C_FLAG_BUSY) || \
                               ((FLAG) == I2C_FLAG_MSL) || ((FLAG) == I2C_FLAG_SMBALERT) || \
                               ((FLAG) == I2C_FLAG_TIMEOUT) || ((FLAG) == I2C_FLAG_PECERR) || \
                               ((FLAG) == I2C_FLAG_OVR) || ((FLAG) == I2C_FLAG_AF) || \
                               ((FLAG) == I2C_FLAG_ARLO) || ((FLAG) == I2C_FLAG_BERR) || \
                               ((FLAG) == I2C_FLAG_TXE) || ((FLAG) == I2C_FLAG_RXNE) || \
                               ((FLAG) == I2C_FLAG_STOPF) || ((FLAG) == I2C_FLAG_ADD10) || \
                               ((FLAG) == I2C_FLAG_BTF) || ((FLAG) == I2C_FLAG_ADDR) || \
                               ((FLAG) == I2C_FLAG_SB))

/* I2C Events */
/* EV1 */
#define  I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED       ((u32)0x00060082) /* TRA, BUSY, TXE and ADDR flags */
#define  I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED          ((u32)0x00020002) /* BUSY and ADDR flags */
#define  I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED ((u32)0x00860080)  /* DUALF, TRA, BUSY and TXE flags */
#define  I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED    ((u32)0x00820000)  /* DUALF and BUSY flags */
#define  I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED        ((u32)0x00120000)  /* GENCALL and BUSY flags */

/* EV2 */
#define  I2C_EVENT_SLAVE_BYTE_RECEIVED                     ((u32)0x00020040)  /* BUSY and RXNE flags */
     
/* EV3 */
#define  I2C_EVENT_SLAVE_BYTE_TRANSMITTED                  ((u32)0x00060084)  /* TRA, BUSY, TXE and BTF flags */

/* EV4 */
#define  I2C_EVENT_SLAVE_STOP_DETECTED                     ((u32)0x00000010)  /* STOPF flag */

/* EV5 */
#define  I2C_EVENT_MASTER_MODE_SELECT                      ((u32)0x00030001)  /* BUSY, MSL and SB flag */

/* EV6 */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((u32)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED           ((u32)0x00030002)  /* BUSY, MSL and ADDR flags */

/* EV7 */
#define  I2C_EVENT_MASTER_BYTE_RECEIVED                    ((u32)0x00030040)  /* BUSY, MSL and RXNE flags */

/* EV8 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((u32)0x00070080) /* TRA, BUSY, MSL, TXE flags */

/* EV8_2 */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((u32)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */
      
/* EV9 */
#define  I2C_EVENT_MASTER_MODE_ADDRESS10                   ((u32)0x00030008)  /* BUSY, MSL and ADD10 flags */
                                          
/* EV3_2 */
#define  I2C_EVENT_SLAVE_ACK_FAILURE                       ((u32)0x00000400)  /* AF flag */

#define IS_I2C_EVENT(EVENT) (((EVENT) == I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) || \
                             ((EVENT) == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) || \
                             ((EVENT) == I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED) || \
                             ((EVENT) == I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED) || \
                             ((EVENT) == I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED) || \
                             ((EVENT) == I2C_EVENT_SLAVE_BYTE_RECEIVED) || \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)) || \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)) || \
                             ((EVENT) == I2C_EVENT_SLAVE_BYTE_TRANSMITTED) || \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)) || \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL)) || \
                             ((EVENT) == I2C_EVENT_SLAVE_STOP_DETECTED) || \
                             ((EVENT) == I2C_EVENT_MASTER_MODE_SELECT) || \
                             ((EVENT) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) || \
                             ((EVENT) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) || \
                             ((EVENT) == I2C_EVENT_MASTER_BYTE_RECEIVED) || \
                             ((EVENT) == I2C_EVENT_MASTER_BYTE_TRANSMITTED) || \
                             ((EVENT) == I2C_EVENT_MASTER_MODE_ADDRESS10) || \
                             ((EVENT) == I2C_EVENT_SLAVE_ACK_FAILURE))

/* I2C own address1 -----------------------------------------------------------*/
#define IS_I2C_OWN_ADDRESS1(ADDRESS1) ((ADDRESS1) <= 0x3FF)
/* I2C clock speed ------------------------------------------------------------*/
#define IS_I2C_CLOCK_SPEED(SPEED) (((SPEED) >= 0x1) && ((SPEED) <= 400000))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void I2C_DeInit(I2C_TypeDef* I2Cx);
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, u8 Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ITConfig(I2C_TypeDef* I2Cx, u16 I2C_IT, FunctionalState NewState);
void I2C_SendData(I2C_TypeDef* I2Cx, u8 Data);
u8 I2C_ReceiveData(I2C_TypeDef* I2Cx);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, u8 Address, u8 I2C_Direction);
u16 I2C_ReadRegister(I2C_TypeDef* I2Cx, u8 I2C_Register);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, u16 I2C_SMBusAlert);
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, u16 I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
u8 I2C_GetPEC(I2C_TypeDef* I2Cx);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, u16 I2C_DutyCycle);
u32 I2C_GetLastEvent(I2C_TypeDef* I2Cx);
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, u32 I2C_EVENT);
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, u32 I2C_FLAG);
void I2C_ClearFlag(I2C_TypeDef* I2Cx, u32 I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, u32 I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, u32 I2C_IT);

#endif /*__STM32F10x_I2C_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_it.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains the headers of the interrupt handlers.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMIException(void);
void HardFaultException(void);
void MemManageException(void);
void BusFaultException(void);
void UsageFaultException(void);
void DebugMonitor(void);
void SVCHandler(void);
void PendSVC(void);
void SysTickHandler(void);
void WWDG_IRQHandler(void);
void PVD_IRQHandler(void);
void TAMPER_IRQHandler(void);
void RTC_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void DMA1_Channel7_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void USB_HP_CAN_TX_IRQHandler(void);
void USB_LP_CAN_RX0_IRQHandler(void);
void CAN_RX1_IRQHandler(void);
void CAN_SCE_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM1_TRG_COM_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void RTCAlarm_IRQHandler(void);
void USBWakeUp_IRQHandler(void);
void TIM8_BRK_IRQHandler(void);
void TIM8_UP_IRQHandler(void);
void TIM8_TRG_COM_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void ADC3_IRQHandler(void);
void FSMC_IRQHandler(void);
void SDIO_IRQHandler(void);
void TIM5_IRQHandler(void);
void SPI3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void TIM6_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);
void DMA2_Channel2_IRQHandler(void);
void DMA2_Channel3_IRQHandler(void);
void DMA2_Channel4_5_IRQHandler(void);

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_iwdg.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      IWDG firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IWDG_H
#define __STM32F10x_IWDG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Write access to IWDG_PR and IWDG_RLR registers */
#define IWDG_WriteAccess_Enable     ((u16)0x5555)
#define IWDG_WriteAccess_Disable    ((u16)0x0000)

#define IS_IWDG_WRITE_ACCESS(ACCESS) (((ACCESS) == IWDG_WriteAccess_Enable) || \
                                      ((ACCESS) == IWDG_WriteAccess_Disable))

/* IWDG prescaler */
#define IWDG_Prescaler_4            ((u8)0x00)
#define IWDG_Prescaler_8            ((u8)0x01)
#define IWDG_Prescaler_16           ((u8)0x02)
#define IWDG_Prescaler_32           ((u8)0x03)
#define IWDG_Prescaler_64           ((u8)0x04)
#define IWDG_Prescaler_128          ((u8)0x05)
#define IWDG_Prescaler_256          ((u8)0x06)

#define IS_IWDG_PRESCALER(PRESCALER) (((PRESCALER) == IWDG_Prescaler_4)  || \
                                      ((PRESCALER) == IWDG_Prescaler_8)  || \
                                      ((PRESCALER) == IWDG_Prescaler_16) || \
                                      ((PRESCALER) == IWDG_Prescaler_32) || \
                                      ((PRESCALER) == IWDG_Prescaler_64) || \
                                      ((PRESCALER) == IWDG_Prescaler_128)|| \
                                      ((PRESCALER) == IWDG_Prescaler_256))

/* IWDG Flag */
#define IWDG_FLAG_PVU               ((u16)0x0001)
#define IWDG_FLAG_RVU               ((u16)0x0002)

#define IS_IWDG_FLAG(FLAG) (((FLAG) == IWDG_FLAG_PVU) || ((FLAG) == IWDG_FLAG_RVU))

#define IS_IWDG_RELOAD(RELOAD) ((RELOAD) <= 0xFFF)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void IWDG_WriteAccessCmd(u16 IWDG_WriteAccess);
void IWDG_SetPrescaler(u8 IWDG_Prescaler);
void IWDG_SetReload(u16 Reload);
void IWDG_ReloadCounter(void);
void IWDG_Enable(void);
FlagStatus IWDG_GetFlagStatus(u16 IWDG_FLAG);

#endif /* __STM32F10x_IWDG_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_lib.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file includes the peripherals header files in the
*                      user application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_LIB_H
#define __STM32F10x_LIB_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

#ifdef _ADC
  #include "stm32f10x_adc.h"
#endif /*_ADC */

#ifdef _BKP
  #include "stm32f10x_bkp.h"
#endif /*_BKP */

#ifdef _CAN
  #include "stm32f10x_can.h"
#endif /*_CAN */

#ifdef _CRC
  #include "stm32f10x_crc.h"
#endif /*_CRC */

#ifdef _DAC
  #include "stm32f10x_dac.h"
#endif /*_DAC */

#ifdef _DBGMCU
  #include "stm32f10x_dbgmcu.h"
#endif /*_DBGMCU */

#ifdef _DMA
  #include "stm32f10x_dma.h"
#endif /*_DMA */

#ifdef _EXTI
  #include "stm32f10x_exti.h"
#endif /*_EXTI */

#ifdef _FLASH
  #include "stm32f10x_flash.h"
#endif /*_FLASH */

#ifdef _FSMC
  #include "stm32f10x_fsmc.h"
#endif /*_FSMC */

#ifdef _GPIO
  #include "stm32f10x_gpio.h"
#endif /*_GPIO */

#ifdef _I2C
  #include "stm32f10x_i2c.h"
#endif /*_I2C */

#ifdef _IWDG
  #include "stm32f10x_iwdg.h"
#endif /*_IWDG */

#ifdef _NVIC
  #include "stm32f10x_nvic.h"
#endif /*_NVIC */

#ifdef _PWR
  #include "stm32f10x_pwr.h"
#endif /*_PWR */

#ifdef _RCC
  #include "stm32f10x_rcc.h"
#endif /*_RCC */

#ifdef _RTC
  #include "stm32f10x_rtc.h"
#endif /*_RTC */

#ifdef _SDIO
  #include "stm32f10x_sdio.h"
#endif /*_SDIO */

#ifdef _SPI
  #include "stm32f10x_spi.h"
#endif /*_SPI */

#ifdef _SysTick
  #include "stm32f10x_systick.h"
#endif /*_SysTick */

#ifdef _TIM
  #include "stm32f10x_tim.h"
#endif /*_TIM */

#ifdef _USART
  #include "stm32f10x_usart.h"
#endif /*_USART */

#ifdef _WWDG
  #include "stm32f10x_wwdg.h"
#endif /*_WWDG */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void debug(void);

#endif /* __STM32F10x_LIB_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_map.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the peripheral register's definitions,
*                      bits definitions and memory mapping.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_MAP_H
#define __STM32F10x_MAP_H

#ifndef EXT
  #define EXT extern
#endif /* EXT */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x_type.h"
#include "cortexm3_macro.h"

/* Exported types ------------------------------------------------------------*/
/******************************************************************************/
/*                         Peripheral registers structures                    */
/******************************************************************************/

/*------------------------ Analog to Digital Converter -----------------------*/
typedef struct
{
  vu32 SR;
  vu32 CR1;
  vu32 CR2;
  vu32 SMPR1;
  vu32 SMPR2;
  vu32 JOFR1;
  vu32 JOFR2;
  vu32 JOFR3;
  vu32 JOFR4;
  vu32 HTR;
  vu32 LTR;
  vu32 SQR1;
  vu32 SQR2;
  vu32 SQR3;
  vu32 JSQR;
  vu32 JDR1;
  vu32 JDR2;
  vu32 JDR3;
  vu32 JDR4;
  vu32 DR;
} ADC_TypeDef;

/*------------------------ Backup Registers ----------------------------------*/
typedef struct
{
  u32  RESERVED0;
  vu16 DR1;
  u16  RESERVED1;
  vu16 DR2;
  u16  RESERVED2;
  vu16 DR3;
  u16  RESERVED3;
  vu16 DR4;
  u16  RESERVED4;
  vu16 DR5;
  u16  RESERVED5;
  vu16 DR6;
  u16  RESERVED6;
  vu16 DR7;
  u16  RESERVED7;
  vu16 DR8;
  u16  RESERVED8;
  vu16 DR9;
  u16  RESERVED9;
  vu16 DR10;
  u16  RESERVED10; 
  vu16 RTCCR;
  u16  RESERVED11;
  vu16 CR;
  u16  RESERVED12;
  vu16 CSR;
  u16  RESERVED13[5];
  vu16 DR11;
  u16  RESERVED14;
  vu16 DR12;
  u16  RESERVED15;
  vu16 DR13;
  u16  RESERVED16;
  vu16 DR14;
  u16  RESERVED17;
  vu16 DR15;
  u16  RESERVED18;
  vu16 DR16;
  u16  RESERVED19;
  vu16 DR17;
  u16  RESERVED20;
  vu16 DR18;
  u16  RESERVED21;
  vu16 DR19;
  u16  RESERVED22;
  vu16 DR20;
  u16  RESERVED23;
  vu16 DR21;
  u16  RESERVED24;
  vu16 DR22;
  u16  RESERVED25;
  vu16 DR23;
  u16  RESERVED26;
  vu16 DR24;
  u16  RESERVED27;
  vu16 DR25;
  u16  RESERVED28;
  vu16 DR26;
  u16  RESERVED29;
  vu16 DR27;
  u16  RESERVED30;
  vu16 DR28;
  u16  RESERVED31;
  vu16 DR29;
  u16  RESERVED32;
  vu16 DR30;
  u16  RESERVED33; 
  vu16 DR31;
  u16  RESERVED34;
  vu16 DR32;
  u16  RESERVED35;
  vu16 DR33;
  u16  RESERVED36;
  vu16 DR34;
  u16  RESERVED37;
  vu16 DR35;
  u16  RESERVED38;
  vu16 DR36;
  u16  RESERVED39;
  vu16 DR37;
  u16  RESERVED40;
  vu16 DR38;
  u16  RESERVED41;
  vu16 DR39;
  u16  RESERVED42;
  vu16 DR40;
  u16  RESERVED43;
  vu16 DR41;
  u16  RESERVED44;
  vu16 DR42;
  u16  RESERVED45;    
} BKP_TypeDef;

/*------------------------ Controller Area Network ---------------------------*/
typedef struct
{
  vu32 TIR;
  vu32 TDTR;
  vu32 TDLR;
  vu32 TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  vu32 RIR;
  vu32 RDTR;
  vu32 RDLR;
  vu32 RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  vu32 FR1;
  vu32 FR2;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  vu32 MCR;
  vu32 MSR;
  vu32 TSR;
  vu32 RF0R;
  vu32 RF1R;
  vu32 IER;
  vu32 ESR;
  vu32 BTR;
  u32  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  u32  RESERVED1[12];
  vu32 FMR;
  vu32 FM1R;
  u32  RESERVED2;
  vu32 FS1R;
  u32  RESERVED3;
  vu32 FFA1R;
  u32  RESERVED4;
  vu32 FA1R;
  u32  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

/*------------------------ CRC calculation unit ------------------------------*/
typedef struct
{
  vu32 DR;
  vu8  IDR;
  u8   RESERVED0;
  u16  RESERVED1;
  vu32 CR;
} CRC_TypeDef;


/*------------------------ Digital to Analog Converter -----------------------*/
typedef struct
{
  vu32 CR;
  vu32 SWTRIGR;
  vu32 DHR12R1;
  vu32 DHR12L1;
  vu32 DHR8R1;
  vu32 DHR12R2;
  vu32 DHR12L2;
  vu32 DHR8R2;
  vu32 DHR12RD;
  vu32 DHR12LD;
  vu32 DHR8RD;
  vu32 DOR1;
  vu32 DOR2;
} DAC_TypeDef;

/*------------------------ Debug MCU -----------------------------------------*/
typedef struct
{
  vu32 IDCODE;
  vu32 CR;	
}DBGMCU_TypeDef;

/*------------------------ DMA Controller ------------------------------------*/
typedef struct
{
  vu32 CCR;
  vu32 CNDTR;
  vu32 CPAR;
  vu32 CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  vu32 ISR;
  vu32 IFCR;
} DMA_TypeDef;

/*------------------------ External Interrupt/Event Controller ---------------*/
typedef struct
{
  vu32 IMR;
  vu32 EMR;
  vu32 RTSR;
  vu32 FTSR;
  vu32 SWIER;
  vu32 PR;
} EXTI_TypeDef;

/*------------------------ FLASH and Option Bytes Registers ------------------*/
typedef struct
{
  vu32 ACR;
  vu32 KEYR;
  vu32 OPTKEYR;
  vu32 SR;
  vu32 CR;
  vu32 AR;
  vu32 RESERVED;
  vu32 OBR;
  vu32 WRPR;
} FLASH_TypeDef;

typedef struct
{
  vu16 RDP;
  vu16 USER;
  vu16 Data0;
  vu16 Data1;
  vu16 WRP0;
  vu16 WRP1;
  vu16 WRP2;
  vu16 WRP3;
} OB_TypeDef;

/*------------------------ Flexible Static Memory Controller -----------------*/
typedef struct
{
  vu32 BTCR[8];   
} FSMC_Bank1_TypeDef; 

typedef struct
{
  vu32 BWTR[7];
} FSMC_Bank1E_TypeDef;

typedef struct
{
  vu32 PCR2;
  vu32 SR2;
  vu32 PMEM2;
  vu32 PATT2;
  u32  RESERVED0;   
  vu32 ECCR2; 
} FSMC_Bank2_TypeDef;  

typedef struct
{
  vu32 PCR3;
  vu32 SR3;
  vu32 PMEM3;
  vu32 PATT3;
  u32  RESERVED0;   
  vu32 ECCR3; 
} FSMC_Bank3_TypeDef; 

typedef struct
{
  vu32 PCR4;
  vu32 SR4;
  vu32 PMEM4;
  vu32 PATT4;
  vu32 PIO4; 
} FSMC_Bank4_TypeDef; 

/*------------------------ General Purpose and Alternate Function IO ---------*/
typedef struct
{
  vu32 CRL;
  vu32 CRH;
  vu32 IDR;
  vu32 ODR;
  vu32 BSRR;
  vu32 BRR;
  vu32 LCKR;
} GPIO_TypeDef;

typedef struct
{
  vu32 EVCR;
  vu32 MAPR;
  vu32 EXTICR[4];
} AFIO_TypeDef;

/*------------------------ Inter-integrated Circuit Interface ----------------*/
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 OAR1;
  u16  RESERVED2;
  vu16 OAR2;
  u16  RESERVED3;
  vu16 DR;
  u16  RESERVED4;
  vu16 SR1;
  u16  RESERVED5;
  vu16 SR2;
  u16  RESERVED6;
  vu16 CCR;
  u16  RESERVED7;
  vu16 TRISE;
  u16  RESERVED8;
} I2C_TypeDef;

/*------------------------ Independent WATCHDOG ------------------------------*/
typedef struct
{
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

/*------------------------ Nested Vectored Interrupt Controller --------------*/
typedef struct
{
  vu32 ISER[2];
  u32  RESERVED0[30];
  vu32 ICER[2];
  u32  RSERVED1[30];
  vu32 ISPR[2];
  u32  RESERVED2[30];
  vu32 ICPR[2];
  u32  RESERVED3[30];
  vu32 IABR[2];
  u32  RESERVED4[62];
  vu32 IPR[15];
} NVIC_TypeDef;

typedef struct
{
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

/*------------------------ Power Control -------------------------------------*/
typedef struct
{
  vu32 CR;
  vu32 CSR;
} PWR_TypeDef;

/*------------------------ Reset and Clock Control ---------------------------*/
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_TypeDef;

/*------------------------ Real-Time Clock -----------------------------------*/
typedef struct
{
  vu16 CRH;
  u16  RESERVED0;
  vu16 CRL;
  u16  RESERVED1;
  vu16 PRLH;
  u16  RESERVED2;
  vu16 PRLL;
  u16  RESERVED3;
  vu16 DIVH;
  u16  RESERVED4;
  vu16 DIVL;
  u16  RESERVED5;
  vu16 CNTH;
  u16  RESERVED6;
  vu16 CNTL;
  u16  RESERVED7;
  vu16 ALRH;
  u16  RESERVED8;
  vu16 ALRL;
  u16  RESERVED9;
} RTC_TypeDef;

/*------------------------ SD host Interface ---------------------------------*/
typedef struct
{
  vu32 POWER;
  vu32 CLKCR;
  vu32 ARG;
  vu32 CMD;
  vuc32 RESPCMD;
  vuc32 RESP1;
  vuc32 RESP2;
  vuc32 RESP3;
  vuc32 RESP4;
  vu32 DTIMER;
  vu32 DLEN;
  vu32 DCTRL;
  vuc32 DCOUNT;
  vuc32 STA;
  vu32 ICR;
  vu32 MASK;
  u32  RESERVED0[2];
  vuc32 FIFOCNT;
  u32  RESERVED1[13];
  vu32 FIFO;
} SDIO_TypeDef;

/*------------------------ Serial Peripheral Interface -----------------------*/
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SR;
  u16  RESERVED2;
  vu16 DR;
  u16  RESERVED3;
  vu16 CRCPR;
  u16  RESERVED4;
  vu16 RXCRCR;
  u16  RESERVED5;
  vu16 TXCRCR;
  u16  RESERVED6;
  vu16 I2SCFGR;
  u16  RESERVED7;
  vu16 I2SPR;
  u16  RESERVED8;  
} SPI_TypeDef;

/*------------------------ SystemTick ----------------------------------------*/
typedef struct
{
  vu32 CTRL;
  vu32 LOAD;
  vu32 VAL;
  vuc32 CALIB;
} SysTick_TypeDef;

/*------------------------ TIM -----------------------------------------------*/
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SMCR;
  u16  RESERVED2;
  vu16 DIER;
  u16  RESERVED3;
  vu16 SR;
  u16  RESERVED4;
  vu16 EGR;
  u16  RESERVED5;
  vu16 CCMR1;
  u16  RESERVED6;
  vu16 CCMR2;
  u16  RESERVED7;
  vu16 CCER;
  u16  RESERVED8;
  vu16 CNT;
  u16  RESERVED9;
  vu16 PSC;
  u16  RESERVED10;
  vu16 ARR;
  u16  RESERVED11;
  vu16 RCR;
  u16  RESERVED12;
  vu16 CCR1;
  u16  RESERVED13;
  vu16 CCR2;
  u16  RESERVED14;
  vu16 CCR3;
  u16  RESERVED15;
  vu16 CCR4;
  u16  RESERVED16;
  vu16 BDTR;
  u16  RESERVED17;
  vu16 DCR;
  u16  RESERVED18;
  vu16 DMAR;
  u16  RESERVED19;
} TIM_TypeDef;

/*----------------- Universal Synchronous Asynchronous Receiver Transmitter --*/
typedef struct
{
  vu16 SR;
  u16  RESERVED0;
  vu16 DR;
  u16  RESERVED1;
  vu16 BRR;
  u16  RESERVED2;
  vu16 CR1;
  u16  RESERVED3;
  vu16 CR2;
  u16  RESERVED4;
  vu16 CR3;
  u16  RESERVED5;
  vu16 GTPR;
  u16  RESERVED6;
} USART_TypeDef;

/*------------------------ Window WATCHDOG -----------------------------------*/
typedef struct
{
  vu32 CR;
  vu32 CFR;
  vu32 SR;
} WWDG_TypeDef;

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address in the alias region */
#define PERIPH_BB_BASE        ((u32)0x42000000)
#define SRAM_BB_BASE          ((u32)0x22000000)

/* Peripheral and SRAM base address in the bit-band region */
#define SRAM_BASE             ((u32)0x20000000)
#define PERIPH_BASE           ((u32)0x40000000)

/* FSMC registers base address */
#define FSMC_R_BASE           ((u32)0xA0000000)

/* Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define CAN_BASE              (APB1PERIPH_BASE + 0x6400)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x3400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x3C00)

#define SDIO_BASE             (PERIPH_BASE + 0x18000)

#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x0080)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x0408)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x041C)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x0430)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x0444)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x0458)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)

/* Flash registers base address */
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000)
/* Flash Option Bytes base address */
#define OB_BASE               ((u32)0x1FFFF800)

/* FSMC Bankx registers base address */
#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000)
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104)
#define FSMC_Bank2_R_BASE     (FSMC_R_BASE + 0x0060)
#define FSMC_Bank3_R_BASE     (FSMC_R_BASE + 0x0080)
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0)

/* Debug MCU registers base address */
#define DBGMCU_BASE          ((u32)0xE0042000)

/* System Control Space memory map */
#define SCS_BASE              ((u32)0xE000E000)

#define SysTick_BASE          (SCS_BASE + 0x0010)
#define NVIC_BASE             (SCS_BASE + 0x0100)
#define SCB_BASE              (SCS_BASE + 0x0D00)

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/*------------------------ Non Debug Mode ------------------------------------*/
#ifndef DEBUG
#ifdef _TIM2
  #define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#endif /*_TIM2 */

#ifdef _TIM3
  #define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#endif /*_TIM3 */

#ifdef _TIM4
  #define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#endif /*_TIM4 */

#ifdef _TIM5
  #define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#endif /*_TIM5 */

#ifdef _TIM6
  #define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#endif /*_TIM6 */

#ifdef _TIM7
  #define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#endif /*_TIM7 */

#ifdef _RTC
  #define RTC                 ((RTC_TypeDef *) RTC_BASE)
#endif /*_RTC */

#ifdef _WWDG
  #define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#endif /*_WWDG */

#ifdef _IWDG
  #define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#endif /*_IWDG */

#ifdef _SPI2
  #define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#endif /*_SPI2 */

#ifdef _SPI3
  #define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#endif /*_SPI3 */

#ifdef _USART2
  #define USART2              ((USART_TypeDef *) USART2_BASE)
#endif /*_USART2 */

#ifdef _USART3
  #define USART3              ((USART_TypeDef *) USART3_BASE)
#endif /*_USART3 */

#ifdef _UART4
  #define UART4              ((USART_TypeDef *) UART4_BASE)
#endif /*_UART4 */

#ifdef _UART5
  #define UART5              ((USART_TypeDef *) UART5_BASE)
#endif /*_USART5 */

#ifdef _I2C1
  #define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#endif /*_I2C1 */

#ifdef _I2C2
  #define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#endif /*_I2C2 */

#ifdef _CAN
  #define CAN                 ((CAN_TypeDef *) CAN_BASE)
#endif /*_CAN */

#ifdef _BKP
  #define BKP                 ((BKP_TypeDef *) BKP_BASE)
#endif /*_BKP */

#ifdef _PWR
  #define PWR                 ((PWR_TypeDef *) PWR_BASE)
#endif /*_PWR */

#ifdef _DAC
  #define DAC                 ((DAC_TypeDef *) DAC_BASE)
#endif /*_DAC */

#ifdef _AFIO
  #define AFIO                ((AFIO_TypeDef *) AFIO_BASE)
#endif /*_AFIO */

#ifdef _EXTI
  #define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#endif /*_EXTI */

#ifdef _GPIOA
  #define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#endif /*_GPIOA */

#ifdef _GPIOB
  #define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#endif /*_GPIOB */

#ifdef _GPIOC
  #define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#endif /*_GPIOC */

#ifdef _GPIOD
  #define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#endif /*_GPIOD */

#ifdef _GPIOE
  #define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#endif /*_GPIOE */

#ifdef _GPIOF
  #define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#endif /*_GPIOF */

#ifdef _GPIOG
  #define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#endif /*_GPIOG */

#ifdef _ADC1
  #define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#endif /*_ADC1 */

#ifdef _ADC2
  #define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#endif /*_ADC2 */

#ifdef _TIM1
  #define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#endif /*_TIM1 */

#ifdef _SPI1
  #define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#endif /*_SPI1 */

#ifdef _TIM8
  #define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#endif /*_TIM8 */

#ifdef _USART1
  #define USART1              ((USART_TypeDef *) USART1_BASE)
#endif /*_USART1 */

#ifdef _ADC3
  #define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#endif /*_ADC3 */

#ifdef _SDIO
  #define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#endif /*_SDIO */

#ifdef _DMA
  #define DMA1                ((DMA_TypeDef *) DMA1_BASE)
  #define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#endif /*_DMA */

#ifdef _DMA1_Channel1
  #define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#endif /*_DMA1_Channel1 */

#ifdef _DMA1_Channel2
  #define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#endif /*_DMA1_Channel2 */

#ifdef _DMA1_Channel3
  #define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#endif /*_DMA1_Channel3 */

#ifdef _DMA1_Channel4
  #define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#endif /*_DMA1_Channel4 */

#ifdef _DMA1_Channel5
  #define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#endif /*_DMA1_Channel5 */

#ifdef _DMA1_Channel6
  #define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#endif /*_DMA1_Channel6 */

#ifdef _DMA1_Channel7
  #define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#endif /*_DMA1_Channel7 */

#ifdef _DMA2_Channel1
  #define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#endif /*_DMA2_Channel1 */

#ifdef _DMA2_Channel2
  #define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#endif /*_DMA2_Channel2 */

#ifdef _DMA2_Channel3
  #define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#endif /*_DMA2_Channel3 */

#ifdef _DMA2_Channel4
  #define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#endif /*_DMA2_Channel4 */

#ifdef _DMA2_Channel5
  #define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#endif /*_DMA2_Channel5 */

#ifdef _RCC
  #define RCC                 ((RCC_TypeDef *) RCC_BASE)
#endif /*_RCC */

#ifdef _CRC
  #define CRC                 ((CRC_TypeDef *) CRC_BASE)
#endif /*_CRC */

#ifdef _FLASH
  #define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
  #define OB                  ((OB_TypeDef *) OB_BASE) 
#endif /*_FLASH */

#ifdef _FSMC
  #define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
  #define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
  #define FSMC_Bank2          ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
  #define FSMC_Bank3          ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
  #define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#endif /*_FSMC */

#ifdef _DBGMCU
  #define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#endif /*_DBGMCU */

#ifdef _SysTick
  #define SysTick             ((SysTick_TypeDef *) SysTick_BASE)
#endif /*_SysTick */

#ifdef _NVIC
  #define NVIC                ((NVIC_TypeDef *) NVIC_BASE)
  #define SCB                 ((SCB_TypeDef *) SCB_BASE)  
#endif /*_NVIC */

/*------------------------ Debug Mode ----------------------------------------*/
#else   /* DEBUG */
#ifdef _TIM2
  EXT TIM_TypeDef             *TIM2;
#endif /*_TIM2 */

#ifdef _TIM3
  EXT TIM_TypeDef             *TIM3;
#endif /*_TIM3 */

#ifdef _TIM4
  EXT TIM_TypeDef             *TIM4;
#endif /*_TIM4 */

#ifdef _TIM5
  EXT TIM_TypeDef             *TIM5;
#endif /*_TIM5 */

#ifdef _TIM6
  EXT TIM_TypeDef             *TIM6;
#endif /*_TIM6 */

#ifdef _TIM7
  EXT TIM_TypeDef             *TIM7;
#endif /*_TIM7 */

#ifdef _RTC
  EXT RTC_TypeDef             *RTC;
#endif /*_RTC */

#ifdef _WWDG
  EXT WWDG_TypeDef            *WWDG;
#endif /*_WWDG */

#ifdef _IWDG
  EXT IWDG_TypeDef            *IWDG;
#endif /*_IWDG */

#ifdef _SPI2
  EXT SPI_TypeDef             *SPI2;
#endif /*_SPI2 */

#ifdef _SPI3
  EXT SPI_TypeDef             *SPI3;
#endif /*_SPI3 */

#ifdef _USART2
  EXT USART_TypeDef           *USART2;
#endif /*_USART2 */

#ifdef _USART3
  EXT USART_TypeDef           *USART3;
#endif /*_USART3 */

#ifdef _UART4
  EXT USART_TypeDef           *UART4;
#endif /*_UART4 */

#ifdef _UART5
  EXT USART_TypeDef           *UART5;
#endif /*_UART5 */

#ifdef _I2C1
  EXT I2C_TypeDef             *I2C1;
#endif /*_I2C1 */

#ifdef _I2C2
  EXT I2C_TypeDef             *I2C2;
#endif /*_I2C2 */

#ifdef _CAN
  EXT CAN_TypeDef             *CAN;
#endif /*_CAN */

#ifdef _BKP
  EXT BKP_TypeDef             *BKP;
#endif /*_BKP */

#ifdef _PWR
  EXT PWR_TypeDef             *PWR;
#endif /*_PWR */

#ifdef _DAC
  EXT DAC_TypeDef             *DAC;
#endif /*_DAC */

#ifdef _AFIO
  EXT AFIO_TypeDef            *AFIO;
#endif /*_AFIO */

#ifdef _EXTI
  EXT EXTI_TypeDef            *EXTI;
#endif /*_EXTI */

#ifdef _GPIOA
  EXT GPIO_TypeDef            *GPIOA;
#endif /*_GPIOA */

#ifdef _GPIOB
  EXT GPIO_TypeDef            *GPIOB;
#endif /*_GPIOB */

#ifdef _GPIOC
  EXT GPIO_TypeDef            *GPIOC;
#endif /*_GPIOC */

#ifdef _GPIOD
  EXT GPIO_TypeDef            *GPIOD;
#endif /*_GPIOD */

#ifdef _GPIOE
  EXT GPIO_TypeDef            *GPIOE;
#endif /*_GPIOE */

#ifdef _GPIOF
  EXT GPIO_TypeDef            *GPIOF;
#endif /*_GPIOF */

#ifdef _GPIOG
  EXT GPIO_TypeDef            *GPIOG;
#endif /*_GPIOG */

#ifdef _ADC1
  EXT ADC_TypeDef             *ADC1;
#endif /*_ADC1 */

#ifdef _ADC2
  EXT ADC_TypeDef             *ADC2;
#endif /*_ADC2 */

#ifdef _TIM1
  EXT TIM_TypeDef             *TIM1;
#endif /*_TIM1 */

#ifdef _SPI1
  EXT SPI_TypeDef             *SPI1;
#endif /*_SPI1 */

#ifdef _TIM8
  EXT TIM_TypeDef             *TIM8;
#endif /*_TIM8 */

#ifdef _USART1
  EXT USART_TypeDef           *USART1;
#endif /*_USART1 */

#ifdef _ADC3
  EXT ADC_TypeDef             *ADC3;
#endif /*_ADC3 */

#ifdef _SDIO
  EXT SDIO_TypeDef            *SDIO;
#endif /*_SDIO */

#ifdef _DMA
  EXT DMA_TypeDef             *DMA1;
  EXT DMA_TypeDef             *DMA2;
#endif /*_DMA */

#ifdef _DMA1_Channel1
  EXT DMA_Channel_TypeDef     *DMA1_Channel1;
#endif /*_DMA1_Channel1 */

#ifdef _DMA1_Channel2
  EXT DMA_Channel_TypeDef     *DMA1_Channel2;
#endif /*_DMA1_Channel2 */

#ifdef _DMA1_Channel3
  EXT DMA_Channel_TypeDef     *DMA1_Channel3;
#endif /*_DMA1_Channel3 */

#ifdef _DMA1_Channel4
  EXT DMA_Channel_TypeDef     *DMA1_Channel4;
#endif /*_DMA1_Channel4 */

#ifdef _DMA1_Channel5
  EXT DMA_Channel_TypeDef     *DMA1_Channel5;
#endif /*_DMA1_Channel5 */

#ifdef _DMA1_Channel6
  EXT DMA_Channel_TypeDef     *DMA1_Channel6;
#endif /*_DMA1_Channel6 */

#ifdef _DMA1_Channel7
  EXT DMA_Channel_TypeDef     *DMA1_Channel7;
#endif /*_DMA1_Channel7 */

#ifdef _DMA2_Channel1
  EXT DMA_Channel_TypeDef     *DMA2_Channel1;
#endif /*_DMA2_Channel1 */

#ifdef _DMA2_Channel2
  EXT DMA_Channel_TypeDef     *DMA2_Channel2;
#endif /*_DMA2_Channel2 */

#ifdef _DMA2_Channel3
  EXT DMA_Channel_TypeDef     *DMA2_Channel3;
#endif /*_DMA2_Channel3 */

#ifdef _DMA2_Channel4
  EXT DMA_Channel_TypeDef     *DMA2_Channel4;
#endif /*_DMA2_Channel4 */

#ifdef _DMA2_Channel5
  EXT DMA_Channel_TypeDef     *DMA2_Channel5;
#endif /*_DMA2_Channel5 */

#ifdef _RCC
  EXT RCC_TypeDef             *RCC;
#endif /*_RCC */

#ifdef _CRC
  EXT CRC_TypeDef             *CRC;
#endif /*_CRC */

#ifdef _FLASH
  EXT FLASH_TypeDef            *FLASH;
  EXT OB_TypeDef               *OB;  
#endif /*_FLASH */

#ifdef _FSMC
  EXT FSMC_Bank1_TypeDef      *FSMC_Bank1;
  EXT FSMC_Bank1E_TypeDef     *FSMC_Bank1E;
  EXT FSMC_Bank2_TypeDef      *FSMC_Bank2;
  EXT FSMC_Bank3_TypeDef      *FSMC_Bank3;
  EXT FSMC_Bank4_TypeDef      *FSMC_Bank4;
#endif /*_FSMC */

#ifdef _DBGMCU
  EXT DBGMCU_TypeDef          *DBGMCU;
#endif /*_DBGMCU */

#ifdef _SysTick
  EXT SysTick_TypeDef         *SysTick;
#endif /*_SysTick */

#ifdef _NVIC
  EXT NVIC_TypeDef            *NVIC;
  EXT SCB_TypeDef             *SCB;
#endif /*_NVIC */

#endif  /* DEBUG */

/* Exported constants --------------------------------------------------------*/
/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((u32)0xFFFFFFFF) /* Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((u8)0xFF)        /* General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((u8)0x01)        /* RESET bit */



/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((u16)0x0001)     /* Low-Power Deepsleep */
#define  PWR_CR_PDDS                         ((u16)0x0002)     /* Power Down Deepsleep */
#define  PWR_CR_CWUF                         ((u16)0x0004)     /* Clear Wakeup Flag */
#define  PWR_CR_CSBF                         ((u16)0x0008)     /* Clear Standby Flag */
#define  PWR_CR_PVDE                         ((u16)0x0010)     /* Power Voltage Detector Enable */

#define  PWR_CR_PLS                          ((u16)0x00E0)     /* PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((u16)0x0020)     /* Bit 0 */
#define  PWR_CR_PLS_1                        ((u16)0x0040)     /* Bit 1 */
#define  PWR_CR_PLS_2                        ((u16)0x0080)     /* Bit 2 */

/* PVD level configuration */
#define  PWR_CR_PLS_2V2                      ((u16)0x0000)     /* PVD level 2.2V */
#define  PWR_CR_PLS_2V3                      ((u16)0x0020)     /* PVD level 2.3V */
#define  PWR_CR_PLS_2V4                      ((u16)0x0040)     /* PVD level 2.4V */
#define  PWR_CR_PLS_2V5                      ((u16)0x0060)     /* PVD level 2.5V */
#define  PWR_CR_PLS_2V6                      ((u16)0x0080)     /* PVD level 2.6V */
#define  PWR_CR_PLS_2V7                      ((u16)0x00A0)     /* PVD level 2.7V */
#define  PWR_CR_PLS_2V8                      ((u16)0x00C0)     /* PVD level 2.8V */
#define  PWR_CR_PLS_2V9                      ((u16)0x00E0)     /* PVD level 2.9V */

#define  PWR_CR_DBP                          ((u16)0x0100)     /* Disable Backup Domain write protection */


/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((u16)0x0001)     /* Wakeup Flag */
#define  PWR_CSR_SBF                         ((u16)0x0002)     /* Standby Flag */
#define  PWR_CSR_PVDO                        ((u16)0x0004)     /* PVD Output */
#define  PWR_CSR_EWUP                        ((u16)0x0100)     /* Enable WKUP pin */



/******************************************************************************/
/*                                                                            */
/*                            Backup registers                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for BKP_DR1 register  ********************/
#define  BKP_DR1_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR2 register  ********************/
#define  BKP_DR2_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR3 register  ********************/
#define  BKP_DR3_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR4 register  ********************/
#define  BKP_DR4_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR5 register  ********************/
#define  BKP_DR5_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR6 register  ********************/
#define  BKP_DR6_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR7 register  ********************/
#define  BKP_DR7_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR8 register  ********************/
#define  BKP_DR8_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR9 register  ********************/
#define  BKP_DR9_D                           ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR10 register  *******************/
#define  BKP_DR10_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR11 register  *******************/
#define  BKP_DR11_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR12 register  *******************/
#define  BKP_DR12_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR13 register  *******************/
#define  BKP_DR13_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR14 register  *******************/
#define  BKP_DR14_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR15 register  *******************/
#define  BKP_DR15_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR16 register  *******************/
#define  BKP_DR16_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR17 register  *******************/
#define  BKP_DR17_D                          ((u16)0xFFFF)     /* Backup data */


/******************  Bit definition for BKP_DR18 register  ********************/
#define  BKP_DR18_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR19 register  *******************/
#define  BKP_DR19_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR20 register  *******************/
#define  BKP_DR20_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR21 register  *******************/
#define  BKP_DR21_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR22 register  *******************/
#define  BKP_DR22_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR23 register  *******************/
#define  BKP_DR23_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR24 register  *******************/
#define  BKP_DR24_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR25 register  *******************/
#define  BKP_DR25_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR26 register  *******************/
#define  BKP_DR26_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR27 register  *******************/
#define  BKP_DR27_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR28 register  *******************/
#define  BKP_DR28_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR29 register  *******************/
#define  BKP_DR29_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR30 register  *******************/
#define  BKP_DR30_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR31 register  *******************/
#define  BKP_DR31_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR32 register  *******************/
#define  BKP_DR32_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR33 register  *******************/
#define  BKP_DR33_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR34 register  *******************/
#define  BKP_DR34_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR35 register  *******************/
#define  BKP_DR35_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR36 register  *******************/
#define  BKP_DR36_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR37 register  *******************/
#define  BKP_DR37_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR38 register  *******************/
#define  BKP_DR38_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR39 register  *******************/
#define  BKP_DR39_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR40 register  *******************/
#define  BKP_DR40_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR41 register  *******************/
#define  BKP_DR41_D                          ((u16)0xFFFF)     /* Backup data */


/*******************  Bit definition for BKP_DR42 register  *******************/
#define  BKP_DR42_D                          ((u16)0xFFFF)     /* Backup data */


/******************  Bit definition for BKP_RTCCR register  *******************/
#define  BKP_RTCCR_CAL                       ((u16)0x007F)     /* Calibration value */
#define  BKP_RTCCR_CCO                       ((u16)0x0080)     /* Calibration Clock Output */
#define  BKP_RTCCR_ASOE                      ((u16)0x0100)     /* Alarm or Second Output Enable */
#define  BKP_RTCCR_ASOS                      ((u16)0x0200)     /* Alarm or Second Output Selection */


/********************  Bit definition for BKP_CR register  ********************/
#define  BKP_CR_TPE                          ((u8)0x01)        /* TAMPER pin enable */
#define  BKP_CR_TPAL                         ((u8)0x02)        /* TAMPER pin active level */


/*******************  Bit definition for BKP_CSR register  ********************/
#define  BKP_CSR_CTE                         ((u16)0x0001)     /* Clear Tamper event */
#define  BKP_CSR_CTI                         ((u16)0x0002)     /* Clear Tamper Interrupt */
#define  BKP_CSR_TPIE                        ((u16)0x0004)     /* TAMPER Pin interrupt enable */
#define  BKP_CSR_TEF                         ((u16)0x0100)     /* Tamper Event Flag */
#define  BKP_CSR_TIF                         ((u16)0x0200)     /* Tamper Interrupt Flag */



/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/


/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((u32)0x00000001)        /* Internal High Speed clock enable */
#define  RCC_CR_HSIRDY                       ((u32)0x00000002)        /* Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM                      ((u32)0x000000F8)        /* Internal High Speed clock trimming */
#define  RCC_CR_HSICAL                       ((u32)0x0000FF00)        /* Internal High Speed clock Calibration */
#define  RCC_CR_HSEON                        ((u32)0x00010000)        /* External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((u32)0x00020000)        /* External High Speed clock ready flag */
#define  RCC_CR_HSEBYP                       ((u32)0x00040000)        /* External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((u32)0x00080000)        /* Clock Security System enable */
#define  RCC_CR_PLLON                        ((u32)0x01000000)        /* PLL enable */
#define  RCC_CR_PLLRDY                       ((u32)0x02000000)        /* PLL clock ready flag */


/*******************  Bit definition for RCC_CFGR register  *******************/
#define  RCC_CFGR_SW                         ((u32)0x00000003)        /* SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((u32)0x00000001)        /* Bit 0 */
#define  RCC_CFGR_SW_1                       ((u32)0x00000002)        /* Bit 1 */

/* SW configuration */
#define  RCC_CFGR_SW_HSI                     ((u32)0x00000000)        /* HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((u32)0x00000001)        /* HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((u32)0x00000002)        /* PLL selected as system clock */

#define  RCC_CFGR_SWS                        ((u32)0x0000000C)        /* SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((u32)0x00000004)        /* Bit 0 */
#define  RCC_CFGR_SWS_1                      ((u32)0x00000008)        /* Bit 1 */

/* SWS configuration */
#define  RCC_CFGR_SWS_HSI                    ((u32)0x00000000)        /* HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((u32)0x00000004)        /* HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((u32)0x00000008)        /* PLL used as system clock */

#define  RCC_CFGR_HPRE                       ((u32)0x000000F0)        /* HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((u32)0x00000010)        /* Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((u32)0x00000020)        /* Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((u32)0x00000040)        /* Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((u32)0x00000080)        /* Bit 3 */

/* HPRE configuration */
#define  RCC_CFGR_HPRE_DIV1                  ((u32)0x00000000)        /* SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((u32)0x00000080)        /* SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((u32)0x00000090)        /* SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((u32)0x000000A0)        /* SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((u32)0x000000B0)        /* SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((u32)0x000000C0)        /* SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((u32)0x000000D0)        /* SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((u32)0x000000E0)        /* SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((u32)0x000000F0)        /* SYSCLK divided by 512 */

#define  RCC_CFGR_PPRE1                      ((u32)0x00000700)        /* PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((u32)0x00000100)        /* Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((u32)0x00000200)        /* Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((u32)0x00000400)        /* Bit 2 */

/* PPRE1 configuration */
#define  RCC_CFGR_PPRE1_DIV1                 ((u32)0x00000000)        /* HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((u32)0x00000400)        /* HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((u32)0x00000500)        /* HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((u32)0x00000600)        /* HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((u32)0x00000700)        /* HCLK divided by 16 */

#define  RCC_CFGR_PPRE2                      ((u32)0x00003800)        /* PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((u32)0x00000800)        /* Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((u32)0x00001000)        /* Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((u32)0x00002000)        /* Bit 2 */

/* PPRE2 configuration */
#define  RCC_CFGR_PPRE2_DIV1                 ((u32)0x00000000)        /* HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((u32)0x00002000)        /* HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((u32)0x00002800)        /* HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((u32)0x00003000)        /* HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((u32)0x00003800)        /* HCLK divided by 16 */

#define  RCC_CFGR_ADCPRE                     ((u32)0x0000C000)        /* ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_CFGR_ADCPRE_0                   ((u32)0x00004000)        /* Bit 0 */
#define  RCC_CFGR_ADCPRE_1                   ((u32)0x00008000)        /* Bit 1 */

/* ADCPPRE configuration */
#define  RCC_CFGR_ADCPRE_DIV2                ((u32)0x00000000)        /* PCLK2 divided by 2 */
#define  RCC_CFGR_ADCPRE_DIV4                ((u32)0x00004000)        /* PCLK2 divided by 4 */
#define  RCC_CFGR_ADCPRE_DIV6                ((u32)0x00008000)        /* PCLK2 divided by 6 */
#define  RCC_CFGR_ADCPRE_DIV8                ((u32)0x0000C000)        /* PCLK2 divided by 8 */

#define  RCC_CFGR_PLLSRC                     ((u32)0x00010000)        /* PLL entry clock source */
#define  RCC_CFGR_PLLXTPRE                   ((u32)0x00020000)        /* HSE divider for PLL entry */

#define  RCC_CFGR_PLLMULL                    ((u32)0x003C0000)        /* PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((u32)0x00040000)        /* Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((u32)0x00080000)        /* Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((u32)0x00100000)        /* Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((u32)0x00200000)        /* Bit 3 */

/* PLLMUL configuration */
#define  RCC_CFGR_PLLMULL2                   ((u32)0x00000000)        /* PLL input clock*2 */
#define  RCC_CFGR_PLLMULL3                   ((u32)0x00040000)        /* PLL input clock*3 */
#define  RCC_CFGR_PLLMULL4                   ((u32)0x00080000)        /* PLL input clock*4 */
#define  RCC_CFGR_PLLMULL5                   ((u32)0x000C0000)        /* PLL input clock*5 */
#define  RCC_CFGR_PLLMULL6                   ((u32)0x00100000)        /* PLL input clock*6 */
#define  RCC_CFGR_PLLMULL7                   ((u32)0x00140000)        /* PLL input clock*7 */
#define  RCC_CFGR_PLLMULL8                   ((u32)0x00180000)        /* PLL input clock*8 */
#define  RCC_CFGR_PLLMULL9                   ((u32)0x001C0000)        /* PLL input clock*9 */
#define  RCC_CFGR_PLLMULL10                  ((u32)0x00200000)        /* PLL input clock10 */
#define  RCC_CFGR_PLLMULL11                  ((u32)0x00240000)        /* PLL input clock*11 */
#define  RCC_CFGR_PLLMULL12                  ((u32)0x00280000)        /* PLL input clock*12 */
#define  RCC_CFGR_PLLMULL13                  ((u32)0x002C0000)        /* PLL input clock*13 */
#define  RCC_CFGR_PLLMULL14                  ((u32)0x00300000)        /* PLL input clock*14 */
#define  RCC_CFGR_PLLMULL15                  ((u32)0x00340000)        /* PLL input clock*15 */
#define  RCC_CFGR_PLLMULL16                  ((u32)0x00380000)        /* PLL input clock*16 */

#define  RCC_CFGR_USBPRE                     ((u32)0x00400000)        /* USB prescaler */

#define  RCC_CFGR_MCO                        ((u32)0x07000000)        /* MCO[2:0] bits (Microcontroller Clock Output) */
#define  RCC_CFGR_MCO_0                      ((u32)0x01000000)        /* Bit 0 */
#define  RCC_CFGR_MCO_1                      ((u32)0x02000000)        /* Bit 1 */
#define  RCC_CFGR_MCO_2                      ((u32)0x04000000)        /* Bit 2 */

/* MCO configuration */
#define  RCC_CFGR_MCO_NOCLOCK                ((u32)0x00000000)        /* No clock */
#define  RCC_CFGR_MCO_SYSCLK                 ((u32)0x04000000)        /* System clock selected */
#define  RCC_CFGR_MCO_HSI                    ((u32)0x05000000)        /* Internal 8 MHz RC oscillator clock selected */
#define  RCC_CFGR_MCO_HSE                    ((u32)0x06000000)        /* External 1-25 MHz oscillator clock selected */
#define  RCC_CFGR_MCO_PLL                    ((u32)0x07000000)        /* PLL clock divided by 2 selected*/


/*******************  Bit definition for RCC_CIR register  ********************/
#define  RCC_CIR_LSIRDYF                     ((u32)0x00000001)        /* LSI Ready Interrupt flag */
#define  RCC_CIR_LSERDYF                     ((u32)0x00000002)        /* LSE Ready Interrupt flag */
#define  RCC_CIR_HSIRDYF                     ((u32)0x00000004)        /* HSI Ready Interrupt flag */
#define  RCC_CIR_HSERDYF                     ((u32)0x00000008)        /* HSE Ready Interrupt flag */
#define  RCC_CIR_PLLRDYF                     ((u32)0x00000010)        /* PLL Ready Interrupt flag */
#define  RCC_CIR_CSSF                        ((u32)0x00000080)        /* Clock Security System Interrupt flag */
#define  RCC_CIR_LSIRDYIE                    ((u32)0x00000100)        /* LSI Ready Interrupt Enable */
#define  RCC_CIR_LSERDYIE                    ((u32)0x00000200)        /* LSE Ready Interrupt Enable */
#define  RCC_CIR_HSIRDYIE                    ((u32)0x00000400)        /* HSI Ready Interrupt Enable */
#define  RCC_CIR_HSERDYIE                    ((u32)0x00000800)        /* HSE Ready Interrupt Enable */
#define  RCC_CIR_PLLRDYIE                    ((u32)0x00001000)        /* PLL Ready Interrupt Enable */
#define  RCC_CIR_LSIRDYC                     ((u32)0x00010000)        /* LSI Ready Interrupt Clear */
#define  RCC_CIR_LSERDYC                     ((u32)0x00020000)        /* LSE Ready Interrupt Clear */
#define  RCC_CIR_HSIRDYC                     ((u32)0x00040000)        /* HSI Ready Interrupt Clear */
#define  RCC_CIR_HSERDYC                     ((u32)0x00080000)        /* HSE Ready Interrupt Clear */
#define  RCC_CIR_PLLRDYC                     ((u32)0x00100000)        /* PLL Ready Interrupt Clear */
#define  RCC_CIR_CSSC                        ((u32)0x00800000)        /* Clock Security System Interrupt Clear */


/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define  RCC_APB2RSTR_AFIORST                ((u16)0x0001)            /* Alternate Function I/O reset */
#define  RCC_APB2RSTR_IOPARST                ((u16)0x0004)            /* I/O port A reset */
#define  RCC_APB2RSTR_IOPBRST                ((u16)0x0008)            /* IO port B reset */
#define  RCC_APB2RSTR_IOPCRST                ((u16)0x0010)            /* IO port C reset */
#define  RCC_APB2RSTR_IOPDRST                ((u16)0x0020)            /* IO port D reset */
#define  RCC_APB2RSTR_IOPERST                ((u16)0x0040)            /* IO port E reset */
#define  RCC_APB2RSTR_IOPFRST                ((u16)0x0080)            /* IO port F reset */
#define  RCC_APB2RSTR_IOPGRST                ((u16)0x0100)            /* IO port G reset */
#define  RCC_APB2RSTR_ADC1RST                ((u16)0x0200)            /* ADC 1 interface reset */
#define  RCC_APB2RSTR_ADC2RST                ((u16)0x0400)            /* ADC 2 interface reset */
#define  RCC_APB2RSTR_TIM1RST                ((u16)0x0800)            /* TIM1 Timer reset */
#define  RCC_APB2RSTR_SPI1RST                ((u16)0x1000)            /* SPI 1 reset */
#define  RCC_APB2RSTR_TIM8RST                ((u16)0x2000)            /* TIM8 Timer reset */
#define  RCC_APB2RSTR_USART1RST              ((u16)0x4000)            /* USART1 reset */
#define  RCC_APB2RSTR_ADC3RST                ((u16)0x8000)            /* ADC3 interface reset */


/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define  RCC_APB1RSTR_TIM2RST                ((u32)0x00000001)        /* Timer 2 reset */
#define  RCC_APB1RSTR_TIM3RST                ((u32)0x00000002)        /* Timer 3 reset */
#define  RCC_APB1RSTR_TIM4RST                ((u32)0x00000004)        /* Timer 4 reset */
#define  RCC_APB1RSTR_TIM5RST                ((u32)0x00000008)        /* Timer 5 reset */
#define  RCC_APB1RSTR_TIM6RST                ((u32)0x00000010)        /* Timer 6 reset */
#define  RCC_APB1RSTR_TIM7RST                ((u32)0x00000020)        /* Timer 7 reset */
#define  RCC_APB1RSTR_WWDGRST                ((u32)0x00000800)        /* Window Watchdog reset */
#define  RCC_APB1RSTR_SPI2RST                ((u32)0x00004000)        /* SPI 2 reset */
#define  RCC_APB1RSTR_SPI3RST                ((u32)0x00008000)        /* SPI 3 reset */
#define  RCC_APB1RSTR_USART2RST              ((u32)0x00020000)        /* USART 2 reset */
#define  RCC_APB1RSTR_USART3RST              ((u32)0x00040000)        /* RUSART 3 reset */
#define  RCC_APB1RSTR_UART4RST               ((u32)0x00080000)        /* USART 4 reset */
#define  RCC_APB1RSTR_UART5RST               ((u32)0x00100000)        /* USART 5 reset */
#define  RCC_APB1RSTR_I2C1RST                ((u32)0x00200000)        /* I2C 1 reset */
#define  RCC_APB1RSTR_I2C2RST                ((u32)0x00400000)        /* I2C 2 reset */
#define  RCC_APB1RSTR_USBRST                 ((u32)0x00800000)        /* USB reset */
#define  RCC_APB1RSTR_CANRST                 ((u32)0x02000000)        /* CAN reset */
#define  RCC_APB1RSTR_BKPRST                 ((u32)0x08000000)        /* Backup interface reset */
#define  RCC_APB1RSTR_PWRRST                 ((u32)0x10000000)        /* Power interface reset */
#define  RCC_APB1RSTR_DACRST                 ((u32)0x20000000)        /* DAC interface reset */


/******************  Bit definition for RCC_AHBENR register  ******************/
#define  RCC_AHBENR_DMA1EN                   ((u16)0x0001)            /* DMA1 clock enable */
#define  RCC_AHBENR_DMA2EN                   ((u16)0x0002)            /* DMA2 clock enable */
#define  RCC_AHBENR_SRAMEN                   ((u16)0x0004)            /* SRAM interface clock enable */
#define  RCC_AHBENR_FLITFEN                  ((u16)0x0010)            /* FLITF clock enable */
#define  RCC_AHBENR_CRCEN                    ((u16)0x0040)            /* CRC clock enable */
#define  RCC_AHBENR_FSMCEN                   ((u16)0x0100)            /* FSMC clock enable */
#define  RCC_AHBENR_SDIOEN                   ((u16)0x0400)            /* SDIO clock enable */


/******************  Bit definition for RCC_APB2ENR register  *****************/
#define  RCC_APB2ENR_AFIOEN                  ((u16)0x0001)            /* Alternate Function I/O clock enable */
#define  RCC_APB2ENR_IOPAEN                  ((u16)0x0004)            /* I/O port A clock enable */
#define  RCC_APB2ENR_IOPBEN                  ((u16)0x0008)            /* I/O port B clock enable */
#define  RCC_APB2ENR_IOPCEN                  ((u16)0x0010)            /* I/O port C clock enable */
#define  RCC_APB2ENR_IOPDEN                  ((u16)0x0020)            /* I/O port D clock enable */
#define  RCC_APB2ENR_IOPEEN                  ((u16)0x0040)            /* I/O port E clock enable */
#define  RCC_APB2ENR_IOPFEN                  ((u16)0x0080)            /* I/O port F clock enable */
#define  RCC_APB2ENR_IOPGEN                  ((u16)0x0100)            /* I/O port G clock enable */
#define  RCC_APB2ENR_ADC1EN                  ((u16)0x0200)            /* ADC 1 interface clock enable */
#define  RCC_APB2ENR_ADC2EN                  ((u16)0x0400)            /* ADC 2 interface clock enable */
#define  RCC_APB2ENR_TIM1EN                  ((u16)0x0800)            /* TIM1 Timer clock enable */
#define  RCC_APB2ENR_SPI1EN                  ((u16)0x1000)            /* SPI 1 clock enable */
#define  RCC_APB2ENR_TIM8EN                  ((u16)0x2000)            /* TIM8 Timer clock enable */
#define  RCC_APB2ENR_USART1EN                ((u16)0x4000)            /* USART1 clock enable */
#define  RCC_APB2ENR_ADC3EN                  ((u16)0x8000)            /* DMA1 clock enable */


/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define  RCC_APB1ENR_TIM2EN                  ((u32)0x00000001)        /* Timer 2 clock enabled*/
#define  RCC_APB1ENR_TIM3EN                  ((u32)0x00000002)        /* Timer 3 clock enable */
#define  RCC_APB1ENR_TIM4EN                  ((u32)0x00000004)        /* Timer 4 clock enable */
#define  RCC_APB1ENR_TIM5EN                  ((u32)0x00000008)        /* Timer 5 clock enable */
#define  RCC_APB1ENR_TIM6EN                  ((u32)0x00000010)        /* Timer 6 clock enable */
#define  RCC_APB1ENR_TIM7EN                  ((u32)0x00000020)        /* Timer 7 clock enable */
#define  RCC_APB1ENR_WWDGEN                  ((u32)0x00000800)        /* Window Watchdog clock enable */
#define  RCC_APB1ENR_SPI2EN                  ((u32)0x00004000)        /* SPI 2 clock enable */
#define  RCC_APB1ENR_SPI3EN                  ((u32)0x00008000)        /* SPI 3 clock enable */
#define  RCC_APB1ENR_USART2EN                ((u32)0x00020000)        /* USART 2 clock enable */
#define  RCC_APB1ENR_USART3EN                ((u32)0x00040000)        /* USART 3 clock enable */
#define  RCC_APB1ENR_UART4EN                 ((u32)0x00080000)        /* USART 4 clock enable */
#define  RCC_APB1ENR_UART5EN                 ((u32)0x00100000)        /* USART 5 clock enable */
#define  RCC_APB1ENR_I2C1EN                  ((u32)0x00200000)        /* I2C 1 clock enable */
#define  RCC_APB1ENR_I2C2EN                  ((u32)0x00400000)        /* I2C 2 clock enable */
#define  RCC_APB1ENR_USBEN                   ((u32)0x00800000)        /* USB clock enable */
#define  RCC_APB1ENR_CANEN                   ((u32)0x02000000)        /* CAN clock enable */
#define  RCC_APB1ENR_BKPEN                   ((u32)0x08000000)        /* Backup interface clock enable */
#define  RCC_APB1ENR_PWREN                   ((u32)0x10000000)        /* Power interface clock enable */
#define  RCC_APB1ENR_DACEN                   ((u32)0x20000000)        /* DAC interface clock enable */


/*******************  Bit definition for RCC_BDCR register  *******************/
#define  RCC_BDCR_LSEON                      ((u32)0x00000001)        /* External Low Speed oscillator enable */
#define  RCC_BDCR_LSERDY                     ((u32)0x00000002)        /* External Low Speed oscillator Ready */
#define  RCC_BDCR_LSEBYP                     ((u32)0x00000004)        /* External Low Speed oscillator Bypass */

#define  RCC_BDCR_RTCSEL                     ((u32)0x00000300)        /* RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_BDCR_RTCSEL_0                   ((u32)0x00000100)        /* Bit 0 */
#define  RCC_BDCR_RTCSEL_1                   ((u32)0x00000200)        /* Bit 1 */
/* RTC congiguration */
#define  RCC_BDCR_RTCSEL_NOCLOCK             ((u32)0x00000000)        /* No clock */
#define  RCC_BDCR_RTCSEL_LSE                 ((u32)0x00000100)        /* LSE oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSI                 ((u32)0x00000200)        /* LSI oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_HSE                 ((u32)0x00000300)        /* HSE oscillator clock divided by 128 used as RTC clock */

#define  RCC_BDCR_RTCEN                      ((u32)0x00008000)        /* RTC clock enable */
#define  RCC_BDCR_BDRST                      ((u32)0x00010000)        /* Backup domain software reset  */


/*******************  Bit definition for RCC_CSR register  ********************/  
#define  RCC_CSR_LSION                       ((u32)0x00000001)        /* Internal Low Speed oscillator enable */
#define  RCC_CSR_LSIRDY                      ((u32)0x00000002)        /* Internal Low Speed oscillator Ready */
#define  RCC_CSR_RMVF                        ((u32)0x01000000)        /* Remove reset flag */
#define  RCC_CSR_PINRSTF                     ((u32)0x04000000)        /* PIN reset flag */
#define  RCC_CSR_PORRSTF                     ((u32)0x08000000)        /* POR/PDR reset flag */
#define  RCC_CSR_SFTRSTF                     ((u32)0x10000000)        /* Software Reset flag */
#define  RCC_CSR_IWDGRSTF                    ((u32)0x20000000)        /* Independent Watchdog reset flag */
#define  RCC_CSR_WWDGRSTF                    ((u32)0x40000000)        /* Window watchdog reset flag */
#define  RCC_CSR_LPWRRSTF                    ((u32)0x80000000)        /* Low-Power reset flag */



/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function IO                   */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CRL register  *******************/
#define  GPIO_CRL_MODE                       ((u32)0x33333333)        /* Port x mode bits */

#define  GPIO_CRL_MODE0                      ((u32)0x00000003)        /* MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CRL_MODE0_0                    ((u32)0x00000001)        /* Bit 0 */
#define  GPIO_CRL_MODE0_1                    ((u32)0x00000002)        /* Bit 1 */

#define  GPIO_CRL_MODE1                      ((u32)0x00000030)        /* MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CRL_MODE1_0                    ((u32)0x00000010)        /* Bit 0 */
#define  GPIO_CRL_MODE1_1                    ((u32)0x00000020)        /* Bit 1 */

#define  GPIO_CRL_MODE2                      ((u32)0x00000300)        /* MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CRL_MODE2_0                    ((u32)0x00000100)        /* Bit 0 */
#define  GPIO_CRL_MODE2_1                    ((u32)0x00000200)        /* Bit 1 */

#define  GPIO_CRL_MODE3                      ((u32)0x00003000)        /* MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CRL_MODE3_0                    ((u32)0x00001000)        /* Bit 0 */
#define  GPIO_CRL_MODE3_1                    ((u32)0x00002000)        /* Bit 1 */

#define  GPIO_CRL_MODE4                      ((u32)0x00030000)        /* MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CRL_MODE4_0                    ((u32)0x00010000)        /* Bit 0 */
#define  GPIO_CRL_MODE4_1                    ((u32)0x00020000)        /* Bit 1 */

#define  GPIO_CRL_MODE5                      ((u32)0x00300000)        /* MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CRL_MODE5_0                    ((u32)0x00100000)        /* Bit 0 */
#define  GPIO_CRL_MODE5_1                    ((u32)0x00200000)        /* Bit 1 */

#define  GPIO_CRL_MODE6                      ((u32)0x03000000)        /* MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CRL_MODE6_0                    ((u32)0x01000000)        /* Bit 0 */
#define  GPIO_CRL_MODE6_1                    ((u32)0x02000000)        /* Bit 1 */

#define  GPIO_CRL_MODE7                      ((u32)0x30000000)        /* MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CRL_MODE7_0                    ((u32)0x10000000)        /* Bit 0 */
#define  GPIO_CRL_MODE7_1                    ((u32)0x20000000)        /* Bit 1 */


#define  GPIO_CRL_CNF                        ((u32)0xCCCCCCCC)        /* Port x configuration bits */

#define  GPIO_CRL_CNF0                       ((u32)0x0000000C)        /* CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CRL_CNF0_0                     ((u32)0x00000004)        /* Bit 0 */
#define  GPIO_CRL_CNF0_1                     ((u32)0x00000008)        /* Bit 1 */

#define  GPIO_CRL_CNF1                       ((u32)0x000000C0)        /* CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CRL_CNF1_0                     ((u32)0x00000040)        /* Bit 0 */
#define  GPIO_CRL_CNF1_1                     ((u32)0x00000080)        /* Bit 1 */

#define  GPIO_CRL_CNF2                       ((u32)0x00000C00)        /* CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CRL_CNF2_0                     ((u32)0x00000400)        /* Bit 0 */
#define  GPIO_CRL_CNF2_1                     ((u32)0x00000800)        /* Bit 1 */

#define  GPIO_CRL_CNF3                       ((u32)0x0000C000)        /* CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CRL_CNF3_0                     ((u32)0x00004000)        /* Bit 0 */
#define  GPIO_CRL_CNF3_1                     ((u32)0x00008000)        /* Bit 1 */

#define  GPIO_CRL_CNF4                       ((u32)0x000C0000)        /* CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CRL_CNF4_0                     ((u32)0x00040000)        /* Bit 0 */
#define  GPIO_CRL_CNF4_1                     ((u32)0x00080000)        /* Bit 1 */

#define  GPIO_CRL_CNF5                       ((u32)0x00C00000)        /* CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CRL_CNF5_0                     ((u32)0x00400000)        /* Bit 0 */
#define  GPIO_CRL_CNF5_1                     ((u32)0x00800000)        /* Bit 1 */

#define  GPIO_CRL_CNF6                       ((u32)0x0C000000)        /* CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CRL_CNF6_0                     ((u32)0x04000000)        /* Bit 0 */
#define  GPIO_CRL_CNF6_1                     ((u32)0x08000000)        /* Bit 1 */

#define  GPIO_CRL_CNF7                       ((u32)0xC0000000)        /* CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CRL_CNF7_0                     ((u32)0x40000000)        /* Bit 0 */
#define  GPIO_CRL_CNF7_1                     ((u32)0x80000000)        /* Bit 1 */


/*******************  Bit definition for GPIO_CRH register  *******************/
#define  GPIO_CRH_MODE                       ((u32)0x33333333)        /* Port x mode bits */

#define  GPIO_CRH_MODE8                      ((u32)0x00000003)        /* MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CRH_MODE8_0                    ((u32)0x00000001)        /* Bit 0 */
#define  GPIO_CRH_MODE8_1                    ((u32)0x00000002)        /* Bit 1 */

#define  GPIO_CRH_MODE9                      ((u32)0x00000030)        /* MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CRH_MODE9_0                    ((u32)0x00000010)        /* Bit 0 */
#define  GPIO_CRH_MODE9_1                    ((u32)0x00000020)        /* Bit 1 */

#define  GPIO_CRH_MODE10                     ((u32)0x00000300)        /* MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CRH_MODE10_0                   ((u32)0x00000100)        /* Bit 0 */
#define  GPIO_CRH_MODE10_1                   ((u32)0x00000200)        /* Bit 1 */

#define  GPIO_CRH_MODE11                     ((u32)0x00003000)        /* MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CRH_MODE11_0                   ((u32)0x00001000)        /* Bit 0 */
#define  GPIO_CRH_MODE11_1                   ((u32)0x00002000)        /* Bit 1 */

#define  GPIO_CRH_MODE12                     ((u32)0x00030000)        /* MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CRH_MODE12_0                   ((u32)0x00010000)        /* Bit 0 */
#define  GPIO_CRH_MODE12_1                   ((u32)0x00020000)        /* Bit 1 */

#define  GPIO_CRH_MODE13                     ((u32)0x00300000)        /* MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CRH_MODE13_0                   ((u32)0x00100000)        /* Bit 0 */
#define  GPIO_CRH_MODE13_1                   ((u32)0x00200000)        /* Bit 1 */

#define  GPIO_CRH_MODE14                     ((u32)0x03000000)        /* MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CRH_MODE14_0                   ((u32)0x01000000)        /* Bit 0 */
#define  GPIO_CRH_MODE14_1                   ((u32)0x02000000)        /* Bit 1 */

#define  GPIO_CRH_MODE15                     ((u32)0x30000000)        /* MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CRH_MODE15_0                   ((u32)0x10000000)        /* Bit 0 */
#define  GPIO_CRH_MODE15_1                   ((u32)0x20000000)        /* Bit 1 */


#define  GPIO_CRH_CNF                        ((u32)0xCCCCCCCC)        /* Port x configuration bits */

#define  GPIO_CRH_CNF8                       ((u32)0x0000000C)        /* CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CRH_CNF8_0                     ((u32)0x00000004)        /* Bit 0 */
#define  GPIO_CRH_CNF8_1                     ((u32)0x00000008)        /* Bit 1 */

#define  GPIO_CRH_CNF9                       ((u32)0x000000C0)        /* CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CRH_CNF9_0                     ((u32)0x00000040)        /* Bit 0 */
#define  GPIO_CRH_CNF9_1                     ((u32)0x00000080)        /* Bit 1 */

#define  GPIO_CRH_CNF10                      ((u32)0x00000C00)        /* CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CRH_CNF10_0                    ((u32)0x00000400)        /* Bit 0 */
#define  GPIO_CRH_CNF10_1                    ((u32)0x00000800)        /* Bit 1 */

#define  GPIO_CRH_CNF11                      ((u32)0x0000C000)        /* CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CRH_CNF11_0                    ((u32)0x00004000)        /* Bit 0 */
#define  GPIO_CRH_CNF11_1                    ((u32)0x00008000)        /* Bit 1 */

#define  GPIO_CRH_CNF12                      ((u32)0x000C0000)        /* CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CRH_CNF12_0                    ((u32)0x00040000)        /* Bit 0 */
#define  GPIO_CRH_CNF12_1                    ((u32)0x00080000)        /* Bit 1 */

#define  GPIO_CRH_CNF13                      ((u32)0x00C00000)        /* CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CRH_CNF13_0                    ((u32)0x00400000)        /* Bit 0 */
#define  GPIO_CRH_CNF13_1                    ((u32)0x00800000)        /* Bit 1 */

#define  GPIO_CRH_CNF14                      ((u32)0x0C000000)        /* CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CRH_CNF14_0                    ((u32)0x04000000)        /* Bit 0 */
#define  GPIO_CRH_CNF14_1                    ((u32)0x08000000)        /* Bit 1 */

#define  GPIO_CRH_CNF15                      ((u32)0xC0000000)        /* CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CRH_CNF15_0                    ((u32)0x40000000)        /* Bit 0 */
#define  GPIO_CRH_CNF15_1                    ((u32)0x80000000)        /* Bit 1 */


/*******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR0                        ((u16)0x0001)            /* Port input data, bit 0 */
#define GPIO_IDR_IDR1                        ((u16)0x0002)            /* Port input data, bit 1 */
#define GPIO_IDR_IDR2                        ((u16)0x0004)            /* Port input data, bit 2 */
#define GPIO_IDR_IDR3                        ((u16)0x0008)            /* Port input data, bit 3 */
#define GPIO_IDR_IDR4                        ((u16)0x0010)            /* Port input data, bit 4 */
#define GPIO_IDR_IDR5                        ((u16)0x0020)            /* Port input data, bit 5 */
#define GPIO_IDR_IDR6                        ((u16)0x0040)            /* Port input data, bit 6 */
#define GPIO_IDR_IDR7                        ((u16)0x0080)            /* Port input data, bit 7 */
#define GPIO_IDR_IDR8                        ((u16)0x0100)            /* Port input data, bit 8 */
#define GPIO_IDR_IDR9                        ((u16)0x0200)            /* Port input data, bit 9 */
#define GPIO_IDR_IDR10                       ((u16)0x0400)            /* Port input data, bit 10 */
#define GPIO_IDR_IDR11                       ((u16)0x0800)            /* Port input data, bit 11 */
#define GPIO_IDR_IDR12                       ((u16)0x1000)            /* Port input data, bit 12 */
#define GPIO_IDR_IDR13                       ((u16)0x2000)            /* Port input data, bit 13 */
#define GPIO_IDR_IDR14                       ((u16)0x4000)            /* Port input data, bit 14 */
#define GPIO_IDR_IDR15                       ((u16)0x8000)            /* Port input data, bit 15 */


/*******************  Bit definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR0                        ((u16)0x0001)            /* Port output data, bit 0 */
#define GPIO_ODR_ODR1                        ((u16)0x0002)            /* Port output data, bit 1 */
#define GPIO_ODR_ODR2                        ((u16)0x0004)            /* Port output data, bit 2 */
#define GPIO_ODR_ODR3                        ((u16)0x0008)            /* Port output data, bit 3 */
#define GPIO_ODR_ODR4                        ((u16)0x0010)            /* Port output data, bit 4 */
#define GPIO_ODR_ODR5                        ((u16)0x0020)            /* Port output data, bit 5 */
#define GPIO_ODR_ODR6                        ((u16)0x0040)            /* Port output data, bit 6 */
#define GPIO_ODR_ODR7                        ((u16)0x0080)            /* Port output data, bit 7 */
#define GPIO_ODR_ODR8                        ((u16)0x0100)            /* Port output data, bit 8 */
#define GPIO_ODR_ODR9                        ((u16)0x0200)            /* Port output data, bit 9 */
#define GPIO_ODR_ODR10                       ((u16)0x0400)            /* Port output data, bit 10 */
#define GPIO_ODR_ODR11                       ((u16)0x0800)            /* Port output data, bit 11 */
#define GPIO_ODR_ODR12                       ((u16)0x1000)            /* Port output data, bit 12 */
#define GPIO_ODR_ODR13                       ((u16)0x2000)            /* Port output data, bit 13 */
#define GPIO_ODR_ODR14                       ((u16)0x4000)            /* Port output data, bit 14 */
#define GPIO_ODR_ODR15                       ((u16)0x8000)            /* Port output data, bit 15 */


/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0                        ((u32)0x00000001)        /* Port x Set bit 0 */
#define GPIO_BSRR_BS1                        ((u32)0x00000002)        /* Port x Set bit 1 */
#define GPIO_BSRR_BS2                        ((u32)0x00000004)        /* Port x Set bit 2 */
#define GPIO_BSRR_BS3                        ((u32)0x00000008)        /* Port x Set bit 3 */
#define GPIO_BSRR_BS4                        ((u32)0x00000010)        /* Port x Set bit 4 */
#define GPIO_BSRR_BS5                        ((u32)0x00000020)        /* Port x Set bit 5 */
#define GPIO_BSRR_BS6                        ((u32)0x00000040)        /* Port x Set bit 6 */
#define GPIO_BSRR_BS7                        ((u32)0x00000080)        /* Port x Set bit 7 */
#define GPIO_BSRR_BS8                        ((u32)0x00000100)        /* Port x Set bit 8 */
#define GPIO_BSRR_BS9                        ((u32)0x00000200)        /* Port x Set bit 9 */
#define GPIO_BSRR_BS10                       ((u32)0x00000400)        /* Port x Set bit 10 */
#define GPIO_BSRR_BS11                       ((u32)0x00000800)        /* Port x Set bit 11 */
#define GPIO_BSRR_BS12                       ((u32)0x00001000)        /* Port x Set bit 12 */
#define GPIO_BSRR_BS13                       ((u32)0x00002000)        /* Port x Set bit 13 */
#define GPIO_BSRR_BS14                       ((u32)0x00004000)        /* Port x Set bit 14 */
#define GPIO_BSRR_BS15                       ((u32)0x00008000)        /* Port x Set bit 15 */

#define GPIO_BSRR_BR0                        ((u32)0x00010000)        /* Port x Reset bit 0 */
#define GPIO_BSRR_BR1                        ((u32)0x00020000)        /* Port x Reset bit 1 */
#define GPIO_BSRR_BR2                        ((u32)0x00040000)        /* Port x Reset bit 2 */
#define GPIO_BSRR_BR3                        ((u32)0x00080000)        /* Port x Reset bit 3 */
#define GPIO_BSRR_BR4                        ((u32)0x00100000)        /* Port x Reset bit 4 */
#define GPIO_BSRR_BR5                        ((u32)0x00200000)        /* Port x Reset bit 5 */
#define GPIO_BSRR_BR6                        ((u32)0x00400000)        /* Port x Reset bit 6 */
#define GPIO_BSRR_BR7                        ((u32)0x00800000)        /* Port x Reset bit 7 */
#define GPIO_BSRR_BR8                        ((u32)0x01000000)        /* Port x Reset bit 8 */
#define GPIO_BSRR_BR9                        ((u32)0x02000000)        /* Port x Reset bit 9 */
#define GPIO_BSRR_BR10                       ((u32)0x04000000)        /* Port x Reset bit 10 */
#define GPIO_BSRR_BR11                       ((u32)0x08000000)        /* Port x Reset bit 11 */
#define GPIO_BSRR_BR12                       ((u32)0x10000000)        /* Port x Reset bit 12 */
#define GPIO_BSRR_BR13                       ((u32)0x20000000)        /* Port x Reset bit 13 */
#define GPIO_BSRR_BR14                       ((u32)0x40000000)        /* Port x Reset bit 14 */
#define GPIO_BSRR_BR15                       ((u32)0x80000000)        /* Port x Reset bit 15 */


/*******************  Bit definition for GPIO_BRR register  *******************/
#define GPIO_BRR_BR0                         ((u16)0x0001)            /* Port x Reset bit 0 */
#define GPIO_BRR_BR1                         ((u16)0x0002)            /* Port x Reset bit 1 */
#define GPIO_BRR_BR2                         ((u16)0x0004)            /* Port x Reset bit 2 */
#define GPIO_BRR_BR3                         ((u16)0x0008)            /* Port x Reset bit 3 */
#define GPIO_BRR_BR4                         ((u16)0x0010)            /* Port x Reset bit 4 */
#define GPIO_BRR_BR5                         ((u16)0x0020)            /* Port x Reset bit 5 */
#define GPIO_BRR_BR6                         ((u16)0x0040)            /* Port x Reset bit 6 */
#define GPIO_BRR_BR7                         ((u16)0x0080)            /* Port x Reset bit 7 */
#define GPIO_BRR_BR8                         ((u16)0x0100)            /* Port x Reset bit 8 */
#define GPIO_BRR_BR9                         ((u16)0x0200)            /* Port x Reset bit 9 */
#define GPIO_BRR_BR10                        ((u16)0x0400)            /* Port x Reset bit 10 */
#define GPIO_BRR_BR11                        ((u16)0x0800)            /* Port x Reset bit 11 */
#define GPIO_BRR_BR12                        ((u16)0x1000)            /* Port x Reset bit 12 */
#define GPIO_BRR_BR13                        ((u16)0x2000)            /* Port x Reset bit 13 */
#define GPIO_BRR_BR14                        ((u16)0x4000)            /* Port x Reset bit 14 */
#define GPIO_BRR_BR15                        ((u16)0x8000)            /* Port x Reset bit 15 */


/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCKR_LCK0                       ((u32)0x00000001)        /* Port x Lock bit 0 */
#define GPIO_LCKR_LCK1                       ((u32)0x00000002)        /* Port x Lock bit 1 */
#define GPIO_LCKR_LCK2                       ((u32)0x00000004)        /* Port x Lock bit 2 */
#define GPIO_LCKR_LCK3                       ((u32)0x00000008)        /* Port x Lock bit 3 */
#define GPIO_LCKR_LCK4                       ((u32)0x00000010)        /* Port x Lock bit 4 */
#define GPIO_LCKR_LCK5                       ((u32)0x00000020)        /* Port x Lock bit 5 */
#define GPIO_LCKR_LCK6                       ((u32)0x00000040)        /* Port x Lock bit 6 */
#define GPIO_LCKR_LCK7                       ((u32)0x00000080)        /* Port x Lock bit 7 */
#define GPIO_LCKR_LCK8                       ((u32)0x00000100)        /* Port x Lock bit 8 */
#define GPIO_LCKR_LCK9                       ((u32)0x00000200)        /* Port x Lock bit 9 */
#define GPIO_LCKR_LCK10                      ((u32)0x00000400)        /* Port x Lock bit 10 */
#define GPIO_LCKR_LCK11                      ((u32)0x00000800)        /* Port x Lock bit 11 */
#define GPIO_LCKR_LCK12                      ((u32)0x00001000)        /* Port x Lock bit 12 */
#define GPIO_LCKR_LCK13                      ((u32)0x00002000)        /* Port x Lock bit 13 */
#define GPIO_LCKR_LCK14                      ((u32)0x00004000)        /* Port x Lock bit 14 */
#define GPIO_LCKR_LCK15                      ((u32)0x00008000)        /* Port x Lock bit 15 */
#define GPIO_LCKR_LCKK                       ((u32)0x00010000)        /* Lock key */


/*----------------------------------------------------------------------------*/


/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN                        ((u8)0x0F)               /* PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_0                      ((u8)0x01)               /* Bit 0 */
#define AFIO_EVCR_PIN_1                      ((u8)0x02)               /* Bit 1 */
#define AFIO_EVCR_PIN_2                      ((u8)0x04)               /* Bit 2 */
#define AFIO_EVCR_PIN_3                      ((u8)0x08)               /* Bit 3 */

/* PIN configuration */
#define AFIO_EVCR_PIN_PX0                    ((u8)0x00)               /* Pin 0 selected */
#define AFIO_EVCR_PIN_PX1                    ((u8)0x01)               /* Pin 1 selected */
#define AFIO_EVCR_PIN_PX2                    ((u8)0x02)               /* Pin 2 selected */
#define AFIO_EVCR_PIN_PX3                    ((u8)0x03)               /* Pin 3 selected */
#define AFIO_EVCR_PIN_PX4                    ((u8)0x04)               /* Pin 4 selected */
#define AFIO_EVCR_PIN_PX5                    ((u8)0x05)               /* Pin 5 selected */
#define AFIO_EVCR_PIN_PX6                    ((u8)0x06)               /* Pin 6 selected */
#define AFIO_EVCR_PIN_PX7                    ((u8)0x07)               /* Pin 7 selected */
#define AFIO_EVCR_PIN_PX8                    ((u8)0x08)               /* Pin 8 selected */
#define AFIO_EVCR_PIN_PX9                    ((u8)0x09)               /* Pin 9 selected */
#define AFIO_EVCR_PIN_PX10                   ((u8)0x0A)               /* Pin 10 selected */
#define AFIO_EVCR_PIN_PX11                   ((u8)0x0B)               /* Pin 11 selected */
#define AFIO_EVCR_PIN_PX12                   ((u8)0x0C)               /* Pin 12 selected */
#define AFIO_EVCR_PIN_PX13                   ((u8)0x0D)               /* Pin 13 selected */
#define AFIO_EVCR_PIN_PX14                   ((u8)0x0E)               /* Pin 14 selected */
#define AFIO_EVCR_PIN_PX15                   ((u8)0x0F)               /* Pin 15 selected */

#define AFIO_EVCR_PORT                       ((u8)0x70)               /* PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_0                     ((u8)0x10)               /* Bit 0 */
#define AFIO_EVCR_PORT_1                     ((u8)0x20)               /* Bit 1 */
#define AFIO_EVCR_PORT_2                     ((u8)0x40)               /* Bit 2 */

/* PORT configuration */
#define AFIO_EVCR_PORT_PA                    ((u8)0x00)               /* Port A selected */
#define AFIO_EVCR_PORT_PB                    ((u8)0x10)               /* Port B selected */
#define AFIO_EVCR_PORT_PC                    ((u8)0x20)               /* Port C selected */
#define AFIO_EVCR_PORT_PD                    ((u8)0x30)               /* Port D selected */
#define AFIO_EVCR_PORT_PE                    ((u8)0x40)               /* Port E selected */

#define AFIO_EVCR_EVOE                       ((u8)0x80)               /* Event Output Enable */


/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1 _REMAP                ((u32)0x00000001)        /* SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP                 ((u32)0x00000002)        /* I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP               ((u32)0x00000004)        /* USART1 remapping */
#define AFIO_MAPR_USART2_REMAP               ((u32)0x00000008)        /* USART2 remapping */

#define AFIO_MAPR_USART3_REMAP               ((u32)0x00000030)        /* USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_0             ((u32)0x00000010)        /* Bit 0 */
#define AFIO_MAPR_USART3_REMAP_1             ((u32)0x00000020)        /* Bit 1 */

/* USART3_REMAP configuration */
#define AFIO_MAPR_USART3_REMAP_NOREMAP       ((u32)0x00000000)        /* No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  ((u32)0x00000010)        /* Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     ((u32)0x00000030)        /* Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP                 ((u32)0x000000C0)        /* TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_0               ((u32)0x00000040)        /* Bit 0 */
#define AFIO_MAPR_TIM1_REMAP_1               ((u32)0x00000080)        /* Bit 1 */

/* TIM1_REMAP configuration */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         ((u32)0x00000000)        /* No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    ((u32)0x00000040)        /* Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       ((u32)0x000000C0)        /* Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP                 ((u32)0x00000300)        /* TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_0               ((u32)0x00000100)        /* Bit 0 */
#define AFIO_MAPR_TIM2_REMAP_1               ((u32)0x00000200)        /* Bit 1 */

/* TIM2_REMAP configuration */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         ((u32)0x00000000)        /* No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   ((u32)0x00000100)        /* Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   ((u32)0x00000200)        /* Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       ((u32)0x00000300)        /* Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP                 ((u32)0x00000C00)        /* TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_0               ((u32)0x00000400)        /* Bit 0 */
#define AFIO_MAPR_TIM3_REMAP_1               ((u32)0x00000800)        /* Bit 1 */

/* TIM3_REMAP configuration */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         ((u32)0x00000000)        /* No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    ((u32)0x00000800)        /* Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       ((u32)0x00000C00)        /* Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP                 ((u32)0x00001000)        /* Port D0/Port D1 mapping on OSC_IN/OSC_OUT */

#define AFIO_MAPR_CAN_REMAP                  ((u32)0x00006000)        /* CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_0                ((u32)0x00002000)        /* Bit 0 */
#define AFIO_MAPR_CAN_REMAP_1                ((u32)0x00004000)        /* Bit 1 */

/* CAN_REMAP configuration */
#define AFIO_MAPR_CAN_REMAP_REMAP1           ((u32)0x00000000)        /* CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2           ((u32)0x00004000)        /* CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3           ((u32)0x00006000)        /* CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP                 ((u32)0x00008000)        /* Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP             ((u32)0x00010000)        /* TIM5 Channel4 Internal Remap */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP         ((u32)0x00020000)        /* ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP         ((u32)0x00040000)        /* ADC 1 External Trigger Regular Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP         ((u32)0x00080000)        /* ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP         ((u32)0x00100000)        /* ADC 2 External Trigger Regular Conversion remapping */

#define AFIO_MAPR_SWJ_CFG                    ((u32)0x07000000)        /* SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_0                  ((u32)0x01000000)        /* Bit 0 */
#define AFIO_MAPR_SWJ_CFG_1                  ((u32)0x02000000)        /* Bit 1 */
#define AFIO_MAPR_SWJ_CFG_2                  ((u32)0x04000000)        /* Bit 2 */

/* SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG_RESET              ((u32)0x00000000)        /* Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           ((u32)0x01000000)        /* Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        ((u32)0x02000000)        /* JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE            ((u32)0x04000000)        /* JTAG-DP Disabled and SW-DP Disabled */


/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0                   ((u16)0x000F)            /* EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1                   ((u16)0x00F0)            /* EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2                   ((u16)0x0F00)            /* EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3                   ((u16)0xF000)            /* EXTI 3 configuration */

/* EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                ((u16)0x0000)            /* PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                ((u16)0x0001)            /* PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                ((u16)0x0002)            /* PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD                ((u16)0x0003)            /* PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE                ((u16)0x0004)            /* PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF                ((u16)0x0005)            /* PF[0] pin */
#define AFIO_EXTICR1_EXTI0_PG                ((u16)0x0006)            /* PG[0] pin */

/* EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                ((u16)0x0000)            /* PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                ((u16)0x0010)            /* PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                ((u16)0x0020)            /* PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD                ((u16)0x0030)            /* PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE                ((u16)0x0040)            /* PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF                ((u16)0x0050)            /* PF[1] pin */
#define AFIO_EXTICR1_EXTI1_PG                ((u16)0x0060)            /* PG[1] pin */

/* EXTI2 configuration */  
#define AFIO_EXTICR1_EXTI2_PA                ((u16)0x0000)            /* PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                ((u16)0x0100)            /* PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                ((u16)0x0200)            /* PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD                ((u16)0x0300)            /* PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE                ((u16)0x0400)            /* PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF                ((u16)0x0500)            /* PF[2] pin */
#define AFIO_EXTICR1_EXTI2_PG                ((u16)0x0600)            /* PG[2] pin */

/* EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                ((u16)0x0000)            /* PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                ((u16)0x1000)            /* PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                ((u16)0x2000)            /* PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD                ((u16)0x3000)            /* PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE                ((u16)0x4000)            /* PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF                ((u16)0x5000)            /* PF[3] pin */
#define AFIO_EXTICR1_EXTI3_PG                ((u16)0x6000)            /* PG[3] pin */


/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI4                   ((u16)0x000F)            /* EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5                   ((u16)0x00F0)            /* EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6                   ((u16)0x0F00)            /* EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7                   ((u16)0xF000)            /* EXTI 7 configuration */

/* EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                ((u16)0x0000)            /* PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB                ((u16)0x0001)            /* PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC                ((u16)0x0002)            /* PC[4] pin */
#define AFIO_EXTICR2_EXTI4_PD                ((u16)0x0003)            /* PD[4] pin */
#define AFIO_EXTICR2_EXTI4_PE                ((u16)0x0004)            /* PE[4] pin */
#define AFIO_EXTICR2_EXTI4_PF                ((u16)0x0005)            /* PF[4] pin */
#define AFIO_EXTICR2_EXTI4_PG                ((u16)0x0006)            /* PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                ((u16)0x0000)            /* PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB                ((u16)0x0010)            /* PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC                ((u16)0x0020)            /* PC[5] pin */
#define AFIO_EXTICR2_EXTI5_PD                ((u16)0x0030)            /* PD[5] pin */
#define AFIO_EXTICR2_EXTI5_PE                ((u16)0x0040)            /* PE[5] pin */
#define AFIO_EXTICR2_EXTI5_PF                ((u16)0x0050)            /* PF[5] pin */
#define AFIO_EXTICR2_EXTI5_PG                ((u16)0x0060)            /* PG[5] pin */

/* EXTI6 configuration */  
#define AFIO_EXTICR2_EXTI6_PA                ((u16)0x0000)            /* PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB                ((u16)0x0100)            /* PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC                ((u16)0x0200)            /* PC[6] pin */
#define AFIO_EXTICR2_EXTI6_PD                ((u16)0x0300)            /* PD[6] pin */
#define AFIO_EXTICR2_EXTI6_PE                ((u16)0x0400)            /* PE[6] pin */
#define AFIO_EXTICR2_EXTI6_PF                ((u16)0x0500)            /* PF[6] pin */
#define AFIO_EXTICR2_EXTI6_PG                ((u16)0x0600)            /* PG[6] pin */

/* EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                ((u16)0x0000)            /* PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB                ((u16)0x1000)            /* PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC                ((u16)0x2000)            /* PC[7] pin */
#define AFIO_EXTICR2_EXTI7_PD                ((u16)0x3000)            /* PD[7] pin */
#define AFIO_EXTICR2_EXTI7_PE                ((u16)0x4000)            /* PE[7] pin */
#define AFIO_EXTICR2_EXTI7_PF                ((u16)0x5000)            /* PF[7] pin */
#define AFIO_EXTICR2_EXTI7_PG                ((u16)0x6000)            /* PG[7] pin */


/*****************  Bit definition for AFIO_EXTICR3 register  *****************/
#define AFIO_EXTICR3_EXTI8                   ((u16)0x000F)            /* EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9                   ((u16)0x00F0)            /* EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10                  ((u16)0x0F00)            /* EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11                  ((u16)0xF000)            /* EXTI 11 configuration */

/* EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                ((u16)0x0000)            /* PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB                ((u16)0x0001)            /* PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC                ((u16)0x0002)            /* PC[8] pin */
#define AFIO_EXTICR3_EXTI8_PD                ((u16)0x0003)            /* PD[8] pin */
#define AFIO_EXTICR3_EXTI8_PE                ((u16)0x0004)            /* PE[8] pin */
#define AFIO_EXTICR3_EXTI8_PF                ((u16)0x0005)            /* PF[8] pin */
#define AFIO_EXTICR3_EXTI8_PG                ((u16)0x0006)            /* PG[8] pin */

/* EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                ((u16)0x0000)            /* PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB                ((u16)0x0010)            /* PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC                ((u16)0x0020)            /* PC[9] pin */
#define AFIO_EXTICR3_EXTI9_PD                ((u16)0x0030)            /* PD[9] pin */
#define AFIO_EXTICR3_EXTI9_PE                ((u16)0x0040)            /* PE[9] pin */
#define AFIO_EXTICR3_EXTI9_PF                ((u16)0x0050)            /* PF[9] pin */
#define AFIO_EXTICR3_EXTI9_PG                ((u16)0x0060)            /* PG[9] pin */

/* EXTI10 configuration */  
#define AFIO_EXTICR3_EXTI10_PA               ((u16)0x0000)            /* PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB               ((u16)0x0100)            /* PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC               ((u16)0x0200)            /* PC[10] pin */
#define AFIO_EXTICR3_EXTI10_PD               ((u16)0x0300)            /* PD[10] pin */
#define AFIO_EXTICR3_EXTI10_PE               ((u16)0x0400)            /* PE[10] pin */
#define AFIO_EXTICR3_EXTI10_PF               ((u16)0x0500)            /* PF[10] pin */
#define AFIO_EXTICR3_EXTI10_PG               ((u16)0x0600)            /* PG[10] pin */

/* EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               ((u16)0x0000)            /* PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB               ((u16)0x1000)            /* PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC               ((u16)0x2000)            /* PC[11] pin */
#define AFIO_EXTICR3_EXTI11_PD               ((u16)0x3000)            /* PD[11] pin */
#define AFIO_EXTICR3_EXTI11_PE               ((u16)0x4000)            /* PE[11] pin */
#define AFIO_EXTICR3_EXTI11_PF               ((u16)0x5000)            /* PF[11] pin */
#define AFIO_EXTICR3_EXTI11_PG               ((u16)0x6000)            /* PG[11] pin */


/*****************  Bit definition for AFIO_EXTICR4 register  *****************/
#define AFIO_EXTICR4_EXTI12                  ((u16)0x000F)            /* EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13                  ((u16)0x00F0)            /* EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14                  ((u16)0x0F00)            /* EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15                  ((u16)0xF000)            /* EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               ((u16)0x0000)            /* PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB               ((u16)0x0001)            /* PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC               ((u16)0x0002)            /* PC[12] pin */
#define AFIO_EXTICR4_EXTI12_PD               ((u16)0x0003)            /* PD[12] pin */
#define AFIO_EXTICR4_EXTI12_PE               ((u16)0x0004)            /* PE[12] pin */
#define AFIO_EXTICR4_EXTI12_PF               ((u16)0x0005)            /* PF[12] pin */
#define AFIO_EXTICR4_EXTI12_PG               ((u16)0x0006)            /* PG[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               ((u16)0x0000)            /* PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB               ((u16)0x0010)            /* PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC               ((u16)0x0020)            /* PC[13] pin */
#define AFIO_EXTICR4_EXTI13_PD               ((u16)0x0030)            /* PD[13] pin */
#define AFIO_EXTICR4_EXTI13_PE               ((u16)0x0040)            /* PE[13] pin */
#define AFIO_EXTICR4_EXTI13_PF               ((u16)0x0050)            /* PF[13] pin */
#define AFIO_EXTICR4_EXTI13_PG               ((u16)0x0060)            /* PG[13] pin */

/* EXTI14 configuration */  
#define AFIO_EXTICR4_EXTI14_PA               ((u16)0x0000)            /* PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB               ((u16)0x0100)            /* PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC               ((u16)0x0200)            /* PC[14] pin */
#define AFIO_EXTICR4_EXTI14_PD               ((u16)0x0300)            /* PD[14] pin */
#define AFIO_EXTICR4_EXTI14_PE               ((u16)0x0400)            /* PE[14] pin */
#define AFIO_EXTICR4_EXTI14_PF               ((u16)0x0500)            /* PF[14] pin */
#define AFIO_EXTICR4_EXTI14_PG               ((u16)0x0600)            /* PG[14] pin */

/* EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               ((u16)0x0000)            /* PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB               ((u16)0x1000)            /* PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC               ((u16)0x2000)            /* PC[15] pin */
#define AFIO_EXTICR4_EXTI15_PD               ((u16)0x3000)            /* PD[15] pin */
#define AFIO_EXTICR4_EXTI15_PE               ((u16)0x4000)            /* PE[15] pin */
#define AFIO_EXTICR4_EXTI15_PF               ((u16)0x5000)            /* PF[15] pin */
#define AFIO_EXTICR4_EXTI15_PG               ((u16)0x6000)            /* PG[15] pin */



/******************************************************************************/
/*                                                                            */
/*                               SystemTick                                   */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SysTick_CTRL register  *****************/
#define  SysTick_CTRL_ENABLE                 ((u32)0x00000001)        /* Counter enable */
#define  SysTick_CTRL_TICKINT                ((u32)0x00000002)        /* Counting down to 0 pends the SysTick handler */
#define  SysTick_CTRL_CLKSOURCE              ((u32)0x00000004)        /* Clock source */
#define  SysTick_CTRL_COUNTFLAG              ((u32)0x00010000)        /* Count Flag */


/*****************  Bit definition for SysTick_LOAD register  *****************/
#define  SysTick_LOAD_RELOAD                 ((u32)0x00FFFFFF)        /* Value to load into the SysTick Current Value Register when the counter reaches 0 */


/*****************  Bit definition for SysTick_VAL register  ******************/
#define  SysTick_VAL_CURRENT                 ((u32)0x00FFFFFF)        /* Current value at the time the register is accessed */


/*****************  Bit definition for SysTick_CALIB register  ****************/
#define  SysTick_CALIB_TENMS                 ((u32)0x00FFFFFF)        /* Reload value to use for 10ms timing */
#define  SysTick_CALIB_SKEW                  ((u32)0x40000000)        /* Calibration value is not exactly 10 ms */
#define  SysTick_CALIB_NOREF                 ((u32)0x80000000)        /* The reference clock is not provided */



/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for NVIC_ISER register  *******************/
#define  NVIC_ISER_SETENA                    ((u32)0xFFFFFFFF)        /* Interrupt set enable bits */
#define  NVIC_ISER_SETENA_0                  ((u32)0x00000001)        /* bit 0 */
#define  NVIC_ISER_SETENA_1                  ((u32)0x00000002)        /* bit 1 */
#define  NVIC_ISER_SETENA_2                  ((u32)0x00000004)        /* bit 2 */
#define  NVIC_ISER_SETENA_3                  ((u32)0x00000008)        /* bit 3 */
#define  NVIC_ISER_SETENA_4                  ((u32)0x00000010)        /* bit 4 */
#define  NVIC_ISER_SETENA_5                  ((u32)0x00000020)        /* bit 5 */
#define  NVIC_ISER_SETENA_6                  ((u32)0x00000040)        /* bit 6 */
#define  NVIC_ISER_SETENA_7                  ((u32)0x00000080)        /* bit 7 */
#define  NVIC_ISER_SETENA_8                  ((u32)0x00000100)        /* bit 8 */
#define  NVIC_ISER_SETENA_9                  ((u32)0x00000200)        /* bit 9 */
#define  NVIC_ISER_SETENA_10                 ((u32)0x00000400)        /* bit 10 */
#define  NVIC_ISER_SETENA_11                 ((u32)0x00000800)        /* bit 11 */
#define  NVIC_ISER_SETENA_12                 ((u32)0x00001000)        /* bit 12 */
#define  NVIC_ISER_SETENA_13                 ((u32)0x00002000)        /* bit 13 */
#define  NVIC_ISER_SETENA_14                 ((u32)0x00004000)        /* bit 14 */
#define  NVIC_ISER_SETENA_15                 ((u32)0x00008000)        /* bit 15 */
#define  NVIC_ISER_SETENA_16                 ((u32)0x00010000)        /* bit 16 */
#define  NVIC_ISER_SETENA_17                 ((u32)0x00020000)        /* bit 17 */
#define  NVIC_ISER_SETENA_18                 ((u32)0x00040000)        /* bit 18 */
#define  NVIC_ISER_SETENA_19                 ((u32)0x00080000)        /* bit 19 */
#define  NVIC_ISER_SETENA_20                 ((u32)0x00100000)        /* bit 20 */
#define  NVIC_ISER_SETENA_21                 ((u32)0x00200000)        /* bit 21 */
#define  NVIC_ISER_SETENA_22                 ((u32)0x00400000)        /* bit 22 */
#define  NVIC_ISER_SETENA_23                 ((u32)0x00800000)        /* bit 23 */
#define  NVIC_ISER_SETENA_24                 ((u32)0x01000000)        /* bit 24 */
#define  NVIC_ISER_SETENA_25                 ((u32)0x02000000)        /* bit 25 */
#define  NVIC_ISER_SETENA_26                 ((u32)0x04000000)        /* bit 26 */
#define  NVIC_ISER_SETENA_27                 ((u32)0x08000000)        /* bit 27 */
#define  NVIC_ISER_SETENA_28                 ((u32)0x10000000)        /* bit 28 */
#define  NVIC_ISER_SETENA_29                 ((u32)0x20000000)        /* bit 29 */
#define  NVIC_ISER_SETENA_30                 ((u32)0x40000000)        /* bit 30 */
#define  NVIC_ISER_SETENA_31                 ((u32)0x80000000)        /* bit 31 */



/******************  Bit definition for NVIC_ICER register  *******************/
#define  NVIC_ICER_CLRENA                   ((u32)0xFFFFFFFF)        /* Interrupt clear-enable bits */
#define  NVIC_ICER_CLRENA_0                  ((u32)0x00000001)        /* bit 0 */
#define  NVIC_ICER_CLRENA_1                  ((u32)0x00000002)        /* bit 1 */
#define  NVIC_ICER_CLRENA_2                  ((u32)0x00000004)        /* bit 2 */
#define  NVIC_ICER_CLRENA_3                  ((u32)0x00000008)        /* bit 3 */
#define  NVIC_ICER_CLRENA_4                  ((u32)0x00000010)        /* bit 4 */
#define  NVIC_ICER_CLRENA_5                  ((u32)0x00000020)        /* bit 5 */
#define  NVIC_ICER_CLRENA_6                  ((u32)0x00000040)        /* bit 6 */
#define  NVIC_ICER_CLRENA_7                  ((u32)0x00000080)        /* bit 7 */
#define  NVIC_ICER_CLRENA_8                  ((u32)0x00000100)        /* bit 8 */
#define  NVIC_ICER_CLRENA_9                  ((u32)0x00000200)        /* bit 9 */
#define  NVIC_ICER_CLRENA_10                 ((u32)0x00000400)        /* bit 10 */
#define  NVIC_ICER_CLRENA_11                 ((u32)0x00000800)        /* bit 11 */
#define  NVIC_ICER_CLRENA_12                 ((u32)0x00001000)        /* bit 12 */
#define  NVIC_ICER_CLRENA_13                 ((u32)0x00002000)        /* bit 13 */
#define  NVIC_ICER_CLRENA_14                 ((u32)0x00004000)        /* bit 14 */
#define  NVIC_ICER_CLRENA_15                 ((u32)0x00008000)        /* bit 15 */
#define  NVIC_ICER_CLRENA_16                 ((u32)0x00010000)        /* bit 16 */
#define  NVIC_ICER_CLRENA_17                 ((u32)0x00020000)        /* bit 17 */
#define  NVIC_ICER_CLRENA_18                 ((u32)0x00040000)        /* bit 18 */
#define  NVIC_ICER_CLRENA_19                 ((u32)0x00080000)        /* bit 19 */
#define  NVIC_ICER_CLRENA_20                 ((u32)0x00100000)        /* bit 20 */
#define  NVIC_ICER_CLRENA_21                 ((u32)0x00200000)        /* bit 21 */
#define  NVIC_ICER_CLRENA_22                 ((u32)0x00400000)        /* bit 22 */
#define  NVIC_ICER_CLRENA_23                 ((u32)0x00800000)        /* bit 23 */
#define  NVIC_ICER_CLRENA_24                 ((u32)0x01000000)        /* bit 24 */
#define  NVIC_ICER_CLRENA_25                 ((u32)0x02000000)        /* bit 25 */
#define  NVIC_ICER_CLRENA_26                 ((u32)0x04000000)        /* bit 26 */
#define  NVIC_ICER_CLRENA_27                 ((u32)0x08000000)        /* bit 27 */
#define  NVIC_ICER_CLRENA_28                 ((u32)0x10000000)        /* bit 28 */
#define  NVIC_ICER_CLRENA_29                 ((u32)0x20000000)        /* bit 29 */
#define  NVIC_ICER_CLRENA_30                 ((u32)0x40000000)        /* bit 30 */
#define  NVIC_ICER_CLRENA_31                 ((u32)0x80000000)        /* bit 31 */


/******************  Bit definition for NVIC_ISPR register  *******************/
#define  NVIC_ISPR_SETPEND                   ((u32)0xFFFFFFFF)        /* Interrupt set-pending bits */
#define  NVIC_ISPR_SETPEND_0                 ((u32)0x00000001)        /* bit 0 */
#define  NVIC_ISPR_SETPEND_1                 ((u32)0x00000002)        /* bit 1 */
#define  NVIC_ISPR_SETPEND_2                 ((u32)0x00000004)        /* bit 2 */
#define  NVIC_ISPR_SETPEND_3                 ((u32)0x00000008)        /* bit 3 */
#define  NVIC_ISPR_SETPEND_4                 ((u32)0x00000010)        /* bit 4 */
#define  NVIC_ISPR_SETPEND_5                 ((u32)0x00000020)        /* bit 5 */
#define  NVIC_ISPR_SETPEND_6                 ((u32)0x00000040)        /* bit 6 */
#define  NVIC_ISPR_SETPEND_7                 ((u32)0x00000080)        /* bit 7 */
#define  NVIC_ISPR_SETPEND_8                 ((u32)0x00000100)        /* bit 8 */
#define  NVIC_ISPR_SETPEND_9                 ((u32)0x00000200)        /* bit 9 */
#define  NVIC_ISPR_SETPEND_10                ((u32)0x00000400)        /* bit 10 */
#define  NVIC_ISPR_SETPEND_11                ((u32)0x00000800)        /* bit 11 */
#define  NVIC_ISPR_SETPEND_12                ((u32)0x00001000)        /* bit 12 */
#define  NVIC_ISPR_SETPEND_13                ((u32)0x00002000)        /* bit 13 */
#define  NVIC_ISPR_SETPEND_14                ((u32)0x00004000)        /* bit 14 */
#define  NVIC_ISPR_SETPEND_15                ((u32)0x00008000)        /* bit 15 */
#define  NVIC_ISPR_SETPEND_16                ((u32)0x00010000)        /* bit 16 */
#define  NVIC_ISPR_SETPEND_17                ((u32)0x00020000)        /* bit 17 */
#define  NVIC_ISPR_SETPEND_18                ((u32)0x00040000)        /* bit 18 */
#define  NVIC_ISPR_SETPEND_19                ((u32)0x00080000)        /* bit 19 */
#define  NVIC_ISPR_SETPEND_20                ((u32)0x00100000)        /* bit 20 */
#define  NVIC_ISPR_SETPEND_21                ((u32)0x00200000)        /* bit 21 */
#define  NVIC_ISPR_SETPEND_22                ((u32)0x00400000)        /* bit 22 */
#define  NVIC_ISPR_SETPEND_23                ((u32)0x00800000)        /* bit 23 */
#define  NVIC_ISPR_SETPEND_24                ((u32)0x01000000)        /* bit 24 */
#define  NVIC_ISPR_SETPEND_25                ((u32)0x02000000)        /* bit 25 */
#define  NVIC_ISPR_SETPEND_26                ((u32)0x04000000)        /* bit 26 */
#define  NVIC_ISPR_SETPEND_27                ((u32)0x08000000)        /* bit 27 */
#define  NVIC_ISPR_SETPEND_28                ((u32)0x10000000)        /* bit 28 */
#define  NVIC_ISPR_SETPEND_29                ((u32)0x20000000)        /* bit 29 */
#define  NVIC_ISPR_SETPEND_30                ((u32)0x40000000)        /* bit 30 */
#define  NVIC_ISPR_SETPEND_31                ((u32)0x80000000)        /* bit 31 */


/******************  Bit definition for NVIC_ICPR register  *******************/
#define  NVIC_ICPR_CLRPEND                   ((u32)0xFFFFFFFF)        /* Interrupt clear-pending bits */
#define  NVIC_ICPR_CLRPEND_0                 ((u32)0x00000001)        /* bit 0 */
#define  NVIC_ICPR_CLRPEND_1                 ((u32)0x00000002)        /* bit 1 */
#define  NVIC_ICPR_CLRPEND_2                 ((u32)0x00000004)        /* bit 2 */
#define  NVIC_ICPR_CLRPEND_3                 ((u32)0x00000008)        /* bit 3 */
#define  NVIC_ICPR_CLRPEND_4                 ((u32)0x00000010)        /* bit 4 */
#define  NVIC_ICPR_CLRPEND_5                 ((u32)0x00000020)        /* bit 5 */
#define  NVIC_ICPR_CLRPEND_6                 ((u32)0x00000040)        /* bit 6 */
#define  NVIC_ICPR_CLRPEND_7                 ((u32)0x00000080)        /* bit 7 */
#define  NVIC_ICPR_CLRPEND_8                 ((u32)0x00000100)        /* bit 8 */
#define  NVIC_ICPR_CLRPEND_9                 ((u32)0x00000200)        /* bit 9 */
#define  NVIC_ICPR_CLRPEND_10                ((u32)0x00000400)        /* bit 10 */
#define  NVIC_ICPR_CLRPEND_11                ((u32)0x00000800)        /* bit 11 */
#define  NVIC_ICPR_CLRPEND_12                ((u32)0x00001000)        /* bit 12 */
#define  NVIC_ICPR_CLRPEND_13                ((u32)0x00002000)        /* bit 13 */
#define  NVIC_ICPR_CLRPEND_14                ((u32)0x00004000)        /* bit 14 */
#define  NVIC_ICPR_CLRPEND_15                ((u32)0x00008000)        /* bit 15 */
#define  NVIC_ICPR_CLRPEND_16                ((u32)0x00010000)        /* bit 16 */
#define  NVIC_ICPR_CLRPEND_17                ((u32)0x00020000)        /* bit 17 */
#define  NVIC_ICPR_CLRPEND_18                ((u32)0x00040000)        /* bit 18 */
#define  NVIC_ICPR_CLRPEND_19                ((u32)0x00080000)        /* bit 19 */
#define  NVIC_ICPR_CLRPEND_20                ((u32)0x00100000)        /* bit 20 */
#define  NVIC_ICPR_CLRPEND_21                ((u32)0x00200000)        /* bit 21 */
#define  NVIC_ICPR_CLRPEND_22                ((u32)0x00400000)        /* bit 22 */
#define  NVIC_ICPR_CLRPEND_23                ((u32)0x00800000)        /* bit 23 */
#define  NVIC_ICPR_CLRPEND_24                ((u32)0x01000000)        /* bit 24 */
#define  NVIC_ICPR_CLRPEND_25                ((u32)0x02000000)        /* bit 25 */
#define  NVIC_ICPR_CLRPEND_26                ((u32)0x04000000)        /* bit 26 */
#define  NVIC_ICPR_CLRPEND_27                ((u32)0x08000000)        /* bit 27 */
#define  NVIC_ICPR_CLRPEND_28                ((u32)0x10000000)        /* bit 28 */
#define  NVIC_ICPR_CLRPEND_29                ((u32)0x20000000)        /* bit 29 */
#define  NVIC_ICPR_CLRPEND_30                ((u32)0x40000000)        /* bit 30 */
#define  NVIC_ICPR_CLRPEND_31                ((u32)0x80000000)        /* bit 31 */


/******************  Bit definition for NVIC_IABR register  *******************/
#define  NVIC_IABR_ACTIVE                    ((u32)0xFFFFFFFF)        /* Interrupt active flags */
#define  NVIC_IABR_ACTIVE_0                  ((u32)0x00000001)        /* bit 0 */
#define  NVIC_IABR_ACTIVE_1                  ((u32)0x00000002)        /* bit 1 */
#define  NVIC_IABR_ACTIVE_2                  ((u32)0x00000004)        /* bit 2 */
#define  NVIC_IABR_ACTIVE_3                  ((u32)0x00000008)        /* bit 3 */
#define  NVIC_IABR_ACTIVE_4                  ((u32)0x00000010)        /* bit 4 */
#define  NVIC_IABR_ACTIVE_5                  ((u32)0x00000020)        /* bit 5 */
#define  NVIC_IABR_ACTIVE_6                  ((u32)0x00000040)        /* bit 6 */
#define  NVIC_IABR_ACTIVE_7                  ((u32)0x00000080)        /* bit 7 */
#define  NVIC_IABR_ACTIVE_8                  ((u32)0x00000100)        /* bit 8 */
#define  NVIC_IABR_ACTIVE_9                  ((u32)0x00000200)        /* bit 9 */
#define  NVIC_IABR_ACTIVE_10                 ((u32)0x00000400)        /* bit 10 */
#define  NVIC_IABR_ACTIVE_11                 ((u32)0x00000800)        /* bit 11 */
#define  NVIC_IABR_ACTIVE_12                 ((u32)0x00001000)        /* bit 12 */
#define  NVIC_IABR_ACTIVE_13                 ((u32)0x00002000)        /* bit 13 */
#define  NVIC_IABR_ACTIVE_14                 ((u32)0x00004000)        /* bit 14 */
#define  NVIC_IABR_ACTIVE_15                 ((u32)0x00008000)        /* bit 15 */
#define  NVIC_IABR_ACTIVE_16                 ((u32)0x00010000)        /* bit 16 */
#define  NVIC_IABR_ACTIVE_17                 ((u32)0x00020000)        /* bit 17 */
#define  NVIC_IABR_ACTIVE_18                 ((u32)0x00040000)        /* bit 18 */
#define  NVIC_IABR_ACTIVE_19                 ((u32)0x00080000)        /* bit 19 */
#define  NVIC_IABR_ACTIVE_20                 ((u32)0x00100000)        /* bit 20 */
#define  NVIC_IABR_ACTIVE_21                 ((u32)0x00200000)        /* bit 21 */
#define  NVIC_IABR_ACTIVE_22                 ((u32)0x00400000)        /* bit 22 */
#define  NVIC_IABR_ACTIVE_23                 ((u32)0x00800000)        /* bit 23 */
#define  NVIC_IABR_ACTIVE_24                 ((u32)0x01000000)        /* bit 24 */
#define  NVIC_IABR_ACTIVE_25                 ((u32)0x02000000)        /* bit 25 */
#define  NVIC_IABR_ACTIVE_26                 ((u32)0x04000000)        /* bit 26 */
#define  NVIC_IABR_ACTIVE_27                 ((u32)0x08000000)        /* bit 27 */
#define  NVIC_IABR_ACTIVE_28                 ((u32)0x10000000)        /* bit 28 */
#define  NVIC_IABR_ACTIVE_29                 ((u32)0x20000000)        /* bit 29 */
#define  NVIC_IABR_ACTIVE_30                 ((u32)0x40000000)        /* bit 30 */
#define  NVIC_IABR_ACTIVE_31                 ((u32)0x80000000)        /* bit 31 */


/******************  Bit definition for NVIC_PRI0 register  *******************/
#define  NVIC_IPR0_PRI_0                     ((u32)0x000000FF)        /* Priority of interrupt 0 */
#define  NVIC_IPR0_PRI_1                     ((u32)0x0000FF00)        /* Priority of interrupt 1 */
#define  NVIC_IPR0_PRI_2                     ((u32)0x00FF0000)        /* Priority of interrupt 2 */
#define  NVIC_IPR0_PRI_3                     ((u32)0xFF000000)        /* Priority of interrupt 3 */


/******************  Bit definition for NVIC_PRI1 register  *******************/
#define  NVIC_IPR1_PRI_4                     ((u32)0x000000FF)        /* Priority of interrupt 4 */
#define  NVIC_IPR1_PRI_5                     ((u32)0x0000FF00)        /* Priority of interrupt 5 */
#define  NVIC_IPR1_PRI_6                     ((u32)0x00FF0000)        /* Priority of interrupt 6 */
#define  NVIC_IPR1_PRI_7                     ((u32)0xFF000000)        /* Priority of interrupt 7 */


/******************  Bit definition for NVIC_PRI2 register  *******************/
#define  NVIC_IPR2_PRI_8                     ((u32)0x000000FF)        /* Priority of interrupt 8 */
#define  NVIC_IPR2_PRI_9                     ((u32)0x0000FF00)        /* Priority of interrupt 9 */
#define  NVIC_IPR2_PRI_10                    ((u32)0x00FF0000)        /* Priority of interrupt 10 */
#define  NVIC_IPR2_PRI_11                    ((u32)0xFF000000)        /* Priority of interrupt 11 */


/******************  Bit definition for NVIC_PRI3 register  *******************/
#define  NVIC_IPR3_PRI_12                    ((u32)0x000000FF)        /* Priority of interrupt 12 */
#define  NVIC_IPR3_PRI_13                    ((u32)0x0000FF00)        /* Priority of interrupt 13 */
#define  NVIC_IPR3_PRI_14                    ((u32)0x00FF0000)        /* Priority of interrupt 14 */
#define  NVIC_IPR3_PRI_15                    ((u32)0xFF000000)        /* Priority of interrupt 15 */


/******************  Bit definition for NVIC_PRI4 register  *******************/
#define  NVIC_IPR4_PRI_16                    ((u32)0x000000FF)        /* Priority of interrupt 16 */
#define  NVIC_IPR4_PRI_17                    ((u32)0x0000FF00)        /* Priority of interrupt 17 */
#define  NVIC_IPR4_PRI_18                    ((u32)0x00FF0000)        /* Priority of interrupt 18 */
#define  NVIC_IPR4_PRI_19                    ((u32)0xFF000000)        /* Priority of interrupt 19 */


/******************  Bit definition for NVIC_PRI5 register  *******************/
#define  NVIC_IPR5_PRI_20                    ((u32)0x000000FF)        /* Priority of interrupt 20 */
#define  NVIC_IPR5_PRI_21                    ((u32)0x0000FF00)        /* Priority of interrupt 21 */
#define  NVIC_IPR5_PRI_22                    ((u32)0x00FF0000)        /* Priority of interrupt 22 */
#define  NVIC_IPR5_PRI_23                    ((u32)0xFF000000)        /* Priority of interrupt 23 */


/******************  Bit definition for NVIC_PRI6 register  *******************/
#define  NVIC_IPR6_PRI_24                    ((u32)0x000000FF)        /* Priority of interrupt 24 */
#define  NVIC_IPR6_PRI_25                    ((u32)0x0000FF00)        /* Priority of interrupt 25 */
#define  NVIC_IPR6_PRI_26                    ((u32)0x00FF0000)        /* Priority of interrupt 26 */
#define  NVIC_IPR6_PRI_27                    ((u32)0xFF000000)        /* Priority of interrupt 27 */


/******************  Bit definition for NVIC_PRI7 register  *******************/
#define  NVIC_IPR7_PRI_28                    ((u32)0x000000FF)        /* Priority of interrupt 28 */
#define  NVIC_IPR7_PRI_29                    ((u32)0x0000FF00)        /* Priority of interrupt 29 */
#define  NVIC_IPR7_PRI_30                    ((u32)0x00FF0000)        /* Priority of interrupt 30 */
#define  NVIC_IPR7_PRI_31                    ((u32)0xFF000000)        /* Priority of interrupt 31 */


/******************  Bit definition for SCB_CPUID register  *******************/
#define  SCB_CPUID_REVISION                  ((u32)0x0000000F)        /* Implementation defined revision number */
#define  SCB_CPUID_PARTNO                    ((u32)0x0000FFF0)        /* Number of processor within family */
#define  SCB_CPUID_Constant                  ((u32)0x000F0000)        /* Reads as 0x0F */
#define  SCB_CPUID_VARIANT                   ((u32)0x00F00000)        /* Implementation defined variant number */
#define  SCB_CPUID_IMPLEMENTER               ((u32)0xFF000000)        /* Implementer code. ARM is 0x41 */


/*******************  Bit definition for SCB_ICSR register  *******************/
#define  SCB_ICSR_VECTACTIVE                 ((u32)0x000001FF)        /* Active ISR number field */
#define  SCB_ICSR_RETTOBASE                  ((u32)0x00000800)        /* All active exceptions minus the IPSR_current_exception yields the empty set */
#define  SCB_ICSR_VECTPENDING                ((u32)0x003FF000)        /* Pending ISR number field */
#define  SCB_ICSR_ISRPENDING                 ((u32)0x00400000)        /* Interrupt pending flag */
#define  SCB_ICSR_ISRPREEMPT                 ((u32)0x00800000)        /* It indicates that a pending interrupt becomes active in the next running cycle */
#define  SCB_ICSR_PENDSTCLR                  ((u32)0x02000000)        /* Clear pending SysTick bit */
#define  SCB_ICSR_PENDSTSET                  ((u32)0x04000000)        /* Set pending SysTick bit */
#define  SCB_ICSR_PENDSVCLR                  ((u32)0x08000000)        /* Clear pending pendSV bit */
#define  SCB_ICSR_PENDSVSET                  ((u32)0x10000000)        /* Set pending pendSV bit */
#define  SCB_ICSR_NMIPENDSET                 ((u32)0x80000000)        /* Set pending NMI bit */


/*******************  Bit definition for SCB_VTOR register  *******************/
#define  SCB_VTOR_TBLOFF                     ((u32)0x1FFFFF80)        /* Vector table base offset field */
#define  SCB_VTOR_TBLBASE                    ((u32)0x20000000)        /* Table base in code(0) or RAM(1) */


/******************  Bit definition for SCB_AIRCR register  *******************/
#define  SCB_AIRCR_VECTRESET                 ((u32)0x00000001)        /* System Reset bit */
#define  SCB_AIRCR_VECTCLRACTIVE             ((u32)0x00000002)        /* Clear active vector bit */
#define  SCB_AIRCR_SYSRESETREQ               ((u32)0x00000004)        /* Requests chip control logic to generate a reset */

#define  SCB_AIRCR_PRIGROUP                  ((u32)0x00000700)        /* PRIGROUP[2:0] bits (Priority group) */
#define  SCB_AIRCR_PRIGROUP_0                ((u32)0x00000100)        /* Bit 0 */
#define  SCB_AIRCR_PRIGROUP_1                ((u32)0x00000200)        /* Bit 1 */
#define  SCB_AIRCR_PRIGROUP_2                ((u32)0x00000400)        /* Bit 2  */

/* prority group configuration */
#define  SCB_AIRCR_PRIGROUP0                 ((u32)0x00000000)        /* Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority) */
#define  SCB_AIRCR_PRIGROUP1                 ((u32)0x00000100)        /* Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP2                 ((u32)0x00000200)        /* Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP3                 ((u32)0x00000300)        /* Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP4                 ((u32)0x00000400)        /* Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP5                 ((u32)0x00000500)        /* Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP6                 ((u32)0x00000600)        /* Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP7                 ((u32)0x00000700)        /* Priority group=7 (no pre-emption priority, 8 bits of subpriority) */

#define  SCB_AIRCR_ENDIANESS                 ((u32)0x00008000)        /* Data endianness bit */
#define  SCB_AIRCR_VECTKEY                   ((u32)0xFFFF0000)        /* Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) */


/*******************  Bit definition for SCB_SCR register  ********************/
#define  SCB_SCR_SLEEPONEXIT                 ((u8)0x02)               /* Sleep on exit bit */
#define  SCB_SCR_SLEEPDEEP                   ((u8)0x04)               /* Sleep deep bit */
#define  SCB_SCR_SEVONPEND                   ((u8)0x10)               /* Wake up from WFE */


/********************  Bit definition for SCB_CCR register  *******************/
#define  SCB_CCR_NONBASETHRDENA              ((u16)0x0001)            /* Thread mode can be entered from any level in Handler mode by controlled return value */
#define  SCB_CCR_USERSETMPEND                ((u16)0x0002)            /* Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception */
#define  SCB_CCR_UNALIGN_TRP                 ((u16)0x0008)            /* Trap for unaligned access */
#define  SCB_CCR_DIV_0_TRP                   ((u16)0x0010)            /* Trap on Divide by 0 */
#define  SCB_CCR_BFHFNMIGN                   ((u16)0x0100)            /* Handlers running at priority -1 and -2 */
#define  SCB_CCR_STKALIGN                    ((u16)0x0200)            /* On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */


/*******************  Bit definition for SCB_SHPR register ********************/
#define  SCB_SHPR_PRI_N                      ((u32)0x000000FF)        /* Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
#define  SCB_SHPR_PRI_N1                     ((u32)0x0000FF00)        /* Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
#define  SCB_SHPR_PRI_N2                     ((u32)0x00FF0000)        /* Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
#define  SCB_SHPR_PRI_N3                     ((u32)0xFF000000)        /* Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick */


/******************  Bit definition for SCB_SHCSR register  *******************/
#define  SCB_SHCSR_MEMFAULTACT               ((u32)0x00000001)        /* MemManage is active */
#define  SCB_SHCSR_BUSFAULTACT               ((u32)0x00000002)        /* BusFault is active */
#define  SCB_SHCSR_USGFAULTACT               ((u32)0x00000008)        /* UsageFault is active */
#define  SCB_SHCSR_SVCALLACT                 ((u32)0x00000080)        /* SVCall is active */
#define  SCB_SHCSR_MONITORACT                ((u32)0x00000100)        /* Monitor is active */
#define  SCB_SHCSR_PENDSVACT                 ((u32)0x00000400)        /* PendSV is active */
#define  SCB_SHCSR_SYSTICKACT                ((u32)0x00000800)        /* SysTick is active */
#define  SCB_SHCSR_USGFAULTPENDED            ((u32)0x00001000)        /* Usage Fault is pended */
#define  SCB_SHCSR_MEMFAULTPENDED            ((u32)0x00002000)        /* MemManage is pended */
#define  SCB_SHCSR_BUSFAULTPENDED            ((u32)0x00004000)        /* Bus Fault is pended */
#define  SCB_SHCSR_SVCALLPENDED              ((u32)0x00008000)        /* SVCall is pended */
#define  SCB_SHCSR_MEMFAULTENA               ((u32)0x00010000)        /* MemManage enable */
#define  SCB_SHCSR_BUSFAULTENA               ((u32)0x00020000)        /* Bus Fault enable */
#define  SCB_SHCSR_USGFAULTENA               ((u32)0x00040000)        /* UsageFault enable */


/*******************  Bit definition for SCB_CFSR register  *******************/
/* MFSR */
#define  SCB_CFSR_IACCVIOL                   ((u32)0x00000001)        /* Instruction access violation */
#define  SCB_CFSR_DACCVIOL                   ((u32)0x00000002)        /* Data access violation */
#define  SCB_CFSR_MUNSTKERR                  ((u32)0x00000008)        /* Unstacking error */
#define  SCB_CFSR_MSTKERR                    ((u32)0x00000010)        /* Stacking error */
#define  SCB_CFSR_MMARVALID                  ((u32)0x00000080)        /* Memory Manage Address Register address valid flag */
/* BFSR */
#define  SCB_CFSR_IBUSERR                    ((u32)0x00000100)        /* Instruction bus error flag */
#define  SCB_CFSR_PRECISERR                  ((u32)0x00000200)        /* Precise data bus error */
#define  SCB_CFSR_IMPRECISERR                ((u32)0x00000400)        /* Imprecise data bus error */
#define  SCB_CFSR_UNSTKERR                   ((u32)0x00000800)        /* Unstacking error */
#define  SCB_CFSR_STKERR                     ((u32)0x00001000)        /* Stacking error */
#define  SCB_CFSR_BFARVALID                  ((u32)0x00008000)        /* Bus Fault Address Register address valid flag */
/* UFSR */
#define  SCB_CFSR_UNDEFINSTR                 ((u32)0x00010000)        /* The processor attempt to excecute an undefined instruction */
#define  SCB_CFSR_INVSTATE                   ((u32)0x00020000)        /* Invalid combination of EPSR and instruction */
#define  SCB_CFSR_INVPC                      ((u32)0x00040000)        /* Attempt to load EXC_RETURN into pc illegally */
#define  SCB_CFSR_NOCP                       ((u32)0x00080000)        /* Attempt to use a coprocessor instruction */
#define  SCB_CFSR_UNALIGNED                  ((u32)0x01000000)        /* Fault occurs when there is an attempt to make an unaligned memory access */
#define  SCB_CFSR_DIVBYZERO                  ((u32)0x02000000)        /* Fault occurs when SDIV or DIV instruction is used with a divisor of 0 */


/*******************  Bit definition for SCB_HFSR register  *******************/
#define  SCB_HFSR_VECTTBL                    ((u32)0x00000002)        /* Fault occures because of vector table read on exception processing */
#define  SCB_HFSR_FORCED                     ((u32)0x40000000)        /* Hard Fault activated when a configurable Fault was received and cannot activate */
#define  SCB_HFSR_DEBUGEVT                   ((u32)0x80000000)        /* Fault related to debug */


/*******************  Bit definition for SCB_DFSR register  *******************/
#define  SCB_DFSR_HALTED                     ((u8)0x01)               /* Halt request flag */
#define  SCB_DFSR_BKPT                       ((u8)0x02)               /* BKPT flag */
#define  SCB_DFSR_DWTTRAP                    ((u8)0x04)               /* Data Watchpoint and Trace (DWT) flag */
#define  SCB_DFSR_VCATCH                     ((u8)0x08)               /* Vector catch flag */
#define  SCB_DFSR_EXTERNAL                   ((u8)0x10)               /* External debug request flag */


/*******************  Bit definition for SCB_MMFAR register  ******************/
#define  SCB_MMFAR_ADDRESS                   ((u32)0xFFFFFFFF)        /* Mem Manage fault address field */


/*******************  Bit definition for SCB_BFAR register  *******************/
#define  SCB_BFAR_ADDRESS                    ((u32)0xFFFFFFFF)        /* Bus fault address field */


/*******************  Bit definition for SCB_afsr register  *******************/
#define  SCB_AFSR_IMPDEF                     ((u32)0xFFFFFFFF)        /* Implementation defined */



/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((u32)0x00000001)        /* Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((u32)0x00000002)        /* Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((u32)0x00000004)        /* Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((u32)0x00000008)        /* Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((u32)0x00000010)        /* Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((u32)0x00000020)        /* Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((u32)0x00000040)        /* Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((u32)0x00000080)        /* Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((u32)0x00000100)        /* Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((u32)0x00000200)        /* Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((u32)0x00000400)        /* Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((u32)0x00000800)        /* Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((u32)0x00001000)        /* Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((u32)0x00002000)        /* Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((u32)0x00004000)        /* Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((u32)0x00008000)        /* Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((u32)0x00010000)        /* Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((u32)0x00020000)        /* Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((u32)0x00040000)        /* Interrupt Mask on line 18 */


/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((u32)0x00000001)        /* Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((u32)0x00000002)        /* Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((u32)0x00000004)        /* Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((u32)0x00000008)        /* Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((u32)0x00000010)        /* Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((u32)0x00000020)        /* Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((u32)0x00000040)        /* Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((u32)0x00000080)        /* Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((u32)0x00000100)        /* Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((u32)0x00000200)        /* Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((u32)0x00000400)        /* Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((u32)0x00000800)        /* Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((u32)0x00001000)        /* Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((u32)0x00002000)        /* Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((u32)0x00004000)        /* Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((u32)0x00008000)        /* Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((u32)0x00010000)        /* Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((u32)0x00020000)        /* Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((u32)0x00040000)        /* Event Mask on line 18 */


/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((u32)0x00000001)        /* Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((u32)0x00000002)        /* Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((u32)0x00000004)        /* Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((u32)0x00000008)        /* Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((u32)0x00000010)        /* Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((u32)0x00000020)        /* Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((u32)0x00000040)        /* Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((u32)0x00000080)        /* Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((u32)0x00000100)        /* Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((u32)0x00000200)        /* Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((u32)0x00000400)        /* Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((u32)0x00000800)        /* Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((u32)0x00001000)        /* Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((u32)0x00002000)        /* Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((u32)0x00004000)        /* Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((u32)0x00008000)        /* Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((u32)0x00010000)        /* Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((u32)0x00020000)        /* Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((u32)0x00040000)        /* Rising trigger event configuration bit of line 18 */


/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((u32)0x00000001)        /* Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((u32)0x00000002)        /* Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((u32)0x00000004)        /* Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((u32)0x00000008)        /* Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((u32)0x00000010)        /* Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((u32)0x00000020)        /* Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((u32)0x00000040)        /* Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((u32)0x00000080)        /* Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((u32)0x00000100)        /* Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((u32)0x00000200)        /* Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((u32)0x00000400)        /* Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((u32)0x00000800)        /* Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((u32)0x00001000)        /* Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((u32)0x00002000)        /* Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((u32)0x00004000)        /* Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((u32)0x00008000)        /* Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((u32)0x00010000)        /* Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((u32)0x00020000)        /* Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((u32)0x00040000)        /* Falling trigger event configuration bit of line 18 */


/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((u32)0x00000001)        /* Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((u32)0x00000002)        /* Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((u32)0x00000004)        /* Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((u32)0x00000008)        /* Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((u32)0x00000010)        /* Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((u32)0x00000020)        /* Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((u32)0x00000040)        /* Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((u32)0x00000080)        /* Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((u32)0x00000100)        /* Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((u32)0x00000200)        /* Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((u32)0x00000400)        /* Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((u32)0x00000800)        /* Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((u32)0x00001000)        /* Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((u32)0x00002000)        /* Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((u32)0x00004000)        /* Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((u32)0x00008000)        /* Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((u32)0x00010000)        /* Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((u32)0x00020000)        /* Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((u32)0x00040000)        /* Software Interrupt on line 18 */


/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((u32)0x00000001)        /* Pending bit 0 */
#define  EXTI_PR_PR1                         ((u32)0x00000002)        /* Pending bit 1 */
#define  EXTI_PR_PR2                         ((u32)0x00000004)        /* Pending bit 2 */
#define  EXTI_PR_PR3                         ((u32)0x00000008)        /* Pending bit 3 */
#define  EXTI_PR_PR4                         ((u32)0x00000010)        /* Pending bit 4 */
#define  EXTI_PR_PR5                         ((u32)0x00000020)        /* Pending bit 5 */
#define  EXTI_PR_PR6                         ((u32)0x00000040)        /* Pending bit 6 */
#define  EXTI_PR_PR7                         ((u32)0x00000080)        /* Pending bit 7 */
#define  EXTI_PR_PR8                         ((u32)0x00000100)        /* Pending bit 8 */
#define  EXTI_PR_PR9                         ((u32)0x00000200)        /* Pending bit 9 */
#define  EXTI_PR_PR10                        ((u32)0x00000400)        /* Pending bit 10 */
#define  EXTI_PR_PR11                        ((u32)0x00000800)        /* Pending bit 11 */
#define  EXTI_PR_PR12                        ((u32)0x00001000)        /* Pending bit 12 */
#define  EXTI_PR_PR13                        ((u32)0x00002000)        /* Pending bit 13 */
#define  EXTI_PR_PR14                        ((u32)0x00004000)        /* Pending bit 14 */
#define  EXTI_PR_PR15                        ((u32)0x00008000)        /* Pending bit 15 */
#define  EXTI_PR_PR16                        ((u32)0x00010000)        /* Pending bit 16 */
#define  EXTI_PR_PR17                        ((u32)0x00020000)        /* Pending bit 17 */
#define  EXTI_PR_PR18                        ((u32)0x00040000)        /* Trigger request occurred on the external interrupt line 18 */



/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((u32)0x00000001)        /* Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((u32)0x00000002)        /* Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((u32)0x00000004)        /* Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((u32)0x00000008)        /* Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((u32)0x00000010)        /* Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((u32)0x00000020)        /* Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((u32)0x00000040)        /* Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((u32)0x00000080)        /* Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((u32)0x00000100)        /* Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((u32)0x00000200)        /* Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((u32)0x00000400)        /* Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((u32)0x00000800)        /* Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((u32)0x00001000)        /* Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((u32)0x00002000)        /* Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((u32)0x00004000)        /* Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((u32)0x00008000)        /* Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((u32)0x00010000)        /* Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((u32)0x00020000)        /* Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((u32)0x00040000)        /* Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((u32)0x00080000)        /* Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((u32)0x00100000)        /* Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((u32)0x00200000)        /* Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((u32)0x00400000)        /* Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((u32)0x00800000)        /* Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((u32)0x01000000)        /* Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((u32)0x02000000)        /* Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((u32)0x04000000)        /* Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((u32)0x08000000)        /* Channel 7 Transfer Error flag */


/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((u32)0x00000001)        /* Channel 1 Global interrupt clearr */
#define  DMA_IFCR_CTCIF1                     ((u32)0x00000002)        /* Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((u32)0x00000004)        /* Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((u32)0x00000008)        /* Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((u32)0x00000010)        /* Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((u32)0x00000020)        /* Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((u32)0x00000040)        /* Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((u32)0x00000080)        /* Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((u32)0x00000100)        /* Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((u32)0x00000200)        /* Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((u32)0x00000400)        /* Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((u32)0x00000800)        /* Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((u32)0x00001000)        /* Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((u32)0x00002000)        /* Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((u32)0x00004000)        /* Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((u32)0x00008000)        /* Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((u32)0x00010000)        /* Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((u32)0x00020000)        /* Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((u32)0x00040000)        /* Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((u32)0x00080000)        /* Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((u32)0x00100000)        /* Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((u32)0x00200000)        /* Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((u32)0x00400000)        /* Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((u32)0x00800000)        /* Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((u32)0x01000000)        /* Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((u32)0x02000000)        /* Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((u32)0x04000000)        /* Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((u32)0x08000000)        /* Channel 7 Transfer Error clear */


/*******************  Bit definition for DMA_CCR1 register  *******************/
#define  DMA_CCR1_EN                         ((u16)0x0001)            /* Channel enable*/
#define  DMA_CCR1_TCIE                       ((u16)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR1_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR1_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR1_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR1_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR1_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR1_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR1_PSIZE                      ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR1_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR1_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR1_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR1_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR1_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR1_PL                         ((u16)0x3000)            /* PL[1:0] bits(Channel Priority level) */
#define  DMA_CCR1_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR1_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR1_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode */


/*******************  Bit definition for DMA_CCR2 register  *******************/
#define  DMA_CCR2_EN                         ((u16)0x0001)            /* Channel enable */
#define  DMA_CCR2_TCIE                       ((u16)0x0002)            /* ransfer complete interrupt enable */
#define  DMA_CCR2_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR2_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR2_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR2_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR2_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR2_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR2_PSIZE                      ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR2_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR2_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR2_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR2_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR2_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR2_PL                         ((u16)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR2_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR2_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR2_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode */


/*******************  Bit definition for DMA_CCR3 register  *******************/
#define  DMA_CCR3_EN                         ((u16)0x0001)            /* Channel enable */
#define  DMA_CCR3_TCIE                       ((u16)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR3_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR3_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR3_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR3_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR3_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR3_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR3_PSIZE                      ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR3_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR3_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR3_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR3_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR3_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR3_PL                         ((u16)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR3_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR3_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR3_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode */


/*******************  Bit definition for DMA_CCR4 register  *******************/
#define  DMA_CCR4_EN                         ((u16)0x0001)            /* Channel enable */
#define  DMA_CCR4_TCIE                       ((u16)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR4_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR4_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR4_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR4_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR4_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR4_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR4_PSIZE                      ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR4_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR4_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR4_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR4_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR4_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR4_PL                         ((u16)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR4_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR4_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR4_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode */


/******************  Bit definition for DMA_CCR5 register  *******************/
#define  DMA_CCR5_EN                         ((u16)0x0001)            /* Channel enable */
#define  DMA_CCR5_TCIE                       ((u16)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR5_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR5_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR5_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR5_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR5_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR5_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR5_PSIZE                      ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR5_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR5_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR5_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR5_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR5_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR5_PL                         ((u16)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR5_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR5_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR5_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode enable */


/*******************  Bit definition for DMA_CCR6 register  *******************/
#define  DMA_CCR6_EN                         ((u16)0x0001)            /* Channel enable */
#define  DMA_CCR6_TCIE                       ((u16)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR6_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR6_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR6_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR6_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR6_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR6_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR6_PSIZE                      ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR6_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR6_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR6_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR6_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR6_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR6_PL                         ((u16)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR6_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR6_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR6_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode */


/*******************  Bit definition for DMA_CCR7 register  *******************/
#define  DMA_CCR7_EN                         ((u16)0x0001)            /* Channel enable */
#define  DMA_CCR7_TCIE                       ((u16)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR7_HTIE                       ((u16)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR7_TEIE                       ((u16)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR7_DIR                        ((u16)0x0010)            /* Data transfer direction */
#define  DMA_CCR7_CIRC                       ((u16)0x0020)            /* Circular mode */
#define  DMA_CCR7_PINC                       ((u16)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR7_MINC                       ((u16)0x0080)            /* Memory increment mode */

#define  DMA_CCR7_PSIZE            ,         ((u16)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR7_PSIZE_0                    ((u16)0x0100)            /* Bit 0 */
#define  DMA_CCR7_PSIZE_1                    ((u16)0x0200)            /* Bit 1 */

#define  DMA_CCR7_MSIZE                      ((u16)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR7_MSIZE_0                    ((u16)0x0400)            /* Bit 0 */
#define  DMA_CCR7_MSIZE_1                    ((u16)0x0800)            /* Bit 1 */

#define  DMA_CCR7_PL                         ((u16)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR7_PL_0                       ((u16)0x1000)            /* Bit 0 */
#define  DMA_CCR7_PL_1                       ((u16)0x2000)            /* Bit 1 */

#define  DMA_CCR7_MEM2MEM                    ((u16)0x4000)            /* Memory to memory mode enable */


/******************  Bit definition for DMA_CNDTR1 register  ******************/
#define  DMA_CNDTR1_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CNDTR2 register  ******************/
#define  DMA_CNDTR2_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CNDTR3 register  ******************/
#define  DMA_CNDTR3_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CNDTR4 register  ******************/
#define  DMA_CNDTR4_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CNDTR5 register  ******************/
#define  DMA_CNDTR5_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CNDTR6 register  ******************/
#define  DMA_CNDTR6_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CNDTR7 register  ******************/
#define  DMA_CNDTR7_NDT                      ((u16)0xFFFF)            /* Number of data to Transfer */


/******************  Bit definition for DMA_CPAR1 register  *******************/
#define  DMA_CPAR1_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR2 register  *******************/
#define  DMA_CPAR2_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR3 register  *******************/
#define  DMA_CPAR3_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR4 register  *******************/
#define  DMA_CPAR4_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR5 register  *******************/
#define  DMA_CPAR5_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR6 register  *******************/
#define  DMA_CPAR6_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR7 register  *******************/
#define  DMA_CPAR7_PA                        ((u32)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CMAR1 register  *******************/
#define  DMA_CMAR1_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR2 register  *******************/
#define  DMA_CMAR2_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR3 register  *******************/
#define  DMA_CMAR3_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR4 register  *******************/
#define  DMA_CMAR4_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR5 register  *******************/
#define  DMA_CMAR5_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR6 register  *******************/
#define  DMA_CMAR6_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR7 register  *******************/
#define  DMA_CMAR7_MA                        ((u32)0xFFFFFFFF)        /* Memory Address */



/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((u8)0x01)               /* Analog watchdog flag */
#define  ADC_SR_EOC                          ((u8)0x02)               /* End of conversion */
#define  ADC_SR_JEOC                         ((u8)0x04)               /* Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((u8)0x08)               /* Injected channel Start flag */
#define  ADC_SR_STRT                         ((u8)0x10)               /* Regular channel Start flag */


/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((u32)0x0000001F)        /* AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((u32)0x00000001)        /* Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((u32)0x00000002)        /* Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((u32)0x00000004)        /* Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((u32)0x00000008)        /* Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((u32)0x00000010)        /* Bit 4 */

#define  ADC_CR1_EOCIE                       ((u32)0x00000020)        /* Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((u32)0x00000040)        /* AAnalog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((u32)0x00000080)        /* Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((u32)0x00000100)        /* Scan mode */
#define  ADC_CR1_AWDSGL                      ((u32)0x00000200)        /* Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((u32)0x00000400)        /* Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((u32)0x00000800)        /* Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((u32)0x00001000)        /* Discontinuous mode on injected channels */

#define  ADC_CR1_DISCNUM                     ((u32)0x0000E000)        /* DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((u32)0x00002000)        /* Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((u32)0x00004000)        /* Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((u32)0x00008000)        /* Bit 2 */

#define  ADC_CR1_DUALMOD                     ((u32)0x000F0000)        /* DUALMOD[3:0] bits (Dual mode selection) */
#define  ADC_CR1_DUALMOD_0                   ((u32)0x00010000)        /* Bit 0 */
#define  ADC_CR1_DUALMOD_1                   ((u32)0x00020000)        /* Bit 1 */
#define  ADC_CR1_DUALMOD_2                   ((u32)0x00040000)        /* Bit 2 */
#define  ADC_CR1_DUALMOD_3                   ((u32)0x00080000)        /* Bit 3 */

#define  ADC_CR1_JAWDEN                      ((u32)0x00400000)        /* Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((u32)0x00800000)        /* Analog watchdog enable on regular channels */

  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((u32)0x00000001)        /* A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((u32)0x00000002)        /* Continuous Conversion */
#define  ADC_CR2_CAL                         ((u32)0x00000004)        /* A/D Calibration */
#define  ADC_CR2_RSTCAL                      ((u32)0x00000008)        /* Reset Calibration */
#define  ADC_CR2_DMA                         ((u32)0x00000100)        /* Direct Memory access mode */
#define  ADC_CR2_ALIGN                       ((u32)0x00000800)        /* Data Alignment */

#define  ADC_CR2_JEXTSEL                     ((u32)0x00007000)        /* JEXTSEL[2:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((u32)0x00001000)        /* Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((u32)0x00002000)        /* Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((u32)0x00004000)        /* Bit 2 */

#define  ADC_CR2_JEXTTRIG                    ((u32)0x00008000)        /* External Trigger Conversion mode for injected channels */

#define  ADC_CR2_EXTSEL                      ((u32)0x000E0000)        /* EXTSEL[2:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((u32)0x00020000)        /* Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((u32)0x00040000)        /* Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((u32)0x00080000)        /* Bit 2 */

#define  ADC_CR2_EXTTRIG                     ((u32)0x00100000)        /* External Trigger Conversion mode for regular channels */
#define  ADC_CR2_JSWSTART                    ((u32)0x00200000)        /* Start Conversion of injected channels */
#define  ADC_CR2_SWSTART                     ((u32)0x00400000)        /* Start Conversion of regular channels */
#define  ADC_CR2_TSVREFE                     ((u32)0x00800000)        /* Temperature Sensor and VREFINT Enable */


/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((u32)0x00000007)        /* SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((u32)0x00000001)        /* Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((u32)0x00000002)        /* Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((u32)0x00000004)        /* Bit 2 */

#define  ADC_SMPR1_SMP11                     ((u32)0x00000038)        /* SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((u32)0x00000008)        /* Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((u32)0x00000010)        /* Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((u32)0x00000020)        /* Bit 2 */

#define  ADC_SMPR1_SMP12                     ((u32)0x000001C0)        /* SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((u32)0x00000040)        /* Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((u32)0x00000080)        /* Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((u32)0x00000100)        /* Bit 2 */

#define  ADC_SMPR1_SMP13                     ((u32)0x00000E00)        /* SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((u32)0x00000200)        /* Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((u32)0x00000400)        /* Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((u32)0x00000800)        /* Bit 2 */

#define  ADC_SMPR1_SMP14                     ((u32)0x00007000)        /* SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((u32)0x00001000)        /* Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((u32)0x00002000)        /* Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((u32)0x00004000)        /* Bit 2 */

#define  ADC_SMPR1_SMP15                     ((u32)0x00038000)        /* SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((u32)0x00008000)        /* Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((u32)0x00010000)        /* Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((u32)0x00020000)        /* Bit 2 */

#define  ADC_SMPR1_SMP16                     ((u32)0x001C0000)        /* SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((u32)0x00040000)        /* Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((u32)0x00080000)        /* Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((u32)0x00100000)        /* Bit 2 */

#define  ADC_SMPR1_SMP17                     ((u32)0x00E00000)        /* SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((u32)0x00200000)        /* Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((u32)0x00400000)        /* Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((u32)0x00800000)        /* Bit 2 */


/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((u32)0x00000007)        /* SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((u32)0x00000001)        /* Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((u32)0x00000002)        /* Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((u32)0x00000004)        /* Bit 2 */

#define  ADC_SMPR2_SMP1                      ((u32)0x00000038)        /* SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((u32)0x00000008)        /* Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((u32)0x00000010)        /* Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((u32)0x00000020)        /* Bit 2 */

#define  ADC_SMPR2_SMP2                      ((u32)0x000001C0)        /* SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((u32)0x00000040)        /* Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((u32)0x00000080)        /* Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((u32)0x00000100)        /* Bit 2 */

#define  ADC_SMPR2_SMP3                      ((u32)0x00000E00)        /* SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((u32)0x00000200)        /* Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((u32)0x00000400)        /* Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((u32)0x00000800)        /* Bit 2 */

#define  ADC_SMPR2_SMP4                      ((u32)0x00007000)        /* SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((u32)0x00001000)        /* Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((u32)0x00002000)        /* Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((u32)0x00004000)        /* Bit 2 */

#define  ADC_SMPR2_SMP5                      ((u32)0x00038000)        /* SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((u32)0x00008000)        /* Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((u32)0x00010000)        /* Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((u32)0x00020000)        /* Bit 2 */

#define  ADC_SMPR2_SMP6                      ((u32)0x001C0000)        /* SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((u32)0x00040000)        /* Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((u32)0x00080000)        /* Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((u32)0x00100000)        /* Bit 2 */

#define  ADC_SMPR2_SMP7                      ((u32)0x00E00000)        /* SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((u32)0x00200000)        /* Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((u32)0x00400000)        /* Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((u32)0x00800000)        /* Bit 2 */

#define  ADC_SMPR2_SMP8                      ((u32)0x07000000)        /* SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((u32)0x01000000)        /* Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((u32)0x02000000)        /* Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((u32)0x04000000)        /* Bit 2 */

#define  ADC_SMPR2_SMP9                      ((u32)0x38000000)        /* SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((u32)0x08000000)        /* Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((u32)0x10000000)        /* Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((u32)0x20000000)        /* Bit 2 */


/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((u16)0x0FFF)            /* Data offset for injected channel 1 */


/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((u16)0x0FFF)            /* Data offset for injected channel 2 */


/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((u16)0x0FFF)            /* Data offset for injected channel 3 */


/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((u16)0x0FFF)            /* Data offset for injected channel 4 */


/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((u16)0x0FFF)            /* Analog watchdog high threshold */


/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((u16)0x0FFF)            /* Analog watchdog low threshold */


/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((u32)0x0000001F)        /* SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((u32)0x00000001)        /* Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((u32)0x00000002)        /* Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((u32)0x00000004)        /* Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((u32)0x00000008)        /* Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((u32)0x00000010)        /* Bit 4 */

#define  ADC_SQR1_SQ14                       ((u32)0x000003E0)        /* SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((u32)0x00000020)        /* Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((u32)0x00000040)        /* Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((u32)0x00000080)        /* Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((u32)0x00000100)        /* Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((u32)0x00000200)        /* Bit 4 */

#define  ADC_SQR1_SQ15                       ((u32)0x00007C00)        /* SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((u32)0x00000400)        /* Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((u32)0x00000800)        /* Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((u32)0x00001000)        /* Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((u32)0x00002000)        /* Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((u32)0x00004000)        /* Bit 4 */

#define  ADC_SQR1_SQ16                       ((u32)0x000F8000)        /* SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((u32)0x00008000)        /* Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((u32)0x00010000)        /* Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((u32)0x00020000)        /* Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((u32)0x00040000)        /* Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((u32)0x00080000)        /* Bit 4 */

#define  ADC_SQR1_L                          ((u32)0x00F00000)        /* L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((u32)0x00100000)        /* Bit 0 */
#define  ADC_SQR1_L_1                        ((u32)0x00200000)        /* Bit 1 */
#define  ADC_SQR1_L_2                        ((u32)0x00400000)        /* Bit 2 */
#define  ADC_SQR1_L_3                        ((u32)0x00800000)        /* Bit 3 */


/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((u32)0x0000001F)        /* SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((u32)0x00000001)        /* Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((u32)0x00000002)        /* Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((u32)0x00000004)        /* Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((u32)0x00000008)        /* Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((u32)0x00000010)        /* Bit 4 */

#define  ADC_SQR2_SQ8                        ((u32)0x000003E0)        /* SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((u32)0x00000020)        /* Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((u32)0x00000040)        /* Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((u32)0x00000080)        /* Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((u32)0x00000100)        /* Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((u32)0x00000200)        /* Bit 4 */

#define  ADC_SQR2_SQ9                        ((u32)0x00007C00)        /* SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((u32)0x00000400)        /* Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((u32)0x00000800)        /* Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((u32)0x00001000)        /* Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((u32)0x00002000)        /* Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((u32)0x00004000)        /* Bit 4 */

#define  ADC_SQR2_SQ10                       ((u32)0x000F8000)        /* SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((u32)0x00008000)        /* Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((u32)0x00010000)        /* Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((u32)0x00020000)        /* Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((u32)0x00040000)        /* Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((u32)0x00080000)        /* Bit 4 */

#define  ADC_SQR2_SQ11                       ((u32)0x01F00000)        /* SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((u32)0x00100000)        /* Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((u32)0x00200000)        /* Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((u32)0x00400000)        /* Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((u32)0x00800000)        /* Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((u32)0x01000000)        /* Bit 4 */

#define  ADC_SQR2_SQ12                       ((u32)0x3E000000)        /* SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((u32)0x02000000)        /* Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((u32)0x04000000)        /* Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((u32)0x08000000)        /* Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((u32)0x10000000)        /* Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((u32)0x20000000)        /* Bit 4 */


/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((u32)0x0000001F)        /* SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((u32)0x00000001)        /* Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((u32)0x00000002)        /* Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((u32)0x00000004)        /* Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((u32)0x00000008)        /* Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((u32)0x00000010)        /* Bit 4 */

#define  ADC_SQR3_SQ2                        ((u32)0x000003E0)        /* SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((u32)0x00000020)        /* Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((u32)0x00000040)        /* Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((u32)0x00000080)        /* Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((u32)0x00000100)        /* Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((u32)0x00000200)        /* Bit 4 */

#define  ADC_SQR3_SQ3                        ((u32)0x00007C00)        /* SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((u32)0x00000400)        /* Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((u32)0x00000800)        /* Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((u32)0x00001000)        /* Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((u32)0x00002000)        /* Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((u32)0x00004000)        /* Bit 4 */

#define  ADC_SQR3_SQ4                        ((u32)0x000F8000)        /* SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((u32)0x00008000)        /* Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((u32)0x00010000)        /* Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((u32)0x00020000)        /* Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((u32)0x00040000)        /* Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((u32)0x00080000)        /* Bit 4 */

#define  ADC_SQR3_SQ5                        ((u32)0x01F00000)        /* SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((u32)0x00100000)        /* Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((u32)0x00200000)        /* Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((u32)0x00400000)        /* Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((u32)0x00800000)        /* Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((u32)0x01000000)        /* Bit 4 */

#define  ADC_SQR3_SQ6                        ((u32)0x3E000000)        /* SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((u32)0x02000000)        /* Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((u32)0x04000000)        /* Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((u32)0x08000000)        /* Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((u32)0x10000000)        /* Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((u32)0x20000000)        /* Bit 4 */


/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((u32)0x0000001F)        /* JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  ADC_JSQR_JSQ1_0                     ((u32)0x00000001)        /* Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((u32)0x00000002)        /* Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((u32)0x00000004)        /* Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((u32)0x00000008)        /* Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((u32)0x00000010)        /* Bit 4 */

#define  ADC_JSQR_JSQ2                       ((u32)0x000003E0)        /* JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((u32)0x00000020)        /* Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((u32)0x00000040)        /* Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((u32)0x00000080)        /* Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((u32)0x00000100)        /* Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((u32)0x00000200)        /* Bit 4 */

#define  ADC_JSQR_JSQ3                       ((u32)0x00007C00)        /* JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((u32)0x00000400)        /* Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((u32)0x00000800)        /* Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((u32)0x00001000)        /* Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((u32)0x00002000)        /* Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((u32)0x00004000)        /* Bit 4 */

#define  ADC_JSQR_JSQ4                       ((u32)0x000F8000)        /* JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((u32)0x00008000)        /* Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((u32)0x00010000)        /* Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((u32)0x00020000)        /* Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((u32)0x00040000)        /* Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((u32)0x00080000)        /* Bit 4 */

#define  ADC_JSQR_JL                         ((u32)0x00300000)        /* JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((u32)0x00100000)        /* Bit 0 */
#define  ADC_JSQR_JL_1                       ((u32)0x00200000)        /* Bit 1 */


/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((u16)0xFFFF)            /* Injected data */


/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((u16)0xFFFF)            /* Injected data */


/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((u16)0xFFFF)            /* Injected data */


/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((u16)0xFFFF)            /* Injected data */


/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((u32)0x0000FFFF)        /* Regular data */
#define  ADC_DR_ADC2DATA                     ((u32)0xFFFF0000)        /* ADC2 data */



/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((u32)0x00000001)        /* DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((u32)0x00000002)        /* DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((u32)0x00000004)        /* DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((u32)0x00000038)        /* TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((u32)0x00000008)        /* Bit 0 */
#define  DAC_CR_TSEL1_1                      ((u32)0x00000010)        /* Bit 1 */
#define  DAC_CR_TSEL1_2                      ((u32)0x00000020)        /* Bit 2 */

#define  DAC_CR_WAVE1                        ((u32)0x000000C0)        /* WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((u32)0x00000040)        /* Bit 0 */
#define  DAC_CR_WAVE1_1                      ((u32)0x00000080)        /* Bit 1 */

#define  DAC_CR_MAMP1                        ((u32)0x00000F00)        /* MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((u32)0x00000100)        /* Bit 0 */
#define  DAC_CR_MAMP1_1                      ((u32)0x00000200)        /* Bit 1 */
#define  DAC_CR_MAMP1_2                      ((u32)0x00000400)        /* Bit 2 */
#define  DAC_CR_MAMP1_3                      ((u32)0x00000800)        /* Bit 3 */

#define  DAC_CR_DMAEN1                       ((u32)0x00001000)        /* DAC channel1 DMA enable */
#define  DAC_CR_EN2                          ((u32)0x00010000)        /* DAC channel2 enable */
#define  DAC_CR_BOFF2                        ((u32)0x00020000)        /* DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         ((u32)0x00040000)        /* DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((u32)0x00380000)        /* TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((u32)0x00080000)        /* Bit 0 */
#define  DAC_CR_TSEL2_1                      ((u32)0x00100000)        /* Bit 1 */
#define  DAC_CR_TSEL2_2                      ((u32)0x00200000)        /* Bit 2 */

#define  DAC_CR_WAVE2                        ((u32)0x00C00000)        /* WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((u32)0x00400000)        /* Bit 0 */
#define  DAC_CR_WAVE2_1                      ((u32)0x00800000)        /* Bit 1 */

#define  DAC_CR_MAMP2                        ((u32)0x0F000000)        /* MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((u32)0x01000000)        /* Bit 0 */
#define  DAC_CR_MAMP2_1                      ((u32)0x02000000)        /* Bit 1 */
#define  DAC_CR_MAMP2_2                      ((u32)0x04000000)        /* Bit 2 */
#define  DAC_CR_MAMP2_3                      ((u32)0x08000000)        /* Bit 3 */

#define  DAC_CR_DMAEN2                       ((u32)0x10000000)        /* DAC channel2 DMA enabled */


/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((u8)0x01)               /* DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((u8)0x02)               /* DAC channel2 software trigger */


/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((u16)0x0FFF)            /* DAC channel1 12-bit Right aligned data */


/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((u16)0xFFF0)            /* DAC channel1 12-bit Left aligned data */


/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((u8)0xFF)               /* DAC channel1 8-bit Right aligned data */


/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((u16)0x0FFF)            /* DAC channel2 12-bit Right aligned data */


/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((u16)0xFFF0)            /* DAC channel2 12-bit Left aligned data */


/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((u8)0xFF)               /* DAC channel2 8-bit Right aligned data */


/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((u32)0x00000FFF)        /* DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((u32)0x0FFF0000)        /* DAC channel2 12-bit Right aligned data */


/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((u32)0x0000FFF0)        /* DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((u32)0xFFF00000)        /* DAC channel2 12-bit Left aligned data */


/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((u16)0x00FF)            /* DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((u16)0xFF00)            /* DAC channel2 8-bit Right aligned data */


/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((u16)0x0FFF)            /* DAC channel1 data output */


/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((u16)0x0FFF)            /* DAC channel2 data output */



/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((u16)0x0001)            /* Counter enable */
#define  TIM_CR1_UDIS                        ((u16)0x0002)            /* Update disable */
#define  TIM_CR1_URS                         ((u16)0x0004)            /* Update request source */
#define  TIM_CR1_OPM                         ((u16)0x0008)            /* One pulse mode */
#define  TIM_CR1_DIR                         ((u16)0x0010)            /* Direction */

#define  TIM_CR1_CMS                         ((u16)0x0060)            /* CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((u16)0x0020)            /* Bit 0 */
#define  TIM_CR1_CMS_1                       ((u16)0x0040)            /* Bit 1 */

#define  TIM_CR1_ARPE                        ((u16)0x0080)            /* Auto-reload preload enable */

#define  TIM_CR1_CKD                         ((u16)0x0300)            /* CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((u16)0x0100)            /* Bit 0 */
#define  TIM_CR1_CKD_1                       ((u16)0x0200)            /* Bit 1 */


/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((u16)0x0001)            /* Capture/Compare Preloaded Control */
#define  TIM_CR2_CCUS                        ((u16)0x0004)            /* Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((u16)0x0008)            /* Capture/Compare DMA Selection */

#define  TIM_CR2_MMS                         ((u16)0x0070)            /* MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((u16)0x0010)            /* Bit 0 */
#define  TIM_CR2_MMS_1                       ((u16)0x0020)            /* Bit 1 */
#define  TIM_CR2_MMS_2                       ((u16)0x0040)            /* Bit 2 */

#define  TIM_CR2_TI1S                        ((u16)0x0080)            /* TI1 Selection */
#define  TIM_CR2_OIS1                        ((u16)0x0100)            /* Output Idle state 1 (OC1 output) */
#define  TIM_CR2_OIS1N                       ((u16)0x0200)            /* Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((u16)0x0400)            /* Output Idle state 2 (OC2 output) */
#define  TIM_CR2_OIS2N                       ((u16)0x0800)            /* Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((u16)0x1000)            /* Output Idle state 3 (OC3 output) */
#define  TIM_CR2_OIS3N                       ((u16)0x2000)            /* Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((u16)0x4000)            /* Output Idle state 4 (OC4 output) */


/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((u16)0x0007)            /* SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMCR_SMS_0                      ((u16)0x0001)            /* Bit 0 */
#define  TIM_SMCR_SMS_1                      ((u16)0x0002)            /* Bit 1 */
#define  TIM_SMCR_SMS_2                      ((u16)0x0004)            /* Bit 2 */

#define  TIM_SMCR_TS                         ((u16)0x0070)            /* TS[2:0] bits (Trigger selection) */
#define  TIM_SMCR_TS_0                       ((u16)0x0010)            /* Bit 0 */
#define  TIM_SMCR_TS_1                       ((u16)0x0020)            /* Bit 1 */
#define  TIM_SMCR_TS_2                       ((u16)0x0040)            /* Bit 2 */

#define  TIM_SMCR_MSM                        ((u16)0x0080)            /* Master/slave mode */

#define  TIM_SMCR_ETF                        ((u16)0x0F00)            /* ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((u16)0x0100)            /* Bit 0 */
#define  TIM_SMCR_ETF_1                      ((u16)0x0200)            /* Bit 1 */
#define  TIM_SMCR_ETF_2                      ((u16)0x0400)            /* Bit 2 */
#define  TIM_SMCR_ETF_3                      ((u16)0x0800)            /* Bit 3 */

#define  TIM_SMCR_ETPS                       ((u16)0x3000)            /* ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((u16)0x1000)            /* Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((u16)0x2000)            /* Bit 1 */

#define  TIM_SMCR_ECE                        ((u16)0x4000)            /* External clock enable */
#define  TIM_SMCR_ETP                        ((u16)0x8000)            /* External trigger polarity */


/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((u16)0x0001)            /* Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((u16)0x0002)            /* Capture/Compare 1 interrupt enable */
#define  TIM_DIER_CC2IE                      ((u16)0x0004)            /* Capture/Compare 2 interrupt enable */
#define  TIM_DIER_CC3IE                      ((u16)0x0008)            /* Capture/Compare 3 interrupt enable */
#define  TIM_DIER_CC4IE                      ((u16)0x0010)            /* Capture/Compare 4 interrupt enable */
#define  TIM_DIER_COMIE                      ((u16)0x0020)            /* COM interrupt enable */
#define  TIM_DIER_TIE                        ((u16)0x0040)            /* Trigger interrupt enable */
#define  TIM_DIER_BIE                        ((u16)0x0080)            /* Break interrupt enable */
#define  TIM_DIER_UDE                        ((u16)0x0100)            /* Update DMA request enable */
#define  TIM_DIER_CC1DE                      ((u16)0x0200)            /* Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((u16)0x0400)            /* Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((u16)0x0800)            /* Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((u16)0x1000)            /* Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((u16)0x2000)            /* COM DMA request enable */
#define  TIM_DIER_TDE                        ((u16)0x4000)            /* Trigger DMA request enable */


/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((u16)0x0001)            /* Update interrupt Flag */
#define  TIM_SR_CC1IF                        ((u16)0x0002)            /* Capture/Compare 1 interrupt Flag */
#define  TIM_SR_CC2IF                        ((u16)0x0004)            /* Capture/Compare 2 interrupt Flag */
#define  TIM_SR_CC3IF                        ((u16)0x0008)            /* Capture/Compare 3 interrupt Flag */
#define  TIM_SR_CC4IF                        ((u16)0x0010)            /* Capture/Compare 4 interrupt Flag */
#define  TIM_SR_COMIF                        ((u16)0x0020)            /* COM interrupt Flag */
#define  TIM_SR_TIF                          ((u16)0x0040)            /* Trigger interrupt Flag */
#define  TIM_SR_BIF                          ((u16)0x0080)            /* Break interrupt Flag */
#define  TIM_SR_CC1OF                        ((u16)0x0200)            /* Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((u16)0x0400)            /* Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((u16)0x0800)            /* Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((u16)0x1000)            /* Capture/Compare 4 Overcapture Flag */


/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((u8)0x01)               /* Update Generation */
#define  TIM_EGR_CC1G                        ((u8)0x02)               /* Capture/Compare 1 Generation */
#define  TIM_EGR_CC2G                        ((u8)0x04)               /* Capture/Compare 2 Generation */
#define  TIM_EGR_CC3G                        ((u8)0x08)               /* Capture/Compare 3 Generation */
#define  TIM_EGR_CC4G                        ((u8)0x10)               /* Capture/Compare 4 Generation */
#define  TIM_EGR_COMG                        ((u8)0x20)               /* Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((u8)0x40)               /* Trigger Generation */
#define  TIM_EGR_BG                          ((u8)0x80)               /* Break Generation */


/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((u16)0x0003)            /* CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((u16)0x0001)            /* Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((u16)0x0002)            /* Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((u16)0x0004)            /* Output Compare 1 Fast enable */
#define  TIM_CCMR1_OC1PE                     ((u16)0x0008)            /* Output Compare 1 Preload enable */

#define  TIM_CCMR1_OC1M                      ((u16)0x0070)            /* OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_CCMR1_OC1M_0                    ((u16)0x0010)            /* Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((u16)0x0020)            /* Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((u16)0x0040)            /* Bit 2 */

#define  TIM_CCMR1_OC1CE                     ((u16)0x0080)            /* Output Compare 1Clear Enable */

#define  TIM_CCMR1_CC2S                      ((u16)0x0300)            /* CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((u16)0x0100)            /* Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((u16)0x0200)            /* Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((u16)0x0400)            /* Output Compare 2 Fast enable */
#define  TIM_CCMR1_OC2PE                     ((u16)0x0800)            /* Output Compare 2 Preload enable */

#define  TIM_CCMR1_OC2M                      ((u16)0x7000)            /* OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_CCMR1_OC2M_0                    ((u16)0x1000)            /* Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((u16)0x2000)            /* Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((u16)0x4000)            /* Bit 2 */

#define  TIM_CCMR1_OC2CE                     ((u16)0x8000)            /* Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((u16)0x000C)            /* IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((u16)0x0004)            /* Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((u16)0x0008)            /* Bit 1 */

#define  TIM_CCMR1_IC1F                      ((u16)0x00F0)            /* IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_CCMR1_IC1F_0                    ((u16)0x0010)            /* Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((u16)0x0020)            /* Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((u16)0x0040)            /* Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((u16)0x0080)            /* Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((u16)0x0C00)            /* IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_CCMR1_IC2PSC_0                  ((u16)0x0400)            /* Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((u16)0x0800)            /* Bit 1 */

#define  TIM_CCMR1_IC2F                      ((u16)0xF000)            /* IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_CCMR1_IC2F_0                    ((u16)0x1000)            /* Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((u16)0x2000)            /* Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((u16)0x4000)            /* Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((u16)0x8000)            /* Bit 3 */


/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((u16)0x0003)            /* CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CCMR2_CC3S_0                    ((u16)0x0001)            /* Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((u16)0x0002)            /* Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((u16)0x0004)            /* Output Compare 3 Fast enable */
#define  TIM_CCMR2_OC3PE                     ((u16)0x0008)            /* Output Compare 3 Preload enable */

#define  TIM_CCMR2_OC3M                      ((u16)0x0070)            /* OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((u16)0x0010)            /* Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((u16)0x0020)            /* Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((u16)0x0040)            /* Bit 2 */

#define  TIM_CCMR2_OC3CE                     ((u16)0x0080)            /* Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((u16)0x0300)            /* CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((u16)0x0100)            /* Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((u16)0x0200)            /* Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((u16)0x0400)            /* Output Compare 4 Fast enable */
#define  TIM_CCMR2_OC4PE                     ((u16)0x0800)            /* Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((u16)0x7000)            /* OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((u16)0x1000)            /* Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((u16)0x2000)            /* Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((u16)0x4000)            /* Bit 2 */

#define  TIM_CCMR2_OC4CE                     ((u16)0x8000)            /* Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((u16)0x000C)            /* IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((u16)0x0004)            /* Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((u16)0x0008)            /* Bit 1 */

#define  TIM_CCMR2_IC3F                      ((u16)0x00F0)            /* IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((u16)0x0010)            /* Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((u16)0x0020)            /* Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((u16)0x0040)            /* Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((u16)0x0080)            /* Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((u16)0x0C00)            /* IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((u16)0x0400)            /* Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((u16)0x0800)            /* Bit 1 */

#define  TIM_CCMR2_IC4F                      ((u16)0xF000)            /* IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((u16)0x1000)            /* Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((u16)0x2000)            /* Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((u16)0x4000)            /* Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((u16)0x8000)            /* Bit 3 */


/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((u16)0x0001)            /* Capture/Compare 1 output enable */
#define  TIM_CCER_CC1P                       ((u16)0x0002)            /* Capture/Compare 1 output Polarity */
#define  TIM_CCER_CC1NE                      ((u16)0x0004)            /* Capture/Compare 1 Complementary output enable */
#define  TIM_CCER_CC1NP                      ((u16)0x0008)            /* Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((u16)0x0010)            /* Capture/Compare 2 output enable */
#define  TIM_CCER_CC2P                       ((u16)0x0020)            /* Capture/Compare 2 output Polarity */
#define  TIM_CCER_CC2NE                      ((u16)0x0040)            /* Capture/Compare 2 Complementary output enable */
#define  TIM_CCER_CC2NP                      ((u16)0x0080)            /* Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((u16)0x0100)            /* Capture/Compare 3 output enable */
#define  TIM_CCER_CC3P                       ((u16)0x0200)            /* Capture/Compare 3 output Polarity */
#define  TIM_CCER_CC3NE                      ((u16)0x0400)            /* Capture/Compare 3 Complementary output enable */
#define  TIM_CCER_CC3NP                      ((u16)0x0800)            /* Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((u16)0x1000)            /* Capture/Compare 4 output enable */
#define  TIM_CCER_CC4P                       ((u16)0x2000)            /* Capture/Compare 4 output Polarity */


/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((u16)0xFFFF)            /* Counter Value */


/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((u16)0xFFFF)            /* Prescaler Value */


/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((u16)0xFFFF)            /* actual auto-reload Value */


/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((u8)0xFF)               /* Repetition Counter Value */


/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((u16)0xFFFF)            /* Capture/Compare 1 Value */


/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((u16)0xFFFF)            /* Capture/Compare 2 Value */


/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((u16)0xFFFF)            /* Capture/Compare 3 Value */


/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((u16)0xFFFF)            /* Capture/Compare 4 Value */


/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((u16)0x00FF)            /* DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((u16)0x0001)            /* Bit 0 */
#define  TIM_BDTR_DTG_1                      ((u16)0x0002)            /* Bit 1 */
#define  TIM_BDTR_DTG_2                      ((u16)0x0004)            /* Bit 2 */
#define  TIM_BDTR_DTG_3                      ((u16)0x0008)            /* Bit 3 */
#define  TIM_BDTR_DTG_4                      ((u16)0x0010)            /* Bit 4 */
#define  TIM_BDTR_DTG_5                      ((u16)0x0020)            /* Bit 5 */
#define  TIM_BDTR_DTG_6                      ((u16)0x0040)            /* Bit 6 */
#define  TIM_BDTR_DTG_7                      ((u16)0x0080)            /* Bit 7 */

#define  TIM_BDTR_LOCK                       ((u16)0x0300)            /* LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((u16)0x0100)            /* Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((u16)0x0200)            /* Bit 1 */

#define  TIM_BDTR_OSSI                       ((u16)0x0400)            /* Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((u16)0x0800)            /* Off-State Selection for Run mode */
#define  TIM_BDTR_BKE                        ((u16)0x1000)            /* Break enable */
#define  TIM_BDTR_BKP                        ((u16)0x2000)            /* Break Polarity */
#define  TIM_BDTR_AOE                        ((u16)0x4000)            /* Automatic Output enable */
#define  TIM_BDTR_MOE                        ((u16)0x8000)            /* Main Output enable */


/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((u16)0x001F)            /* DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((u16)0x0001)            /* Bit 0 */
#define  TIM_DCR_DBA_1                       ((u16)0x0002)            /* Bit 1 */
#define  TIM_DCR_DBA_2                       ((u16)0x0004)            /* Bit 2 */
#define  TIM_DCR_DBA_3                       ((u16)0x0008)            /* Bit 3 */
#define  TIM_DCR_DBA_4                       ((u16)0x0010)            /* Bit 4 */

#define  TIM_DCR_DBL                         ((u16)0x1F00)            /* DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((u16)0x0100)            /* Bit 0 */
#define  TIM_DCR_DBL_1                       ((u16)0x0200)            /* Bit 1 */
#define  TIM_DCR_DBL_2                       ((u16)0x0400)            /* Bit 2 */
#define  TIM_DCR_DBL_3                       ((u16)0x0800)            /* Bit 3 */
#define  TIM_DCR_DBL_4                       ((u16)0x1000)            /* Bit 4 */


/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((u16)0xFFFF)            /* DMA register for burst accesses */



/******************************************************************************/
/*                                                                            */
/*                             Real-Time Clock                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for RTC_CRH register  ********************/
#define  RTC_CRH_SECIE                       ((u8)0x01)               /* Second Interrupt Enable */
#define  RTC_CRH_ALRIE                       ((u8)0x02)               /* Alarm Interrupt Enable */
#define  RTC_CRH_OWIE                        ((u8)0x04)               /* OverfloW Interrupt Enable */


/*******************  Bit definition for RTC_CRL register  ********************/
#define  RTC_CRL_SECF                        ((u8)0x01)               /* Second Flag */
#define  RTC_CRL_ALRF                        ((u8)0x02)               /* Alarm Flag */
#define  RTC_CRL_OWF                         ((u8)0x04)               /* OverfloW Flag */
#define  RTC_CRL_RSF                         ((u8)0x08)               /* Registers Synchronized Flag */
#define  RTC_CRL_CNF                         ((u8)0x10)               /* Configuration Flag */
#define  RTC_CRL_RTOFF                       ((u8)0x20)               /* RTC operation OFF */


/*******************  Bit definition for RTC_PRLH register  *******************/
#define  RTC_PRLH_PRL                        ((u16)0x000F)            /* RTC Prescaler Reload Value High */


/*******************  Bit definition for RTC_PRLL register  *******************/
#define  RTC_PRLL_PRL                        ((u16)0xFFFF)            /* RTC Prescaler Reload Value Low */


/*******************  Bit definition for RTC_DIVH register  *******************/
#define  RTC_DIVH_RTC_DIV                    ((u16)0x000F)            /* RTC Clock Divider High */


/*******************  Bit definition for RTC_DIVL register  *******************/
#define  RTC_DIVL_RTC_DIV                    ((u16)0xFFFF)            /* RTC Clock Divider Low */


/*******************  Bit definition for RTC_CNTH register  *******************/
#define  RTC_CNTH_RTC_CNT                    ((u16)0xFFFF)            /* RTC Counter High */


/*******************  Bit definition for RTC_CNTL register  *******************/
#define  RTC_CNTL_RTC_CNT                    ((u16)0xFFFF)            /* RTC Counter Low */


/*******************  Bit definition for RTC_ALRH register  *******************/
#define  RTC_ALRH_RTC_ALR                    ((u16)0xFFFF)            /* RTC Alarm High */


/*******************  Bit definition for RTC_ALRL register  *******************/
#define  RTC_ALRL_RTC_ALR                    ((u16)0xFFFF)            /* RTC Alarm Low */



/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((u16)0xFFFF)            /* Key value (write only, read 0000h) */


/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((u8)0x07)               /* PR[2:0] (Prescaler divider) */
#define  IWDG_PR_PR_0                        ((u8)0x01)               /* Bit 0 */
#define  IWDG_PR_PR_1                        ((u8)0x02)               /* Bit 1 */
#define  IWDG_PR_PR_2                        ((u8)0x04)               /* Bit 2 */


/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((u16)0x0FFF)            /* Watchdog counter reload value */


/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((u8)0x01)               /* Watchdog prescaler value update */
#define  IWDG_SR_RVU                         ((u8)0x02)               /* Watchdog counter reload value update */



/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((u8)0x7F)               /* T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((u8)0x01)               /* Bit 0 */
#define  WWDG_CR_T1                          ((u8)0x02)               /* Bit 1 */
#define  WWDG_CR_T2                          ((u8)0x04)               /* Bit 2 */
#define  WWDG_CR_T3                          ((u8)0x08)               /* Bit 3 */
#define  WWDG_CR_T4                          ((u8)0x10)               /* Bit 4 */
#define  WWDG_CR_T5                          ((u8)0x20)               /* Bit 5 */
#define  WWDG_CR_T6                          ((u8)0x40)               /* Bit 6 */

#define  WWDG_CR_WDGA                        ((u8)0x80)               /* Activation bit */


/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((u16)0x007F)            /* W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((u16)0x0001)            /* Bit 0 */
#define  WWDG_CFR_W1                         ((u16)0x0002)            /* Bit 1 */
#define  WWDG_CFR_W2                         ((u16)0x0004)            /* Bit 2 */
#define  WWDG_CFR_W3                         ((u16)0x0008)            /* Bit 3 */
#define  WWDG_CFR_W4                         ((u16)0x0010)            /* Bit 4 */
#define  WWDG_CFR_W5                         ((u16)0x0020)            /* Bit 5 */
#define  WWDG_CFR_W6                         ((u16)0x0040)            /* Bit 6 */

#define  WWDG_CFR_WDGTB                      ((u16)0x0180)            /* WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((u16)0x0080)            /* Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((u16)0x0100)            /* Bit 1 */

#define  WWDG_CFR_EWI                        ((u16)0x0200)            /* Early Wakeup Interrupt */


/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((u8)0x01)               /* Early Wakeup Interrupt Flag */



/******************************************************************************/
/*                                                                            */
/*                       Flexible Static Memory Controller                    */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for FSMC_BCR1 register  *******************/
#define  FSMC_BCR1_MBKEN                     ((u32)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR1_MUXEN                     ((u32)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR1_MTYP                      ((u32)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR1_MTYP_0                    ((u32)0x00000004)        /* Bit 0 */
#define  FSMC_BCR1_MTYP_1                    ((u32)0x00000008)        /* Bit 1 */

#define  FSMC_BCR1_MWID                      ((u32)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR1_MWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BCR1_MWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_BCR1_FACCEN                    ((u32)0x00000040)        /* Flash access enable */
#define  FSMC_BCR1_BURSTEN                   ((u32)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR1_WAITPOL                   ((u32)0x00000200)        /* Wait signal polarity bit */
#define  FSMC_BCR1_WRAPMOD                   ((u32)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR1_WAITCFG                   ((u32)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR1_WREN                      ((u32)0x00001000)        /* Write enable bit */
#define  FSMC_BCR1_WAITEN                    ((u32)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR1_EXTMOD                    ((u32)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR1_CBURSTRW                  ((u32)0x00080000)        /* Write burst enable */


/******************  Bit definition for FSMC_BCR2 register  *******************/
#define  FSMC_BCR2_MBKEN                     ((u32)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR2_MUXEN                     ((u32)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR2_MTYP                      ((u32)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR2_MTYP_0                    ((u32)0x00000004)        /* Bit 0 */
#define  FSMC_BCR2_MTYP_1                    ((u32)0x00000008)        /* Bit 1 */

#define  FSMC_BCR2_MWID                      ((u32)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR2_MWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BCR2_MWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_BCR2_FACCEN                    ((u32)0x00000040)        /* Flash access enable */
#define  FSMC_BCR2_BURSTEN                   ((u32)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR2_WAITPOL                   ((u32)0x00000200)        /* Wait signal polarity bit */
#define  FSMC_BCR2_WRAPMOD                   ((u32)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR2_WAITCFG                   ((u32)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR2_WREN                      ((u32)0x00001000)        /* Write enable bit */
#define  FSMC_BCR2_WAITEN                    ((u32)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR2_EXTMOD                    ((u32)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR2_CBURSTRW                  ((u32)0x00080000)        /* Write burst enable */


/******************  Bit definition for FSMC_BCR3 register  *******************/
#define  FSMC_BCR3_MBKEN                     ((u32)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR3_MUXEN                     ((u32)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR3_MTYP                      ((u32)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR3_MTYP_0                    ((u32)0x00000004)        /* Bit 0 */
#define  FSMC_BCR3_MTYP_1                    ((u32)0x00000008)        /* Bit 1 */

#define  FSMC_BCR3_MWID                      ((u32)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR3_MWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BCR3_MWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_BCR3_FACCEN                    ((u32)0x00000040)        /* Flash access enable */
#define  FSMC_BCR3_BURSTEN                   ((u32)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR3_WAITPOL                   ((u32)0x00000200)        /* Wait signal polarity bit. */
#define  FSMC_BCR3_WRAPMOD                   ((u32)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR3_WAITCFG                   ((u32)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR3_WREN                      ((u32)0x00001000)        /* Write enable bit */
#define  FSMC_BCR3_WAITEN                    ((u32)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR3_EXTMOD                    ((u32)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR3_CBURSTRW                  ((u32)0x00080000)        /* Write burst enable */


/******************  Bit definition for FSMC_BCR4 register  *******************/
#define  FSMC_BCR4_MBKEN                     ((u32)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR4_MUXEN                     ((u32)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR4_MTYP                      ((u32)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR4_MTYP_0                    ((u32)0x00000004)        /* Bit 0 */
#define  FSMC_BCR4_MTYP_1                    ((u32)0x00000008)        /* Bit 1 */

#define  FSMC_BCR4_MWID                      ((u32)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR4_MWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BCR4_MWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_BCR4_FACCEN                    ((u32)0x00000040)        /* Flash access enable */
#define  FSMC_BCR4_BURSTEN                   ((u32)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR4_WAITPOL                   ((u32)0x00000200)        /* Wait signal polarity bit */
#define  FSMC_BCR4_WRAPMOD                   ((u32)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR4_WAITCFG                   ((u32)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR4_WREN                      ((u32)0x00001000)        /* Write enable bit */
#define  FSMC_BCR4_WAITEN                    ((u32)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR4_EXTMOD                    ((u32)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR4_CBURSTRW                  ((u32)0x00080000)        /* Write burst enable */


/******************  Bit definition for FSMC_BTR1 register  ******************/
#define  FSMC_BTR1_ADDSET                    ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR1_ADDSET_0                  ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BTR1_ADDSET_1                  ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BTR1_ADDSET_2                  ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BTR1_ADDSET_3                  ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BTR1_ADDHLD                    ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR1_ADDHLD_0                  ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BTR1_ADDHLD_1                  ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BTR1_ADDHLD_2                  ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BTR1_ADDHLD_3                  ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BTR1_DATAST                    ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR1_DATAST_0                  ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BTR1_DATAST_1                  ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BTR1_DATAST_2                  ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BTR1_DATAST_3                  ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BTR1_BUSTURN                   ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR1_BUSTURN_0                 ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BTR1_BUSTURN_1                 ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BTR1_BUSTURN_2                 ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BTR1_BUSTURN_3                 ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BTR1_CLKDIV                    ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR1_CLKDIV_0                  ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BTR1_CLKDIV_1                  ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BTR1_CLKDIV_2                  ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BTR1_CLKDIV_3                  ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BTR1_DATLAT                    ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR1_DATLAT_0                  ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BTR1_DATLAT_1                  ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BTR1_DATLAT_2                  ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BTR1_DATLAT_3                  ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BTR1_ACCMOD                    ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR1_ACCMOD_0                  ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BTR1_ACCMOD_1                  ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_BTR2 register  *******************/
#define  FSMC_BTR2_ADDSET                    ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR2_ADDSET_0                  ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BTR2_ADDSET_1                  ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BTR2_ADDSET_2                  ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BTR2_ADDSET_3                  ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BTR2_ADDHLD                    ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR2_ADDHLD_0                  ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BTR2_ADDHLD_1                  ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BTR2_ADDHLD_2                  ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BTR2_ADDHLD_3                  ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BTR2_DATAST                    ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR2_DATAST_0                  ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BTR2_DATAST_1                  ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BTR2_DATAST_2                  ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BTR2_DATAST_3                  ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BTR2_BUSTURN                   ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR2_BUSTURN_0                 ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BTR2_BUSTURN_1                 ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BTR2_BUSTURN_2                 ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BTR2_BUSTURN_3                 ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BTR2_CLKDIV                    ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR2_CLKDIV_0                  ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BTR2_CLKDIV_1                  ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BTR2_CLKDIV_2                  ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BTR2_CLKDIV_3                  ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BTR2_DATLAT                    ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR2_DATLAT_0                  ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BTR2_DATLAT_1                  ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BTR2_DATLAT_2                  ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BTR2_DATLAT_3                  ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BTR2_ACCMOD                    ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR2_ACCMOD_0                  ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BTR2_ACCMOD_1                  ((u32)0x20000000)        /* Bit 1 */


/*******************  Bit definition for FSMC_BTR3 register  *******************/
#define  FSMC_BTR3_ADDSET                    ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR3_ADDSET_0                  ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BTR3_ADDSET_1                  ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BTR3_ADDSET_2                  ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BTR3_ADDSET_3                  ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BTR3_ADDHLD                    ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR3_ADDHLD_0                  ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BTR3_ADDHLD_1                  ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BTR3_ADDHLD_2                  ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BTR3_ADDHLD_3                  ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BTR3_DATAST                    ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR3_DATAST_0                  ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BTR3_DATAST_1                  ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BTR3_DATAST_2                  ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BTR3_DATAST_3                  ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BTR3_BUSTURN                   ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR3_BUSTURN_0                 ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BTR3_BUSTURN_1                 ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BTR3_BUSTURN_2                 ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BTR3_BUSTURN_3                 ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BTR3_CLKDIV                    ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR3_CLKDIV_0                  ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BTR3_CLKDIV_1                  ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BTR3_CLKDIV_2                  ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BTR3_CLKDIV_3                  ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BTR3_DATLAT                    ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR3_DATLAT_0                  ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BTR3_DATLAT_1                  ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BTR3_DATLAT_2                  ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BTR3_DATLAT_3                  ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BTR3_ACCMOD                    ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR3_ACCMOD_0                  ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BTR3_ACCMOD_1                  ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_BTR4 register  *******************/
#define  FSMC_BTR4_ADDSET                    ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR4_ADDSET_0                  ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BTR4_ADDSET_1                  ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BTR4_ADDSET_2                  ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BTR4_ADDSET_3                  ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BTR4_ADDHLD                    ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR4_ADDHLD_0                  ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BTR4_ADDHLD_1                  ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BTR4_ADDHLD_2                  ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BTR4_ADDHLD_3                  ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BTR4_DATAST                    ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR4_DATAST_0                  ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BTR4_DATAST_1                  ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BTR4_DATAST_2                  ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BTR4_DATAST_3                  ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BTR4_BUSTURN                   ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR4_BUSTURN_0                 ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BTR4_BUSTURN_1                 ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BTR4_BUSTURN_2                 ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BTR4_BUSTURN_3                 ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BTR4_CLKDIV                    ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR4_CLKDIV_0                  ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BTR4_CLKDIV_1                  ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BTR4_CLKDIV_2                  ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BTR4_CLKDIV_3                  ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BTR4_DATLAT                    ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR4_DATLAT_0                  ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BTR4_DATLAT_1                  ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BTR4_DATLAT_2                  ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BTR4_DATLAT_3                  ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BTR4_ACCMOD                    ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR4_ACCMOD_0                  ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BTR4_ACCMOD_1                  ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_BWTR1 register  ******************/
#define  FSMC_BWTR1_ADDSET                   ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR1_ADDSET_0                 ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR1_ADDSET_1                 ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR1_ADDSET_2                 ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR1_ADDSET_3                 ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR1_ADDHLD                   ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR1_ADDHLD_0                 ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR1_ADDHLD_1                 ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR1_ADDHLD_2                 ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR1_ADDHLD_3                 ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR1_DATAST                   ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR1_DATAST_0                 ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR1_DATAST_1                 ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR1_DATAST_2                 ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR1_DATAST_3                 ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BWTR1_BUSTURN                  ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BWTR1_BUSTURN_0                ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BWTR1_BUSTURN_1                ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BWTR1_BUSTURN_2                ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BWTR1_BUSTURN_3                ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BWTR1_CLKDIV                   ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR1_CLKDIV_0                 ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR1_CLKDIV_1                 ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BWTR1_CLKDIV_2                 ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR1_CLKDIV_3                 ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR1_DATLAT                   ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR1_DATLAT_0                 ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR1_DATLAT_1                 ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR1_DATLAT_2                 ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR1_DATLAT_3                 ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR1_ACCMOD                   ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR1_ACCMOD_0                 ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR1_ACCMOD_1                 ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_BWTR2 register  ******************/
#define  FSMC_BWTR2_ADDSET                   ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR2_ADDSET_0                 ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR2_ADDSET_1                 ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR2_ADDSET_2                 ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR2_ADDSET_3                 ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR2_ADDHLD                   ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR2_ADDHLD_0                 ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR2_ADDHLD_1                 ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR2_ADDHLD_2                 ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR2_ADDHLD_3                 ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR2_DATAST                   ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR2_DATAST_0                 ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR2_DATAST_1                 ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR2_DATAST_2                 ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR2_DATAST_3                 ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BWTR2_BUSTURN                  ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BWTR2_BUSTURN_0                ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BWTR2_BUSTURN_1                ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BWTR2_BUSTURN_2                ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BWTR2_BUSTURN_3                ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BWTR2_CLKDIV                   ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR2_CLKDIV_0                 ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR2_CLKDIV_1                 ((u32)0x00200000)        /* Bit 1*/
#define  FSMC_BWTR2_CLKDIV_2                 ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR2_CLKDIV_3                 ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR2_DATLAT                   ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR2_DATLAT_0                 ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR2_DATLAT_1                 ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR2_DATLAT_2                 ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR2_DATLAT_3                 ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR2_ACCMOD                   ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR2_ACCMOD_0                 ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR2_ACCMOD_1                 ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_BWTR3 register  ******************/
#define  FSMC_BWTR3_ADDSET                   ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR3_ADDSET_0                 ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR3_ADDSET_1                 ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR3_ADDSET_2                 ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR3_ADDSET_3                 ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR3_ADDHLD                   ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR3_ADDHLD_0                 ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR3_ADDHLD_1                 ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR3_ADDHLD_2                 ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR3_ADDHLD_3                 ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR3_DATAST                   ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR3_DATAST_0                 ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR3_DATAST_1                 ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR3_DATAST_2                 ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR3_DATAST_3                 ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BWTR3_BUSTURN                  ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BWTR3_BUSTURN_0                ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BWTR3_BUSTURN_1                ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BWTR3_BUSTURN_2                ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BWTR3_BUSTURN_3                ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BWTR3_CLKDIV                   ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR3_CLKDIV_0                 ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR3_CLKDIV_1                 ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BWTR3_CLKDIV_2                 ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR3_CLKDIV_3                 ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR3_DATLAT                   ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR3_DATLAT_0                 ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR3_DATLAT_1                 ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR3_DATLAT_2                 ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR3_DATLAT_3                 ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR3_ACCMOD                   ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR3_ACCMOD_0                 ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR3_ACCMOD_1                 ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_BWTR4 register  ******************/
#define  FSMC_BWTR4_ADDSET                   ((u32)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR4_ADDSET_0                 ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR4_ADDSET_1                 ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR4_ADDSET_2                 ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR4_ADDSET_3                 ((u32)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR4_ADDHLD                   ((u32)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR4_ADDHLD_0                 ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR4_ADDHLD_1                 ((u32)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR4_ADDHLD_2                 ((u32)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR4_ADDHLD_3                 ((u32)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR4_DATAST                   ((u32)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR4_DATAST_0                 ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR4_DATAST_1                 ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR4_DATAST_2                 ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR4_DATAST_3                 ((u32)0x00000800)        /* Bit 3 */

#define  FSMC_BWTR4_BUSTURN                  ((u32)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BWTR4_BUSTURN_0                ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_BWTR4_BUSTURN_1                ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_BWTR4_BUSTURN_2                ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_BWTR4_BUSTURN_3                ((u32)0x00080000)        /* Bit 3 */

#define  FSMC_BWTR4_CLKDIV                   ((u32)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR4_CLKDIV_0                 ((u32)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR4_CLKDIV_1                 ((u32)0x00200000)        /* Bit 1 */
#define  FSMC_BWTR4_CLKDIV_2                 ((u32)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR4_CLKDIV_3                 ((u32)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR4_DATLAT                   ((u32)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR4_DATLAT_0                 ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR4_DATLAT_1                 ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR4_DATLAT_2                 ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR4_DATLAT_3                 ((u32)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR4_ACCMOD                   ((u32)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR4_ACCMOD_0                 ((u32)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR4_ACCMOD_1                 ((u32)0x20000000)        /* Bit 1 */


/******************  Bit definition for FSMC_PCR2 register  *******************/
#define  FSMC_PCR2_PWAITEN                   ((u32)0x00000002)        /* Wait feature enable bit */
#define  FSMC_PCR2_PBKEN                     ((u32)0x00000004)        /* PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR2_PTYP                      ((u32)0x00000008)        /* Memory type */

#define  FSMC_PCR2_PWID                      ((u32)0x00000030)        /* PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR2_PWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_PCR2_PWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_PCR2_ECCEN                     ((u32)0x00000040)        /* ECC computation logic enable bit */
#define  FSMC_PCR2_ADLOW                     ((u32)0x00000100)        /* Address low bit delivery */

#define  FSMC_PCR2_TCLR                      ((u32)0x00001E00)        /* TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR2_TCLR_0                    ((u32)0x00000200)        /* Bit 0 */
#define  FSMC_PCR2_TCLR_1                    ((u32)0x00000400)        /* Bit 1 */
#define  FSMC_PCR2_TCLR_2                    ((u32)0x00000800)        /* Bit 2 */
#define  FSMC_PCR2_TCLR_3                    ((u32)0x00001000)        /* Bit 3 */

#define  FSMC_PCR2_TAR                       ((u32)0x0001E000)        /* TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR2_TAR_0                     ((u32)0x00002000)        /* Bit 0 */
#define  FSMC_PCR2_TAR_1                     ((u32)0x00004000)        /* Bit 1 */
#define  FSMC_PCR2_TAR_2                     ((u32)0x00008000)        /* Bit 2 */
#define  FSMC_PCR2_TAR_3                     ((u32)0x00010000)        /* Bit 3 */

#define  FSMC_PCR2_ECCPS                     ((u32)0x000E0000)        /* ECCPS[1:0] bits (ECC page size) */
#define  FSMC_PCR2_ECCPS_0                   ((u32)0x00020000)        /* Bit 0 */
#define  FSMC_PCR2_ECCPS_1                   ((u32)0x00040000)        /* Bit 1 */
#define  FSMC_PCR2_ECCPS_2                   ((u32)0x00080000)        /* Bit 2 */


/******************  Bit definition for FSMC_PCR3 register  *******************/
#define  FSMC_PCR3_PWAITEN                   ((u32)0x00000002)        /* Wait feature enable bit */
#define  FSMC_PCR3_PBKEN                     ((u32)0x00000004)        /* PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR3_PTYP                      ((u32)0x00000008)        /* Memory type */

#define  FSMC_PCR3_PWID                      ((u32)0x00000030)        /* PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR3_PWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_PCR3_PWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_PCR3_ECCEN                     ((u32)0x00000040)        /* ECC computation logic enable bit */
#define  FSMC_PCR3_ADLOW                     ((u32)0x00000100)        /* Address low bit delivery */

#define  FSMC_PCR3_TCLR                      ((u32)0x00001E00)        /* TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR3_TCLR_0                    ((u32)0x00000200)        /* Bit 0 */
#define  FSMC_PCR3_TCLR_1                    ((u32)0x00000400)        /* Bit 1 */
#define  FSMC_PCR3_TCLR_2                    ((u32)0x00000800)        /* Bit 2 */
#define  FSMC_PCR3_TCLR_3                    ((u32)0x00001000)        /* Bit 3 */

#define  FSMC_PCR3_TAR                       ((u32)0x0001E000)        /* TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR3_TAR_0                     ((u32)0x00002000)        /* Bit 0 */
#define  FSMC_PCR3_TAR_1                     ((u32)0x00004000)        /* Bit 1 */
#define  FSMC_PCR3_TAR_2                     ((u32)0x00008000)        /* Bit 2 */
#define  FSMC_PCR3_TAR_3                     ((u32)0x00010000)        /* Bit 3 */

#define  FSMC_PCR3_ECCPS                     ((u32)0x000E0000)        /* ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR3_ECCPS_0                   ((u32)0x00020000)        /* Bit 0 */
#define  FSMC_PCR3_ECCPS_1                   ((u32)0x00040000)        /* Bit 1 */
#define  FSMC_PCR3_ECCPS_2                   ((u32)0x00080000)        /* Bit 2 */


/******************  Bit definition for FSMC_PCR4 register  *******************/
#define  FSMC_PCR4_PWAITEN                   ((u32)0x00000002)        /* Wait feature enable bit */
#define  FSMC_PCR4_PBKEN                     ((u32)0x00000004)        /* PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR4_PTYP                      ((u32)0x00000008)        /* Memory type */

#define  FSMC_PCR4_PWID                      ((u32)0x00000030)        /* PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR4_PWID_0                    ((u32)0x00000010)        /* Bit 0 */
#define  FSMC_PCR4_PWID_1                    ((u32)0x00000020)        /* Bit 1 */

#define  FSMC_PCR4_ECCEN                     ((u32)0x00000040)        /* ECC computation logic enable bit */
#define  FSMC_PCR4_ADLOW                     ((u32)0x00000100)        /* Address low bit delivery */

#define  FSMC_PCR4_TCLR                      ((u32)0x00001E00)        /* TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR4_TCLR_0                    ((u32)0x00000200)        /* Bit 0 */
#define  FSMC_PCR4_TCLR_1                    ((u32)0x00000400)        /* Bit 1 */
#define  FSMC_PCR4_TCLR_2                    ((u32)0x00000800)        /* Bit 2 */
#define  FSMC_PCR4_TCLR_3                    ((u32)0x00001000)        /* Bit 3 */

#define  FSMC_PCR4_TAR                       ((u32)0x0001E000)        /* TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR4_TAR_0                     ((u32)0x00002000)        /* Bit 0 */
#define  FSMC_PCR4_TAR_1                     ((u32)0x00004000)        /* Bit 1 */
#define  FSMC_PCR4_TAR_2                     ((u32)0x00008000)        /* Bit 2 */
#define  FSMC_PCR4_TAR_3                     ((u32)0x00010000)        /* Bit 3 */

#define  FSMC_PCR4_ECCPS                     ((u32)0x000E0000)        /* ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR4_ECCPS_0                   ((u32)0x00020000)        /* Bit 0 */
#define  FSMC_PCR4_ECCPS_1                   ((u32)0x00040000)        /* Bit 1 */
#define  FSMC_PCR4_ECCPS_2                   ((u32)0x00080000)        /* Bit 2 */


/*******************  Bit definition for FSMC_SR2 register  *******************/
#define  FSMC_SR2_IRS                        ((u8)0x01)               /* Interrupt Rising Edge status */
#define  FSMC_SR2_ILS                        ((u8)0x02)               /* Interrupt Level status */
#define  FSMC_SR2_IFS                        ((u8)0x04)               /* Interrupt Falling Edge status */
#define  FSMC_SR2_IREN                       ((u8)0x08)               /* Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR2_ILEN                       ((u8)0x10)               /* Interrupt Level detection Enable bit */
#define  FSMC_SR2_IFEN                       ((u8)0x20)               /* Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR2_FEMPT                      ((u8)0x40)               /* FIFO empty */


/*******************  Bit definition for FSMC_SR3 register  *******************/
#define  FSMC_SR3_IRS                        ((u8)0x01)               /* Interrupt Rising Edge status */
#define  FSMC_SR3_ILS                        ((u8)0x02)               /* Interrupt Level status */
#define  FSMC_SR3_IFS                        ((u8)0x04)               /* Interrupt Falling Edge status */
#define  FSMC_SR3_IREN                       ((u8)0x08)               /* Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR3_ILEN                       ((u8)0x10)               /* Interrupt Level detection Enable bit */
#define  FSMC_SR3_IFEN                       ((u8)0x20)               /* Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR3_FEMPT                      ((u8)0x40)               /* FIFO empty */


/*******************  Bit definition for FSMC_SR4 register  *******************/
#define  FSMC_SR4_IRS                        ((u8)0x01)               /* Interrupt Rising Edge status */
#define  FSMC_SR4_ILS                        ((u8)0x02)               /* Interrupt Level status */
#define  FSMC_SR4_IFS                        ((u8)0x04)               /* Interrupt Falling Edge status */
#define  FSMC_SR4_IREN                       ((u8)0x08)               /* Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR4_ILEN                       ((u8)0x10)               /* Interrupt Level detection Enable bit */
#define  FSMC_SR4_IFEN                       ((u8)0x20)               /* Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR4_FEMPT                      ((u8)0x40)               /* FIFO empty */


/******************  Bit definition for FSMC_PMEM2 register  ******************/
#define  FSMC_PMEM2_MEMSET2                  ((u32)0x000000FF)        /* MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FSMC_PMEM2_MEMSET2_0                ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PMEM2_MEMSET2_1                ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PMEM2_MEMSET2_2                ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PMEM2_MEMSET2_3                ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PMEM2_MEMSET2_4                ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PMEM2_MEMSET2_5                ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PMEM2_MEMSET2_6                ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PMEM2_MEMSET2_7                ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PMEM2_MEMWAIT2                 ((u32)0x0000FF00)        /* MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FSMC_PMEM2_MEMWAIT2_0               ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PMEM2_MEMWAIT2_1               ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PMEM2_MEMWAIT2_2               ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PMEM2_MEMWAIT2_3               ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PMEM2_MEMWAIT2_4               ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PMEM2_MEMWAIT2_5               ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PMEM2_MEMWAIT2_6               ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PMEM2_MEMWAIT2_7               ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PMEM2_MEMHOLD2                 ((u32)0x00FF0000)        /* MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FSMC_PMEM2_MEMHOLD2_0               ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PMEM2_MEMHOLD2_1               ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PMEM2_MEMHOLD2_2               ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PMEM2_MEMHOLD2_3               ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PMEM2_MEMHOLD2_4               ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PMEM2_MEMHOLD2_5               ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PMEM2_MEMHOLD2_6               ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PMEM2_MEMHOLD2_7               ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PMEM2_MEMHIZ2                  ((u32)0xFF000000)        /* MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FSMC_PMEM2_MEMHIZ2_0                ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PMEM2_MEMHIZ2_1                ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PMEM2_MEMHIZ2_2                ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PMEM2_MEMHIZ2_3                ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PMEM2_MEMHIZ2_4                ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PMEM2_MEMHIZ2_5                ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PMEM2_MEMHIZ2_6                ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PMEM2_MEMHIZ2_7                ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_PMEM3 register  ******************/
#define  FSMC_PMEM3_MEMSET3                  ((u32)0x000000FF)        /* MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FSMC_PMEM3_MEMSET3_0                ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PMEM3_MEMSET3_1                ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PMEM3_MEMSET3_2                ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PMEM3_MEMSET3_3                ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PMEM3_MEMSET3_4                ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PMEM3_MEMSET3_5                ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PMEM3_MEMSET3_6                ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PMEM3_MEMSET3_7                ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PMEM3_MEMWAIT3                 ((u32)0x0000FF00)        /* MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FSMC_PMEM3_MEMWAIT3_0               ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PMEM3_MEMWAIT3_1               ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PMEM3_MEMWAIT3_2               ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PMEM3_MEMWAIT3_3               ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PMEM3_MEMWAIT3_4               ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PMEM3_MEMWAIT3_5               ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PMEM3_MEMWAIT3_6               ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PMEM3_MEMWAIT3_7               ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PMEM3_MEMHOLD3                 ((u32)0x00FF0000)        /* MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FSMC_PMEM3_MEMHOLD3_0               ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PMEM3_MEMHOLD3_1               ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PMEM3_MEMHOLD3_2               ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PMEM3_MEMHOLD3_3               ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PMEM3_MEMHOLD3_4               ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PMEM3_MEMHOLD3_5               ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PMEM3_MEMHOLD3_6               ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PMEM3_MEMHOLD3_7               ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PMEM3_MEMHIZ3                  ((u32)0xFF000000)        /* MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FSMC_PMEM3_MEMHIZ3_0                ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PMEM3_MEMHIZ3_1                ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PMEM3_MEMHIZ3_2                ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PMEM3_MEMHIZ3_3                ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PMEM3_MEMHIZ3_4                ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PMEM3_MEMHIZ3_5                ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PMEM3_MEMHIZ3_6                ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PMEM3_MEMHIZ3_7                ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_PMEM4 register  ******************/
#define  FSMC_PMEM4_MEMSET4                  ((u32)0x000000FF)        /* MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FSMC_PMEM4_MEMSET4_0                ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PMEM4_MEMSET4_1                ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PMEM4_MEMSET4_2                ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PMEM4_MEMSET4_3                ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PMEM4_MEMSET4_4                ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PMEM4_MEMSET4_5                ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PMEM4_MEMSET4_6                ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PMEM4_MEMSET4_7                ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PMEM4_MEMWAIT4                 ((u32)0x0000FF00)        /* MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FSMC_PMEM4_MEMWAIT4_0               ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PMEM4_MEMWAIT4_1               ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PMEM4_MEMWAIT4_2               ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PMEM4_MEMWAIT4_3               ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PMEM4_MEMWAIT4_4               ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PMEM4_MEMWAIT4_5               ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PMEM4_MEMWAIT4_6               ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PMEM4_MEMWAIT4_7               ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PMEM4_MEMHOLD4                 ((u32)0x00FF0000)        /* MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FSMC_PMEM4_MEMHOLD4_0               ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PMEM4_MEMHOLD4_1               ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PMEM4_MEMHOLD4_2               ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PMEM4_MEMHOLD4_3               ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PMEM4_MEMHOLD4_4               ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PMEM4_MEMHOLD4_5               ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PMEM4_MEMHOLD4_6               ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PMEM4_MEMHOLD4_7               ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PMEM4_MEMHIZ4                  ((u32)0xFF000000)        /* MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FSMC_PMEM4_MEMHIZ4_0                ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PMEM4_MEMHIZ4_1                ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PMEM4_MEMHIZ4_2                ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PMEM4_MEMHIZ4_3                ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PMEM4_MEMHIZ4_4                ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PMEM4_MEMHIZ4_5                ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PMEM4_MEMHIZ4_6                ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PMEM4_MEMHIZ4_7                ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_PATT2 register  ******************/
#define  FSMC_PATT2_ATTSET2                  ((u32)0x000000FF)        /* ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FSMC_PATT2_ATTSET2_0                ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PATT2_ATTSET2_1                ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PATT2_ATTSET2_2                ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PATT2_ATTSET2_3                ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PATT2_ATTSET2_4                ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PATT2_ATTSET2_5                ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PATT2_ATTSET2_6                ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PATT2_ATTSET2_7                ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PATT2_ATTWAIT2                 ((u32)0x0000FF00)        /* ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FSMC_PATT2_ATTWAIT2_0               ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PATT2_ATTWAIT2_1               ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PATT2_ATTWAIT2_2               ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PATT2_ATTWAIT2_3               ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PATT2_ATTWAIT2_4               ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PATT2_ATTWAIT2_5               ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PATT2_ATTWAIT2_6               ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PATT2_ATTWAIT2_7               ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PATT2_ATTHOLD2                 ((u32)0x00FF0000)        /* ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FSMC_PATT2_ATTHOLD2_0               ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PATT2_ATTHOLD2_1               ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PATT2_ATTHOLD2_2               ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PATT2_ATTHOLD2_3               ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PATT2_ATTHOLD2_4               ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PATT2_ATTHOLD2_5               ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PATT2_ATTHOLD2_6               ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PATT2_ATTHOLD2_7               ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PATT2_ATTHIZ2                  ((u32)0xFF000000)        /* ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FSMC_PATT2_ATTHIZ2_0                ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PATT2_ATTHIZ2_1                ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PATT2_ATTHIZ2_2                ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PATT2_ATTHIZ2_3                ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PATT2_ATTHIZ2_4                ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PATT2_ATTHIZ2_5                ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PATT2_ATTHIZ2_6                ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PATT2_ATTHIZ2_7                ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_PATT3 register  ******************/
#define  FSMC_PATT3_ATTSET3                  ((u32)0x000000FF)        /* ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FSMC_PATT3_ATTSET3_0                ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PATT3_ATTSET3_1                ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PATT3_ATTSET3_2                ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PATT3_ATTSET3_3                ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PATT3_ATTSET3_4                ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PATT3_ATTSET3_5                ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PATT3_ATTSET3_6                ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PATT3_ATTSET3_7                ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PATT3_ATTWAIT3                 ((u32)0x0000FF00)        /* ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FSMC_PATT3_ATTWAIT3_0               ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PATT3_ATTWAIT3_1               ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PATT3_ATTWAIT3_2               ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PATT3_ATTWAIT3_3               ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PATT3_ATTWAIT3_4               ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PATT3_ATTWAIT3_5               ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PATT3_ATTWAIT3_6               ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PATT3_ATTWAIT3_7               ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PATT3_ATTHOLD3                 ((u32)0x00FF0000)        /* ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FSMC_PATT3_ATTHOLD3_0               ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PATT3_ATTHOLD3_1               ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PATT3_ATTHOLD3_2               ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PATT3_ATTHOLD3_3               ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PATT3_ATTHOLD3_4               ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PATT3_ATTHOLD3_5               ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PATT3_ATTHOLD3_6               ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PATT3_ATTHOLD3_7               ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PATT3_ATTHIZ3                  ((u32)0xFF000000)        /* ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FSMC_PATT3_ATTHIZ3_0                ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PATT3_ATTHIZ3_1                ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PATT3_ATTHIZ3_2                ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PATT3_ATTHIZ3_3                ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PATT3_ATTHIZ3_4                ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PATT3_ATTHIZ3_5                ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PATT3_ATTHIZ3_6                ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PATT3_ATTHIZ3_7                ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_PATT4 register  ******************/
#define  FSMC_PATT4_ATTSET4                  ((u32)0x000000FF)        /* ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FSMC_PATT4_ATTSET4_0                ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PATT4_ATTSET4_1                ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PATT4_ATTSET4_2                ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PATT4_ATTSET4_3                ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PATT4_ATTSET4_4                ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PATT4_ATTSET4_5                ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PATT4_ATTSET4_6                ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PATT4_ATTSET4_7                ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PATT4_ATTWAIT4                 ((u32)0x0000FF00)        /* ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FSMC_PATT4_ATTWAIT4_0               ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PATT4_ATTWAIT4_1               ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PATT4_ATTWAIT4_2               ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PATT4_ATTWAIT4_3               ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PATT4_ATTWAIT4_4               ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PATT4_ATTWAIT4_5               ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PATT4_ATTWAIT4_6               ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PATT4_ATTWAIT4_7               ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PATT4_ATTHOLD4                 ((u32)0x00FF0000)        /* ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FSMC_PATT4_ATTHOLD4_0               ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PATT4_ATTHOLD4_1               ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PATT4_ATTHOLD4_2               ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PATT4_ATTHOLD4_3               ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PATT4_ATTHOLD4_4               ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PATT4_ATTHOLD4_5               ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PATT4_ATTHOLD4_6               ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PATT4_ATTHOLD4_7               ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PATT4_ATTHIZ4                  ((u32)0xFF000000)        /* ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FSMC_PATT4_ATTHIZ4_0                ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PATT4_ATTHIZ4_1                ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PATT4_ATTHIZ4_2                ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PATT4_ATTHIZ4_3                ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PATT4_ATTHIZ4_4                ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PATT4_ATTHIZ4_5                ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PATT4_ATTHIZ4_6                ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PATT4_ATTHIZ4_7                ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_PIO4 register  *******************/
#define  FSMC_PIO4_IOSET4                    ((u32)0x000000FF)        /* IOSET4[7:0] bits (I/O 4 setup time) */
#define  FSMC_PIO4_IOSET4_0                  ((u32)0x00000001)        /* Bit 0 */
#define  FSMC_PIO4_IOSET4_1                  ((u32)0x00000002)        /* Bit 1 */
#define  FSMC_PIO4_IOSET4_2                  ((u32)0x00000004)        /* Bit 2 */
#define  FSMC_PIO4_IOSET4_3                  ((u32)0x00000008)        /* Bit 3 */
#define  FSMC_PIO4_IOSET4_4                  ((u32)0x00000010)        /* Bit 4 */
#define  FSMC_PIO4_IOSET4_5                  ((u32)0x00000020)        /* Bit 5 */
#define  FSMC_PIO4_IOSET4_6                  ((u32)0x00000040)        /* Bit 6 */
#define  FSMC_PIO4_IOSET4_7                  ((u32)0x00000080)        /* Bit 7 */

#define  FSMC_PIO4_IOWAIT4                   ((u32)0x0000FF00)        /* IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FSMC_PIO4_IOWAIT4_0                 ((u32)0x00000100)        /* Bit 0 */
#define  FSMC_PIO4_IOWAIT4_1                 ((u32)0x00000200)        /* Bit 1 */
#define  FSMC_PIO4_IOWAIT4_2                 ((u32)0x00000400)        /* Bit 2 */
#define  FSMC_PIO4_IOWAIT4_3                 ((u32)0x00000800)        /* Bit 3 */
#define  FSMC_PIO4_IOWAIT4_4                 ((u32)0x00001000)        /* Bit 4 */
#define  FSMC_PIO4_IOWAIT4_5                 ((u32)0x00002000)        /* Bit 5 */
#define  FSMC_PIO4_IOWAIT4_6                 ((u32)0x00004000)        /* Bit 6 */
#define  FSMC_PIO4_IOWAIT4_7                 ((u32)0x00008000)        /* Bit 7 */

#define  FSMC_PIO4_IOHOLD4                   ((u32)0x00FF0000)        /* IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FSMC_PIO4_IOHOLD4_0                 ((u32)0x00010000)        /* Bit 0 */
#define  FSMC_PIO4_IOHOLD4_1                 ((u32)0x00020000)        /* Bit 1 */
#define  FSMC_PIO4_IOHOLD4_2                 ((u32)0x00040000)        /* Bit 2 */
#define  FSMC_PIO4_IOHOLD4_3                 ((u32)0x00080000)        /* Bit 3 */
#define  FSMC_PIO4_IOHOLD4_4                 ((u32)0x00100000)        /* Bit 4 */
#define  FSMC_PIO4_IOHOLD4_5                 ((u32)0x00200000)        /* Bit 5 */
#define  FSMC_PIO4_IOHOLD4_6                 ((u32)0x00400000)        /* Bit 6 */
#define  FSMC_PIO4_IOHOLD4_7                 ((u32)0x00800000)        /* Bit 7 */

#define  FSMC_PIO4_IOHIZ4                    ((u32)0xFF000000)        /* IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FSMC_PIO4_IOHIZ4_0                  ((u32)0x01000000)        /* Bit 0 */
#define  FSMC_PIO4_IOHIZ4_1                  ((u32)0x02000000)        /* Bit 1 */
#define  FSMC_PIO4_IOHIZ4_2                  ((u32)0x04000000)        /* Bit 2 */
#define  FSMC_PIO4_IOHIZ4_3                  ((u32)0x08000000)        /* Bit 3 */
#define  FSMC_PIO4_IOHIZ4_4                  ((u32)0x10000000)        /* Bit 4 */
#define  FSMC_PIO4_IOHIZ4_5                  ((u32)0x20000000)        /* Bit 5 */
#define  FSMC_PIO4_IOHIZ4_6                  ((u32)0x40000000)        /* Bit 6 */
#define  FSMC_PIO4_IOHIZ4_7                  ((u32)0x80000000)        /* Bit 7 */


/******************  Bit definition for FSMC_ECCR2 register  ******************/
#define  FSMC_ECCR2_ECC2                     ((u32)0xFFFFFFFF)        /* ECC result */

/******************  Bit definition for FSMC_ECCR3 register  ******************/
#define  FSMC_ECCR3_ECC3                     ((u32)0xFFFFFFFF)        /* ECC result */



/******************************************************************************/
/*                                                                            */
/*                           SD host Interface                                */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  ((u8)0x03)               /* PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                ((u8)0x01)               /* Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                ((u8)0x02)               /* Bit 1 */


/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   ((u16)0x00FF)            /* Clock divide factor */
#define  SDIO_CLKCR_CLKEN                    ((u16)0x0100)            /* Clock enable bit */
#define  SDIO_CLKCR_PWRSAV                   ((u16)0x0200)            /* Power saving configuration bit */
#define  SDIO_CLKCR_BYPASS                   ((u16)0x0400)            /* Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   ((u16)0x1800)            /* WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 ((u16)0x0800)            /* Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 ((u16)0x1000)            /* Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  ((u16)0x2000)            /* SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  ((u16)0x4000)            /* HW Flow Control enable */


/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     ((u32)0xFFFFFFFF)            /* Command argument */


/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   ((u16)0x003F)            /* Command Index */

#define  SDIO_CMD_WAITRESP                   ((u16)0x00C0)            /* WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 ((u16)0x0040)            /*  Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 ((u16)0x0080)            /*  Bit 1 */

#define  SDIO_CMD_WAITINT                    ((u16)0x0100)            /* CPSM Waits for Interrupt Request */
#define  SDIO_CMD_WAITPEND                   ((u16)0x0200)            /* CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     ((u16)0x0400)            /* Command path state machine (CPSM) Enable bit */
#define  SDIO_CMD_SDIOSUSPEND                ((u16)0x0800)            /* SD I/O suspend command */
#define  SDIO_CMD_ENCMDCOMPL                 ((u16)0x1000)            /* Enable CMD completion */
#define  SDIO_CMD_NIEN                       ((u16)0x2000)            /* Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   ((u16)0x4000)            /* CE-ATA command */


/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                ((u8)0x3F)               /* Response command index */


/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0              ((u32)0xFFFFFFFF)        /* Card Status */


/******************  Bit definition for SDIO_RESP1 register  ******************/
#define  SDIO_RESP1_CARDSTATUS1              ((u32)0xFFFFFFFF)        /* Card Status */


/******************  Bit definition for SDIO_RESP2 register  ******************/
#define  SDIO_RESP2_CARDSTATUS2              ((u32)0xFFFFFFFF)        /* Card Status */


/******************  Bit definition for SDIO_RESP3 register  ******************/
#define  SDIO_RESP3_CARDSTATUS3              ((u32)0xFFFFFFFF)        /* Card Status */


/******************  Bit definition for SDIO_RESP4 register  ******************/
#define  SDIO_RESP4_CARDSTATUS4              ((u32)0xFFFFFFFF)        /* Card Status */


/******************  Bit definition for SDIO_DTIMER register  *****************/
#define  SDIO_DTIMER_DATATIME                ((u32)0xFFFFFFFF)        /* Data timeout period. */


/******************  Bit definition for SDIO_DLEN register  *******************/
#define  SDIO_DLEN_DATALENGTH                ((u32)0x01FFFFFF)        /* Data length value */


/******************  Bit definition for SDIO_DCTRL register  ******************/
#define  SDIO_DCTRL_DTEN                     ((u16)0x0001)            /* Data transfer enabled bit */
#define  SDIO_DCTRL_DTDIR                    ((u16)0x0002)            /* Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   ((u16)0x0004)            /* Data transfer mode selection */
#define  SDIO_DCTRL_DMAEN                    ((u16)0x0008)            /* DMA enabled bit */

#define  SDIO_DCTRL_DBLOCKSIZE               ((u16)0x00F0)            /* DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             ((u16)0x0010)            /* Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             ((u16)0x0020)            /* Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             ((u16)0x0040)            /* Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             ((u16)0x0080)            /* Bit 3 */

#define  SDIO_DCTRL_RWSTART                  ((u16)0x0100)            /* Read wait start */
#define  SDIO_DCTRL_RWSTOP                   ((u16)0x0200)            /* Read wait stop */
#define  SDIO_DCTRL_RWMOD                    ((u16)0x0400)            /* Read wait mode */
#define  SDIO_DCTRL_SDIOEN                   ((u16)0x0800)            /* SD I/O enable functions */


/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define  SDIO_DCOUNT_DATACOUNT               ((u32)0x01FFFFFF)        /* Data count value */


/******************  Bit definition for SDIO_STA register  ********************/
#define  SDIO_STA_CCRCFAIL                   ((u32)0x00000001)        /* Command response received (CRC check failed) */
#define  SDIO_STA_DCRCFAIL                   ((u32)0x00000002)        /* Data block sent/received (CRC check failed) */
#define  SDIO_STA_CTIMEOUT                   ((u32)0x00000004)        /* Command response timeout */
#define  SDIO_STA_DTIMEOUT                   ((u32)0x00000008)        /* Data timeout */
#define  SDIO_STA_TXUNDERR                   ((u32)0x00000010)        /* Transmit FIFO underrun error */
#define  SDIO_STA_RXOVERR                    ((u32)0x00000020)        /* Received FIFO overrun error */
#define  SDIO_STA_CMDREND                    ((u32)0x00000040)        /* Command response received (CRC check passed) */
#define  SDIO_STA_CMDSENT                    ((u32)0x00000080)        /* Command sent (no response required) */
#define  SDIO_STA_DATAEND                    ((u32)0x00000100)        /* Data end (data counter, SDIDCOUNT, is zero) */
#define  SDIO_STA_STBITERR                   ((u32)0x00000200)        /* Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STA_DBCKEND                    ((u32)0x00000400)        /* Data block sent/received (CRC check passed) */
#define  SDIO_STA_CMDACT                     ((u32)0x00000800)        /* Command transfer in progress */
#define  SDIO_STA_TXACT                      ((u32)0x00001000)        /* Data transmit in progress */
#define  SDIO_STA_RXACT                      ((u32)0x00002000)        /* Data receive in progress */
#define  SDIO_STA_TXFIFOHE                   ((u32)0x00004000)        /* Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STA_RXFIFOHF                   ((u32)0x00008000)        /* Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STA_TXFIFOF                    ((u32)0x00010000)        /* Transmit FIFO full */
#define  SDIO_STA_RXFIFOF                    ((u32)0x00020000)        /* Receive FIFO full */
#define  SDIO_STA_TXFIFOE                    ((u32)0x00040000)        /* Transmit FIFO empty */
#define  SDIO_STA_RXFIFOE                    ((u32)0x00080000)        /* Receive FIFO empty */
#define  SDIO_STA_TXDAVL                     ((u32)0x00100000)        /* Data available in transmit FIFO */
#define  SDIO_STA_RXDAVL                     ((u32)0x00200000)        /* Data available in receive FIFO */
#define  SDIO_STA_SDIOIT                     ((u32)0x00400000)        /* SDIO interrupt received */
#define  SDIO_STA_CEATAEND                   ((u32)0x00800000)        /* CE-ATA command completion signal received for CMD61 */


/*******************  Bit definition for SDIO_ICR register  *******************/
#define  SDIO_ICR_CCRCFAILC                  ((u32)0x00000001)        /* CCRCFAIL flag clear bit */
#define  SDIO_ICR_DCRCFAILC                  ((u32)0x00000002)        /* DCRCFAIL flag clear bit */
#define  SDIO_ICR_CTIMEOUTC                  ((u32)0x00000004)        /* CTIMEOUT flag clear bit */
#define  SDIO_ICR_DTIMEOUTC                  ((u32)0x00000008)        /* DTIMEOUT flag clear bit */
#define  SDIO_ICR_TXUNDERRC                  ((u32)0x00000010)        /* TXUNDERR flag clear bit */
#define  SDIO_ICR_RXOVERRC                   ((u32)0x00000020)        /* RXOVERR flag clear bit */
#define  SDIO_ICR_CMDRENDC                   ((u32)0x00000040)        /* CMDREND flag clear bit */
#define  SDIO_ICR_CMDSENTC                   ((u32)0x00000080)        /* CMDSENT flag clear bit */
#define  SDIO_ICR_DATAENDC                   ((u32)0x00000100)        /* DATAEND flag clear bit */
#define  SDIO_ICR_STBITERRC                  ((u32)0x00000200)        /* STBITERR flag clear bit */
#define  SDIO_ICR_DBCKENDC                   ((u32)0x00000400)        /* DBCKEND flag clear bit */
#define  SDIO_ICR_SDIOITC                    ((u32)0x00400000)        /* SDIOIT flag clear bit */
#define  SDIO_ICR_CEATAENDC                  ((u32)0x00800000)        /* CEATAEND flag clear bit */


/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE                ((u32)0x00000001)        /* Command CRC Fail Interrupt Enable */
#define  SDIO_MASK_DCRCFAILIE                ((u32)0x00000002)        /* Data CRC Fail Interrupt Enable */
#define  SDIO_MASK_CTIMEOUTIE                ((u32)0x00000004)        /* Command TimeOut Interrupt Enable */
#define  SDIO_MASK_DTIMEOUTIE                ((u32)0x00000008)        /* Data TimeOut Interrupt Enable */
#define  SDIO_MASK_TXUNDERRIE                ((u32)0x00000010)        /* Tx FIFO UnderRun Error Interrupt Enable */
#define  SDIO_MASK_RXOVERRIE                 ((u32)0x00000020)        /* Rx FIFO OverRun Error Interrupt Enable */
#define  SDIO_MASK_CMDRENDIE                 ((u32)0x00000040)        /* Command Response Received Interrupt Enable */
#define  SDIO_MASK_CMDSENTIE                 ((u32)0x00000080)        /* Command Sent Interrupt Enable */
#define  SDIO_MASK_DATAENDIE                 ((u32)0x00000100)        /* Data End Interrupt Enable */
#define  SDIO_MASK_STBITERRIE                ((u32)0x00000200)        /* Start Bit Error Interrupt Enable */
#define  SDIO_MASK_DBCKENDIE                 ((u32)0x00000400)        /* Data Block End Interrupt Enable */
#define  SDIO_MASK_CMDACTIE                  ((u32)0x00000800)        /* CCommand Acting Interrupt Enable */
#define  SDIO_MASK_TXACTIE                   ((u32)0x00001000)        /* Data Transmit Acting Interrupt Enable */
#define  SDIO_MASK_RXACTIE                   ((u32)0x00002000)        /* Data receive acting interrupt enabled */
#define  SDIO_MASK_TXFIFOHEIE                ((u32)0x00004000)        /* Tx FIFO Half Empty interrupt Enable */
#define  SDIO_MASK_RXFIFOHFIE                ((u32)0x00008000)        /* Rx FIFO Half Full interrupt Enable */
#define  SDIO_MASK_TXFIFOFIE                 ((u32)0x00010000)        /* Tx FIFO Full interrupt Enable */
#define  SDIO_MASK_RXFIFOFIE                 ((u32)0x00020000)        /* Rx FIFO Full interrupt Enable */
#define  SDIO_MASK_TXFIFOEIE                 ((u32)0x00040000)        /* Tx FIFO Empty interrupt Enable */
#define  SDIO_MASK_RXFIFOEIE                 ((u32)0x00080000)        /* Rx FIFO Empty interrupt Enable */
#define  SDIO_MASK_TXDAVLIE                  ((u32)0x00100000)        /* Data available in Tx FIFO interrupt Enable */
#define  SDIO_MASK_RXDAVLIE                  ((u32)0x00200000)        /* Data available in Rx FIFO interrupt Enable */
#define  SDIO_MASK_SDIOITIE                  ((u32)0x00400000)        /* SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_MASK_CEATAENDIE                ((u32)0x00800000)        /* CE-ATA command completion signal received Interrupt Enable */


/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define  SDIO_FIFOCNT_FIFOCOUNT              ((u32)0x00FFFFFF)        /* Remaining number of words to be written to or read from the FIFO */


/******************  Bit definition for SDIO_FIFO register  *******************/
#define  SDIO_FIFO_FIFODATA                  ((u32)0xFFFFFFFF)        /* Receive and transmit FIFO data */



/******************************************************************************/
/*                                                                            */
/*                                   USB                                      */
/*                                                                            */
/******************************************************************************/

/* Endpoint-specific registers */
/*******************  Bit definition for USB_EP0R register  *******************/
#define  USB_EP0R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP0R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP0R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP0R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP0R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP0R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP0R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP0R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP0R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP0R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP0R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP0R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP0R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP0R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP0R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP0R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP1R register  *******************/
#define  USB_EP1R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP1R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP1R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP1R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP1R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP1R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP1R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP1R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP1R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP1R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP1R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP1R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP1R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP1R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP1R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP1R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP2R register  *******************/
#define  USB_EP2R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP2R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP2R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP2R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP2R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP2R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP2R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP2R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP2R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP2R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP2R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP2R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP2R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP2R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP2R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP2R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP3R register  *******************/
#define  USB_EP3R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP3R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP3R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP3R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP3R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP3R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP3R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP3R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP3R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP3R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP3R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP3R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP3R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP3R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP3R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP3R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP4R register  *******************/
#define  USB_EP4R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP4R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP4R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP4R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP4R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP4R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP4R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP4R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP4R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP4R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP4R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP4R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP4R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP4R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP4R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP4R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP5R register  *******************/
#define  USB_EP5R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP5R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP5R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP5R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP5R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP5R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP5R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP5R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP5R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP5R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP5R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP5R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP5R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP5R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP5R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP5R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP6R register  *******************/
#define  USB_EP6R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP6R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP6R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP6R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP6R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP6R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP6R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP6R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP6R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP6R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP6R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP6R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP6R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP6R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP6R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP6R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/*******************  Bit definition for USB_EP7R register  *******************/
#define  USB_EP7R_EA                         ((u16)0x000F)            /* Endpoint Address */

#define  USB_EP7R_STAT_TX                    ((u16)0x0030)            /* STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EP7R_STAT_TX_0                  ((u16)0x0010)            /* Bit 0 */
#define  USB_EP7R_STAT_TX_1                  ((u16)0x0020)            /* Bit 1 */

#define  USB_EP7R_DTOG_TX                    ((u16)0x0040)            /* Data Toggle, for transmission transfers */
#define  USB_EP7R_CTR_TX                     ((u16)0x0080)            /* Correct Transfer for transmission */
#define  USB_EP7R_EP_KIND                    ((u16)0x0100)            /* Endpoint Kind */

#define  USB_EP7R_EP_TYPE                    ((u16)0x0600)            /* EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EP7R_EP_TYPE_0                  ((u16)0x0200)            /* Bit 0 */
#define  USB_EP7R_EP_TYPE_1                  ((u16)0x0400)            /* Bit 1 */

#define  USB_EP7R_SETUP                      ((u16)0x0800)            /* Setup transaction completed */

#define  USB_EP7R_STAT_RX                    ((u16)0x3000)            /* STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EP7R_STAT_RX_0                  ((u16)0x1000)            /* Bit 0 */
#define  USB_EP7R_STAT_RX_1                  ((u16)0x2000)            /* Bit 1 */

#define  USB_EP7R_DTOG_RX                    ((u16)0x4000)            /* Data Toggle, for reception transfers */
#define  USB_EP7R_CTR_RX                     ((u16)0x8000)            /* Correct Transfer for reception */


/* Common registers */
/*******************  Bit definition for USB_CNTR register  *******************/
#define  USB_CNTR_FRES                       ((u16)0x0001)            /* Force USB Reset */
#define  USB_CNTR_PDWN                       ((u16)0x0002)            /* Power down */
#define  USB_CNTR_LP_MODE                    ((u16)0x0004)            /* Low-power mode */
#define  USB_CNTR_FSUSP                      ((u16)0x0008)            /* Force suspend */
#define  USB_CNTR_RESUME                     ((u16)0x0010)            /* Resume request */
#define  USB_CNTR_ESOFM                      ((u16)0x0100)            /* Expected Start Of Frame Interrupt Mask */
#define  USB_CNTR_SOFM                       ((u16)0x0200)            /* Start Of Frame Interrupt Mask */
#define  USB_CNTR_RESETM                     ((u16)0x0400)            /* RESET Interrupt Mask */
#define  USB_CNTR_SUSPM                      ((u16)0x0800)            /* Suspend mode Interrupt Mask */
#define  USB_CNTR_WKUPM                      ((u16)0x1000)            /* Wakeup Interrupt Mask */
#define  USB_CNTR_ERRM                       ((u16)0x2000)            /* Error Interrupt Mask */
#define  USB_CNTR_PMAOVRM                    ((u16)0x4000)            /* Packet Memory Area Over / Underrun Interrupt Mask */
#define  USB_CNTR_CTRM                       ((u16)0x8000)            /* Correct Transfer Interrupt Mask */


/*******************  Bit definition for USB_ISTR register  *******************/
#define  USB_ISTR_EP_ID                      ((u16)0x000F)            /* Endpoint Identifier */
#define  USB_ISTR_DIR                        ((u16)0x0010)            /* Direction of transaction */
#define  USB_ISTR_ESOF                       ((u16)0x0100)            /* Expected Start Of Frame */
#define  USB_ISTR_SOF                        ((u16)0x0200)            /* Start Of Frame */
#define  USB_ISTR_RESET                      ((u16)0x0400)            /* USB RESET request */
#define  USB_ISTR_SUSP                       ((u16)0x0800)            /* Suspend mode request */
#define  USB_ISTR_WKUP                       ((u16)0x1000)            /* Wake up */
#define  USB_ISTR_ERR                        ((u16)0x2000)            /* Error */
#define  USB_ISTR_PMAOVR                     ((u16)0x4000)            /* Packet Memory Area Over / Underrun */
#define  USB_ISTR_CTR                        ((u16)0x8000)            /* Correct Transfer */


/*******************  Bit definition for USB_FNR register  ********************/
#define  USB_FNR_FN                          ((u16)0x07FF)            /* Frame Number */
#define  USB_FNR_LSOF                        ((u16)0x1800)            /* Lost SOF */
#define  USB_FNR_LCK                         ((u16)0x2000)            /* Locked */
#define  USB_FNR_RXDM                        ((u16)0x4000)            /* Receive Data - Line Status */
#define  USB_FNR_RXDP                        ((u16)0x8000)            /* Receive Data + Line Status */


/******************  Bit definition for USB_DADDR register  *******************/
#define  USB_DADDR_ADD                       ((u8)0x7F)               /* ADD[6:0] bits (Device Address) */
#define  USB_DADDR_ADD0                      ((u8)0x01)               /* Bit 0 */
#define  USB_DADDR_ADD1                      ((u8)0x02)               /* Bit 1 */
#define  USB_DADDR_ADD2                      ((u8)0x04)               /* Bit 2 */
#define  USB_DADDR_ADD3                      ((u8)0x08)               /* Bit 3 */
#define  USB_DADDR_ADD4                      ((u8)0x10)               /* Bit 4 */
#define  USB_DADDR_ADD5                      ((u8)0x20)               /* Bit 5 */
#define  USB_DADDR_ADD6                      ((u8)0x40)               /* Bit 6 */

#define  USB_DADDR_EF                        ((u8)0x80)               /* Enable Function */


/******************  Bit definition for USB_BTABLE register  ******************/    
#define  USB_BTABLE_BTABLE                   ((u16)0xFFF8)            /* Buffer Table */


/* Buffer descriptor table */
/*****************  Bit definition for USB_ADDR0_TX register  *****************/
#define  USB_ADDR0_TX_ADDR0_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 0 */


/*****************  Bit definition for USB_ADDR1_TX register  *****************/
#define  USB_ADDR1_TX_ADDR1_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 1 */


/*****************  Bit definition for USB_ADDR2_TX register  *****************/
#define  USB_ADDR2_TX_ADDR2_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 2 */


/*****************  Bit definition for USB_ADDR3_TX register  *****************/
#define  USB_ADDR3_TX_ADDR3_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 3 */


/*****************  Bit definition for USB_ADDR4_TX register  *****************/
#define  USB_ADDR4_TX_ADDR4_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 4 */


/*****************  Bit definition for USB_ADDR5_TX register  *****************/
#define  USB_ADDR5_TX_ADDR5_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 5 */


/*****************  Bit definition for USB_ADDR6_TX register  *****************/
#define  USB_ADDR6_TX_ADDR6_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 6 */


/*****************  Bit definition for USB_ADDR7_TX register  *****************/
#define  USB_ADDR7_TX_ADDR7_TX               ((u16)0xFFFE)            /* Transmission Buffer Address 7 */


/*----------------------------------------------------------------------------*/


/*****************  Bit definition for USB_COUNT0_TX register  ****************/
#define  USB_COUNT0_TX_COUNT0_TX             ((u16)0x03FF)            /* Transmission Byte Count 0 */


/*****************  Bit definition for USB_COUNT1_TX register  ****************/
#define  USB_COUNT1_TX_COUNT1_TX             ((u16)0x03FF)            /* Transmission Byte Count 1 */


/*****************  Bit definition for USB_COUNT2_TX register  ****************/
#define  USB_COUNT2_TX_COUNT2_TX             ((u16)0x03FF)            /* Transmission Byte Count 2 */


/*****************  Bit definition for USB_COUNT3_TX register  ****************/
#define  USB_COUNT3_TX_COUNT3_TX             ((u16)0x03FF)            /* Transmission Byte Count 3 */


/*****************  Bit definition for USB_COUNT4_TX register  ****************/
#define  USB_COUNT4_TX_COUNT4_TX             ((u16)0x03FF)            /* Transmission Byte Count 4 */

/*****************  Bit definition for USB_COUNT5_TX register  ****************/
#define  USB_COUNT5_TX_COUNT5_TX             ((u16)0x03FF)            /* Transmission Byte Count 5 */


/*****************  Bit definition for USB_COUNT6_TX register  ****************/
#define  USB_COUNT6_TX_COUNT6_TX             ((u16)0x03FF)            /* Transmission Byte Count 6 */


/*****************  Bit definition for USB_COUNT7_TX register  ****************/
#define  USB_COUNT7_TX_COUNT7_TX             ((u16)0x03FF)            /* Transmission Byte Count 7 */


/*----------------------------------------------------------------------------*/


/****************  Bit definition for USB_COUNT0_TX_0 register  ***************/
#define  USB_COUNT0_TX_0_COUNT0_TX_0         ((u32)0x000003FF)        /* Transmission Byte Count 0 (low) */

/****************  Bit definition for USB_COUNT0_TX_1 register  ***************/
#define  USB_COUNT0_TX_1_COUNT0_TX_1         ((u32)0x03FF0000)        /* Transmission Byte Count 0 (high) */



/****************  Bit definition for USB_COUNT1_TX_0 register  ***************/
#define  USB_COUNT1_TX_0_COUNT1_TX_0          ((u32)0x000003FF)        /* Transmission Byte Count 1 (low) */

/****************  Bit definition for USB_COUNT1_TX_1 register  ***************/
#define  USB_COUNT1_TX_1_COUNT1_TX_1          ((u32)0x03FF0000)        /* Transmission Byte Count 1 (high) */



/****************  Bit definition for USB_COUNT2_TX_0 register  ***************/
#define  USB_COUNT2_TX_0_COUNT2_TX_0         ((u32)0x000003FF)        /* Transmission Byte Count 2 (low) */

/****************  Bit definition for USB_COUNT2_TX_1 register  ***************/
#define  USB_COUNT2_TX_1_COUNT2_TX_1         ((u32)0x03FF0000)        /* Transmission Byte Count 2 (high) */



/****************  Bit definition for USB_COUNT3_TX_0 register  ***************/
#define  USB_COUNT3_TX_0_COUNT3_TX_0         ((u16)0x000003FF)        /* Transmission Byte Count 3 (low) */

/****************  Bit definition for USB_COUNT3_TX_1 register  ***************/
#define  USB_COUNT3_TX_1_COUNT3_TX_1         ((u16)0x03FF0000)        /* Transmission Byte Count 3 (high) */



/****************  Bit definition for USB_COUNT4_TX_0 register  ***************/
#define  USB_COUNT4_TX_0_COUNT4_TX_0         ((u32)0x000003FF)        /* Transmission Byte Count 4 (low) */

/****************  Bit definition for USB_COUNT4_TX_1 register  ***************/
#define  USB_COUNT4_TX_1_COUNT4_TX_1         ((u32)0x03FF0000)        /* Transmission Byte Count 4 (high) */



/****************  Bit definition for USB_COUNT5_TX_0 register  ***************/
#define  USB_COUNT5_TX_0_COUNT5_TX_0         ((u32)0x000003FF)        /* Transmission Byte Count 5 (low) */

/****************  Bit definition for USB_COUNT5_TX_1 register  ***************/
#define  USB_COUNT5_TX_1_COUNT5_TX_1         ((u32)0x03FF0000)        /* Transmission Byte Count 5 (high) */



/****************  Bit definition for USB_COUNT6_TX_0 register  ***************/
#define  USB_COUNT6_TX_0_COUNT6_TX_0         ((u32)0x000003FF)        /* Transmission Byte Count 6 (low) */

/****************  Bit definition for USB_COUNT6_TX_1 register  ***************/
#define  USB_COUNT6_TX_1_COUNT6_TX_1         ((u32)0x03FF0000)        /* Transmission Byte Count 6 (high) */



/****************  Bit definition for USB_COUNT7_TX_0 register  ***************/
#define  USB_COUNT7_TX_0_COUNT7_TX_0         ((u32)0x000003FF)        /* Transmission Byte Count 7 (low) */

/****************  Bit definition for USB_COUNT7_TX_1 register  ***************/
#define  USB_COUNT7_TX_1_COUNT7_TX_1         ((u32)0x03FF0000)        /* Transmission Byte Count 7 (high) */


/*----------------------------------------------------------------------------*/


/*****************  Bit definition for USB_ADDR0_RX register  *****************/
#define  USB_ADDR0_RX_ADDR0_RX               ((u16)0xFFFE)            /* Reception Buffer Address 0 */


/*****************  Bit definition for USB_ADDR1_RX register  *****************/
#define  USB_ADDR1_RX_ADDR1_RX               ((u16)0xFFFE)            /* Reception Buffer Address 1 */


/*****************  Bit definition for USB_ADDR2_RX register  *****************/
#define  USB_ADDR2_RX_ADDR2_RX               ((u16)0xFFFE)            /* Reception Buffer Address 2 */


/*****************  Bit definition for USB_ADDR3_RX register  *****************/
#define  USB_ADDR3_RX_ADDR3_RX               ((u16)0xFFFE)            /* Reception Buffer Address 3 */


/*****************  Bit definition for USB_ADDR4_RX register  *****************/
#define  USB_ADDR4_RX_ADDR4_RX               ((u16)0xFFFE)            /* Reception Buffer Address 4 */


/*****************  Bit definition for USB_ADDR5_RX register  *****************/
#define  USB_ADDR5_RX_ADDR5_RX               ((u16)0xFFFE)            /* Reception Buffer Address 5 */


/*****************  Bit definition for USB_ADDR6_RX register  *****************/
#define  USB_ADDR6_RX_ADDR6_RX               ((u16)0xFFFE)            /* Reception Buffer Address 6 */


/*****************  Bit definition for USB_ADDR7_RX register  *****************/
#define  USB_ADDR7_RX_ADDR7_RX               ((u16)0xFFFE)            /* Reception Buffer Address 7 */


/*----------------------------------------------------------------------------*/


/*****************  Bit definition for USB_COUNT0_RX register  ****************/
#define  USB_COUNT0_RX_COUNT0_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT0_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT0_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT0_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT0_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT0_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT0_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT0_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*****************  Bit definition for USB_COUNT1_RX register  ****************/
#define  USB_COUNT1_RX_COUNT1_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT1_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT1_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT1_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT1_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT1_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT1_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT1_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*****************  Bit definition for USB_COUNT2_RX register  ****************/
#define  USB_COUNT2_RX_COUNT2_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT2_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT2_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT2_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT2_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT2_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT2_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT2_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*****************  Bit definition for USB_COUNT3_RX register  ****************/
#define  USB_COUNT3_RX_COUNT3_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT3_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT3_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT3_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT3_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT3_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT3_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT3_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*****************  Bit definition for USB_COUNT4_RX register  ****************/
#define  USB_COUNT4_RX_COUNT4_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT4_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT4_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT4_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT4_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT4_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT4_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT4_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*****************  Bit definition for USB_COUNT5_RX register  ****************/
#define  USB_COUNT5_RX_COUNT5_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT5_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT5_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT5_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT5_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT5_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT5_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT5_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */

/*****************  Bit definition for USB_COUNT6_RX register  ****************/
#define  USB_COUNT6_RX_COUNT6_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT6_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT6_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT6_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT6_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT6_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT6_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT6_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*****************  Bit definition for USB_COUNT7_RX register  ****************/
#define  USB_COUNT7_RX_COUNT7_RX             ((u16)0x03FF)            /* Reception Byte Count */

#define  USB_COUNT7_RX_NUM_BLOCK             ((u16)0x7C00)            /* NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_COUNT7_RX_NUM_BLOCK_0           ((u16)0x0400)            /* Bit 0 */
#define  USB_COUNT7_RX_NUM_BLOCK_1           ((u16)0x0800)            /* Bit 1 */
#define  USB_COUNT7_RX_NUM_BLOCK_2           ((u16)0x1000)            /* Bit 2 */
#define  USB_COUNT7_RX_NUM_BLOCK_3           ((u16)0x2000)            /* Bit 3 */
#define  USB_COUNT7_RX_NUM_BLOCK_4           ((u16)0x4000)            /* Bit 4 */

#define  USB_COUNT7_RX_BLSIZE                ((u16)0x8000)            /* BLock SIZE */


/*----------------------------------------------------------------------------*/


/****************  Bit definition for USB_COUNT0_RX_0 register  ***************/
#define  USB_COUNT0_RX_0_COUNT0_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT0_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT0_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT0_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT0_RX_1 register  ***************/
#define  USB_COUNT0_RX_1_COUNT0_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT0_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 1 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT0_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT0_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/****************  Bit definition for USB_COUNT1_RX_0 register  ***************/
#define  USB_COUNT1_RX_0_COUNT1_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT1_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT1_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT1_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT1_RX_1 register  ***************/
#define  USB_COUNT1_RX_1_COUNT1_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT1_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT1_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT1_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/****************  Bit definition for USB_COUNT2_RX_0 register  ***************/
#define  USB_COUNT2_RX_0_COUNT2_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT2_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT2_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT2_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT2_RX_1 register  ***************/
#define  USB_COUNT2_RX_1_COUNT2_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT2_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT2_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT2_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/****************  Bit definition for USB_COUNT3_RX_0 register  ***************/
#define  USB_COUNT3_RX_0_COUNT3_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT3_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT3_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT3_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT3_RX_1 register  ***************/
#define  USB_COUNT3_RX_1_COUNT3_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT3_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT3_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT3_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/****************  Bit definition for USB_COUNT4_RX_0 register  ***************/
#define  USB_COUNT4_RX_0_COUNT4_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT4_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_0      ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_1      ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_2      ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_3      ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT4_RX_0_NUM_BLOCK_0_4      ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT4_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT4_RX_1 register  ***************/
#define  USB_COUNT4_RX_1_COUNT4_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT4_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT4_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT4_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/****************  Bit definition for USB_COUNT5_RX_0 register  ***************/
#define  USB_COUNT5_RX_0_COUNT5_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT5_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT5_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT5_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT5_RX_1 register  ***************/
#define  USB_COUNT5_RX_1_COUNT5_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT5_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT5_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT5_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/***************  Bit definition for USB_COUNT6_RX_0  register  ***************/
#define  USB_COUNT6_RX_0_COUNT6_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT6_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT6_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT6_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/****************  Bit definition for USB_COUNT6_RX_1 register  ***************/
#define  USB_COUNT6_RX_1_COUNT6_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT6_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT6_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT6_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/***************  Bit definition for USB_COUNT7_RX_0 register  ****************/
#define  USB_COUNT7_RX_0_COUNT7_RX_0         ((u32)0x000003FF)        /* Reception Byte Count (low) */

#define  USB_COUNT7_RX_0_NUM_BLOCK_0         ((u32)0x00007C00)        /* NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_0       ((u32)0x00000400)        /* Bit 0 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_1       ((u32)0x00000800)        /* Bit 1 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_2       ((u32)0x00001000)        /* Bit 2 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_3       ((u32)0x00002000)        /* Bit 3 */
#define  USB_COUNT7_RX_0_NUM_BLOCK_0_4       ((u32)0x00004000)        /* Bit 4 */

#define  USB_COUNT7_RX_0_BLSIZE_0            ((u32)0x00008000)        /* BLock SIZE (low) */

/***************  Bit definition for USB_COUNT7_RX_1 register  ****************/
#define  USB_COUNT7_RX_1_COUNT7_RX_1         ((u32)0x03FF0000)        /* Reception Byte Count (high) */

#define  USB_COUNT7_RX_1_NUM_BLOCK_1         ((u32)0x7C000000)        /* NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_0       ((u32)0x04000000)        /* Bit 0 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_1       ((u32)0x08000000)        /* Bit 1 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_2       ((u32)0x10000000)        /* Bit 2 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_3       ((u32)0x20000000)        /* Bit 3 */
#define  USB_COUNT7_RX_1_NUM_BLOCK_1_4       ((u32)0x40000000)        /* Bit 4 */

#define  USB_COUNT7_RX_1_BLSIZE_1            ((u32)0x80000000)        /* BLock SIZE (high) */



/******************************************************************************/
/*                                                                            */
/*                          Controller Area Network                           */
/*                                                                            */
/******************************************************************************/

/* CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        ((u16)0x0001)            /* Initialization Request */
#define  CAN_MCR_SLEEP                       ((u16)0x0002)            /* Sleep Mode Request */
#define  CAN_MCR_TXFP                        ((u16)0x0004)            /* Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        ((u16)0x0008)            /* Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        ((u16)0x0010)            /* No Automatic Retransmission */
#define  CAN_MCR_AWUM                        ((u16)0x0020)            /* Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        ((u16)0x0040)            /* Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        ((u16)0x0080)            /* Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       ((u16)0x8000)            /* bxCAN software master reset */


/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((u16)0x0001)            /* Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((u16)0x0002)            /* Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((u16)0x0004)            /* Error Interrupt */
#define  CAN_MSR_WKUI                        ((u16)0x0008)            /* Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((u16)0x0010)            /* Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((u16)0x0100)            /* Transmit Mode */
#define  CAN_MSR_RXM                         ((u16)0x0200)            /* Receive Mode */
#define  CAN_MSR_SAMP                        ((u16)0x0400)            /* Last Sample Point */
#define  CAN_MSR_RX                          ((u16)0x0800)            /* CAN Rx Signal */


/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       ((u32)0x00000001)        /* Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       ((u32)0x00000002)        /* Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       ((u32)0x00000004)        /* Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       ((u32)0x00000008)        /* Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       ((u32)0x00000080)        /* Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       ((u32)0x00000100)        /* Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       ((u32)0x00000200)        /* Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       ((u32)0x00000400)        /* Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       ((u32)0x00000800)        /* Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       ((u32)0x00008000)        /* Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       ((u32)0x00010000)        /* Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       ((u32)0x00020000)        /* Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       ((u32)0x00040000)        /* Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       ((u32)0x00080000)        /* Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       ((u32)0x00800000)        /* Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        ((u32)0x03000000)        /* Mailbox Code */

#define  CAN_TSR_TME                         ((u32)0x1C000000)        /* TME[2:0] bits */
#define  CAN_TSR_TME0                        ((u32)0x04000000)        /* Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        ((u32)0x08000000)        /* Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        ((u32)0x10000000)        /* Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         ((u32)0xE0000000)        /* LOW[2:0] bits */
#define  CAN_TSR_LOW0                        ((u32)0x20000000)        /* Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        ((u32)0x40000000)        /* Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        ((u32)0x80000000)        /* Lowest Priority Flag for Mailbox 2 */


/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       ((u8)0x03)               /* FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((u8)0x08)               /* FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((u8)0x10)               /* FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((u8)0x20)               /* Release FIFO 0 Output Mailbox */


/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((u8)0x03)               /* FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((u8)0x08)               /* FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((u8)0x10)               /* FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((u8)0x20)               /* Release FIFO 1 Output Mailbox */


/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       ((u32)0x00000001)        /* Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      ((u32)0x00000002)        /* FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       ((u32)0x00000004)        /* FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      ((u32)0x00000008)        /* FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      ((u32)0x00000010)        /* FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       ((u32)0x00000020)        /* FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      ((u32)0x00000040)        /* FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       ((u32)0x00000100)        /* Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       ((u32)0x00000200)        /* Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       ((u32)0x00000400)        /* Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       ((u32)0x00000800)        /* Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       ((u32)0x00008000)        /* Error Interrupt Enable */
#define  CAN_IER_WKUIE                       ((u32)0x00010000)        /* Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       ((u32)0x00020000)        /* Sleep Interrupt Enable */


/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        ((u32)0x00000001)        /* Error Warning Flag */
#define  CAN_ESR_EPVF                        ((u32)0x00000002)        /* Error Passive Flag */
#define  CAN_ESR_BOFF                        ((u32)0x00000004)        /* Bus-Off Flag */

#define  CAN_ESR_LEC                         ((u32)0x00000070)        /* LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       ((u32)0x00000010)        /* Bit 0 */
#define  CAN_ESR_LEC_1                       ((u32)0x00000020)        /* Bit 1 */
#define  CAN_ESR_LEC_2                       ((u32)0x00000040)        /* Bit 2 */

#define  CAN_ESR_TEC                         ((u32)0x00FF0000)        /* Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         ((u32)0xFF000000)        /* Receive Error Counter */


/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         ((u32)0x000003FF)        /* Baud Rate Prescaler */
#define  CAN_BTR_TS1                         ((u32)0x000F0000)        /* Time Segment 1 */
#define  CAN_BTR_TS2                         ((u32)0x00700000)        /* Time Segment 2 */
#define  CAN_BTR_SJW                         ((u32)0x03000000)        /* Resynchronization Jump Width */
#define  CAN_BTR_LBKM                        ((u32)0x40000000)        /* Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        ((u32)0x80000000)        /* Silent Mode */


/* Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       ((u32)0x00000001)        /* Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        ((u32)0x00000002)        /* Remote Transmission Request */
#define  CAN_TI0R_IDE                        ((u32)0x00000004)        /* Identifier Extension */
#define  CAN_TI0R_EXID                       ((u32)0x001FFFF8)        /* Extended Identifier */
#define  CAN_TI0R_STID                       ((u32)0xFFE00000)        /* Standard Identifier or Extended Identifier */


/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       ((u32)0x0000000F)        /* Data Length Code */
#define  CAN_TDT0R_TGT                       ((u32)0x00000100)        /* Transmit Global Time */
#define  CAN_TDT0R_TIME                      ((u32)0xFFFF0000)        /* Message Time Stamp */


/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     ((u32)0x000000FF)        /* Data byte 0 */
#define  CAN_TDL0R_DATA1                     ((u32)0x0000FF00)        /* Data byte 1 */
#define  CAN_TDL0R_DATA2                     ((u32)0x00FF0000)        /* Data byte 2 */
#define  CAN_TDL0R_DATA3                     ((u32)0xFF000000)        /* Data byte 3 */


/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     ((u32)0x000000FF)        /* Data byte 4 */
#define  CAN_TDH0R_DATA5                     ((u32)0x0000FF00)        /* Data byte 5 */
#define  CAN_TDH0R_DATA6                     ((u32)0x00FF0000)        /* Data byte 6 */
#define  CAN_TDH0R_DATA7                     ((u32)0xFF000000)        /* Data byte 7 */


/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       ((u32)0x00000001)        /* Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        ((u32)0x00000002)        /* Remote Transmission Request */
#define  CAN_TI1R_IDE                        ((u32)0x00000004)        /* Identifier Extension */
#define  CAN_TI1R_EXID                       ((u32)0x001FFFF8)        /* Extended Identifier */
#define  CAN_TI1R_STID                       ((u32)0xFFE00000)        /* Standard Identifier or Extended Identifier */


/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       ((u32)0x0000000F)        /* Data Length Code */
#define  CAN_TDT1R_TGT                       ((u32)0x00000100)        /* Transmit Global Time */
#define  CAN_TDT1R_TIME                      ((u32)0xFFFF0000)        /* Message Time Stamp */


/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     ((u32)0x000000FF)        /* Data byte 0 */
#define  CAN_TDL1R_DATA1                     ((u32)0x0000FF00)        /* Data byte 1 */
#define  CAN_TDL1R_DATA2                     ((u32)0x00FF0000)        /* Data byte 2 */
#define  CAN_TDL1R_DATA3                     ((u32)0xFF000000)        /* Data byte 3 */


/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     ((u32)0x000000FF)        /* Data byte 4 */
#define  CAN_TDH1R_DATA5                     ((u32)0x0000FF00)        /* Data byte 5 */
#define  CAN_TDH1R_DATA6                     ((u32)0x00FF0000)        /* Data byte 6 */
#define  CAN_TDH1R_DATA7                     ((u32)0xFF000000)        /* Data byte 7 */


/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       ((u32)0x00000001)        /* Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        ((u32)0x00000002)        /* Remote Transmission Request */
#define  CAN_TI2R_IDE                        ((u32)0x00000004)        /* Identifier Extension */
#define  CAN_TI2R_EXID                       ((u32)0x001FFFF8)        /* Extended identifier */
#define  CAN_TI2R_STID                       ((u32)0xFFE00000)        /* Standard Identifier or Extended Identifier */


/*******************  Bit definition for CAN_TDT2R register  ******************/  
#define  CAN_TDT2R_DLC                       ((u32)0x0000000F)        /* Data Length Code */
#define  CAN_TDT2R_TGT                       ((u32)0x00000100)        /* Transmit Global Time */
#define  CAN_TDT2R_TIME                      ((u32)0xFFFF0000)        /* Message Time Stamp */


/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     ((u32)0x000000FF)        /* Data byte 0 */
#define  CAN_TDL2R_DATA1                     ((u32)0x0000FF00)        /* Data byte 1 */
#define  CAN_TDL2R_DATA2                     ((u32)0x00FF0000)        /* Data byte 2 */
#define  CAN_TDL2R_DATA3                     ((u32)0xFF000000)        /* Data byte 3 */


/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     ((u32)0x000000FF)        /* Data byte 4 */
#define  CAN_TDH2R_DATA5                     ((u32)0x0000FF00)        /* Data byte 5 */
#define  CAN_TDH2R_DATA6                     ((u32)0x00FF0000)        /* Data byte 6 */
#define  CAN_TDH2R_DATA7                     ((u32)0xFF000000)        /* Data byte 7 */


/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        ((u32)0x00000002)        /* Remote Transmission Request */
#define  CAN_RI0R_IDE                        ((u32)0x00000004)        /* Identifier Extension */
#define  CAN_RI0R_EXID                       ((u32)0x001FFFF8)        /* Extended Identifier */
#define  CAN_RI0R_STID                       ((u32)0xFFE00000)        /* Standard Identifier or Extended Identifier */


/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       ((u32)0x0000000F)        /* Data Length Code */
#define  CAN_RDT0R_FMI                       ((u32)0x0000FF00)        /* Filter Match Index */
#define  CAN_RDT0R_TIME                      ((u32)0xFFFF0000)        /* Message Time Stamp */


/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     ((u32)0x000000FF)        /* Data byte 0 */
#define  CAN_RDL0R_DATA1                     ((u32)0x0000FF00)        /* Data byte 1 */
#define  CAN_RDL0R_DATA2                     ((u32)0x00FF0000)        /* Data byte 2 */
#define  CAN_RDL0R_DATA3                     ((u32)0xFF000000)        /* Data byte 3 */


/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     ((u32)0x000000FF)        /* Data byte 4 */
#define  CAN_RDH0R_DATA5                     ((u32)0x0000FF00)        /* Data byte 5 */
#define  CAN_RDH0R_DATA6                     ((u32)0x00FF0000)        /* Data byte 6 */
#define  CAN_RDH0R_DATA7                     ((u32)0xFF000000)        /* Data byte 7 */


/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        ((u32)0x00000002)        /* Remote Transmission Request */
#define  CAN_RI1R_IDE                        ((u32)0x00000004)        /* Identifier Extension */
#define  CAN_RI1R_EXID                       ((u32)0x001FFFF8)        /* Extended identifier */
#define  CAN_RI1R_STID                       ((u32)0xFFE00000)        /* Standard Identifier or Extended Identifier */


/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       ((u32)0x0000000F)        /* Data Length Code */
#define  CAN_RDT1R_FMI                       ((u32)0x0000FF00)        /* Filter Match Index */
#define  CAN_RDT1R_TIME                      ((u32)0xFFFF0000)        /* Message Time Stamp */


/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     ((u32)0x000000FF)        /* Data byte 0 */
#define  CAN_RDL1R_DATA1                     ((u32)0x0000FF00)        /* Data byte 1 */
#define  CAN_RDL1R_DATA2                     ((u32)0x00FF0000)        /* Data byte 2 */
#define  CAN_RDL1R_DATA3                     ((u32)0xFF000000)        /* Data byte 3 */


/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     ((u32)0x000000FF)        /* Data byte 4 */
#define  CAN_RDH1R_DATA5                     ((u32)0x0000FF00)        /* Data byte 5 */
#define  CAN_RDH1R_DATA6                     ((u32)0x00FF0000)        /* Data byte 6 */
#define  CAN_RDH1R_DATA7                     ((u32)0xFF000000)        /* Data byte 7 */

/* CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       ((u8)0x01)               /* Filter Init Mode */


/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((u16)0x3FFF)            /* Filter Mode */
#define  CAN_FM1R_FBM0                       ((u16)0x0001)            /* Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       ((u16)0x0002)            /* Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       ((u16)0x0004)            /* Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       ((u16)0x0008)            /* Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       ((u16)0x0010)            /* Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       ((u16)0x0020)            /* Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       ((u16)0x0040)            /* Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       ((u16)0x0080)            /* Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       ((u16)0x0100)            /* Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       ((u16)0x0200)            /* Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      ((u16)0x0400)            /* Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      ((u16)0x0800)            /* Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      ((u16)0x1000)            /* Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      ((u16)0x2000)            /* Filter Init Mode bit 13 */


/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((u16)0x3FFF)            /* Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       ((u16)0x0001)            /* Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       ((u16)0x0002)            /* Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       ((u16)0x0004)            /* Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       ((u16)0x0008)            /* Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       ((u16)0x0010)            /* Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       ((u16)0x0020)            /* Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       ((u16)0x0040)            /* Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       ((u16)0x0080)            /* Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       ((u16)0x0100)            /* Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       ((u16)0x0200)            /* Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      ((u16)0x0400)            /* Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      ((u16)0x0800)            /* Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      ((u16)0x1000)            /* Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      ((u16)0x2000)            /* Filter Scale Configuration bit 13 */


/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       ((u16)0x3FFF)            /* Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      ((u16)0x0001)            /* Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      ((u16)0x0002)            /* Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      ((u16)0x0004)            /* Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      ((u16)0x0008)            /* Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      ((u16)0x0010)            /* Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      ((u16)0x0020)            /* Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      ((u16)0x0040)            /* Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      ((u16)0x0080)            /* Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      ((u16)0x0100)            /* Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      ((u16)0x0200)            /* Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     ((u16)0x0400)            /* Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     ((u16)0x0800)            /* Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     ((u16)0x1000)            /* Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     ((u16)0x2000)            /* Filter FIFO Assignment for Filter 13 */


/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       ((u16)0x3FFF)            /* Filter Active */
#define  CAN_FA1R_FACT0                      ((u16)0x0001)            /* Filter 0 Active */
#define  CAN_FA1R_FACT1                      ((u16)0x0002)            /* Filter 1 Active */
#define  CAN_FA1R_FACT2                      ((u16)0x0004)            /* Filter 2 Active */
#define  CAN_FA1R_FACT3                      ((u16)0x0008)            /* Filter 3 Active */
#define  CAN_FA1R_FACT4                      ((u16)0x0010)            /* Filter 4 Active */
#define  CAN_FA1R_FACT5                      ((u16)0x0020)            /* Filter 5 Active */
#define  CAN_FA1R_FACT6                      ((u16)0x0040)            /* Filter 6 Active */
#define  CAN_FA1R_FACT7                      ((u16)0x0080)            /* Filter 7 Active */
#define  CAN_FA1R_FACT8                      ((u16)0x0100)            /* Filter 8 Active */
#define  CAN_FA1R_FACT9                      ((u16)0x0200)            /* Filter 9 Active */
#define  CAN_FA1R_FACT10                     ((u16)0x0400)            /* Filter 10 Active */
#define  CAN_FA1R_FACT11                     ((u16)0x0800)            /* Filter 11 Active */
#define  CAN_FA1R_FACT12                     ((u16)0x1000)            /* Filter 12 Active */
#define  CAN_FA1R_FACT13                     ((u16)0x2000)            /* Filter 13 Active */


/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F0R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F0R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F0R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F0R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F0R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F0R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F0R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F0R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F0R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F0R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F0R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F0R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F0R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F0R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F0R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F0R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F0R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F0R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F0R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F0R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F0R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F0R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F0R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F0R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F0R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F0R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F0R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F0R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F0R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F0R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F0R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F1R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F1R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F1R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F1R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F1R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F1R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F1R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F1R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F1R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F1R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F1R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F1R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F1R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F1R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F1R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F1R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F1R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F1R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F1R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F1R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F1R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F1R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F1R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F1R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F1R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F1R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F1R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F1R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F1R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F1R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F1R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F2R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F2R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F2R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F2R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F2R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F2R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F2R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F2R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F2R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F2R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F2R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F2R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F2R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F2R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F2R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F2R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F2R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F2R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F2R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F2R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F2R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F2R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F2R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F2R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F2R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F2R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F2R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F2R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F2R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F2R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F2R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F3R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F3R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F3R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F3R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F3R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F3R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F3R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F3R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F3R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F3R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F3R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F3R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F3R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F3R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F3R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F3R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F3R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F3R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F3R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F3R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F3R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F3R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F3R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F3R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F3R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F3R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F3R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F3R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F3R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F3R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F3R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F4R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F4R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F4R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F4R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F4R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F4R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F4R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F4R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F4R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F4R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F4R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F4R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F4R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F4R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F4R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F4R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F4R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F4R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F4R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F4R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F4R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F4R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F4R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F4R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F4R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F4R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F4R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F4R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F4R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F4R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F4R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F5R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F5R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F5R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F5R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F5R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F5R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F5R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F5R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F5R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F5R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F5R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F5R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F5R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F5R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F5R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F5R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F5R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F5R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F5R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F5R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F5R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F5R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F5R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F5R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F5R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F5R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F5R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F5R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F5R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F5R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F5R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F6R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F6R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F6R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F6R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F6R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F6R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F6R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F6R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F6R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F6R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F6R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F6R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F6R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F6R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F6R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F6R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F6R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F6R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F6R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F6R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F6R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F6R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F6R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F6R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F6R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F6R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F6R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F6R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F6R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F6R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F6R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F7R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F7R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F7R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F7R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F7R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F7R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F7R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F7R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F7R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F7R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F7R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F7R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F7R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F7R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F7R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F7R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F7R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F7R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F7R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F7R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F7R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F7R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F7R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F7R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F7R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F7R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F7R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F7R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F7R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F7R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F7R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F8R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F8R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F8R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F8R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F8R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F8R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F8R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F8R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F8R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F8R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F8R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F8R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F8R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F8R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F8R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F8R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F8R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F8R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F8R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F8R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F8R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F8R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F8R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F8R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F8R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F8R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F8R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F8R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F8R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F8R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F8R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F9R1_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F9R1_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F9R1_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F9R1_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F9R1_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F9R1_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F9R1_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F9R1_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F9R1_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F9R1_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F9R1_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F9R1_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F9R1_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F9R1_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F9R1_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F9R1_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F9R1_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F9R1_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F9R1_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F9R1_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F9R1_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F9R1_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F9R1_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F9R1_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F9R1_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F9R1_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F9R1_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F9R1_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F9R1_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F9R1_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F9R1_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F10R1_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F10R1_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F10R1_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F10R1_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F10R1_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F10R1_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F10R1_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F10R1_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F10R1_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F10R1_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F10R1_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F10R1_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F10R1_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F10R1_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F10R1_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F10R1_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F10R1_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F10R1_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F10R1_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F10R1_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F10R1_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F10R1_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F10R1_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F10R1_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F10R1_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F10R1_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F10R1_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F10R1_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F10R1_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F10R1_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F10R1_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F11R1_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F11R1_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F11R1_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F11R1_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F11R1_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F11R1_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F11R1_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F11R1_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F11R1_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F11R1_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F11R1_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F11R1_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F11R1_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F11R1_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F11R1_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F11R1_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F11R1_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F11R1_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F11R1_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F11R1_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F11R1_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F11R1_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F11R1_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F11R1_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F11R1_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F11R1_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F11R1_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F11R1_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F11R1_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F11R1_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F11R1_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F12R1_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F12R1_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F12R1_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F12R1_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F12R1_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F12R1_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F12R1_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F12R1_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F12R1_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F12R1_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F12R1_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F12R1_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F12R1_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F12R1_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F12R1_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F12R1_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F12R1_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F12R1_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F12R1_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F12R1_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F12R1_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F12R1_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F12R1_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F12R1_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F12R1_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F12R1_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F12R1_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F12R1_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F12R1_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F12R1_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F12R1_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F13R1_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F13R1_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F13R1_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F13R1_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F13R1_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F13R1_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F13R1_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F13R1_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F13R1_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F13R1_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F13R1_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F13R1_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F13R1_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F13R1_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F13R1_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F13R1_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F13R1_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F13R1_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F13R1_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F13R1_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F13R1_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F13R1_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F13R1_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F13R1_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F13R1_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F13R1_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F13R1_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F13R1_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F13R1_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F13R1_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F13R1_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F0R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F0R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F0R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F0R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F0R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F0R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F0R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F0R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F0R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F0R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F0R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F0R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F0R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F0R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F0R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F0R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F0R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F0R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F0R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F0R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F0R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F0R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F0R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F0R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F0R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F0R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F0R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F0R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F0R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F0R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F0R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F1R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F1R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F1R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F1R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F1R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F1R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F1R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F1R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F1R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F1R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F1R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F1R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F1R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F1R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F1R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F1R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F1R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F1R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F1R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F1R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F1R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F1R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F1R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F1R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F1R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F1R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F1R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F1R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F1R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F1R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F1R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F2R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F2R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F2R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F2R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F2R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F2R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F2R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F2R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F2R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F2R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F2R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F2R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F2R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F2R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F2R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F2R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F2R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F2R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F2R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F2R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F2R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F2R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F2R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F2R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F2R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F2R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F2R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F2R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F2R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F2R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F2R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F3R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F3R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F3R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F3R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F3R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F3R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F3R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F3R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F3R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F3R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F3R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F3R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F3R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F3R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F3R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F3R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F3R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F3R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F3R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F3R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F3R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F3R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F3R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F3R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F3R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F3R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F3R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F3R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F3R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F3R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F3R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F4R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F4R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F4R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F4R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F4R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F4R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F4R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F4R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F4R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F4R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F4R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F4R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F4R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F4R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F4R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F4R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F4R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F4R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F4R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F4R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F4R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F4R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F4R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F4R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F4R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F4R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F4R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F4R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F4R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F4R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F4R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F5R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F5R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F5R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F5R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F5R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F5R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F5R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F5R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F5R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F5R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F5R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F5R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F5R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F5R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F5R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F5R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F5R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F5R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F5R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F5R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F5R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F5R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F5R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F5R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F5R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F5R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F5R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F5R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F5R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F5R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F5R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F6R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F6R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F6R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F6R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F6R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F6R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F6R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F6R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F6R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F6R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F6R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F6R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F6R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F6R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F6R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F6R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F6R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F6R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F6R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F6R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F6R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F6R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F6R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F6R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F6R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F6R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F6R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F6R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F6R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F6R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F6R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F7R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F7R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F7R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F7R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F7R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F7R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F7R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F7R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F7R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F7R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F7R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F7R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F7R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F7R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F7R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F7R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F7R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F7R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F7R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F7R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F7R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F7R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F7R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F7R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F7R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F7R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F7R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F7R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F7R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F7R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F7R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F8R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F8R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F8R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F8R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F8R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F8R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F8R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F8R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F8R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F8R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F8R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F8R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F8R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F8R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F8R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F8R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F8R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F8R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F8R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F8R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F8R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F8R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F8R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F8R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F8R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F8R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F8R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F8R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F8R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F8R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F8R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F9R2_FB1                        ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F9R2_FB2                        ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F9R2_FB3                        ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F9R2_FB4                        ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F9R2_FB5                        ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F9R2_FB6                        ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F9R2_FB7                        ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F9R2_FB8                        ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F9R2_FB9                        ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F9R2_FB10                       ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F9R2_FB11                       ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F9R2_FB12                       ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F9R2_FB13                       ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F9R2_FB14                       ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F9R2_FB15                       ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F9R2_FB16                       ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F9R2_FB17                       ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F9R2_FB18                       ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F9R2_FB19                       ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F9R2_FB20                       ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F9R2_FB21                       ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F9R2_FB22                       ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F9R2_FB23                       ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F9R2_FB24                       ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F9R2_FB25                       ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F9R2_FB26                       ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F9R2_FB27                       ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F9R2_FB28                       ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F9R2_FB29                       ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F9R2_FB30                       ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F9R2_FB31                       ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F10R2_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F10R2_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F10R2_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F10R2_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F10R2_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F10R2_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F10R2_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F10R2_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F10R2_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F10R2_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F10R2_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F10R2_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F10R2_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F10R2_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F10R2_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F10R2_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F10R2_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F10R2_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F10R2_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F10R2_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F10R2_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F10R2_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F10R2_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F10R2_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F10R2_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F10R2_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F10R2_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F10R2_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F10R2_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F10R2_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F10R2_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F11R2_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F11R2_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F11R2_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F11R2_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F11R2_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F11R2_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F11R2_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F11R2_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F11R2_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F11R2_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F11R2_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F11R2_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F11R2_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F11R2_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F11R2_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F11R2_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F11R2_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F11R2_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F11R2_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F11R2_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F11R2_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F11R2_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F11R2_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F11R2_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F11R2_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F11R2_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F11R2_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F11R2_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F11R2_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F11R2_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F11R2_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F12R2_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F12R2_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F12R2_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F12R2_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F12R2_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F12R2_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F12R2_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F12R2_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F12R2_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F12R2_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F12R2_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F12R2_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F12R2_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F12R2_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F12R2_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F12R2_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F12R2_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F12R2_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F12R2_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F12R2_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F12R2_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F12R2_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F12R2_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F12R2_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F12R2_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F12R2_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F12R2_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F12R2_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F12R2_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F12R2_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F12R2_FB31                      ((u32)0x80000000)        /* Filter bit 31 */


/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       ((u32)0x00000001)        /* Filter bit 0 */
#define  CAN_F13R2_FB1                       ((u32)0x00000002)        /* Filter bit 1 */
#define  CAN_F13R2_FB2                       ((u32)0x00000004)        /* Filter bit 2 */
#define  CAN_F13R2_FB3                       ((u32)0x00000008)        /* Filter bit 3 */
#define  CAN_F13R2_FB4                       ((u32)0x00000010)        /* Filter bit 4 */
#define  CAN_F13R2_FB5                       ((u32)0x00000020)        /* Filter bit 5 */
#define  CAN_F13R2_FB6                       ((u32)0x00000040)        /* Filter bit 6 */
#define  CAN_F13R2_FB7                       ((u32)0x00000080)        /* Filter bit 7 */
#define  CAN_F13R2_FB8                       ((u32)0x00000100)        /* Filter bit 8 */
#define  CAN_F13R2_FB9                       ((u32)0x00000200)        /* Filter bit 9 */
#define  CAN_F13R2_FB10                      ((u32)0x00000400)        /* Filter bit 10 */
#define  CAN_F13R2_FB11                      ((u32)0x00000800)        /* Filter bit 11 */
#define  CAN_F13R2_FB12                      ((u32)0x00001000)        /* Filter bit 12 */
#define  CAN_F13R2_FB13                      ((u32)0x00002000)        /* Filter bit 13 */
#define  CAN_F13R2_FB14                      ((u32)0x00004000)        /* Filter bit 14 */
#define  CAN_F13R2_FB15                      ((u32)0x00008000)        /* Filter bit 15 */
#define  CAN_F13R2_FB16                      ((u32)0x00010000)        /* Filter bit 16 */
#define  CAN_F13R2_FB17                      ((u32)0x00020000)        /* Filter bit 17 */
#define  CAN_F13R2_FB18                      ((u32)0x00040000)        /* Filter bit 18 */
#define  CAN_F13R2_FB19                      ((u32)0x00080000)        /* Filter bit 19 */
#define  CAN_F13R2_FB20                      ((u32)0x00100000)        /* Filter bit 20 */
#define  CAN_F13R2_FB21                      ((u32)0x00200000)        /* Filter bit 21 */
#define  CAN_F13R2_FB22                      ((u32)0x00400000)        /* Filter bit 22 */
#define  CAN_F13R2_FB23                      ((u32)0x00800000)        /* Filter bit 23 */
#define  CAN_F13R2_FB24                      ((u32)0x01000000)        /* Filter bit 24 */
#define  CAN_F13R2_FB25                      ((u32)0x02000000)        /* Filter bit 25 */
#define  CAN_F13R2_FB26                      ((u32)0x04000000)        /* Filter bit 26 */
#define  CAN_F13R2_FB27                      ((u32)0x08000000)        /* Filter bit 27 */
#define  CAN_F13R2_FB28                      ((u32)0x10000000)        /* Filter bit 28 */
#define  CAN_F13R2_FB29                      ((u32)0x20000000)        /* Filter bit 29 */
#define  CAN_F13R2_FB30                      ((u32)0x40000000)        /* Filter bit 30 */
#define  CAN_F13R2_FB31                      ((u32)0x80000000)        /* Filter bit 31 */



/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((u16)0x0001)            /* Clock Phase */
#define  SPI_CR1_CPOL                        ((u16)0x0002)            /* Clock Polarity */
#define  SPI_CR1_MSTR                        ((u16)0x0004)            /* Master Selection */

#define  SPI_CR1_BR                          ((u16)0x0038)            /* BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((u16)0x0008)            /* Bit 0 */
#define  SPI_CR1_BR_1                        ((u16)0x0010)            /* Bit 1 */
#define  SPI_CR1_BR_2                        ((u16)0x0020)            /* Bit 2 */

#define  SPI_CR1_SPE                         ((u16)0x0040)            /* SPI Enable */
#define  SPI_CR1_LSBFIRST                    ((u16)0x0080)            /* Frame Format */
#define  SPI_CR1_SSI                         ((u16)0x0100)            /* Internal slave select */
#define  SPI_CR1_SSM                         ((u16)0x0200)            /* Software slave management */
#define  SPI_CR1_RXONLY                      ((u16)0x0400)            /* Receive only */
#define  SPI_CR1_DFF                         ((u16)0x0800)            /* Data Frame Format */
#define  SPI_CR1_CRCNEXT                     ((u16)0x1000)            /* Transmit CRC next */
#define  SPI_CR1_CRCEN                       ((u16)0x2000)            /* Hardware CRC calculation enable */
#define  SPI_CR1_BIDIOE                      ((u16)0x4000)            /* Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((u16)0x8000)            /* Bidirectional data mode enable */


/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((u8)0x01)               /* Rx Buffer DMA Enable */
#define  SPI_CR2_TXDMAEN                     ((u8)0x02)               /* Tx Buffer DMA Enable */
#define  SPI_CR2_SSOE                        ((u8)0x04)               /* SS Output Enable */
#define  SPI_CR2_ERRIE                       ((u8)0x20)               /* Error Interrupt Enable */
#define  SPI_CR2_RXNEIE                      ((u8)0x40)               /* RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((u8)0x80)               /* Tx buffer Empty Interrupt Enable */


/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((u8)0x01)               /* Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((u8)0x02)               /* Transmit buffer Empty */
#define  SPI_SR_CHSIDE                       ((u8)0x04)               /* Channel side */
#define  SPI_SR_UDR                          ((u8)0x08)               /* Underrun flag */
#define  SPI_SR_CRCERR                       ((u8)0x10)               /* CRC Error flag */
#define  SPI_SR_MODF                         ((u8)0x20)               /* Mode fault */
#define  SPI_SR_OVR                          ((u8)0x40)               /* Overrun flag */
#define  SPI_SR_BSY                          ((u8)0x80)               /* Busy flag */


/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((u16)0xFFFF)            /* Data Register */


/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((u16)0xFFFF)            /* CRC polynomial register */


/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((u16)0xFFFF)            /* Rx CRC Register */


/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((u16)0xFFFF)            /* Tx CRC Register */


/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((u16)0x0001)            /* Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  ((u16)0x0006)            /* DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCFGR_DATLEN_0                ((u16)0x0002)            /* Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((u16)0x0004)            /* Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   ((u16)0x0008)            /* steady state clock polarity */

#define  SPI_I2SCFGR_I2SSTD                  ((u16)0x0030)            /* I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((u16)0x0010)            /* Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((u16)0x0020)            /* Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 ((u16)0x0080)            /* PCM frame synchronization */

#define  SPI_I2SCFGR_I2SCFG                  ((u16)0x0300)            /* I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((u16)0x0100)            /* Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((u16)0x0200)            /* Bit 1 */

#define  SPI_I2SCFGR_I2SE                    ((u16)0x0400)            /* I2S Enable */
#define  SPI_I2SCFGR_I2SMOD                  ((u16)0x0800)            /* I2S mode selection */


/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((u16)0x00FF)            /* I2S Linear prescaler */
#define  SPI_I2SPR_ODD                       ((u16)0x0100)            /* Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((u16)0x0200)            /* Master Clock Output Enable */



/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          ((u16)0x0001)            /* Peripheral Enable */
#define  I2C_CR1_SMBUS                       ((u16)0x0002)            /* SMBus Mode */
#define  I2C_CR1_SMBTYPE                     ((u16)0x0008)            /* SMBus Type */
#define  I2C_CR1_ENARP                       ((u16)0x0010)            /* ARP Enable */
#define  I2C_CR1_ENPEC                       ((u16)0x0020)            /* PEC Enable */
#define  I2C_CR1_ENGC                        ((u16)0x0040)            /* General Call Enable */
#define  I2C_CR1_NOSTRETCH                   ((u16)0x0080)            /* Clock Stretching Disable (Slave mode) */
#define  I2C_CR1_START                       ((u16)0x0100)            /* Start Generation */
#define  I2C_CR1_STOP                        ((u16)0x0200)            /* Stop Generation */
#define  I2C_CR1_ACK                         ((u16)0x0400)            /* Acknowledge Enable */
#define  I2C_CR1_POS                         ((u16)0x0800)            /* Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         ((u16)0x1000)            /* Packet Error Checking */
#define  I2C_CR1_ALERT                       ((u16)0x2000)            /* SMBus Alert */
#define  I2C_CR1_SWRST                       ((u16)0x8000)            /* Software Reset */


/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        ((u16)0x003F)            /* FREQ[5:0] bits (Peripheral Clock Frequency) */
#define  I2C_CR2_FREQ_0                      ((u16)0x0001)            /* Bit 0 */
#define  I2C_CR2_FREQ_1                      ((u16)0x0002)            /* Bit 1 */
#define  I2C_CR2_FREQ_2                      ((u16)0x0004)            /* Bit 2 */
#define  I2C_CR2_FREQ_3                      ((u16)0x0008)            /* Bit 3 */
#define  I2C_CR2_FREQ_4                      ((u16)0x0010)            /* Bit 4 */
#define  I2C_CR2_FREQ_5                      ((u16)0x0020)            /* Bit 5 */

#define  I2C_CR2_ITERREN                     ((u16)0x0100)            /* Error Interrupt Enable */
#define  I2C_CR2_ITEVTEN                     ((u16)0x0200)            /* Event Interrupt Enable */
#define  I2C_CR2_ITBUFEN                     ((u16)0x0400)            /* Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       ((u16)0x0800)            /* DMA Requests Enable */
#define  I2C_CR2_LAST                        ((u16)0x1000)            /* DMA Last Transfer */


/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     ((u16)0x00FE)            /* Interface Address */
#define  I2C_OAR1_ADD8_9                     ((u16)0x0300)            /* Interface Address */

#define  I2C_OAR1_ADD0                       ((u16)0x0001)            /* Bit 0 */
#define  I2C_OAR1_ADD1                       ((u16)0x0002)            /* Bit 1 */
#define  I2C_OAR1_ADD2                       ((u16)0x0004)            /* Bit 2 */
#define  I2C_OAR1_ADD3                       ((u16)0x0008)            /* Bit 3 */
#define  I2C_OAR1_ADD4                       ((u16)0x0010)            /* Bit 4 */
#define  I2C_OAR1_ADD5                       ((u16)0x0020)            /* Bit 5 */
#define  I2C_OAR1_ADD6                       ((u16)0x0040)            /* Bit 6 */
#define  I2C_OAR1_ADD7                       ((u16)0x0080)            /* Bit 7 */
#define  I2C_OAR1_ADD8                       ((u16)0x0100)            /* Bit 8 */
#define  I2C_OAR1_ADD9                       ((u16)0x0200)            /* Bit 9 */

#define  I2C_OAR1_ADDMODE                    ((u16)0x8000)            /* Addressing Mode (Slave mode) */


/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     ((u8)0x01)               /* Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       ((u8)0xFE)               /* Interface address */


/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           ((u8)0xFF)               /* 8-bit Data Register */


/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          ((u16)0x0001)            /* Start Bit (Master mode) */
#define  I2C_SR1_ADDR                        ((u16)0x0002)            /* Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         ((u16)0x0004)            /* Byte Transfer Finished */
#define  I2C_SR1_ADD10                       ((u16)0x0008)            /* 10-bit header sent (Master mode) */
#define  I2C_SR1_STOPF                       ((u16)0x0010)            /* Stop detection (Slave mode) */
#define  I2C_SR1_RXNE                        ((u16)0x0040)            /* Data Register not Empty (receivers) */
#define  I2C_SR1_TXE                         ((u16)0x0080)            /* Data Register Empty (transmitters) */
#define  I2C_SR1_BERR                        ((u16)0x0100)            /* Bus Error */
#define  I2C_SR1_ARLO                        ((u16)0x0200)            /* Arbitration Lost (master mode) */
#define  I2C_SR1_AF                          ((u16)0x0400)            /* Acknowledge Failure */
#define  I2C_SR1_OVR                         ((u16)0x0800)            /* Overrun/Underrun */
#define  I2C_SR1_PECERR                      ((u16)0x1000)            /* PEC Error in reception */
#define  I2C_SR1_TIMEOUT                     ((u16)0x4000)            /* Timeout or Tlow Error */
#define  I2C_SR1_SMBALERT                    ((u16)0x8000)            /* SMBus Alert */


/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         ((u16)0x0001)            /* Master/Slave */
#define  I2C_SR2_BUSY                        ((u16)0x0002)            /* Bus Busy */
#define  I2C_SR2_TRA                         ((u16)0x0004)            /* Transmitter/Receiver */
#define  I2C_SR2_GENCALL                     ((u16)0x0010)            /* General Call Address (Slave mode) */
#define  I2C_SR2_SMBDEFAULT                  ((u16)0x0020)            /* SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     ((u16)0x0040)            /* SMBus Host Header (Slave mode) */
#define  I2C_SR2_DUALF                       ((u16)0x0080)            /* Dual Flag (Slave mode) */
#define  I2C_SR2_PEC                         ((u16)0xFF00)            /* Packet Error Checking Register */


/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         ((u16)0x0FFF)            /* Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        ((u16)0x4000)            /* Fast Mode Duty Cycle */
#define  I2C_CCR_FS                          ((u16)0x8000)            /* I2C Master Mode Selection */


/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     ((u8)0x3F)               /* Maximum Rise Time in Fast/Standard mode (Master mode) */



/******************************************************************************/
/*                                                                            */
/*          Universal Synchronous Asynchronous Receiver Transmitter           */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         ((u16)0x0001)            /* Parity Error */
#define  USART_SR_FE                         ((u16)0x0002)            /* Framing Error */
#define  USART_SR_NE                         ((u16)0x0004)            /* Noise Error Flag */
#define  USART_SR_ORE                        ((u16)0x0008)            /* OverRun Error */
#define  USART_SR_IDLE                       ((u16)0x0010)            /* IDLE line detected */
#define  USART_SR_RXNE                       ((u16)0x0020)            /* Read Data Register Not Empty */
#define  USART_SR_TC                         ((u16)0x0040)            /* Transmission Complete */
#define  USART_SR_TXE                        ((u16)0x0080)            /* Transmit Data Register Empty */
#define  USART_SR_LBD                        ((u16)0x0100)            /* LIN Break Detection Flag */
#define  USART_SR_CTS                        ((u16)0x0200)            /* CTS Flag */


/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         ((u16)0x01FF)            /* Data value */


/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              ((u16)0x000F)            /* Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              ((u16)0xFFF0)            /* Mantissa of USARTDIV */


/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       ((u16)0x0001)            /* Send Break */
#define  USART_CR1_RWU                       ((u16)0x0002)            /* Receiver wakeup */
#define  USART_CR1_RE                        ((u16)0x0004)            /* Receiver Enable */
#define  USART_CR1_TE                        ((u16)0x0008)            /* Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((u16)0x0010)            /* IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((u16)0x0020)            /* RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((u16)0x0040)            /* Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((u16)0x0080)            /* PE Interrupt Enable */
#define  USART_CR1_PEIE                      ((u16)0x0100)            /* PE Interrupt Enable */
#define  USART_CR1_PS                        ((u16)0x0200)            /* Parity Selection */
#define  USART_CR1_PCE                       ((u16)0x0400)            /* Parity Control Enable */
#define  USART_CR1_WAKE                      ((u16)0x0800)            /* Wakeup method */
#define  USART_CR1_M                         ((u16)0x1000)            /* Word length */
#define  USART_CR1_UE                        ((u16)0x2000)            /* USART Enable */


/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       ((u16)0x000F)            /* Address of the USART node */
#define  USART_CR2_LBDL                      ((u16)0x0020)            /* LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((u16)0x0040)            /* LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((u16)0x0100)            /* Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((u16)0x0200)            /* Clock Phase */
#define  USART_CR2_CPOL                      ((u16)0x0400)            /* Clock Polarity */
#define  USART_CR2_CLKEN                     ((u16)0x0800)            /* Clock Enable */

#define  USART_CR2_STOP                      ((u16)0x3000)            /* STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((u16)0x1000)            /* Bit 0 */
#define  USART_CR2_STOP_1                    ((u16)0x2000)            /* Bit 1 */

#define  USART_CR2_LINEN                     ((u16)0x4000)            /* LIN mode enable */


/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((u16)0x0001)            /* Error Interrupt Enable */
#define  USART_CR3_IREN                      ((u16)0x0002)            /* IrDA mode Enable */
#define  USART_CR3_IRLP                      ((u16)0x0004)            /* IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((u16)0x0008)            /* Half-Duplex Selection */
#define  USART_CR3_NACK                      ((u16)0x0010)            /* Smartcard NACK enable */
#define  USART_CR3_SCEN                      ((u16)0x0020)            /* Smartcard mode enable */
#define  USART_CR3_DMAR                      ((u16)0x0040)            /* DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((u16)0x0080)            /* DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((u16)0x0100)            /* RTS Enable */
#define  USART_CR3_CTSE                      ((u16)0x0200)            /* CTS Enable */
#define  USART_CR3_CTSIE                     ((u16)0x0400)            /* CTS Interrupt Enable */


/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((u16)0x00FF)            /* PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    ((u16)0x0001)            /* Bit 0 */
#define  USART_GTPR_PSC_1                    ((u16)0x0002)            /* Bit 1 */
#define  USART_GTPR_PSC_2                    ((u16)0x0004)            /* Bit 2 */
#define  USART_GTPR_PSC_3                    ((u16)0x0008)            /* Bit 3 */
#define  USART_GTPR_PSC_4                    ((u16)0x0010)            /* Bit 4 */
#define  USART_GTPR_PSC_5                    ((u16)0x0020)            /* Bit 5 */
#define  USART_GTPR_PSC_6                    ((u16)0x0040)            /* Bit 6 */
#define  USART_GTPR_PSC_7                    ((u16)0x0080)            /* Bit 7 */

#define  USART_GTPR_GT                       ((u16)0xFF00)            /* Guard time value */



/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define  DBGMCU_IDCODE_DEV_ID                ((u32)0x00000FFF)        /* Device Identifier */

#define  DBGMCU_IDCODE_REV_ID                ((u32)0xFFFF0000)        /* REV_ID[15:0] bits (Revision Identifier) */
#define  DBGMCU_IDCODE_REV_ID_0              ((u32)0x00010000)        /* Bit 0 */
#define  DBGMCU_IDCODE_REV_ID_1              ((u32)0x00020000)        /* Bit 1 */
#define  DBGMCU_IDCODE_REV_ID_2              ((u32)0x00040000)        /* Bit 2 */
#define  DBGMCU_IDCODE_REV_ID_3              ((u32)0x00080000)        /* Bit 3 */
#define  DBGMCU_IDCODE_REV_ID_4              ((u32)0x00100000)        /* Bit 4 */
#define  DBGMCU_IDCODE_REV_ID_5              ((u32)0x00200000)        /* Bit 5 */
#define  DBGMCU_IDCODE_REV_ID_6              ((u32)0x00400000)        /* Bit 6 */
#define  DBGMCU_IDCODE_REV_ID_7              ((u32)0x00800000)        /* Bit 7 */
#define  DBGMCU_IDCODE_REV_ID_8              ((u32)0x01000000)        /* Bit 8 */
#define  DBGMCU_IDCODE_REV_ID_9              ((u32)0x02000000)        /* Bit 9 */
#define  DBGMCU_IDCODE_REV_ID_10             ((u32)0x04000000)        /* Bit 10 */
#define  DBGMCU_IDCODE_REV_ID_11             ((u32)0x08000000)        /* Bit 11 */
#define  DBGMCU_IDCODE_REV_ID_12             ((u32)0x10000000)        /* Bit 12 */
#define  DBGMCU_IDCODE_REV_ID_13             ((u32)0x20000000)        /* Bit 13 */
#define  DBGMCU_IDCODE_REV_ID_14             ((u32)0x40000000)        /* Bit 14 */
#define  DBGMCU_IDCODE_REV_ID_15             ((u32)0x80000000)        /* Bit 15 */


/******************  Bit definition for DBGMCU_CR register  *******************/
#define  DBGMCU_CR_DBG_SLEEP                 ((u32)0x00000001)        /* Debug Sleep Mode */
#define  DBGMCU_CR_DBG_STOP                  ((u32)0x00000002)        /* Debug Stop Mode */
#define  DBGMCU_CR_DBG_STANDBY               ((u32)0x00000004)        /* Debug Standby mode */
#define  DBGMCU_CR_TRACE_IOEN                ((u32)0x00000020)        /* Trace Pin Assignment Control */

#define  DBGMCU_CR_TRACE_MODE                ((u32)0x000000C0)        /* TRACE_MODE[1:0] bits (Trace Pin Assignment Control) */
#define  DBGMCU_CR_TRACE_MODE_0              ((u32)0x00000040)        /* Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((u32)0x00000080)        /* Bit 1 */

#define  DBGMCU_CR_DBG_IWDG_STOP             ((u32)0x00000100)        /* Debug Independent Watchdog stopped when Core is halted */
#define  DBGMCU_CR_DBG_WWDG_STOP             ((u32)0x00000200)        /* Debug Window Watchdog stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM1_STOP             ((u32)0x00000400)        /* TIM1 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM2_STOP             ((u32)0x00000800)        /* TIM2 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM3_STOP             ((u32)0x00001000)        /* TIM3 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM4_STOP             ((u32)0x00002000)        /* TIM4 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_CAN_STOP              ((u32)0x00004000)        /* Debug CAN stopped when Core is halted */
#define  DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT    ((u32)0x00008000)        /* SMBUS timeout mode stopped when Core is halted */
#define  DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT    ((u32)0x00010000)        /* SMBUS timeout mode stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM5_STOP             ((u32)0x00020000)        /* TIM5 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM6_STOP             ((u32)0x00040000)        /* TIM6 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM7_STOP             ((u32)0x00080000)        /* TIM7 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM8_STOP             ((u32)0x00100000)        /* TIM8 counter stopped when core is halted */



/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACR register  ******************/
#define  FLASH_ACR_LATENCY                   ((u8)0x07)               /* LATENCY[2:0] bits (Latency) */
#define  FLASH_ACR_LATENCY_0                 ((u8)0x01)               /* Bit 0 */
#define  FLASH_ACR_LATENCY_1                 ((u8)0x02)               /* Bit 1 */
#define  FLASH_ACR_LATENCY_2                 ((u8)0x04)               /* Bit 2 */
  
#define  FLASH_ACR_HLFCYA                    ((u8)0x08)               /* Flash Half Cycle Access Enable */
#define  FLASH_ACR_PRFTBE                    ((u8)0x10)               /* Prefetch Buffer Enable */
#define  FLASH_ACR_PRFTBS                    ((u8)0x20)               /* Prefetch Buffer Status */


/******************  Bit definition for FLASH_KEYR register  ******************/
#define  FLASH_KEYR_FKEYR                    ((u32)0xFFFFFFFF)        /* FPEC Key */


/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define  FLASH_OPTKEYR_OPTKEYR               ((u32)0xFFFFFFFF)        /* Option Byte Key */


/******************  Bit definition for FLASH_SR register  *******************/
#define  FLASH_SR_BSY                        ((u8)0x01)               /* Busy */
#define  FLASH_SR_PGERR                      ((u8)0x04)               /* Programming Error */
#define  FLASH_SR_WRPRTERR                   ((u8)0x10)               /* Write Protection Error */
#define  FLASH_SR_EOP                        ((u8)0x20)               /* End of operation */


/*******************  Bit definition for FLASH_CR register  *******************/
#define  FLASH_CR_PG                         ((u16)0x0001)            /* Programming */
#define  FLASH_CR_PER                        ((u16)0x0002)            /* Page Erase */
#define  FLASH_CR_MER                        ((u16)0x0004)            /* Mass Erase */
#define  FLASH_CR_OPTPG                      ((u16)0x0010)            /* Option Byte Programming */
#define  FLASH_CR_OPTER                      ((u16)0x0020)            /* Option Byte Erase */
#define  FLASH_CR_STRT                       ((u16)0x0040)            /* Start */
#define  FLASH_CR_LOCK                       ((u16)0x0080)            /* Lock */
#define  FLASH_CR_OPTWRE                     ((u16)0x0200)            /* Option Bytes Write Enable */
#define  FLASH_CR_ERRIE                      ((u16)0x0400)            /* Error Interrupt Enable */
#define  FLASH_CR_EOPIE                      ((u16)0x1000)            /* End of operation interrupt enable */


/*******************  Bit definition for FLASH_AR register  *******************/
#define  FLASH_AR_FAR                        ((u32)0xFFFFFFFF)        /* Flash Address */


/******************  Bit definition for FLASH_OBR register  *******************/
#define  FLASH_OBR_OPTERR                    ((u16)0x0001)            /* Option Byte Error */
#define  FLASH_OBR_RDPRT                     ((u16)0x0002)            /* Read protection */

#define  FLASH_OBR_USER                      ((u16)0x03FC)            /* User Option Bytes */
#define  FLASH_OBR_WDG_SW                    ((u16)0x0004)            /* WDG_SW */
#define  FLASH_OBR_nRST_STOP                 ((u16)0x0008)            /* nRST_STOP */
#define  FLASH_OBR_nRST_STDBY                ((u16)0x0010)            /* nRST_STDBY */
#define  FLASH_OBR_Notused                   ((u16)0x03E0)            /* Not used */


/******************  Bit definition for FLASH_WRPR register  ******************/
#define  FLASH_WRPR_WRP                        ((u32)0xFFFFFFFF)        /* Write Protect */


/*----------------------------------------------------------------------------*/


/******************  Bit definition for FLASH_RDP register  *******************/
#define  FLASH_RDP_RDP                       ((u32)0x000000FF)        /* Read protection option byte */
#define  FLASH_RDP_nRDP                      ((u32)0x0000FF00)        /* Read protection complemented option byte */


/******************  Bit definition for FLASH_USER register  ******************/
#define  FLASH_USER_USER                     ((u32)0x00FF0000)        /* User option byte */
#define  FLASH_USER_nUSER                    ((u32)0xFF000000)        /* User complemented option byte */


/******************  Bit definition for FLASH_Data0 register  *****************/
#define  FLASH_Data0_Data0                   ((u32)0x000000FF)        /* User data storage option byte */
#define  FLASH_Data0_nData0                  ((u32)0x0000FF00)        /* User data storage complemented option byte */


/******************  Bit definition for FLASH_Data1 register  *****************/
#define  FLASH_Data1_Data1                   ((u32)0x00FF0000)        /* User data storage option byte */
#define  FLASH_Data1_nData1                  ((u32)0xFF000000)        /* User data storage complemented option byte */


/******************  Bit definition for FLASH_WRP0 register  ******************/
#define  FLASH_WRP0_WRP0                     ((u32)0x000000FF)        /* Flash memory write protection option bytes */
#define  FLASH_WRP0_nWRP0                    ((u32)0x0000FF00)        /* Flash memory write protection complemented option bytes */


/******************  Bit definition for FLASH_WRP1 register  ******************/
#define  FLASH_WRP1_WRP1                     ((u32)0x00FF0000)        /* Flash memory write protection option bytes */
#define  FLASH_WRP1_nWRP1                    ((u32)0xFF000000)        /* Flash memory write protection complemented option bytes */


/******************  Bit definition for FLASH_WRP2 register  ******************/
#define  FLASH_WRP2_WRP2                     ((u32)0x000000FF)        /* Flash memory write protection option bytes */
#define  FLASH_WRP2_nWRP2                    ((u32)0x0000FF00)        /* Flash memory write protection complemented option bytes */


/******************  Bit definition for FLASH_WRP3 register  ******************/
#define  FLASH_WRP3_WRP3                     ((u32)0x00FF0000)        /* Flash memory write protection option bytes */
#define  FLASH_WRP3_nWRP3                    ((u32)0xFF000000)        /* Flash memory write protection complemented option bytes */


/* Exported macro ------------------------------------------------------------*/
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = 0x0)

#define WRITE_REG(REG, VAL)   ((REG) = VAL)

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~CLEARMASK)) | (SETMASK)))

/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10x_MAP_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_nvic.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      NVIC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_NVIC_H
#define __STM32F10x_NVIC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* NVIC Init Structure definition */
typedef struct
{
  u8 NVIC_IRQChannel;
  u8 NVIC_IRQChannelPreemptionPriority;
  u8 NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
/* IRQ Channels --------------------------------------------------------------*/
#define WWDG_IRQChannel              ((u8)0x00)  /* Window WatchDog Interrupt */
#define PVD_IRQChannel               ((u8)0x01)  /* PVD through EXTI Line detection Interrupt */
#define TAMPER_IRQChannel            ((u8)0x02)  /* Tamper Interrupt */
#define RTC_IRQChannel               ((u8)0x03)  /* RTC global Interrupt */
#define FLASH_IRQChannel             ((u8)0x04)  /* FLASH global Interrupt */
#define RCC_IRQChannel               ((u8)0x05)  /* RCC global Interrupt */
#define EXTI0_IRQChannel             ((u8)0x06)  /* EXTI Line0 Interrupt */
#define EXTI1_IRQChannel             ((u8)0x07)  /* EXTI Line1 Interrupt */
#define EXTI2_IRQChannel             ((u8)0x08)  /* EXTI Line2 Interrupt */
#define EXTI3_IRQChannel             ((u8)0x09)  /* EXTI Line3 Interrupt */
#define EXTI4_IRQChannel             ((u8)0x0A)  /* EXTI Line4 Interrupt */
#define DMA1_Channel1_IRQChannel     ((u8)0x0B)  /* DMA1 Channel 1 global Interrupt */
#define DMA1_Channel2_IRQChannel     ((u8)0x0C)  /* DMA1 Channel 2 global Interrupt */
#define DMA1_Channel3_IRQChannel     ((u8)0x0D)  /* DMA1 Channel 3 global Interrupt */
#define DMA1_Channel4_IRQChannel     ((u8)0x0E)  /* DMA1 Channel 4 global Interrupt */
#define DMA1_Channel5_IRQChannel     ((u8)0x0F)  /* DMA1 Channel 5 global Interrupt */
#define DMA1_Channel6_IRQChannel     ((u8)0x10)  /* DMA1 Channel 6 global Interrupt */
#define DMA1_Channel7_IRQChannel     ((u8)0x11)  /* DMA1 Channel 7 global Interrupt */
#define ADC1_2_IRQChannel            ((u8)0x12)  /* ADC1 et ADC2 global Interrupt */
#define USB_HP_CAN_TX_IRQChannel     ((u8)0x13)  /* USB High Priority or CAN TX Interrupts */
#define USB_LP_CAN_RX0_IRQChannel    ((u8)0x14)  /* USB Low Priority or CAN RX0 Interrupts */
#define CAN_RX1_IRQChannel           ((u8)0x15)  /* CAN RX1 Interrupt */
#define CAN_SCE_IRQChannel           ((u8)0x16)  /* CAN SCE Interrupt */
#define EXTI9_5_IRQChannel           ((u8)0x17)  /* External Line[9:5] Interrupts */
#define TIM1_BRK_IRQChannel          ((u8)0x18)  /* TIM1 Break Interrupt */
#define TIM1_UP_IRQChannel           ((u8)0x19)  /* TIM1 Update Interrupt */
#define TIM1_TRG_COM_IRQChannel      ((u8)0x1A)  /* TIM1 Trigger and Commutation Interrupt */
#define TIM1_CC_IRQChannel           ((u8)0x1B)  /* TIM1 Capture Compare Interrupt */
#define TIM2_IRQChannel              ((u8)0x1C)  /* TIM2 global Interrupt */
#define TIM3_IRQChannel              ((u8)0x1D)  /* TIM3 global Interrupt */
#define TIM4_IRQChannel              ((u8)0x1E)  /* TIM4 global Interrupt */
#define I2C1_EV_IRQChannel           ((u8)0x1F)  /* I2C1 Event Interrupt */
#define I2C1_ER_IRQChannel           ((u8)0x20)  /* I2C1 Error Interrupt */
#define I2C2_EV_IRQChannel           ((u8)0x21)  /* I2C2 Event Interrupt */
#define I2C2_ER_IRQChannel           ((u8)0x22)  /* I2C2 Error Interrupt */
#define SPI1_IRQChannel              ((u8)0x23)  /* SPI1 global Interrupt */
#define SPI2_IRQChannel              ((u8)0x24)  /* SPI2 global Interrupt */
#define USART1_IRQChannel            ((u8)0x25)  /* USART1 global Interrupt */
#define USART2_IRQChannel            ((u8)0x26)  /* USART2 global Interrupt */
#define USART3_IRQChannel            ((u8)0x27)  /* USART3 global Interrupt */
#define EXTI15_10_IRQChannel         ((u8)0x28)  /* External Line[15:10] Interrupts */
#define RTCAlarm_IRQChannel          ((u8)0x29)  /* RTC Alarm through EXTI Line Interrupt */
#define USBWakeUp_IRQChannel         ((u8)0x2A)  /* USB WakeUp from suspend through EXTI Line Interrupt */
#define TIM8_BRK_IRQChannel          ((u8)0x2B)  /* TIM8 Break Interrupt */
#define TIM8_UP_IRQChannel           ((u8)0x2C)  /* TIM8 Update Interrupt */
#define TIM8_TRG_COM_IRQChannel      ((u8)0x2D)  /* TIM8 Trigger and Commutation Interrupt */
#define TIM8_CC_IRQChannel           ((u8)0x2E)  /* TIM8 Capture Compare Interrupt */
#define ADC3_IRQChannel              ((u8)0x2F)  /* ADC3 global Interrupt */
#define FSMC_IRQChannel              ((u8)0x30)  /* FSMC global Interrupt */
#define SDIO_IRQChannel              ((u8)0x31)  /* SDIO global Interrupt */
#define TIM5_IRQChannel              ((u8)0x32)  /* TIM5 global Interrupt */
#define SPI3_IRQChannel              ((u8)0x33)  /* SPI3 global Interrupt */
#define UART4_IRQChannel             ((u8)0x34)  /* UART4 global Interrupt */
#define UART5_IRQChannel             ((u8)0x35)  /* UART5 global Interrupt */
#define TIM6_IRQChannel              ((u8)0x36)  /* TIM6 global Interrupt */
#define TIM7_IRQChannel              ((u8)0x37)  /* TIM7 global Interrupt */
#define DMA2_Channel1_IRQChannel     ((u8)0x38)  /* DMA2 Channel 1 global Interrupt */
#define DMA2_Channel2_IRQChannel     ((u8)0x39)  /* DMA2 Channel 2 global Interrupt */
#define DMA2_Channel3_IRQChannel     ((u8)0x3A)  /* DMA2 Channel 3 global Interrupt */
#define DMA2_Channel4_5_IRQChannel   ((u8)0x3B)  /* DMA2 Channel 4 and DMA2 Channel 5 global Interrupt */


#define IS_NVIC_IRQ_CHANNEL(CHANNEL) (((CHANNEL) == WWDG_IRQChannel) || \
                                      ((CHANNEL) == PVD_IRQChannel) || \
                                      ((CHANNEL) == TAMPER_IRQChannel) || \
                                      ((CHANNEL) == RTC_IRQChannel) || \
                                      ((CHANNEL) == FLASH_IRQChannel) || \
                                      ((CHANNEL) == RCC_IRQChannel) || \
                                      ((CHANNEL) == EXTI0_IRQChannel) || \
                                      ((CHANNEL) == EXTI1_IRQChannel) || \
                                      ((CHANNEL) == EXTI2_IRQChannel) || \
                                      ((CHANNEL) == EXTI3_IRQChannel) || \
                                      ((CHANNEL) == EXTI4_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel1_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel2_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel3_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel4_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel5_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel6_IRQChannel) || \
                                      ((CHANNEL) == DMA1_Channel7_IRQChannel) || \
                                      ((CHANNEL) == ADC1_2_IRQChannel) || \
                                      ((CHANNEL) == USB_HP_CAN_TX_IRQChannel) || \
                                      ((CHANNEL) == USB_LP_CAN_RX0_IRQChannel) || \
                                      ((CHANNEL) == CAN_RX1_IRQChannel) || \
                                      ((CHANNEL) == CAN_SCE_IRQChannel) || \
                                      ((CHANNEL) == EXTI9_5_IRQChannel) || \
                                      ((CHANNEL) == TIM1_BRK_IRQChannel) || \
                                      ((CHANNEL) == TIM1_UP_IRQChannel) || \
                                      ((CHANNEL) == TIM1_TRG_COM_IRQChannel) || \
                                      ((CHANNEL) == TIM1_CC_IRQChannel) || \
                                      ((CHANNEL) == TIM2_IRQChannel) || \
                                      ((CHANNEL) == TIM3_IRQChannel) || \
                                      ((CHANNEL) == TIM4_IRQChannel) || \
                                      ((CHANNEL) == I2C1_EV_IRQChannel) || \
                                      ((CHANNEL) == I2C1_ER_IRQChannel) || \
                                      ((CHANNEL) == I2C2_EV_IRQChannel) || \
                                      ((CHANNEL) == I2C2_ER_IRQChannel) || \
                                      ((CHANNEL) == SPI1_IRQChannel) || \
                                      ((CHANNEL) == SPI2_IRQChannel) || \
                                      ((CHANNEL) == USART1_IRQChannel) || \
                                      ((CHANNEL) == USART2_IRQChannel) || \
                                      ((CHANNEL) == USART3_IRQChannel) || \
                                      ((CHANNEL) == EXTI15_10_IRQChannel) || \
                                      ((CHANNEL) == RTCAlarm_IRQChannel) || \
                                      ((CHANNEL) == USBWakeUp_IRQChannel) || \
                                      ((CHANNEL) == TIM8_BRK_IRQChannel) || \
                                      ((CHANNEL) == TIM8_UP_IRQChannel) || \
                                      ((CHANNEL) == TIM8_TRG_COM_IRQChannel) || \
                                      ((CHANNEL) == TIM8_CC_IRQChannel) || \
                                      ((CHANNEL) == ADC3_IRQChannel) || \
                                      ((CHANNEL) == FSMC_IRQChannel) || \
                                      ((CHANNEL) == SDIO_IRQChannel) || \
                                      ((CHANNEL) == TIM5_IRQChannel) || \
                                      ((CHANNEL) == SPI3_IRQChannel) || \
                                      ((CHANNEL) == UART4_IRQChannel) || \
                                      ((CHANNEL) == UART5_IRQChannel) || \
                                      ((CHANNEL) == TIM6_IRQChannel) || \
                                      ((CHANNEL) == TIM7_IRQChannel) || \
                                      ((CHANNEL) == DMA2_Channel1_IRQChannel) || \
                                      ((CHANNEL) == DMA2_Channel2_IRQChannel) || \
                                      ((CHANNEL) == DMA2_Channel3_IRQChannel) || \
                                      ((CHANNEL) == DMA2_Channel4_5_IRQChannel))


/* System Handlers -----------------------------------------------------------*/
#define SystemHandler_NMI            ((u32)0x00001F) /* NMI Handler */
#define SystemHandler_HardFault      ((u32)0x000000) /* Hard Fault Handler */
#define SystemHandler_MemoryManage   ((u32)0x043430) /* Memory Manage Handler */
#define SystemHandler_BusFault       ((u32)0x547931) /* Bus Fault Handler */
#define SystemHandler_UsageFault     ((u32)0x24C232) /* Usage Fault Handler */
#define SystemHandler_SVCall         ((u32)0x01FF40) /* SVCall Handler */
#define SystemHandler_DebugMonitor   ((u32)0x0A0080) /* Debug Monitor Handler */
#define SystemHandler_PSV            ((u32)0x02829C) /* PSV Handler */
#define SystemHandler_SysTick        ((u32)0x02C39A) /* SysTick Handler */

#define IS_CONFIG_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_MemoryManage) || \
                                           ((HANDLER) == SystemHandler_BusFault) || \
                                           ((HANDLER) == SystemHandler_UsageFault))

#define IS_PRIORITY_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_MemoryManage) || \
                                             ((HANDLER) == SystemHandler_BusFault) || \
                                             ((HANDLER) == SystemHandler_UsageFault) || \
                                             ((HANDLER) == SystemHandler_SVCall) || \
                                             ((HANDLER) == SystemHandler_DebugMonitor) || \
                                             ((HANDLER) == SystemHandler_PSV) || \
                                             ((HANDLER) == SystemHandler_SysTick))

#define IS_GET_PENDING_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_MemoryManage) || \
                                                ((HANDLER) == SystemHandler_BusFault) || \
                                                ((HANDLER) == SystemHandler_SVCall))

#define IS_SET_PENDING_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_NMI) || \
                                                ((HANDLER) == SystemHandler_PSV) || \
                                                ((HANDLER) == SystemHandler_SysTick))

#define IS_CLEAR_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_PSV) || \
                                          ((HANDLER) == SystemHandler_SysTick))

#define IS_GET_ACTIVE_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_MemoryManage) || \
                                               ((HANDLER) == SystemHandler_BusFault) || \
                                               ((HANDLER) == SystemHandler_UsageFault) || \
                                               ((HANDLER) == SystemHandler_SVCall) || \
                                               ((HANDLER) == SystemHandler_DebugMonitor) || \
                                               ((HANDLER) == SystemHandler_PSV) || \
                                               ((HANDLER) == SystemHandler_SysTick))

#define IS_FAULT_SOURCE_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_HardFault) || \
                                                 ((HANDLER) == SystemHandler_MemoryManage) || \
                                                 ((HANDLER) == SystemHandler_BusFault) || \
                                                 ((HANDLER) == SystemHandler_UsageFault) || \
                                                 ((HANDLER) == SystemHandler_DebugMonitor)) 

#define IS_FAULT_ADDRESS_SYSTEM_HANDLER(HANDLER) (((HANDLER) == SystemHandler_MemoryManage) || \
                                                  ((HANDLER) == SystemHandler_BusFault))


/* Vector Table Base ---------------------------------------------------------*/
#define NVIC_VectTab_RAM             ((u32)0x20000000)
#define NVIC_VectTab_FLASH           ((u32)0x08000000)

#define IS_NVIC_VECTTAB(VECTTAB) (((VECTTAB) == NVIC_VectTab_RAM) || \
                                  ((VECTTAB) == NVIC_VectTab_FLASH))

/* System Low Power ----------------------------------------------------------*/
#define NVIC_LP_SEVONPEND            ((u8)0x10)
#define NVIC_LP_SLEEPDEEP            ((u8)0x04)
#define NVIC_LP_SLEEPONEXIT          ((u8)0x02)

#define IS_NVIC_LP(LP) (((LP) == NVIC_LP_SEVONPEND) || \
                        ((LP) == NVIC_LP_SLEEPDEEP) || \
                        ((LP) == NVIC_LP_SLEEPONEXIT))

/* Preemption Priority Group -------------------------------------------------*/
#define NVIC_PriorityGroup_0         ((u32)0x700) /* 0 bits for pre-emption priority
                                                     4 bits for subpriority */
#define NVIC_PriorityGroup_1         ((u32)0x600) /* 1 bits for pre-emption priority
                                                     3 bits for subpriority */
#define NVIC_PriorityGroup_2         ((u32)0x500) /* 2 bits for pre-emption priority
                                                     2 bits for subpriority */
#define NVIC_PriorityGroup_3         ((u32)0x400) /* 3 bits for pre-emption priority
                                                     1 bits for subpriority */
#define NVIC_PriorityGroup_4         ((u32)0x300) /* 4 bits for pre-emption priority
                                                     0 bits for subpriority */

#define IS_NVIC_PRIORITY_GROUP(GROUP) (((GROUP) == NVIC_PriorityGroup_0) || \
                                       ((GROUP) == NVIC_PriorityGroup_1) || \
                                       ((GROUP) == NVIC_PriorityGroup_2) || \
                                       ((GROUP) == NVIC_PriorityGroup_3) || \
                                       ((GROUP) == NVIC_PriorityGroup_4))

#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)
#define IS_NVIC_SUB_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)
#define IS_NVIC_OFFSET(OFFSET)  ((OFFSET) < 0x0007FFFF)
#define IS_NVIC_BASE_PRI(PRI)   ((PRI) < 0x10)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void NVIC_DeInit(void);
void NVIC_SCBDeInit(void);
void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_StructInit(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SETPRIMASK(void);
void NVIC_RESETPRIMASK(void);
void NVIC_SETFAULTMASK(void);
void NVIC_RESETFAULTMASK(void);
void NVIC_BASEPRICONFIG(u32 NewPriority);
u32 NVIC_GetBASEPRI(void);
u16 NVIC_GetCurrentPendingIRQChannel(void);
ITStatus NVIC_GetIRQChannelPendingBitStatus(u8 NVIC_IRQChannel);
void NVIC_SetIRQChannelPendingBit(u8 NVIC_IRQChannel);
void NVIC_ClearIRQChannelPendingBit(u8 NVIC_IRQChannel);
u16 NVIC_GetCurrentActiveHandler(void);
ITStatus NVIC_GetIRQChannelActiveBitStatus(u8 NVIC_IRQChannel);
u32 NVIC_GetCPUID(void);
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void NVIC_GenerateSystemReset(void);
void NVIC_GenerateCoreReset(void);
void NVIC_SystemLPConfig(u8 LowPowerMode, FunctionalState NewState);
void NVIC_SystemHandlerConfig(u32 SystemHandler, FunctionalState NewState);
void NVIC_SystemHandlerPriorityConfig(u32 SystemHandler, u8 SystemHandlerPreemptionPriority,
                                      u8 SystemHandlerSubPriority);
ITStatus NVIC_GetSystemHandlerPendingBitStatus(u32 SystemHandler);
void NVIC_SetSystemHandlerPendingBit(u32 SystemHandler);
void NVIC_ClearSystemHandlerPendingBit(u32 SystemHandler);
ITStatus NVIC_GetSystemHandlerActiveBitStatus(u32 SystemHandler);
u32 NVIC_GetFaultHandlerSources(u32 SystemHandler);
u32 NVIC_GetFaultAddress(u32 SystemHandler);

#endif /* __STM32F10x_NVIC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_pwr.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      PWR firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_PWR_H
#define __STM32F10x_PWR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* PVD detection level */
#define PWR_PVDLevel_2V2          ((u32)0x00000000)
#define PWR_PVDLevel_2V3          ((u32)0x00000020)
#define PWR_PVDLevel_2V4          ((u32)0x00000040)
#define PWR_PVDLevel_2V5          ((u32)0x00000060)
#define PWR_PVDLevel_2V6          ((u32)0x00000080)
#define PWR_PVDLevel_2V7          ((u32)0x000000A0)
#define PWR_PVDLevel_2V8          ((u32)0x000000C0)
#define PWR_PVDLevel_2V9          ((u32)0x000000E0)

#define IS_PWR_PVD_LEVEL(LEVEL) (((LEVEL) == PWR_PVDLevel_2V2) || ((LEVEL) == PWR_PVDLevel_2V3)|| \
                                 ((LEVEL) == PWR_PVDLevel_2V4) || ((LEVEL) == PWR_PVDLevel_2V5)|| \
                                 ((LEVEL) == PWR_PVDLevel_2V6) || ((LEVEL) == PWR_PVDLevel_2V7)|| \
                                 ((LEVEL) == PWR_PVDLevel_2V8) || ((LEVEL) == PWR_PVDLevel_2V9))

/* Regulator state is STOP mode */
#define PWR_Regulator_ON          ((u32)0x00000000)
#define PWR_Regulator_LowPower    ((u32)0x00000001)

#define IS_PWR_REGULATOR(REGULATOR) (((REGULATOR) == PWR_Regulator_ON) || \
                                     ((REGULATOR) == PWR_Regulator_LowPower))

/* STOP mode entry */
#define PWR_STOPEntry_WFI         ((u8)0x01)
#define PWR_STOPEntry_WFE         ((u8)0x02)

#define IS_PWR_STOP_ENTRY(ENTRY) (((ENTRY) == PWR_STOPEntry_WFI) || ((ENTRY) == PWR_STOPEntry_WFE))
 
/* PWR Flag */
#define PWR_FLAG_WU               ((u32)0x00000001)
#define PWR_FLAG_SB               ((u32)0x00000002)
#define PWR_FLAG_PVDO             ((u32)0x00000004)

#define IS_PWR_GET_FLAG(FLAG) (((FLAG) == PWR_FLAG_WU) || ((FLAG) == PWR_FLAG_SB) || \
                               ((FLAG) == PWR_FLAG_PVDO))
#define IS_PWR_CLEAR_FLAG(FLAG) (((FLAG) == PWR_FLAG_WU) || ((FLAG) == PWR_FLAG_SB))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(u32 PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(u32 PWR_Regulator, u8 PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(u32 PWR_FLAG);
void PWR_ClearFlag(u32 PWR_FLAG);

#endif /* __STM32F10x_PWR_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/





















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_rcc.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      RCC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_RCC_H
#define __STM32F10x_RCC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  u32 SYSCLK_Frequency;
  u32 HCLK_Frequency;
  u32 PCLK1_Frequency;
  u32 PCLK2_Frequency;
  u32 ADCCLK_Frequency;
}RCC_ClocksTypeDef;

/* Exported constants --------------------------------------------------------*/
/* HSE configuration */
#define RCC_HSE_OFF                      ((u32)0x00000000)
#define RCC_HSE_ON                       ((u32)0x00010000)
#define RCC_HSE_Bypass                   ((u32)0x00040000)

#define IS_RCC_HSE(HSE) (((HSE) == RCC_HSE_OFF) || ((HSE) == RCC_HSE_ON) || \
                         ((HSE) == RCC_HSE_Bypass))

/* PLL entry clock source */
#define RCC_PLLSource_HSI_Div2           ((u32)0x00000000)
#define RCC_PLLSource_HSE_Div1           ((u32)0x00010000)
#define RCC_PLLSource_HSE_Div2           ((u32)0x00030000)

#define IS_RCC_PLL_SOURCE(SOURCE) (((SOURCE) == RCC_PLLSource_HSI_Div2) || \
                                   ((SOURCE) == RCC_PLLSource_HSE_Div1) || \
                                   ((SOURCE) == RCC_PLLSource_HSE_Div2))

/* PLL multiplication factor */
#define RCC_PLLMul_2                     ((u32)0x00000000)
#define RCC_PLLMul_3                     ((u32)0x00040000)
#define RCC_PLLMul_4                     ((u32)0x00080000)
#define RCC_PLLMul_5                     ((u32)0x000C0000)
#define RCC_PLLMul_6                     ((u32)0x00100000)
#define RCC_PLLMul_7                     ((u32)0x00140000)
#define RCC_PLLMul_8                     ((u32)0x00180000)
#define RCC_PLLMul_9                     ((u32)0x001C0000)
#define RCC_PLLMul_10                    ((u32)0x00200000)
#define RCC_PLLMul_11                    ((u32)0x00240000)
#define RCC_PLLMul_12                    ((u32)0x00280000)
#define RCC_PLLMul_13                    ((u32)0x002C0000)
#define RCC_PLLMul_14                    ((u32)0x00300000)
#define RCC_PLLMul_15                    ((u32)0x00340000)
#define RCC_PLLMul_16                    ((u32)0x00380000)

#define IS_RCC_PLL_MUL(MUL) (((MUL) == RCC_PLLMul_2) || ((MUL) == RCC_PLLMul_3)   || \
                             ((MUL) == RCC_PLLMul_4) || ((MUL) == RCC_PLLMul_5)   || \
                             ((MUL) == RCC_PLLMul_6) || ((MUL) == RCC_PLLMul_7)   || \
                             ((MUL) == RCC_PLLMul_8) || ((MUL) == RCC_PLLMul_9)   || \
                             ((MUL) == RCC_PLLMul_10) || ((MUL) == RCC_PLLMul_11) || \
                             ((MUL) == RCC_PLLMul_12) || ((MUL) == RCC_PLLMul_13) || \
                             ((MUL) == RCC_PLLMul_14) || ((MUL) == RCC_PLLMul_15) || \
                             ((MUL) == RCC_PLLMul_16))

/* System clock source */
#define RCC_SYSCLKSource_HSI             ((u32)0x00000000)
#define RCC_SYSCLKSource_HSE             ((u32)0x00000001)
#define RCC_SYSCLKSource_PLLCLK          ((u32)0x00000002)

#define IS_RCC_SYSCLK_SOURCE(SOURCE) (((SOURCE) == RCC_SYSCLKSource_HSI) || \
                                      ((SOURCE) == RCC_SYSCLKSource_HSE) || \
                                      ((SOURCE) == RCC_SYSCLKSource_PLLCLK))

/* AHB clock source */
#define RCC_SYSCLK_Div1                  ((u32)0x00000000)
#define RCC_SYSCLK_Div2                  ((u32)0x00000080)
#define RCC_SYSCLK_Div4                  ((u32)0x00000090)
#define RCC_SYSCLK_Div8                  ((u32)0x000000A0)
#define RCC_SYSCLK_Div16                 ((u32)0x000000B0)
#define RCC_SYSCLK_Div64                 ((u32)0x000000C0)
#define RCC_SYSCLK_Div128                ((u32)0x000000D0)
#define RCC_SYSCLK_Div256                ((u32)0x000000E0)
#define RCC_SYSCLK_Div512                ((u32)0x000000F0)

#define IS_RCC_HCLK(HCLK) (((HCLK) == RCC_SYSCLK_Div1) || ((HCLK) == RCC_SYSCLK_Div2) || \
                           ((HCLK) == RCC_SYSCLK_Div4) || ((HCLK) == RCC_SYSCLK_Div8) || \
                           ((HCLK) == RCC_SYSCLK_Div16) || ((HCLK) == RCC_SYSCLK_Div64) || \
                           ((HCLK) == RCC_SYSCLK_Div128) || ((HCLK) == RCC_SYSCLK_Div256) || \
                           ((HCLK) == RCC_SYSCLK_Div512))

/* APB1/APB2 clock source */
#define RCC_HCLK_Div1                    ((u32)0x00000000)
#define RCC_HCLK_Div2                    ((u32)0x00000400)
#define RCC_HCLK_Div4                    ((u32)0x00000500)
#define RCC_HCLK_Div8                    ((u32)0x00000600)
#define RCC_HCLK_Div16                   ((u32)0x00000700)

#define IS_RCC_PCLK(PCLK) (((PCLK) == RCC_HCLK_Div1) || ((PCLK) == RCC_HCLK_Div2) || \
                           ((PCLK) == RCC_HCLK_Div4) || ((PCLK) == RCC_HCLK_Div8) || \
                           ((PCLK) == RCC_HCLK_Div16))

/* RCC Interrupt source */
#define RCC_IT_LSIRDY                    ((u8)0x01)
#define RCC_IT_LSERDY                    ((u8)0x02)
#define RCC_IT_HSIRDY                    ((u8)0x04)
#define RCC_IT_HSERDY                    ((u8)0x08)
#define RCC_IT_PLLRDY                    ((u8)0x10)
#define RCC_IT_CSS                       ((u8)0x80)

#define IS_RCC_IT(IT) ((((IT) & (u8)0xE0) == 0x00) && ((IT) != 0x00))
#define IS_RCC_GET_IT(IT) (((IT) == RCC_IT_LSIRDY) || ((IT) == RCC_IT_LSERDY) || \
                           ((IT) == RCC_IT_HSIRDY) || ((IT) == RCC_IT_HSERDY) || \
                           ((IT) == RCC_IT_PLLRDY) || ((IT) == RCC_IT_CSS))
#define IS_RCC_CLEAR_IT(IT) ((((IT) & (u8)0x60) == 0x00) && ((IT) != 0x00))

/* USB clock source */
#define RCC_USBCLKSource_PLLCLK_1Div5    ((u8)0x00)
#define RCC_USBCLKSource_PLLCLK_Div1     ((u8)0x01)

#define IS_RCC_USBCLK_SOURCE(SOURCE) (((SOURCE) == RCC_USBCLKSource_PLLCLK_1Div5) || \
                                      ((SOURCE) == RCC_USBCLKSource_PLLCLK_Div1))

/* ADC clock source */
#define RCC_PCLK2_Div2                   ((u32)0x00000000)
#define RCC_PCLK2_Div4                   ((u32)0x00004000)
#define RCC_PCLK2_Div6                   ((u32)0x00008000)
#define RCC_PCLK2_Div8                   ((u32)0x0000C000)

#define IS_RCC_ADCCLK(ADCCLK) (((ADCCLK) == RCC_PCLK2_Div2) || ((ADCCLK) == RCC_PCLK2_Div4) || \
                               ((ADCCLK) == RCC_PCLK2_Div6) || ((ADCCLK) == RCC_PCLK2_Div8))

/* LSE configuration */
#define RCC_LSE_OFF                      ((u8)0x00)
#define RCC_LSE_ON                       ((u8)0x01)
#define RCC_LSE_Bypass                   ((u8)0x04)

#define IS_RCC_LSE(LSE) (((LSE) == RCC_LSE_OFF) || ((LSE) == RCC_LSE_ON) || \
                         ((LSE) == RCC_LSE_Bypass))

/* RTC clock source */
#define RCC_RTCCLKSource_LSE             ((u32)0x00000100)
#define RCC_RTCCLKSource_LSI             ((u32)0x00000200)
#define RCC_RTCCLKSource_HSE_Div128      ((u32)0x00000300)

#define IS_RCC_RTCCLK_SOURCE(SOURCE) (((SOURCE) == RCC_RTCCLKSource_LSE) || \
                                      ((SOURCE) == RCC_RTCCLKSource_LSI) || \
                                      ((SOURCE) == RCC_RTCCLKSource_HSE_Div128))

/* AHB peripheral */
#define RCC_AHBPeriph_DMA1               ((u32)0x00000001)
#define RCC_AHBPeriph_DMA2               ((u32)0x00000002)
#define RCC_AHBPeriph_SRAM               ((u32)0x00000004)
#define RCC_AHBPeriph_FLITF              ((u32)0x00000010)
#define RCC_AHBPeriph_CRC                ((u32)0x00000040)
#define RCC_AHBPeriph_FSMC               ((u32)0x00000100)
#define RCC_AHBPeriph_SDIO               ((u32)0x00000400)

#define IS_RCC_AHB_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFFAA8) == 0x00) && ((PERIPH) != 0x00))

/* APB2 peripheral */
#define RCC_APB2Periph_AFIO              ((u32)0x00000001)
#define RCC_APB2Periph_GPIOA             ((u32)0x00000004)
#define RCC_APB2Periph_GPIOB             ((u32)0x00000008)
#define RCC_APB2Periph_GPIOC             ((u32)0x00000010)
#define RCC_APB2Periph_GPIOD             ((u32)0x00000020)
#define RCC_APB2Periph_GPIOE             ((u32)0x00000040)
#define RCC_APB2Periph_GPIOF             ((u32)0x00000080)
#define RCC_APB2Periph_GPIOG             ((u32)0x00000100)
#define RCC_APB2Periph_ADC1              ((u32)0x00000200)
#define RCC_APB2Periph_ADC2              ((u32)0x00000400)
#define RCC_APB2Periph_TIM1              ((u32)0x00000800)
#define RCC_APB2Periph_SPI1              ((u32)0x00001000)
#define RCC_APB2Periph_TIM8              ((u32)0x00002000)
#define RCC_APB2Periph_USART1            ((u32)0x00004000)
#define RCC_APB2Periph_ADC3              ((u32)0x00008000)
#define RCC_APB2Periph_ALL               ((u32)0x0000FFFD)

#define IS_RCC_APB2_PERIPH(PERIPH) ((((PERIPH) & 0xFFFF0002) == 0x00) && ((PERIPH) != 0x00))

/* APB1 peripheral */
#define RCC_APB1Periph_TIM2              ((u32)0x00000001)
#define RCC_APB1Periph_TIM3              ((u32)0x00000002)
#define RCC_APB1Periph_TIM4              ((u32)0x00000004)
#define RCC_APB1Periph_TIM5              ((u32)0x00000008)
#define RCC_APB1Periph_TIM6              ((u32)0x00000010)
#define RCC_APB1Periph_TIM7              ((u32)0x00000020)
#define RCC_APB1Periph_WWDG              ((u32)0x00000800)
#define RCC_APB1Periph_SPI2              ((u32)0x00004000)
#define RCC_APB1Periph_SPI3              ((u32)0x00008000)
#define RCC_APB1Periph_USART2            ((u32)0x00020000)
#define RCC_APB1Periph_USART3            ((u32)0x00040000)
#define RCC_APB1Periph_UART4             ((u32)0x00080000)
#define RCC_APB1Periph_UART5             ((u32)0x00100000)
#define RCC_APB1Periph_I2C1              ((u32)0x00200000)
#define RCC_APB1Periph_I2C2              ((u32)0x00400000)
#define RCC_APB1Periph_USB               ((u32)0x00800000)
#define RCC_APB1Periph_CAN               ((u32)0x02000000)
#define RCC_APB1Periph_BKP               ((u32)0x08000000)
#define RCC_APB1Periph_PWR               ((u32)0x10000000)
#define RCC_APB1Periph_DAC               ((u32)0x20000000)
#define RCC_APB1Periph_ALL               ((u32)0x3AFEC83F)

#define IS_RCC_APB1_PERIPH(PERIPH) ((((PERIPH) & 0xC50137C0) == 0x00) && ((PERIPH) != 0x00))

/* Clock source to output on MCO pin */
#define RCC_MCO_NoClock                  ((u8)0x00)
#define RCC_MCO_SYSCLK                   ((u8)0x04)
#define RCC_MCO_HSI                      ((u8)0x05)
#define RCC_MCO_HSE                      ((u8)0x06)
#define RCC_MCO_PLLCLK_Div2              ((u8)0x07)

#define IS_RCC_MCO(MCO) (((MCO) == RCC_MCO_NoClock) || ((MCO) == RCC_MCO_HSI) || \
                         ((MCO) == RCC_MCO_SYSCLK)  || ((MCO) == RCC_MCO_HSE) || \
                         ((MCO) == RCC_MCO_PLLCLK_Div2))

/* RCC Flag */
#define RCC_FLAG_HSIRDY                  ((u8)0x21)
#define RCC_FLAG_HSERDY                  ((u8)0x31)
#define RCC_FLAG_PLLRDY                  ((u8)0x39)
#define RCC_FLAG_LSERDY                  ((u8)0x41)
#define RCC_FLAG_LSIRDY                  ((u8)0x61)
#define RCC_FLAG_PINRST                  ((u8)0x7A)
#define RCC_FLAG_PORRST                  ((u8)0x7B)
#define RCC_FLAG_SFTRST                  ((u8)0x7C)
#define RCC_FLAG_IWDGRST                 ((u8)0x7D)
#define RCC_FLAG_WWDGRST                 ((u8)0x7E)
#define RCC_FLAG_LPWRRST                 ((u8)0x7F)

#define IS_RCC_FLAG(FLAG) (((FLAG) == RCC_FLAG_HSIRDY) || ((FLAG) == RCC_FLAG_HSERDY) || \
                           ((FLAG) == RCC_FLAG_PLLRDY) || ((FLAG) == RCC_FLAG_LSERDY) || \
                           ((FLAG) == RCC_FLAG_LSIRDY) || ((FLAG) == RCC_FLAG_PINRST) || \
                           ((FLAG) == RCC_FLAG_PORRST) || ((FLAG) == RCC_FLAG_SFTRST) || \
                           ((FLAG) == RCC_FLAG_IWDGRST)|| ((FLAG) == RCC_FLAG_WWDGRST)|| \
                           ((FLAG) == RCC_FLAG_LPWRRST))

#define IS_RCC_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x1F)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RCC_DeInit(void);
void RCC_HSEConfig(u32 RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(u8 HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(u32 RCC_PLLSource, u32 RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_SYSCLKConfig(u32 RCC_SYSCLKSource);
u8 RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(u32 RCC_SYSCLK);
void RCC_PCLK1Config(u32 RCC_HCLK);
void RCC_PCLK2Config(u32 RCC_HCLK);
void RCC_ITConfig(u8 RCC_IT, FunctionalState NewState);
void RCC_USBCLKConfig(u32 RCC_USBCLKSource);
void RCC_ADCCLKConfig(u32 RCC_PCLK2);
void RCC_LSEConfig(u8 RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(u32 RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(u32 RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(u32 RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(u32 RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(u32 RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(u32 RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(u8 RCC_MCO);
FlagStatus RCC_GetFlagStatus(u8 RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(u8 RCC_IT);
void RCC_ClearITPendingBit(u8 RCC_IT);

#endif /* __STM32F10x_RCC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_rtc.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      RTC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_RTC_H
#define __STM32F10x_RTC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* RTC interrupts define -----------------------------------------------------*/
#define RTC_IT_OW            ((u16)0x0004)  /* Overflow interrupt */
#define RTC_IT_ALR           ((u16)0x0002)  /* Alarm interrupt */
#define RTC_IT_SEC           ((u16)0x0001)  /* Second interrupt */

#define IS_RTC_IT(IT) ((((IT) & (u16)0xFFF8) == 0x00) && ((IT) != 0x00))

#define IS_RTC_GET_IT(IT) (((IT) == RTC_IT_OW) || ((IT) == RTC_IT_ALR) || \
                           ((IT) == RTC_IT_SEC))
                                                                     
/* RTC interrupts flags ------------------------------------------------------*/
#define RTC_FLAG_RTOFF       ((u16)0x0020)  /* RTC Operation OFF flag */
#define RTC_FLAG_RSF         ((u16)0x0008)  /* Registers Synchronized flag */
#define RTC_FLAG_OW          ((u16)0x0004)  /* Overflow flag */
#define RTC_FLAG_ALR         ((u16)0x0002)  /* Alarm flag */
#define RTC_FLAG_SEC         ((u16)0x0001)  /* Second flag */

#define IS_RTC_CLEAR_FLAG(FLAG) ((((FLAG) & (u16)0xFFF0) == 0x00) && ((FLAG) != 0x00))

#define IS_RTC_GET_FLAG(FLAG) (((FLAG) == RTC_FLAG_RTOFF) || ((FLAG) == RTC_FLAG_RSF) || \
                               ((FLAG) == RTC_FLAG_OW) || ((FLAG) == RTC_FLAG_ALR) || \
                               ((FLAG) == RTC_FLAG_SEC))

#define IS_RTC_PRESCALER(PRESCALER) ((PRESCALER) <= 0xFFFFF)
                           
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RTC_ITConfig(u16 RTC_IT, FunctionalState NewState);
void RTC_EnterConfigMode(void);
void RTC_ExitConfigMode(void);
u32  RTC_GetCounter(void);
void RTC_SetCounter(u32 CounterValue);
void RTC_SetPrescaler(u32 PrescalerValue);
void RTC_SetAlarm(u32 AlarmValue);
u32  RTC_GetDivider(void);
void RTC_WaitForLastTask(void);
void RTC_WaitForSynchro(void);
FlagStatus RTC_GetFlagStatus(u16 RTC_FLAG);
void RTC_ClearFlag(u16 RTC_FLAG);
ITStatus RTC_GetITStatus(u16 RTC_IT);
void RTC_ClearITPendingBit(u16 RTC_IT);

#endif /* __STM32F10x_RTC_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/





















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_sdio.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      SDIO firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SDIO_H
#define __STM32F10x_SDIO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  u8 SDIO_ClockDiv;
  u32 SDIO_ClockEdge;
  u32 SDIO_ClockBypass;
  u32 SDIO_ClockPowerSave;
  u32 SDIO_BusWide;
  u32 SDIO_HardwareFlowControl;
} SDIO_InitTypeDef;

typedef struct
{
  u32 SDIO_Argument;
  u32 SDIO_CmdIndex;
  u32 SDIO_Response;
  u32 SDIO_Wait;
  u32 SDIO_CPSM;
} SDIO_CmdInitTypeDef;

typedef struct
{
  u32 SDIO_DataTimeOut;
  u32 SDIO_DataLength;
  u32 SDIO_DataBlockSize;
  u32 SDIO_TransferDir;
  u32 SDIO_TransferMode;
  u32 SDIO_DPSM;
} SDIO_DataInitTypeDef;

/* Exported constants --------------------------------------------------------*/
/* SDIO Clock Edge -----------------------------------------------------------*/
#define SDIO_ClockEdge_Rising               ((u32)0x00000000)
#define SDIO_ClockEdge_Falling              ((u32)0x00002000)

#define IS_SDIO_CLOCK_EDGE(EDGE) (((EDGE) == SDIO_ClockEdge_Rising) || \
                                  ((EDGE) == SDIO_ClockEdge_Falling))
/* SDIO Clock Bypass ----------------------------------------------------------*/                                  
#define SDIO_ClockBypass_Disable             ((u32)0x00000000)
#define SDIO_ClockBypass_Enable              ((u32)0x00000400)    

#define IS_SDIO_CLOCK_BYPASS(BYPASS) (((BYPASS) == SDIO_ClockBypass_Disable) || \
                                     ((BYPASS) == SDIO_ClockBypass_Enable))                             

/* SDIO Clock Power Save  ----------------------------------------------------*/ 
#define SDIO_ClockPowerSave_Disable         ((u32)0x00000000)
#define SDIO_ClockPowerSave_Enable          ((u32)0x00000200) 

#define IS_SDIO_CLOCK_POWER_SAVE(SAVE) (((SAVE) == SDIO_ClockPowerSave_Disable) || \
                                        ((SAVE) == SDIO_ClockPowerSave_Enable))

/* SDIO Bus Wide -------------------------------------------------------------*/
#define SDIO_BusWide_1b                     ((u32)0x00000000)
#define SDIO_BusWide_4b                     ((u32)0x00000800)
#define SDIO_BusWide_8b                     ((u32)0x00001000)

#define IS_SDIO_BUS_WIDE(WIDE) (((WIDE) == SDIO_BusWide_1b) || ((WIDE) == SDIO_BusWide_4b) || \
                                ((WIDE) == SDIO_BusWide_8b))
                                
/* SDIO Hardware Flow Control  -----------------------------------------------*/ 
#define SDIO_HardwareFlowControl_Disable    ((u32)0x00000000)
#define SDIO_HardwareFlowControl_Enable     ((u32)0x00004000)

#define IS_SDIO_HARDWARE_FLOW_CONTROL(CONTROL) (((CONTROL) == SDIO_HardwareFlowControl_Disable) || \
                                                ((CONTROL) == SDIO_HardwareFlowControl_Enable))
                                  
/* SDIO Power State ----------------------------------------------------------*/
#define SDIO_PowerState_OFF                 ((u32)0x00000000)
#define SDIO_PowerState_ON                  ((u32)0x00000003)

#define IS_SDIO_POWER_STATE(STATE) (((STATE) == SDIO_PowerState_OFF) || ((STATE) == SDIO_PowerState_ON)) 

/* SDIO Interrupt soucres ----------------------------------------------------*/
#define SDIO_IT_CCRCFAIL                    ((u32)0x00000001)
#define SDIO_IT_DCRCFAIL                    ((u32)0x00000002)
#define SDIO_IT_CTIMEOUT                    ((u32)0x00000004)
#define SDIO_IT_DTIMEOUT                    ((u32)0x00000008)
#define SDIO_IT_TXUNDERR                    ((u32)0x00000010)
#define SDIO_IT_RXOVERR                     ((u32)0x00000020)
#define SDIO_IT_CMDREND                     ((u32)0x00000040)
#define SDIO_IT_CMDSENT                     ((u32)0x00000080)
#define SDIO_IT_DATAEND                     ((u32)0x00000100)
#define SDIO_IT_STBITERR                    ((u32)0x00000200)
#define SDIO_IT_DBCKEND                     ((u32)0x00000400)
#define SDIO_IT_CMDACT                      ((u32)0x00000800)
#define SDIO_IT_TXACT                       ((u32)0x00001000)
#define SDIO_IT_RXACT                       ((u32)0x00002000)
#define SDIO_IT_TXFIFOHE                    ((u32)0x00004000)
#define SDIO_IT_RXFIFOHF                    ((u32)0x00008000)
#define SDIO_IT_TXFIFOF                     ((u32)0x00010000)
#define SDIO_IT_RXFIFOF                     ((u32)0x00020000)
#define SDIO_IT_TXFIFOE                     ((u32)0x00040000)
#define SDIO_IT_RXFIFOE                     ((u32)0x00080000)
#define SDIO_IT_TXDAVL                      ((u32)0x00100000)
#define SDIO_IT_RXDAVL                      ((u32)0x00200000)
#define SDIO_IT_SDIOIT                      ((u32)0x00400000)
#define SDIO_IT_CEATAEND                    ((u32)0x00800000)

#define IS_SDIO_IT(IT) ((((IT) & (u32)0xFF000000) == 0x00) && ((IT) != (u32)0x00))

/* SDIO Command Index  -------------------------------------------------------*/
#define IS_SDIO_CMD_INDEX(INDEX)            ((INDEX) < 0x40)

/* SDIO Response Type --------------------------------------------------------*/
#define SDIO_Response_No                    ((u32)0x00000000)
#define SDIO_Response_Short                 ((u32)0x00000040)
#define SDIO_Response_Long                  ((u32)0x000000C0)

#define IS_SDIO_RESPONSE(RESPONSE) (((RESPONSE) == SDIO_Response_No) || \
                                    ((RESPONSE) == SDIO_Response_Short) || \
                                    ((RESPONSE) == SDIO_Response_Long))

/* SDIO Wait Interrupt State -------------------------------------------------*/
#define SDIO_Wait_No                        ((u32)0x00000000) /* SDIO No Wait, TimeOut is enabled */
#define SDIO_Wait_IT                        ((u32)0x00000100) /* SDIO Wait Interrupt Request */
#define SDIO_Wait_Pend                      ((u32)0x00000200) /* SDIO Wait End of transfer */

#define IS_SDIO_WAIT(WAIT) (((WAIT) == SDIO_Wait_No) || ((WAIT) == SDIO_Wait_IT) || \
                            ((WAIT) == SDIO_Wait_Pend))

/* SDIO CPSM State -----------------------------------------------------------*/
#define SDIO_CPSM_Disable                    ((u32)0x00000000)
#define SDIO_CPSM_Enable                     ((u32)0x00000400)

#define IS_SDIO_CPSM(CPSM) (((CPSM) == SDIO_CPSM_Enable) || ((CPSM) == SDIO_CPSM_Disable))

/* SDIO Response Registers ---------------------------------------------------*/
#define SDIO_RESP1                          ((u32)0x00000000)
#define SDIO_RESP2                          ((u32)0x00000004)
#define SDIO_RESP3                          ((u32)0x00000008)
#define SDIO_RESP4                          ((u32)0x0000000C)

#define IS_SDIO_RESP(RESP) (((RESP) == SDIO_RESP1) || ((RESP) == SDIO_RESP2) || \
                            ((RESP) == SDIO_RESP3) || ((RESP) == SDIO_RESP4))

/* SDIO Data Length ----------------------------------------------------------*/
#define IS_SDIO_DATA_LENGTH(LENGTH) ((LENGTH) <= 0x01FFFFFF)

/* SDIO Data Block Size ------------------------------------------------------*/
#define SDIO_DataBlockSize_1b               ((u32)0x00000000)
#define SDIO_DataBlockSize_2b               ((u32)0x00000010)
#define SDIO_DataBlockSize_4b               ((u32)0x00000020)
#define SDIO_DataBlockSize_8b               ((u32)0x00000030)
#define SDIO_DataBlockSize_16b              ((u32)0x00000040)
#define SDIO_DataBlockSize_32b              ((u32)0x00000050)
#define SDIO_DataBlockSize_64b              ((u32)0x00000060)
#define SDIO_DataBlockSize_128b             ((u32)0x00000070)
#define SDIO_DataBlockSize_256b             ((u32)0x00000080)
#define SDIO_DataBlockSize_512b             ((u32)0x00000090)
#define SDIO_DataBlockSize_1024b            ((u32)0x000000A0)
#define SDIO_DataBlockSize_2048b            ((u32)0x000000B0)
#define SDIO_DataBlockSize_4096b            ((u32)0x000000C0)
#define SDIO_DataBlockSize_8192b            ((u32)0x000000D0)
#define SDIO_DataBlockSize_16384b           ((u32)0x000000E0)

#define IS_SDIO_BLOCK_SIZE(SIZE) (((SIZE) == SDIO_DataBlockSize_1b) || \
                                  ((SIZE) == SDIO_DataBlockSize_2b) || \
                                  ((SIZE) == SDIO_DataBlockSize_4b) || \
                                  ((SIZE) == SDIO_DataBlockSize_8b) || \
                                  ((SIZE) == SDIO_DataBlockSize_16b) || \
                                  ((SIZE) == SDIO_DataBlockSize_32b) || \
                                  ((SIZE) == SDIO_DataBlockSize_64b) || \
                                  ((SIZE) == SDIO_DataBlockSize_128b) || \
                                  ((SIZE) == SDIO_DataBlockSize_256b) || \
                                  ((SIZE) == SDIO_DataBlockSize_512b) || \
                                  ((SIZE) == SDIO_DataBlockSize_1024b) || \
                                  ((SIZE) == SDIO_DataBlockSize_2048b) || \
                                  ((SIZE) == SDIO_DataBlockSize_4096b) || \
                                  ((SIZE) == SDIO_DataBlockSize_8192b) || \
                                  ((SIZE) == SDIO_DataBlockSize_16384b)) 

/* SDIO Transfer Direction ---------------------------------------------------*/
#define SDIO_TransferDir_ToCard             ((u32)0x00000000)
#define SDIO_TransferDir_ToSDIO             ((u32)0x00000002)

#define IS_SDIO_TRANSFER_DIR(DIR) (((DIR) == SDIO_TransferDir_ToCard) || \
                                   ((DIR) == SDIO_TransferDir_ToSDIO))  

/* SDIO Transfer Type --------------------------------------------------------*/
#define SDIO_TransferMode_Block             ((u32)0x00000000)
#define SDIO_TransferMode_Stream            ((u32)0x00000004)

#define IS_SDIO_TRANSFER_MODE(MODE) (((MODE) == SDIO_TransferMode_Stream) || \
                                     ((MODE) == SDIO_TransferMode_Block))                                

/* SDIO DPSM State -----------------------------------------------------------*/
#define SDIO_DPSM_Disable                    ((u32)0x00000000)
#define SDIO_DPSM_Enable                     ((u32)0x00000001)

#define IS_SDIO_DPSM(DPSM) (((DPSM) == SDIO_DPSM_Enable) || ((DPSM) == SDIO_DPSM_Disable))

/* SDIO Flags ----------------------------------------------------------------*/
#define SDIO_FLAG_CCRCFAIL                  ((u32)0x00000001)
#define SDIO_FLAG_DCRCFAIL                  ((u32)0x00000002)
#define SDIO_FLAG_CTIMEOUT                  ((u32)0x00000004)
#define SDIO_FLAG_DTIMEOUT                  ((u32)0x00000008)
#define SDIO_FLAG_TXUNDERR                  ((u32)0x00000010)
#define SDIO_FLAG_RXOVERR                   ((u32)0x00000020)
#define SDIO_FLAG_CMDREND                   ((u32)0x00000040)
#define SDIO_FLAG_CMDSENT                   ((u32)0x00000080)
#define SDIO_FLAG_DATAEND                   ((u32)0x00000100)
#define SDIO_FLAG_STBITERR                  ((u32)0x00000200)
#define SDIO_FLAG_DBCKEND                   ((u32)0x00000400)
#define SDIO_FLAG_CMDACT                    ((u32)0x00000800)
#define SDIO_FLAG_TXACT                     ((u32)0x00001000)
#define SDIO_FLAG_RXACT                     ((u32)0x00002000)
#define SDIO_FLAG_TXFIFOHE                  ((u32)0x00004000)
#define SDIO_FLAG_RXFIFOHF                  ((u32)0x00008000)
#define SDIO_FLAG_TXFIFOF                   ((u32)0x00010000)
#define SDIO_FLAG_RXFIFOF                   ((u32)0x00020000)
#define SDIO_FLAG_TXFIFOE                   ((u32)0x00040000)
#define SDIO_FLAG_RXFIFOE                   ((u32)0x00080000)
#define SDIO_FLAG_TXDAVL                    ((u32)0x00100000)
#define SDIO_FLAG_RXDAVL                    ((u32)0x00200000)
#define SDIO_FLAG_SDIOIT                    ((u32)0x00400000)
#define SDIO_FLAG_CEATAEND                  ((u32)0x00800000)

#define IS_SDIO_FLAG(FLAG) (((FLAG)  == SDIO_FLAG_CCRCFAIL) || \
                            ((FLAG)  == SDIO_FLAG_DCRCFAIL) || \
                            ((FLAG)  == SDIO_FLAG_CTIMEOUT) || \
                            ((FLAG)  == SDIO_FLAG_DTIMEOUT) || \
                            ((FLAG)  == SDIO_FLAG_TXUNDERR) || \
                            ((FLAG)  == SDIO_FLAG_RXOVERR) || \
                            ((FLAG)  == SDIO_FLAG_CMDREND) || \
                            ((FLAG)  == SDIO_FLAG_CMDSENT) || \
                            ((FLAG)  == SDIO_FLAG_DATAEND) || \
                            ((FLAG)  == SDIO_FLAG_STBITERR) || \
                            ((FLAG)  == SDIO_FLAG_DBCKEND) || \
                            ((FLAG)  == SDIO_FLAG_CMDACT) || \
                            ((FLAG)  == SDIO_FLAG_TXACT) || \
                            ((FLAG)  == SDIO_FLAG_RXACT) || \
                            ((FLAG)  == SDIO_FLAG_TXFIFOHE) || \
                            ((FLAG)  == SDIO_FLAG_RXFIFOHF) || \
                            ((FLAG)  == SDIO_FLAG_TXFIFOF) || \
                            ((FLAG)  == SDIO_FLAG_RXFIFOF) || \
                            ((FLAG)  == SDIO_FLAG_TXFIFOE) || \
                            ((FLAG)  == SDIO_FLAG_RXFIFOE) || \
                            ((FLAG)  == SDIO_FLAG_TXDAVL) || \
                            ((FLAG)  == SDIO_FLAG_RXDAVL) || \
                            ((FLAG)  == SDIO_FLAG_SDIOIT) || \
                            ((FLAG)  == SDIO_FLAG_CEATAEND))

#define IS_SDIO_CLEAR_FLAG(FLAG) ((((FLAG) & (u32)0xFF3FF800) == 0x00) && ((FLAG) != (u32)0x00))

#define IS_SDIO_GET_IT(IT) (((IT)  == SDIO_IT_CCRCFAIL) || \
                            ((IT)  == SDIO_IT_DCRCFAIL) || \
                            ((IT)  == SDIO_IT_CTIMEOUT) || \
                            ((IT)  == SDIO_IT_DTIMEOUT) || \
                            ((IT)  == SDIO_IT_TXUNDERR) || \
                            ((IT)  == SDIO_IT_RXOVERR) || \
                            ((IT)  == SDIO_IT_CMDREND) || \
                            ((IT)  == SDIO_IT_CMDSENT) || \
                            ((IT)  == SDIO_IT_DATAEND) || \
                            ((IT)  == SDIO_IT_STBITERR) || \
                            ((IT)  == SDIO_IT_DBCKEND) || \
                            ((IT)  == SDIO_IT_CMDACT) || \
                            ((IT)  == SDIO_IT_TXACT) || \
                            ((IT)  == SDIO_IT_RXACT) || \
                            ((IT)  == SDIO_IT_TXFIFOHE) || \
                            ((IT)  == SDIO_IT_RXFIFOHF) || \
                            ((IT)  == SDIO_IT_TXFIFOF) || \
                            ((IT)  == SDIO_IT_RXFIFOF) || \
                            ((IT)  == SDIO_IT_TXFIFOE) || \
                            ((IT)  == SDIO_IT_RXFIFOE) || \
                            ((IT)  == SDIO_IT_TXDAVL) || \
                            ((IT)  == SDIO_IT_RXDAVL) || \
                            ((IT)  == SDIO_IT_SDIOIT) || \
                            ((IT)  == SDIO_IT_CEATAEND))

#define IS_SDIO_CLEAR_IT(IT) ((((IT) & (u32)0xFF3FF800) == 0x00) && ((IT) != (u32)0x00))
                                                        
/* SDIO Read Wait Mode -------------------------------------------------------*/
#define SDIO_ReadWaitMode_CLK               ((u32)0x00000000)
#define SDIO_ReadWaitMode_DATA2             ((u32)0x00000001)

#define IS_SDIO_READWAIT_MODE(MODE) (((MODE) == SDIO_ReadWaitMode_CLK) || \
                                     ((MODE) == SDIO_ReadWaitMode_DATA2))  

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SDIO_DeInit(void);
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(u32 SDIO_PowerState);
u32 SDIO_GetPowerState(void);
void SDIO_ITConfig(u32 SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
u8 SDIO_GetCommandResponse(void);
u32 SDIO_GetResponse(u32 SDIO_RESP);
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
u32 SDIO_GetDataCounter(void);
u32 SDIO_ReadData(void);
void SDIO_WriteData(u32 Data);
u32 SDIO_GetFIFOCount(void);
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(u32 SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(u32 SDIO_FLAG);
void SDIO_ClearFlag(u32 SDIO_FLAG);
ITStatus SDIO_GetITStatus(u32 SDIO_IT);
void SDIO_ClearITPendingBit(u32 SDIO_IT);

#endif /* __STM32F10x_SDIO_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_spi.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      SPI firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

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

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_systick.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      SysTick firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SYSTICK_H
#define __STM32F10x_SYSTICK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* SysTick clock source */
#define SysTick_CLKSource_HCLK_Div8    ((u32)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK         ((u32)0x00000004)

#define IS_SYSTICK_CLK_SOURCE(SOURCE) (((SOURCE) == SysTick_CLKSource_HCLK) || \
                                       ((SOURCE) == SysTick_CLKSource_HCLK_Div8))

/* SysTick counter state */
#define SysTick_Counter_Disable        ((u32)0xFFFFFFFE)
#define SysTick_Counter_Enable         ((u32)0x00000001)
#define SysTick_Counter_Clear          ((u32)0x00000000)

#define IS_SYSTICK_COUNTER(COUNTER) (((COUNTER) == SysTick_Counter_Disable) || \
                                     ((COUNTER) == SysTick_Counter_Enable)  || \
                                     ((COUNTER) == SysTick_Counter_Clear))

/* SysTick Flag */
#define SysTick_FLAG_COUNT             ((u32)0x00000010)
#define SysTick_FLAG_SKEW              ((u32)0x0000001E)
#define SysTick_FLAG_NOREF             ((u32)0x0000001F)

#define IS_SYSTICK_FLAG(FLAG) (((FLAG) == SysTick_FLAG_COUNT) || \
                               ((FLAG) == SysTick_FLAG_SKEW)  || \
                               ((FLAG) == SysTick_FLAG_NOREF))

#define IS_SYSTICK_RELOAD(RELOAD) (((RELOAD) > 0) && ((RELOAD) <= 0xFFFFFF))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SysTick_CLKSourceConfig(u32 SysTick_CLKSource);
void SysTick_SetReload(u32 Reload);
void SysTick_CounterCmd(u32 SysTick_Counter);
void SysTick_ITConfig(FunctionalState NewState);
u32 SysTick_GetCounter(void);
FlagStatus SysTick_GetFlagStatus(u8 SysTick_FLAG);

#endif /* __STM32F10x_SYSTICK_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/



















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_tim.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the 
*                      TIM firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_TIM_H
#define __STM32F10x_TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/

/* TIM Time Base Init structure definition */
typedef struct
{
  u16 TIM_Prescaler;
  u16 TIM_CounterMode;
  u16 TIM_Period;
  u16 TIM_ClockDivision;
  u8 TIM_RepetitionCounter;
} TIM_TimeBaseInitTypeDef;

/* TIM Output Compare Init structure definition */
typedef struct
{
  u16 TIM_OCMode;
  u16 TIM_OutputState;
  u16 TIM_OutputNState;
  u16 TIM_Pulse;
  u16 TIM_OCPolarity;
  u16 TIM_OCNPolarity;
  u16 TIM_OCIdleState;
  u16 TIM_OCNIdleState;
} TIM_OCInitTypeDef;

/* TIM Input Capture Init structure definition */
typedef struct
{
  u16 TIM_Channel;
  u16 TIM_ICPolarity;
  u16 TIM_ICSelection;
  u16 TIM_ICPrescaler;
  u16 TIM_ICFilter;
} TIM_ICInitTypeDef;

/* BDTR structure definition */
typedef struct
{
  u16 TIM_OSSRState;
  u16 TIM_OSSIState;
  u16 TIM_LOCKLevel; 
  u16 TIM_DeadTime;
  u16 TIM_Break;
  u16 TIM_BreakPolarity;
  u16 TIM_AutomaticOutput;
} TIM_BDTRInitTypeDef;

/* Exported constants --------------------------------------------------------*/                             

#define IS_TIM_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == TIM1_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM2_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM3_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM4_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM5_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM6_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM7_BASE) || \
                                   ((*(u32*)&(PERIPH)) == TIM8_BASE))

#define IS_TIM_18_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == TIM1_BASE) || \
                                  ((*(u32*)&(PERIPH)) == TIM8_BASE))

#define IS_TIM_123458_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == TIM1_BASE) || \
                                      ((*(u32*)&(PERIPH)) == TIM2_BASE) || \
                                      ((*(u32*)&(PERIPH)) == TIM3_BASE) || \
                                      ((*(u32*)&(PERIPH)) == TIM4_BASE) || \
                                      ((*(u32*)&(PERIPH)) == TIM5_BASE) || \
                                      ((*(u32*)&(PERIPH)) == TIM8_BASE))

/* TIM Output Compare and PWM modes -----------------------------------------*/
#define TIM_OCMode_Timing                  ((u16)0x0000)
#define TIM_OCMode_Active                  ((u16)0x0010)
#define TIM_OCMode_Inactive                ((u16)0x0020)
#define TIM_OCMode_Toggle                  ((u16)0x0030)
#define TIM_OCMode_PWM1                    ((u16)0x0060)
#define TIM_OCMode_PWM2                    ((u16)0x0070)

#define IS_TIM_OC_MODE(MODE) (((MODE) == TIM_OCMode_Timing) || \
                              ((MODE) == TIM_OCMode_Active) || \
                              ((MODE) == TIM_OCMode_Inactive) || \
                              ((MODE) == TIM_OCMode_Toggle)|| \
                              ((MODE) == TIM_OCMode_PWM1) || \
                              ((MODE) == TIM_OCMode_PWM2))

#define IS_TIM_OCM(MODE) (((MODE) == TIM_OCMode_Timing) || \
                          ((MODE) == TIM_OCMode_Active) || \
                          ((MODE) == TIM_OCMode_Inactive) || \
                          ((MODE) == TIM_OCMode_Toggle)|| \
                          ((MODE) == TIM_OCMode_PWM1) || \
                          ((MODE) == TIM_OCMode_PWM2) ||	\
                          ((MODE) == TIM_ForcedAction_Active) || \
                          ((MODE) == TIM_ForcedAction_InActive))
/* TIM One Pulse Mode -------------------------------------------------------*/
#define TIM_OPMode_Single                  ((u16)0x0008)
#define TIM_OPMode_Repetitive              ((u16)0x0000)

#define IS_TIM_OPM_MODE(MODE) (((MODE) == TIM_OPMode_Single) || \
                               ((MODE) == TIM_OPMode_Repetitive))

/* TIM Channel -------------------------------------------------------------*/
#define TIM_Channel_1                      ((u16)0x0000)
#define TIM_Channel_2                      ((u16)0x0004)
#define TIM_Channel_3                      ((u16)0x0008)
#define TIM_Channel_4                      ((u16)0x000C)

#define IS_TIM_CHANNEL(CHANNEL) (((CHANNEL) == TIM_Channel_1) || \
                                 ((CHANNEL) == TIM_Channel_2) || \
                                 ((CHANNEL) == TIM_Channel_3) || \
                                 ((CHANNEL) == TIM_Channel_4))

#define IS_TIM_PWMI_CHANNEL(CHANNEL) (((CHANNEL) == TIM_Channel_1) || \
                                      ((CHANNEL) == TIM_Channel_2))

#define IS_TIM_COMPLEMENTARY_CHANNEL(CHANNEL) (((CHANNEL) == TIM_Channel_1) || \
                                               ((CHANNEL) == TIM_Channel_2) || \
                                               ((CHANNEL) == TIM_Channel_3))
/* TIM Clock Division CKD --------------------------------------------------*/
#define TIM_CKD_DIV1                       ((u16)0x0000)
#define TIM_CKD_DIV2                       ((u16)0x0100)
#define TIM_CKD_DIV4                       ((u16)0x0200)

#define IS_TIM_CKD_DIV(DIV) (((DIV) == TIM_CKD_DIV1) || \
                             ((DIV) == TIM_CKD_DIV2) || \
                             ((DIV) == TIM_CKD_DIV4))

/* TIM Counter Mode --------------------------------------------------------*/
#define TIM_CounterMode_Up                 ((u16)0x0000)
#define TIM_CounterMode_Down               ((u16)0x0010)
#define TIM_CounterMode_CenterAligned1     ((u16)0x0020)
#define TIM_CounterMode_CenterAligned2     ((u16)0x0040)
#define TIM_CounterMode_CenterAligned3     ((u16)0x0060)

#define IS_TIM_COUNTER_MODE(MODE) (((MODE) == TIM_CounterMode_Up) ||  \
                                   ((MODE) == TIM_CounterMode_Down) || \
                                   ((MODE) == TIM_CounterMode_CenterAligned1) || \
                                   ((MODE) == TIM_CounterMode_CenterAligned2) || \
                                   ((MODE) == TIM_CounterMode_CenterAligned3))

/* TIM Output Compare Polarity ---------------------------------------------*/
#define TIM_OCPolarity_High                ((u16)0x0000)
#define TIM_OCPolarity_Low                 ((u16)0x0002)

#define IS_TIM_OC_POLARITY(POLARITY) (((POLARITY) == TIM_OCPolarity_High) || \
                                      ((POLARITY) == TIM_OCPolarity_Low))

/* TIM Output Compare N Polarity -------------------------------------------*/
#define TIM_OCNPolarity_High               ((u16)0x0000)
#define TIM_OCNPolarity_Low                ((u16)0x0008)

#define IS_TIM_OCN_POLARITY(POLARITY) (((POLARITY) == TIM_OCNPolarity_High) || \
                                       ((POLARITY) == TIM_OCNPolarity_Low))

/* TIM Output Compare states -----------------------------------------------*/
#define TIM_OutputState_Disable            ((u16)0x0000)
#define TIM_OutputState_Enable             ((u16)0x0001)

#define IS_TIM_OUTPUT_STATE(STATE) (((STATE) == TIM_OutputState_Disable) || \
                                    ((STATE) == TIM_OutputState_Enable))

/* TIM Output Compare N States ---------------------------------------------*/
#define TIM_OutputNState_Disable           ((u16)0x0000)
#define TIM_OutputNState_Enable            ((u16)0x0004)

#define IS_TIM_OUTPUTN_STATE(STATE) (((STATE) == TIM_OutputNState_Disable) || \
                                     ((STATE) == TIM_OutputNState_Enable))

/* TIM Capture Compare States -----------------------------------------------*/
#define TIM_CCx_Enable                      ((u16)0x0001)
#define TIM_CCx_Disable                     ((u16)0x0000)

#define IS_TIM_CCX(CCX) (((CCX) == TIM_CCx_Enable) || \
                         ((CCX) == TIM_CCx_Disable))

/* TIM Capture Compare N States --------------------------------------------*/
#define TIM_CCxN_Enable                     ((u16)0x0004)
#define TIM_CCxN_Disable                    ((u16)0x0000)                                     

#define IS_TIM_CCXN(CCXN) (((CCXN) == TIM_CCxN_Enable) || \
                           ((CCXN) == TIM_CCxN_Disable))

/* Break Input enable/disable -----------------------------------------------*/
#define TIM_Break_Enable                   ((u16)0x1000)
#define TIM_Break_Disable                  ((u16)0x0000)

#define IS_TIM_BREAK_STATE(STATE) (((STATE) == TIM_Break_Enable) || \
                                   ((STATE) == TIM_Break_Disable))

/* Break Polarity -----------------------------------------------------------*/
#define TIM_BreakPolarity_Low              ((u16)0x0000)
#define TIM_BreakPolarity_High             ((u16)0x2000)

#define IS_TIM_BREAK_POLARITY(POLARITY) (((POLARITY) == TIM_BreakPolarity_Low) || \
                                         ((POLARITY) == TIM_BreakPolarity_High))

/* TIM AOE Bit Set/Reset ---------------------------------------------------*/
#define TIM_AutomaticOutput_Enable         ((u16)0x4000)
#define TIM_AutomaticOutput_Disable        ((u16)0x0000)

#define IS_TIM_AUTOMATIC_OUTPUT_STATE(STATE) (((STATE) == TIM_AutomaticOutput_Enable) || \
                                              ((STATE) == TIM_AutomaticOutput_Disable))
/* Lock levels --------------------------------------------------------------*/
#define TIM_LOCKLevel_OFF                  ((u16)0x0000)
#define TIM_LOCKLevel_1                    ((u16)0x0100)
#define TIM_LOCKLevel_2                    ((u16)0x0200)
#define TIM_LOCKLevel_3                    ((u16)0x0300)

#define IS_TIM_LOCK_LEVEL(LEVEL) (((LEVEL) == TIM_LOCKLevel_OFF) || \
                                  ((LEVEL) == TIM_LOCKLevel_1) || \
                                  ((LEVEL) == TIM_LOCKLevel_2) || \
                                  ((LEVEL) == TIM_LOCKLevel_3))

/* OSSI: Off-State Selection for Idle mode states ---------------------------*/
#define TIM_OSSIState_Enable               ((u16)0x0400)
#define TIM_OSSIState_Disable              ((u16)0x0000)

#define IS_TIM_OSSI_STATE(STATE) (((STATE) == TIM_OSSIState_Enable) || \
                                  ((STATE) == TIM_OSSIState_Disable))

/* OSSR: Off-State Selection for Run mode states ----------------------------*/
#define TIM_OSSRState_Enable               ((u16)0x0800)
#define TIM_OSSRState_Disable              ((u16)0x0000)

#define IS_TIM_OSSR_STATE(STATE) (((STATE) == TIM_OSSRState_Enable) || \
                                  ((STATE) == TIM_OSSRState_Disable))

/* TIM Output Compare Idle State -------------------------------------------*/
#define TIM_OCIdleState_Set                ((u16)0x0100)
#define TIM_OCIdleState_Reset              ((u16)0x0000)

#define IS_TIM_OCIDLE_STATE(STATE) (((STATE) == TIM_OCIdleState_Set) || \
                                    ((STATE) == TIM_OCIdleState_Reset))

/* TIM Output Compare N Idle State -----------------------------------------*/
#define TIM_OCNIdleState_Set               ((u16)0x0200)
#define TIM_OCNIdleState_Reset             ((u16)0x0000)

#define IS_TIM_OCNIDLE_STATE(STATE) (((STATE) == TIM_OCNIdleState_Set) || \
                                     ((STATE) == TIM_OCNIdleState_Reset))

/* TIM Input Capture Polarity ----------------------------------------------*/
#define  TIM_ICPolarity_Rising             ((u16)0x0000)
#define  TIM_ICPolarity_Falling            ((u16)0x0002)

#define IS_TIM_IC_POLARITY(POLARITY) (((POLARITY) == TIM_ICPolarity_Rising) || \
                                      ((POLARITY) == TIM_ICPolarity_Falling))

/* TIM Input Capture Selection ---------------------------------------------*/
#define TIM_ICSelection_DirectTI           ((u16)0x0001)
#define TIM_ICSelection_IndirectTI         ((u16)0x0002)
#define TIM_ICSelection_TRC                ((u16)0x0003)

#define IS_TIM_IC_SELECTION(SELECTION) (((SELECTION) == TIM_ICSelection_DirectTI) || \
                                        ((SELECTION) == TIM_ICSelection_IndirectTI) || \
                                        ((SELECTION) == TIM_ICSelection_TRC))

/* TIM Input Capture Prescaler ---------------------------------------------*/
#define TIM_ICPSC_DIV1                     ((u16)0x0000)
#define TIM_ICPSC_DIV2                     ((u16)0x0004)
#define TIM_ICPSC_DIV4                     ((u16)0x0008)
#define TIM_ICPSC_DIV8                     ((u16)0x000C)

#define IS_TIM_IC_PRESCALER(PRESCALER) (((PRESCALER) == TIM_ICPSC_DIV1) || \
                                        ((PRESCALER) == TIM_ICPSC_DIV2) || \
                                        ((PRESCALER) == TIM_ICPSC_DIV4) || \
                                        ((PRESCALER) == TIM_ICPSC_DIV8))                                          

/* TIM interrupt sources ---------------------------------------------------*/
#define TIM_IT_Update                      ((u16)0x0001)
#define TIM_IT_CC1                         ((u16)0x0002)
#define TIM_IT_CC2                         ((u16)0x0004)
#define TIM_IT_CC3                         ((u16)0x0008)
#define TIM_IT_CC4                         ((u16)0x0010)
#define TIM_IT_COM                         ((u16)0x0020)
#define TIM_IT_Trigger                     ((u16)0x0040)
#define TIM_IT_Break                       ((u16)0x0080)

#define IS_TIM_IT(IT) ((((IT) & (u16)0xFF00) == 0x0000) && ((IT) != 0x0000))

#define IS_TIM_PERIPH_IT(PERIPH, TIM_IT) ((((((*(u32*)&(PERIPH)) == TIM2_BASE) || (((*(u32*)&(PERIPH)) == TIM3_BASE))||\
                                            (((*(u32*)&(PERIPH)) == TIM4_BASE)) || (((*(u32*)&(PERIPH)) == TIM5_BASE))))&& \
                                            (((TIM_IT) & (u16)0xFFA0) == 0x0000) && ((TIM_IT) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM1_BASE) || (((*(u32*)&(PERIPH)) == TIM8_BASE))))&& \
                                            (((TIM_IT) & (u16)0xFF00) == 0x0000) && ((TIM_IT) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM6_BASE) || (((*(u32*)&(PERIPH)) == TIM7_BASE))))&& \
                                            (((TIM_IT) & (u16)0xFFFE) == 0x0000) && ((TIM_IT) != 0x0000)))

#define IS_TIM_GET_IT(IT) (((IT) == TIM_IT_Update) || \
                           ((IT) == TIM_IT_CC1) || \
                           ((IT) == TIM_IT_CC2) || \
                           ((IT) == TIM_IT_CC3) || \
                           ((IT) == TIM_IT_CC4) || \
                           ((IT) == TIM_IT_COM) || \
                           ((IT) == TIM_IT_Trigger) || \
                           ((IT) == TIM_IT_Break))

/* TIM DMA Base address ----------------------------------------------------*/
#define TIM_DMABase_CR1                    ((u16)0x0000)
#define TIM_DMABase_CR2                    ((u16)0x0001)
#define TIM_DMABase_SMCR                   ((u16)0x0002)
#define TIM_DMABase_DIER                   ((u16)0x0003)
#define TIM_DMABase_SR                     ((u16)0x0004)
#define TIM_DMABase_EGR                    ((u16)0x0005)
#define TIM_DMABase_CCMR1                  ((u16)0x0006)
#define TIM_DMABase_CCMR2                  ((u16)0x0007)
#define TIM_DMABase_CCER                   ((u16)0x0008)
#define TIM_DMABase_CNT                    ((u16)0x0009)
#define TIM_DMABase_PSC                    ((u16)0x000A)
#define TIM_DMABase_ARR                    ((u16)0x000B)
#define TIM_DMABase_RCR                    ((u16)0x000C)
#define TIM_DMABase_CCR1                   ((u16)0x000D)
#define TIM_DMABase_CCR2                   ((u16)0x000E)
#define TIM_DMABase_CCR3                   ((u16)0x000F)
#define TIM_DMABase_CCR4                   ((u16)0x0010)
#define TIM_DMABase_BDTR                   ((u16)0x0011)
#define TIM_DMABase_DCR                    ((u16)0x0012)

#define IS_TIM_DMA_BASE(BASE) (((BASE) == TIM_DMABase_CR1) || \
                               ((BASE) == TIM_DMABase_CR2) || \
                               ((BASE) == TIM_DMABase_SMCR) || \
                               ((BASE) == TIM_DMABase_DIER) || \
                               ((BASE) == TIM_DMABase_SR) || \
                               ((BASE) == TIM_DMABase_EGR) || \
                               ((BASE) == TIM_DMABase_CCMR1) || \
                               ((BASE) == TIM_DMABase_CCMR2) || \
                               ((BASE) == TIM_DMABase_CCER) || \
                               ((BASE) == TIM_DMABase_CNT) || \
                               ((BASE) == TIM_DMABase_PSC) || \
                               ((BASE) == TIM_DMABase_ARR) || \
                               ((BASE) == TIM_DMABase_RCR) || \
                               ((BASE) == TIM_DMABase_CCR1) || \
                               ((BASE) == TIM_DMABase_CCR2) || \
                               ((BASE) == TIM_DMABase_CCR3) || \
                               ((BASE) == TIM_DMABase_CCR4) || \
                               ((BASE) == TIM_DMABase_BDTR) || \
                               ((BASE) == TIM_DMABase_DCR))

/* TIM DMA Burst Length ----------------------------------------------------*/
#define TIM_DMABurstLength_1Byte           ((u16)0x0000)
#define TIM_DMABurstLength_2Bytes          ((u16)0x0100)
#define TIM_DMABurstLength_3Bytes          ((u16)0x0200)
#define TIM_DMABurstLength_4Bytes          ((u16)0x0300)
#define TIM_DMABurstLength_5Bytes          ((u16)0x0400)
#define TIM_DMABurstLength_6Bytes          ((u16)0x0500)
#define TIM_DMABurstLength_7Bytes          ((u16)0x0600)
#define TIM_DMABurstLength_8Bytes          ((u16)0x0700)
#define TIM_DMABurstLength_9Bytes          ((u16)0x0800)
#define TIM_DMABurstLength_10Bytes         ((u16)0x0900)
#define TIM_DMABurstLength_11Bytes         ((u16)0x0A00)
#define TIM_DMABurstLength_12Bytes         ((u16)0x0B00)
#define TIM_DMABurstLength_13Bytes         ((u16)0x0C00)
#define TIM_DMABurstLength_14Bytes         ((u16)0x0D00)
#define TIM_DMABurstLength_15Bytes         ((u16)0x0E00)
#define TIM_DMABurstLength_16Bytes         ((u16)0x0F00)
#define TIM_DMABurstLength_17Bytes         ((u16)0x1000)
#define TIM_DMABurstLength_18Bytes         ((u16)0x1100)

#define IS_TIM_DMA_LENGTH(LENGTH) (((LENGTH) == TIM_DMABurstLength_1Byte) || \
                                   ((LENGTH) == TIM_DMABurstLength_2Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_3Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_4Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_5Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_6Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_7Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_8Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_9Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_10Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_11Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_12Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_13Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_14Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_15Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_16Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_17Bytes) || \
                                   ((LENGTH) == TIM_DMABurstLength_18Bytes))

/* TIM DMA sources ---------------------------------------------------------*/
#define TIM_DMA_Update                     ((u16)0x0100)
#define TIM_DMA_CC1                        ((u16)0x0200)
#define TIM_DMA_CC2                        ((u16)0x0400)
#define TIM_DMA_CC3                        ((u16)0x0800)
#define TIM_DMA_CC4                        ((u16)0x1000)
#define TIM_DMA_COM                        ((u16)0x2000)
#define TIM_DMA_Trigger                    ((u16)0x4000)

#define IS_TIM_DMA_SOURCE(SOURCE) ((((SOURCE) & (u16)0x80FF) == 0x0000) && ((SOURCE) != 0x0000))

#define IS_TIM_PERIPH_DMA(PERIPH, SOURCE) ((((((*(u32*)&(PERIPH)) == TIM2_BASE) || (((*(u32*)&(PERIPH)) == TIM3_BASE))||\
                                            (((*(u32*)&(PERIPH)) == TIM4_BASE)) || (((*(u32*)&(PERIPH)) == TIM5_BASE))))&& \
                                            (((SOURCE) & (u16)0xA0FF) == 0x0000) && ((SOURCE) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM1_BASE) || (((*(u32*)&(PERIPH)) == TIM8_BASE))))&& \
                                            (((SOURCE) & (u16)0x80FF) == 0x0000) && ((SOURCE) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM6_BASE) || (((*(u32*)&(PERIPH)) == TIM7_BASE))))&& \
                                            (((SOURCE) & (u16)0xFEFF) == 0x0000) && ((SOURCE) != 0x0000)))

/* TIM External Trigger Prescaler ------------------------------------------*/
#define TIM_ExtTRGPSC_OFF                  ((u16)0x0000)
#define TIM_ExtTRGPSC_DIV2                 ((u16)0x1000)
#define TIM_ExtTRGPSC_DIV4                 ((u16)0x2000)
#define TIM_ExtTRGPSC_DIV8                 ((u16)0x3000)

#define IS_TIM_EXT_PRESCALER(PRESCALER) (((PRESCALER) == TIM_ExtTRGPSC_OFF) || \
                                         ((PRESCALER) == TIM_ExtTRGPSC_DIV2) || \
                                         ((PRESCALER) == TIM_ExtTRGPSC_DIV4) || \
                                         ((PRESCALER) == TIM_ExtTRGPSC_DIV8))

/* TIM Internal Trigger Selection ------------------------------------------*/
#define TIM_TS_ITR0                        ((u16)0x0000)
#define TIM_TS_ITR1                        ((u16)0x0010)
#define TIM_TS_ITR2                        ((u16)0x0020)
#define TIM_TS_ITR3                        ((u16)0x0030)
#define TIM_TS_TI1F_ED                     ((u16)0x0040)
#define TIM_TS_TI1FP1                      ((u16)0x0050)
#define TIM_TS_TI2FP2                      ((u16)0x0060)
#define TIM_TS_ETRF                        ((u16)0x0070)

#define IS_TIM_TRIGGER_SELECTION(SELECTION) (((SELECTION) == TIM_TS_ITR0) || \
                                             ((SELECTION) == TIM_TS_ITR1) || \
                                             ((SELECTION) == TIM_TS_ITR2) || \
                                             ((SELECTION) == TIM_TS_ITR3) || \
                                             ((SELECTION) == TIM_TS_TI1F_ED) || \
                                             ((SELECTION) == TIM_TS_TI1FP1) || \
                                             ((SELECTION) == TIM_TS_TI2FP2) || \
                                             ((SELECTION) == TIM_TS_ETRF))

#define IS_TIM_INTERNAL_TRIGGER_SELECTION(SELECTION) (((SELECTION) == TIM_TS_ITR0) || \
                                                      ((SELECTION) == TIM_TS_ITR1) || \
                                                      ((SELECTION) == TIM_TS_ITR2) || \
                                                      ((SELECTION) == TIM_TS_ITR3))

/* TIM TIx External Clock Source -------------------------------------------*/
#define TIM_TIxExternalCLK1Source_TI1      ((u16)0x0050)
#define TIM_TIxExternalCLK1Source_TI2      ((u16)0x0060)
#define TIM_TIxExternalCLK1Source_TI1ED    ((u16)0x0040)

#define IS_TIM_TIXCLK_SOURCE(SOURCE) (((SOURCE) == TIM_TIxExternalCLK1Source_TI1) || \
                                      ((SOURCE) == TIM_TIxExternalCLK1Source_TI2) || \
                                      ((SOURCE) == TIM_TIxExternalCLK1Source_TI1ED))

/* TIM External Trigger Polarity -------------------------------------------*/
#define TIM_ExtTRGPolarity_Inverted        ((u16)0x8000)
#define TIM_ExtTRGPolarity_NonInverted     ((u16)0x0000)

#define IS_TIM_EXT_POLARITY(POLARITY) (((POLARITY) == TIM_ExtTRGPolarity_Inverted) || \
                                       ((POLARITY) == TIM_ExtTRGPolarity_NonInverted))

/* TIM Prescaler Reload Mode -----------------------------------------------*/
#define TIM_PSCReloadMode_Update           ((u16)0x0000)
#define TIM_PSCReloadMode_Immediate        ((u16)0x0001)

#define IS_TIM_PRESCALER_RELOAD(RELOAD) (((RELOAD) == TIM_PSCReloadMode_Update) || \
                                         ((RELOAD) == TIM_PSCReloadMode_Immediate))

/* TIM Forced Action -------------------------------------------------------*/
#define TIM_ForcedAction_Active            ((u16)0x0050)
#define TIM_ForcedAction_InActive          ((u16)0x0040)

#define IS_TIM_FORCED_ACTION(ACTION) (((ACTION) == TIM_ForcedAction_Active) || \
                                      ((ACTION) == TIM_ForcedAction_InActive))

/* TIM Encoder Mode --------------------------------------------------------*/ 
#define TIM_EncoderMode_TI1                ((u16)0x0001)
#define TIM_EncoderMode_TI2                ((u16)0x0002)
#define TIM_EncoderMode_TI12               ((u16)0x0003)

#define IS_TIM_ENCODER_MODE(MODE) (((MODE) == TIM_EncoderMode_TI1) || \
                                   ((MODE) == TIM_EncoderMode_TI2) || \
                                   ((MODE) == TIM_EncoderMode_TI12))

/* TIM Event Source --------------------------------------------------------*/
#define TIM_EventSource_Update             ((u16)0x0001)
#define TIM_EventSource_CC1                ((u16)0x0002)
#define TIM_EventSource_CC2                ((u16)0x0004)
#define TIM_EventSource_CC3                ((u16)0x0008)
#define TIM_EventSource_CC4                ((u16)0x0010)
#define TIM_EventSource_COM                ((u16)0x0020)
#define TIM_EventSource_Trigger            ((u16)0x0040)
#define TIM_EventSource_Break              ((u16)0x0080)

#define IS_TIM_EVENT_SOURCE(SOURCE) ((((SOURCE) & (u16)0xFF00) == 0x0000) && ((SOURCE) != 0x0000))

#define IS_TIM_PERIPH_EVENT(PERIPH, EVENT) ((((((*(u32*)&(PERIPH)) == TIM2_BASE) || (((*(u32*)&(PERIPH)) == TIM3_BASE))||\
                                            (((*(u32*)&(PERIPH)) == TIM4_BASE)) || (((*(u32*)&(PERIPH)) == TIM5_BASE))))&& \
                                            (((EVENT) & (u16)0xFFA0) == 0x0000) && ((EVENT) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM1_BASE) || (((*(u32*)&(PERIPH)) == TIM8_BASE))))&& \
                                            (((EVENT) & (u16)0xFF00) == 0x0000) && ((EVENT) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM6_BASE) || (((*(u32*)&(PERIPH)) == TIM7_BASE))))&& \
                                            (((EVENT) & (u16)0xFFFE) == 0x0000) && ((EVENT) != 0x0000)))

/* TIM Update Source --------------------------------------------------------*/
#define TIM_UpdateSource_Global            ((u16)0x0000)
#define TIM_UpdateSource_Regular           ((u16)0x0001)

#define IS_TIM_UPDATE_SOURCE(SOURCE) (((SOURCE) == TIM_UpdateSource_Global) || \
                                      ((SOURCE) == TIM_UpdateSource_Regular))

/* TIM Ouput Compare Preload State ------------------------------------------*/
#define TIM_OCPreload_Enable               ((u16)0x0008)
#define TIM_OCPreload_Disable              ((u16)0x0000)

#define IS_TIM_OCPRELOAD_STATE(STATE) (((STATE) == TIM_OCPreload_Enable) || \
                                       ((STATE) == TIM_OCPreload_Disable))

/* TIM Ouput Compare Fast State ---------------------------------------------*/
#define TIM_OCFast_Enable                  ((u16)0x0004)
#define TIM_OCFast_Disable                 ((u16)0x0000)

#define IS_TIM_OCFAST_STATE(STATE) (((STATE) == TIM_OCFast_Enable) || \
                                    ((STATE) == TIM_OCFast_Disable))
                                     
/* TIM Ouput Compare Clear State --------------------------------------------*/
#define TIM_OCClear_Enable                 ((u16)0x0080)
#define TIM_OCClear_Disable                ((u16)0x0000)

#define IS_TIM_OCCLEAR_STATE(STATE) (((STATE) == TIM_OCClear_Enable) || \
                                     ((STATE) == TIM_OCClear_Disable))                                     

/* TIM Trigger Output Source ------------------------------------------------*/ 
#define TIM_TRGOSource_Reset               ((u16)0x0000)
#define TIM_TRGOSource_Enable              ((u16)0x0010)
#define TIM_TRGOSource_Update              ((u16)0x0020)
#define TIM_TRGOSource_OC1                 ((u16)0x0030)
#define TIM_TRGOSource_OC1Ref              ((u16)0x0040)
#define TIM_TRGOSource_OC2Ref              ((u16)0x0050)
#define TIM_TRGOSource_OC3Ref              ((u16)0x0060)
#define TIM_TRGOSource_OC4Ref              ((u16)0x0070)

#define IS_TIM_TRGO_SOURCE(SOURCE) (((SOURCE) == TIM_TRGOSource_Reset) || \
                                    ((SOURCE) == TIM_TRGOSource_Enable) || \
                                    ((SOURCE) == TIM_TRGOSource_Update) || \
                                    ((SOURCE) == TIM_TRGOSource_OC1) || \
                                    ((SOURCE) == TIM_TRGOSource_OC1Ref) || \
                                    ((SOURCE) == TIM_TRGOSource_OC2Ref) || \
                                    ((SOURCE) == TIM_TRGOSource_OC3Ref) || \
                                    ((SOURCE) == TIM_TRGOSource_OC4Ref))

#define IS_TIM_PERIPH_TRGO(PERIPH, TRGO)  (((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM6_BASE))||(((*(u32*)&(PERIPH)) == TIM7_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_Reset)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM6_BASE))||(((*(u32*)&(PERIPH)) == TIM7_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_Enable)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM6_BASE))||(((*(u32*)&(PERIPH)) == TIM7_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_Update)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_OC1)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_OC1Ref)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_OC2Ref)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_OC3Ref)) ||\
                                           ((((*(u32*)&(PERIPH)) == TIM2_BASE)||(((*(u32*)&(PERIPH)) == TIM1_BASE))||\
                                           (((*(u32*)&(PERIPH)) == TIM3_BASE))||(((*(u32*)&(PERIPH)) == TIM4_BASE))|| \
                                           (((*(u32*)&(PERIPH)) == TIM5_BASE))||(((*(u32*)&(PERIPH)) == TIM8_BASE))) && \
                                           ((TRGO) == TIM_TRGOSource_OC4Ref)))

/* TIM Slave Mode ----------------------------------------------------------*/
#define TIM_SlaveMode_Reset                ((u16)0x0004)
#define TIM_SlaveMode_Gated                ((u16)0x0005)
#define TIM_SlaveMode_Trigger              ((u16)0x0006)
#define TIM_SlaveMode_External1            ((u16)0x0007)

#define IS_TIM_SLAVE_MODE(MODE) (((MODE) == TIM_SlaveMode_Reset) || \
                                 ((MODE) == TIM_SlaveMode_Gated) || \
                                 ((MODE) == TIM_SlaveMode_Trigger) || \
                                 ((MODE) == TIM_SlaveMode_External1))

/* TIM Master Slave Mode ---------------------------------------------------*/
#define TIM_MasterSlaveMode_Enable         ((u16)0x0080)
#define TIM_MasterSlaveMode_Disable        ((u16)0x0000)

#define IS_TIM_MSM_STATE(STATE) (((STATE) == TIM_MasterSlaveMode_Enable) || \
                                 ((STATE) == TIM_MasterSlaveMode_Disable))

/* TIM Flags ---------------------------------------------------------------*/
#define TIM_FLAG_Update                    ((u16)0x0001)
#define TIM_FLAG_CC1                       ((u16)0x0002)
#define TIM_FLAG_CC2                       ((u16)0x0004)
#define TIM_FLAG_CC3                       ((u16)0x0008)
#define TIM_FLAG_CC4                       ((u16)0x0010)
#define TIM_FLAG_COM                       ((u16)0x0020)
#define TIM_FLAG_Trigger                   ((u16)0x0040)
#define TIM_FLAG_Break                     ((u16)0x0080)
#define TIM_FLAG_CC1OF                     ((u16)0x0200)
#define TIM_FLAG_CC2OF                     ((u16)0x0400)
#define TIM_FLAG_CC3OF                     ((u16)0x0800)
#define TIM_FLAG_CC4OF                     ((u16)0x1000)

#define IS_TIM_GET_FLAG(FLAG) (((FLAG) == TIM_FLAG_Update) || \
                               ((FLAG) == TIM_FLAG_CC1) || \
                               ((FLAG) == TIM_FLAG_CC2) || \
                               ((FLAG) == TIM_FLAG_CC3) || \
                               ((FLAG) == TIM_FLAG_CC4) || \
                               ((FLAG) == TIM_FLAG_COM) || \
                               ((FLAG) == TIM_FLAG_Trigger) || \
                               ((FLAG) == TIM_FLAG_Break) || \
                               ((FLAG) == TIM_FLAG_CC1OF) || \
                               ((FLAG) == TIM_FLAG_CC2OF) || \
                               ((FLAG) == TIM_FLAG_CC3OF) || \
                               ((FLAG) == TIM_FLAG_CC4OF))

#define IS_TIM_CLEAR_FLAG(PERIPH, TIM_FLAG) ((((((*(u32*)&(PERIPH)) == TIM2_BASE) || (((*(u32*)&(PERIPH)) == TIM3_BASE))||\
                                            (((*(u32*)&(PERIPH)) == TIM4_BASE)) || (((*(u32*)&(PERIPH)) == TIM5_BASE))))&& \
                                            (((TIM_FLAG) & (u16)0xE1A0) == 0x0000) && ((TIM_FLAG) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM1_BASE) || (((*(u32*)&(PERIPH)) == TIM8_BASE))))&& \
                                            (((TIM_FLAG) & (u16)0xE100) == 0x0000) && ((TIM_FLAG) != 0x0000)) ||\
                                            (((((*(u32*)&(PERIPH)) == TIM6_BASE) || (((*(u32*)&(PERIPH)) == TIM7_BASE))))&& \
                                            (((TIM_FLAG) & (u16)0xFFFE) == 0x0000) && ((TIM_FLAG) != 0x0000)))

#define IS_TIM_PERIPH_FLAG(PERIPH, TIM_FLAG)  (((((*(u32*)&(PERIPH))==TIM2_BASE) || ((*(u32*)&(PERIPH)) == TIM3_BASE) ||\
                                                 ((*(u32*)&(PERIPH)) == TIM4_BASE) || ((*(u32*)&(PERIPH))==TIM5_BASE) || \
                                                 ((*(u32*)&(PERIPH))==TIM1_BASE) || ((*(u32*)&(PERIPH))==TIM8_BASE)) &&\
                                                 (((TIM_FLAG) == TIM_FLAG_CC1) || ((TIM_FLAG) == TIM_FLAG_CC2) ||\
                                                 ((TIM_FLAG) == TIM_FLAG_CC3) || ((TIM_FLAG) == TIM_FLAG_CC4) || \
                                                 ((TIM_FLAG) == TIM_FLAG_Trigger))) ||\
                                                 ((((*(u32*)&(PERIPH))==TIM2_BASE) || ((*(u32*)&(PERIPH)) == TIM3_BASE) || \
                                                 ((*(u32*)&(PERIPH)) == TIM4_BASE) || ((*(u32*)&(PERIPH))==TIM5_BASE) ||\
                                                 ((*(u32*)&(PERIPH))==TIM1_BASE)|| ((*(u32*)&(PERIPH))==TIM8_BASE) || \
                                                 ((*(u32*)&(PERIPH))==TIM7_BASE) || ((*(u32*)&(PERIPH))==TIM6_BASE)) && \
                                                 (((TIM_FLAG) == TIM_FLAG_Update))) ||\
                                                 ((((*(u32*)&(PERIPH))==TIM1_BASE) || ((*(u32*)&(PERIPH)) == TIM8_BASE)) &&\
                                                 (((TIM_FLAG) == TIM_FLAG_COM) || ((TIM_FLAG) == TIM_FLAG_Break))) ||\
                                                 ((((*(u32*)&(PERIPH))==TIM2_BASE) || ((*(u32*)&(PERIPH)) == TIM3_BASE) || \
                                                 ((*(u32*)&(PERIPH)) == TIM4_BASE) || ((*(u32*)&(PERIPH))==TIM5_BASE) || \
                                                 ((*(u32*)&(PERIPH))==TIM1_BASE) || ((*(u32*)&(PERIPH))==TIM8_BASE)) &&\
                                                 (((TIM_FLAG) == TIM_FLAG_CC1OF) || ((TIM_FLAG) == TIM_FLAG_CC2OF) ||\
                                                 ((TIM_FLAG) == TIM_FLAG_CC3OF) || ((TIM_FLAG) == TIM_FLAG_CC4OF))))             
                                                                                            
/* TIM Input Capture Filer Value ---------------------------------------------*/
#define IS_TIM_IC_FILTER(ICFILTER) ((ICFILTER) <= 0xF) 

/* TIM External Trigger Filter -----------------------------------------------*/
#define IS_TIM_EXT_FILTER(EXTFILTER) ((EXTFILTER) <= 0xF)                              

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, u16 TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, u16 TIM_EventSource);
void TIM_DMAConfig(TIM_TypeDef* TIMx, u16 TIM_DMABase, u16 TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, u16 TIM_DMASource, FunctionalState NewState);
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, u16 TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, u16 TIM_TIxExternalCLKSource,
                                u16 TIM_ICPolarity, u16 ICFilter);                                
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, u16 TIM_ExtTRGPrescaler, u16 TIM_ExtTRGPolarity,
                             u16 ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, u16 TIM_ExtTRGPrescaler, 
                             u16 TIM_ExtTRGPolarity, u16 ExtTRGFilter);
void TIM_ETRConfig(TIM_TypeDef* TIMx, u16 TIM_ExtTRGPrescaler, u16 TIM_ExtTRGPolarity,
                   u16 ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, u16 Prescaler, u16 TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, u16 TIM_CounterMode);
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, u16 TIM_InputTriggerSource);
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, u16 TIM_EncoderMode,
                                u16 TIM_IC1Polarity, u16 TIM_IC2Polarity);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, u16 TIM_Channel, u16 TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, u16 TIM_Channel, u16 TIM_CCxN);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, u16 TIM_Channel, u16 TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, u16 TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, u16 TIM_OPMode);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, u16 TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, u16 TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, u16 TIM_MasterSlaveMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, u16 Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, u16 Autoreload);
void TIM_SetCompare1(TIM_TypeDef* TIMx, u16 Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, u16 Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, u16 Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, u16 Compare4);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, u16 TIM_CKD);
u16 TIM_GetCapture1(TIM_TypeDef* TIMx);
u16 TIM_GetCapture2(TIM_TypeDef* TIMx);
u16 TIM_GetCapture3(TIM_TypeDef* TIMx);
u16 TIM_GetCapture4(TIM_TypeDef* TIMx);
u16 TIM_GetCounter(TIM_TypeDef* TIMx);
u16 TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, u16 TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, u16 TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, u16 TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, u16 TIM_IT);
                                                                                                             
#endif /*__STM32F10x_TIM_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


























/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_type.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the common data types used for the
*                      STM32F10x firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
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

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_usart.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      USART firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_USART_H
#define __STM32F10x_USART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* USART Init Structure definition */
typedef struct
{
  u32 USART_BaudRate;
  u16 USART_WordLength;
  u16 USART_StopBits;
  u16 USART_Parity;
  u16 USART_Mode;
  u16 USART_HardwareFlowControl;  
} USART_InitTypeDef;

/* USART Clock Init Structure definition */
typedef struct
{
  u16 USART_Clock;
  u16 USART_CPOL;
  u16 USART_CPHA;
  u16 USART_LastBit;
} USART_ClockInitTypeDef;

/* Exported constants --------------------------------------------------------*/
#define IS_USART_ALL_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == USART1_BASE) || \
                                     ((*(u32*)&(PERIPH)) == USART2_BASE) || \
                                     ((*(u32*)&(PERIPH)) == USART3_BASE) || \
                                     ((*(u32*)&(PERIPH)) == UART4_BASE) || \
                                     ((*(u32*)&(PERIPH)) == UART5_BASE))

#define IS_USART_123_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == USART1_BASE) || \
                                     ((*(u32*)&(PERIPH)) == USART2_BASE) || \
                                     ((*(u32*)&(PERIPH)) == USART3_BASE))

#define IS_USART_1234_PERIPH(PERIPH) (((*(u32*)&(PERIPH)) == USART1_BASE) || \
                                      ((*(u32*)&(PERIPH)) == USART2_BASE) || \
                                      ((*(u32*)&(PERIPH)) == USART3_BASE) || \
                                      ((*(u32*)&(PERIPH)) == UART4_BASE))

/* USART Word Length ---------------------------------------------------------*/
#define USART_WordLength_8b                  ((u16)0x0000)
#define USART_WordLength_9b                  ((u16)0x1000)
                                    
#define IS_USART_WORD_LENGTH(LENGTH) (((LENGTH) == USART_WordLength_8b) || \
                                      ((LENGTH) == USART_WordLength_9b))

/* USART Stop Bits -----------------------------------------------------------*/
#define USART_StopBits_1                     ((u16)0x0000)
#define USART_StopBits_0_5                   ((u16)0x1000)
#define USART_StopBits_2                     ((u16)0x2000)
#define USART_StopBits_1_5                   ((u16)0x3000)

#define IS_USART_STOPBITS(STOPBITS) (((STOPBITS) == USART_StopBits_1) || \
                                     ((STOPBITS) == USART_StopBits_0_5) || \
                                     ((STOPBITS) == USART_StopBits_2) || \
                                     ((STOPBITS) == USART_StopBits_1_5))
/* USART Parity --------------------------------------------------------------*/
#define USART_Parity_No                      ((u16)0x0000)
#define USART_Parity_Even                    ((u16)0x0400)
#define USART_Parity_Odd                     ((u16)0x0600) 

#define IS_USART_PARITY(PARITY) (((PARITY) == USART_Parity_No) || \
                                 ((PARITY) == USART_Parity_Even) || \
                                 ((PARITY) == USART_Parity_Odd))

/* USART Mode ----------------------------------------------------------------*/
#define USART_Mode_Rx                        ((u16)0x0004)
#define USART_Mode_Tx                        ((u16)0x0008)

#define IS_USART_MODE(MODE) ((((MODE) & (u16)0xFFF3) == 0x00) && ((MODE) != (u16)0x00))

/* USART Hardware Flow Control -----------------------------------------------*/
#define USART_HardwareFlowControl_None       ((u16)0x0000)
#define USART_HardwareFlowControl_RTS        ((u16)0x0100)
#define USART_HardwareFlowControl_CTS        ((u16)0x0200)
#define USART_HardwareFlowControl_RTS_CTS    ((u16)0x0300)

#define IS_USART_HARDWARE_FLOW_CONTROL(CONTROL)\
                              (((CONTROL) == USART_HardwareFlowControl_None) || \
                               ((CONTROL) == USART_HardwareFlowControl_RTS) || \
                               ((CONTROL) == USART_HardwareFlowControl_CTS) || \
                               ((CONTROL) == USART_HardwareFlowControl_RTS_CTS))

#define IS_USART_PERIPH_HFC(PERIPH, HFC) ((((*(u32*)&(PERIPH)) != UART4_BASE) && \
                                          ((*(u32*)&(PERIPH)) != UART5_BASE)) \
                                          || ((HFC) == USART_HardwareFlowControl_None))                                

/* USART Clock ---------------------------------------------------------------*/
#define USART_Clock_Disable                  ((u16)0x0000)
#define USART_Clock_Enable                   ((u16)0x0800)

#define IS_USART_CLOCK(CLOCK) (((CLOCK) == USART_Clock_Disable) || \
                               ((CLOCK) == USART_Clock_Enable))

/* USART Clock Polarity ------------------------------------------------------*/
#define USART_CPOL_Low                       ((u16)0x0000)
#define USART_CPOL_High                      ((u16)0x0400)

#define IS_USART_CPOL(CPOL) (((CPOL) == USART_CPOL_Low) || ((CPOL) == USART_CPOL_High))
                               
/* USART Clock Phase ---------------------------------------------------------*/
#define USART_CPHA_1Edge                     ((u16)0x0000)
#define USART_CPHA_2Edge                     ((u16)0x0200)
#define IS_USART_CPHA(CPHA) (((CPHA) == USART_CPHA_1Edge) || ((CPHA) == USART_CPHA_2Edge))

/* USART Last Bit ------------------------------------------------------------*/
#define USART_LastBit_Disable                ((u16)0x0000)
#define USART_LastBit_Enable                 ((u16)0x0100)

#define IS_USART_LASTBIT(LASTBIT) (((LASTBIT) == USART_LastBit_Disable) || \
                                   ((LASTBIT) == USART_LastBit_Enable))

/* USART Interrupt definition ------------------------------------------------*/
#define USART_IT_PE                          ((u16)0x0028)
#define USART_IT_TXE                         ((u16)0x0727)
#define USART_IT_TC                          ((u16)0x0626)
#define USART_IT_RXNE                        ((u16)0x0525)
#define USART_IT_IDLE                        ((u16)0x0424)
#define USART_IT_LBD                         ((u16)0x0846)
#define USART_IT_CTS                         ((u16)0x096A)
#define USART_IT_ERR                         ((u16)0x0060)
#define USART_IT_ORE                         ((u16)0x0360)
#define USART_IT_NE                          ((u16)0x0260)
#define USART_IT_FE                          ((u16)0x0160)

#define IS_USART_CONFIG_IT(IT) (((IT) == USART_IT_PE) || ((IT) == USART_IT_TXE) || \
                               ((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                               ((IT) == USART_IT_IDLE) || ((IT) == USART_IT_LBD) || \
                               ((IT) == USART_IT_CTS) || ((IT) == USART_IT_ERR))

#define IS_USART_GET_IT(IT) (((IT) == USART_IT_PE) || ((IT) == USART_IT_TXE) || \
                            ((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                            ((IT) == USART_IT_IDLE) || ((IT) == USART_IT_LBD) || \
                            ((IT) == USART_IT_CTS) || ((IT) == USART_IT_ORE) || \
                            ((IT) == USART_IT_NE) || ((IT) == USART_IT_FE))

#define IS_USART_CLEAR_IT(IT) (((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                               ((IT) == USART_IT_LBD) || ((IT) == USART_IT_CTS))

#define IS_USART_PERIPH_IT(PERIPH, USART_IT) ((((*(u32*)&(PERIPH)) != UART4_BASE) && \
                                              ((*(u32*)&(PERIPH)) != UART5_BASE)) \
                                              || ((USART_IT) != USART_IT_CTS))                                                                           

/* USART DMA Requests --------------------------------------------------------*/
#define USART_DMAReq_Tx                      ((u16)0x0080)
#define USART_DMAReq_Rx                      ((u16)0x0040)

#define IS_USART_DMAREQ(DMAREQ) ((((DMAREQ) & (u16)0xFF3F) == 0x00) && ((DMAREQ) != (u16)0x00))

/* USART WakeUp methods ------------------------------------------------------*/
#define USART_WakeUp_IdleLine                ((u16)0x0000)
#define USART_WakeUp_AddressMark             ((u16)0x0800)

#define IS_USART_WAKEUP(WAKEUP) (((WAKEUP) == USART_WakeUp_IdleLine) || \
                                 ((WAKEUP) == USART_WakeUp_AddressMark))

/* USART LIN Break Detection Length ------------------------------------------*/
#define USART_LINBreakDetectLength_10b      ((u16)0x0000)
#define USART_LINBreakDetectLength_11b      ((u16)0x0020)

#define IS_USART_LIN_BREAK_DETECT_LENGTH(LENGTH) \
                               (((LENGTH) == USART_LINBreakDetectLength_10b) || \
                                ((LENGTH) == USART_LINBreakDetectLength_11b))

/* USART IrDA Low Power ------------------------------------------------------*/
#define USART_IrDAMode_LowPower              ((u16)0x0004)
#define USART_IrDAMode_Normal                ((u16)0x0000)

#define IS_USART_IRDA_MODE(MODE) (((MODE) == USART_IrDAMode_LowPower) || \
                                  ((MODE) == USART_IrDAMode_Normal))

/* USART Flags ---------------------------------------------------------------*/
#define USART_FLAG_CTS                       ((u16)0x0200)
#define USART_FLAG_LBD                       ((u16)0x0100)
#define USART_FLAG_TXE                       ((u16)0x0080)
#define USART_FLAG_TC                        ((u16)0x0040)
#define USART_FLAG_RXNE                      ((u16)0x0020)
#define USART_FLAG_IDLE                      ((u16)0x0010)
#define USART_FLAG_ORE                       ((u16)0x0008)
#define USART_FLAG_NE                        ((u16)0x0004)
#define USART_FLAG_FE                        ((u16)0x0002)
#define USART_FLAG_PE                        ((u16)0x0001)

#define IS_USART_FLAG(FLAG) (((FLAG) == USART_FLAG_PE) || ((FLAG) == USART_FLAG_TXE) || \
                             ((FLAG) == USART_FLAG_TC) || ((FLAG) == USART_FLAG_RXNE) || \
                             ((FLAG) == USART_FLAG_IDLE) || ((FLAG) == USART_FLAG_LBD) || \
                             ((FLAG) == USART_FLAG_CTS) || ((FLAG) == USART_FLAG_ORE) || \
                             ((FLAG) == USART_FLAG_NE) || ((FLAG) == USART_FLAG_FE))
                              
#define IS_USART_CLEAR_FLAG(FLAG) ((((FLAG) & (u16)0xFC9F) == 0x00) && ((FLAG) != (u16)0x00))

#define IS_USART_PERIPH_FLAG(PERIPH, USART_FLAG) ((((*(u32*)&(PERIPH)) != UART4_BASE) &&\
                                                  ((*(u32*)&(PERIPH)) != UART5_BASE)) \
                                                  || ((USART_FLAG) != USART_FLAG_CTS)) 

#define IS_USART_BAUDRATE(BAUDRATE) (((BAUDRATE) > 0) && ((BAUDRATE) < 0x0044AA21))
#define IS_USART_ADDRESS(ADDRESS) ((ADDRESS) <= 0xF)
#define IS_USART_DATA(DATA) ((DATA) <= 0x1FF)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, u16 USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, u16 USART_DMAReq, FunctionalState NewState);
void USART_SetAddress(USART_TypeDef* USARTx, u8 USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, u16 USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, u16 USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendData(USART_TypeDef* USARTx, u16 Data);
u16 USART_ReceiveData(USART_TypeDef* USARTx);
void USART_SendBreak(USART_TypeDef* USARTx);
void USART_SetGuardTime(USART_TypeDef* USARTx, u8 USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef* USARTx, u8 USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_IrDAConfig(USART_TypeDef* USARTx, u16 USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, u16 USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, u16 USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, u16 USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, u16 USART_IT);

#endif /* __STM32F10x_USART_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_wwdg.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      WWDG firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_WWDG_H
#define __STM32F10x_WWDG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_map.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* WWDG Prescaler */
#define WWDG_Prescaler_1    ((u32)0x00000000)
#define WWDG_Prescaler_2    ((u32)0x00000080)
#define WWDG_Prescaler_4    ((u32)0x00000100)
#define WWDG_Prescaler_8    ((u32)0x00000180)

#define IS_WWDG_PRESCALER(PRESCALER) (((PRESCALER) == WWDG_Prescaler_1) || \
                                      ((PRESCALER) == WWDG_Prescaler_2) || \
                                      ((PRESCALER) == WWDG_Prescaler_4) || \
                                      ((PRESCALER) == WWDG_Prescaler_8))

#define IS_WWDG_WINDOW_VALUE(VALUE) ((VALUE) <= 0x7F)

#define IS_WWDG_COUNTER(COUNTER) (((COUNTER) >= 0x40) && ((COUNTER) <= 0x7F))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void WWDG_DeInit(void);
void WWDG_SetPrescaler(u32 WWDG_Prescaler);
void WWDG_SetWindowValue(u8 WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(u8 Counter);
void WWDG_Enable(u8 Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);

#endif /* __STM32F10x_WWDG_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/





















/* Define to prevent recursive inclusion */
#ifndef SYSCLOCK_H
#define SYSCLOCK_H

/* Includes */
#include "stm32f10x_lib.h"



/* Function Prototypes */
void SYSCLOCK_Init(void (*ptr2Func)(void));
void SYSCLOCK_Enable(void);
void SYSCLOCK_Disable(void);
void SYSCLOCK_SetTimeSpan(u32 timeSpan);
void SysTickHandler(void);



#endif




















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_math.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the function prototypes for the
*                    : math related functions of the firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_MATH_H
#define __VN_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define VN_PI              3.14159265
#define VN_INV_MAX_SIZE    9

/* Exported macro ------------------------------------------------------------*/
#define VN_RAD2DEG(deg)    (deg*180.0f/VN_PI)
#define VN_DEG2RAD(rad)   (rad*VN_PI/180.0f)
#define VN_SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}                                   

/* Matrix macros -------------------------------------------------------------*/
/* The following macros are used to allow for the creation of statically
   allocated matricies.  The matrix representation provide here consists
   of three variables for each matrix. They are as follows:
   
   VariableName_data : The data for the given matrix
   VariableName_ptr  : A array of pointers to the first element in each row
   VariableName      : A pointer to the VariableName_ptr. In other words a
                       pointer to the array of pointers of the column vectors
                       of the array.
                       
   Example: Lets say you want to create a 3x3 matrix. You could do this using
            the following three lines of code:
            
   float  A_data[9] = {0};
   float  A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
   float  **A = A_ptr;
   
   Now if you want to use any of the elements in the array you can do so as
   you normally would if it was a standard two-dimensional array.
   
   // Example - set the diagonal elements to one
   A[0][0] = 1.0f;
   A[1][1] = 1.0f;
   A[2][2] = 1.0f;
   
   You might be asking why not just initialize the array as a standard two
   dimensional array such as:
   
   float A[3][3];
   
   The reason for this is well explained in page 20 of the book titled
   "Numerical Recipes in C - The Art of Scientific Computing" Second
   Edition. In a nutshell the standard two dimensional arrays of C are
   not efficient for numerical methods because of the way in which C 
   references the array indicies. Take for example the following line
   of code:
   
   A[i][j] = 1.0f;
   
   When this line is compiled to assembly it will result in something similar
   to the following line of code:
   
   *(A + 3*i + j) = 1.0f;
   
   This means "to the address of A add 3 times i plus j and return 
   the value thus addressed. This code results in two integer adds and one
   integer multiplication for every array index reference. An alternative 
   method that is more efficient is to have a pointer to an array of
   pointers given as **A.  This will result in code similar to: 
   
   *(*(A+i) + j) = 1.0f;
   
   This means "to the address of A add i, take the value thus addressed as
   a new address, add j to it, return the value addressed by this new
   address". In this version only two integer adds are required, which
   eliminates a integer multiplication.  This will result in more
   efficient embedded code. The penalty paid for this more efficient method
   is that it will require the storage of additional terms in the form of a
   array of pointers to the rows of the matrix. This is a slight
   inconvenience as it will require that the user to initialize three
   variables as opposed to one for each matrix.  The macros provided here 
   are designed to simplify this process. A matrix can be initialized using
   these macros as shown below:
   
   CreateMatrix(A, 3, 3, {1, 0, 0, 0, 1, 0, 0, 0, 1});
   
   This line will result in the folloing lines of code:
   
   static float A_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
   static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
   static float       **A = A_ptr;
   
   The macros given below will only work for matricies with sizes less
   than 12 in each dimension.   These macros are only compatible with C99 and
   above versions of C. If you are using strict ANSI C then you will need to 
   write out the three lines shown above for each matrix you want to
   initialize.
   */
#if __STDC_VERSION__ >= 199901L  /* Determine if compiler is C99 compatible */
#define VN_MATRIXPtrList_1(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0]}
#define VN_MATRIXPtrList_2(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)]}
#define VN_MATRIXPtrList_3(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)]}
#define VN_MATRIXPtrList_4(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)]}
#define VN_MATRIXPtrList_5(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)]}
#define VN_MATRIXPtrList_6(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)]}
#define VN_MATRIXPtrList_7(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)]}
#define VN_MATRIXPtrList_8(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)]}
#define VN_MATRIXPtrList_9(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)]}
#define VN_MATRIXPtrList_10(VN_MATRIX, COLS)  {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)], &VN_MATRIX##_data[9*(COLS)]}
#define VN_MATRIXPtrList_11(VN_MATRIX, COLS)  {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)], &VN_MATRIX##_data[9*(COLS)], &VN_MATRIX##_data[10(COLS)]}
#define VN_MATRIXPtrList_12(VN_MATRIX, COLS)  {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)], &VN_MATRIX##_data[9*(COLS)], &VN_MATRIX##_data[10(COLS)], &VN_MATRIX##_data[11(COLS)]}

#define VN_CreateMatrix(VN_MATRIX, ROWS, COLS, ...)    static float   VN_MATRIX##_data[(ROWS) * (COLS)] = __VA_ARGS__;                \
                                                      static float*  VN_MATRIX##_ptr[] = VN_MATRIXPtrList_##ROWS(VN_MATRIX, COLS);  \
                                                      static float** VN_MATRIX = VN_MATRIX##_ptr
#endif
/* Exported functions ------------------------------------------------------- */

void VN_CrossP(float *A, float *B, float *C);
void VN_VecAdd(float *A, float *B, unsigned long rows, float *C);
void VN_VecSub(float *A, float *B, unsigned long rows, float *C);
void VN_VecMultT(float *A, float *BT, unsigned long rows, float **C);
void VN_Identity(float scalar, unsigned long Arows, unsigned long Acols, float **A);
void VN_MatAdd(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C);
void VN_MatSub(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C);
void VN_MatMult(float **A, float **B, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C);
void VN_MatMultMT(float **A, float **BT, unsigned long Arows, unsigned long Acols, unsigned long Brows, float **C);
void VN_MatScalarMult(double **A, double scalar, unsigned long Arows, unsigned long Acols, double **C);
void VN_MatVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C);
void VN_MatTVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C);
void VN_MatCopy(float **A, unsigned long nrows, unsigned long ncols, float **B);
void VN_MatInv(float **A, signed long n, float **B);
void VN_SkewMatrix(float *V, float **A);
void VN_Transpose(float **A, unsigned long Arows, unsigned long Acols, float **B);
float VN_Norm(float *A, unsigned long m);
void VN_Normalize(float *V1, unsigned long m, float *V2);
void VN_TriU2TriL(float **A, unsigned long rows);
void VN_Quat2DCM(float *q, float **A);
void VN_YPR2DCM(float *YPR, float **A);
void VN_MatZeros(float **A, unsigned long Arows, unsigned long Acols);
void VN_Quat2Euler121(float *q, float *Euler121);
void VN_Quat2Euler123(float *q, float *Euler123);
void VN_Quat2Euler131(float *q, float *Euler131);
void VN_Quat2Euler132(float *q, float *Euler132);
void VN_Quat2Euler212(float *q, float *Euler212);
void VN_Quat2Euler213(float *q, float *Euler213);
void VN_Quat2Euler231(float *q, float *Euler231);
void VN_Quat2Euler232(float *q, float *Euler232);
void VN_Quat2Euler312(float *q, float *Euler312);
void VN_Quat2Euler313(float *q, float *Euler313);
void VN_Quat2Euler321(float *q, float *Euler321);
void VN_Quat2Euler323(float *q, float *Euler323);
void VN_Quat2Gibbs(float *q, float *Gibb);
void VN_Quat2MRP(float *q, float *MRP);
void VN_Quat2PRV(float *q, float *PRV);
void VN_AddQuat(float *q1, float *q2, float *q3);
void VN_SubQuat(float *q1, float *q2, float *q3);
void VN_QuatKinematicDiffEq(float *q, float *rates, float *q_dot);
void VN_YPRKinematicDiffEq(float *YPR, float *rates, float *YPR_dot);


#endif /* __VN_MATH_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/



















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_type.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the common data types used for the
*                    : firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

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

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/





















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_user.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This is the user configuration header file. It contains
*                    : setup parameters and all the function prototypes for the 
*                    : methods that are hardware specific. These methods need to
*                    : be modified by the user in the file VN_user.c to be
*                    : compatible with their specific hardware architecture.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_USER_H
#define __VN_USER_H 

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Comment the lines below to disable the specific device inclusion */

/*********************************** VN-100 ***********************************/
#define _VN100


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void VN_SPI_SetSS(unsigned char sensorID, VN_PinState state);
unsigned long VN_SPI_SendReceive(unsigned long data);
void VN_Delay(unsigned long delay_uS);

#endif /* __VN_USER_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/



















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN100.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the function prototypes for
*                    : the firmware specific to the VN-100.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN100_H
#define __VN100_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"
#include "VN_lib.h"

/* Exported constants --------------------------------------------------------*/

/* VN-100 Registers */
#define     VN100_REG_MODEL     1
#define     VN100_REG_HWREV     2
#define     VN100_REG_SN        3
#define     VN100_REG_FWVER     4
#define     VN100_REG_SBAUD     5
#define     VN100_REG_ADOR      6
#define     VN100_REG_ADOF      7
#define     VN100_REG_YPR       8
#define     VN100_REG_QTN       9
#define     VN100_REG_QTM       10
#define     VN100_REG_QTA       11
#define     VN100_REG_QTR       12
#define     VN100_REG_QMA       13
#define     VN100_REG_QAR       14
#define     VN100_REG_QMR       15
#define     VN100_REG_DCM       16
#define     VN100_REG_MAG       17
#define     VN100_REG_ACC       18
#define     VN100_REG_GYR       19
#define     VN100_REG_MAR       20
#define     VN100_REG_REF       21
#define     VN100_REG_SIG       22
#define     VN100_REG_HSI       23
#define     VN100_REG_ATP       24
#define     VN100_REG_ACT       25
#define     VN100_REG_RFR       26
#define     VN100_REG_YMR       27
#define     VN100_REG_ACG       28
  
#define     VN100_REG_RAW         251
#define     VN100_REG_CMV       252
#define     VN100_REG_STV         253
#define     VN100_REG_COV       254
#define     VN100_REG_CAL         255

/* SPI Buffer size */
#define     VN100_SPI_BUFFER_SIZE    93

/* Exported types ------------------------------------------------------------*/
/* Command IDs */
typedef enum VN100_CmdID
{
  VN100_CmdID_ReadRegister             =  0x01,
  VN100_CmdID_WriteRegister            =  0x02,
  VN100_CmdID_WriteSettings            =  0x03,
  VN100_CmdID_RestoreFactorySettings   =  0x04,
  VN100_CmdID_Tare                     = 0x05,
  VN100_CmdID_Reset                    = 0x06,
  VN100_CmdID_FlashFirmware            =  0x07,
  VN100_CmdID_SetRefFrame              = 0x08,
  VN100_CmdID_HardwareInLoop           =  0x09,
  VN100_CmdID_GetFlashCNT              =  0x0A,
  VN100_CmdID_Calibrate                = 0x0B  
} VN100_CmdID;

/* System Error */
typedef enum VN100_Error
{
  VN100_Error_None                      = 0,
  VN100_Error_ErrorListOverflow          = 255,
  VN100_Error_HardFaultException        = 1,
  VN100_Error_InputBufferOverflow        =  2,
  VN100_Error_InvalidChecksum            = 3,
  VN100_Error_InvalidCommand            = 4,
  VN100_Error_NotEnoughParameters        = 5,
  VN100_Error_TooManyParameters          = 6,
  VN100_Error_InvalidParameter          = 7,
  VN100_Error_InvalidRegister            = 8,
  VN100_Error_UnauthorizedAccess        = 9,
  VN100_Error_WatchdogReset              = 10,
  VN100_Error_OutputBufferOverflow      = 11,
  VN100_Error_InsufficientBandwidth      = 12
} VN100_Error;

/* Asynchronous Data Output Register */
typedef enum VN100_ADORType
{
  VN100_ADOR_OFF  = 0,
  VN100_ADOR_YPR  = 1,
  VN100_ADOR_QTN  = 2,
  VN100_ADOR_QTM  = 3,
  VN100_ADOR_QTA  = 4,
  VN100_ADOR_QTR  = 5,
  VN100_ADOR_QMA  = 6,
  VN100_ADOR_QAR  = 7,
  VN100_ADOR_QMR  = 8,
  VN100_ADOR_DCM  = 9,
  VN100_ADOR_MAG  = 10,
  VN100_ADOR_ACC  = 11,
  VN100_ADOR_GYR  = 12,
  VN100_ADOR_MAR  = 13,
  VN100_ADOR_YMR  = 14,

  VN100_ADOR_RAB  = 251,
  VN100_ADOR_RAW  = 252,
  VN100_ADOR_CMV  = 253,
  VN100_ADOR_STV  = 254,
  VN100_ADOR_COV  = 255
} VN100_ADORType;

/* Asynchronous Data Ouput Rate Register */
typedef enum VN100_ADOFType {
  VN100_ADOF_1HZ     =  1,
  VN100_ADOF_2HZ     = 2,
  VN100_ADOF_4HZ     = 4,
  VN100_ADOF_5HZ     = 5,
  VN100_ADOF_10HZ    = 10,
  VN100_ADOF_20HZ    = 20,
  VN100_ADOF_25HZ    = 25,
  VN100_ADOF_40HZ    = 40,
  VN100_ADOF_50HZ    = 50,
  VN100_ADOF_100HZ   = 100,
  VN100_ADOF_200HZ   = 200
} VN100_ADOFType;

/* Serial Baud Rate Register */
typedef enum VN100_BaudType {
  VN100_Baud_9600     =  9600,
  VN100_Baud_19200    = 19200,
  VN100_Baud_38400    = 38400,
  VN100_Baud_57600    = 57600,
  VN100_Baud_115200   = 115200,
  VN100_Baud_128000   = 128000,
  VN100_Baud_230400   = 230400,
  VN100_Baud_460800   = 460800,
  VN100_Baud_921600   = 921600
} VN100_BaudType;

/* Accelerometer Gain Type */
typedef enum VN100_AccGainType {
  VN100_AccGain_2G = 0,
  VN100_AccGain_6G = 1
} VN100_AccGainType;

/* 32-bit Parameter Type */
typedef union {
  unsigned long      UInt;
  float             Float;
} VN100_Param;

/* SPI Response Packet */
typedef struct {
  unsigned char         ZeroByte;
  unsigned char         CmdID;
  unsigned char          RegID;
  unsigned char          ErrID;
  VN100_Param Data[VN100_SPI_BUFFER_SIZE];
} VN100_SPI_Packet;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
VN100_SPI_Packet* VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth);
VN100_SPI_Packet* VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues);
VN100_SPI_Packet* VN100_SPI_GetModel(unsigned char sensorID, char* model);
VN100_SPI_Packet* VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision);
VN100_SPI_Packet* VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber);
VN100_SPI_Packet* VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion);
VN100_SPI_Packet* VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType* baudRate);
VN100_SPI_Packet* VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate);
VN100_SPI_Packet* VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType* ADOR);
VN100_SPI_Packet* VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR);
VN100_SPI_Packet* VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType* ADOF);
VN100_SPI_Packet* VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF);
VN100_SPI_Packet* VN100_SPI_GetYPR(unsigned char sensorID, float* yaw, float* pitch, float* roll);
VN100_SPI_Packet* VN100_SPI_GetQuat(unsigned char sensorID, float* q);
VN100_SPI_Packet* VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag);
VN100_SPI_Packet* VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* Acc);
VN100_SPI_Packet* VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates);
VN100_SPI_Packet* VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* Acc);
VN100_SPI_Packet* VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetDCM(unsigned char sensorID, float **DCM);
VN100_SPI_Packet* VN100_SPI_GetMag(unsigned char sensorID, float* mag);
VN100_SPI_Packet* VN100_SPI_GetAcc(unsigned char sensorID, float* Acc);
VN100_SPI_Packet* VN100_SPI_GetRates(unsigned char sensorID, float* rates);
VN100_SPI_Packet* VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc);
VN100_SPI_Packet* VN100_SPI_SetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc);
VN100_SPI_Packet* VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVariance);
VN100_SPI_Packet* VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar);
VN100_SPI_Packet* VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI);
VN100_SPI_Packet* VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI);
VN100_SPI_Packet* VN100_SPI_GetFiltActTuning(unsigned char sensorID, float* gainM, float* gainA, float* memM, float* memA);
VN100_SPI_Packet* VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA);
VN100_SPI_Packet* VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp);
VN100_SPI_Packet* VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp);
VN100_SPI_Packet* VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot);
VN100_SPI_Packet* VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot);
VN100_SPI_Packet* VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType* gain);
VN100_SPI_Packet* VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain);
VN100_SPI_Packet* VN100_SPI_RestoreFactorySettings(unsigned char sensorID);
VN100_SPI_Packet* VN100_SPI_Tare(unsigned char sensorID);
void VN100_SPI_Reset(unsigned char sensorID);
void VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI);
void VN100_SPI_GetMagInertial(unsigned char sensorID, float *AccI);


#endif /* __VN100_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/





