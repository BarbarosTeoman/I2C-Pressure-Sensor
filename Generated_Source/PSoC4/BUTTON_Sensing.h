/***************************************************************************//**
* \file BUTTON_Sensing.h
* \version 7.0
*
* \brief
*   This file provides the headers of APIs specific to implementation of the
*   sensing FW block.
*
* \see BUTTON v7.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016-2019), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined(CY_SENSE_BUTTON_SENSING_H)
#define CY_SENSE_BUTTON_SENSING_H

#include <CyLib.h>
#include <cyfitter.h>
#include "cytypes.h"
#include <cypins.h>
#include "BUTTON_Configuration.h"
#include "BUTTON_Structure.h"
#include "BUTTON_ISR.h"

/*******************************************************************************
* Enumeration types definition
*******************************************************************************/

/* Pin states */
#define BUTTON_GROUND                                 (0u)
#define BUTTON_HIGHZ                                  (1u)
#define BUTTON_SHIELD                                 (2u)
#define BUTTON_SENSOR                                 (3u)
#define BUTTON_TX_PIN                                 (4u)
#define BUTTON_RX_PIN                                 (5u)

/***************************************************************************//**
* \brief Defines operation modes types
*******************************************************************************/
typedef enum
{
    BUTTON_SCAN_CSD_E     = 0x00u,
    BUTTON_SCAN_CSX_E     = 0x01u,
    BUTTON_BIST_CSD_SNS_CAP_E = 0x02u,
    BUTTON_BIST_CSX_SNS_CAP_E = 0x03u,
    BUTTON_BIST_CSD_SH_CAP_E  = 0x04u,
} BUTTON_OPERATION_MODE_ENUM;


#if (BUTTON_ENABLE != BUTTON_ADC_STANDALONE_EN)
/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/**
* \cond (SECTION_C_HIGH_LEVEL || SECTION_I_HIGH_LEVEL)
* \addtogroup group_c_high_level
* \{
*/

cystatus BUTTON_SetupWidget(uint32 widgetId);
cystatus BUTTON_Scan(void);
cystatus BUTTON_ScanAllWidgets(void);
uint32 BUTTON_IsBusy(void);

/** \}
* \endcond */

/**
* \cond (SECTION_C_LOW_LEVEL || SECTION_I_LOW_LEVEL)
* \addtogroup group_c_low_level
* \{
*/

void BUTTON_SetPinState(uint32 widgetId, uint32 sensorElement, uint32 state);

cystatus BUTTON_SetupWidgetExt(uint32 widgetId, uint32 sensorId);
cystatus BUTTON_ScanExt(void);

#if (BUTTON_ANYMODE_AUTOCAL)
    cystatus BUTTON_CalibrateWidget(uint32 widgetId);
    cystatus BUTTON_CalibrateAllWidgets(void);
#endif /* (BUTTON_ANYMODE_AUTOCAL) */

#if((BUTTON_ENABLE == BUTTON_CSD_EN) ||\
    (BUTTON_ENABLE == BUTTON_CSX_EN))
    uint32_t BUTTON_SetInactiveElectrodeState(BUTTON_OPERATION_MODE_ENUM mode, uint32_t state);
#endif /* ((BUTTON_ENABLE == BUTTON_CSD_EN) ||\
           (BUTTON_ENABLE == BUTTON_CSX_EN)) */

/** \}
* \endcond */

/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

cystatus BUTTON_SsInitialize(void);
void BUTTON_SsSetModClkClockDivider(uint32 modClk);
void BUTTON_SsSetSnsClockDivider(uint32 snsClk);
void BUTTON_SsSetClockDividers(uint32 snsClk, uint32 modClk);
void BUTTON_SsIsrInitialize(cyisraddress address);
void BUTTON_SsPostAllWidgetsScan(void);
void BUTTON_SsSetAllIOsState(uint32 state);
void BUTTON_SsSwitchSensingMode(BUTTON_SENSE_METHOD_ENUM mode);

#if (BUTTON_CSD_SS_DIS != BUTTON_CSD_AUTOTUNE)
    cystatus BUTTON_SsAutoTune(void);
#endif

#if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
    void BUTTON_SsChangeImoFreq(uint32 value);
    void BUTTON_SsChangeClkFreq(uint32 chId);
#endif

#if (BUTTON_ENABLE == BUTTON_ADC_EN)
    cystatus BUTTON_SsReleaseResources(void);
#endif

#if((BUTTON_ENABLE == BUTTON_CSD_EN) || \
    (((BUTTON_ENABLE == BUTTON_CSX_EN) || \
    (BUTTON_ENABLE == BUTTON_ISX_EN)) && \
    (BUTTON_ENABLE == BUTTON_CSDV2)))
    void BUTTON_SsInitializeSourceSenseClk(void);
#endif

#if (BUTTON_ENABLE == BUTTON_CSD_EN)
    uint32 BUTTON_SsCSDGetColSnsClkDivider(uint32 widgetId);

    #if ((BUTTON_ENABLE == BUTTON_CSD_MATRIX_WIDGET_EN) || \
         (BUTTON_ENABLE == BUTTON_CSD_TOUCHPAD_WIDGET_EN))
        uint32 BUTTON_SsCSDGetRowSnsClkDivider(uint32 widgetId);
    #endif

    #if (BUTTON_CLK_SOURCE_PRSAUTO == BUTTON_CSD_SNS_CLK_SOURCE)
        uint8 BUTTON_SsCSDCalcPrsSize(uint32 snsClkDivider, uint32 resolution);
    #endif
#endif

void BUTTON_DischargeExtCapacitors(uint32 dischargeTime);
void BUTTON_WriteBitsSafe(reg32 * regPtr, uint32 mask, uint32 value);

/** \}
* \endcond */

/***************************************
* Global software/external variables
***************************************/
extern volatile uint8 BUTTON_widgetIndex;
extern volatile uint8 BUTTON_sensorIndex;
extern uint8 BUTTON_requestScanAllWidget;

extern BUTTON_RAM_SNS_STRUCT *BUTTON_curRamSnsPtr;
extern BUTTON_SENSE_METHOD_ENUM BUTTON_currentSenseMethod;

#if(BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
    extern uint8 BUTTON_scanFreqIndex;
    extern uint8 BUTTON_immunity[BUTTON_NUM_SCAN_FREQS];
#else
    extern const uint8 BUTTON_scanFreqIndex;
#endif

#if ((BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN) || \
     (BUTTON_ENABLE == BUTTON_CSX_EN) || \
     (BUTTON_ENABLE == BUTTON_ISX_EN))
    extern BUTTON_FLASH_WD_STRUCT const *BUTTON_curFlashWdgtPtr;
#endif

#if (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN)
    /* Pointer to Flash structure holding info of sensor to be scanned */
    extern BUTTON_FLASH_SNS_STRUCT const *BUTTON_curFlashSnsPtr;
#endif

extern BUTTON_FLASH_IO_STRUCT const *BUTTON_curSnsIOPtr;

/***************************************************************************//**
* \brief Declares the IDAC gain table structure
*******************************************************************************/
typedef struct
{
    /**
     *  Register value of IDAC gain
     */
    uint32 gainReg;

    /**
     *  Absolute gain value in pA
     */
    uint32 gainValue;
} BUTTON_IDAC_GAIN_TABLE_STRUCT;

#if (BUTTON_ENABLE == BUTTON_CSDV1)
    #define BUTTON_IDAC_GAIN_MASK                         (0x00000400uL)
    #define BUTTON_IDAC_GAIN_NUMBER                       (2u)
    #define BUTTON_IDAC_GAIN_TABLE_INIT                   {{0x00000000u, 1200000u},\
                                                                     {0x00000400u, 2400000u}}
#elif (BUTTON_ENABLE == BUTTON_CSDV2_VER1)
    #define BUTTON_IDAC_GAIN_MASK                         (0x03C00000uL)
    #define BUTTON_IDAC_GAIN_NUMBER                       (6u)
    #define BUTTON_IDAC_GAIN_TABLE_INIT                   {{0x01000000u,   37500u},\
                                                                     {0x03000000u,   75000u},\
                                                                     {0x01400000u,  300000u},\
                                                                     {0x03400000u,  600000u},\
                                                                     {0x01800000u, 2400000u},\
                                                                     {0x03800000u, 4800000u}}
#elif (BUTTON_ENABLE == BUTTON_CSDV2_VER2)
    #define BUTTON_IDAC_GAIN_MASK                         (0x03C00000uL)
    #define BUTTON_IDAC_GAIN_NUMBER                       (7u)
    #define BUTTON_IDAC_GAIN_TABLE_INIT                   {{0x01000000u,   37500u},\
                                                                     {0x03000000u,   75000u},\
                                                                     {0x01400000u,  300000u},\
                                                                     {0x03400000u,  600000u},\
                                                                     {0x01C00000u, 1200000u},\
                                                                     {0x01800000u, 2400000u},\
                                                                     {0x03800000u, 4800000u}}
#endif
extern BUTTON_IDAC_GAIN_TABLE_STRUCT const BUTTON_idacGainTable[BUTTON_IDAC_GAIN_NUMBER];

#endif /* (BUTTON_ENABLE != BUTTON_ADC_STANDALONE_EN) */

/****************************************************************************
* m0s8csd, m0s8peri, hsiom, and IO hardware-related registers address
****************************************************************************/
#if (BUTTON_ENABLE == BUTTON_CSDV2)

    /* Fourth-generation HW block registers   */
    #define BUTTON_CONFIG_REG                         (*(reg32 *) CYREG_CSD_CONFIG)
    #define BUTTON_CONFIG_PTR                         ( (reg32 *) CYREG_CSD_CONFIG)
    #define BUTTON_STAT_SEQ_REG                       (*(reg32 *) CYREG_CSD_CONFIG)
    #define BUTTON_STAT_SEQ_PTR                       ( (reg32 *) CYREG_CSD_CONFIG)
    #define BUTTON_CSD_IDACA_REG                      (*(reg32 *) CYREG_CSD_IDACA)
    #define BUTTON_CSD_IDACA_PTR                      ( (reg32 *) CYREG_CSD_IDACA)
    #define BUTTON_CSD_IDACB_REG                      (*(reg32 *) CYREG_CSD_IDACB)
    #define BUTTON_CSD_IDACB_PTR                      ( (reg32 *) CYREG_CSD_IDACB)
    #define BUTTON_SENSE_PERIOD_REG                   (*(reg32 *) CYREG_CSD_SENSE_PERIOD)
    #define BUTTON_SENSE_PERIOD_PTR                   ( (reg32 *) CYREG_CSD_SENSE_PERIOD)
    #define BUTTON_RESULT_VAL1_REG                    (*(reg32 *) CYREG_CSD_RESULT_VAL1)
    #define BUTTON_RESULT_VAL1_PTR                    ( (reg32 *) CYREG_CSD_RESULT_VAL1)
    #define BUTTON_RESULT_VAL2_REG                    (*(reg32 *) CYREG_CSD_RESULT_VAL2)
    #define BUTTON_RESULT_VAL2_PTR                    ( (reg32 *) CYREG_CSD_RESULT_VAL2)
    #define BUTTON_INTR_REG                           (*(reg32 *) CYREG_CSD_INTR)
    #define BUTTON_INTR_PTR                           ( (reg32 *) CYREG_CSD_INTR)
    #define BUTTON_INTR_SET_REG                       (*(reg32 *) CYREG_CSD_INTR_SET)
    #define BUTTON_INTR_SET_PTR                       ( (reg32 *) CYREG_CSD_INTR_SET)
    #define BUTTON_INTR_MASK_REG                      (*(reg32 *) CYREG_CSD_INTR_MASK)
    #define BUTTON_INTR_MASK_PTR                      ( (reg32 *) CYREG_CSD_INTR_MASK)
    #define BUTTON_COUNTER_REG                        (BUTTON_RESULT_VAL1_REG)
    #define BUTTON_COUNTER_PTR                        (BUTTON_RESULT_VAL1_PTR)
    #define BUTTON_REFGEN_REG                         (*(reg32 *) CYREG_CSD_REFGEN)
    #define BUTTON_REFGEN_PTR                         ( (reg32 *) CYREG_CSD_REFGEN)
    #define BUTTON_SEQ_TIME_REG                       (*(reg32 *) CYREG_CSD_SEQ_TIME)
    #define BUTTON_SEQ_TIME_PTR                       ( (reg32 *) CYREG_CSD_SEQ_TIME)
    #define BUTTON_SEQ_INIT_CNT_REG                   (*(reg32 *) CYREG_CSD_SEQ_INIT_CNT)
    #define BUTTON_SEQ_INIT_CNT_PTR                   ( (reg32 *) CYREG_CSD_SEQ_INIT_CNT)
    #define BUTTON_SEQ_NORM_CNT_REG                   (*(reg32 *) CYREG_CSD_SEQ_NORM_CNT)
    #define BUTTON_SEQ_NORM_CNT_PTR                   ( (reg32 *) CYREG_CSD_SEQ_NORM_CNT)
    #define BUTTON_SEQ_START_REG                      (*(reg32 *) CYREG_CSD_SEQ_START)
    #define BUTTON_SEQ_START_PTR                      ( (reg32 *) CYREG_CSD_SEQ_START)
    #define BUTTON_CSDCMP_REG                         (*(reg32 *) CYREG_CSD_CSDCMP)
    #define BUTTON_CSDCMP_PTR                         ( (reg32 *) CYREG_CSD_CSDCMP)
    #define BUTTON_HSCMP_REG                          (*(reg32 *) CYREG_CSD_HSCMP)
    #define BUTTON_HSCMP_PTR                          ( (reg32 *) CYREG_CSD_HSCMP)
    #define BUTTON_SENSE_DUTY_REG                     (*(reg32 *) CYREG_CSD_SENSE_DUTY)
    #define BUTTON_SENSE_DUTY_PTR                     ( (reg32 *) CYREG_CSD_SENSE_DUTY)
    #define BUTTON_AMBUF_REG                          (*(reg32 *) CYREG_CSD_AMBUF)
    #define BUTTON_AMBUF_PTR                          ( (reg32 *) CYREG_CSD_AMBUF)
    #define BUTTON_SW_BYP_SEL_REG                     (*(reg32 *) CYREG_CSD_SW_BYP_SEL)
    #define BUTTON_SW_BYP_SEL_PTR                     ( (reg32 *) CYREG_CSD_SW_BYP_SEL)
    #define BUTTON_SW_CMP_P_SEL_REG                   (*(reg32 *) CYREG_CSD_SW_CMP_P_SEL)
    #define BUTTON_SW_CMP_P_SEL_PTR                   ( (reg32 *) CYREG_CSD_SW_CMP_P_SEL)
    #define BUTTON_SW_REFGEN_SEL_REG                  (*(reg32 *) CYREG_CSD_SW_REFGEN_SEL)
    #define BUTTON_SW_REFGEN_SEL_PTR                  ( (reg32 *) CYREG_CSD_SW_REFGEN_SEL)
    #define BUTTON_SW_CMP_N_SEL_REG                   (*(reg32 *) CYREG_CSD_SW_CMP_N_SEL)
    #define BUTTON_SW_CMP_N_SEL_PTR                   ( (reg32 *) CYREG_CSD_SW_CMP_N_SEL)
    #define BUTTON_SW_RES_REG                         (*(reg32 *) CYREG_CSD_SW_RES)
    #define BUTTON_SW_RES_PTR                         ( (reg32 *) CYREG_CSD_SW_RES)
    #define BUTTON_SW_HS_P_SEL_REG                    (*(reg32 *) CYREG_CSD_SW_HS_P_SEL)
    #define BUTTON_SW_HS_P_SEL_PTR                    ( (reg32 *) CYREG_CSD_SW_HS_P_SEL)
    #define BUTTON_SW_SHIELD_SEL_REG                  (*(reg32 *) CYREG_CSD_SW_SHIELD_SEL)
    #define BUTTON_SW_SHIELD_SEL_PTR                  ( (reg32 *) CYREG_CSD_SW_SHIELD_SEL)
    #define BUTTON_SW_AMUXBUF_SEL_REG                 (*(reg32 *) CYREG_CSD_SW_AMUXBUF_SEL)
    #define BUTTON_SW_AMUXBUF_SEL_PTR                 ( (reg32 *) CYREG_CSD_SW_AMUXBUF_SEL)
    #define BUTTON_SW_HS_N_SEL_REG                    (*(reg32 *) CYREG_CSD_SW_HS_N_SEL)
    #define BUTTON_SW_HS_N_SEL_PTR                    ( (reg32 *) CYREG_CSD_SW_HS_N_SEL)
    #define BUTTON_SW_FW_MOD_SEL_REG                  (*(reg32 *) CYREG_CSD_SW_FW_MOD_SEL)
    #define BUTTON_SW_FW_MOD_SEL_PTR                  ( (reg32 *) CYREG_CSD_SW_FW_MOD_SEL)
    #define BUTTON_SW_FW_TANK_SEL_REG                 (*(reg32 *) CYREG_CSD_SW_FW_TANK_SEL)
    #define BUTTON_SW_FW_TANK_SEL_PTR                 ( (reg32 *) CYREG_CSD_SW_FW_TANK_SEL)
    #define BUTTON_SW_DSI_SEL_REG                     (*(reg32 *) CYREG_CSD_SW_DSI_SEL)
    #define BUTTON_SW_DSI_SEL_PTR                     ( (reg32 *) CYREG_CSD_SW_DSI_SEL)
    #define BUTTON_ADC_CTL_REG                        (*(reg32 *) CYREG_CSD_ADC_CTL)
    #define BUTTON_ADC_CTL_PTR                        ( (reg32 *) CYREG_CSD_ADC_CTL)
    #define BUTTON_ADC_RES_REG                        (*(reg32 *) CYREG_CSD_ADC_RES)
    #define BUTTON_ADC_RES_PTR                        ( (reg32 *) CYREG_CSD_ADC_RES)

#else

    /* CSD block registers   */
    #define BUTTON_CONFIG_REG                         (*(reg32 *) BUTTON_CSD__CSD_CONFIG)
    #define BUTTON_CONFIG_PTR                         ( (reg32 *) BUTTON_CSD__CSD_CONFIG)
    #define BUTTON_IDAC_CONTR_REG                     (*(reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CONTROL)
    #define BUTTON_IDAC_CONTR_PTR                     ( (reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CONTROL)
    #define BUTTON_IDAC_REG                           (*(reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CSD_IDAC)
    #define BUTTON_IDAC_PTR                           ( (reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CSD_IDAC)
    #define BUTTON_COUNTER_REG                        (*(reg32 *) BUTTON_CSD__CSD_COUNTER)
    #define BUTTON_COUNTER_PTR                        ( (reg32 *) BUTTON_CSD__CSD_COUNTER)
    #define BUTTON_STATUS_REG                         (*(reg32 *) BUTTON_CSD__CSD_STATUS)
    #define BUTTON_STATUS_PTR                         ( (reg32 *) BUTTON_CSD__CSD_STATUS)
    #define BUTTON_INTR_REG                           (*(reg32 *) BUTTON_CSD__CSD_INTR)
    #define BUTTON_INTR_PTR                           ( (reg32 *) BUTTON_CSD__CSD_INTR)
    #define BUTTON_INTR_SET_REG                       (*(reg32 *) CYREG_CSD_INTR_SET)
    #define BUTTON_INTR_SET_PTR                       ( (reg32 *) CYREG_CSD_INTR_SET)
    #define BUTTON_PWM_REG                            (*(reg32 *) CYREG_CSD_PWM)
    #define BUTTON_PWM_PTR                            ( (reg32 *) CYREG_CSD_PWM)
    #define BUTTON_TRIM1_REG                          (*(reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CSD_TRIM1)
    #define BUTTON_TRIM1_PTR                          ( (reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CSD_TRIM1)
    #define BUTTON_TRIM2_REG                          (*(reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CSD_TRIM2)
    #define BUTTON_TRIM2_PTR                          ( (reg32 *) BUTTON_IDACMod_cy_psoc4_idac__CSD_TRIM2)

    #if (CY_PSOC4_4100M || CY_PSOC4_4200M || CY_PSOC4_4200L)
        #if (1u != BUTTON_CSD__CSD_NUMBER)
            #define BUTTON_SFLASH_TRIM1_REG           (*(reg8 *) CYREG_SFLASH_CSD_TRIM1_CSD)
            #define BUTTON_SFLASH_TRIM1_PTR           ( (reg8 *) CYREG_SFLASH_CSD_TRIM1_CSD)

            #define BUTTON_SFLASH_TRIM2_REG           (*(reg8 *) CYREG_SFLASH_CSD_TRIM2_CSD)
            #define BUTTON_SFLASH_TRIM2_PTR           ( (reg8 *) CYREG_SFLASH_CSD_TRIM2_CSD)
        #else
            #define BUTTON_SFLASH_TRIM1_REG           (*(reg8 *) CYREG_SFLASH_CSD1_TRIM1_CSD)
            #define BUTTON_SFLASH_TRIM1_PTR           ( (reg8 *) CYREG_SFLASH_CSD1_TRIM1_CSD)

            #define BUTTON_SFLASH_TRIM2_REG           (*(reg8 *) CYREG_SFLASH_CSD1_TRIM2_CSD)
            #define BUTTON_SFLASH_TRIM2_PTR           ( (reg8 *) CYREG_SFLASH_CSD1_TRIM2_CSD)
        #endif /* (1u != BUTTON_CSD__CSD_NUMBER) */
    #else
        #define BUTTON_SFLASH_TRIM1_REG               (*(reg8 *) CYREG_SFLASH_CSD_TRIM1_CSD)
        #define BUTTON_SFLASH_TRIM1_PTR               ( (reg8 *) CYREG_SFLASH_CSD_TRIM1_CSD)

        #define BUTTON_SFLASH_TRIM2_REG               (*(reg8 *) CYREG_SFLASH_CSD_TRIM2_CSD)
        #define BUTTON_SFLASH_TRIM2_PTR               ( (reg8 *) CYREG_SFLASH_CSD_TRIM2_CSD)
    #endif /* (CY_PSOC4_4100M || CY_PSOC4_4200M || CY_PSOC4_4200L) */
#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */

/* Third-generation and Fourth-generation HW block PERI CLOCK registers */
#if defined(CYIPBLOCK_m0s8peri_VERSION)
    #define BUTTON_IS_M0S8PERI_BLOCK                  (1u)
    #define BUTTON_M0S8PERI_BLOCK_VER                 ((CYIPBLOCK_m0s8peri_VERSION))
#else
    #define BUTTON_IS_M0S8PERI_BLOCK                  (0u)
    #define BUTTON_M0S8PERI_BLOCK_VER                 (0u)
#endif /* (CYIPBLOCK_m0s8peri_VERSION) */

#if (BUTTON_ENABLE == BUTTON_IS_M0S8PERI_BLOCK)
    #define BUTTON_MODCLK_DIV_REG                     (*(reg32 *)BUTTON_ModClk__DIV_REGISTER)
    #define BUTTON_MODCLK_DIV_PTR                     ( (reg32 *)BUTTON_ModClk__DIV_REGISTER)
    #define BUTTON_MODCLK_CMD_REG                     (*(reg32 *)CYREG_PERI_DIV_CMD)
    #define BUTTON_MODCLK_CMD_PTR                     ( (reg32 *)CYREG_PERI_DIV_CMD)
    #if (BUTTON_DISABLE == BUTTON_CSDV2)
        #define BUTTON_SNSCLK_DIV_REG                 (*(reg32 *)BUTTON_SnsClk__DIV_REGISTER)
        #define BUTTON_SNSCLK_DIV_PTR                 ( (reg32 *)BUTTON_SnsClk__DIV_REGISTER)
        #define BUTTON_SNSCLK_CMD_REG                 (*(reg32 *)CYREG_PERI_DIV_CMD)
        #define BUTTON_SNSCLK_CMD_PTR                 ( (reg32 *)CYREG_PERI_DIV_CMD)
    #endif /* (BUTTON_DISABLE == BUTTON_CSDV2) */
#else
    #define BUTTON_MODCLK_DIV_REG                     (*(reg32 *)BUTTON_ModClk__REGISTER)
    #define BUTTON_MODCLK_DIV_PTR                     ( (reg32 *)BUTTON_ModClk__REGISTER)
    #define BUTTON_SNSCLK_DIV_REG                     (*(reg32 *)BUTTON_SnsClk__REGISTER)
    #define BUTTON_SNSCLK_DIV_PTR                     ( (reg32 *)BUTTON_SnsClk__REGISTER)
    #define BUTTON_MODCLK_CMD_REG                     (BUTTON_MODCLK_DIV_REG)
    #define BUTTON_MODCLK_CMD_PTR                     (BUTTON_MODCLK_DIV_PTR)
    #define BUTTON_SNSCLK_CMD_REG                     (BUTTON_SNSCLK_DIV_REG)
    #define BUTTON_SNSCLK_CMD_PTR                     (BUTTON_SNSCLK_DIV_PTR)
#endif /* (BUTTON_ENABLE == BUTTON_IS_M0S8PERI_BLOCK) */

/****************************************************************************
* m0s8csd, m0s8peri, hsiom, and IO hardware-related registers masks
****************************************************************************/
#if (BUTTON_ENABLE == BUTTON_CSDV2)

    #if ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
         (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN))
        #define BUTTON_CTANK_CONNECTION           (BUTTON_CSD__DEDICATED_IO1)
    #else
        #define BUTTON_CTANK_CONNECTION               (0x00000000uL)
    #endif /* ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
               (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN)) */

    #if(0u != BUTTON_CSX_EN)
        #define BUTTON_CMOD_CONNECTION                (BUTTON_CSD__CSHIELD_PAD)
    #else
        #define BUTTON_CMOD_CONNECTION                (BUTTON_CSD__DEDICATED_IO0)
    #endif /* (BUTTON_CSX_EN == 1u) */

    /* Fourth-generation HW block masks for the resistance or low EMI (slow ramp)  */
    #define BUTTON_RES_LOW                            (0u)
    #define BUTTON_RES_MED                            (1u)
    #define BUTTON_RES_HIGH                           (2u)
    #define BUTTON_RES_LOWEMI                         (3u)

    /* Fourth-generation HW block masks for the waveforms for corresponding switch  */
    #define BUTTON_STATIC_OPEN                        (0x00000000Lu)
    #define BUTTON_STATIC_CLOSE                       (0x00000001Lu)
    #define BUTTON_PHI1                               (0x00000002Lu)
    #define BUTTON_PHI2                               (0x00000003Lu)
    #define BUTTON_PHI1_HSCMP                         (0x00000004Lu)
    #define BUTTON_PHI2_HSCMP                         (0x00000005Lu)
    #define BUTTON_HSCMP                              (0x00000006Lu)

    #define BUTTON_SW_DSI_CMOD                        (1uL << 4u)
    #define BUTTON_SW_DSI_CTANK                       (1uL << 0u)

    /* Fourth-generation HW block CSD_CONFIG register masks    */
    #define BUTTON_CONFIG_FILTER_DELAY_MASK           (((0x00000001Lu << CYFLD_CSD_FILTER_DELAY__SIZE) - 1u) << CYFLD_CSD_FILTER_DELAY__OFFSET)
    #define BUTTON_CONFIG_FILTER_DELAY_2_CYCLES       (0x00000002Lu)
    #define BUTTON_CONFIG_FILTER_DELAY_3_CYCLES       (0x00000003Lu)
    #define BUTTON_CONFIG_FILTER_DELAY_4_CYCLES       (0x00000004Lu)
    #define BUTTON_CONFIG_FILTER_DELAY_12MHZ          (BUTTON_CONFIG_FILTER_DELAY_2_CYCLES << CYFLD_CSD_FILTER_DELAY__OFFSET)
    #define BUTTON_CONFIG_FILTER_DELAY_24MHZ          (BUTTON_CONFIG_FILTER_DELAY_3_CYCLES << CYFLD_CSD_FILTER_DELAY__OFFSET)
    #define BUTTON_CONFIG_FILTER_DELAY_48MHZ          (BUTTON_CONFIG_FILTER_DELAY_4_CYCLES << CYFLD_CSD_FILTER_DELAY__OFFSET)
    #define BUTTON_CONFIG_SHIELD_DELAY_MASK           (((0x00000001Lu << CYFLD_CSD_SHIELD_DELAY__SIZE) - 1u) << CYFLD_CSD_SHIELD_DELAY__OFFSET)
    #define BUTTON_CONFIG_SENSE_EN_MASK               (((0x00000001Lu << CYFLD_CSD_SENSE_EN__SIZE) - 1u) << CYFLD_CSD_SENSE_EN__OFFSET)
    #define BUTTON_CONFIG_CHARGE_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_CHARGE_MODE__SIZE) - 1u) << CYFLD_CSD_CHARGE_MODE__OFFSET)
    #define BUTTON_CONFIG_MUTUAL_CAP_MASK             (((0x00000001Lu << CYFLD_CSD_MUTUAL_CAP__SIZE) - 1u) << CYFLD_CSD_MUTUAL_CAP__OFFSET)
    #define BUTTON_CONFIG_CSX_DUAL_CNT_MASK           (((0x00000001Lu << CYFLD_CSD_CSX_DUAL_CNT__SIZE) - 1u) << CYFLD_CSD_CSX_DUAL_CNT__OFFSET)
    #define BUTTON_CONFIG_DSI_COUNT_SEL_MASK          (((0x00000001Lu << CYFLD_CSD_DSI_COUNT_SEL__SIZE) - 1u) << CYFLD_CSD_DSI_COUNT_SEL__OFFSET)
    #define BUTTON_CONFIG_DSI_SAMPLE_EN_MASK          (((0x00000001Lu << CYFLD_CSD_DSI_SAMPLE_EN__SIZE) - 1u) << CYFLD_CSD_DSI_SAMPLE_EN__OFFSET)
    #define BUTTON_CONFIG_SAMPLE_SYNC_MASK            (((0x00000001Lu << CYFLD_CSD_SAMPLE_SYNC__SIZE) - 1u) << CYFLD_CSD_SAMPLE_SYNC__OFFSET)
    #define BUTTON_CONFIG_DSI_SENSE_EN_MASK           (((0x00000001Lu << CYFLD_CSD_DSI_SENSE_EN__SIZE) - 1u) << CYFLD_CSD_DSI_SENSE_EN__OFFSET)
    #define BUTTON_CONFIG_LP_MODE_MASK                (((0x00000001Lu << CYFLD_CSD_LP_MODE__SIZE) - 1u) << CYFLD_CSD_LP_MODE__OFFSET)
    #define BUTTON_CONFIG_ENABLE_MASK                 (((0x00000001Lu << CYFLD_CSD_ENABLE__SIZE) - 1u) << CYFLD_CSD_ENABLE__OFFSET)

    /* Fourth-generation HW block CSD_STATUS register masks    */
    #define BUTTON_STATUS_CSD_CHARGE_MASK             (((0x00000001Lu << CYFLD_CSD_CSD_CHARGE__SIZE) - 1u) << CYFLD_CSD_CSD_CHARGE__OFFSET)
    #define BUTTON_STATUS_CSD_SENSE_MASK              (((0x00000001Lu << CYFLD_CSD_CSD_SENSE__SIZE) - 1u) << CYFLD_CSD_CSD_SENSE__OFFSET)
    #define BUTTON_STATUS_HSCMP_OUT_MASK              (((0x00000001Lu << CYFLD_CSD_HSCMP_OUT__SIZE) - 1u) << CYFLD_CSD_HSCMP_OUT__OFFSET)
    #define BUTTON_STATUS_CSDCMP_OUT_MASK             (((0x00000001Lu << CYFLD_CSD_CSDCMP_OUT__SIZE) - 1u) << CYFLD_CSD_CSDCMP_OUT__OFFSET)

    /* Fourth-generation HW block STAT_SEQ register masks    */
    #define BUTTON_STAT_SEQ_SEQ_STATE_MASK            (((0x00000001Lu << CYFLD_CSD_SEQ_STATE__SIZE) - 1u) << CYFLD_CSD_SEQ_STATE__OFFSET)
    #define BUTTON_STAT_SEQ_ADC_STATE_MASK            (((0x00000001Lu << CYFLD_CSD_ADC_STATE__SIZE) - 1u) << CYFLD_CSD_ADC_STATE__OFFSET)

    /* Fourth-generation HW block sequencer state codes */
    #define BUTTON_STAT_SEQ_IDLE                      (0x00000000Lu << CYFLD_CSD_SEQ_STATE__OFFSET)
    #define BUTTON_STAT_SEQ_INIT_COARSE               (0x00000001Lu << CYFLD_CSD_SEQ_STATE__OFFSET)
    #define BUTTON_STAT_SEQ_AUTO_ZERO_0               (0x00000002Lu << CYFLD_CSD_SEQ_STATE__OFFSET)
    #define BUTTON_STAT_SEQ_SAMPLE_INIT               (0x00000003Lu << CYFLD_CSD_SEQ_STATE__OFFSET)
    #define BUTTON_STAT_SEQ_AUTO_ZERO_1               (0x00000004Lu << CYFLD_CSD_SEQ_STATE__OFFSET)
    #define BUTTON_STAT_SEQ_SAMPLE_NORM               (0x00000005Lu << CYFLD_CSD_SEQ_STATE__OFFSET)


    /* Fourth-generation HW block STAT_CNTS register masks    */
    #define BUTTON_STAT_CNTS_NUM_CONV_MASK            (((0x00000001Lu << CYFLD_CSD_NUM_CONV__SIZE) - 1u) << CYFLD_CSD_NUM_CONV__OFFSET)

    /* Fourth-generation HW block RESULT_VAL1 register masks    */
    #define BUTTON_RESULT_VAL1_VALUE_MASK             (((0x00000001Lu << CYFLD_CSD_VALUE__SIZE) - 1u) << CYFLD_CSD_VALUE__OFFSET)
    #define BUTTON_RESULT_VAL1_BAD_CONVS_MASK         (((0x00000001Lu << CYFLD_CSD_BAD_CONVS__SIZE) - 1u) << CYFLD_CSD_BAD_CONVS__OFFSET)

    /* Fourth-generation HW block RESULT_VAL2 register masks    */
    #define BUTTON_RESULT_VAL2_VALUE_MASK             (0x0000FFFFLu)

    /* Fourth-generation HW block INTR register masks    */
    #define BUTTON_INTR_SAMPLE_MASK                   (((0x00000001Lu << CYFLD_CSD_SAMPLE__SIZE) - 1u) << CYFLD_CSD_SAMPLE__OFFSET)
    #define BUTTON_INTR_INIT_MASK                     (((0x00000001Lu << CYFLD_CSD_INIT__SIZE) - 1u) << CYFLD_CSD_INIT__OFFSET)
    #define BUTTON_INTR_ADC_RES_MASK                  (((0x00000001Lu << CYFLD_CSD_ADC_RES__SIZE) - 1u) << CYFLD_CSD_ADC_RES__OFFSET)
    #define BUTTON_CLEAR_MASK                         (0Lu)
    #define BUTTON_INTR_ALL_MASK                      (BUTTON_INTR_SAMPLE_MASK | \
                                                                 BUTTON_INTR_INIT_MASK | \
                                                                 BUTTON_INTR_ADC_RES_MASK)

    /* Fourth-generation HW block INTR_SET register masks    */
    #define BUTTON_INTR_SET_SAMPLE_MASK               (BUTTON_INTR_SAMPLE_MASK)
    #define BUTTON_INTR_SET_INIT_MASK                 (BUTTON_INTR_INIT_MASK)
    #define BUTTON_INTR_SET_ADC_RES_MASK              (BUTTON_INTR_ADC_RES_MASK)
    #define BUTTON_INTR_SET_MASK                      (BUTTON_INTR_SET_SAMPLE_MASK | \
                                                                 BUTTON_INTR_SET_INIT_MASK | \
                                                                 BUTTON_INTR_SET_ADC_RES_MASK)

    /* Fourth-generation HW block INTR_MASK register masks    */
    #define BUTTON_INTR_MASK_SAMPLE_MASK              (BUTTON_INTR_SAMPLE_MASK)
    #define BUTTON_INTR_MASK_INIT_MASK                (BUTTON_INTR_INIT_MASK)
    #define BUTTON_INTR_MASK_ADC_RES_MASK             (BUTTON_INTR_ADC_RES_MASK)
    #define BUTTON_INTR_MASK_CLEAR_MASK               (0uL)


    /* Fourth-generation HW block HSCMP v2 register masks    */
    #define BUTTON_HSCMP_EN_MASK                      (((0x00000001Lu << CYFLD_CSD_HSCMP_EN__SIZE) - 1u) << CYFLD_CSD_HSCMP_EN__OFFSET)
    #define BUTTON_HSCMP_INVERT_MASK                  (((0x00000001Lu << CYFLD_CSD_HSCMP_INVERT__SIZE) - 1u) << CYFLD_CSD_HSCMP_INVERT__OFFSET)
    #define BUTTON_CSD_AZ_EN_MASK                     (((0x00000001Lu << CYFLD_CSD_AZ_EN__SIZE) - 1u) << CYFLD_CSD_AZ_EN__OFFSET)

    /* Fourth-generation HW block AMBUF v2 register masks    */
    #define BUTTON_AMBUF_AMBUF_EN_MASK                ((0x00000001Lu << CYFLD_CSD_PWR_MODE__SIZE) - 1u)
    #define BUTTON_AMBUF_PWR_MODE_OFF                 (CYVAL_CSD_PWR_MODE_OFF)
    #define BUTTON_AMBUF_PWR_MODE_NORM                (CYVAL_CSD_PWR_MODE_NORM)
    #define BUTTON_AMBUF_PWR_MODE_HI                  (CYVAL_CSD_PWR_MODE_HI)

    /* Fourth-generation HW block REFGEN v2 register masks    */
    #define BUTTON_REFGEN_REFGEN_EN_MASK              ((0x00000001Lu << CYFLD_CSD_REFGEN_EN__SIZE) - 1u)
    #define BUTTON_REFGEN_BYPASS_MASK                 (((0x00000001Lu << CYFLD_CSD_BYPASS__SIZE) - 1u) << CYFLD_CSD_BYPASS__OFFSET)
    #define BUTTON_REFGEN_VDDA_EN_MASK                (((0x00000001Lu << CYFLD_CSD_VDDA_EN__SIZE) - 1u) << CYFLD_CSD_VDDA_EN__OFFSET)
    #define BUTTON_REFGEN_RES_EN_MASK                 (((0x00000001Lu << CYFLD_CSD_RES_EN__SIZE) - 1u) << CYFLD_CSD_RES_EN__OFFSET)
    #define BUTTON_REFGEN_GAIN_MASK                   (((0x00000001Lu << CYFLD_CSD_GAIN__SIZE) - 1u) << CYFLD_CSD_GAIN__OFFSET)
    #define BUTTON_REFGEN_VREFLO_SEL_MASK             (((0x00000001Lu << CYFLD_CSD_VREFLO_SEL__SIZE) - 1u) << CYFLD_CSD_VREFLO_SEL__OFFSET)
    #define BUTTON_REFGEN_VREFLO_INT_MASK             (((0x00000001Lu << CYFLD_CSD_VREFLO_INT__SIZE) - 1u) << CYFLD_CSD_VREFLO_INT__OFFSET)

    /* Fourth-generation HW block IDACA v2 register masks  */
    #define BUTTON_IDAC_MOD_VAL_MASK                  (((0x00000001Lu << CYFLD_CSD_VAL__SIZE) - 1u) << CYFLD_CSD_VAL__OFFSET)
    #define BUTTON_IDAC_MOD_POLARITY_MASK             (((0x00000001Lu << CYFLD_CSD_POLARITY__SIZE) - 1u) << CYFLD_CSD_POLARITY__OFFSET)
    #define BUTTON_IDAC_MOD_POLARITY_VSSA_SRC         ((uint32)CYVAL_CSD_POLARITY_VSSA_SRC)
    #define BUTTON_IDAC_MOD_POLARITY_VDDA_SNK         ((uint32)CYVAL_CSD_POLARITY_VDDA_SNK)
    #define BUTTON_IDAC_MOD_POLARITY_SENSE            ((uint32)CYVAL_CSD_POLARITY_SENSE)
    #define BUTTON_IDAC_MOD_POLARITY_SENSE_INV        ((uint32)CYVAL_CSD_POLARITY_SENSE_INV)
    #define BUTTON_IDAC_MOD_POL_DYN_MASK              (((0x00000001Lu << CYFLD_CSD_POL_DYN__SIZE) - 1u) << CYFLD_CSD_POL_DYN__OFFSET)
    #define BUTTON_IDAC_MOD_BALL_MODE_MASK             (((0x00000001Lu << CYFLD_CSD_BAL_MODE__SIZE) - 1u) << CYFLD_CSD_BAL_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_BALL_MODE_FULL            (CYVAL_CSD_BAL_MODE_FULL)
    #define BUTTON_IDAC_MOD_LEG1_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_LEG1_MODE__SIZE) - 1u) << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG1_MODE_GP_STATIC_MASK  (CYVAL_CSD_LEG1_MODE_GP_STATIC)
    #define BUTTON_IDAC_MOD_LEG1_MODE_GP_MASK         ((uint32)CYVAL_CSD_LEG1_MODE_GP << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG1_MODE_CSD_STATIC      ((uint32)CYVAL_CSD_LEG1_MODE_CSD_STATIC)
    #define BUTTON_IDAC_MOD_LEG1_MODE_CSD_STATIC_MASK ((uint32)CYVAL_CSD_LEG1_MODE_CSD_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG1_MODE_CSD             (0x00000003Lu)
    #define BUTTON_IDAC_MOD_LEG1_MODE_CSD_MASK        (0x00000003Lu << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG2_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_LEG2_MODE__SIZE) - 1u) << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG2_MODE_GP_STATIC       (CYVAL_CSD_LEG2_MODE_GP_STATIC)
    #define BUTTON_IDAC_MOD_LEG2_MODE_GP_STATIC_MASK  (CYVAL_CSD_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG2_MODE_GP_MASK         ((uint32)CYVAL_CSD_LEG2_MODE_GP << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG2_MODE_CSD_STATIC_MASK (0x00000002Lu << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_LEG2_MODE_CSD             (0x00000003Lu)
    #define BUTTON_IDAC_MOD_LEG2_MODE_CSD_MASK        (0x00000003Lu << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_MOD_BAL_MODE_PHI1             ((uint32)CYVAL_CSD_BAL_MODE_PHI1)
    #define BUTTON_IDAC_MOD_DSI_CTRL_EN_MASK          (0x00200000Lu)
    #define BUTTON_IDAC_MOD_RANGE_MASK                (0x00C00000Lu)
    #define BUTTON_IDAC_MOD_RANGE_IDAC_LO             (CYVAL_CSD_RANGE_IDAC_LO)
    #define BUTTON_IDAC_MOD_RANGE_IDAC_LO_MASK        (0x00000000Lu)
    #define BUTTON_IDAC_MOD_RANGE_IDAC_MED            (CYVAL_CSD_RANGE_IDAC_MED)
    #define BUTTON_IDAC_MOD_RANGE_IDAC_MED_MASK       (0x00400000Lu)
    #define BUTTON_IDAC_MOD_RANGE_IDAC_HI             (CYVAL_CSD_RANGE_IDAC_HI)
    #define BUTTON_IDAC_MOD_RANGE_IDAC_HI_MASK        (0x00800000Lu)
    #define BUTTON_IDAC_MOD_LEG1_EN_MASK              (0x01000000Lu)
    #define BUTTON_IDAC_MOD_LEG2_EN_MASK              (0x02000000Lu)

    /* Fourth-generation HW block IDACB v2 register masks  */
    #define BUTTON_IDAC_COMP_VAL_MASK                  (((0x00000001Lu << CYFLD_CSD_VAL__SIZE) - 1u) << CYFLD_CSD_VAL__OFFSET)
    #define BUTTON_IDAC_COMP_POLARITY_MASK             (((0x00000001Lu << CYFLD_CSD_POLARITY__SIZE) - 1u) << CYFLD_CSD_POLARITY__OFFSET)
    #define BUTTON_IDAC_COMP_POLARITY_VSSA_SRC         ((uint32)CYVAL_CSD_POLARITY_VSSA_SRC)
    #define BUTTON_IDAC_COMP_POLARITY_VDDA_SNK         ((uint32)CYVAL_CSD_POLARITY_VDDA_SNK)
    #define BUTTON_IDAC_COMP_POLARITY_SENSE            ((uint32)CYVAL_CSD_POLARITY_SENSE)
    #define BUTTON_IDAC_COMP_POLARITY_SENSE_INV        ((uint32)CYVAL_CSD_POLARITY_SENSE_INV)
    #define BUTTON_IDAC_COMP_POL_DYN_MASK              (((0x00000001Lu << CYFLD_CSD_POL_DYN__SIZE) - 1u) << CYFLD_CSD_POL_DYN__OFFSET)
    #define BUTTON_IDAC_COMP_POL_DYN_STATIC            (0uL)
    #define BUTTON_IDAC_COMP_POL_DYN_DYNAMIC           (1uL)
    #define BUTTON_IDAC_COMP_BALL_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_BAL_MODE__SIZE) - 1u) << CYFLD_CSD_BAL_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_BALL_MODE_FULL            (CYVAL_CSD_BAL_MODE_FULL)
    #define BUTTON_IDAC_COMP_LEG1_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_LEG1_MODE__SIZE) - 1u) << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG1_MODE_GP_STATIC       (CYVAL_CSD_LEG1_MODE_GP_STATIC)
    #define BUTTON_IDAC_COMP_LEG1_MODE_GP_STATIC_MASK  (CYVAL_CSD_LEG1_MODE_GP_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG1_MODE_GP_MASK         ((uint32)CYVAL_CSD_LEG1_MODE_GP << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG1_MODE_CSD_STATIC      ((uint32)CYVAL_CSD_LEG1_MODE_CSD_STATIC)
    #define BUTTON_IDAC_COMP_LEG1_MODE_CSD_STATIC_MASK ((uint32)CYVAL_CSD_LEG1_MODE_CSD_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG1_MODE_CSD             (0x00000003Lu)
    #define BUTTON_IDAC_COMP_LEG1_MODE_CSD_MASK        (0x00000003Lu << CYFLD_CSD_LEG1_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG2_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_LEG2_MODE__SIZE) - 1u) << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG2_MODE_GP_STATIC       ((uint32)CYVAL_CSD_LEG2_MODE_GP_STATIC)
    #define BUTTON_IDAC_COMP_LEG2_MODE_GP_STATIC_MASK  (CYVAL_CSD_LEG2_MODE_GP_STATIC)
    #define BUTTON_IDAC_COMP_LEG2_MODE_GP_MASK         ((uint32)CYVAL_CSD_LEG2_MODE_GP << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG2_MODE_CSD_STATIC      (0x00000002Lu)
    #define BUTTON_IDAC_COMP_LEG2_MODE_CSD_STATIC_MASK (0x00000002Lu << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_LEG2_MODE_CSD             (0x00000003Lu)
    #define BUTTON_IDAC_COMP_LEG2_MODE_CSD_MASK        (0x00000003Lu << CYFLD_CSD_LEG2_MODE__OFFSET)
    #define BUTTON_IDAC_COMP_DSI_CTRL_EN_MASK          (0x00200000Lu)
    #define BUTTON_IDAC_COMP_RANGE_MASK                (0x00C00000Lu)
    #define BUTTON_IDAC_COMP_RANGE_IDAC_LO             (CYVAL_CSD_RANGE_IDAC_LO)
    #define BUTTON_IDAC_COMP_RANGE_IDAC_LO_MASK        (0x00000000Lu)
    #define BUTTON_IDAC_COMP_RANGE_IDAC_MED            (CYVAL_CSD_RANGE_IDAC_MED)
    #define BUTTON_IDAC_COMP_RANGE_IDAC_MED_MASK       (0x00400000Lu)
    #define BUTTON_IDAC_COMP_RANGE_IDAC_HI             (CYVAL_CSD_RANGE_IDAC_HI)
    #define BUTTON_IDAC_COMP_RANGE_IDAC_HI_MASK        (0x00800000Lu)
    #define BUTTON_IDAC_COMP_LEG1_EN_MASK              (0x01000000Lu)
    #define BUTTON_IDAC_COMP_LEG2_EN_MASK              (0x02000000Lu)
    #define BUTTON_IDAC_COMP_LEG3_EN_MASK              (0x04000000Lu)

    /* Fourth-generation HW block SENSE_PERIOD register masks  */
    #define BUTTON_SENSE_PERIOD_SENSE_DIV_MASK        (((0x00000001Lu << CYFLD_CSD_SENSE_DIV__SIZE) - 1u) << CYFLD_CSD_SENSE_DIV__OFFSET)
    #define BUTTON_SENSE_PERIOD_LFSR_SIZE_MASK        (((0x00000001Lu << CYFLD_CSD_LFSR_SIZE__SIZE) - 1u) << CYFLD_CSD_LFSR_SIZE__OFFSET)
    #define BUTTON_SENSE_PERIOD_LFSR_SCALE_MASK       (((0x00000001Lu << CYFLD_CSD_LFSR_SCALE__SIZE) - 1u) << CYFLD_CSD_LFSR_SCALE__OFFSET)
    #define BUTTON_SENSE_PERIOD_LFSR_CLEAR_MASK       (((0x00000001Lu << CYFLD_CSD_LFSR_CLEAR__SIZE) - 1u) << CYFLD_CSD_LFSR_CLEAR__OFFSET)
    #define BUTTON_SENSE_PERIOD_SEL_LFSR_MSB_MASK     (((0x00000001Lu << CYFLD_CSD_SEL_LFSR_MSB__SIZE) - 1u) << CYFLD_CSD_SEL_LFSR_MSB__OFFSET)
    #define BUTTON_SENSE_6MHZ                         (6u)
    #define BUTTON_SENSE_PERIOD_SENSE_DIV_6MHZ        (CYDEV_BCLK__HFCLK__KHZ/BUTTON_CSD_SCANSPEED_DIVIDER/BUTTON_SENSE_6MHZ)

    /* Fourth-generation HW block SW_BYP_SEL register masks  */
    #define BUTTON_SW_BYP_SEL_SW_BYA_MASK             (((0x00000001Lu << CYFLD_CSD_SW_BYA__SIZE) - 1u) << CYFLD_CSD_SW_BYA__OFFSET)
    #define BUTTON_SW_BYP_SEL_SW_BYB_MASK             (((0x00000001Lu << CYFLD_CSD_SW_BYB__SIZE) - 1u) << CYFLD_CSD_SW_BYB__OFFSET)
    #define BUTTON_SW_BYP_SEL_SW_CBCC_MASK            (((0x00000001Lu << CYFLD_CSD_SW_CBCC__SIZE) - 1u) << CYFLD_CSD_SW_CBCC__OFFSET)

    /* Fourth-generation HW block SW_REFGEN_SEL register masks  */
    #define BUTTON_SW_REFGEN_SEL_SW_IAIB_MASK         (((0x00000001Lu << CYFLD_CSD_SW_IAIB__SIZE) - 1u) << CYFLD_CSD_SW_IAIB__OFFSET)
    #define BUTTON_SW_REFGEN_SEL_SW_IBCB_MASK         (((0x00000001Lu << CYFLD_CSD_SW_IBCB__SIZE) - 1u) << CYFLD_CSD_SW_IBCB__OFFSET)
    #define BUTTON_SW_REFGEN_SEL_SW_SGMB_MASK         (((0x00000001Lu << CYFLD_CSD_SW_SGMB__SIZE) - 1u) << CYFLD_CSD_SW_SGMB__OFFSET)
    #define BUTTON_SW_REFGEN_SEL_SW_SGRE_MASK         (((0x00000001Lu << CYFLD_CSD_SW_SGRE__SIZE) - 1u) << CYFLD_CSD_SW_SGRE__OFFSET)
    #define BUTTON_SW_REFGEN_SEL_SW_SGR_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SGR__SIZE) - 1u) << CYFLD_CSD_SW_SGR__OFFSET)
    #define BUTTON_SW_REFGEN_SEL_SW_MASK              (BUTTON_SW_REFGEN_SEL_SW_IAIB_MASK |\
                                                                 BUTTON_SW_REFGEN_SEL_SW_IBCB_MASK |\
                                                                 BUTTON_SW_REFGEN_SEL_SW_SGMB_MASK |\
                                                                 BUTTON_SW_REFGEN_SEL_SW_SGRE_MASK |\
                                                                 BUTTON_SW_REFGEN_SEL_SW_SGR_MASK)

    /* Fourth-generation HW block SW_CMP_N_SEL register masks  */
    #define BUTTON_SW_CMP_N_SEL_SW_SCRH_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SCRH__SIZE) - 1u) << CYFLD_CSD_SW_SCRH__OFFSET)
    #define BUTTON_SW_CMP_N_SEL_SW_SCRH_STATIC_OPEN   (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_SCRH__OFFSET)
    #define BUTTON_SW_CMP_N_SEL_SW_SCRH_STATIC_CLOSE  (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_SCRH__OFFSET)
    #define BUTTON_SW_CMP_N_SEL_SW_SCRL_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SCRL__SIZE) - 1u) << CYFLD_CSD_SW_SCRL__OFFSET)
    #define BUTTON_SW_CMP_N_SEL_SW_SCRL_STATIC_OPEN   (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_SCRL__OFFSET)
    #define BUTTON_SW_CMP_N_SEL_SW_SCRL_STATIC_CLOSE  (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_SCRL__OFFSET)

    /* Fourth-generation HW block SEQ_TIME register masks  */
    #define BUTTON_SEQ_TIME_AZ_TIME_MASK              (((0x00000001Lu << CYFLD_CSD_AZ_TIME__SIZE) - 1u) << CYFLD_CSD_AZ_TIME__OFFSET)

    /* Fourth-generation HW block SEQ_INIT_CNT register masks  */
    #define BUTTON_SEQ_INIT_CNT_CONV_CNT_MASK         (((0x00000001Lu << CYFLD_CSD_CONV_CNT__SIZE) - 1u) << CYFLD_CSD_CONV_CNT__OFFSET)

    /* Fourth-generation HW block SEQ_NORM_CNT register masks  */
    #define BUTTON_SEQ_NORM_CNT_CONV_CNT_MASK         (((0x00000001Lu << CYFLD_CSD_CONV_CNT__SIZE) - 1u) << CYFLD_CSD_CONV_CNT__OFFSET)

    /* Fourth-generation HW block SW_RES register masks  */
    #define BUTTON_SW_RES_RES_HCAV_MASK               (((0x00000001Lu << CYFLD_CSD_RES_HCAV__SIZE) - 1u) << CYFLD_CSD_RES_HCAV__OFFSET)
    #define BUTTON_SW_RES_RES_HCAG_MASK               (((0x00000001Lu << CYFLD_CSD_RES_HCAG__SIZE) - 1u) << CYFLD_CSD_RES_HCAG__OFFSET)
    #define BUTTON_SW_RES_RES_HCBV_MASK               (((0x00000001Lu << CYFLD_CSD_RES_HCBV__SIZE) - 1u) << CYFLD_CSD_RES_HCBV__OFFSET)
    #define BUTTON_SW_RES_RES_HCBG_MASK               (((0x00000001Lu << CYFLD_CSD_RES_HCBG__SIZE) - 1u) << CYFLD_CSD_RES_HCBG__OFFSET)
    #define BUTTON_SW_RES_RES_F1PM_MASK               (((0x00000001Lu << CYFLD_CSD_RES_F1PM__SIZE) - 1u) << CYFLD_CSD_RES_F1PM__OFFSET)
    #define BUTTON_SW_RES_RES_F2PT_MASK               (((0x00000001Lu << CYFLD_CSD_RES_F2PT__SIZE) - 1u) << CYFLD_CSD_RES_F2PT__OFFSET)

    #define BUTTON_SW_RES_RES_DEFAULT                 ((BUTTON_CSD_INIT_SWITCH_RES << CYFLD_CSD_RES_HCAV__OFFSET) | \
                                                                 (BUTTON_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBV__OFFSET))

    /* Fourth-generation HW block SW_HS_P_SEL register masks  */
    #define BUTTON_SW_HS_P_SEL_SW_HMPM_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMPM__SIZE) - 1u) << CYFLD_CSD_SW_HMPM__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMPM__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMPM__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPT_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMPT__SIZE) - 1u) << CYFLD_CSD_SW_HMPT__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMPT__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMPT__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPS_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMPS__SIZE) - 1u) << CYFLD_CSD_SW_HMPS__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMPS__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMPS__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMMA_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMMA__SIZE) - 1u) << CYFLD_CSD_SW_HMMA__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMMA_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMMA__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMMA_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMMA__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMMB_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMMB__SIZE) - 1u) << CYFLD_CSD_SW_HMMB__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMMB_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMMB__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMMB__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMCA_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMCA__SIZE) - 1u) << CYFLD_CSD_SW_HMCA__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMCA_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMCA__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMCA_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMCA__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMCB_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMCB__SIZE) - 1u) << CYFLD_CSD_SW_HMCB__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMCB_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMCB__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMCB_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMCB__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMRH_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HMRH__SIZE) - 1u) << CYFLD_CSD_SW_HMRH__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMRH_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HMRH__OFFSET)
    #define BUTTON_SW_HS_P_SEL_SW_HMRH_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HMRH__OFFSET)

    /* Fourth-generation HW block SW_SHIELD_SEL register masks  */
    #define BUTTON_SW_SHIELD_SEL_SW_HCAV_MASK         (((0x00000001Lu << CYFLD_CSD_SW_HCAV__SIZE) - 1u) << CYFLD_CSD_SW_HCAV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCAV_STATIC_OPEN  (BUTTON_STATIC_OPEN)
    #define BUTTON_SW_SHIELD_SEL_SW_HCAV_HSCMP        (BUTTON_HSCMP)
    #define BUTTON_SW_SHIELD_SEL_SW_HCAG_MASK         (((0x00000001Lu << CYFLD_CSD_SW_HCAG__SIZE) - 1u) << CYFLD_CSD_SW_HCAG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_MASK         (((0x00000001Lu << CYFLD_CSD_SW_HCBV__SIZE) - 1u) << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_HSCMP        (BUTTON_HSCMP << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_PHI1         (BUTTON_PHI1 << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_PHI2         (BUTTON_PHI2 << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_PHI1_HSCMP   (BUTTON_PHI1_HSCMP << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP   (BUTTON_PHI2_HSCMP << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_STATIC_OPEN  (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HCBV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_MASK         (((0x00000001Lu << CYFLD_CSD_SW_HCBG__SIZE) - 1u) << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_HSCMP        (BUTTON_HSCMP << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_PHI1         (BUTTON_PHI1 << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_PHI2         (BUTTON_PHI2 << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_PHI1_HSCMP   (BUTTON_PHI1_HSCMP << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_PHI2_HSCMP   (BUTTON_PHI2_HSCMP << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBG_STATIC_OPEN  (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HCBG__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCCV_MASK         (((0x00000001Lu << CYFLD_CSD_SW_HCCV__SIZE) - 1u) << CYFLD_CSD_SW_HCCV__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCCG_MASK         (((0x00000001Lu << CYFLD_CSD_SW_HCCG__SIZE) - 1u) << CYFLD_CSD_SW_HCCG__OFFSET)

    /* Fourth-generation HW block SEQ_START register masks  */
    #define BUTTON_SEQ_START_START_MASK               (((0x00000001Lu << CYFLD_CSD_START__SIZE) - 1u) << CYFLD_CSD_START__OFFSET)
    #define BUTTON_SEQ_START_SEQ_MODE_MASK            (((0x00000001Lu << CYFLD_CSD_SEQ_MODE__SIZE) - 1u) << CYFLD_CSD_SEQ_MODE__OFFSET)
    #define BUTTON_SEQ_START_ABORT_MASK               (((0x00000001Lu << CYFLD_CSD_ABORT__SIZE) - 1u) << CYFLD_CSD_ABORT__OFFSET)
    #define BUTTON_SEQ_START_DSI_START_EN_MASK        (((0x00000001Lu << CYFLD_CSD_DSI_START_EN__SIZE) - 1u) << CYFLD_CSD_DSI_START_EN__OFFSET)
    #define BUTTON_SEQ_START_AZ0_SKIP_MASK            (((0x00000001Lu << CYFLD_CSD_AZ0_SKIP__SIZE) - 1u) << CYFLD_CSD_AZ0_SKIP__OFFSET)
    #define BUTTON_SEQ_START_AZ1_SKIP_MASK            (((0x00000001Lu << CYFLD_CSD_AZ1_SKIP__SIZE) - 1u) << CYFLD_CSD_AZ1_SKIP__OFFSET)

    /* Fourth-generation HW block SW_CMP_P_SEL register masks  */
    #define BUTTON_SW_CMP_P_SEL_SW_SFPM_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFPM__SIZE) - 1u) << CYFLD_CSD_SW_SFPM__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPM_STATIC_OPEN   (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_SFPM__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPM_STATIC_CLOSE  (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_SFPM__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPT_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFPT__SIZE) - 1u) << CYFLD_CSD_SW_SFPT__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPT_STATIC_OPEN   (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_SFPT__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPT_STATIC_CLOSE  (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_SFPT__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPS_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFPS__SIZE) - 1u) << CYFLD_CSD_SW_SFPS__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPS_STATIC_OPEN   (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_SFPS__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFPS_STATIC_CLOSE  (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_SFPS__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFMA_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFMA__SIZE) - 1u) << CYFLD_CSD_SW_SFMA__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFMA_STATIC_OPEN   (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_SFMA__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFMA_STATIC_CLOSE  (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_SFMA__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFMB_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFMB__SIZE) - 1u) << CYFLD_CSD_SW_SFMB__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFCA_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFCA__SIZE) - 1u) << CYFLD_CSD_SW_SFCA__OFFSET)
    #define BUTTON_SW_CMP_P_SEL_SW_SFCB_MASK          (((0x00000001Lu << CYFLD_CSD_SW_SFCB__SIZE) - 1u) << CYFLD_CSD_SW_SFCB__OFFSET)

    /* Fourth-generation HW block CSDCMP register masks    */
    #define BUTTON_CSDCMP_CSDCMP_DISABLED             (0uL)
    #define BUTTON_CSDCMP_CSDCMP_EN_MASK              (((0x00000001Lu << CYFLD_CSD_CSDCMP_EN__SIZE) - 1u) << CYFLD_CSD_CSDCMP_EN__OFFSET)
    #define BUTTON_CSDCMP_POLARITY_SEL_MASK           (((0x00000001Lu << CYFLD_CSD_POLARITY_SEL__SIZE) - 1u) << CYFLD_CSD_POLARITY_SEL__OFFSET)
    #define BUTTON_CSDCMP_FEEDBACK_MODE_MASK          (((0x00000001Lu << CYFLD_CSD_FEEDBACK_MODE__SIZE) - 1u) << CYFLD_CSD_FEEDBACK_MODE__OFFSET)
    #define BUTTON_CSDCMP_AZ_EN_MASK                  (((0x00000001Lu << CYFLD_CSD_AZ_EN__SIZE) - 1u) << CYFLD_CSD_AZ_EN__OFFSET)

    /* Fourth-generation HW block SENSE_DUTY register masks  */
    #define BUTTON_SENSE_DUTY_SENSE_WIDTH_MASK        (((0x00000001Lu << CYFLD_CSD_SENSE_WIDTH__SIZE) - 1u) << CYFLD_CSD_SENSE_WIDTH__OFFSET)
    #define BUTTON_SENSE_DUTY_SENSE_POL_MASK          (((0x00000001Lu << CYFLD_CSD_SENSE_POL__SIZE) - 1u) << CYFLD_CSD_SENSE_POL__OFFSET)
    #define BUTTON_SENSE_DUTY_SENSE_POL_PHI_LOW       (0uL)
    #define BUTTON_SENSE_DUTY_SENSE_POL_PHI_HIGH      (BUTTON_SENSE_DUTY_SENSE_POL_MASK)
    #define BUTTON_SENSE_DUTY_OVERLAP_PHI1_MASK       (((0x00000001Lu << CYFLD_CSD_OVERLAP_PHI1__SIZE) - 1u) << CYFLD_CSD_OVERLAP_PHI1__OFFSET)
    #define BUTTON_SENSE_DUTY_OVERLAP_PHI2_MASK       (((0x00000001Lu << CYFLD_CSD_OVERLAP_PHI2__SIZE) - 1u) << CYFLD_CSD_OVERLAP_PHI2__OFFSET)

    /* Fourth-generation HW block SW_AMUXBUF_SEL register masks  */
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRBY_MASK        (((0x00000001Lu << CYFLD_CSD_SW_IRBY__SIZE) - 1u) << CYFLD_CSD_SW_IRBY__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRLB_MASK        (((0x00000001Lu << CYFLD_CSD_SW_IRLB__SIZE) - 1u) << CYFLD_CSD_SW_IRLB__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRLB_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_IRLB__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_ICA_MASK         (((0x00000001Lu << CYFLD_CSD_SW_ICA__SIZE) - 1u) << CYFLD_CSD_SW_ICA__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_ICB_MASK         (((0x00000001Lu << CYFLD_CSD_SW_ICB__SIZE) - 1u) << CYFLD_CSD_SW_ICB__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_ICB_PHI2_HSCMP   (BUTTON_PHI2_HSCMP << CYFLD_CSD_SW_ICB__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRLI_MASK        (((0x00000001Lu << CYFLD_CSD_SW_IRLI__SIZE) - 1u) << CYFLD_CSD_SW_IRLI__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRH_MASK         (((0x00000001Lu << CYFLD_CSD_SW_IRH__SIZE) - 1u) << CYFLD_CSD_SW_IRH__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRH_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_IRH__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRL_MASK         (((0x00000001Lu << CYFLD_CSD_SW_IRL__SIZE) - 1u) << CYFLD_CSD_SW_IRL__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_IRL_MASK         (((0x00000001Lu << CYFLD_CSD_SW_IRL__SIZE) - 1u) << CYFLD_CSD_SW_IRL__OFFSET)
    #define BUTTON_SW_AMUXBUF_SEL_SW_DEFAULT          (0x00000000Lu)
    #define BUTTON_SW_AMUXBUF_SEL_SW_ICB_PHI2         (0x00030000uL)
    #define BUTTON_SW_AMUXBUF_SEL_SW_ICB_PHI1         (BUTTON_PHI1 << CYFLD_CSD_SW_ICB__OFFSET)

    /* Fourth-generation HW block SW_FW_TANK_SEL register masks  */
    #define BUTTON_SW_HS_N_SEL_SW_HCCC_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HCCC__SIZE) - 1u) << CYFLD_CSD_SW_HCCC__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCCC_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HCCC__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCCC_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HCCC__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCCD_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HCCD__SIZE) - 1u) << CYFLD_CSD_SW_HCCD__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCCD_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HCCD__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCCD_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HCCD__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCRH_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HCRH__SIZE) - 1u) << CYFLD_CSD_SW_HCRH__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCRH_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HCRH__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCRH_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HCRH__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCRL_MASK           (((0x00000001Lu << CYFLD_CSD_SW_HCRL__SIZE) - 1u) << CYFLD_CSD_SW_HCRL__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCRL_STATIC_OPEN    (BUTTON_STATIC_OPEN << CYFLD_CSD_SW_HCRL__OFFSET)
    #define BUTTON_SW_HS_N_SEL_SW_HCRL_STATIC_CLOSE   (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_HCRL__OFFSET)

    /* Fourth-generation HW block SW_FW_MOD_SEL register masks  */
    #define BUTTON_SW_FW_MOD_SEL_SW_F1PM_MASK         (((0x00000001Lu << CYFLD_CSD_SW_F1PM__SIZE) - 1u) << CYFLD_CSD_SW_F1PM__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F1PM__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_F1MA_MASK         (((0x00000001Lu << CYFLD_CSD_SW_F1MA__SIZE) - 1u) << CYFLD_CSD_SW_F1MA__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_F1MA_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F1MA__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_F1CA_MASK         (((0x00000001Lu << CYFLD_CSD_SW_F1CA__SIZE) - 1u) << CYFLD_CSD_SW_F1CA__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_F1CA_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F1CA__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_C1CC_MASK         (((0x00000001Lu << CYFLD_CSD_SW_C1CC__SIZE) - 1u) << CYFLD_CSD_SW_C1CC__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_C1CC_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_C1CC__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_C1CD_MASK         (((0x00000001Lu << CYFLD_CSD_SW_C1CD__SIZE) - 1u) << CYFLD_CSD_SW_C1CD__OFFSET)
    #define BUTTON_SW_FW_MOD_SEL_SW_C1CD_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_C1CD__OFFSET)

    /* Fourth-generation HW block SW_FW_TANK_SEL register masks  */
    #define BUTTON_SW_FW_TANK_SEL_SW_F2PT_MASK        (((0x00000001Lu << CYFLD_CSD_SW_F2PT__SIZE) - 1u) << CYFLD_CSD_SW_F2PT__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F2PT__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2CB_MASK        (((0x00000001Lu << CYFLD_CSD_SW_F2CB__SIZE) - 1u) << CYFLD_CSD_SW_F2CB__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2CB_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F2CB__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2CB_PHI2        (BUTTON_PHI2 << CYFLD_CSD_SW_F2CB__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2MA_MASK        (((0x00000001Lu << CYFLD_CSD_SW_F2MA__SIZE) - 1u) << CYFLD_CSD_SW_F2MA__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2MA_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F2MA__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2CA_MASK        (((0x00000001Lu << CYFLD_CSD_SW_F2CA__SIZE) - 1u) << CYFLD_CSD_SW_F2CA__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_F2CA_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_F2CA__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_C2CC_MASK        (((0x00000001Lu << CYFLD_CSD_SW_C2CC__SIZE) - 1u) << CYFLD_CSD_SW_C2CC__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_C2CC_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_C2CC__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_C2CD_MASK        (((0x00000001Lu << CYFLD_CSD_SW_C2CD__SIZE) - 1u) << CYFLD_CSD_SW_C2CD__OFFSET)
    #define BUTTON_SW_FW_TANK_SEL_SW_C2CD_STATIC_CLOSE (BUTTON_STATIC_CLOSE << CYFLD_CSD_SW_C2CD__OFFSET)

    /* Fourth-generation HW block CTANK masks    */
    #define BUTTON_CTANK_DR_VDDIO                     (0x0u)
    #define BUTTON_CTANK_DR_VSSIO                     (BUTTON_DR_MASK << BUTTON_CSH_SHIFT)

    #define BUTTON_CALIBRATE_SW_REFGEN_SEL            (BUTTON_SW_REFGEN_SEL_SW_SGR_MASK)

    #define BUTTON_DEFAULT_REFGEN_GAIN                (BUTTON_CSD_GAIN << CYFLD_CSD_GAIN__OFFSET)
    #define BUTTON_REFGEN_LV                          (BUTTON_REFGEN_REFGEN_EN_MASK |\
                                                                BUTTON_REFGEN_BYPASS_MASK)

    #define BUTTON_REFGEN_HV                          (BUTTON_REFGEN_REFGEN_EN_MASK |\
                                                                BUTTON_REFGEN_RES_EN_MASK |\
                                                                BUTTON_DEFAULT_REFGEN_GAIN)

    #define BUTTON_SENSE_PERIOD_LFSR_SIZE_SHIFT       (CYFLD_CSD_LFSR_SIZE__OFFSET)

    /* Initial PRS mode */
    #define BUTTON_PRS_8_CONFIG                       (BUTTON_CLK_SOURCE_PRS8)
    #define BUTTON_PRS_12_CONFIG                      (BUTTON_CLK_SOURCE_PRS12)

    /* SW_HS_P_SEL switches state for Coarse initialization of CMOD (sense path) */
    #if (BUTTON_ENABLE == BUTTON_CSD_EN)
        #if (BUTTON_CSD__CMOD_PAD == BUTTON_CMOD_CONNECTION)
            #define BUTTON_HS_P_SEL_COARSE_CMOD       (BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #elif (BUTTON_CSD__CSHIELD_PAD == BUTTON_CMOD_CONNECTION)
            #define BUTTON_HS_P_SEL_COARSE_CMOD       (BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #else
            #define BUTTON_HS_P_SEL_COARSE_CMOD       (BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #endif /* (BUTTON_CSD__CMOD_PAD == BUTTON_CMOD_CONNECTION) */
    #else
        #define BUTTON_HS_P_SEL_COARSE_CMOD           (0x00000000uL)
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_EN) */

    #if ((0u != BUTTON_CSD_SHIELD_TANK_EN) && (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_HS_P_SEL_COARSE_TANK       (BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
            #define BUTTON_HS_P_SEL_SCAN_TANK         (BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (BUTTON_CSD__CSHIELD_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_HS_P_SEL_COARSE_TANK       (BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
            #define BUTTON_HS_P_SEL_SCAN_TANK         (BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (BUTTON_CSD__CMOD_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_HS_P_SEL_COARSE_TANK       (BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
            #define BUTTON_HS_P_SEL_SCAN_TANK         (BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define BUTTON_HS_P_SEL_COARSE_TANK       (BUTTON_SW_HS_P_SEL_SW_HMMA_STATIC_CLOSE)
            #define BUTTON_HS_P_SEL_SCAN_TANK         (BUTTON_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
        #endif /* (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION) */
        #define BUTTON_SW_HS_P_SEL_SCAN               (BUTTON_HS_P_SEL_SCAN_TANK)
    #else
        #define BUTTON_HS_P_SEL_COARSE_TANK           (0x00000000uL)
        #define BUTTON_SW_HS_P_SEL_SCAN               (BUTTON_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
    #endif /* ((0u != BUTTON_CSD__CSD_CSH_TANK_ENABLE) && (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)) */

    #define BUTTON_SW_HS_P_SEL_COARSE                 (BUTTON_HS_P_SEL_COARSE_CMOD | BUTTON_HS_P_SEL_COARSE_TANK)

    /***************************************
    * Fourth-generation HW block Registers shifts
    ***************************************/
    #define BUTTON_SHIELD_DELAY_SHIFT                 (CYFLD_CSD_SHIELD_DELAY__OFFSET)
    #define BUTTON_LFSR_SIZE_SHIFT                    (CYFLD_CSD_LFSR_SIZE__OFFSET)
    #define BUTTON_RESULT_VAL1_BAD_CONVS_SHIFT        (CYFLD_CSD_BAD_CONVS__OFFSET)
    #define BUTTON_SW_SHIELD_SEL_SW_HCBV_SHIFT        (CYFLD_CSD_SW_HCBV__OFFSET)

    /***************************************
    * LFSR Register masks
    ***************************************/
    #define BUTTON_RESOLUTION_OFFSET                  (0u)
    #define BUTTON_LFSR_TABLE_SIZE                    (4u)

    #define BUTTON_PRS_LENGTH_2_BITS                  (0x00000003Lu)
    #define BUTTON_PRS_LENGTH_3_BITS                  (0x00000007Lu)
    #define BUTTON_PRS_LENGTH_4_BITS                  (0x0000000FLu)
    #define BUTTON_PRS_LENGTH_5_BITS                  (0x0000001FLu)
    #define BUTTON_PRS_LENGTH_8_BITS                  (0x000000FFLu)
    #define BUTTON_PRS_LENGTH_12_BITS                 (0x00000FFFLu)

    #if(BUTTON_ENABLE != BUTTON_CSDV2_REF9P6UA_EN)
        #define BUTTON_SNSCLK_SSC1_PERIOD             (3u)
        #define BUTTON_SNSCLK_SSC2_PERIOD             (7u)
        #define BUTTON_SNSCLK_SSC3_PERIOD             (15u)
        #define BUTTON_SNSCLK_SSC4_PERIOD             (31u)

        #define BUTTON_SNSCLK_SSC1_RANGE              (1u)
        #define BUTTON_SNSCLK_SSC2_RANGE              (3u)
        #define BUTTON_SNSCLK_SSC3_RANGE              (7u)
        #define BUTTON_SNSCLK_SSC4_RANGE              (15u)
    #else

        #define BUTTON_SNSCLK_SSC1_PERIOD             (63u)
        #define BUTTON_SNSCLK_SSC2_PERIOD             (127u)
        #define BUTTON_SNSCLK_SSC3_PERIOD             (511u)
        #define BUTTON_SNSCLK_SSC4_PERIOD             (1023u)

        #define BUTTON_SNSCLK_SSC1_RANGE              (16u)
        #define BUTTON_SNSCLK_SSC2_RANGE              (16u)
        #define BUTTON_SNSCLK_SSC3_RANGE              (16u)
        #define BUTTON_SNSCLK_SSC4_RANGE              (16u)
    #endif /* (BUTTON_ENABLE != BUTTON_CSDV2_REF9P6UA_EN) */

    #define BUTTON_LFSR_DITHER_PERCENTAGE             (10uL)
    #define BUTTON_SNSCLK_SSC1_THRESHOLD              (BUTTON_LFSR_DITHER_PERCENTAGE * BUTTON_SNSCLK_SSC1_RANGE)
    #define BUTTON_SNSCLK_SSC2_THRESHOLD              (BUTTON_LFSR_DITHER_PERCENTAGE * BUTTON_SNSCLK_SSC2_RANGE)
    #define BUTTON_SNSCLK_SSC3_THRESHOLD              (BUTTON_LFSR_DITHER_PERCENTAGE * BUTTON_SNSCLK_SSC3_RANGE)
    #define BUTTON_SNSCLK_SSC4_THRESHOLD              (BUTTON_LFSR_DITHER_PERCENTAGE * BUTTON_SNSCLK_SSC4_RANGE)

    #define BUTTON_HFCLK_SNSCLK_FACTOR                (160u)
    #define BUTTON_SKIP_INIT_CYCLES                   (4u)

#else

    /* Third-generation HW block CSD_CONFIG register masks    */
    #define BUTTON_CONFIG_BYPASS_SEL_MASK             (0x00000004Lu)
    #define BUTTON_CONFIG_FILTER_EN_MASK              (0x00000008Lu)
    #define BUTTON_CONFIG_DUAL_CAP_EN_MASK            (0x00000010Lu)
    #define BUTTON_CONFIG_PRS_CLEAR_MASK              (0x00000020Lu)
    #define BUTTON_CONFIG_PRS_SELECT_MASK             (0x00000040Lu)
    #define BUTTON_CONFIG_PRS_12_8_MASK               (0x00000080Lu)
    #define BUTTON_CONFIG_SHIELD_DELAY_MASK           (0x00000600Lu)
    #define BUTTON_CONFIG_SENSE_COMP_BW_MASK          (0x00000800Lu)
    #define BUTTON_CONFIG_SENSE_EN_MASK               (0x00001000Lu)
    #define BUTTON_CONFIG_REFBUF_EN_MASK              (0x00002000Lu)
    #define BUTTON_CONFIG_COMP_MODE_MASK              (0x00004000Lu)
    #define BUTTON_CONFIG_COMP_PIN_MASK               (0x00008000Lu)
    #define BUTTON_CONFIG_POLARITY_MASK               (0x00010000Lu)
    #define BUTTON_CONFIG_POLARITY2_MASK              (0x00020000Lu)
    #define BUTTON_CONFIG_MUTUALCAP_EN_MASK           (0x00040000Lu)
    #define BUTTON_CONFIG_SENSE_COMP_EN_MASK          (0x00080000Lu)
    #define BUTTON_CONFIG_REFBUF_OUTSEL_MASK          (0x00200000Lu)
    #define BUTTON_CONFIG_SENSE_INSEL_MASK            (0x00400000Lu)
    #define BUTTON_CONFIG_REFBUF_DRV_MASK             (0x01800000Lu)
    #define BUTTON_CONFIG_DDFTSEL_MASK                (0x1C000000Lu)
    #define BUTTON_CONFIG_ADFTEN_MASK                 (0x20000000Lu)
    #define BUTTON_CONFIG_DDFT_COMP_MASK              (0x40000000Lu)
    #define BUTTON_CONFIG_ENABLE_MASK                 (0x80000000Lu)

    /* Third-generation HW block CSD_IDAC register masks  */
    #define BUTTON_IDAC_MOD_MASK                      (0x000000FFLu)
    #define BUTTON_IDAC_MOD_MODE_MASK                 (0x00000300Lu)
    #define BUTTON_IDAC_MOD_RANGE_MASK                (0x00000400Lu)
    #define BUTTON_IDAC_POLARITY1_MIR_MASK            (0x00001000Lu)
    #define BUTTON_IDAC_COMP_MASK                     (0x007F0000Lu)
    #define BUTTON_IDAC_COMP_MODE_MASK                (0x03000000Lu)
    #define BUTTON_IDAC_COMP_RANGE_MASK               (0x04000000Lu)
    #define BUTTON_IDAC_POLARITY2_MIR_MASK            (0x10000000Lu)
    #define BUTTON_IDAC_FEEDBACK_MODE_MASK            (0x40000000Lu)
    #define BUTTON_IDAC_MOD_MODE_OFF                  (0x00000000Lu)
    #define BUTTON_IDAC_MOD_MODE_FIXED                (0x00000100Lu)
    #define BUTTON_IDAC_MOD_MODE_VARIABLE             (0x00000200Lu)
    #define BUTTON_IDAC_MOD_MODE_DSI                  (0x00000300Lu)
    #define BUTTON_IDAC_COMP_MODE_OFF                 (0x00000000Lu)
    #define BUTTON_IDAC_COMP_MODE_FIXED               (0x01000000Lu)
    #define BUTTON_IDAC_COMP_MODE_VARIABLE            (0x02000000Lu)
    #define BUTTON_IDAC_COMP_MODE_DSI                 (0x03000000Lu)
    #define BUTTON_IDAC_MOD_VALUE_SHIFT               (0u)
    #define BUTTON_IDAC_MOD_CFG_MASK                  (BUTTON_IDAC_POLARITY1_MIR_MASK |\
                                                                 BUTTON_IDAC_MOD_RANGE_MASK |\
                                                                 BUTTON_IDAC_MOD_MODE_MASK |\
                                                                 BUTTON_IDAC_MOD_MASK)

    #define BUTTON_IDAC_COMP_CFG_MASK                 (BUTTON_IDAC_POLARITY2_MIR_MASK |\
                                                                 BUTTON_IDAC_COMP_RANGE_MASK |\
                                                                 BUTTON_IDAC_COMP_MODE_MASK |\
                                                                 BUTTON_IDAC_COMP_MASK)

    /* Third-generation HW block CSD_COUNTER register masks   */
    #define BUTTON_COUNTER_COUNTER_MASK               (0x0000FFFFLu)
    #define BUTTON_COUNTER_PERIOD_MASK                (0xFFFF0000Lu)

    /* Third-generation HW block CSD_STATUS register masks    */
    #define BUTTON_STATUS_CSD_CHARGE_MASK             (0x00000001Lu)
    #define BUTTON_STATUS_CSD_SENSE_MASK              (0x00000002Lu)
    #define BUTTON_STATUS_COMP_OUT_MASK               (0x00000004Lu)
    #define BUTTON_STATUS_SAMPLE                      (0x00000008Lu)

    /* Third-generation HW block CSD_INTR, CSD_INTR_SET register masks  */
    #define BUTTON_INTR_CSD_MASK                      (0x00000001Lu)
    #define BUTTON_INTR_SET_CSD_MASK                  (0x00000001Lu)
    #define BUTTON_INTR_SET_MASK                      (BUTTON_INTR_SET_CSD_MASK)

    /* Third-generation HW block CSD_PWM register masks   */
    #define BUTTON_PWM_COUNT_MASK                     (0x0000000FLu)
    #define BUTTON_PWM_SEL_MASK                       (0x00000030Lu)

    /* Third-generation HW block CSD_TRIM1/2 (for IDAC) register masks    */
    #define BUTTON_IDAC_TRIM2_MOD_SNK_MASK            (0x0000000FLu)
    #define BUTTON_IDAC_TRIM2_COMP_SNK_MASK           (0x000000F0Lu)
    #define BUTTON_IDAC_TRIM1_MOD_SRC_MASK            (0x0000000FLu)
    #define BUTTON_IDAC_TRIM1_COMP_SRC_MASK           (0x000000F0Lu)

    /* Third-generation HW block CSD_TRIM FLASH register masks    */
    #define BUTTON_SFLASH_TRIM_IDAC_MOD_MASK          (0x0FLu)
    #define BUTTON_SFLASH_TRIM_IDAC_COMP_MASK         (0xF0Lu)

    /* Third-generation HW block Clock register masks    */
    #define BUTTON_SNSCLK_CMD_DIV_SHIFT               (0u)
    #define BUTTON_SNSCLK_CMD_PA_DIV_SHIFT            (8u)
    #define BUTTON_SNSCLK_CMD_DISABLE_SHIFT           (30u)
    #define BUTTON_SNSCLK_CMD_ENABLE_SHIFT            (31u)
    #define BUTTON_SNSCLK_CMD_DISABLE_MASK            ((uint32)((uint32)1u << BUTTON_SNSCLK_CMD_DISABLE_SHIFT))
    #define BUTTON_SNSCLK_CMD_ENABLE_MASK             ((uint32)((uint32)1u << BUTTON_SNSCLK_CMD_ENABLE_SHIFT))

    /* Third-generation HW block CTANK masks    */
    #define BUTTON_CTANK_DR_VDDIO                     (0x0u)
    #define BUTTON_CTANK_DR_VSSIO                     (BUTTON_DR_MASK << BUTTON_CSH_SHIFT)

    #define BUTTON_RESOLUTION_OFFSET                  (16u)
    #define BUTTON_PRS_LENGTH_12_BITS                 (0x00000FFFLu)
    #define BUTTON_PRS_LENGTH_8_BITS                  (0x000000FFLu)
    #define BUTTON_CSD_PRS_12_BIT                     (BUTTON_CONFIG_PRS_12_8_MASK)
    #define BUTTON_PRS_MODE_MASK                      (BUTTON_CONFIG_PRS_12_8_MASK)

    /* Third-generation HW block Set IDAC polarity */
    #if (BUTTON_CSD_IDAC_CONFIG == BUTTON_IDAC_SINKING)
        #define BUTTON_DEFAULT_IDAC_POLARITY          BUTTON_CONFIG_POLARITY_MASK
        #define BUTTON_CSH_DR_CONFIG                  BUTTON_CTANK_DR_VDDIO
    #else
        #define BUTTON_DEFAULT_IDAC_POLARITY          (0u)
        #define BUTTON_CSH_DR_CONFIG                  BUTTON_CTANK_DR_VSSIO
    #endif /* (BUTTON_CSD_IDAC_CONFIG == BUTTON_IDAC_SINKING) */
    
    /* Defining default CSD configuration according to settings in customizer. */
    #define BUTTON_CSD_ENABLE_MASK                    (BUTTON_CONFIG_ENABLE_MASK |\
                                                                BUTTON_CONFIG_SENSE_EN_MASK |\
                                                                BUTTON_CONFIG_SENSE_COMP_EN_MASK)
    
    /***************************************
    * Third-generation HW block Registers shifts
    ***************************************/
    #define BUTTON_SHIELD_DELAY_SHIFT                 (0x09u)
    #define BUTTON_IDAC_COMP_DATA_OFFSET              (16u)

#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */

/* Third-generation and Fourth-generation HW block Clock register masks    */
#define BUTTON_MODCLK_CMD_DIV_SHIFT                   (0u)
#define BUTTON_MODCLK_CMD_PA_DIV_SHIFT                (8u)
#define BUTTON_MODCLK_CMD_DISABLE_SHIFT               (30u)
#define BUTTON_MODCLK_CMD_ENABLE_SHIFT                (31u)
#define BUTTON_MODCLK_CMD_DISABLE_MASK                ((uint32)((uint32)1u << BUTTON_MODCLK_CMD_DISABLE_SHIFT))
#define BUTTON_MODCLK_CMD_ENABLE_MASK                 ((uint32)((uint32)1u << BUTTON_MODCLK_CMD_ENABLE_SHIFT))

/* CintB and CintA pin registers  */
#if ((BUTTON_ENABLE == BUTTON_CSX_EN) || \
     (BUTTON_ENABLE == BUTTON_ISX_EN))

    #define BUTTON_CintA_PC_REG                       (* (reg32 *) BUTTON_CintA__0__PC)
    #define BUTTON_CintA_DR_REG                       (* (reg32 *) BUTTON_CintA__0__DR)
    #define BUTTON_CintA_HSIOM_REG                    (* (reg32 *) BUTTON_CintA__0__HSIOM)
    #define BUTTON_CintB_PC_REG                       (* (reg32 *) BUTTON_CintB__0__PC)
    #define BUTTON_CintB_DR_REG                       (* (reg32 *) BUTTON_CintB__0__DR)
    #define BUTTON_CintB_HSIOM_REG                    (* (reg32 *) BUTTON_CintB__0__HSIOM)

    #define BUTTON_CintA_PC_PTR                       ((reg32 *) BUTTON_CintA__0__PC)
    #define BUTTON_CintA_DR_PTR                       ((reg32 *) BUTTON_CintA__0__DR)
    #define BUTTON_CintA_HSIOM_PTR                    ((reg32 *) BUTTON_CintA__0__HSIOM)
    #define BUTTON_CintB_PC_PTR                       ((reg32 *) BUTTON_CintB__0__PC)
    #define BUTTON_CintB_DR_PTR                       ((reg32 *) BUTTON_CintB__0__DR)
    #define BUTTON_CintB_HSIOM_PTR                    ((reg32 *) BUTTON_CintB__0__HSIOM)

    /* CintB and CintA pin masks                                                */
    #define BUTTON_CintA_HSIOM_MASK                   ((uint32)BUTTON_CintA__0__HSIOM_MASK)
    #define BUTTON_CintA_HSIOM_SHIFT                  ((uint32)BUTTON_CintA__0__HSIOM_SHIFT)
    #define BUTTON_CintA_SHIFT                        ((uint32)BUTTON_CintA__0__SHIFT)

    #define BUTTON_CintB_HSIOM_MASK                   ((uint32)BUTTON_CintB__0__HSIOM_MASK)
    #define BUTTON_CintB_HSIOM_SHIFT                  ((uint32)BUTTON_CintB__0__HSIOM_SHIFT)
    #define BUTTON_CintB_SHIFT                        ((uint32)BUTTON_CintB__0__SHIFT)

#endif

/* CMOD and CSH capacitor port-pins registers */
#if (BUTTON_ENABLE == BUTTON_CSD_EN)

    #define BUTTON_CSH_PC_REG                         (* (reg32 *) BUTTON_Csh__PC)
    #define BUTTON_CSH_PC_PTR                         (  (reg32 *) BUTTON_Csh__PC)
    #define BUTTON_CSH_DR_REG                         (* (reg32 *) BUTTON_Csh__DR)
    #define BUTTON_CSH_DR_PTR                         (  (reg32 *) BUTTON_Csh__DR)
    #define BUTTON_CSH_HSIOM_REG                      (* (reg32 *) BUTTON_Csh__0__HSIOM)
    #define BUTTON_CSH_HSIOM_PTR                      (  (reg32 *) BUTTON_Csh__0__HSIOM)

    #define BUTTON_CMOD_PC_REG                        (* (reg32 *) BUTTON_Cmod__PC)
    #define BUTTON_CMOD_PC_PTR                        (  (reg32 *) BUTTON_Cmod__PC)
    #define BUTTON_CMOD_PC_SHIFT                      (BUTTON_Cmod__0__SHIFT * 3u)
    #define BUTTON_CMOD_DR_REG                        (* (reg32 *) BUTTON_Cmod__DR)
    #define BUTTON_CMOD_DR_PTR                        (  (reg32 *) BUTTON_Cmod__DR)
    #define BUTTON_CMOD_DR_SHIFT                      BUTTON_Cmod__0__SHIFT
    #define BUTTON_CMOD_HSIOM_REG                     (* (reg32 *) BUTTON_Cmod__0__HSIOM)
    #define BUTTON_CMOD_HSIOM_PTR                     (  (reg32 *) BUTTON_Cmod__0__HSIOM)

    /* Cmod and Csh pin masks */
    #define BUTTON_CMOD_HSIOM_MASK                    BUTTON_Cmod__0__HSIOM_MASK
    #define BUTTON_CMOD_HSIOM_SHIFT                   BUTTON_Cmod__0__HSIOM_SHIFT
    #define BUTTON_CSH_HSIOM_MASK                     BUTTON_Csh__0__HSIOM_MASK
    #define BUTTON_CSH_HSIOM_SHIFT                    BUTTON_Csh__0__HSIOM_SHIFT
    #define BUTTON_CSH_SHIFT                          BUTTON_Csh__0__SHIFT
    #define BUTTON_CSH_PC_SHIFT                       (BUTTON_Csh__0__SHIFT * 3u)

#endif

/* GPIO register masks */
#define BUTTON_DR_MASK                                (0x00000001Lu)

#define BUTTON_CSH_PC_MASK                            (0x00000007Lu)
#define BUTTON_CSH_PC_STRONG_MODE                     (0x00000006Lu)

#define BUTTON_HSIOM_SEL_MASK                         (0x0000000FLu)
#define BUTTON_HSIOM_SEL_GPIO                         (0x00000000Lu)
#define BUTTON_HSIOM_SEL_CSD_SENSE                    (0x00000004Lu)
#define BUTTON_HSIOM_SEL_CSD_SHIELD                   (0x00000005Lu)
#define BUTTON_HSIOM_SEL_AMUXA                        (0x00000006Lu)
#define BUTTON_HSIOM_SEL_AMUXB                        (0x00000007Lu)
#define BUTTON_HSIOM_SEL_ACT_0                        (0x00000008Lu)

#define BUTTON_GPIO_PC_MASK                           (0x00000007Lu)
#define BUTTON_GPIO_HIGHZ                             (0x00000000Lu)
#define BUTTON_GPIO_STRGDRV                           (0x00000006Lu)

#define BUTTON_SNS_GROUND_CONNECT                     (0x00000006Lu)

#define BUTTON_GPIO_PC_BIT_SIZE                       (3u)

#define BUTTON_EXT_CAP_DISCHARGE_TIME                 (1u)

#define BUTTON_SW_WATCHDOG_SCAN_ALL                   (5000000u)

#if (BUTTON_ENABLE == BUTTON_CSD_EN)
    #define BUTTON_EXT_CAP_HSIOM_PTR                  ((reg32 *) BUTTON_Cmod__0__HSIOM)
    #define BUTTON_EXT_CAP_DR_PTR                     ((reg32 *) BUTTON_Cmod__DR)
    #define BUTTON_EXT_CAP_PC_PTR                     ((reg32 *) BUTTON_Cmod__PC)
#else
    #define BUTTON_EXT_CAP_HSIOM_PTR                  ((reg32 *) BUTTON_CintA__0__HSIOM)
    #define BUTTON_EXT_CAP_DR_PTR                     ((reg32 *) BUTTON_CintA__DR)
    #define BUTTON_EXT_CAP_PC_PTR                     ((reg32 *) BUTTON_CintA__PC)
#endif

#if (BUTTON_ENABLE == BUTTON_CSD_EN)
    #define BUTTON_EXT_CAP_HSIOM_MASK_CMOD                (BUTTON_Cmod__0__HSIOM_MASK)
    #define BUTTON_EXT_CAP_DR_MASK_CMOD                   (BUTTON_Cmod__0__MASK)
    #define BUTTON_EXT_CAP_PC_MASK_CMOD                   ((uint32)BUTTON_GPIO_PC_MASK << BUTTON_CMOD_PC_SHIFT)
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CMOD             ((uint32)BUTTON_GPIO_STRGDRV << BUTTON_CMOD_PC_SHIFT)
#else
    #define BUTTON_EXT_CAP_HSIOM_MASK_CMOD                (0u)
    #define BUTTON_EXT_CAP_DR_MASK_CMOD                   (0u)
    #define BUTTON_EXT_CAP_PC_MASK_CMOD                   (0u)
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CMOD             (0u)
#endif

#if((BUTTON_ENABLE == BUTTON_CSD_EN) && \
    (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
    (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN))
    #define BUTTON_EXT_CAP_HSIOM_MASK_CSH                 (BUTTON_Csh__0__HSIOM_MASK)
    #define BUTTON_EXT_CAP_DR_MASK_CSH                    (BUTTON_Csh__0__MASK)
    #define BUTTON_EXT_CAP_PC_MASK_CSH                    ((uint32)BUTTON_GPIO_PC_MASK << BUTTON_CSH_PC_SHIFT)
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CSH             ((uint32)BUTTON_GPIO_STRGDRV << BUTTON_CSH_PC_SHIFT)
#else
    #define BUTTON_EXT_CAP_HSIOM_MASK_CSH                 (0u)
    #define BUTTON_EXT_CAP_DR_MASK_CSH                    (0u)
    #define BUTTON_EXT_CAP_PC_MASK_CSH                    (0u)
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CSH              (0u)
#endif

#if (BUTTON_ENABLE == BUTTON_CSX_EN)
    #define BUTTON_EXT_CAP_HSIOM_MASK_CINTA               (BUTTON_CintA__0__HSIOM_MASK)
    #define BUTTON_EXT_CAP_DR_MASK_CINTA                  (BUTTON_CintA__0__MASK)
    #define BUTTON_EXT_CAP_PC_MASK_CINTA                  ((uint32)BUTTON_GPIO_PC_MASK << (BUTTON_CintA_SHIFT * 3u))
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CINTA            ((uint32)BUTTON_GPIO_STRGDRV << (BUTTON_CintA_SHIFT * 3u))
    #define BUTTON_EXT_CAP_HSIOM_MASK_CINTB               (BUTTON_CintB__0__HSIOM_MASK)
    #define BUTTON_EXT_CAP_DR_MASK_CINTB                  (BUTTON_CintB__0__MASK)
    #define BUTTON_EXT_CAP_PC_MASK_CINTB                  ((uint32)BUTTON_GPIO_PC_MASK << (BUTTON_CintB_SHIFT * 3u))
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CINTB            ((uint32)BUTTON_GPIO_STRGDRV << (BUTTON_CintB_SHIFT * 3u))
#else
    #define BUTTON_EXT_CAP_HSIOM_MASK_CINTA               (0u)
    #define BUTTON_EXT_CAP_DR_MASK_CINTA                  (0u)
    #define BUTTON_EXT_CAP_PC_MASK_CINTA                  (0u)
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CINTA            (0u)
    #define BUTTON_EXT_CAP_HSIOM_MASK_CINTB               (0u)
    #define BUTTON_EXT_CAP_DR_MASK_CINTB                  (0u)
    #define BUTTON_EXT_CAP_PC_MASK_CINTB                  (0u)
    #define BUTTON_EXT_CAP_PC_STRONG_CFG_CINTB            (0u)
#endif

/* External capacitors related to the all enabled methods */
#define BUTTON_EXT_CAP_HSIOM_MASK                         (BUTTON_EXT_CAP_HSIOM_MASK_CMOD | \
                                                                     BUTTON_EXT_CAP_HSIOM_MASK_CSH | \
                                                                     BUTTON_EXT_CAP_HSIOM_MASK_CINTA | \
                                                                     BUTTON_EXT_CAP_HSIOM_MASK_CINTB)

#define BUTTON_EXT_CAP_DR_MASK                            (BUTTON_EXT_CAP_DR_MASK_CMOD | \
                                                                     BUTTON_EXT_CAP_DR_MASK_CSH | \
                                                                     BUTTON_EXT_CAP_DR_MASK_CINTA | \
                                                                     BUTTON_EXT_CAP_DR_MASK_CINTB)

#define BUTTON_EXT_CAP_PC_MASK                            (BUTTON_EXT_CAP_PC_MASK_CMOD | \
                                                                     BUTTON_EXT_CAP_PC_MASK_CSH | \
                                                                     BUTTON_EXT_CAP_PC_MASK_CINTA | \
                                                                     BUTTON_EXT_CAP_PC_MASK_CINTB)

#define BUTTON_EXT_CAP_PC_STRONG_CFG                      (BUTTON_EXT_CAP_PC_STRONG_CFG_CMOD | \
                                                                     BUTTON_EXT_CAP_PC_STRONG_CFG_CSH | \
                                                                     BUTTON_EXT_CAP_PC_STRONG_CFG_CINTA | \
                                                                     BUTTON_EXT_CAP_PC_STRONG_CFG_CINTB)


/***************************************
* API Constants
***************************************/

#define BUTTON_MOD_CSD_CLK_12MHZ                      (12000000uL)
#define BUTTON_MOD_CSD_CLK_24MHZ                      (24000000uL)
#define BUTTON_MOD_CSD_CLK_48MHZ                      (48000000uL)

#if (BUTTON_ENABLE == BUTTON_CSDV2)
    #define BUTTON_MIN_SNS_CLK_DIVIDER                (4000u)
#else
    #define BUTTON_MIN_SNS_CLK_DIVIDER                (1000u)
#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */

/* ISR Number to work with CyLib functions */
#define BUTTON_ISR_NUMBER                             (BUTTON_ISR__INTC_NUMBER)

/* Multi-frequency scanning constants */
#define BUTTON_FREQ_CHANNEL_0                         (0u)
#define BUTTON_FREQ_CHANNEL_1                         (1u)
#define BUTTON_FREQ_CHANNEL_2                         (2u)

/* Bit-mask which says that scanning is not complete */
#define BUTTON_SW_STS_BUSY                            (BUTTON_STATUS_CSD0_MASK)
#define BUTTON_NOT_BUSY                               (0u)

#define BUTTON_WDGT_SW_STS_BUSY                       (BUTTON_STATUS_WDGT0_BUSY_MASK)

#define BUTTON_DELAY_EXTRACYCLES_NUM                  (9u)
#define BUTTON_GLITCH_ELIMINATION_CYCLES_CALC         (BUTTON_CSD_PRESCAN_SETTLING_TIME * CYDEV_BCLK__SYSCLK__MHZ)
#define BUTTON_CMOD_DISCHARGE_TIME                    (2u)
#define BUTTON_CMOD_DISCHARGE_CYCLES_CALC             (BUTTON_CMOD_DISCHARGE_TIME * CYDEV_BCLK__SYSCLK__MHZ)

#if(BUTTON_GLITCH_ELIMINATION_CYCLES_CALC >= BUTTON_DELAY_EXTRACYCLES_NUM)
    #define BUTTON_GLITCH_ELIMINATION_CYCLES          (BUTTON_GLITCH_ELIMINATION_CYCLES_CALC -\
                                                                BUTTON_DELAY_EXTRACYCLES_NUM)
#else
    #define BUTTON_GLITCH_ELIMINATION_CYCLES          (BUTTON_GLITCH_ELIMINATION_CYCLES_CALC)
#endif /* (BUTTON_GLITCH_ELIMINATION_CYCLES_CALC >= BUTTON_DELAY_EXTRACYCLES_NUM) */

#if(BUTTON_CMOD_DISCHARGE_CYCLES_CALC >= BUTTON_DELAY_EXTRACYCLES_NUM)
    #define BUTTON_CMOD_DISCHARGE_CYCLES              (BUTTON_CMOD_DISCHARGE_CYCLES_CALC -\
                                                                BUTTON_DELAY_EXTRACYCLES_NUM)
#else
    #define BUTTON_CMOD_DISCHARGE_CYCLES              (BUTTON_CMOD_DISCHARGE_CYCLES_CALC)
#endif /* (BUTTON_CMOD_DISCHARGE_CYCLES_CALC >= BUTTON_DELAY_EXTRACYCLES_NUM) */

#define BUTTON_PRECHARGE_IDAC_MOD_VAL                 (0xF0u)
#define BUTTON_CSD_IDAC_PRECHARGE_CONFIG              (BUTTON_IDAC_MOD_MODE_VARIABLE |\
                                                                 BUTTON_IDAC_MOD_RANGE_MASK |\
                                                                 BUTTON_IDAC_FEEDBACK_MODE_MASK |\
                                                                 BUTTON_PRECHARGE_IDAC_MOD_VAL)

#define BUTTON_CNT_RESOLUTION_12_BITS                 (0x0FFF0000Lu)
#define BUTTON_ONE_CYCLE                              (0x00010000Lu)
#define BUTTON_RESOLUTION_16_BITS                     (0xFFFF0000Lu)
#define BUTTON_DISCONNECT_IO_FLAG                     (1u)

#define BUTTON_PERCENTAGE_100                         (0x00000064Lu)


#endif /* End CY_SENSE_BUTTON_SENSING_H */


/* [] END OF FILE */
