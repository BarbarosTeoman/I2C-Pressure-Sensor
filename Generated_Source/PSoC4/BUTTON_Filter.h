/***************************************************************************//**
* \file BUTTON_Filter.h
* \version 7.0
*
* \brief
*   This file contains the definitions for all firmware filters
*   implementation.
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

#if !defined(CY_SENSE_BUTTON_FILTER_H)
#define CY_SENSE_BUTTON_FILTER_H

#include "cytypes.h"
#include "CyLib.h"
#include "cyfitter.h"

#include "BUTTON_Structure.h"
#include "BUTTON_Configuration.h"

/***************************************
* Function Prototypes
***************************************/

/*******************************************************************************
* LOW LEVEL API
*******************************************************************************/
/**
* \cond (SECTION_C_LOW_LEVEL || SECTION_I_LOW_LEVEL)
* \addtogroup group_c_low_level
* \{
*/

cystatus BUTTON_UpdateAllBaselines(void);
cystatus BUTTON_UpdateWidgetBaseline(uint32 widgetId);
cystatus BUTTON_UpdateSensorBaseline(uint32 widgetId, uint32 sensorId);

void BUTTON_InitializeAllBaselines(void);
void BUTTON_InitializeWidgetBaseline(uint32 widgetId);
void BUTTON_InitializeSensorBaseline(uint32 widgetId, uint32 sensorId);

#if ((BUTTON_ENABLE == BUTTON_RC_FILTER_EN) || \
     (0u != (BUTTON_CSD_AUTOTUNE & BUTTON_CSD_SS_TH_EN)))
    void BUTTON_InitializeAllFilters(void);
    void BUTTON_InitializeWidgetFilter(uint32 widgetId);
#endif /* ((BUTTON_ENABLE == BUTTON_RC_FILTER_EN) || \
           (0u != (BUTTON_CSD_AUTOTUNE & BUTTON_CSD_SS_TH_EN))) */

/** \}
* \endcond */


/*******************************************************************************
* Function Prototypes - internal functions
*******************************************************************************/
/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

void BUTTON_FtInitialize(void);

#if (0u != (BUTTON_CSD_AUTOTUNE & BUTTON_CSD_SS_TH_EN))
    void BUTTON_RunNoiseEnvelope(uint32 widgetId, uint32 sensorId);
    void BUTTON_InitializeNoiseEnvelope(uint32 widgetId, uint32 sensorId);
#endif /* (0u != (BUTTON_CSD_AUTOTUNE & BUTTON_CSD_SS_TH_EN)) */

#if (BUTTON_REGULAR_RC_IIR_FILTER_EN || BUTTON_PROX_RC_IIR_FILTER_EN)
    void BUTTON_InitializeIIR(uint32 widgetId, uint32 sensorId);
    void BUTTON_RunIIR(uint32 widgetId, uint32 sensorId);
#endif /* (BUTTON_REGULAR_RC_IIR_FILTER_EN || BUTTON_PROX_RC_IIR_FILTER_EN) */

#if (BUTTON_REGULAR_RC_MEDIAN_FILTER_EN || BUTTON_PROX_RC_MEDIAN_FILTER_EN)
    void BUTTON_InitializeMedian(uint32 widgetId, uint32 sensorId);
    void BUTTON_RunMedian(uint32 widgetId, uint32 sensorId);
#endif /* (BUTTON_REGULAR_RC_MEDIAN_FILTER_EN || BUTTON_PROX_RC_MEDIAN_FILTER_EN) */

#if (BUTTON_REGULAR_RC_AVERAGE_FILTER_EN || BUTTON_PROX_RC_AVERAGE_FILTER_EN)
    void BUTTON_InitializeAverage(uint32 widgetId, uint32 sensorId);
    void BUTTON_RunAverage(uint32 widgetId, uint32 sensorId);
#endif /* (BUTTON_REGULAR_RC_AVERAGE_FILTER_EN || BUTTON_PROX_RC_AVERAGE_FILTER_EN) */

void BUTTON_FtInitializeBaseline(BUTTON_RAM_SNS_STRUCT *ptrSensor, uint32 wdType);
uint32 BUTTON_FtUpdateBaseline(
                            BUTTON_RAM_WD_BASE_STRUCT *ptrWidgetRam,
                            BUTTON_RAM_SNS_STRUCT *ptrSensor,
                            uint32 wdType);

#if (BUTTON_POS_MEDIAN_FILTER_EN || BUTTON_REGULAR_RC_MEDIAN_FILTER_EN || BUTTON_PROX_RC_MEDIAN_FILTER_EN)
uint32 BUTTON_FtMedian(uint32 x1, uint32 x2, uint32 x3);
#endif /*BUTTON_POS_MEDIAN_FILTER_EN || BUTTON_REGULAR_RC_MEDIAN_FILTER_EN || BUTTON_PROX_RC_MEDIAN_FILTER_EN*/

uint32 BUTTON_FtIIR1stOrder(uint32 input, uint32 prevOutput, uint32 n, uint32 shift);

#if (BUTTON_POS_JITTER_FILTER_EN)
    uint32 BUTTON_FtJitter(uint32 input, uint32 prevOutput);
#endif /* BUTTON_POS_JITTER_FILTER_EN */

void BUTTON_FtInitializeBaselineChannel(BUTTON_RAM_SNS_STRUCT *ptrSensor, uint32 wdType, uint32 channel);

#if (BUTTON_ENABLE == BUTTON_RC_FILTER_EN)
    void BUTTON_FtRunEnabledFilters(uint32 widgetId, uint32 sensorId);
    void BUTTON_FtRunEnabledFiltersInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                      BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (BUTTON_ENABLE == BUTTON_RC_FILTER_EN) */


#if (BUTTON_REGULAR_RC_IIR_FILTER_EN || BUTTON_PROX_RC_IIR_FILTER_EN)
    void BUTTON_InitializeIIRInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                  BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
    void BUTTON_RunIIRInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                           BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (BUTTON_REGULAR_RC_IIR_FILTER_EN || BUTTON_PROX_RC_IIR_FILTER_EN) */

#if (BUTTON_REGULAR_RC_MEDIAN_FILTER_EN || BUTTON_PROX_RC_MEDIAN_FILTER_EN)
    void BUTTON_InitializeMedianInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                     BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
    void BUTTON_RunMedianInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                              BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (BUTTON_REGULAR_RC_MEDIAN_FILTER_EN || BUTTON_PROX_RC_MEDIAN_FILTER_EN) */

#if (BUTTON_REGULAR_RC_AVERAGE_FILTER_EN || BUTTON_PROX_RC_AVERAGE_FILTER_EN)
    void BUTTON_InitializeAverageInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                      BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
    void BUTTON_RunAverageInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                               BUTTON_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (BUTTON_REGULAR_RC_AVERAGE_FILTER_EN || BUTTON_PROX_RC_AVERAGE_FILTER_EN) */

#if (BUTTON_ENABLE == BUTTON_ALP_FILTER_EN)
    void BUTTON_InitializeALP(uint32 widgetId, uint32 sensorId);
    void BUTTON_RunALP(uint32 widgetId, uint32 sensorId);
    void BUTTON_InitializeALPInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                BUTTON_RAM_SNS_STRUCT *ptrSensorObj,
                                                uint32 wdType);
    void BUTTON_ConfigRunALPInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                        BUTTON_RAM_WD_BASE_STRUCT *ptrRamWdgt,
                                        BUTTON_RAM_SNS_STRUCT *ptrSensorObj,
                                        uint32 wdType);
    void BUTTON_RunALPInternal(BUTTON_PTR_FILTER_VARIANT ptrFilterHistObj,
                                        ALP_FLTR_CONFIG_STRUCT *ptrAlpFilterConfig,
                                        BUTTON_RAM_SNS_STRUCT *ptrSensorObj,
                                        uint32 wdType);
#endif /* (BUTTON_ENABLE == BUTTON_ALP_FILTER_EN) */

/** \}
* \endcond */

/***************************************
* Initial Parameter Constants
***************************************/
#define NOISE_ENVELOPE_SHIFT                        (0x02u)
#define NOISE_ENVELOPE_RUN_DIFF_SHIFT               (0x03u)
#define NOISE_ENVELOPE_SIGN_REG                     (0x0Fu)
#define NOISE_ENVELOPE_SIGN_REG_SHIFT               (0x01u)
#define NOISE_ENVELOPE_RESET_COUNTER                (0x0Au)
#define NOISE_ENVELOPE_4_TIMES                      (0x02u)

#endif /* End CY_SENSE_BUTTON_FILTER_H */


/* [] END OF FILE */
