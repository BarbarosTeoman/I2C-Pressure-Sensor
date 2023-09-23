/***************************************************************************//**
* \file BUTTON_SensingCSD_LL.h
* \version 7.0
*
* \brief
*   This file provides the headers of APIs specific to CSD sensing implementation.
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

#if !defined(CY_SENSE_BUTTON_SENSINGCSD_LL_H)
#define CY_SENSE_BUTTON_SENSINGCSD_LL_H

#include <CyLib.h>
#include <cyfitter.h>
#include <cytypes.h>
#include <cypins.h>
#include "BUTTON_Structure.h"
#include "BUTTON_Configuration.h"
#include "BUTTON_Sensing.h"

/****************************************************************************
* Register and mode mask definition
****************************************************************************/

#if (BUTTON_ENABLE == BUTTON_CSDV2)
    #define BUTTON_CSD_CSDCMP_INIT                                (BUTTON_CSDCMP_CSDCMP_DISABLED)

    /* SW_HS_P_SEL switches state for Coarse initialization of CMOD (sense path) */
    #if (BUTTON_ENABLE == BUTTON_CSD_EN)
        #if (BUTTON_CSD__CMOD_PAD == BUTTON_CMOD_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_COARSE_CMOD               (BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #elif (BUTTON_CSD__CSHIELD_PAD == BUTTON_CMOD_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_COARSE_CMOD               (BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #else
            #define BUTTON_CSD_HS_P_SEL_COARSE_CMOD               (BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #endif /* (BUTTON_CSD__CMOD_PAD == BUTTON_CMOD_CONNECTION) */
    #else
        #define BUTTON_CSD_HS_P_SEL_COARSE_CMOD                   (0x00000000uL)
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_EN) */

    #if ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) && \
        (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_COARSE_TANK               (BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (BUTTON_CSD__CSHIELD_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_COARSE_TANK               (BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (BUTTON_CSD__CMOD_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_COARSE_TANK               (BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define BUTTON_CSD_HS_P_SEL_COARSE_TANK               (BUTTON_SW_HS_P_SEL_SW_HMMA_STATIC_CLOSE)
        #endif /* (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION) */
    #else
        #define BUTTON_CSD_HS_P_SEL_COARSE_TANK                   (0x00000000uL)
    #endif /* ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) && \
               (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)) */

    #define BUTTON_CSD_SW_HS_P_SEL_COARSE                         (BUTTON_HS_P_SEL_COARSE_CMOD | BUTTON_CSD_HS_P_SEL_COARSE_TANK)

    /* C_mod config */
    #if ((BUTTON_CSD__CMOD_PAD == BUTTON_CMOD_CONNECTION) || (BUTTON_CSD__CMOD_PAD == BUTTON_CTANK_CONNECTION))
        #define BUTTON_CSD_SW_FW_MOD_SEL_INIT             (BUTTON_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE |\
                                                                     BUTTON_SW_FW_MOD_SEL_SW_F1MA_STATIC_CLOSE |\
                                                                     BUTTON_SW_FW_MOD_SEL_SW_F1CA_STATIC_CLOSE)
        #define BUTTON_SW_DSI_SEL_CMODPAD                 (BUTTON_SW_DSI_CMOD)
    #else
        #define BUTTON_CSD_SW_FW_MOD_SEL_INIT             (0x00000000uL)
        #define BUTTON_SW_DSI_SEL_CMODPAD                 (0x00000000uL)
    #endif /* (BUTTON_CSD__CMOD_PAD == BUTTON_CMOD_CONNECTION) */

    /* C_tank config */
    #if ((BUTTON_CSD__CSH_TANK_PAD == BUTTON_CMOD_CONNECTION) || (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION))
        #define BUTTON_CSD_SW_FW_TANK_SEL_INIT            (BUTTON_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                     BUTTON_SW_FW_TANK_SEL_SW_F2MA_STATIC_CLOSE |\
                                                                     BUTTON_SW_FW_TANK_SEL_SW_F2CA_STATIC_CLOSE)
        #define BUTTON_SW_DSI_SEL_TANKPAD                 (BUTTON_SW_DSI_CTANK)
    #else
        #define BUTTON_CSD_SW_FW_TANK_SEL_INIT            (0x00000000uL)
        #define BUTTON_SW_DSI_SEL_TANKPAD                 (0x00000000uL)
    #endif /* (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION) */

    #define BUTTON_CSD_SW_SHIELD_SEL_INIT                 (BUTTON_SW_SHIELD_SEL_SW_HCAV_HSCMP)

    /* Defining default HW configuration according to settings in customizer. */
    #define BUTTON_DEFAULT_CSD_CONFIG                 (BUTTON_CONFIG_FILTER_DELAY_12MHZ |\
                                                                 BUTTON_CONFIG_SAMPLE_SYNC_MASK)
    #if (BUTTON_ENABLE == BUTTON_CSD_AUTO_ZERO_EN)
        /* Enable CSDCMP */
        #define BUTTON_CSD_CSDCMP_SCAN                (BUTTON_CSDCMP_CSDCMP_EN_MASK |\
                                                                 BUTTON_CSDCMP_AZ_EN_MASK)
    #else
        /* Enable CSDCMP */
        #define BUTTON_CSD_CSDCMP_SCAN                (BUTTON_CSDCMP_CSDCMP_EN_MASK)
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_AUTO_ZERO_EN) */

    #if ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) && \
        (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_SCAN_TANK                 (BUTTON_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (BUTTON_CSD__CSHIELD_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_SCAN_TANK                 (BUTTON_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (BUTTON_CSD__CMOD_PAD == BUTTON_CTANK_CONNECTION)
            #define BUTTON_CSD_HS_P_SEL_SCAN_TANK                 (BUTTON_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define BUTTON_CSD_HS_P_SEL_SCAN_TANK                 (BUTTON_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
        #endif /* (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION) */
        #define BUTTON_CSD_SW_HS_P_SEL_SCAN                       (BUTTON_HS_P_SEL_SCAN_TANK)
    #elif(BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)
        #define BUTTON_CSD_SW_HS_P_SEL_SCAN                       (BUTTON_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
    #else
        #define BUTTON_CSD_SW_HS_P_SEL_SCAN                       (BUTTON_STATIC_OPEN)
    #endif /* ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) && \
               (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)) */

    /* SW_FW_MOD_SEL switches state for Coarse initialization of CMOD (sense path) */
    #define BUTTON_CSD_SW_FW_MOD_SEL_SCAN                     (0x00000000uL)

    #if((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
        (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) && \
        (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION))
        #define BUTTON_CSD_SW_FW_TANK_SEL_SCAN                (BUTTON_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE | \
                                                                         BUTTON_SW_FW_TANK_SEL_SW_F2CB_STATIC_CLOSE)
    #else
        #define BUTTON_CSD_SW_FW_TANK_SEL_SCAN                (0x00000000uL)
    #endif /* ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
               (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) && \
               (BUTTON_CSD__CSH_TANK_PAD == BUTTON_CTANK_CONNECTION)) */

    /* Shield switch default config */
    #if ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
         (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN))
        #if (BUTTON_IDAC_SINKING == BUTTON_CSD_IDAC_CONFIG)
            #define BUTTON_CSD_SW_SHIELD_SEL_SCAN             (BUTTON_SW_SHIELD_SEL_SW_HCBG_HSCMP)
        #else
            #define BUTTON_CSD_SW_SHIELD_SEL_SCAN             (BUTTON_SW_SHIELD_SEL_SW_HCBV_HSCMP)
        #endif /* (BUTTON_IDAC_SINKING == BUTTON_CSD_IDAC_CONFIG) */
    #elif(BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)
        #if (BUTTON_IDAC_SINKING == BUTTON_CSD_IDAC_CONFIG)
            #define BUTTON_CSD_SW_SHIELD_SEL_SCAN             (BUTTON_SW_SHIELD_SEL_SW_HCBV_PHI1 | \
                                                                         BUTTON_SW_SHIELD_SEL_SW_HCBG_PHI2_HSCMP)
        #else
            #define BUTTON_CSD_SW_SHIELD_SEL_SCAN             (BUTTON_SW_SHIELD_SEL_SW_HCBG_PHI1 | \
                                                                         BUTTON_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP)
        #endif /* (BUTTON_IDAC_SINKING == BUTTON_CSD_IDAC_CONFIG) */
    #else
        #define BUTTON_CSD_SW_SHIELD_SEL_SCAN                 (0x00000000uL)
    #endif /* ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
               (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN)) */

    #define BUTTON_CSD_SW_RES_INIT                            (BUTTON_CSD_INIT_SWITCH_RES << CYFLD_CSD_RES_HCAV__OFFSET)
    #define BUTTON_CSD_SW_RES_SCAN                            ((BUTTON_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBV__OFFSET) |\
                                                                         (BUTTON_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBG__OFFSET))

    #define BUTTON_CSD_SHIELD_GPIO_DM                         (BUTTON_GPIO_STRGDRV)
    #define BUTTON_CSD_SENSOR_HSIOM_SEL                       (BUTTON_HSIOM_SEL_CSD_SENSE)
    #define BUTTON_CSD_SHIELD_HSIOM_SEL                       (BUTTON_HSIOM_SEL_CSD_SHIELD)
    #define BUTTON_CSD_CMOD_HSIOM_SEL                         (BUTTON_HSIOM_SEL_AMUXA)

    #define BUTTON_DEFAULT_IDAC_MOD_BALL_MODE                 ((uint32)BUTTON_IDAC_MOD_BALL_MODE_FULL <<\
                                                                         CYFLD_CSD_BAL_MODE__OFFSET)
    #define BUTTON_DEFAULT_IDAC_COMP_BALL_MODE                ((uint32)BUTTON_IDAC_COMP_BALL_MODE_FULL <<\
                                                                         CYFLD_CSD_BAL_MODE__OFFSET)

    #define BUTTON_DEFAULT_SENSE_DUTY_SEL                     (BUTTON_SENSE_DUTY_OVERLAP_PHI1_MASK |\
                                                                         BUTTON_SENSE_DUTY_OVERLAP_PHI2_MASK)

    #define BUTTON_CSD_CAL_MIDDLE_VALUE                       (0x40u)
    #define BUTTON_CSD_CAL_IDAC_MAX_VALUE                     (127u)

    #define BUTTON_DELAY_EXTRACYCLES_NUM                      (9u)

    /* Clock Source Mode */
    #if (BUTTON_CLK_SOURCE_DIRECT == BUTTON_CSD_SNS_CLK_SOURCE)
        #define BUTTON_DEFAULT_MODULATION_MODE                (BUTTON_CLK_SOURCE_DIRECT)
    #elif (BUTTON_CLK_SOURCE_PRSAUTO == BUTTON_CSD_SNS_CLK_SOURCE)
        #define BUTTON_DEFAULT_MODULATION_MODE                (BUTTON_CLK_SOURCE_SSC2)
    #elif ((BUTTON_CLK_SOURCE_PRS8) == BUTTON_CSD_SNS_CLK_SOURCE)
        #define BUTTON_DEFAULT_MODULATION_MODE                (BUTTON_CSD_SNS_CLK_SOURCE)
    #elif ((BUTTON_CLK_SOURCE_PRS12) == BUTTON_CSD_SNS_CLK_SOURCE)
        #define BUTTON_DEFAULT_MODULATION_MODE                (BUTTON_CSD_SNS_CLK_SOURCE)
    #else
        #define BUTTON_DEFAULT_MODULATION_MODE                (BUTTON_CSD_SNS_CLK_SOURCE)
    #endif /* (BUTTON_CLK_SOURCE_DIRECT != BUTTON_CSD_SNS_CLK_SOURCE) */

    /* IDACs Ranges */
    #if (BUTTON_IDAC_GAIN_LOW == BUTTON_CSD_IDAC_GAIN)
        #define BUTTON_DEFAULT_IDAC_MOD_RANGE                 ((uint32)BUTTON_IDAC_MOD_RANGE_IDAC_LO << CYFLD_CSD_RANGE__OFFSET)
        #define BUTTON_DEFAULT_IDAC_COMP_RANGE                ((uint32)BUTTON_IDAC_COMP_RANGE_IDAC_LO << CYFLD_CSD_RANGE__OFFSET)
    #elif (BUTTON_IDAC_GAIN_MEDIUM == BUTTON_CSD_IDAC_GAIN)
        #define BUTTON_DEFAULT_IDAC_MOD_RANGE                 ((uint32)BUTTON_IDAC_MOD_RANGE_IDAC_MED << CYFLD_CSD_RANGE__OFFSET)
        #define BUTTON_DEFAULT_IDAC_COMP_RANGE                ((uint32)BUTTON_IDAC_COMP_RANGE_IDAC_MED << CYFLD_CSD_RANGE__OFFSET)
    #else
        #define BUTTON_DEFAULT_IDAC_MOD_RANGE                 ((uint32)BUTTON_IDAC_MOD_RANGE_IDAC_HI << CYFLD_CSD_RANGE__OFFSET)
        #define BUTTON_DEFAULT_IDAC_COMP_RANGE                ((uint32)BUTTON_IDAC_COMP_RANGE_IDAC_HI << CYFLD_CSD_RANGE__OFFSET)
    #endif

    /* IDACs Polarities */
    #if (BUTTON_IDAC_SINKING == BUTTON_CSD_IDAC_CONFIG)
        #define BUTTON_DEFAULT_IDAC_MOD_POLARITY              ((uint32)BUTTON_IDAC_MOD_POLARITY_VDDA_SNK << CYFLD_CSD_POLARITY__OFFSET)
        #define BUTTON_DEFAULT_IDAC_COMP_POLARITY             ((uint32)BUTTON_IDAC_COMP_POLARITY_VDDA_SNK << CYFLD_CSD_POLARITY__OFFSET)
    #else
        #define BUTTON_DEFAULT_IDAC_MOD_POLARITY              ((uint32)BUTTON_IDAC_MOD_POLARITY_VSSA_SRC << CYFLD_CSD_POLARITY__OFFSET)
        #define BUTTON_DEFAULT_IDAC_COMP_POLARITY             ((uint32)BUTTON_IDAC_COMP_POLARITY_VSSA_SRC << CYFLD_CSD_POLARITY__OFFSET)
    #endif /* (BUTTON_IDAC_SINKING == BUTTON_CSD_IDAC_CONFIG) */

    #define BUTTON_SW_REFGEN_VREF_SRC                         (BUTTON_SW_REFGEN_SEL_SW_SGR_MASK)

    /* IDAC legs configuration */
    #if (BUTTON_ENABLE == BUTTON_CSD_IDAC_COMP_EN)
            #define BUTTON_DEFAULT_SW_REFGEN_SEL              (BUTTON_SW_REFGEN_VREF_SRC | BUTTON_SW_REFGEN_SEL_SW_IAIB_MASK)
    #else
            #define BUTTON_DEFAULT_SW_REFGEN_SEL              (BUTTON_SW_REFGEN_VREF_SRC)
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_IDAC_COMP_EN) */

    /* IDACs register configuration is based on the Component configuration */
    #define BUTTON_IDAC_MOD_DEFAULT_CFG                       (BUTTON_DEFAULT_IDAC_MOD_RANGE | \
                                                                         BUTTON_DEFAULT_IDAC_MOD_POLARITY | \
                                                                         BUTTON_DEFAULT_IDAC_MOD_BALL_MODE | \
                                                                        ((uint32)(BUTTON_IDAC_MOD_LEG1_MODE_CSD << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                        ((uint32)(BUTTON_IDAC_MOD_LEG2_MODE_CSD << CYFLD_CSD_LEG2_MODE__OFFSET)) | \
                                                                         BUTTON_IDAC_MOD_LEG1_EN_MASK)

    #define BUTTON_IDAC_COMP_DEFAULT_CFG                      (BUTTON_DEFAULT_IDAC_COMP_RANGE | \
                                                                         BUTTON_DEFAULT_IDAC_COMP_POLARITY | \
                                                                         BUTTON_DEFAULT_IDAC_COMP_BALL_MODE | \
                                                                        ((uint32)(BUTTON_IDAC_COMP_LEG1_MODE_CSD_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                        ((uint32)(BUTTON_IDAC_COMP_LEG2_MODE_CSD_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET)) | \
                                                                         BUTTON_IDAC_COMP_LEG1_EN_MASK)

    #define BUTTON_IDAC_MOD_CALIBRATION_CFG                   ((uint32)(BUTTON_DEFAULT_IDAC_MOD_RANGE | \
                                                                         BUTTON_DEFAULT_IDAC_MOD_POLARITY | \
                                                                         BUTTON_DEFAULT_IDAC_MOD_BALL_MODE | \
                                                                        ((uint32)(BUTTON_IDAC_MOD_LEG1_MODE_CSD << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                         BUTTON_IDAC_MOD_LEG1_EN_MASK | \
                                                                        ((uint32)((uint32)BUTTON_IDAC_MOD_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET))))

    #define BUTTON_IDAC_COMP_CALIBRATION_CFG                  ((uint32)(BUTTON_DEFAULT_IDAC_COMP_RANGE | \
                                                                         BUTTON_DEFAULT_IDAC_COMP_POLARITY | \
                                                                         BUTTON_DEFAULT_IDAC_COMP_BALL_MODE | \
                                                                        ((uint32)((uint32)BUTTON_IDAC_COMP_LEG1_MODE_GP_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                        ((uint32)((uint32)BUTTON_IDAC_COMP_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET))))
#else
    #if (BUTTON_ENABLE == BUTTON_CSD_IDAC_COMP_EN)
        #define BUTTON_CSD_CAL_IDAC_MAX_VALUE                 (127u)
        #define BUTTON_CSD_CAL_MIDDLE_VALUE                   (0x40u)
    #else
        #define BUTTON_CSD_CAL_IDAC_MAX_VALUE                 (255u)
        #define BUTTON_CSD_CAL_MIDDLE_VALUE                   (0x80u)
    #endif

    #define BUTTON_IDAC_MOD_CFG_MASK                  (BUTTON_IDAC_POLARITY1_MIR_MASK |\
                                                                BUTTON_IDAC_MOD_RANGE_MASK |\
                                                                BUTTON_IDAC_MOD_MODE_MASK |\
                                                                BUTTON_IDAC_MOD_MASK)

    #define BUTTON_IDAC_COMP_CFG_MASK                 (BUTTON_IDAC_POLARITY2_MIR_MASK |\
                                                                BUTTON_IDAC_COMP_RANGE_MASK |\
                                                                BUTTON_IDAC_COMP_MODE_MASK |\
                                                                BUTTON_IDAC_COMP_MASK)

        #define BUTTON_PRS_8_CONFIG                       BUTTON_CONFIG_PRS_SELECT_MASK
    #define BUTTON_PRS_12_CONFIG                      (BUTTON_CONFIG_PRS_12_8_MASK |\
                                                                BUTTON_CONFIG_PRS_SELECT_MASK)

    /* Third-generation HW block Initial PRS mode */
    #if (BUTTON_CLK_SOURCE_PRS8 == BUTTON_CSD_SNS_CLK_SOURCE)
        #define BUTTON_DEFAULT_MODULATION_MODE        BUTTON_CONFIG_PRS_SELECT_MASK

    #elif (BUTTON_CLK_SOURCE_PRS12 == BUTTON_CSD_SNS_CLK_SOURCE)
        #define BUTTON_DEFAULT_MODULATION_MODE        (BUTTON_CONFIG_PRS_12_8_MASK |\
                                                                BUTTON_CONFIG_PRS_SELECT_MASK)
    #else
        #define BUTTON_DEFAULT_MODULATION_MODE        (0u)
    #endif /* (BUTTON_CSD_SNS_CLK_SOURCE == BUTTON_PRS_8BITS) */

    /* Defining default CSD configuration according to settings in customizer. */
    #define BUTTON_DEFAULT_CSD_CONFIG                 (BUTTON_CONFIG_SENSE_COMP_BW_MASK |\
                                                                BUTTON_DEFAULT_IDAC_POLARITY |\
                                                                BUTTON_CONFIG_SENSE_INSEL_MASK |\
                                                                BUTTON_CONFIG_REFBUF_DRV_MASK)

    /* Third-generation HW block Defining mask intended for clearing bits related to pre-charging options. */
    #define BUTTON_PRECHARGE_CONFIG_MASK              (BUTTON_CONFIG_REFBUF_EN_MASK |\
                                                                BUTTON_CONFIG_COMP_MODE_MASK |\
                                                                BUTTON_CONFIG_COMP_PIN_MASK  |\
                                                                BUTTON_CONFIG_REFBUF_OUTSEL_MASK)

    #define BUTTON_CMOD_PRECHARGE_CONFIG              (BUTTON_DEFAULT_CSD_CONFIG |\
                                                                BUTTON_CONFIG_REFBUF_EN_MASK |\
                                                                BUTTON_CONFIG_COMP_PIN_MASK)

    #define BUTTON_CMOD_PRECHARGE_CONFIG_CSD_EN       (BUTTON_DEFAULT_CSD_CONFIG |\
                                                                BUTTON_CSD_ENABLE_MASK |\
                                                                BUTTON_CONFIG_REFBUF_EN_MASK |\
                                                                BUTTON_CONFIG_COMP_PIN_MASK)


    #if (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)
        #if((BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN) &&\
            (BUTTON_CSH_PRECHARGE_IO_BUF == BUTTON_CSD_CSH_PRECHARGE_SRC))
            #define  BUTTON_CTANK_PRECHARGE_CONFIG    (BUTTON_DEFAULT_CSD_CONFIG |\
                                                                 BUTTON_CONFIG_REFBUF_OUTSEL_MASK |\
                                                                 BUTTON_CONFIG_REFBUF_EN_MASK |\
                                                                 BUTTON_CONFIG_COMP_MODE_MASK |\
                                                                 BUTTON_CONFIG_PRS_CLEAR_MASK |\
                                                                 BUTTON_CONFIG_COMP_PIN_MASK)
        #else
            #define  BUTTON_CTANK_PRECHARGE_CONFIG    (BUTTON_DEFAULT_CSD_CONFIG |\
                                                                 BUTTON_CONFIG_REFBUF_EN_MASK |\
                                                                 BUTTON_CONFIG_PRS_CLEAR_MASK |\
                                                                 BUTTON_CONFIG_REFBUF_OUTSEL_MASK)
        #endif
    #else
        #define  BUTTON_CTANK_PRECHARGE_CONFIG    (BUTTON_DEFAULT_CSD_CONFIG |\
                                                             BUTTON_CONFIG_REFBUF_OUTSEL_MASK |\
                                                             BUTTON_CONFIG_PRS_CLEAR_MASK)
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) */


    #define  BUTTON_CTANK_PRECHARGE_CONFIG_CSD_EN     (BUTTON_CTANK_PRECHARGE_CONFIG |\
                                                                 BUTTON_CONFIG_ENABLE_MASK |\
                                                                 BUTTON_CONFIG_SENSE_COMP_EN_MASK)

#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */

#define BUTTON_CSD_AVG_CYCLES_PER_LOOP                   (5u)
#define BUTTON_CSD_MEASURE_MAX_TIME_US                   (200000u)
#define BUTTON_CSD_PRECHARGE_MAX_TIME_US                 (50u)
#define BUTTON_CSD_ONE_CLOCK_MAX_TIME_US                 (50u)
#define BUTTON_WIDGET_MAX_SCAN_TIME_US                   (5u * 1000000u)

#define BUTTON_CSD_CALIBR_WATCHDOG_CYCLES_NUM            (((CYDEV_BCLK__SYSCLK__MHZ) * (BUTTON_CSD_MEASURE_MAX_TIME_US)) /\
                                                                    (BUTTON_CSD_AVG_CYCLES_PER_LOOP))
#define BUTTON_CSD_PRECHARGE_WATCHDOG_CYCLES_NUM         (((CYDEV_BCLK__SYSCLK__MHZ) * (BUTTON_CSD_PRECHARGE_MAX_TIME_US)) /\
                                                                    (BUTTON_CSD_AVG_CYCLES_PER_LOOP))
#define BUTTON_ONE_CLOCK_WATCHDOG_CYCLES_NUM             (((CYDEV_BCLK__SYSCLK__MHZ) * (BUTTON_CSD_ONE_CLOCK_MAX_TIME_US)) /\
                                                                    (BUTTON_CSD_AVG_CYCLES_PER_LOOP))
#define BUTTON_WIDGET_MAX_SCAN_TIME                      (((CYDEV_BCLK__SYSCLK__MHZ) * (BUTTON_WIDGET_MAX_SCAN_TIME_US)) /\
                                                                    (BUTTON_CSD_AVG_CYCLES_PER_LOOP))

/***************************************
* Global software variables
***************************************/

extern uint32 BUTTON_configCsd;

#if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
    extern uint8 BUTTON_badConversionsNum;
#endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */


/***************************************
* Function Prototypes
**************************************/

/**
* \cond SECTION_C_LOW_LEVEL
* \addtogroup group_c_low_level
* \{
*/

void BUTTON_CSDSetupWidget(uint32 widgetId);
void BUTTON_CSDSetupWidgetExt(uint32 widgetId, uint32 sensorId);
void BUTTON_CSDScan(void);
void BUTTON_CSDScanExt(void);
#if ((BUTTON_CSD_SS_DIS != BUTTON_CSD_AUTOTUNE) || \
     (BUTTON_ENABLE == BUTTON_CSD_IDAC_AUTOCAL_EN))
    cystatus BUTTON_CSDCalibrateWidget(uint32 widgetId, uint32 target);
#endif /* ((BUTTON_CSD_SS_DIS != BUTTON_CSD_AUTOTUNE) || \
           (BUTTON_ENABLE == BUTTON_CSD_IDAC_AUTOCAL_EN))  */
void BUTTON_CSDConnectSns(BUTTON_FLASH_IO_STRUCT const *snsAddrPtr);
void BUTTON_CSDDisconnectSns(BUTTON_FLASH_IO_STRUCT const *snsAddrPtr);

/** \}
* \endcond */

/*****************************************************
* Function Prototypes - internal Low Level functions
*****************************************************/

/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

void BUTTON_SsCSDInitialize(void);
void BUTTON_SsCSDStartSample(void);
void BUTTON_SsCSDSetUpIdacs(BUTTON_RAM_WD_BASE_STRUCT const *ptrWdgt);
void BUTTON_SsCSDConfigClock(void);
void BUTTON_SsCSDElectrodeCheck(void);
#if ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
     (0u != BUTTON_CSD_TOTAL_SHIELD_COUNT))
    void BUTTON_SsCSDEnableShieldElectrodes(void);
    void BUTTON_SsCSDDisableShieldElectrodes(void);
    void BUTTON_SsCSDSetShieldElectrodesState(uint32 state);
#endif /* ((BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) && \
           (0u != BUTTON_CSD_TOTAL_SHIELD_COUNT)) */
#if (BUTTON_ENABLE == BUTTON_CSDV2)
    uint32 BUTTON_SsCSDGetNumberOfConversions(uint32 snsClkDivider, uint32 resolution, uint32 snsClkSrc);
#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */
void BUTTON_SsCSDCalculateScanDuration(BUTTON_RAM_WD_BASE_STRUCT const *ptrWdgt);
void BUTTON_SsCSDConnectSensorExt(uint32 widgetId, uint32 sensorId);
void BUTTON_SsCSDDisconnectSnsExt(uint32 widgetId, uint32 sensorId);

#if ((BUTTON_CSD_SS_DIS != BUTTON_CSD_AUTOTUNE) || \
     (BUTTON_ENABLE == BUTTON_SELF_TEST_EN) || \
     (BUTTON_ENABLE == BUTTON_CSD_IDAC_AUTOCAL_EN))
#endif /* ((BUTTON_CSD_SS_DIS != BUTTON_CSD_AUTOTUNE) || \
           (BUTTON_ENABLE == BUTTON_SELF_TEST_EN) || \
           (BUTTON_ENABLE == BUTTON_CSD_IDAC_AUTOCAL_EN)) */

/** \}
* \endcond */

/***************************************
* Global software variables
***************************************/
extern uint32 BUTTON_configCsd;

/* Interrupt handler */
extern CY_ISR_PROTO(BUTTON_CSDPostSingleScan);
extern CY_ISR_PROTO(BUTTON_CSDPostMultiScan);
#if (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN)
extern CY_ISR_PROTO(BUTTON_CSDPostMultiScanGanged);
#endif /* (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN) */
#if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
    extern uint8 BUTTON_badConversionsNum;
#endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

#endif /* End CY_SENSE_BUTTON_SENSINGCSD_LL_H */


/* [] END OF FILE */
