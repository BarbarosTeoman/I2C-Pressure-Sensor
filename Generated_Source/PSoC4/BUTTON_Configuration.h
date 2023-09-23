/*******************************************************************************
* \file BUTTON_Configuration.h
* \version 7.0
*
* \brief
*   This file provides the customizer parameters definitions.
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

#if !defined(CY_SENSE_BUTTON_CONFIGURATION_H)
#define CY_SENSE_BUTTON_CONFIGURATION_H

#include <cytypes.h>
#include <cyfitter.h>

/*******************************************************************************
* Customizer-generated defines
*******************************************************************************/
#define BUTTON_ENABLE                             (1u)
#define BUTTON_DISABLE                            (0u)

#define BUTTON_THIRD_GENERATION_BLOCK             (1u)
#define BUTTON_FOURTH_GENERATION_BLOCK            (2u)

#define BUTTON_GENERATION_BLOCK_VERSION           (2u)

/*******************************************************************************
* HW IP block global defines
*******************************************************************************/

#if (BUTTON_GENERATION_BLOCK_VERSION == BUTTON_THIRD_GENERATION_BLOCK)
    #define BUTTON_CSDV1                          (BUTTON_ENABLE)

    #ifdef CYIPBLOCK_m0s8csd_VERSION
        #if (0u == CYIPBLOCK_m0s8csd_VERSION)
            #define BUTTON_CSDV1_VER1             (BUTTON_ENABLE)
            #define BUTTON_CSDV1_VER2             (BUTTON_DISABLE)
        #else
            #define BUTTON_CSDV1_VER1             (BUTTON_DISABLE)
            #define BUTTON_CSDV1_VER2             (BUTTON_ENABLE)
        #endif
    #else
        #error "HW IP block version is not specified"
    #endif
#else
    #define BUTTON_CSDV1                          (BUTTON_DISABLE)
    #define BUTTON_CSDV1_VER1                     (BUTTON_DISABLE)
    #define BUTTON_CSDV1_VER2                     (BUTTON_DISABLE)
#endif

#if (BUTTON_GENERATION_BLOCK_VERSION == BUTTON_FOURTH_GENERATION_BLOCK)
    #define BUTTON_CSDV2                          (BUTTON_ENABLE)

    #ifdef CYIPBLOCK_m0s8csdv2_VERSION
        #if (1u == CYIPBLOCK_m0s8csdv2_VERSION)
            #define BUTTON_CSDV2_VER1             (BUTTON_ENABLE)
        #else
            #define BUTTON_CSDV2_VER1             (BUTTON_DISABLE)
        #endif
        #if (2u == CYIPBLOCK_m0s8csdv2_VERSION)
            #define BUTTON_CSDV2_VER2             (BUTTON_ENABLE)
        #else
            #define BUTTON_CSDV2_VER2             (BUTTON_DISABLE)
        #endif
    #else
        #error "HW IP block version is not specified"
    #endif
#else
    #define BUTTON_CSDV2                          (BUTTON_DISABLE)
    #define BUTTON_CSDV2_VER1                     (BUTTON_DISABLE)
    #define BUTTON_CSDV2_VER2                     (BUTTON_DISABLE)
#endif

/*******************************************************************************
* Project-global defines
*******************************************************************************/

#define BUTTON_2000_MV                            (2000u)

#ifdef CYDEV_VDDA_MV
    #define BUTTON_CYDEV_VDDA_MV                  (CYDEV_VDDA_MV)
#else
    #ifdef CYDEV_VDD_MV
        #define BUTTON_CYDEV_VDDA_MV              (CYDEV_VDD_MV)
    #endif
#endif

#define BUTTON_BAD_CONVERSIONS_NUM                (1u)
#define BUTTON_RESAMPLING_CYCLES_MAX_NUMBER       (1u)


/*******************************************************************************
* Enabled Scan Methods
*******************************************************************************/
#define BUTTON_CSD_EN                             (1u)
#define BUTTON_CSX_EN                             (0u)
#define BUTTON_ISX_EN                             (0u)
#define BUTTON_CSD_CSX_EN                         (BUTTON_CSD_EN && BUTTON_CSX_EN)

#define BUTTON_MANY_SENSE_MODES_EN                ((BUTTON_CSD_EN && BUTTON_CSX_EN) || \
                                                             (BUTTON_CSD_EN && BUTTON_ISX_EN) || \
                                                             (BUTTON_CSX_EN && BUTTON_ISX_EN) || \
                                                             (BUTTON_SELF_TEST_EN))

#define BUTTON_MANY_WIDGET_METHODS_EN             ((BUTTON_CSD_EN && BUTTON_CSX_EN) || \
                                                             (BUTTON_CSD_EN && BUTTON_ISX_EN) || \
                                                             (BUTTON_CSX_EN && BUTTON_ISX_EN))

#define BUTTON_CSD2X_EN                           (0u)
#define BUTTON_CSX2X_EN                           (0u)

/*******************************************************************************
* Definitions for number of widgets and sensors
*******************************************************************************/
#define BUTTON_TOTAL_WIDGETS                      (1u)
#define BUTTON_TOTAL_CSD_WIDGETS                  (1u)
#define BUTTON_TOTAL_CSD_SENSORS                  (1u)
#define BUTTON_TOTAL_CSX_WIDGETS                  (0u)
#define BUTTON_TOTAL_ISX_WIDGETS                  (0u)
#define BUTTON_TOTAL_CSX_NODES                    (0u)
#define BUTTON_TOTAL_ISX_NODES                    (0u)

/*******************************************************************************
* Total number of CSD sensors + CSX nodes
*******************************************************************************/
#define BUTTON_TOTAL_SENSORS            (BUTTON_TOTAL_CSD_SENSORS + \
                                                   BUTTON_TOTAL_CSX_NODES+ \
                                                   BUTTON_TOTAL_ISX_NODES)

/*******************************************************************************
* Total number of scan slots (used only when dual-channel scan is enabled)
*******************************************************************************/
#define BUTTON_TOTAL_SCAN_SLOTS         (1u)

/*******************************************************************************
* Defines widget IDs
*******************************************************************************/
#define BUTTON_BUTTON0_WDGT_ID                  (0u)

/*******************************************************************************
* Defines sensor IDs
*******************************************************************************/

/* Button0 sensor names */
#define BUTTON_BUTTON0_SNS0_ID                  (0u)



/*******************************************************************************
* Enabled widget types
*******************************************************************************/
#define BUTTON_BUTTON_WIDGET_EN         (1u)
#define BUTTON_SLIDER_WIDGET_EN         (0u)
#define BUTTON_MATRIX_WIDGET_EN         (0u)
#define BUTTON_PROXIMITY_WIDGET_EN      (0u)
#define BUTTON_TOUCHPAD_WIDGET_EN       (0u)
#define BUTTON_ENCODERDIAL_WIDGET_EN    (0u)

#define BUTTON_CSD_MATRIX_WIDGET_EN     (0u)
#define BUTTON_CSD_TOUCHPAD_WIDGET_EN   (0u)

#define BUTTON_CSX_MATRIX_WIDGET_EN     (0u)
#define BUTTON_CSX_TOUCHPAD_WIDGET_EN   (0u)

/*******************************************************************************
* Centroid APIs
*******************************************************************************/
#define BUTTON_CENTROID_EN              (0u)
#define BUTTON_TOTAL_DIPLEXED_SLIDERS   (0u)
#define BUTTON_TOTAL_LINEAR_SLIDERS     (0u)
#define BUTTON_TOTAL_RADIAL_SLIDERS     (0u)
#define BUTTON_TOTAL_TOUCHPADS          (0u)
#define BUTTON_MAX_CENTROID_LENGTH      (0u)
#define BUTTON_SLIDER_MULT_METHOD       (0u)
#define BUTTON_TOUCHPAD_MULT_METHOD     (0u)

/*******************************************************************************
* Enabled sensor types
*******************************************************************************/
#define BUTTON_REGULAR_SENSOR_EN        (1u)
#define BUTTON_PROXIMITY_SENSOR_EN      (0u)

/*******************************************************************************
* Sensor ganging
*******************************************************************************/
#define BUTTON_GANGED_SNS_EN            (0u)
#define BUTTON_CSD_GANGED_SNS_EN        (0u)
#define BUTTON_CSX_GANGED_SNS_EN        (0u)

/*******************************************************************************
* Max number of sensors used among all the widgets
*******************************************************************************/
#define BUTTON_MAX_SENSORS_PER_WIDGET   (1u)
#define BUTTON_MAX_SENSORS_PER_5X5_TOUCHPAD (1u)

/*******************************************************************************
* Total number of all used electrodes (NOT unique)
*******************************************************************************/
#define BUTTON_TOTAL_ELECTRODES         (1u)
/* Obsolete */
#define BUTTON_TOTAL_SENSOR_IOS         BUTTON_TOTAL_ELECTRODES

/*******************************************************************************
* Total number of used physical IOs (unique)
*******************************************************************************/
#define BUTTON_TOTAL_IO_CNT             (1u)

/*******************************************************************************
* Array length for widget status registers
*******************************************************************************/
#define BUTTON_WDGT_STATUS_WORDS        \
                        (((uint8)((BUTTON_TOTAL_WIDGETS - 1u) / 32u)) + 1u)


/*******************************************************************************
* Auto-tuning mode selection
*******************************************************************************/
#define BUTTON_CSD_SS_DIS         (0x00ul)
#define BUTTON_CSD_SS_HW_EN       (0x01ul)
#define BUTTON_CSD_SS_TH_EN       (0x02ul)
#define BUTTON_CSD_SS_HWTH_EN     (BUTTON_CSD_SS_HW_EN | \
                                             BUTTON_CSD_SS_TH_EN)

#define BUTTON_CSD_AUTOTUNE       BUTTON_CSD_SS_HWTH_EN


/*******************************************************************************
* General settings
*******************************************************************************/

#define BUTTON_AUTO_RESET_METHOD_LEGACY (0u)
#define BUTTON_AUTO_RESET_METHOD_SAMPLE (1u)

#define BUTTON_MULTI_FREQ_SCAN_EN       (0u)
#define BUTTON_SENSOR_AUTO_RESET_EN     (0u)
#define BUTTON_SENSOR_AUTO_RESET_METHOD (0u)
#define BUTTON_NUM_CENTROIDS            (1u)
#define BUTTON_4PTS_LOCAL_MAX_EN        (0u)
#define BUTTON_OFF_DEBOUNCE_EN          (0u)
#define BUTTON_CUSTOM_DS_RAM_SIZE       (0u)

/* Defines power status of HW block after scanning */
#define BUTTON_BLOCK_OFF_AFTER_SCAN_EN  (0u)

/* Defines number of scan frequencies */
#if (BUTTON_DISABLE != BUTTON_MULTI_FREQ_SCAN_EN)
    #define BUTTON_NUM_SCAN_FREQS       (3u)
#else
    #define BUTTON_NUM_SCAN_FREQS       (1u)
#endif /* #if (BUTTON_DISABLE != BUTTON_MULTI_FREQ_SCAN_EN) */

/* Data size for thresholds / low baseline reset */
#define BUTTON_SIZE_8BITS               (8u)
#define BUTTON_SIZE_16BITS              (16u)

#define BUTTON_THRESHOLD_SIZE           BUTTON_SIZE_16BITS
typedef uint16 BUTTON_THRESHOLD_TYPE;

#if (BUTTON_AUTO_RESET_METHOD_LEGACY == BUTTON_SENSOR_AUTO_RESET_METHOD)
    #define BUTTON_LOW_BSLN_RST_SIZE        BUTTON_SIZE_8BITS
    typedef uint8 BUTTON_LOW_BSLN_RST_TYPE;
#else
    #define BUTTON_LOW_BSLN_RST_SIZE    (16u)
    typedef uint16 BUTTON_LOW_BSLN_RST_TYPE;
#endif /* #if (BUTTON_AUTO_RESET_METHOD_LEGACY == BUTTON_SENSOR_AUTO_RESET_METHOD) */

/* Coefficient to define touch threshold for proximity sensors */
#define BUTTON_PROX_TOUCH_COEFF         (300u)

/*******************************************************************************
* General Filter Constants
*******************************************************************************/

/* Baseline algorithm options */
#define BUTTON_IIR_BASELINE                 (0u)
#define BUTTON_BUCKET_BASELINE              (1u)

#define BUTTON_BASELINE_TYPE                BUTTON_IIR_BASELINE

/* IIR baseline filter algorithm for regular sensors*/
#define BUTTON_REGULAR_IIR_BL_TYPE          BUTTON_IIR_FILTER_PERFORMANCE

/* IIR baseline coefficients for regular sensors */
#define BUTTON_REGULAR_IIR_BL_N             (1u)
#define BUTTON_REGULAR_IIR_BL_SHIFT         (8u)

/* IIR baseline filter algorithm for proximity sensors*/
#define BUTTON_PROX_IIR_BL_TYPE             BUTTON_IIR_FILTER_PERFORMANCE

/* IIR baseline coefficients for proximity sensors */
#define BUTTON_PROX_IIR_BL_N                (1u)
#define BUTTON_PROX_IIR_BL_SHIFT            (8u)


/* IIR filter constants */
#define BUTTON_IIR_COEFFICIENT_K            (256u)

/* IIR filter type */
#define BUTTON_IIR_FILTER_STANDARD          (1u)
#define BUTTON_IIR_FILTER_PERFORMANCE       (2u)
#define BUTTON_IIR_FILTER_MEMORY            (3u)

/* Regular sensor raw count filters */
#define BUTTON_REGULAR_RC_FILTER_EN         (0u)
#define BUTTON_REGULAR_RC_IIR_FILTER_EN     (0u)
#define BUTTON_REGULAR_RC_MEDIAN_FILTER_EN  (0u)
#define BUTTON_REGULAR_RC_AVERAGE_FILTER_EN (0u)
#define BUTTON_REGULAR_RC_CUSTOM_FILTER_EN  (0u)
#define BUTTON_REGULAR_RC_ALP_FILTER_EN     (0u)

/* Proximity sensor raw count filters */
#define BUTTON_PROX_RC_FILTER_EN            (0u)
#define BUTTON_PROX_RC_IIR_FILTER_EN        (0u)
#define BUTTON_PROX_RC_MEDIAN_FILTER_EN     (0u)
#define BUTTON_PROX_RC_AVERAGE_FILTER_EN    (0u)
#define BUTTON_PROX_RC_CUSTOM_FILTER_EN     (0u)
#define BUTTON_PROX_RC_ALP_FILTER_EN        (0u)

#define BUTTON_ALP_FILTER_EN                (0u)
#define BUTTON_REGULAR_RC_ALP_FILTER_COEFF  (2u)
#define BUTTON_PROX_RC_ALP_FILTER_COEFF     (2u)

/* Raw count filters */
#define BUTTON_RC_FILTER_EN                 (BUTTON_REGULAR_RC_FILTER_EN || BUTTON_PROX_RC_FILTER_EN)

/* IIR raw count filter algorithm for regular sensors */
#define BUTTON_REGULAR_IIR_RC_TYPE          (BUTTON_IIR_FILTER_STANDARD)

/* IIR raw count filter coefficients for regular sensors */
#define BUTTON_REGULAR_IIR_RC_N             (128u)
#define BUTTON_REGULAR_IIR_RC_SHIFT         (0u)

/* IIR raw count filter algorithm for proximity sensors*/
#define BUTTON_PROX_IIR_RC_TYPE             (BUTTON_IIR_FILTER_STANDARD)

/* IIR raw count filter coefficients for proximity sensors */
#define BUTTON_PROX_IIR_RC_N                (128u)
#define BUTTON_PROX_IIR_RC_SHIFT            (0u)

/* Median filter constants */

/* Order of regular sensor median filter */
#define BUTTON_REGULAR_MEDIAN_LEN           (2u)

/* Order of proximity sensor median filter */
#define BUTTON_PROX_MEDIAN_LEN              (2u)

/* Average filter constants*/
#define BUTTON_AVERAGE_FILTER_LEN_2         (1u)
#define BUTTON_AVERAGE_FILTER_LEN_4         (3u)

/* Order of regular sensor average filter */
#define BUTTON_REGULAR_AVERAGE_LEN          (BUTTON_AVERAGE_FILTER_LEN_4)

/* Order of proximity sensor average filter */
#define BUTTON_PROX_AVERAGE_LEN             (BUTTON_AVERAGE_FILTER_LEN_4)

/* Widget baseline coefficient enable */
#define BUTTON_WD_BSLN_COEFF_EN             (0u)

/* Centroid position filters */
#define BUTTON_POSITION_FILTER_EN           (0u)
#define BUTTON_POS_MEDIAN_FILTER_EN         (0u)
#define BUTTON_POS_IIR_FILTER_EN            (0u)
#define BUTTON_POS_ADAPTIVE_IIR_FILTER_EN   (0u)
#define BUTTON_POS_AVERAGE_FILTER_EN        (0u)
#define BUTTON_POS_JITTER_FILTER_EN         (0u)
#define BUTTON_BALLISTIC_MULTIPLIER_EN      (0u)
#define BUTTON_CENTROID_3X3_CSD_EN          (0u)
#define BUTTON_CENTROID_5X5_CSD_EN          (0u)
#define BUTTON_CSD_5X5_MAX_FINGERS          (1u)

#define BUTTON_POS_IIR_COEFF                (128u)
#define BUTTON_POS_IIR_RESET_RADIAL_SLIDER  (35u)

#define BUTTON_CSX_TOUCHPAD_UNDEFINED       (40u)

/* IDAC options */

/* Third-generation HW block IDAC gain */
#define BUTTON_IDAC_GAIN_4X                 (4u)
#define BUTTON_IDAC_GAIN_8X                 (8u)

/* Fourth-generation HW block IDAC gain */
#define BUTTON_IDAC_GAIN_LOW                (0uL)
#define BUTTON_IDAC_GAIN_MEDIUM             (1uL)
#define BUTTON_IDAC_GAIN_HIGH               (2uL)

#define BUTTON_IDAC_SOURCING                (0u)
#define BUTTON_IDAC_SINKING                 (1u)

/* Shield tank capacitor precharge source */
#define BUTTON_CSH_PRECHARGE_VREF           (0u)
#define BUTTON_CSH_PRECHARGE_IO_BUF         (1u)

/* Shield electrode delay */
#define BUTTON_NO_DELAY                     (0u)

#if(BUTTON_ENABLE == BUTTON_CSDV2)
    #define BUTTON_SH_DELAY_5NS             (1u)
    #define BUTTON_SH_DELAY_10NS            (2u)
    #define BUTTON_SH_DELAY_20NS            (3u)
#else
    #if(BUTTON_ENABLE == BUTTON_CSDV1_VER2)
        #define BUTTON_SH_DELAY_10NS        (3u)
        #define BUTTON_SH_DELAY_50NS        (2u)
    #else
        #define BUTTON_SH_DELAY_1CYCLES     (1u)
        #define BUTTON_SH_DELAY_2CYCLES     (2u)
    #endif /* (BUTTON_ENABLE == BUTTON_CSDV1_VER2) */
#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */

/* Inactive sensor connection options */
#define BUTTON_SNS_CONNECTION_GROUND        (0x00000000u)
#define BUTTON_SNS_CONNECTION_HIGHZ         (0x00000001u)
#define BUTTON_SNS_CONNECTION_SHIELD        (0x00000002u)
#define BUTTON_SNS_CONNECTION_UNDEFINED     (0x00000003u)

/* Sense clock selection options */
#if defined(BUTTON_TAPEOUT_STAR_USED)
    #define BUTTON_CSDV2_REF9P6UA_EN            (0u)
#else
    #define BUTTON_CSDV2_REF9P6UA_EN            (1u)
#endif /* defined(BUTTON_TAPEOUT_STAR_USED) */

#define BUTTON_CLK_SOURCE_DIRECT            (0x00000000Lu)

#define BUTTON_CLK_SOURCE_SSC1              (0x01u)
#define BUTTON_CLK_SOURCE_SSC2              (0x02u)
#define BUTTON_CLK_SOURCE_SSC3              (0x03u)
#define BUTTON_CLK_SOURCE_SSC4              (0x04u)

#define BUTTON_CLK_SOURCE_PRS8              (0x05u)
#define BUTTON_CLK_SOURCE_PRS12             (0x06u)
#define BUTTON_CLK_SOURCE_PRSAUTO           (0xFFu)

#define BUTTON_MFS_IMO                      (0u)
#define BUTTON_MFS_SNS_CLK                  (1u)

/* Defines scan resolutions */
#define BUTTON_RES6BIT                      (6u)
#define BUTTON_RES7BIT                      (7u)
#define BUTTON_RES8BIT                      (8u)
#define BUTTON_RES9BIT                      (9u)
#define BUTTON_RES10BIT                     (10u)
#define BUTTON_RES11BIT                     (11u)
#define BUTTON_RES12BIT                     (12u)
#define BUTTON_RES13BIT                     (13u)
#define BUTTON_RES14BIT                     (14u)
#define BUTTON_RES15BIT                     (15u)
#define BUTTON_RES16BIT                     (16u)

/* Fourth-generation HW block: Initialization switch resistance */
#define BUTTON_INIT_SW_RES_LOW              (0x00000000Lu)
#define BUTTON_INIT_SW_RES_MEDIUM           (0x00000001Lu)
#define BUTTON_INIT_SW_RES_HIGH             (0x00000002Lu)

/* Fourth-generation HW block: Initialization switch resistance */
#define BUTTON_SCAN_SW_RES_LOW              (0x00000000Lu)
#define BUTTON_SCAN_SW_RES_MEDIUM           (0x00000001Lu)
#define BUTTON_SCAN_SW_RES_HIGH             (0x00000002Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define BUTTON_SHIELD_SW_RES_LOW            (0x00000000Lu)
#define BUTTON_SHIELD_SW_RES_MEDIUM         (0x00000001Lu)
#define BUTTON_SHIELD_SW_RES_HIGH           (0x00000002Lu)
#define BUTTON_SHIELD_SW_RES_LOW_EMI        (0x00000003Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define BUTTON_INIT_SHIELD_SW_RES_LOW       (0x00000000Lu)
#define BUTTON_INIT_SHIELD_SW_RES_MEDIUM    (0x00000001Lu)
#define BUTTON_INIT_SHIELD_SW_RES_HIGH      (0x00000002Lu)
#define BUTTON_INIT_SHIELD_SW_RES_LOW_EMI   (0x00000003Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define BUTTON_SCAN_SHIELD_SW_RES_LOW       (0x00000000Lu)
#define BUTTON_SCAN_SHIELD_SW_RES_MEDIUM    (0x00000001Lu)
#define BUTTON_SCAN_SHIELD_SW_RES_HIGH      (0x00000002Lu)
#define BUTTON_SCAN_SHIELD_SW_RES_LOW_EMI   (0x00000003Lu)

/* Sensing method */
#define BUTTON_SENSING_LEGACY               (0x00000000Lu)
#define BUTTON_SENSING_LOW_EMI              (0x00000001Lu)
#define BUTTON_SENSING_FULL_WAVE            (0x00000002Lu)


/*******************************************************************************
* CSD/CSX Common Settings
*******************************************************************************/

#define BUTTON_BLOCK_ANALOG_WAKEUP_DELAY_US (0u)

#define BUTTON_MFS_METHOD                   (0u)
#define BUTTON_IMO_FREQUENCY_OFFSET_F1      (20u)
#define BUTTON_IMO_FREQUENCY_OFFSET_F2      (20u)

/*******************************************************************************
* CSD Specific Settings
*******************************************************************************/

/* CSD scan method settings */
#define BUTTON_CSD_IDAC_AUTOCAL_EN          (1u)
#define BUTTON_CSD_IDAC_GAIN                (BUTTON_IDAC_GAIN_HIGH)
#define BUTTON_CSD_SHIELD_EN                (1u)
#define BUTTON_CSD_SHIELD_TANK_EN           (0u)
#define BUTTON_CSD_CSH_PRECHARGE_SRC        (BUTTON_CSH_PRECHARGE_VREF)
#define BUTTON_CSD_SHIELD_DELAY             (BUTTON_NO_DELAY)
#define BUTTON_CSD_TOTAL_SHIELD_COUNT       (1u)
#define BUTTON_CSD_SCANSPEED_DIVIDER        (1u)
#define BUTTON_CSD_COMMON_SNS_CLK_EN        (0u)
#define BUTTON_CSD_SNS_CLK_SOURCE           (BUTTON_CLK_SOURCE_PRSAUTO)
#define BUTTON_CSD_SNS_CLK_DIVIDER          (8u)
#define BUTTON_CSD_INACTIVE_SNS_CONNECTION  (BUTTON_SNS_CONNECTION_GROUND)
#define BUTTON_CSD_IDAC_COMP_EN             (1u)
#define BUTTON_CSD_IDAC_CONFIG              (BUTTON_IDAC_SOURCING)
#define BUTTON_CSD_RAWCOUNT_CAL_LEVEL       (85u)
#define BUTTON_CSD_DUALIDAC_LEVEL           (50u)
#define BUTTON_CSD_PRESCAN_SETTLING_TIME    (5u)
#define BUTTON_CSD_SNSCLK_R_CONST           (1000u)
#define BUTTON_CSD_VREF_MV                  (2021u)

#define BUTTON_CSD_CALIBRATION_ERROR        (10u)
#define BUTTON_CSD_IDAC_AUTO_GAIN_EN        (1u)
#define BUTTON_CSD_IDAC_GAIN_INDEX_DEFAULT  (5u)
#define BUTTON_CSD_IDAC_MIN                 (20u)
#define BUTTON_CSD_COL_ROW_IDAC_ALIGNMENT_EN      (1u)

/* The macro is obsolete and should not be used */
#define BUTTON_CSD_DEDICATED_IDAC_COMP_EN   (1u)
/* CSD settings - Fourth-generation HW block */
#define BUTTON_CSD_ANALOG_STARTUP_DELAY_US  (10u)
#define BUTTON_CSD_FINE_INIT_TIME           (10u)
#define BUTTON_CSD_AUTO_ZERO_EN             (0u)
#define BUTTON_CSD_AUTO_ZERO_TIME           (15Lu)
#define BUTTON_CSD_NOISE_METRIC_EN          (0u)
#define BUTTON_CSD_NOISE_METRIC_TH          (1Lu)
#define BUTTON_CSD_INIT_SWITCH_RES          (BUTTON_INIT_SW_RES_MEDIUM)
#define BUTTON_CSD_SENSING_METHOD           (0)
#define BUTTON_CSD_SHIELD_SWITCH_RES        (BUTTON_SHIELD_SW_RES_MEDIUM)
#define BUTTON_CSD_GAIN                     (18Lu)

#define BUTTON_CSD_MFS_DIVIDER_OFFSET_F1    (1u)
#define BUTTON_CSD_MFS_DIVIDER_OFFSET_F2    (2u)

/*******************************************************************************
* CSX Specific Settings
*******************************************************************************/

/* CSX scan method settings */
#define BUTTON_CSX_SCANSPEED_DIVIDER        (1u)
#define BUTTON_CSX_COMMON_TX_CLK_EN         (0u)
#define BUTTON_CSX_TX_CLK_SOURCE            (BUTTON_CLK_SOURCE_PRSAUTO)
#define BUTTON_CSX_TX_CLK_DIVIDER           (80u)
#define BUTTON_CSX_INACTIVE_SNS_CONNECTION  (BUTTON_SNS_CONNECTION_GROUND)
#define BUTTON_CSX_MAX_FINGERS              (1u)
#define BUTTON_CSX_MAX_LOCAL_PEAKS          (5u)
#define BUTTON_CSX_IDAC_AUTOCAL_EN          (0u)
#define BUTTON_CSX_IDAC_BITS_USED           (7u)
#define BUTTON_CSX_RAWCOUNT_CAL_LEVEL       (40u)
#define BUTTON_CSX_IDAC_GAIN                (BUTTON_IDAC_GAIN_MEDIUM)
#define BUTTON_CSX_CALIBRATION_ERROR        (20u)
#define BUTTON_CSX_SKIP_OVSMPL_SPECIFIC_NODES (0u)
#define BUTTON_CSX_MULTIPHASE_TX_EN         (0u)
#define BUTTON_CSX_MAX_TX_PHASE_LENGTH      (0u)

/* CSX settings - Fourth-generation HW block */
#define BUTTON_CSX_ANALOG_STARTUP_DELAY_US  (10u)
#define BUTTON_CSX_AUTO_ZERO_EN             (0u)
#define BUTTON_CSX_AUTO_ZERO_TIME           (15u)
#define BUTTON_CSX_FINE_INIT_TIME           (4u)
#define BUTTON_CSX_NOISE_METRIC_EN          (0u)
#define BUTTON_CSX_NOISE_METRIC_TH          (1u)
#define BUTTON_CSX_INIT_SWITCH_RES          (BUTTON_INIT_SW_RES_MEDIUM)
#define BUTTON_CSX_SCAN_SWITCH_RES          (BUTTON_SCAN_SW_RES_LOW)
#define BUTTON_CSX_INIT_SHIELD_SWITCH_RES   (BUTTON_INIT_SHIELD_SW_RES_HIGH)
#define BUTTON_CSX_SCAN_SHIELD_SWITCH_RES   (BUTTON_SCAN_SHIELD_SW_RES_LOW)

#define BUTTON_CSX_MFS_DIVIDER_OFFSET_F1    (1u)
#define BUTTON_CSX_MFS_DIVIDER_OFFSET_F2    (2u)

/* Gesture parameters */
#define BUTTON_GES_GLOBAL_EN                (0u)

/*******************************************************************************
* ISX Specific Settings
*******************************************************************************/

/* ISX scan method settings */
#define BUTTON_ISX_SCANSPEED_DIVIDER        (1u)
#define BUTTON_ISX_LX_CLK_DIVIDER           (80u)
#define BUTTON_ISX_IDAC_AUTOCAL_EN          (0u)
#define BUTTON_ISX_IDAC_BITS_USED           (7u)
#define BUTTON_ISX_RAWCOUNT_CAL_LEVEL       (30u)
#define BUTTON_ISX_IDAC_GAIN                (BUTTON_IDAC_GAIN_MEDIUM)
#define BUTTON_ISX_SKIP_OVSMPL_SPECIFIC_NODES (0u)
#define BUTTON_ISX_MAX_TX_PHASE_LENGTH      (0u)
#define BUTTON_ISX_PIN_COUNT_LX             (u)
/* ISX settings - Fourth-generation HW block */
#define BUTTON_ISX_AUTO_ZERO_EN             (0u)
#define BUTTON_ISX_AUTO_ZERO_TIME           (15u)
#define BUTTON_ISX_FINE_INIT_TIME           (20u)
#define BUTTON_ISX_NOISE_METRIC_EN          (0u)
#define BUTTON_ISX_NOISE_METRIC_TH          (1u)
#define BUTTON_ISX_INIT_SWITCH_RES          (BUTTON_INIT_SW_RES_MEDIUM)
#define BUTTON_ISX_SCAN_SWITCH_RES          (BUTTON_SCAN_SW_RES_LOW)
#define BUTTON_ISX_INIT_SHIELD_SWITCH_RES   (BUTTON_INIT_SHIELD_SW_RES_HIGH)
#define BUTTON_ISX_SCAN_SHIELD_SWITCH_RES   (BUTTON_SCAN_SHIELD_SW_RES_LOW)
#define BUTTON_ISX_SAMPLE_PHASE_DEG         (30u)

/*******************************************************************************
* Global Parameter Definitions
*******************************************************************************/

/* Compound section definitions */
#define BUTTON_ANY_NONSS_AUTOCAL ((0u != BUTTON_CSX_IDAC_AUTOCAL_EN) || \
                                       (0u != BUTTON_ISX_IDAC_AUTOCAL_EN) || \
                                      ((BUTTON_CSD_AUTOTUNE == BUTTON_CSD_SS_DIS) && (0u != BUTTON_CSD_IDAC_AUTOCAL_EN)))
#define BUTTON_ANYMODE_AUTOCAL (((0u != BUTTON_CSX_IDAC_AUTOCAL_EN) \
                                       || (0u != BUTTON_ISX_IDAC_AUTOCAL_EN)) \
                                       || (0u != BUTTON_CSD_IDAC_AUTOCAL_EN))
/* RAM Global Parameters Definitions */
#define BUTTON_CONFIG_ID                        (0xD8A4u)
#define BUTTON_DEVICE_ID                        (0x0900u)
#define BUTTON_HW_CLOCK                         (0x0BB8u)
#define BUTTON_CSD0_CONFIG                      (0x0108u)

/*******************************************************************************
* Button0 initialization values for FLASH data structure
*******************************************************************************/
#define BUTTON_BUTTON0_STATIC_CONFIG            (10241u)
#define BUTTON_BUTTON0_NUM_SENSORS              (1u)

/*******************************************************************************
* Button0 initialization values for RAM data structure
*******************************************************************************/
#define BUTTON_BUTTON0_RESOLUTION               (BUTTON_RES12BIT)
#define BUTTON_BUTTON0_FINGER_TH                (100u)
#define BUTTON_BUTTON0_NOISE_TH                 (40u)
#define BUTTON_BUTTON0_NNOISE_TH                (40u)
#define BUTTON_BUTTON0_HYSTERESIS               (10u)
#define BUTTON_BUTTON0_ON_DEBOUNCE              (3u)
#define BUTTON_BUTTON0_LOW_BSLN_RST             (30u)
#define BUTTON_BUTTON0_IDAC_MOD0                (32u)
#define BUTTON_BUTTON0_IDAC_GAIN_INDEX          (5u)
#define BUTTON_BUTTON0_SNS_CLK                  (4u)
#define BUTTON_BUTTON0_SNS_CLK_SOURCE           (0u)
#define BUTTON_BUTTON0_FINGER_CAP               (160u)
#define BUTTON_BUTTON0_SIGPFC                   (0u)

/* RAM Sensor Parameters Definitions */
#define BUTTON_BUTTON0_SNS0_IDAC_COMP0          (32u)



/*******************************************************************************
* ADC Specific Macros
*******************************************************************************/
#define BUTTON_ADC_RES8BIT                  (8u)
#define BUTTON_ADC_RES10BIT                 (10u)

#define BUTTON_ADC_FULLRANGE_MODE           (0u)
#define BUTTON_ADC_VREF_MODE                (1u)

#define BUTTON_ADC_MIN_CHANNELS             (1u)
#define BUTTON_ADC_EN                       (0u)
#define BUTTON_ADC_STANDALONE_EN            (0u)
#define BUTTON_ADC_TOTAL_CHANNELS           (1u)
#define BUTTON_ADC_RESOLUTION               (BUTTON_ADC_RES10BIT)
#define BUTTON_ADC_AMUXB_INPUT_EN           (0u)
#define BUTTON_ADC_SELECT_AMUXB_CH          (0u)
#define BUTTON_ADC_AZ_EN                    (1Lu)
#define BUTTON_ADC_AZ_TIME                  (5u)
#define BUTTON_ADC_VREF_MV                  (2133u)
#define BUTTON_ADC_GAIN                     (17Lu)
#define BUTTON_ADC_IDAC_DEFAULT             (19u)
#define BUTTON_ADC_MODCLK_DIV_DEFAULT       (1u)
#define BUTTON_ADC_MEASURE_MODE             (BUTTON_ADC_FULLRANGE_MODE)
#define BUTTON_ADC_ANALOG_STARTUP_DELAY_US  (5u)
#define BUTTON_ADC_ACQUISITION_TIME_US      (13u)

/*******************************************************************************
* Built-In Self-Test Configuration
*******************************************************************************/
#define BUTTON_SELF_TEST_EN                   (0Lu)
#define BUTTON_TST_GLOBAL_CRC_EN              (0Lu)
#define BUTTON_TST_WDGT_CRC_EN                (0Lu)
#define BUTTON_TST_BSLN_DUPLICATION_EN        (0Lu)
#define BUTTON_TST_BSLN_RAW_OUT_RANGE_EN      (0Lu)
#define BUTTON_TST_SNS_SHORT_EN               (0Lu)
#define BUTTON_TST_SNS_CAP_EN                 (0Lu)
#define BUTTON_TST_SH_CAP_EN                  (0Lu)
#define BUTTON_TST_EXTERNAL_CAP_EN            (0Lu)
#define BUTTON_TST_VDDA_EN                    (0Lu)


#define BUTTON_GLOBAL_CRC_AREA_START          (0u)
#define BUTTON_GLOBAL_CRC_AREA_SIZE           (0u)
#define BUTTON_WIDGET_CRC_AREA_START          (0u)
#define BUTTON_WIDGET_CRC_AREA_SIZE           (0u)

/* The resolution for sensor capacity measurement */
#define BUTTON_TST_SNS_CAP_RESOLUTION         (12u)

/* VDDA measurement test configuration */
/* The resolution for VDDA measurement */
#define BUTTON_TST_VDDA_RESOLUTION            (10u)
/* The ModClk divider for external capacitor capacity measurement */
#define BUTTON_TST_VDDA_MOD_CLK_DIVIDER       (1u)

#define BUTTON_TST_VDDA_VREF_MV               (0u)
#define BUTTON_TST_VDDA_VREF_GAIN             (0u)
#define BUTTON_TST_VDDA_IDAC_DEFAULT          (0u)

#define BUTTON_TST_FINE_INIT_TIME             (10u)
#define BUTTON_TST_ANALOG_STARTUP_DELAY_US    (23u)

#define BUTTON_TST_IDAC_AUTO_GAIN_EN          (1u)
#define BUTTON_TST_SNS_CAP_RAW_ERROR          (10u)
#define BUTTON_TST_IDAC_GAIN_INDEX            (0xFFu)
#define BUTTON_TST_RAW_TARGET                 (85u)

#define BUTTON_TST_SNS_SHORT_TIME             (2u)

#define BUTTON_SNS_CAP_CSD_INACTIVE_CONNECTION        (BUTTON_SNS_CONNECTION_GROUND)
#define BUTTON_SNS_CAP_CSX_INACTIVE_CONNECTION        (BUTTON_SNS_CONNECTION_GROUND)
#define BUTTON_SHIELD_CAP_INACTIVE_CONNECTION         (BUTTON_SNS_CONNECTION_GROUND)


/*******************************************************************************
* Gesture Configuration
*******************************************************************************/
#define BUTTON_TIMESTAMP_INTERVAL             (1Lu)
#define BUTTON_GESTURE_EN_WIDGET_ID           (0Lu)
#define BUTTON_BALLISTIC_EN_WIDGET_ID         (0Lu)


#endif /* CY_SENSE_BUTTON_CONFIGURATION_H */


/* [] END OF FILE */
