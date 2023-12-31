/***************************************************************************//**
* \file BUTTON_RegisterMap.h
* \version 7.0
*
* \brief
*   This file provides the definitions for the Component data structure register.
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

#if !defined(CY_SENSE_BUTTON_REGISTER_MAP_H)
#define CY_SENSE_BUTTON_REGISTER_MAP_H

#include <cytypes.h>
#include "BUTTON_Configuration.h"
#include "BUTTON_Structure.h"

/*****************************************************************************/
/* RAM Data structure register definitions                                   */
/*****************************************************************************/
#define BUTTON_CONFIG_ID_VALUE                              (BUTTON_dsRam.configId)
#define BUTTON_CONFIG_ID_OFFSET                             (0u)
#define BUTTON_CONFIG_ID_SIZE                               (2u)
#define BUTTON_CONFIG_ID_PARAM_ID                           (0x87000000u)

#define BUTTON_DEVICE_ID_VALUE                              (BUTTON_dsRam.deviceId)
#define BUTTON_DEVICE_ID_OFFSET                             (2u)
#define BUTTON_DEVICE_ID_SIZE                               (2u)
#define BUTTON_DEVICE_ID_PARAM_ID                           (0x8B000002u)

#define BUTTON_HW_CLOCK_VALUE                               (BUTTON_dsRam.hwClock)
#define BUTTON_HW_CLOCK_OFFSET                              (4u)
#define BUTTON_HW_CLOCK_SIZE                                (2u)
#define BUTTON_HW_CLOCK_PARAM_ID                            (0x86000004u)

#define BUTTON_TUNER_CMD_VALUE                              (BUTTON_dsRam.tunerCmd)
#define BUTTON_TUNER_CMD_OFFSET                             (6u)
#define BUTTON_TUNER_CMD_SIZE                               (2u)
#define BUTTON_TUNER_CMD_PARAM_ID                           (0xA1000006u)

#define BUTTON_SCAN_COUNTER_VALUE                           (BUTTON_dsRam.scanCounter)
#define BUTTON_SCAN_COUNTER_OFFSET                          (8u)
#define BUTTON_SCAN_COUNTER_SIZE                            (2u)
#define BUTTON_SCAN_COUNTER_PARAM_ID                        (0x85000008u)

#define BUTTON_STATUS_VALUE                                 (BUTTON_dsRam.status)
#define BUTTON_STATUS_OFFSET                                (12u)
#define BUTTON_STATUS_SIZE                                  (4u)
#define BUTTON_STATUS_PARAM_ID                              (0xCB00000Cu)

#define BUTTON_WDGT_ENABLE0_VALUE                           (BUTTON_dsRam.wdgtEnable[0u])
#define BUTTON_WDGT_ENABLE0_OFFSET                          (16u)
#define BUTTON_WDGT_ENABLE0_SIZE                            (4u)
#define BUTTON_WDGT_ENABLE0_PARAM_ID                        (0xE6000010u)

#define BUTTON_WDGT_STATUS0_VALUE                           (BUTTON_dsRam.wdgtStatus[0u])
#define BUTTON_WDGT_STATUS0_OFFSET                          (20u)
#define BUTTON_WDGT_STATUS0_SIZE                            (4u)
#define BUTTON_WDGT_STATUS0_PARAM_ID                        (0xCC000014u)

#define BUTTON_SNS_STATUS0_VALUE                            (BUTTON_dsRam.snsStatus[0u])
#define BUTTON_SNS_STATUS0_OFFSET                           (24u)
#define BUTTON_SNS_STATUS0_SIZE                             (1u)
#define BUTTON_SNS_STATUS0_PARAM_ID                         (0x48000018u)

#define BUTTON_CSD0_CONFIG_VALUE                            (BUTTON_dsRam.csd0Config)
#define BUTTON_CSD0_CONFIG_OFFSET                           (26u)
#define BUTTON_CSD0_CONFIG_SIZE                             (2u)
#define BUTTON_CSD0_CONFIG_PARAM_ID                         (0xAA80001Au)

#define BUTTON_MOD_CSD_CLK_VALUE                            (BUTTON_dsRam.modCsdClk)
#define BUTTON_MOD_CSD_CLK_OFFSET                           (28u)
#define BUTTON_MOD_CSD_CLK_SIZE                             (1u)
#define BUTTON_MOD_CSD_CLK_PARAM_ID                         (0x6F80001Cu)

#define BUTTON_BUTTON0_RESOLUTION_VALUE                     (BUTTON_dsRam.wdgtList.button0.resolution)
#define BUTTON_BUTTON0_RESOLUTION_OFFSET                    (30u)
#define BUTTON_BUTTON0_RESOLUTION_SIZE                      (2u)
#define BUTTON_BUTTON0_RESOLUTION_PARAM_ID                  (0x8080001Eu)

#define BUTTON_BUTTON0_FINGER_TH_VALUE                      (BUTTON_dsRam.wdgtList.button0.fingerTh)
#define BUTTON_BUTTON0_FINGER_TH_OFFSET                     (32u)
#define BUTTON_BUTTON0_FINGER_TH_SIZE                       (2u)
#define BUTTON_BUTTON0_FINGER_TH_PARAM_ID                   (0x80800020u)

#define BUTTON_BUTTON0_NOISE_TH_VALUE                       (BUTTON_dsRam.wdgtList.button0.noiseTh)
#define BUTTON_BUTTON0_NOISE_TH_OFFSET                      (34u)
#define BUTTON_BUTTON0_NOISE_TH_SIZE                        (1u)
#define BUTTON_BUTTON0_NOISE_TH_PARAM_ID                    (0x44800022u)

#define BUTTON_BUTTON0_NNOISE_TH_VALUE                      (BUTTON_dsRam.wdgtList.button0.nNoiseTh)
#define BUTTON_BUTTON0_NNOISE_TH_OFFSET                     (35u)
#define BUTTON_BUTTON0_NNOISE_TH_SIZE                       (1u)
#define BUTTON_BUTTON0_NNOISE_TH_PARAM_ID                   (0x42800023u)

#define BUTTON_BUTTON0_HYSTERESIS_VALUE                     (BUTTON_dsRam.wdgtList.button0.hysteresis)
#define BUTTON_BUTTON0_HYSTERESIS_OFFSET                    (36u)
#define BUTTON_BUTTON0_HYSTERESIS_SIZE                      (1u)
#define BUTTON_BUTTON0_HYSTERESIS_PARAM_ID                  (0x49800024u)

#define BUTTON_BUTTON0_ON_DEBOUNCE_VALUE                    (BUTTON_dsRam.wdgtList.button0.onDebounce)
#define BUTTON_BUTTON0_ON_DEBOUNCE_OFFSET                   (37u)
#define BUTTON_BUTTON0_ON_DEBOUNCE_SIZE                     (1u)
#define BUTTON_BUTTON0_ON_DEBOUNCE_PARAM_ID                 (0x4F800025u)

#define BUTTON_BUTTON0_LOW_BSLN_RST_VALUE                   (BUTTON_dsRam.wdgtList.button0.lowBslnRst)
#define BUTTON_BUTTON0_LOW_BSLN_RST_OFFSET                  (38u)
#define BUTTON_BUTTON0_LOW_BSLN_RST_SIZE                    (1u)
#define BUTTON_BUTTON0_LOW_BSLN_RST_PARAM_ID                (0x45800026u)

#define BUTTON_BUTTON0_IDAC_MOD0_VALUE                      (BUTTON_dsRam.wdgtList.button0.idacMod[0u])
#define BUTTON_BUTTON0_IDAC_MOD0_OFFSET                     (39u)
#define BUTTON_BUTTON0_IDAC_MOD0_SIZE                       (1u)
#define BUTTON_BUTTON0_IDAC_MOD0_PARAM_ID                   (0x4E000027u)

#define BUTTON_BUTTON0_IDAC_GAIN_INDEX_VALUE                (BUTTON_dsRam.wdgtList.button0.idacGainIndex)
#define BUTTON_BUTTON0_IDAC_GAIN_INDEX_OFFSET               (40u)
#define BUTTON_BUTTON0_IDAC_GAIN_INDEX_SIZE                 (1u)
#define BUTTON_BUTTON0_IDAC_GAIN_INDEX_PARAM_ID             (0x4A800028u)

#define BUTTON_BUTTON0_SNS_CLK_VALUE                        (BUTTON_dsRam.wdgtList.button0.snsClk)
#define BUTTON_BUTTON0_SNS_CLK_OFFSET                       (42u)
#define BUTTON_BUTTON0_SNS_CLK_SIZE                         (2u)
#define BUTTON_BUTTON0_SNS_CLK_PARAM_ID                     (0x8E80002Au)

#define BUTTON_BUTTON0_SNS_CLK_SOURCE_VALUE                 (BUTTON_dsRam.wdgtList.button0.snsClkSource)
#define BUTTON_BUTTON0_SNS_CLK_SOURCE_OFFSET                (44u)
#define BUTTON_BUTTON0_SNS_CLK_SOURCE_SIZE                  (1u)
#define BUTTON_BUTTON0_SNS_CLK_SOURCE_PARAM_ID              (0x4B80002Cu)

#define BUTTON_BUTTON0_FINGER_CAP_VALUE                     (BUTTON_dsRam.wdgtList.button0.fingerCap)
#define BUTTON_BUTTON0_FINGER_CAP_OFFSET                    (46u)
#define BUTTON_BUTTON0_FINGER_CAP_SIZE                      (2u)
#define BUTTON_BUTTON0_FINGER_CAP_PARAM_ID                  (0xA900002Eu)

#define BUTTON_BUTTON0_SIGPFC_VALUE                         (BUTTON_dsRam.wdgtList.button0.sigPFC)
#define BUTTON_BUTTON0_SIGPFC_OFFSET                        (48u)
#define BUTTON_BUTTON0_SIGPFC_SIZE                          (2u)
#define BUTTON_BUTTON0_SIGPFC_PARAM_ID                      (0xA3000030u)

#define BUTTON_BUTTON0_SNS0_RAW0_VALUE                      (BUTTON_dsRam.snsList.button0[0u].raw[0u])
#define BUTTON_BUTTON0_SNS0_RAW0_OFFSET                     (50u)
#define BUTTON_BUTTON0_SNS0_RAW0_SIZE                       (2u)
#define BUTTON_BUTTON0_SNS0_RAW0_PARAM_ID                   (0x84000032u)

#define BUTTON_BUTTON0_SNS0_BSLN0_VALUE                     (BUTTON_dsRam.snsList.button0[0u].bsln[0u])
#define BUTTON_BUTTON0_SNS0_BSLN0_OFFSET                    (52u)
#define BUTTON_BUTTON0_SNS0_BSLN0_SIZE                      (2u)
#define BUTTON_BUTTON0_SNS0_BSLN0_PARAM_ID                  (0x89000034u)

#define BUTTON_BUTTON0_SNS0_BSLN_EXT0_VALUE                 (BUTTON_dsRam.snsList.button0[0u].bslnExt[0u])
#define BUTTON_BUTTON0_SNS0_BSLN_EXT0_OFFSET                (54u)
#define BUTTON_BUTTON0_SNS0_BSLN_EXT0_SIZE                  (1u)
#define BUTTON_BUTTON0_SNS0_BSLN_EXT0_PARAM_ID              (0x4D000036u)

#define BUTTON_BUTTON0_SNS0_DIFF_VALUE                      (BUTTON_dsRam.snsList.button0[0u].diff)
#define BUTTON_BUTTON0_SNS0_DIFF_OFFSET                     (56u)
#define BUTTON_BUTTON0_SNS0_DIFF_SIZE                       (2u)
#define BUTTON_BUTTON0_SNS0_DIFF_PARAM_ID                   (0x8A000038u)

#define BUTTON_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_VALUE         (BUTTON_dsRam.snsList.button0[0u].negBslnRstCnt[0u])
#define BUTTON_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (58u)
#define BUTTON_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define BUTTON_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4E00003Au)

#define BUTTON_BUTTON0_SNS0_IDAC_COMP0_VALUE                (BUTTON_dsRam.snsList.button0[0u].idacComp[0u])
#define BUTTON_BUTTON0_SNS0_IDAC_COMP0_OFFSET               (59u)
#define BUTTON_BUTTON0_SNS0_IDAC_COMP0_SIZE                 (1u)
#define BUTTON_BUTTON0_SNS0_IDAC_COMP0_PARAM_ID             (0x4800003Bu)

#define BUTTON_SNR_TEST_WIDGET_ID_VALUE                     (BUTTON_dsRam.snrTestWidgetId)
#define BUTTON_SNR_TEST_WIDGET_ID_OFFSET                    (60u)
#define BUTTON_SNR_TEST_WIDGET_ID_SIZE                      (1u)
#define BUTTON_SNR_TEST_WIDGET_ID_PARAM_ID                  (0x6800003Cu)

#define BUTTON_SNR_TEST_SENSOR_ID_VALUE                     (BUTTON_dsRam.snrTestSensorId)
#define BUTTON_SNR_TEST_SENSOR_ID_OFFSET                    (61u)
#define BUTTON_SNR_TEST_SENSOR_ID_SIZE                      (1u)
#define BUTTON_SNR_TEST_SENSOR_ID_PARAM_ID                  (0x6E00003Du)

#define BUTTON_SNR_TEST_SCAN_COUNTER_VALUE                  (BUTTON_dsRam.snrTestScanCounter)
#define BUTTON_SNR_TEST_SCAN_COUNTER_OFFSET                 (62u)
#define BUTTON_SNR_TEST_SCAN_COUNTER_SIZE                   (2u)
#define BUTTON_SNR_TEST_SCAN_COUNTER_PARAM_ID               (0x8700003Eu)

#define BUTTON_SNR_TEST_RAW_COUNT0_VALUE                    (BUTTON_dsRam.snrTestRawCount[0u])
#define BUTTON_SNR_TEST_RAW_COUNT0_OFFSET                   (64u)
#define BUTTON_SNR_TEST_RAW_COUNT0_SIZE                     (2u)
#define BUTTON_SNR_TEST_RAW_COUNT0_PARAM_ID                 (0x8A000040u)

#define BUTTON_SCAN_CSD_ISC_VALUE                           (BUTTON_dsRam.scanCsdISC)
#define BUTTON_SCAN_CSD_ISC_OFFSET                          (66u)
#define BUTTON_SCAN_CSD_ISC_SIZE                            (1u)
#define BUTTON_SCAN_CSD_ISC_PARAM_ID                        (0x4E000042u)

#define BUTTON_SCAN_CURRENT_ISC_VALUE                       (BUTTON_dsRam.scanCurrentISC)
#define BUTTON_SCAN_CURRENT_ISC_OFFSET                      (67u)
#define BUTTON_SCAN_CURRENT_ISC_SIZE                        (1u)
#define BUTTON_SCAN_CURRENT_ISC_PARAM_ID                    (0x48000043u)


/*****************************************************************************/
/* Flash Data structure register definitions                                 */
/*****************************************************************************/
#define BUTTON_BUTTON0_PTR2SNS_FLASH_VALUE                  (BUTTON_dsFlash.wdgtArray[0].ptr2SnsFlash)
#define BUTTON_BUTTON0_PTR2SNS_FLASH_OFFSET                 (0u)
#define BUTTON_BUTTON0_PTR2SNS_FLASH_SIZE                   (4u)
#define BUTTON_BUTTON0_PTR2SNS_FLASH_PARAM_ID               (0xD1000000u)

#define BUTTON_BUTTON0_PTR2WD_RAM_VALUE                     (BUTTON_dsFlash.wdgtArray[0].ptr2WdgtRam)
#define BUTTON_BUTTON0_PTR2WD_RAM_OFFSET                    (4u)
#define BUTTON_BUTTON0_PTR2WD_RAM_SIZE                      (4u)
#define BUTTON_BUTTON0_PTR2WD_RAM_PARAM_ID                  (0xD0000004u)

#define BUTTON_BUTTON0_PTR2SNS_RAM_VALUE                    (BUTTON_dsFlash.wdgtArray[0].ptr2SnsRam)
#define BUTTON_BUTTON0_PTR2SNS_RAM_OFFSET                   (8u)
#define BUTTON_BUTTON0_PTR2SNS_RAM_SIZE                     (4u)
#define BUTTON_BUTTON0_PTR2SNS_RAM_PARAM_ID                 (0xD3000008u)

#define BUTTON_BUTTON0_PTR2FLTR_HISTORY_VALUE               (BUTTON_dsFlash.wdgtArray[0].ptr2FltrHistory)
#define BUTTON_BUTTON0_PTR2FLTR_HISTORY_OFFSET              (12u)
#define BUTTON_BUTTON0_PTR2FLTR_HISTORY_SIZE                (4u)
#define BUTTON_BUTTON0_PTR2FLTR_HISTORY_PARAM_ID            (0xD200000Cu)

#define BUTTON_BUTTON0_PTR2DEBOUNCE_VALUE                   (BUTTON_dsFlash.wdgtArray[0].ptr2DebounceArr)
#define BUTTON_BUTTON0_PTR2DEBOUNCE_OFFSET                  (16u)
#define BUTTON_BUTTON0_PTR2DEBOUNCE_SIZE                    (4u)
#define BUTTON_BUTTON0_PTR2DEBOUNCE_PARAM_ID                (0xD4000010u)

#define BUTTON_BUTTON0_STATIC_CONFIG_VALUE                  (BUTTON_dsFlash.wdgtArray[0].staticConfig)
#define BUTTON_BUTTON0_STATIC_CONFIG_OFFSET                 (20u)
#define BUTTON_BUTTON0_STATIC_CONFIG_SIZE                   (4u)
#define BUTTON_BUTTON0_STATIC_CONFIG_PARAM_ID               (0xD5000014u)

#define BUTTON_BUTTON0_TOTAL_NUM_SNS_VALUE                  (BUTTON_dsFlash.wdgtArray[0].totalNumSns)
#define BUTTON_BUTTON0_TOTAL_NUM_SNS_OFFSET                 (24u)
#define BUTTON_BUTTON0_TOTAL_NUM_SNS_SIZE                   (2u)
#define BUTTON_BUTTON0_TOTAL_NUM_SNS_PARAM_ID               (0x99000018u)

#define BUTTON_BUTTON0_TYPE_VALUE                           (BUTTON_dsFlash.wdgtArray[0].wdgtType)
#define BUTTON_BUTTON0_TYPE_OFFSET                          (26u)
#define BUTTON_BUTTON0_TYPE_SIZE                            (1u)
#define BUTTON_BUTTON0_TYPE_PARAM_ID                        (0x5D00001Au)

#define BUTTON_BUTTON0_NUM_COLS_VALUE                       (BUTTON_dsFlash.wdgtArray[0].numCols)
#define BUTTON_BUTTON0_NUM_COLS_OFFSET                      (27u)
#define BUTTON_BUTTON0_NUM_COLS_SIZE                        (1u)
#define BUTTON_BUTTON0_NUM_COLS_PARAM_ID                    (0x5B00001Bu)

#define BUTTON_BUTTON0_PTR2NOISE_ENVLP_VALUE                (BUTTON_dsFlash.wdgtArray[0].ptr2NoiseEnvlp)
#define BUTTON_BUTTON0_PTR2NOISE_ENVLP_OFFSET               (28u)
#define BUTTON_BUTTON0_PTR2NOISE_ENVLP_SIZE                 (4u)
#define BUTTON_BUTTON0_PTR2NOISE_ENVLP_PARAM_ID             (0xD700001Cu)


#endif /* End CY_SENSE_BUTTON_REGISTER_MAP_H */


/* [] END OF FILE */
