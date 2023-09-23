/***************************************************************************//**
* \file BUTTON_INT.c
* \version 7.0
*
* \brief
*   This file contains the source code for implementation of the Component's
*   Interrupt Service Routine (ISR).
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
#include "cytypes.h"
#include "cyfitter.h"
#include "BUTTON_Configuration.h"
#include "BUTTON_Structure.h"
#include "BUTTON_Sensing.h"
#if (BUTTON_ENABLE == BUTTON_CSD_EN)
    #include "BUTTON_SensingCSD_LL.h"
#endif /* (BUTTON_ENABLE == BUTTON_CSD_EN) */
#include "cyapicallbacks.h"

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

#if (((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN)) && \
     (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN))
    static void BUTTON_SsNextFrequencyScan(void);
#endif /* (((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN)) && \
            (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)) */

#if ((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN))
    static void BUTTON_SsCSDPostScan(void);
    static void BUTTON_SsCSDInitNextScan(void);
#endif /* ((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN)) */
/** \}
* \endcond */


/**
* \cond SECTION_C_INTERRUPT
* \addtogroup group_c_interrupt
* \{
*/


#if ((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN))

#if (BUTTON_ENABLE == BUTTON_CSDV2)
    /* Fourth-generation HW block part */

    /*******************************************************************************
    * Function Name: BUTTON_CSDPostSingleScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the single-sensor scanning implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  BUTTON_CSDScanExt() function.
    *
    *  The following tasks are performed for Third-generation HW block:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The following tasks are performed for Fourth-generation HW block:
    *    1. Check if the raw data is not noisy.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Configure and start the scan for the next frequency if the
    *      multi-frequency is enabled.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The ISR handler changes the IMO and initializes scanning for the next frequency
    *  channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user code
    *  from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(BUTTON_CSDPostSingleScan)
    {
        #ifdef BUTTON_ENTRY_CALLBACK
            BUTTON_EntryCallback();
        #endif /* BUTTON_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(BUTTON_INTR_PTR, BUTTON_INTR_SET_MASK);

        #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
            if ((BUTTON_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(BUTTON_RESULT_VAL1_PTR) &
                                                        BUTTON_RESULT_VAL1_BAD_CONVS_MASK) >>
                                                        BUTTON_RESULT_VAL1_BAD_CONVS_SHIFT)) &&
                                                        (0u < BUTTON_badConversionsNum))
            {
                /* Decrement bad conversions number */
                BUTTON_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(BUTTON_SEQ_START_PTR, BUTTON_SEQ_START_AZ0_SKIP_MASK |
                                                             BUTTON_SEQ_START_AZ1_SKIP_MASK |
                                                             BUTTON_SEQ_START_START_MASK);
            }
            else
            {
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

            BUTTON_SsCSDPostScan();

            #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
                if (BUTTON_FREQ_CHANNEL_2 > BUTTON_scanFreqIndex)
                {
                    /* Scan the next channel */
                    BUTTON_SsNextFrequencyScan();
                }
                else
                {
                    /* All channels are scanned. Set IMO to zero channel */
                    BUTTON_scanFreqIndex = BUTTON_FREQ_CHANNEL_0;

                    #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
                        BUTTON_SsChangeImoFreq(BUTTON_FREQ_CHANNEL_0);
                    #else
                        BUTTON_SsChangeClkFreq(BUTTON_FREQ_CHANNEL_0);
                    #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

                    #if (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN)
                        /* Disable Fourth-generation HW block */
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd);
                    #endif /* (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN) */

                    /* Update Scan Counter */
                    BUTTON_dsRam.scanCounter++;

                    /* Sensor is totally scanned. Reset BUSY flag */
                    BUTTON_dsRam.status &= ~(BUTTON_SW_STS_BUSY | BUTTON_WDGT_SW_STS_BUSY);
                }
            #else
                {
                    #if (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN)
                        /* Disable Fourth-generation HW block */
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd);
                    #endif /* (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN) */

                    /* Update Scan Counter */
                    BUTTON_dsRam.scanCounter++;

                    /* Sensor is totally scanned. Reset BUSY flag */
                    BUTTON_dsRam.status &= ~(BUTTON_SW_STS_BUSY | BUTTON_WDGT_SW_STS_BUSY);
                }
            #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */
    #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
        }
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

        #ifdef BUTTON_EXIT_CALLBACK
            BUTTON_ExitCallback();
        #endif /* BUTTON_EXIT_CALLBACK */
    }


    /*******************************************************************************
    * Function Name: BUTTON_CSDPostMultiScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  BUTTON_Scan() or BUTTON_ScanAllWidgets() APIs.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  BUTTON_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(BUTTON_CSDPostMultiScan)
    {
        /* Declare and initialize ptr to sensor IO structure to appropriate address */
        BUTTON_FLASH_IO_STRUCT const *curSnsIOPtr = (BUTTON_FLASH_IO_STRUCT const *)
                                                          BUTTON_dsFlash.wdgtArray[BUTTON_widgetIndex].ptr2SnsFlash
                                                          + BUTTON_sensorIndex;

        #ifdef BUTTON_ENTRY_CALLBACK
            BUTTON_EntryCallback();
        #endif /* BUTTON_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(BUTTON_INTR_PTR, BUTTON_INTR_SET_MASK);

        #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
            if ((BUTTON_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(BUTTON_RESULT_VAL1_PTR) &
                                                      BUTTON_RESULT_VAL1_BAD_CONVS_MASK) >>
                                                      BUTTON_RESULT_VAL1_BAD_CONVS_SHIFT)) &&
                                                      (0u < BUTTON_badConversionsNum))
            {
                /* Decrement bad conversions number */
                BUTTON_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(BUTTON_SEQ_START_PTR, BUTTON_SEQ_START_AZ0_SKIP_MASK |
                                                             BUTTON_SEQ_START_AZ1_SKIP_MASK |
                                                             BUTTON_SEQ_START_START_MASK);
            }
            else
            {
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

            BUTTON_SsCSDPostScan();

            /* Disable sensor when all frequency channels are scanned */
            #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
                if (BUTTON_FREQ_CHANNEL_2 == BUTTON_scanFreqIndex)
            #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */
            {
                /* Disable sensor */
                BUTTON_CSDDisconnectSns(curSnsIOPtr);
            }

            #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
                if (BUTTON_FREQ_CHANNEL_2 > BUTTON_scanFreqIndex)
                {
                     /* Scan the next channel */
                    BUTTON_SsNextFrequencyScan();
                }
                else
                {
                     /* All channels are scanned. Set IMO to zero channel */
                    BUTTON_scanFreqIndex = BUTTON_FREQ_CHANNEL_0;

                    #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
                        BUTTON_SsChangeImoFreq(BUTTON_FREQ_CHANNEL_0);
                    #else
                        BUTTON_SsChangeClkFreq(BUTTON_FREQ_CHANNEL_0);
                    #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

                     /* Scan the next sensor */
                    BUTTON_SsCSDInitNextScan();
                }
            #else
                /* Scan the next sensor */
                BUTTON_SsCSDInitNextScan();
            #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */
        #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
            }
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

        #ifdef BUTTON_EXIT_CALLBACK
            BUTTON_ExitCallback();
        #endif /* BUTTON_EXIT_CALLBACK */
    }


    #if (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN)
    /*******************************************************************************
    * Function Name: BUTTON_CSDPostMultiScanGanged
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation for ganged sensors.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  BUTTON_Scan() API for a ganged sensor or the
    *  BUTTON_ScanAllWidgets() API in the project with ganged sensors.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  BUTTON_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(BUTTON_CSDPostMultiScanGanged)
    {
        #ifdef BUTTON_ENTRY_CALLBACK
            BUTTON_EntryCallback();
        #endif /* BUTTON_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(BUTTON_INTR_PTR, BUTTON_INTR_SET_MASK);

        #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
            if ((BUTTON_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(BUTTON_RESULT_VAL1_PTR) &
                                                      BUTTON_RESULT_VAL1_BAD_CONVS_MASK) >>
                                                      BUTTON_RESULT_VAL1_BAD_CONVS_SHIFT)) &&
                                                      (0u < BUTTON_badConversionsNum))
            {
                /* Decrement bad conversions number */
                BUTTON_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(BUTTON_SEQ_START_PTR, BUTTON_SEQ_START_AZ0_SKIP_MASK |
                                                             BUTTON_SEQ_START_AZ1_SKIP_MASK |
                                                             BUTTON_SEQ_START_START_MASK);
            }
            else
            {
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

            BUTTON_SsCSDPostScan();

            #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
                if (BUTTON_FREQ_CHANNEL_2 == BUTTON_scanFreqIndex)
            #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */
            {
                BUTTON_SsCSDDisconnectSnsExt((uint32)BUTTON_widgetIndex, (uint32)BUTTON_sensorIndex);
            }

            #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
                if (BUTTON_FREQ_CHANNEL_2 > BUTTON_scanFreqIndex)
                {
                     /* Scan the next channel */
                    BUTTON_SsNextFrequencyScan();
                }
                else
                {
                    /* All channels are scanned. Set IMO to zero channel */
                    BUTTON_scanFreqIndex = BUTTON_FREQ_CHANNEL_0;

                    #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
                        BUTTON_SsChangeImoFreq(BUTTON_FREQ_CHANNEL_0);
                    #else
                        BUTTON_SsChangeClkFreq(BUTTON_FREQ_CHANNEL_0);
                    #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

                     /* Scan the next sensor */
                    BUTTON_SsCSDInitNextScan();
                }
            #else
                 /* Scan the next sensor */
                BUTTON_SsCSDInitNextScan();
            #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */
        #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
            }
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

        #ifdef BUTTON_EXIT_CALLBACK
            BUTTON_ExitCallback();
        #endif /* BUTTON_EXIT_CALLBACK */
    }
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN) */

#else

    /* Third-generation HW block part */

    /*******************************************************************************
    * Function Name: BUTTON_CSDPostSingleScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the single-sensor scanning implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  BUTTON_CSDScanExt() function.
    *
    *  The following tasks are performed for Third-generation HW block:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The following tasks are performed for Fourth-generation HW block:
    *    1. Check if the raw data is not noisy.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Configure and start the scan for the next frequency if the
    *      multi-frequency is enabled.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The ISR handler changes the IMO and initializes scanning for the next frequency
    *  channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user code
    *  from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(BUTTON_CSDPostSingleScan)
    {
        #ifdef BUTTON_ENTRY_CALLBACK
            BUTTON_EntryCallback();
        #endif /* BUTTON_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(BUTTON_INTR_PTR, BUTTON_INTR_SET_MASK);

        /* Read Rawdata */
        BUTTON_SsCSDPostScan();

        #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
            if (BUTTON_FREQ_CHANNEL_2 > BUTTON_scanFreqIndex)
            {
                /* Connect Vref Buffer to AMUX bus. Third-generation HW block is enabled */
                CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG_CSD_EN);

                BUTTON_SsNextFrequencyScan();
            }
            else
            {
                BUTTON_scanFreqIndex = BUTTON_FREQ_CHANNEL_0;

                #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
                    BUTTON_SsChangeImoFreq(BUTTON_FREQ_CHANNEL_0);
                #else
                    BUTTON_SsChangeClkFreq(BUTTON_FREQ_CHANNEL_0);
                #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

                #if (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN)
                    /* Disable Third-generation HW block. Connect Vref Buffer to AMUX bus */
                    #if ((BUTTON_CSH_PRECHARGE_IO_BUF == BUTTON_CSD_CSH_PRECHARGE_SRC) &&\
                         (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN))
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CMOD_PRECHARGE_CONFIG);
                    #else
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG);
                    #endif
                #else
                    /* Connect Vref Buffer to AMUX bus. Third-generation HW block is enabled */
                    CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG_CSD_EN);
                #endif

                /* Update Scan Counter */
                BUTTON_dsRam.scanCounter++;

                /* Sensor is totally scanned. Reset BUSY flag */
                BUTTON_dsRam.status &= ~(BUTTON_SW_STS_BUSY | BUTTON_WDGT_SW_STS_BUSY);
            }
        #else
            {
                #if (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN)
                    /* Disable Third-generation HW block. Connect Vref Buffer to AMUX bus */
                    #if ((BUTTON_CSH_PRECHARGE_IO_BUF == BUTTON_CSD_CSH_PRECHARGE_SRC) &&\
                         (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN))
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CMOD_PRECHARGE_CONFIG);
                    #else
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG);
                    #endif
                #else
                    /* Connect Vref Buffer to AMUX bus. Third-generation HW block is enabled */
                    CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG_CSD_EN);
                #endif

                /* Update Scan Counter */
                BUTTON_dsRam.scanCounter++;

                /* Sensor is totally scanned. Reset BUSY flag */
                BUTTON_dsRam.status &= ~(BUTTON_SW_STS_BUSY | BUTTON_WDGT_SW_STS_BUSY);
            }
        #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */

        #ifdef BUTTON_EXIT_CALLBACK
            BUTTON_ExitCallback();
        #endif /* BUTTON_EXIT_CALLBACK */
    }


    /*******************************************************************************
    * Function Name: BUTTON_CSDPostMultiScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  BUTTON_Scan() or BUTTON_ScanAllWidgets() APIs.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  BUTTON_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(BUTTON_CSDPostMultiScan)
    {
        /* Declare and initialize ptr to sensor IO structure to appropriate address        */
        BUTTON_FLASH_IO_STRUCT const *curSnsIOPtr = (BUTTON_FLASH_IO_STRUCT const *)
                                                          BUTTON_dsFlash.wdgtArray[BUTTON_widgetIndex].ptr2SnsFlash
                                                          + BUTTON_sensorIndex;

        #ifdef BUTTON_ENTRY_CALLBACK
            BUTTON_EntryCallback();
        #endif /* BUTTON_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(BUTTON_INTR_PTR, BUTTON_INTR_SET_MASK);

         /* Read Rawdata */
        BUTTON_SsCSDPostScan();

        /* Connect Vref Buffer to AMUX bus */
        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG_CSD_EN);

        #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
            /* Disable sensor when all frequency channels are scanned */
            if (BUTTON_FREQ_CHANNEL_2 == BUTTON_scanFreqIndex)
            {
                /* Disable sensor */
                BUTTON_CSDDisconnectSns(curSnsIOPtr);
            }
        #else
            /* Disable sensor */
            BUTTON_CSDDisconnectSns(curSnsIOPtr);
        #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */

        #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
            if (BUTTON_FREQ_CHANNEL_2 > BUTTON_scanFreqIndex)
            {
                 /* Scan the next channel */
                BUTTON_SsNextFrequencyScan();
            }
            else
            {
                 /* All channels are scanned. Set IMO to zero channel */
                BUTTON_scanFreqIndex = BUTTON_FREQ_CHANNEL_0;

                #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
                    BUTTON_SsChangeImoFreq(BUTTON_FREQ_CHANNEL_0);
                #else
                    BUTTON_SsChangeClkFreq(BUTTON_FREQ_CHANNEL_0);
                #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

                 /* Scan the next sensor */
                BUTTON_SsCSDInitNextScan();
            }
        #else
            /* Scan the next sensor */
            BUTTON_SsCSDInitNextScan();
        #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */

        #ifdef BUTTON_EXIT_CALLBACK
            BUTTON_ExitCallback();
        #endif /* BUTTON_EXIT_CALLBACK */
    }


    #if (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN)
    /*******************************************************************************
    * Function Name: BUTTON_CSDPostMultiScanGanged
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation for ganged sensors.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  BUTTON_Scan() API for a ganged sensor or the
    *  BUTTON_ScanAllWidgets() API in the project with ganged sensors.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  BUTTON_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(BUTTON_CSDPostMultiScanGanged)
    {
        #ifdef BUTTON_ENTRY_CALLBACK
            BUTTON_EntryCallback();
        #endif /* BUTTON_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(BUTTON_INTR_PTR, BUTTON_INTR_SET_MASK);

         /* Read Rawdata */
        BUTTON_SsCSDPostScan();

        /* Connect Vref Buffer to AMUX bus */
        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG_CSD_EN);

        #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
            if (BUTTON_FREQ_CHANNEL_2 == BUTTON_scanFreqIndex)
            {
                BUTTON_SsCSDDisconnectSnsExt((uint32)BUTTON_widgetIndex, (uint32)BUTTON_sensorIndex);
            }
        #else
            BUTTON_SsCSDDisconnectSnsExt((uint32)BUTTON_widgetIndex, (uint32)BUTTON_sensorIndex);
        #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */

        #if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
            if (BUTTON_FREQ_CHANNEL_2 > BUTTON_scanFreqIndex)
            {
                 /* Scan the next channel */
                BUTTON_SsNextFrequencyScan();
            }
            else
            {
                /* All channels are scanned. Set IMO to zero channel */
                BUTTON_scanFreqIndex = BUTTON_FREQ_CHANNEL_0;

                #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
                    BUTTON_SsChangeImoFreq(BUTTON_FREQ_CHANNEL_0);
                #else
                    BUTTON_SsChangeClkFreq(BUTTON_FREQ_CHANNEL_0);
                #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

                 /* Scan the next sensor */
                BUTTON_SsCSDInitNextScan();
            }
        #else
             /* Scan the next sensor */
            BUTTON_SsCSDInitNextScan();
        #endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */

        #ifdef BUTTON_EXIT_CALLBACK
            BUTTON_ExitCallback();
        #endif /* BUTTON_EXIT_CALLBACK */
    }
    #endif /* (BUTTON_ENABLE == BUTTON_CSD_GANGED_SNS_EN) */

#endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */

#endif /* ((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN)) */

/** \}
 * \endcond */


#if ((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN))

/*******************************************************************************
* Function Name: BUTTON_SsCSDPostScan
****************************************************************************//**
*
* \brief
*   This function reads rawdata and releases required HW resources after scan.
*
* \details
*   This function performs following tasks after scan:
*   - Reads SlotResult from Raw Counter;
*   - Inits bad Conversions number;
*   - Disconnects Vrefhi from AMUBUF positive input;
*   - Disconnects AMUBUF output from CSDBUSB with sych PHI2+HSCMP;
*   - Opens HCBV and HCBG switches.
*
*******************************************************************************/
static void BUTTON_SsCSDPostScan(void)
{
    #if (BUTTON_ENABLE == BUTTON_CSDV2)

        uint32 tmpRawData;
        uint32 tmpMaxCount;
        BUTTON_RAM_WD_BASE_STRUCT const *ptrWdgt = (BUTTON_RAM_WD_BASE_STRUCT *)
                                            BUTTON_dsFlash.wdgtArray[BUTTON_widgetIndex].ptr2WdgtRam;

        /* Read SlotResult from Raw Counter */
        tmpRawData = BUTTON_RESULT_VAL1_VALUE_MASK & CY_GET_REG32(BUTTON_COUNTER_PTR);

        tmpMaxCount = ((1uL << ptrWdgt->resolution) - 1uL);
        if(tmpRawData < tmpMaxCount)
        {
            BUTTON_curRamSnsPtr->raw[BUTTON_scanFreqIndex] = LO16(tmpRawData);
        }
        else
        {
            BUTTON_curRamSnsPtr->raw[BUTTON_scanFreqIndex] = LO16(tmpMaxCount);
        }

        #if (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN)
            /* Init bad Conversions number */
            BUTTON_badConversionsNum = BUTTON_BAD_CONVERSIONS_NUM;
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_NOISE_METRIC_EN) */

        #if (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN)
            /* Open HCBV and HCBG switches */
            CY_SET_REG32(BUTTON_SW_SHIELD_SEL_PTR, BUTTON_SW_SHIELD_SEL_SW_HCBV_STATIC_OPEN |
                                                             BUTTON_SW_SHIELD_SEL_SW_HCBG_STATIC_OPEN);
        #endif /* (BUTTON_ENABLE == BUTTON_CSD_SHIELD_EN) */

    #else

        /* Read SlotResult from Raw Counter */
       BUTTON_curRamSnsPtr->raw[BUTTON_scanFreqIndex] = (uint16)CY_GET_REG32(BUTTON_COUNTER_PTR);

    #endif /* (BUTTON_ENABLE == BUTTON_CSDV2) */
}


/*******************************************************************************
* Function Name: BUTTON_SsCSDInitNextScan
****************************************************************************//**
*
* \brief
*   This function initializes the next sensor scan.
*
* \details
*   The function increments the sensor index, updates sense clock for matrix
*   or touchpad widgets only, sets up Compensation IDAC, enables the sensor and
*   scans it. When all the sensors are scanned it continues to set up the next widget
*   until all the widgets are scanned. The CSD block is disabled when all the widgets are
*   scanned.
*
*******************************************************************************/
static void BUTTON_SsCSDInitNextScan(void)
{
    /* Declare and initialize ptr to widget and sensor structures to appropriate address */
    #if (((BUTTON_ENABLE == BUTTON_CSD_IDAC_COMP_EN) || \
          (BUTTON_CSD_MATRIX_WIDGET_EN || BUTTON_CSD_TOUCHPAD_WIDGET_EN)) || \
          (((BUTTON_DISABLE == BUTTON_CSD_COMMON_SNS_CLK_EN) && \
          (BUTTON_ENABLE == BUTTON_CSDV2) && \
          (BUTTON_CSD_MATRIX_WIDGET_EN || BUTTON_CSD_TOUCHPAD_WIDGET_EN))))
        BUTTON_RAM_WD_BASE_STRUCT *ptrWdgt = (BUTTON_RAM_WD_BASE_STRUCT *)
                                                        BUTTON_dsFlash.wdgtArray[BUTTON_widgetIndex].ptr2WdgtRam;
    #endif

    /* Check if all the sensors are scanned in widget */
    if (((uint8)BUTTON_dsFlash.wdgtArray[(BUTTON_widgetIndex)].totalNumSns - 1u) > BUTTON_sensorIndex)
    {
        /* Increment sensor index to configure next sensor in widget */
        BUTTON_sensorIndex++;

        /* Update global pointer to BUTTON_RAM_SNS_STRUCT to current sensor  */
        BUTTON_curRamSnsPtr = (BUTTON_RAM_SNS_STRUCT *)
                                                  BUTTON_dsFlash.wdgtArray[BUTTON_widgetIndex].ptr2SnsRam
                                                  + BUTTON_sensorIndex;

        /* Configure clock divider to row or column */
        #if ((BUTTON_DISABLE == BUTTON_CSD_COMMON_SNS_CLK_EN) && \
             (BUTTON_CSD_MATRIX_WIDGET_EN || BUTTON_CSD_TOUCHPAD_WIDGET_EN))
            BUTTON_SsCSDConfigClock();

            #if (BUTTON_ENABLE == BUTTON_CSDV2)
                 /* Set up scanning resolution */
                BUTTON_SsCSDCalculateScanDuration(ptrWdgt);
            #endif
        #endif

        /* Setup Compensation IDAC for next sensor in widget */
        #if ((BUTTON_ENABLE == BUTTON_CSD_IDAC_COMP_EN) || \
             (BUTTON_CSD_MATRIX_WIDGET_EN || BUTTON_CSD_TOUCHPAD_WIDGET_EN))
            BUTTON_SsCSDSetUpIdacs(ptrWdgt);
        #endif /* ((BUTTON_ENABLE == BUTTON_CSD_IDAC_COMP_EN) || \
                   (BUTTON_CSD_MATRIX_WIDGET_EN || BUTTON_CSD_TOUCHPAD_WIDGET_EN)) */

        /* Enable sensor */
        BUTTON_SsCSDConnectSensorExt((uint32)BUTTON_widgetIndex, (uint32)BUTTON_sensorIndex);

        /* Proceed scanning */
        BUTTON_SsCSDStartSample();
    }
    /* Call scan next widget API if requested, if not, complete the scan  */
    else
    {
        BUTTON_sensorIndex = 0u;

        /* Current widget is totally scanned. Reset WIDGET BUSY flag */
        BUTTON_dsRam.status &= ~BUTTON_WDGT_SW_STS_BUSY;

        /* Check if all the widgets have been scanned */
        if (BUTTON_ENABLE == BUTTON_requestScanAllWidget)
        {
            /* Configure and begin scanning next widget */
            BUTTON_SsPostAllWidgetsScan();
        }
        else
        {
            #if (BUTTON_ENABLE == BUTTON_BLOCK_OFF_AFTER_SCAN_EN)
                #if (BUTTON_ENABLE == BUTTON_CSDV2)
                    /* Disable the CSD block */
                    CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd);
                #else
                    /* Disable the CSD block. Connect Vref Buffer to AMUX bus */
                    #if ((BUTTON_CSH_PRECHARGE_IO_BUF == BUTTON_CSD_CSH_PRECHARGE_SRC) &&\
                         (BUTTON_ENABLE == BUTTON_CSD_SHIELD_TANK_EN))
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CMOD_PRECHARGE_CONFIG);
                    #else
                        CY_SET_REG32(BUTTON_CONFIG_PTR, BUTTON_configCsd | BUTTON_CTANK_PRECHARGE_CONFIG);
                    #endif
                #endif
            #endif

            /* All widgets are totally scanned. Reset BUSY flag */
            BUTTON_dsRam.status &= ~BUTTON_SW_STS_BUSY;

            /* Update scan Counter */
            BUTTON_dsRam.scanCounter++;
        }
    }
}

#if (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN)
    /*******************************************************************************
    * Function Name: BUTTON_SsNextFrequencyScan
    ****************************************************************************//**
    *
    * \brief
    *   This function scans the sensor on the next frequency channel.
    *
    * \details
    *   The function increments the frequency channel, changes IMO and initializes
    *   the scanning process of the same sensor.
    *
    *******************************************************************************/
    static void BUTTON_SsNextFrequencyScan(void)
    {
        BUTTON_RAM_WD_BASE_STRUCT const *ptrWdgt = (BUTTON_RAM_WD_BASE_STRUCT *)
                                                        BUTTON_dsFlash.wdgtArray[BUTTON_widgetIndex].ptr2WdgtRam;

        BUTTON_scanFreqIndex++;

        /* Set Immunity */
        #if (BUTTON_MFS_IMO == BUTTON_MFS_METHOD)
            BUTTON_SsChangeImoFreq((uint32)BUTTON_scanFreqIndex);
        #else
            BUTTON_SsChangeClkFreq((uint32)BUTTON_scanFreqIndex);
        #endif /* (BUTTON_MFS_IMO == BUTTON_MFS_METHOD) */

        /* Update IDAC registers */
        BUTTON_SsCSDSetUpIdacs(ptrWdgt);

        /* Proceed scanning */
        BUTTON_SsCSDStartSample();
    }
#endif /* (BUTTON_ENABLE == BUTTON_MULTI_FREQ_SCAN_EN) */

#endif /* ((BUTTON_ENABLE == BUTTON_CSD_EN) || (BUTTON_ENABLE == BUTTON_CSD_CSX_EN)) */


/* [] END OF FILE */
