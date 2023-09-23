/***************************************************************************//**
* \file BUTTON_Structure.h
* \version 7.0
*
* \brief
*   This file provides the top level declarations of the Component data structure.
*   Also, the file declares the high-level and low-level APIs for data access.
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

#if !defined(CY_SENSE_BUTTON_STRUCTURE_H)
#define CY_SENSE_BUTTON_STRUCTURE_H

#include <cytypes.h>
#include "cyfitter.h"
#include "BUTTON_Configuration.h"
#if (BUTTON_CSD_SS_DIS != BUTTON_CSD_AUTOTUNE)
    #include "BUTTON_SmartSense_LL.h"
#endif

#if (BUTTON_ENABLE == BUTTON_ALP_FILTER_EN)
    #include "BUTTON_AlpFilter_LL.h"
#endif

#if ((BUTTON_ENABLE == BUTTON_GES_GLOBAL_EN) ||\
     (BUTTON_ENABLE == BUTTON_BALLISTIC_MULTIPLIER_EN))
    #include "BUTTON_TMG.h"
#endif

#if (BUTTON_ENABLE == BUTTON_CENTROID_5X5_CSD_EN)
    #include "BUTTON_AdvancedCentroid_LL.h"
#endif

#if (BUTTON_ENABLE == BUTTON_POS_ADAPTIVE_IIR_FILTER_EN)
    #include "BUTTON_AdaptiveFilter_LL.h"
#endif

/*******************************************************************************
* Constant Definitions
*******************************************************************************/

/* Defines size of Sensor Status Register in Data Structure */
#define BUTTON_SNS_STS_TYPE               uint8

/* No touch condition for slider position report */
#define BUTTON_SLIDER_NO_TOUCH            (0xFFFFu)
#define BUTTON_TOUCHPAD_NO_TOUCH          (0xFFFFFFFFLu)

#define BUTTON_SLIDER_POS_NONE            (0xFFFFu)
#define BUTTON_TOUCHPAD_POS_NONE          (0xFFFFu)

#define BUTTON_MATRIX_POS_NONE            (0xFFu)
#define BUTTON_MATRIX_POS_MULTI           (0xFEu)

#define BUTTON_PROX_STS_MASK              (3Lu)
#define BUTTON_PROX_STS_OFFSET(proxId)    ((proxId) << 1u)

#define BUTTON_MATRIX_BUTTONS_TOUCHED     (0x80000000Lu)

/*******************************************************************************
* Enumeration types definition
*******************************************************************************/

/***************************************************************************//**
* \brief Defines widget types
*******************************************************************************/
typedef enum
{
    BUTTON_WD_BUTTON_E        = 0x01u,
    BUTTON_WD_LINEAR_SLIDER_E = 0x02u,
    BUTTON_WD_RADIAL_SLIDER_E = 0x03u,
    BUTTON_WD_MATRIX_BUTTON_E = 0x04u,
    BUTTON_WD_TOUCHPAD_E      = 0x05u,
    BUTTON_WD_PROXIMITY_E     = 0x06u,
    BUTTON_WD_ENCODERDIAL_E   = 0x07u
} BUTTON_WD_TYPE_ENUM;


/***************************************************************************//**
* \brief Defines sensing methods types
*******************************************************************************/
typedef enum
{
    BUTTON_UNDEFINED_E            = 0x00u,
    BUTTON_SENSE_METHOD_CSD_E     = 0x01u,
    BUTTON_SENSE_METHOD_CSX_E     = 0x02u,
    BUTTON_SENSE_METHOD_BIST_E    = 0x03u,
    BUTTON_SENSE_METHOD_ISX_E     = 0x04u,
} BUTTON_SENSE_METHOD_ENUM;

/***************************************************************************//**
* \brief Defines electrode types
*******************************************************************************/
typedef enum
{
    BUTTON_ELTD_TYPE_SELF_E   = 0x01u,
    BUTTON_ELTD_TYPE_MUT_TX_E = 0x02u,
    BUTTON_ELTD_TYPE_MUT_RX_E = 0x03u
} BUTTON_ELTD_TYPE_ENUM;


/***************************************************************************//**
* \brief Built-in Self-test measurement functions return status enumeration type
*******************************************************************************/
typedef enum
{
  BUTTON_TST_MEASUREMENT_SUCCESS = 0x00uL,
/* function complete successfully, a measurement result is valid */
  BUTTON_TST_MEASUREMENT_BAD_PARAM = 0x01uL,
/* any of input parameters (widgetID, electrodeId, shieldId, integrationCapId) is invalid or any of pointers
   are null, function incomplete, a measurement result is invalid */
  BUTTON_TST_MEASUREMENT_BAD_DATA = 0x02uL,
/* a measurement result is invalid */
  BUTTON_TST_MEASUREMENT_ERROR = 0x03uL,
/* some fault occurred during the measurement, for instance a sensor short, a measurement result is invalid */
  BUTTON_TST_MEASUREMENT_LOW_LIMIT = 0x04uL,
/* Raw count is equal to a minimum value (FilterDelay * NumberOfConversions). A measurement result is invalid.
   A sensor or Cmod should be shorted to VDD */
  BUTTON_TST_MEASUREMENT_HIGH_LIMIT = 0x05uL,
/* Raw count is above 45% of the maximum value (2^Resolution – 1). A measurement result is invalid.
   A possible root cause: sensor or Cmod are shorted to GND */
  BUTTON_TST_MEASUREMENT_OVERFLOW = 0x06uL,
/* Raw count is above the maximum value (2^Resolution – 1). A measurement result is invalid.
   A possible root cause: a sensor or Cmod are shorted to GND */
  BUTTON_TST_MEASUREMENT_TIMEOUT = 0x07uL,
/* Any of measurement scan is not complete. A measurement result is invalid.
   A possible root cause: BIST measurement is not configured properly (CSD HW block is not configured properly: PeriClk, SnsClk, Vref,
   IDAC Gain/Code are not valid. ISR is not configured properly */
}BUTTON_TST_MEASUREMENT_STATUS_ENUM;


/**
* \cond SECTION_STRUCTURES
* \addtogroup group_structures
* \{
*/
/*******************************************************************************
* Declares RAM structures for all used widgets
*******************************************************************************/

/***************************************************************************//**
* \brief Declares common widget RAM parameters
*******************************************************************************/
typedef struct
{
    /**
     *  Provides scan resolution or number of sub-conversions.
     */
    uint16 resolution;

    /**
     *  Widget Finger Threshold.
     */
    BUTTON_THRESHOLD_TYPE fingerTh;

    /**
     *  Widget Noise Threshold.
     */
    uint8  noiseTh;

    /**
     *  Widget Negative Noise Threshold.
     */
    uint8  nNoiseTh;

    /**
     *  Widget Hysteresis for the signal crossing finger or touch/proximity 
     *  threshold.
     */
    uint8  hysteresis;

    /**
     *  Widget Debounce for the signal above the finger or touch/proximity 
     *  threshold. OFF to ON.
     */
    uint8  onDebounce;

    /**
     *  The widget low baseline reset count. Specifies the number of 
     *  samples the sensor has to be below the Negative Noise 
     *  Threshold to trigger a baseline reset.
     */
    BUTTON_LOW_BSLN_RST_TYPE lowBslnRst;

    /**
     *  Sets the current of the modulation IDAC for the widgets. 
     *  For the CSD Touchpad and Matrix Button widgets, sets 
     *  the current of the modulation IDAC for the column sensors.
     */
    uint8  idacMod [BUTTON_NUM_SCAN_FREQS];

    /**
     *  The index of the IDAC gain in the IDAC gain table structure for the widgets.
     */
    uint8  idacGainIndex;

    /**
     *  Specifies the sense clock divider. Present only if individual 
     *  clock dividers are enabled. Specifies the sense clock divider 
     *  for the Column sensors for the Matrix Buttons and Touchpad 
     *  widgets. Sets Tx clock divider for CSX Widgets.
     */
    uint16 snsClk;

    /**
     *  Register for internal use
     */
    uint8  snsClkSource;

    /**
     *  Widget Finger capacitance parameter. Available only if the 
     *  SmartSense is enabled. Not used for the CSX/ISX Widgets.
     */
    uint16 fingerCap;

    /**
     *  The 75% of signal per user-defined finger capacitance
     */
    uint16 sigPFC;
} BUTTON_RAM_WD_BASE_STRUCT;

/***************************************************************************//**
* \brief Declares RAM parameters for the CSD Button
*******************************************************************************/
typedef struct
{
    /**
     *  Provides scan resolution or number of sub-conversions.
     */
    uint16 resolution;

    /**
     *  Widget Finger Threshold.
     */
    BUTTON_THRESHOLD_TYPE fingerTh;

    /**
     *  Widget Noise Threshold.
     */
    uint8  noiseTh;

    /**
     *  Widget Negative Noise Threshold.
     */
    uint8  nNoiseTh;

    /**
     *  Widget Hysteresis for the signal crossing finger or touch/proximity 
     *  threshold.
     */
    uint8  hysteresis;

    /**
     *  Widget Debounce for the signal above the finger or touch/proximity 
     *  threshold. OFF to ON.
     */
    uint8  onDebounce;

    /**
     *  The widget low baseline reset count. Specifies the number of 
     *  samples the sensor has to be below the Negative Noise 
     *  Threshold to trigger a baseline reset.
     */
    BUTTON_LOW_BSLN_RST_TYPE lowBslnRst;

    /**
     *  Sets the current of the modulation IDAC for the widgets. 
     *  For the CSD Touchpad and Matrix Button widgets, sets 
     *  the current of the modulation IDAC for the column sensors.
     */
    uint8  idacMod [BUTTON_NUM_SCAN_FREQS];

    /**
     *  The index of the IDAC gain in the IDAC gain table structure for the widgets.
     */
    uint8  idacGainIndex;

    /**
     *  Specifies the sense clock divider. Present only if individual 
     *  clock dividers are enabled. Specifies the sense clock divider 
     *  for the Column sensors for the Matrix Buttons and Touchpad 
     *  widgets. Sets Tx clock divider for CSX Widgets.
     */
    uint16 snsClk;

    /**
     *  Register for internal use
     */
    uint8  snsClkSource;

    /**
     *  Widget Finger capacitance parameter. Available only if the 
     *  SmartSense is enabled. Not used for the CSX/ISX Widgets.
     */
    uint16 fingerCap;

    /**
     *  The 75% of signal per user-defined finger capacitance
     */
    uint16 sigPFC;
} BUTTON_RAM_WD_BUTTON_STRUCT;


/***************************************************************************//**
* \brief Declares RAM structure with all defined widgets
*******************************************************************************/
typedef struct
{
    /**
     *  Button0 widget RAM structure
     */
    BUTTON_RAM_WD_BUTTON_STRUCT button0;
} BUTTON_RAM_WD_LIST_STRUCT;


/***************************************************************************//**
* \brief Declares RAM structure for sensors
*******************************************************************************/
typedef struct
{
    /**
     *  The sensor raw counts.
     */
    uint16 raw [BUTTON_NUM_SCAN_FREQS];

    /**
     *  The sensor baseline.
     */
    uint16 bsln [BUTTON_NUM_SCAN_FREQS];

    /**
     *  For the bucket baseline algorithm holds the bucket state, 
     *  For the IIR baseline keeps LSB of the baseline value.
     */
    uint8  bslnExt [BUTTON_NUM_SCAN_FREQS];

    /**
     *  Sensor differences.
     */
    BUTTON_THRESHOLD_TYPE diff;

    /**
     *  The baseline reset counter for the low baseline reset function.
     */
    BUTTON_LOW_BSLN_RST_TYPE negBslnRstCnt [BUTTON_NUM_SCAN_FREQS];

    /**
     *  The compensation IDAC value or the balancing IDAC value.
     */
    uint8  idacComp [BUTTON_NUM_SCAN_FREQS];
} BUTTON_RAM_SNS_STRUCT;


/***************************************************************************//**
* \brief Declares RAM structure with all defined sensors
*******************************************************************************/
typedef struct
{
    /**
     *  Button0 sensors RAM structures array
     */
    BUTTON_RAM_SNS_STRUCT button0 [BUTTON_BUTTON0_NUM_SENSORS];
} BUTTON_RAM_SNS_LIST_STRUCT;


/***************************************************************************//**
* \brief Declares the top-level RAM Data Structure
*******************************************************************************/
typedef struct
{
    /**
     *  16-bit CRC calculated by the customizer for the component 
     *  configuration. Used by the Tuner application to identify if 
     *  the FW corresponds to the specific user configuration.
     */
    uint16 configId;

    /**
     *  Used by the Tuner application to identify device-specific configuration.
     */
    uint16 deviceId;

    /**
     *  Used by the Tuner application to identify the system clock frequency.
     */
    uint16 hwClock;

    /**
     *  Tuner Command Register. Used for the communication 
     *  between the Tuner GUI and the component.
     */
    uint16 tunerCmd;

    /**
     *  This counter gets incremented after each scan.
     */
    uint16 scanCounter;

    /**
     *  Status information: Current Widget, Scan active, Error code.
     */
    volatile uint32 status;

    /**
     *  The bitmask that sets which Widgets are enabled and 
     *  scanned, each bit corresponds to one widget.
     */
    uint32 wdgtEnable [BUTTON_WDGT_STATUS_WORDS];

    /**
     *  The bitmask that reports activated Widgets (widgets that 
     *  detect a touch signal above the threshold), each bit 
     *  corresponds to one widget.
     */
    uint32 wdgtStatus [BUTTON_WDGT_STATUS_WORDS];

    /**
     *  For Buttons, Sliders, Matrix Buttons and CSD Touchpad each bit 
     *  reports status of the individual sensor of the widget: 1 - active 
     *  (above the finger threshold); 0 - inactive; For the CSD Touchpad 
     *  and CSD Matrix Buttons, the column sensors occupy the least 
     *  significant bits. For the Proximity widget, each sensor uses two 
     *  bits with the following meaning: 00 - Not active; 01 - Proximity 
     *  detected (signal above finger threshold); 11 - A finger touch 
     *  detected (signal above the touch threshold); For the CSX Touchpad 
     *  Widget, this register provides a number of the detected touches. 
     *  The array size is equal to the total number of widgets. The size of 
     *  the array element depends on the max number of sensors per 
     *  widget used in the current design. It could be 1, 2 or 4 bytes.
     */
    BUTTON_SNS_STS_TYPE snsStatus [BUTTON_TOTAL_WIDGETS];

    /**
     *  The configuration register for global parameters of the 
     *  SENSE_HW0 block.
     */
    uint16 csd0Config;

    /**
     *  The modulator clock divider for the CSD widgets.
     */
    uint8  modCsdClk;

    /**
     *  RAM Widget Objects.
     */
    BUTTON_RAM_WD_LIST_STRUCT wdgtList;

    /**
     *  RAM Sensor Objects.
     */
    BUTTON_RAM_SNS_LIST_STRUCT snsList;

    /**
     *  The selected widget ID.
     */
    uint8  snrTestWidgetId;

    /**
     *  The selected sensor ID.
     */
    uint8  snrTestSensorId;

    /**
     *  The scan counter.
     */
    uint16 snrTestScanCounter;

    /**
     *  The sensor raw counts.
     */
    uint16 snrTestRawCount [BUTTON_NUM_SCAN_FREQS];

    /**
     *  The inactive sensor connection state for the CSD sensors.
     */
    uint8  scanCsdISC;

    /**
     *  The current inactive sensor connection state for the sensors.
     */
    uint8  scanCurrentISC;
} BUTTON_RAM_STRUCT;


/***************************************************************************//**
* \brief Declares the Flash IO object
*******************************************************************************/
typedef struct
{
    /**
     *  Pointer to the HSIOM configuration register of the IO.
     */
    reg32  * hsiomPtr;

    /**
     *  Pointer to the port configuration register of the IO.
     */
    reg32  * pcPtr;

    /**
     *  Pointer to the port data register of the IO.
     */
    reg32  * drPtr;

    /**
     *  Pointer to the pin state data register of the IO.
     */
    reg32  * psPtr;

    /**
     *  IO mask in the HSIOM configuration register.
     */
    uint32   hsiomMask;

    /**
     *  IO mask in the DR and PS registers.
     */
    uint32   mask;

    /**
     *  Position of the IO configuration bits in the HSIOM register.
     */
    uint8    hsiomShift;

    /**
     *  Position of the IO configuration bits in the DR and PS registers.
     */
    uint8    drShift;

    /**
     *  Position of the IO configuration bits in the PC register.
     */
    uint8    shift;
} BUTTON_FLASH_IO_STRUCT;


/***************************************************************************//**
* \brief Declares the Flash Electrode object
*******************************************************************************/
typedef struct
{
    /**
     *  Index of the first IO in the Flash IO Object Array.
     */
    uint16 firstPinId;

    /**
     *  Total number of IOs in this sensor.
     */
    uint8  numPins;

    /**
     * Sensor type:
     * \if SECTION_C_DS
     * - ELTD_TYPE_SELF_E - CSD sensor;
     * - ELTD_TYPE_MUT_TX_E - CSX Tx;
     * - ELTD_TYPE_MUT_RX_E - CSX Rx;
     * \endif
     * \if SECTION_I_DS
     * - ELTD_TYPE_MUT_TX_E - ISX Lx sensor;
     * - ELTD_TYPE_MUT_RX_E - ISX Rx sensor;
     * \endif
     */
    uint8  type;
} BUTTON_FLASH_SNS_STRUCT;


/***************************************************************************//**
* \brief Declares the structure with all Flash electrode objects
*******************************************************************************/
typedef struct
{
    /**
     *  No ganged sensors available
     */
    uint8 notUsed;
} BUTTON_FLASH_SNS_LIST_STRUCT;




/***************************************************************************//**
* \brief Declares Flash widget object
*******************************************************************************/
typedef struct
{
    /**
     *  Points to the array of the FLASH Sensor Objects or FLASH IO 
     *  Objects that belong to this widget. Sensing block uses this 
     *  pointer to access and configure IOs for the scanning. Bit #2 in 
     *  WD_STATIC_CONFIG field indicates the type of array: 
     *  1 - Sensor Object; 0 - IO Object.
     */
    void const * ptr2SnsFlash;

    /**
     *  Points to the Widget Object in RAM. Sensing block uses it to 
     *  access scan parameters. Processing uses it to access threshold 
     *  and widget specific data.
     */
    void * ptr2WdgtRam;

    /**
     *  Points to the array of Sensor Objects in RAM. The sensing and 
     *  processing blocks use it to access the scan data.
     */
    BUTTON_RAM_SNS_STRUCT * ptr2SnsRam;

    /**
     *  Points to the array of the Filter History Objects in RAM that 
     *  belongs to this widget.
     */
    void * ptr2FltrHistory;

    /**
     *  Points to the array of the debounce counters. The size of the 
     *  debounce counter is 8 bits. These arrays are not part of the 
     *  data structure.
     */
    uint8 * ptr2DebounceArr;

    /**
     *  Miscellaneous configuration flags.
     */
    uint32 staticConfig;

    /**
     *  The total number of sensors. 
     *  For CSD widgets: WD_NUM_ROWS + WD_NUM_COLS. 
     *  For CSX widgets: WD_NUM_ROWS * WD_NUM_COLS.
     */
    uint16 totalNumSns;

    /**
     *  Specifies one of the following widget types: 
     *  WD_BUTTON_E, WD_LINEAR_SLIDER_E, WD_RADIAL_SLIDER_E, 
     *  WD_MATRIX_BUTTON_E, WD_TOUCHPAD_E, WD_PROXIMITY_E
     */
    uint8  wdgtType;

    /**
     *  For CSD Button and Proximity Widgets, the number of sensors. 
     *  For CSD Slider Widget, the number of segments. 
     *  For CSD Touchpad and Matrix Button, the number of the 
     *  column sensors. 
     *  For CSX Button, Touchpad and Matrix Button, the number 
     *  of the Rx electrodes.
     */
    uint8  numCols;

    /**
     *  The pointer to the array with the sensor noise envelope data. 
     *  Set to the valid value only for the CSD widgets. For the CSX 
     *  widgets this pointer is set to NULL. The pointed array is not 
     *  part of the data structure.
     */
    SMARTSENSE_CSD_NOISE_ENVELOPE_STRUCT * ptr2NoiseEnvlp;
} BUTTON_FLASH_WD_STRUCT;


/***************************************************************************//**
* \brief Declares top-level Flash Data Structure
*******************************************************************************/
typedef struct
{
    /**
     *  Array of flash widget objects
     */
    BUTTON_FLASH_WD_STRUCT wdgtArray[BUTTON_TOTAL_WIDGETS];
} BUTTON_FLASH_STRUCT;

/***************************************************************************//**
* \brief Declares the Flash IO structure for Shield electrodes
*******************************************************************************/
typedef struct
{
    /**
     *  The pointer to the HSIOM configuration register of the IO.
     */
    reg32  * hsiomPtr;

    /**
     *  The pointer to the port configuration register of the IO.
     */
    reg32  * pcPtr;
    /**
     *  The pointer to the port data register of the IO.
     */
    reg32  * drPtr;

    /**
     *  The IO mask in the HSIOM configuration register.
     */
    uint32   hsiomMask;

    /**
     *  The position of the IO configuration bits in the HSIOM register.
     */
    uint8    hsiomShift;

    /**
     *  The position of the IO configuration bits in the DR and PS registers.
     */
    uint8    drShift;

    /**
     *  The position of the IO configuration bits in the PC register.
     */
    uint8    shift;
} BUTTON_SHIELD_IO_STRUCT;



#if (BUTTON_ENABLE == BUTTON_TST_BSLN_RAW_OUT_RANGE_EN)
    /***************************************************************************//**
    * \brief Defines the structure for test of baseline and raw count limits which
    * will be determined by user for every sensor grounding on the
    * manufacturing specific data
    *******************************************************************************/
    typedef struct
    {
        /**
         *  Upper limit of a sensor baseline.
         */
        uint16 bslnHiLim;
        /**
         *  Lower limit of a sensor baseline.
         */
        uint16 bslnLoLim;
        /**
         *  Upper limit of a sensor raw count.
         */
        uint16 rawHiLim;
        /**
         *  Lower limit of a sensor raw count.
         */
        uint16 rawLoLim;
    } BUTTON_BSLN_RAW_RANGE_STRUCT;
#endif /* (BUTTON_ENABLE == BUTTON_TST_BSLN_RAW_OUT_RANGE_EN) */

/** \}
* \endcond */


/***************************************************************************//**
* Declares Dual-channel scan structure
*******************************************************************************/
typedef struct
{
    uint8 csd0WidgetId;
    uint8 csd0SensorId;
    uint8 csd1WidgetId;
    uint8 csd1SensorId;
} BUTTON_SCAN_SLOT_STRUCT;


/***************************************************************************//**
* Declares Filter module structures
*******************************************************************************/

/***************************************************************************//**
* \brief Declares filter channel structure for regular sensors
*******************************************************************************/
typedef struct
{
    uint16 notUsed;
} BUTTON_REGULAR_FLTR_CHANNEL_STRUCT;

/***************************************************************************//**
* \brief Declares filter structure for regular sensors
*******************************************************************************/
typedef struct
{
    /**
     *  Array of BUTTON_REGULAR_FLTR_CHANNEL_STRUCT for each available scan frequency
     */
    BUTTON_REGULAR_FLTR_CHANNEL_STRUCT regularChannel[BUTTON_NUM_SCAN_FREQS];
} BUTTON_REGULAR_FLTR_STRUCT;

/***************************************************************************//**
* \brief Declares union for filter structure variants
*******************************************************************************/
typedef union
{
    /**
     *  Pointer to void type
     */
    void *ptr;

    /**
     *  Pointer to BUTTON_REGULAR_FLTR_STRUCT type
     */
    BUTTON_REGULAR_FLTR_STRUCT *ptrRegular;
} BUTTON_PTR_FILTER_VARIANT;


#if (0u != BUTTON_POSITION_FILTER_EN)
typedef struct
{
    #if (0u != BUTTON_POS_MEDIAN_FILTER_EN)
        uint16 posMedianZ1;
        uint16 posMedianZ2;
    #endif /* #if (0u != BUTTON_POS_MEDIAN_FILTER_EN) */

    #if (0u != BUTTON_POS_IIR_FILTER_EN)
        uint16 posIIR;
    #endif /* #if (0u != BUTTON_POS_IIR_FILTER_EN) */

    #if (0u != BUTTON_POS_ADAPTIVE_IIR_FILTER_EN)
        uint16 posAIIR;
    #endif /* (0u != BUTTON_POS_ADAPTIVE_IIR_FILTER_EN) */

    #if (0u != BUTTON_POS_AVERAGE_FILTER_EN)
        uint16 posAverage;
    #endif /* #if (0u != BUTTON_POS_AVERAGE_FILTER_EN) */

    #if (0u != BUTTON_POS_JITTER_FILTER_EN)
        uint16 posJitter;
    #endif /* #if (0u != BUTTON_POS_JITTER_FILTER_EN) */

    #if (0u != BUTTON_POS_ADAPTIVE_IIR_FILTER_EN)
        uint8 posAIIRCoeff;
    #endif /* (0u != BUTTON_POS_ADAPTIVE_IIR_FILTER_EN) */
} BUTTON_SLIDER_POS_HISTORY_STRUCT;

typedef struct
{
    BUTTON_SLIDER_POS_HISTORY_STRUCT xPos;
    BUTTON_SLIDER_POS_HISTORY_STRUCT yPos;
} BUTTON_TOUCHPAD_POS_HISTORY_STRUCT;
#endif /* (0u != BUTTON_POSITION_FILTER_EN) */

/*******************************************************************************
* API Constants
*******************************************************************************/

/*******************************************************************************
* Defines shift/mask values for data structure register bit fields
*******************************************************************************/

/* CSD0_CONFIG bit fields */

/* Bit 0: The multi-frequency scan enable/disable at runtime. This bit 
 * is functional only if the multi-frequency scan functionality 
 * has been activated in general component configuration. */
#define BUTTON_MULTI_FREQ_EN_SIZE         (0x00000001Lu)
#define BUTTON_MULTI_FREQ_EN_SHIFT        (0u)
#define BUTTON_MULTI_FREQ_EN_MASK         (BUTTON_MULTI_FREQ_EN_SIZE << BUTTON_MULTI_FREQ_EN_SHIFT)

/* Bit 1: The sensor auto-reset is enabled */
#define BUTTON_SNS_ARST_SIZE              (0x00000001Lu)
#define BUTTON_SNS_ARST_SHIFT             (1u)
#define BUTTON_SNS_ARST_MASK              (BUTTON_SNS_ARST_SIZE << BUTTON_SNS_ARST_SHIFT)

/* Bits 2-3: The IDAC range setting (4x/8x) */
#define BUTTON_IDAC_RANGE_SIZE            (0x00000003Lu)
#define BUTTON_IDAC_RANGE_SHIFT           (2u)
#define BUTTON_IDAC_RANGE_MASK            (BUTTON_IDAC_RANGE_SIZE << BUTTON_IDAC_RANGE_SHIFT)

/* Bits 4-7: Reserved */

/* Bit 8: The shield electrode signal Enable/Disable */
#define BUTTON_SHLD_EN_SIZE               (0x00000001Lu)
#define BUTTON_SHLD_EN_SHIFT              (8u)
#define BUTTON_SHLD_EN_MASK               (BUTTON_SHLD_EN_SIZE << BUTTON_SHLD_EN_SHIFT)

/* Bits 9-10: Selects the delay by which the csd_shield is delayed relative 
 * to csd_sense */
#define BUTTON_SHLD_DLY_SIZE              (0x00000003Lu)
#define BUTTON_SHLD_DLY_SHIFT             (9u)
#define BUTTON_SHLD_DLY_MASK              (BUTTON_SHLD_DLY_SIZE << BUTTON_SHLD_DLY_SHIFT)


/* STATUS bit fields */

/* Bits 0-6: The widget currently selected for SENSE_HW 0 */
#define BUTTON_STATUS_WDGT0_SIZE          (0x0000007FLu)
#define BUTTON_STATUS_WDGT0_SHIFT         (0u)
#define BUTTON_STATUS_WDGT0_MASK          (BUTTON_STATUS_WDGT0_SIZE << BUTTON_STATUS_WDGT0_SHIFT)

/* Bit 7: The SENSE_HW0 block status */
#define BUTTON_STATUS_CSD0_SIZE           (0x00000001Lu)
#define BUTTON_STATUS_CSD0_SHIFT          (7u)
#define BUTTON_STATUS_CSD0_MASK           (BUTTON_STATUS_CSD0_SIZE << BUTTON_STATUS_CSD0_SHIFT)

/* Bits 8-14: The widget currently selected for SENSE_HW1 */
#define BUTTON_STATUS_WDGT1_SIZE          (0x0000007FLu)
#define BUTTON_STATUS_WDGT1_SHIFT         (8u)
#define BUTTON_STATUS_WDGT1_MASK          (BUTTON_STATUS_WDGT1_SIZE << BUTTON_STATUS_WDGT1_SHIFT)

/* Bit 15: The SENSE_HW1 block status */
#define BUTTON_STATUS_CSD1_SIZE           (0x00000001Lu)
#define BUTTON_STATUS_CSD1_SHIFT          (15u)
#define BUTTON_STATUS_CSD1_MASK           (BUTTON_STATUS_CSD1_SIZE << BUTTON_STATUS_CSD1_SHIFT)

/* Bit 16: The WIDGET BUSY flag status for SENSE_HW0 */
#define BUTTON_STATUS_WDGT0_BUSY_SIZE     (0x00000001Lu)
#define BUTTON_STATUS_WDGT0_BUSY_SHIFT    (16u)
#define BUTTON_STATUS_WDGT0_BUSY_MASK     (BUTTON_STATUS_WDGT0_BUSY_SIZE << BUTTON_STATUS_WDGT0_BUSY_SHIFT)

/* Bit 17: The WIDGET BUSY flag status for SENSE_HW1 */
#define BUTTON_STATUS_WDGT1_BUSY_SIZE     (0x00000001Lu)
#define BUTTON_STATUS_WDGT1_BUSY_SHIFT    (17u)
#define BUTTON_STATUS_WDGT1_BUSY_MASK     (BUTTON_STATUS_WDGT1_BUSY_SIZE << BUTTON_STATUS_WDGT1_BUSY_SHIFT)

/* Bits 18-23: Reserved */

/* Bits 24-31: Component error code */
#define BUTTON_STATUS_ERR_SIZE            (0x000000FFLu)
#define BUTTON_STATUS_ERR_SHIFT           (24u)
#define BUTTON_STATUS_ERR_MASK            (BUTTON_STATUS_ERR_SIZE << BUTTON_STATUS_ERR_SHIFT)


/* WD_STATIC_CONFIG bit fields */

/* Bit 0: The sense/Tx frequency selection: 0 - Common, 1 - Individual 
 * (per widget). */
#define BUTTON_SNS_FREQ_SIZE              (0x00000001Lu)
#define BUTTON_SNS_FREQ_SHIFT             (0u)
#define BUTTON_SNS_FREQ_MASK              (BUTTON_SNS_FREQ_SIZE << BUTTON_SNS_FREQ_SHIFT)

/* Bit 1: Duplexing Enable/Disable for linear sliders. */
#define BUTTON_DIPLEXING_SIZE             (0x00000001Lu)
#define BUTTON_DIPLEXING_SHIFT            (1u)
#define BUTTON_DIPLEXING_MASK             (BUTTON_DIPLEXING_SIZE << BUTTON_DIPLEXING_SHIFT)

/* Bit 2: The widget contains a sensor ganged to other sensors. */
#define BUTTON_GANGED_SNS_SIZE            (0x00000001Lu)
#define BUTTON_GANGED_SNS_SHIFT           (2u)
#define BUTTON_GANGED_SNS_MASK            (BUTTON_GANGED_SNS_SIZE << BUTTON_GANGED_SNS_SHIFT)

/* Bit 3: Some pin from this widget is used by other sensors to make 
 * a ganged sensor. */
#define BUTTON_SHARED_IO_SIZE             (0x00000001Lu)
#define BUTTON_SHARED_IO_SHIFT            (3u)
#define BUTTON_SHARED_IO_MASK             (BUTTON_SHARED_IO_SIZE << BUTTON_SHARED_IO_SHIFT)

/* Bit 4: The centroid position IIR filter Enable/Disable. */
#define BUTTON_POS_IIR_FILTER_SIZE        (0x00000001Lu)
#define BUTTON_POS_IIR_FILTER_SHIFT       (4u)
#define BUTTON_POS_IIR_FILTER_MASK        (BUTTON_POS_IIR_FILTER_SIZE << BUTTON_POS_IIR_FILTER_SHIFT)

/* Bit 5: The centroid position median filter Enable/Disable. */
#define BUTTON_POS_MEDIAN_FILTER_SIZE     (0x00000001Lu)
#define BUTTON_POS_MEDIAN_FILTER_SHIFT    (5u)
#define BUTTON_POS_MEDIAN_FILTER_MASK     (BUTTON_POS_MEDIAN_FILTER_SIZE << BUTTON_POS_MEDIAN_FILTER_SHIFT)

/* Bit 6: The centroid position average filter Enable/Disable. */
#define BUTTON_POS_AVERAGE_FILTER_SIZE    (0x00000001Lu)
#define BUTTON_POS_AVERAGE_FILTER_SHIFT   (6u)
#define BUTTON_POS_AVERAGE_FILTER_MASK    (BUTTON_POS_AVERAGE_FILTER_SIZE << BUTTON_POS_AVERAGE_FILTER_SHIFT)

/* Bit 7: The centroid position jitter filter Enable/Disable. */
#define BUTTON_POS_JITTER_FILTER_SIZE     (0x00000001Lu)
#define BUTTON_POS_JITTER_FILTER_SHIFT    (7u)
#define BUTTON_POS_JITTER_FILTER_MASK     (BUTTON_POS_JITTER_FILTER_SIZE << BUTTON_POS_JITTER_FILTER_SHIFT)

/* Bit 8: The multiphase Tx scan Enable (CSX widgets only). */
#define BUTTON_MULTIPHASE_TX_SIZE         (0x00000001Lu)
#define BUTTON_MULTIPHASE_TX_SHIFT        (8u)
#define BUTTON_MULTIPHASE_TX_MASK         (BUTTON_MULTIPHASE_TX_SIZE << BUTTON_MULTIPHASE_TX_SHIFT)

/* Bit 9: The centroid position adaptive IIR filter Enable/Disable. */
#define BUTTON_AIIR_FILTER_SIZE           (0x00000001Lu)
#define BUTTON_AIIR_FILTER_SHIFT          (9u)
#define BUTTON_AIIR_FILTER_MASK           (BUTTON_AIIR_FILTER_SIZE << BUTTON_AIIR_FILTER_SHIFT)

/* Bit 10: Ballistic multiplier Enable/Disable. */
#define BUTTON_BALLISTIC_SIZE             (0x00000001Lu)
#define BUTTON_BALLISTIC_SHIFT            (10u)
#define BUTTON_BALLISTIC_MASK             (BUTTON_BALLISTIC_SIZE << BUTTON_BALLISTIC_SHIFT)

/* Bit 11: 3x3 centroid Enable/Disable. */
#define BUTTON_CENTROID_3X3_SIZE          (0x00000001Lu)
#define BUTTON_CENTROID_3X3_SHIFT         (11u)
#define BUTTON_CENTROID_3X3_MASK          (BUTTON_CENTROID_3X3_SIZE << BUTTON_CENTROID_3X3_SHIFT)

/* Bit 12: 5x5 centroid Enable/Disable. */
#define BUTTON_CENTROID_5X5_SIZE          (0x00000001Lu)
#define BUTTON_CENTROID_5X5_SHIFT         (12u)
#define BUTTON_CENTROID_5X5_MASK          (BUTTON_CENTROID_5X5_SIZE << BUTTON_CENTROID_5X5_SHIFT)

/* Bit 13: Edge correction Enable/Disable. */
#define BUTTON_EDGE_CORRECTION_SIZE       (0x00000001Lu)
#define BUTTON_EDGE_CORRECTION_SHIFT      (13u)
#define BUTTON_EDGE_CORRECTION_MASK       (BUTTON_EDGE_CORRECTION_SIZE << BUTTON_EDGE_CORRECTION_SHIFT)

/* Bit 14: Two finger detection Enable/Disable. */
#define BUTTON_TWO_FINGER_DETECTION_SIZE  (0x00000001Lu)
#define BUTTON_TWO_FINGER_DETECTION_SHIFT (14u)
#define BUTTON_TWO_FINGER_DETECTION_MASK  (BUTTON_TWO_FINGER_DETECTION_SIZE << BUTTON_TWO_FINGER_DETECTION_SHIFT)


/*******************************************************************************
* Defines Data Structure Macro helpers
*******************************************************************************/

/*******************************************************************************
* Determines the widget specific word in the wdgtStatus array by widget ID
*******************************************************************************/
/* Divide by 32 - size of the wdgtStatus word in bits to find the word index */
#define BUTTON_GET_WDGT_STATUS_INDEX(wdgtId)  ((wdgtId) >> 5u)

/*******************************************************************************
* Determines the widget bitmask in wdgtStatus word by widget ID
*******************************************************************************/
/* Take the least 5 bits of widget id to find the bit number */
#define BUTTON_GET_WDGT_STATUS_MASK(wdgtId)   (1Lu << ((wdgtId) & 0x1FLu))

/*******************************************************************************
* Determines the sense method of the widget
*******************************************************************************/
/*
* If there are multiple sensing methods, get senseMethod from flash structure,
* otherwise, hardcode it for speed.
*/
#if (BUTTON_MANY_WIDGET_METHODS_EN)
    #define BUTTON_GET_SENSE_METHOD(ptrFlash) ((BUTTON_SENSE_METHOD_ENUM)(ptrFlash)->senseMethod)
#elif (0u != BUTTON_TOTAL_CSD_WIDGETS)
    #define BUTTON_GET_SENSE_METHOD(ptrFlash) BUTTON_SENSE_METHOD_CSD_E
#elif (0u != BUTTON_TOTAL_CSX_WIDGETS)
    #define BUTTON_GET_SENSE_METHOD(ptrFlash) BUTTON_SENSE_METHOD_CSX_E
#elif (0u != BUTTON_TOTAL_ISX_WIDGETS)
    #define BUTTON_GET_SENSE_METHOD(ptrFlash) BUTTON_SENSE_METHOD_ISX_E
#endif

/*******************************************************************************
* Gets the widget type
*******************************************************************************/
#define BUTTON_GET_WIDGET_TYPE(ptrFlashWdgt)  ((BUTTON_WD_TYPE_ENUM)((ptrFlashWdgt)->wdgtType))

/*******************************************************************************
* Gets the number of the widget's sensors
*******************************************************************************/
#define BUTTON_GET_SENSOR_COUNT(widgetId)         BUTTON_dsFlash.wdgtArray[(widgetId)].totalNumSns
#define BUTTON_GET_SNS_CNT_BY_PTR(ptrFlashWidget) (ptrFlashWidget)->totalNumSns

/*******************************************************************************
* Increments the pointer to the Regular Filter History Object
*******************************************************************************/
#if (0u != BUTTON_REGULAR_RC_ALP_FILTER_EN)
    #define BUTTON_INC_REG_FLTR_OBJ(fltrVariant)      \
        do {                                                    \
            (fltrVariant).ptrAlp++;                             \
        } while(0)
#elif (0u != BUTTON_REGULAR_RC_FILTER_EN)
    #define BUTTON_INC_REG_FLTR_OBJ(fltrVariant)      \
        do {                                                    \
            (fltrVariant).ptrRegular++;                         \
        } while(0)
#else
    #define BUTTON_INC_REG_FLTR_OBJ(fltrVariant)      \
        do {                                                    \
        } while(0)
#endif

/*******************************************************************************
* Increments the pointer to the Proximity Filter History Object
*******************************************************************************/
#if (0u != BUTTON_PROX_RC_ALP_FILTER_EN)
    #define BUTTON_INC_PROX_FLTR_OBJ(fltrVariant)     \
        do {                                                    \
            (fltrVariant).ptrAlp++;                             \
        } while(0)
#elif (0u != BUTTON_PROX_RC_FILTER_EN)
    #define BUTTON_INC_PROX_FLTR_OBJ(fltrVariant)     \
        do {                                                    \
            (fltrVariant).ptrProx++;                            \
        } while(0)
#else
    #define BUTTON_INC_PROX_FLTR_OBJ(fltrVariant)     \
        do {                                                    \
        } while(0)
#endif

/*******************************************************************************
* Increments the pointer to the Filter History Object Variant
*******************************************************************************/
#define BUTTON_INC_FLTR_OBJ_VARIANT(isProxHistObj, fltrVariant)   \
    do {                                                                    \
        if (0u == (isProxHistObj))                                          \
        {                                                                   \
            BUTTON_INC_REG_FLTR_OBJ(fltrVariant);                 \
        }                                                                   \
        else                                                                \
        {                                                                   \
            BUTTON_INC_PROX_FLTR_OBJ(fltrVariant);                \
        }                                                                   \
    } while(0)

/*******************************************************************************
* Gets a widget status in the global enable register dsRam.wdgtEnable[]
*******************************************************************************/
#if (BUTTON_ENABLE == BUTTON_SELF_TEST_EN)
    #define BUTTON_GET_WIDGET_EN_STATUS(wdId)                                                     \
                 (BUTTON_GET_WDGT_STATUS_MASK(wdId) &                                             \
                  BUTTON_dsRam.wdgtEnable[BUTTON_GET_WDGT_STATUS_INDEX(wdId)] &         \
                  BUTTON_dsRam.wdgtWorking[BUTTON_GET_WDGT_STATUS_INDEX(wdId)])
#else
    #define BUTTON_GET_WIDGET_EN_STATUS(wdId)                    \
                (BUTTON_GET_WDGT_STATUS_MASK(wdId) &             \
                 BUTTON_dsRam.wdgtEnable[BUTTON_GET_WDGT_STATUS_INDEX(wdId)])
#endif /* (BUTTON_ENABLE == BUTTON_SELF_TEST_EN) */

/*******************************************************************************
* Gets a widget active status from the dsRam.wdgtStatus[] register
*******************************************************************************/
#define BUTTON_GET_WIDGET_ACTIVE_STATUS(wdId)                    \
            (BUTTON_GET_WDGT_STATUS_MASK(wdId) &                 \
             BUTTON_dsRam.wdgtStatus[BUTTON_GET_WDGT_STATUS_INDEX(wdId)])

/*******************************************************************************
* Declares Flash and RAM Data Structure variables
*******************************************************************************/
extern BUTTON_RAM_STRUCT            BUTTON_dsRam;
extern const BUTTON_FLASH_STRUCT    BUTTON_dsFlash;
extern const BUTTON_FLASH_IO_STRUCT BUTTON_ioList[BUTTON_TOTAL_ELECTRODES];
extern const BUTTON_RAM_WD_LIST_STRUCT BUTTON_ramWidgetInit;
extern const uint8 BUTTON_ramIdacInit[BUTTON_TOTAL_SENSORS];



extern const BUTTON_SHIELD_IO_STRUCT
    BUTTON_shieldIoList[BUTTON_CSD_TOTAL_SHIELD_COUNT];



/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* High-level API
*******************************************************************************/

/**
* \cond (SECTION_C_HIGH_LEVEL || SECTION_I_HIGH_LEVEL)
* \addtogroup group_c_high_level
* \{
*/

uint32 BUTTON_IsAnyWidgetActive(void);
uint32 BUTTON_IsWidgetActive(uint32 widgetId);
uint32 BUTTON_IsSensorActive(uint32 widgetId, uint32 sensorId);

#if (0u != BUTTON_PROXIMITY_WIDGET_EN)
    uint32 BUTTON_IsProximitySensorActive(uint32 widgetId, uint32 proxId);
#endif /* #if (0u != BUTTON_PROXIMITY_WIDGET_EN) */

/** \}
* \endcond
* \cond (SECTION_C_HIGH_LEVEL)
* \addtogroup group_c_high_level
* \{
*/
#if (0u != BUTTON_MATRIX_WIDGET_EN)
    uint32 BUTTON_IsMatrixButtonsActive(uint32 widgetId);
#endif /* #if (0u != BUTTON_MATRIX_WIDGET_EN) */

#if (0u != BUTTON_SLIDER_WIDGET_EN)
    uint32 BUTTON_GetCentroidPos(uint32 widgetId);
#endif /* #if (0u != BUTTON_SLIDER_WIDGET_EN) */

#if (0u != BUTTON_TOUCHPAD_WIDGET_EN)
    uint32 BUTTON_GetXYCoordinates(uint32 widgetId);
#endif /* #if (0u != BUTTON_TOUCHPAD_WIDGET_EN) */

/** \}
* \endcond */

/*******************************************************************************
* Low level API
*******************************************************************************/

/**
* \cond (SECTION_C_LOW_LEVEL || SECTION_I_LOW_LEVEL)
* \addtogroup group_c_low_level
* \{
*/

cystatus BUTTON_GetParam(uint32 paramId, uint32 *value);
cystatus BUTTON_SetParam(uint32 paramId, uint32 value);

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



void BUTTON_DsInitialize(void);
#if (0u != BUTTON_ADC_EN)
    void BUTTON_AdcDsInitialize(void);
#endif /* (0u != BUTTON_ADC_EN) */

/** \}
* \endcond */

#endif /* End CY_SENSE_BUTTON_STRUCTURE_H */


/* [] END OF FILE */
