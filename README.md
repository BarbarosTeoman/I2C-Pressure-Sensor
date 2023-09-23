# Pressure Sensor Data Processing

## Overview
This project involves the integration of a pressure sensor with a microcontroller to collect and process pressure data. The processed data includes pressure in pascals (Pa) and corresponding frequency values. The code is written in C and designed to run on a microcontroller, which communicates with the pressure sensor via I2C and incorporates touch sensing capabilities using the CapSense library.

## Project Components
1. **Microcontroller**: The code is intended for a microcontroller with I2C and CapSense capabilities.

2. **Pressure Sensor**: The code assumes the presence of a pressure sensor that communicates with the microcontroller over the I2C protocol.

3. **CapSense**: The project utilizes the CapSense library to interface with touch-sensitive buttons, enabling user interactions such as resetting the data.

## Code Explanation

### I2C Initialization
The I2C communication with the pressure sensor is initialized using the `I2C_Start()` function.

### CapSense Initialization
The CapSense library is initialized, and baseline values for touch sensing are initialized using `CapSense_InitializeAllBaselines()`.

### Main Loop
- The main loop continuously checks the status of the CapSense touch sensors and I2C communication.
- If the I2C communication with the pressure sensor is successful, pressure data is read and converted to pascals (Pa) based on a predefined formula.
- The program then calculates the corresponding frequency based on the pressure range the data falls into (region1, region2, region3, or region4).
- If a certain number of samples (SAMPLE_AMOUNT) are collected, the code calculates the average pressure (avgPa) and average frequency (avgFrequency) and resets the counters.
- If the CapSense button is active (indicating user interaction), the sample counters and averages are reset.
- CapSense baselines are updated, and widgets are scanned to detect touch interactions.

### Constants and Slopes
- The code defines constants such as SLAVE_ADDRESS, pressure data regions (REGION_1_END, REGION_2_END, etc.), and the number of samples to average (SAMPLE_AMOUNT).
- Slope values are precalculated for each pressure region to determine frequency based on pressure readings.

### User Interaction
- The code allows for user interaction through the CapSense touch buttons. Pressing the button with ID `CapSense_BUTTON0_WDGT_ID` resets the sample counters and averages.

## Note
- This code assumes specific hardware configurations and sensor characteristics. Ensure that these match your setup before deploying the code.
- Further development may involve integrating this code with additional features or communication protocols as required.

## Date
- Date of last modification

## License
- This code is provided under the MIT License, which can be found in the LICENSE file.

## Contact
- For inquiries or support, please contact "btkkteoman@icloud.com".
