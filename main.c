#include "project.h"
#include "stdbool.h"

#define SLAVE_ADDRESS 0x27
#define REGION_1_END 752
#define REGION_2_END 1000
#define REGION_3_END 1504
#define REGION_4_END 3000
#define SAMPLE_AMOUNT 20000

uint8_t pressureData1, pressureData2;
uint16_t data16Bit;
uint32 start, readByte1, readByte2, stop;
int i;
int m = 0;

float kPaData, paPressure;
float sum, avgPa, frequencySum, avgFrequency;

float under752Slope = (625 - 589.4) / (0 - REGION_1_END);
float under1000Slope = (589.4 - 578) / (REGION_1_END - REGION_2_END);
float under1504Slope = (578 - 556.3) / (REGION_2_END - REGION_3_END);
float under3000Slope = (556.3 - 503.1) / (REGION_3_END - REGION_4_END);
float frequency;

float region1(float pressure) { return 625 + under752Slope * pressure; }
float region2(float pressure) { return 589.4 + under1000Slope * (pressure - REGION_1_END); }
float region3(float pressure) { return 578 + under1504Slope * (pressure - REGION_2_END); }
float region4(float pressure) { return 556.3 + under3000Slope * (pressure - REGION_3_END); }

bool pause = false;

uint32 busy = 1;

int main(void)
{
    CyGlobalIntEnable;

    pressureData1 = pressureData2 = i = 0;
    sum = avgPa = frequencySum = avgFrequency = 0;
    
    I2C_Start();

    CapSense_Start();
    CapSense_InitializeAllBaselines();

    for (;;)
    {

        busy = CapSense_IsBusy();

        start = I2C_I2CMasterSendStart(SLAVE_ADDRESS, I2C_I2C_READ_XFER_MODE, 1u);

        if (start == I2C_I2C_MSTR_NO_ERROR)
        {
            readByte1 = I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA, &pressureData1, 1u);

            if (readByte1 == I2C_I2C_MSTR_NO_ERROR)
            {
                readByte2 = I2C_I2CMasterReadByte(I2C_I2C_NAK_DATA, &pressureData2, 1u);

                if (readByte2 == I2C_I2C_MSTR_NO_ERROR)
                {
                    data16Bit = ((uint16_t)pressureData1 << 8) | (uint16_t)pressureData2;

                    paPressure = ((float)data16Bit - 820 + 18) * 6000 / 14744;

                    if (paPressure < REGION_1_END)
                    {
                        frequency = region1(paPressure);
                    }
                    else if (paPressure < REGION_2_END)
                    {
                        frequency = region2(paPressure);
                    }
                    else if (paPressure < REGION_3_END)
                    {
                        frequency = region3(paPressure);
                    }
                    else
                    {
                        frequency = region4(paPressure);
                    }

                    if (i < SAMPLE_AMOUNT)
                    {
                        sum += paPressure;
                        frequencySum += frequency;

                        i++;
                    }
                    else if (i == SAMPLE_AMOUNT)
                    {
                        avgPa = sum / SAMPLE_AMOUNT;
                        avgFrequency = frequencySum / SAMPLE_AMOUNT;

                        sum = frequencySum = 0;

                        i++;
                    }
                }
            }
        }

        stop = I2C_I2CMasterSendStop(0u);

        if (!busy)
        {

            CapSense_ProcessAllWidgets();

            if (CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
            {
                i = avgPa = avgFrequency = 0;
            }

            CapSense_UpdateAllBaselines();
            CapSense_ScanAllWidgets();
        }
    }
}