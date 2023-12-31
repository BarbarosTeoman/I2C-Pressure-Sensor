/*******************************************************************************
* File Name: cyfittergnu.inc
* 
* PSoC Creator  4.4
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2020 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

.ifndef INCLUDED_CYFITTERGNU_INC
.set INCLUDED_CYFITTERGNU_INC, 1
.include "cydevicegnu_trm.inc"

/* I2C */
.set I2C_SCB__CTRL, CYREG_SCB0_CTRL
.set I2C_SCB__EZ_DATA0, CYREG_SCB0_EZ_DATA0
.set I2C_SCB__EZ_DATA1, CYREG_SCB0_EZ_DATA1
.set I2C_SCB__EZ_DATA10, CYREG_SCB0_EZ_DATA10
.set I2C_SCB__EZ_DATA11, CYREG_SCB0_EZ_DATA11
.set I2C_SCB__EZ_DATA12, CYREG_SCB0_EZ_DATA12
.set I2C_SCB__EZ_DATA13, CYREG_SCB0_EZ_DATA13
.set I2C_SCB__EZ_DATA14, CYREG_SCB0_EZ_DATA14
.set I2C_SCB__EZ_DATA15, CYREG_SCB0_EZ_DATA15
.set I2C_SCB__EZ_DATA16, CYREG_SCB0_EZ_DATA16
.set I2C_SCB__EZ_DATA17, CYREG_SCB0_EZ_DATA17
.set I2C_SCB__EZ_DATA18, CYREG_SCB0_EZ_DATA18
.set I2C_SCB__EZ_DATA19, CYREG_SCB0_EZ_DATA19
.set I2C_SCB__EZ_DATA2, CYREG_SCB0_EZ_DATA2
.set I2C_SCB__EZ_DATA20, CYREG_SCB0_EZ_DATA20
.set I2C_SCB__EZ_DATA21, CYREG_SCB0_EZ_DATA21
.set I2C_SCB__EZ_DATA22, CYREG_SCB0_EZ_DATA22
.set I2C_SCB__EZ_DATA23, CYREG_SCB0_EZ_DATA23
.set I2C_SCB__EZ_DATA24, CYREG_SCB0_EZ_DATA24
.set I2C_SCB__EZ_DATA25, CYREG_SCB0_EZ_DATA25
.set I2C_SCB__EZ_DATA26, CYREG_SCB0_EZ_DATA26
.set I2C_SCB__EZ_DATA27, CYREG_SCB0_EZ_DATA27
.set I2C_SCB__EZ_DATA28, CYREG_SCB0_EZ_DATA28
.set I2C_SCB__EZ_DATA29, CYREG_SCB0_EZ_DATA29
.set I2C_SCB__EZ_DATA3, CYREG_SCB0_EZ_DATA3
.set I2C_SCB__EZ_DATA30, CYREG_SCB0_EZ_DATA30
.set I2C_SCB__EZ_DATA31, CYREG_SCB0_EZ_DATA31
.set I2C_SCB__EZ_DATA4, CYREG_SCB0_EZ_DATA4
.set I2C_SCB__EZ_DATA5, CYREG_SCB0_EZ_DATA5
.set I2C_SCB__EZ_DATA6, CYREG_SCB0_EZ_DATA6
.set I2C_SCB__EZ_DATA7, CYREG_SCB0_EZ_DATA7
.set I2C_SCB__EZ_DATA8, CYREG_SCB0_EZ_DATA8
.set I2C_SCB__EZ_DATA9, CYREG_SCB0_EZ_DATA9
.set I2C_SCB__I2C_CFG, CYREG_SCB0_I2C_CFG
.set I2C_SCB__I2C_CTRL, CYREG_SCB0_I2C_CTRL
.set I2C_SCB__I2C_M_CMD, CYREG_SCB0_I2C_M_CMD
.set I2C_SCB__I2C_S_CMD, CYREG_SCB0_I2C_S_CMD
.set I2C_SCB__I2C_STATUS, CYREG_SCB0_I2C_STATUS
.set I2C_SCB__INTR_CAUSE, CYREG_SCB0_INTR_CAUSE
.set I2C_SCB__INTR_I2C_EC, CYREG_SCB0_INTR_I2C_EC
.set I2C_SCB__INTR_I2C_EC_MASK, CYREG_SCB0_INTR_I2C_EC_MASK
.set I2C_SCB__INTR_I2C_EC_MASKED, CYREG_SCB0_INTR_I2C_EC_MASKED
.set I2C_SCB__INTR_M, CYREG_SCB0_INTR_M
.set I2C_SCB__INTR_M_MASK, CYREG_SCB0_INTR_M_MASK
.set I2C_SCB__INTR_M_MASKED, CYREG_SCB0_INTR_M_MASKED
.set I2C_SCB__INTR_M_SET, CYREG_SCB0_INTR_M_SET
.set I2C_SCB__INTR_RX, CYREG_SCB0_INTR_RX
.set I2C_SCB__INTR_RX_MASK, CYREG_SCB0_INTR_RX_MASK
.set I2C_SCB__INTR_RX_MASKED, CYREG_SCB0_INTR_RX_MASKED
.set I2C_SCB__INTR_RX_SET, CYREG_SCB0_INTR_RX_SET
.set I2C_SCB__INTR_S, CYREG_SCB0_INTR_S
.set I2C_SCB__INTR_S_MASK, CYREG_SCB0_INTR_S_MASK
.set I2C_SCB__INTR_S_MASKED, CYREG_SCB0_INTR_S_MASKED
.set I2C_SCB__INTR_S_SET, CYREG_SCB0_INTR_S_SET
.set I2C_SCB__INTR_SPI_EC, CYREG_SCB0_INTR_SPI_EC
.set I2C_SCB__INTR_SPI_EC_MASK, CYREG_SCB0_INTR_SPI_EC_MASK
.set I2C_SCB__INTR_SPI_EC_MASKED, CYREG_SCB0_INTR_SPI_EC_MASKED
.set I2C_SCB__INTR_TX, CYREG_SCB0_INTR_TX
.set I2C_SCB__INTR_TX_MASK, CYREG_SCB0_INTR_TX_MASK
.set I2C_SCB__INTR_TX_MASKED, CYREG_SCB0_INTR_TX_MASKED
.set I2C_SCB__INTR_TX_SET, CYREG_SCB0_INTR_TX_SET
.set I2C_SCB__RX_CTRL, CYREG_SCB0_RX_CTRL
.set I2C_SCB__RX_FIFO_CTRL, CYREG_SCB0_RX_FIFO_CTRL
.set I2C_SCB__RX_FIFO_RD, CYREG_SCB0_RX_FIFO_RD
.set I2C_SCB__RX_FIFO_RD_SILENT, CYREG_SCB0_RX_FIFO_RD_SILENT
.set I2C_SCB__RX_FIFO_STATUS, CYREG_SCB0_RX_FIFO_STATUS
.set I2C_SCB__RX_MATCH, CYREG_SCB0_RX_MATCH
.set I2C_SCB__SPI_CTRL, CYREG_SCB0_SPI_CTRL
.set I2C_SCB__SPI_STATUS, CYREG_SCB0_SPI_STATUS
.set I2C_SCB__SS0_POSISTION, 0
.set I2C_SCB__SS1_POSISTION, 1
.set I2C_SCB__SS2_POSISTION, 2
.set I2C_SCB__SS3_POSISTION, 3
.set I2C_SCB__STATUS, CYREG_SCB0_STATUS
.set I2C_SCB__TX_CTRL, CYREG_SCB0_TX_CTRL
.set I2C_SCB__TX_FIFO_CTRL, CYREG_SCB0_TX_FIFO_CTRL
.set I2C_SCB__TX_FIFO_STATUS, CYREG_SCB0_TX_FIFO_STATUS
.set I2C_SCB__TX_FIFO_WR, CYREG_SCB0_TX_FIFO_WR
.set I2C_SCB__UART_CTRL, CYREG_SCB0_UART_CTRL
.set I2C_SCB__UART_FLOW_CTRL, CYREG_SCB0_UART_FLOW_CTRL
.set I2C_SCB__UART_RX_CTRL, CYREG_SCB0_UART_RX_CTRL
.set I2C_SCB__UART_RX_STATUS, CYREG_SCB0_UART_RX_STATUS
.set I2C_SCB__UART_TX_CTRL, CYREG_SCB0_UART_TX_CTRL
.set I2C_SCB_IRQ__INTC_CLR_EN_REG, CYREG_CM0P_ICER
.set I2C_SCB_IRQ__INTC_CLR_PD_REG, CYREG_CM0P_ICPR
.set I2C_SCB_IRQ__INTC_MASK, 0x80
.set I2C_SCB_IRQ__INTC_NUMBER, 7
.set I2C_SCB_IRQ__INTC_PRIOR_MASK, 0xC0000000
.set I2C_SCB_IRQ__INTC_PRIOR_NUM, 3
.set I2C_SCB_IRQ__INTC_PRIOR_REG, CYREG_CM0P_IPR1
.set I2C_SCB_IRQ__INTC_SET_EN_REG, CYREG_CM0P_ISER
.set I2C_SCB_IRQ__INTC_SET_PD_REG, CYREG_CM0P_ISPR
.set I2C_SCBCLK__CTRL_REGISTER, CYREG_PERI_PCLK_CTL0
.set I2C_SCBCLK__DIV_ID, 0x00000041
.set I2C_SCBCLK__DIV_REGISTER, CYREG_PERI_DIV_16_CTL1
.set I2C_SCBCLK__PA_DIV_ID, 0x000000FF
.set I2C_scl__0__DR, CYREG_GPIO_PRT1_DR
.set I2C_scl__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_scl__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_scl__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_scl__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set I2C_scl__0__HSIOM_GPIO, 0
.set I2C_scl__0__HSIOM_I2C, 14
.set I2C_scl__0__HSIOM_I2C_SCL, 14
.set I2C_scl__0__HSIOM_MASK, 0x0000000F
.set I2C_scl__0__HSIOM_SHIFT, 0
.set I2C_scl__0__HSIOM_SPI, 15
.set I2C_scl__0__HSIOM_SPI_MOSI, 15
.set I2C_scl__0__HSIOM_UART, 9
.set I2C_scl__0__HSIOM_UART_RX, 9
.set I2C_scl__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_scl__0__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_scl__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_scl__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_scl__0__MASK, 0x01
.set I2C_scl__0__PC, CYREG_GPIO_PRT1_PC
.set I2C_scl__0__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_scl__0__PORT, 1
.set I2C_scl__0__PS, CYREG_GPIO_PRT1_PS
.set I2C_scl__0__SHIFT, 0
.set I2C_scl__DR, CYREG_GPIO_PRT1_DR
.set I2C_scl__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_scl__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_scl__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_scl__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_scl__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_scl__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_scl__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_scl__MASK, 0x01
.set I2C_scl__PC, CYREG_GPIO_PRT1_PC
.set I2C_scl__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_scl__PORT, 1
.set I2C_scl__PS, CYREG_GPIO_PRT1_PS
.set I2C_scl__SHIFT, 0
.set I2C_sda__0__DR, CYREG_GPIO_PRT1_DR
.set I2C_sda__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_sda__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_sda__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_sda__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set I2C_sda__0__HSIOM_GPIO, 0
.set I2C_sda__0__HSIOM_I2C, 14
.set I2C_sda__0__HSIOM_I2C_SDA, 14
.set I2C_sda__0__HSIOM_MASK, 0x000000F0
.set I2C_sda__0__HSIOM_SHIFT, 4
.set I2C_sda__0__HSIOM_SPI, 15
.set I2C_sda__0__HSIOM_SPI_MISO, 15
.set I2C_sda__0__HSIOM_UART, 9
.set I2C_sda__0__HSIOM_UART_TX, 9
.set I2C_sda__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_sda__0__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_sda__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_sda__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_sda__0__MASK, 0x02
.set I2C_sda__0__PC, CYREG_GPIO_PRT1_PC
.set I2C_sda__0__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_sda__0__PORT, 1
.set I2C_sda__0__PS, CYREG_GPIO_PRT1_PS
.set I2C_sda__0__SHIFT, 1
.set I2C_sda__DR, CYREG_GPIO_PRT1_DR
.set I2C_sda__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_sda__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_sda__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_sda__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_sda__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_sda__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_sda__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_sda__MASK, 0x02
.set I2C_sda__PC, CYREG_GPIO_PRT1_PC
.set I2C_sda__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_sda__PORT, 1
.set I2C_sda__PS, CYREG_GPIO_PRT1_PS
.set I2C_sda__SHIFT, 1

/* Pin_1 */
.set Pin_1__0__DR, CYREG_GPIO_PRT1_DR
.set Pin_1__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set Pin_1__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set Pin_1__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set Pin_1__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set Pin_1__0__HSIOM_MASK, 0x0F000000
.set Pin_1__0__HSIOM_SHIFT, 24
.set Pin_1__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set Pin_1__0__INTR, CYREG_GPIO_PRT1_INTR
.set Pin_1__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set Pin_1__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set Pin_1__0__MASK, 0x40
.set Pin_1__0__PC, CYREG_GPIO_PRT1_PC
.set Pin_1__0__PC2, CYREG_GPIO_PRT1_PC2
.set Pin_1__0__PORT, 1
.set Pin_1__0__PS, CYREG_GPIO_PRT1_PS
.set Pin_1__0__SHIFT, 6
.set Pin_1__DR, CYREG_GPIO_PRT1_DR
.set Pin_1__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set Pin_1__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set Pin_1__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set Pin_1__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set Pin_1__INTR, CYREG_GPIO_PRT1_INTR
.set Pin_1__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set Pin_1__INTSTAT, CYREG_GPIO_PRT1_INTR
.set Pin_1__MASK, 0x40
.set Pin_1__PC, CYREG_GPIO_PRT1_PC
.set Pin_1__PC2, CYREG_GPIO_PRT1_PC2
.set Pin_1__PORT, 1
.set Pin_1__PS, CYREG_GPIO_PRT1_PS
.set Pin_1__SHIFT, 6

/* CapSense */
.set CapSense_Cmod__0__DR, CYREG_GPIO_PRT4_DR
.set CapSense_Cmod__0__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set CapSense_Cmod__0__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set CapSense_Cmod__0__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set CapSense_Cmod__0__HSIOM, CYREG_HSIOM_PORT_SEL4
.set CapSense_Cmod__0__HSIOM_MASK, 0x00000F00
.set CapSense_Cmod__0__HSIOM_SHIFT, 8
.set CapSense_Cmod__0__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Cmod__0__INTR, CYREG_GPIO_PRT4_INTR
.set CapSense_Cmod__0__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Cmod__0__INTSTAT, CYREG_GPIO_PRT4_INTR
.set CapSense_Cmod__0__MASK, 0x04
.set CapSense_Cmod__0__PC, CYREG_GPIO_PRT4_PC
.set CapSense_Cmod__0__PC2, CYREG_GPIO_PRT4_PC2
.set CapSense_Cmod__0__PORT, 4
.set CapSense_Cmod__0__PS, CYREG_GPIO_PRT4_PS
.set CapSense_Cmod__0__SHIFT, 2
.set CapSense_Cmod__Cmod__DR, CYREG_GPIO_PRT4_DR
.set CapSense_Cmod__Cmod__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set CapSense_Cmod__Cmod__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set CapSense_Cmod__Cmod__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set CapSense_Cmod__Cmod__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Cmod__Cmod__INTR, CYREG_GPIO_PRT4_INTR
.set CapSense_Cmod__Cmod__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Cmod__Cmod__INTSTAT, CYREG_GPIO_PRT4_INTR
.set CapSense_Cmod__Cmod__MASK, 0x04
.set CapSense_Cmod__Cmod__PC, CYREG_GPIO_PRT4_PC
.set CapSense_Cmod__Cmod__PC2, CYREG_GPIO_PRT4_PC2
.set CapSense_Cmod__Cmod__PORT, 4
.set CapSense_Cmod__Cmod__PS, CYREG_GPIO_PRT4_PS
.set CapSense_Cmod__Cmod__SHIFT, 2
.set CapSense_Cmod__DR, CYREG_GPIO_PRT4_DR
.set CapSense_Cmod__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set CapSense_Cmod__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set CapSense_Cmod__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set CapSense_Cmod__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Cmod__INTR, CYREG_GPIO_PRT4_INTR
.set CapSense_Cmod__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Cmod__INTSTAT, CYREG_GPIO_PRT4_INTR
.set CapSense_Cmod__MASK, 0x04
.set CapSense_Cmod__PC, CYREG_GPIO_PRT4_PC
.set CapSense_Cmod__PC2, CYREG_GPIO_PRT4_PC2
.set CapSense_Cmod__PORT, 4
.set CapSense_Cmod__PS, CYREG_GPIO_PRT4_PS
.set CapSense_Cmod__SHIFT, 2
.set CapSense_CSD__ADC_CTL, CYREG_CSD_ADC_CTL
.set CapSense_CSD__CMOD_PAD, 1
.set CapSense_CSD__CSD_CONFIG, CYREG_CSD_CONFIG
.set CapSense_CSD__CSD_INTR, CYREG_CSD_INTR
.set CapSense_CSD__CSD_INTR_SET, CYREG_CSD_INTR_SET
.set CapSense_CSD__CSD_NUMBER, 0
.set CapSense_CSD__CSD_STATUS, CYREG_CSD_STATUS
.set CapSense_CSD__CSDCMP, CYREG_CSD_CSDCMP
.set CapSense_CSD__CSH_TANK_PAD, 2
.set CapSense_CSD__CSHIELD_PAD, 4
.set CapSense_CSD__DEDICATED_IO0, CapSense_CSD__CMOD_PAD
.set CapSense_CSD__HSCMP, CYREG_CSD_HSCMP
.set CapSense_CSD__INTR_MASK, CYREG_CSD_INTR_MASK
.set CapSense_CSD__REFGEN, CYREG_CSD_REFGEN
.set CapSense_CSD__RESULT_VAL1, CYREG_CSD_RESULT_VAL1
.set CapSense_CSD__RESULT_VAL2, CYREG_CSD_RESULT_VAL2
.set CapSense_CSD__SENSE_DUTY, CYREG_CSD_SENSE_DUTY
.set CapSense_CSD__SENSE_PERIOD, CYREG_CSD_SENSE_PERIOD
.set CapSense_CSD__SEQ_INIT_CNT, CYREG_CSD_SEQ_INIT_CNT
.set CapSense_CSD__SEQ_NORM_CNT, CYREG_CSD_SEQ_NORM_CNT
.set CapSense_CSD__SEQ_START, CYREG_CSD_SEQ_START
.set CapSense_CSD__SEQ_TIME, CYREG_CSD_SEQ_TIME
.set CapSense_CSD__SW_AMUXBUF_SEL, CYREG_CSD_SW_AMUXBUF_SEL
.set CapSense_CSD__SW_BYP_SEL, CYREG_CSD_SW_BYP_SEL
.set CapSense_CSD__SW_CMP_N_SEL, CYREG_CSD_SW_CMP_N_SEL
.set CapSense_CSD__SW_CMP_P_SEL, CYREG_CSD_SW_CMP_P_SEL
.set CapSense_CSD__SW_DSI_SEL, CYREG_CSD_SW_DSI_SEL
.set CapSense_CSD__SW_FW_MOD_SEL, CYREG_CSD_SW_FW_MOD_SEL
.set CapSense_CSD__SW_FW_TANK_SEL, CYREG_CSD_SW_FW_TANK_SEL
.set CapSense_CSD__SW_HS_N_SEL, CYREG_CSD_SW_HS_N_SEL
.set CapSense_CSD__SW_HS_P_SEL, CYREG_CSD_SW_HS_P_SEL
.set CapSense_CSD__SW_REFGEN_SEL, CYREG_CSD_SW_REFGEN_SEL
.set CapSense_CSD__SW_RES, CYREG_CSD_SW_RES
.set CapSense_CSD__SW_SHIELD_SEL, CYREG_CSD_SW_SHIELD_SEL
.set CapSense_CSD__VREF_EXT_PAD, 8
.set CapSense_IDACComp__CONFIG, CYREG_CSD_CONFIG
.set CapSense_IDACComp__IDAC, CYREG_CSD_IDACB
.set CapSense_IDACComp__POSITION, 1
.set CapSense_IDACMod__CONFIG, CYREG_CSD_CONFIG
.set CapSense_IDACMod__IDAC, CYREG_CSD_IDACA
.set CapSense_IDACMod__POSITION, 0
.set CapSense_ISR__INTC_CLR_EN_REG, CYREG_CM0P_ICER
.set CapSense_ISR__INTC_CLR_PD_REG, CYREG_CM0P_ICPR
.set CapSense_ISR__INTC_MASK, 0x20000
.set CapSense_ISR__INTC_NUMBER, 17
.set CapSense_ISR__INTC_PRIOR_MASK, 0xC000
.set CapSense_ISR__INTC_PRIOR_NUM, 3
.set CapSense_ISR__INTC_PRIOR_REG, CYREG_CM0P_IPR4
.set CapSense_ISR__INTC_SET_EN_REG, CYREG_CM0P_ISER
.set CapSense_ISR__INTC_SET_PD_REG, CYREG_CM0P_ISPR
.set CapSense_ModClk__CTRL_REGISTER, CYREG_PERI_PCLK_CTL5
.set CapSense_ModClk__DIV_ID, 0x00000040
.set CapSense_ModClk__DIV_REGISTER, CYREG_PERI_DIV_16_CTL0
.set CapSense_ModClk__PA_DIV_ID, 0x000000FF
.set CapSense_Shield__0__DR, CYREG_GPIO_PRT4_DR
.set CapSense_Shield__0__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set CapSense_Shield__0__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set CapSense_Shield__0__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set CapSense_Shield__0__HSIOM, CYREG_HSIOM_PORT_SEL4
.set CapSense_Shield__0__HSIOM_MASK, 0x000000F0
.set CapSense_Shield__0__HSIOM_SHIFT, 4
.set CapSense_Shield__0__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Shield__0__INTR, CYREG_GPIO_PRT4_INTR
.set CapSense_Shield__0__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Shield__0__INTSTAT, CYREG_GPIO_PRT4_INTR
.set CapSense_Shield__0__MASK, 0x02
.set CapSense_Shield__0__PC, CYREG_GPIO_PRT4_PC
.set CapSense_Shield__0__PC2, CYREG_GPIO_PRT4_PC2
.set CapSense_Shield__0__PORT, 4
.set CapSense_Shield__0__PS, CYREG_GPIO_PRT4_PS
.set CapSense_Shield__0__SHIFT, 1
.set CapSense_Shield__DR, CYREG_GPIO_PRT4_DR
.set CapSense_Shield__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set CapSense_Shield__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set CapSense_Shield__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set CapSense_Shield__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Shield__INTR, CYREG_GPIO_PRT4_INTR
.set CapSense_Shield__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Shield__INTSTAT, CYREG_GPIO_PRT4_INTR
.set CapSense_Shield__MASK, 0x02
.set CapSense_Shield__PC, CYREG_GPIO_PRT4_PC
.set CapSense_Shield__PC2, CYREG_GPIO_PRT4_PC2
.set CapSense_Shield__PORT, 4
.set CapSense_Shield__PS, CYREG_GPIO_PRT4_PS
.set CapSense_Shield__Shield__DR, CYREG_GPIO_PRT4_DR
.set CapSense_Shield__Shield__DR_CLR, CYREG_GPIO_PRT4_DR_CLR
.set CapSense_Shield__Shield__DR_INV, CYREG_GPIO_PRT4_DR_INV
.set CapSense_Shield__Shield__DR_SET, CYREG_GPIO_PRT4_DR_SET
.set CapSense_Shield__Shield__INTCFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Shield__Shield__INTR, CYREG_GPIO_PRT4_INTR
.set CapSense_Shield__Shield__INTR_CFG, CYREG_GPIO_PRT4_INTR_CFG
.set CapSense_Shield__Shield__INTSTAT, CYREG_GPIO_PRT4_INTR
.set CapSense_Shield__Shield__MASK, 0x02
.set CapSense_Shield__Shield__PC, CYREG_GPIO_PRT4_PC
.set CapSense_Shield__Shield__PC2, CYREG_GPIO_PRT4_PC2
.set CapSense_Shield__Shield__PORT, 4
.set CapSense_Shield__Shield__PS, CYREG_GPIO_PRT4_PS
.set CapSense_Shield__Shield__SHIFT, 1
.set CapSense_Shield__SHIFT, 1
.set CapSense_Sns__0__DR, CYREG_GPIO_PRT0_DR
.set CapSense_Sns__0__DR_CLR, CYREG_GPIO_PRT0_DR_CLR
.set CapSense_Sns__0__DR_INV, CYREG_GPIO_PRT0_DR_INV
.set CapSense_Sns__0__DR_SET, CYREG_GPIO_PRT0_DR_SET
.set CapSense_Sns__0__HSIOM, CYREG_HSIOM_PORT_SEL0
.set CapSense_Sns__0__HSIOM_MASK, 0x000F0000
.set CapSense_Sns__0__HSIOM_SHIFT, 16
.set CapSense_Sns__0__INTCFG, CYREG_GPIO_PRT0_INTR_CFG
.set CapSense_Sns__0__INTR, CYREG_GPIO_PRT0_INTR
.set CapSense_Sns__0__INTR_CFG, CYREG_GPIO_PRT0_INTR_CFG
.set CapSense_Sns__0__INTSTAT, CYREG_GPIO_PRT0_INTR
.set CapSense_Sns__0__MASK, 0x10
.set CapSense_Sns__0__PC, CYREG_GPIO_PRT0_PC
.set CapSense_Sns__0__PC2, CYREG_GPIO_PRT0_PC2
.set CapSense_Sns__0__PORT, 0
.set CapSense_Sns__0__PS, CYREG_GPIO_PRT0_PS
.set CapSense_Sns__0__SHIFT, 4
.set CapSense_Sns__Button0_Sns0__DR, CYREG_GPIO_PRT0_DR
.set CapSense_Sns__Button0_Sns0__DR_CLR, CYREG_GPIO_PRT0_DR_CLR
.set CapSense_Sns__Button0_Sns0__DR_INV, CYREG_GPIO_PRT0_DR_INV
.set CapSense_Sns__Button0_Sns0__DR_SET, CYREG_GPIO_PRT0_DR_SET
.set CapSense_Sns__Button0_Sns0__INTCFG, CYREG_GPIO_PRT0_INTR_CFG
.set CapSense_Sns__Button0_Sns0__INTR, CYREG_GPIO_PRT0_INTR
.set CapSense_Sns__Button0_Sns0__INTR_CFG, CYREG_GPIO_PRT0_INTR_CFG
.set CapSense_Sns__Button0_Sns0__INTSTAT, CYREG_GPIO_PRT0_INTR
.set CapSense_Sns__Button0_Sns0__MASK, 0x10
.set CapSense_Sns__Button0_Sns0__PC, CYREG_GPIO_PRT0_PC
.set CapSense_Sns__Button0_Sns0__PC2, CYREG_GPIO_PRT0_PC2
.set CapSense_Sns__Button0_Sns0__PORT, 0
.set CapSense_Sns__Button0_Sns0__PS, CYREG_GPIO_PRT0_PS
.set CapSense_Sns__Button0_Sns0__SHIFT, 4
.set CapSense_Sns__DR, CYREG_GPIO_PRT0_DR
.set CapSense_Sns__DR_CLR, CYREG_GPIO_PRT0_DR_CLR
.set CapSense_Sns__DR_INV, CYREG_GPIO_PRT0_DR_INV
.set CapSense_Sns__DR_SET, CYREG_GPIO_PRT0_DR_SET
.set CapSense_Sns__INTCFG, CYREG_GPIO_PRT0_INTR_CFG
.set CapSense_Sns__INTR, CYREG_GPIO_PRT0_INTR
.set CapSense_Sns__INTR_CFG, CYREG_GPIO_PRT0_INTR_CFG
.set CapSense_Sns__INTSTAT, CYREG_GPIO_PRT0_INTR
.set CapSense_Sns__MASK, 0x10
.set CapSense_Sns__PC, CYREG_GPIO_PRT0_PC
.set CapSense_Sns__PC2, CYREG_GPIO_PRT0_PC2
.set CapSense_Sns__PORT, 0
.set CapSense_Sns__PS, CYREG_GPIO_PRT0_PS
.set CapSense_Sns__SHIFT, 4

/* Miscellaneous */
.set CYDEV_BCLK__HFCLK__HZ, 24000000
.set CYDEV_BCLK__HFCLK__KHZ, 24000
.set CYDEV_BCLK__HFCLK__MHZ, 24
.set CYDEV_BCLK__SYSCLK__HZ, 24000000
.set CYDEV_BCLK__SYSCLK__KHZ, 24000
.set CYDEV_BCLK__SYSCLK__MHZ, 24
.set CYDEV_CHIP_DIE_LEOPARD, 1
.set CYDEV_CHIP_DIE_PSOC4A, 26
.set CYDEV_CHIP_DIE_PSOC5LP, 2
.set CYDEV_CHIP_DIE_PSOC5TM, 3
.set CYDEV_CHIP_DIE_TMA4, 4
.set CYDEV_CHIP_DIE_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_FM0P, 5
.set CYDEV_CHIP_FAMILY_FM3, 6
.set CYDEV_CHIP_FAMILY_FM4, 7
.set CYDEV_CHIP_FAMILY_PSOC3, 1
.set CYDEV_CHIP_FAMILY_PSOC4, 2
.set CYDEV_CHIP_FAMILY_PSOC5, 3
.set CYDEV_CHIP_FAMILY_PSOC6, 4
.set CYDEV_CHIP_FAMILY_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_USED, CYDEV_CHIP_FAMILY_PSOC4
.set CYDEV_CHIP_JTAG_ID, 0x283A11B8
.set CYDEV_CHIP_MEMBER_3A, 1
.set CYDEV_CHIP_MEMBER_4A, 26
.set CYDEV_CHIP_MEMBER_4AA, 25
.set CYDEV_CHIP_MEMBER_4AB, 30
.set CYDEV_CHIP_MEMBER_4AC, 14
.set CYDEV_CHIP_MEMBER_4AD, 15
.set CYDEV_CHIP_MEMBER_4AE, 16
.set CYDEV_CHIP_MEMBER_4D, 20
.set CYDEV_CHIP_MEMBER_4E, 6
.set CYDEV_CHIP_MEMBER_4F, 27
.set CYDEV_CHIP_MEMBER_4G, 4
.set CYDEV_CHIP_MEMBER_4H, 24
.set CYDEV_CHIP_MEMBER_4I, 32
.set CYDEV_CHIP_MEMBER_4J, 21
.set CYDEV_CHIP_MEMBER_4K, 22
.set CYDEV_CHIP_MEMBER_4L, 31
.set CYDEV_CHIP_MEMBER_4M, 29
.set CYDEV_CHIP_MEMBER_4N, 11
.set CYDEV_CHIP_MEMBER_4O, 8
.set CYDEV_CHIP_MEMBER_4P, 28
.set CYDEV_CHIP_MEMBER_4Q, 17
.set CYDEV_CHIP_MEMBER_4R, 9
.set CYDEV_CHIP_MEMBER_4S, 12
.set CYDEV_CHIP_MEMBER_4T, 10
.set CYDEV_CHIP_MEMBER_4U, 5
.set CYDEV_CHIP_MEMBER_4V, 23
.set CYDEV_CHIP_MEMBER_4W, 13
.set CYDEV_CHIP_MEMBER_4X, 7
.set CYDEV_CHIP_MEMBER_4Y, 18
.set CYDEV_CHIP_MEMBER_4Z, 19
.set CYDEV_CHIP_MEMBER_5A, 3
.set CYDEV_CHIP_MEMBER_5B, 2
.set CYDEV_CHIP_MEMBER_6A, 33
.set CYDEV_CHIP_MEMBER_FM3, 37
.set CYDEV_CHIP_MEMBER_FM4, 38
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1, 34
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2, 35
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3, 36
.set CYDEV_CHIP_MEMBER_UNKNOWN, 0
.set CYDEV_CHIP_MEMBER_USED, CYDEV_CHIP_MEMBER_4AB
.set CYDEV_CHIP_DIE_EXPECT, CYDEV_CHIP_MEMBER_USED
.set CYDEV_CHIP_DIE_ACTUAL, CYDEV_CHIP_DIE_EXPECT
.set CYDEV_CHIP_REV_LEOPARD_ES1, 0
.set CYDEV_CHIP_REV_LEOPARD_ES2, 1
.set CYDEV_CHIP_REV_LEOPARD_ES3, 3
.set CYDEV_CHIP_REV_LEOPARD_PRODUCTION, 3
.set CYDEV_CHIP_REV_PSOC4A_ES0, 17
.set CYDEV_CHIP_REV_PSOC4A_PRODUCTION, 17
.set CYDEV_CHIP_REV_PSOC5LP_ES0, 0
.set CYDEV_CHIP_REV_PSOC5LP_PRODUCTION, 0
.set CYDEV_CHIP_REV_PSOC5TM_ES0, 0
.set CYDEV_CHIP_REV_PSOC5TM_ES1, 1
.set CYDEV_CHIP_REV_PSOC5TM_PRODUCTION, 1
.set CYDEV_CHIP_REV_TMA4_ES, 17
.set CYDEV_CHIP_REV_TMA4_ES2, 33
.set CYDEV_CHIP_REV_TMA4_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_3A_ES1, 0
.set CYDEV_CHIP_REVISION_3A_ES2, 1
.set CYDEV_CHIP_REVISION_3A_ES3, 3
.set CYDEV_CHIP_REVISION_3A_PRODUCTION, 3
.set CYDEV_CHIP_REVISION_4A_ES0, 17
.set CYDEV_CHIP_REVISION_4A_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_4AA_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AB_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AC_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AD_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4AE_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4D_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD, 0
.set CYDEV_CHIP_REVISION_4E_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION_256K, 0
.set CYDEV_CHIP_REVISION_4G_ES, 17
.set CYDEV_CHIP_REVISION_4G_ES2, 33
.set CYDEV_CHIP_REVISION_4G_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_4H_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4I_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4J_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4K_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4L_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4M_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4N_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4O_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4P_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Q_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4R_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4S_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4T_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4U_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4V_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4W_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4X_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Y_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Z_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_5A_ES0, 0
.set CYDEV_CHIP_REVISION_5A_ES1, 1
.set CYDEV_CHIP_REVISION_5A_PRODUCTION, 1
.set CYDEV_CHIP_REVISION_5B_ES0, 0
.set CYDEV_CHIP_REVISION_5B_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_6A_ES, 17
.set CYDEV_CHIP_REVISION_6A_NO_UDB, 33
.set CYDEV_CHIP_REVISION_6A_PRODUCTION, 33
.set CYDEV_CHIP_REVISION_FM3_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_FM4_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_USED, CYDEV_CHIP_REVISION_4AB_PRODUCTION
.set CYDEV_CHIP_REV_EXPECT, CYDEV_CHIP_REVISION_USED
.set CYDEV_CONFIG_READ_ACCELERATOR, 1
.set CYDEV_CONFIG_UNUSED_IO_AllowButWarn, 0
.set CYDEV_CONFIG_UNUSED_IO_AllowWithInfo, 1
.set CYDEV_CONFIG_UNUSED_IO_Disallowed, 2
.set CYDEV_CONFIG_UNUSED_IO, CYDEV_CONFIG_UNUSED_IO_Disallowed
.set CYDEV_CONFIGURATION_COMPRESSED, 1
.set CYDEV_CONFIGURATION_MODE_COMPRESSED, 0
.set CYDEV_CONFIGURATION_MODE, CYDEV_CONFIGURATION_MODE_COMPRESSED
.set CYDEV_CONFIGURATION_MODE_DMA, 2
.set CYDEV_CONFIGURATION_MODE_UNCOMPRESSED, 1
.set CYDEV_DEBUG_PROTECT_KILL, 4
.set CYDEV_DEBUG_PROTECT_OPEN, 1
.set CYDEV_DEBUG_PROTECT, CYDEV_DEBUG_PROTECT_OPEN
.set CYDEV_DEBUG_PROTECT_PROTECTED, 2
.set CYDEV_DEBUGGING_DPS_Disable, 3
.set CYDEV_DEBUGGING_DPS_SWD, 2
.set CYDEV_DEBUGGING_DPS, CYDEV_DEBUGGING_DPS_SWD
.set CYDEV_DEBUGGING_ENABLE, 1
.set CYDEV_DFT_SELECT_CLK0, 8
.set CYDEV_DFT_SELECT_CLK1, 9
.set CYDEV_DMA_CHANNELS_AVAILABLE, 8
.set CYDEV_HEAP_SIZE, 0x80
.set CYDEV_IMO_TRIMMED_BY_USB, 0
.set CYDEV_IMO_TRIMMED_BY_WCO, 0
.set CYDEV_INTR_NUMBER_DMA, 15
.set CYDEV_IS_EXPORTING_CODE, 0
.set CYDEV_IS_IMPORTING_CODE, 0
.set CYDEV_PROJ_TYPE, 0
.set CYDEV_PROJ_TYPE_BOOTLOADER, 1
.set CYDEV_PROJ_TYPE_LAUNCHER, 5
.set CYDEV_PROJ_TYPE_LOADABLE, 2
.set CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER, 4
.set CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER, 3
.set CYDEV_PROJ_TYPE_STANDARD, 0
.set CYDEV_STACK_SIZE, 0x0800
.set CYDEV_USE_BUNDLED_CMSIS, 1
.set CYDEV_VARIABLE_VDDA, 1
.set CYDEV_VDDA_MV, 3300
.set CYDEV_VDDD_MV, 3300
.set CYDEV_WDT_GENERATE_ISR, 1
.set CYIPBLOCK_m0s8cpussv3_VERSION, 1
.set CYIPBLOCK_m0s8csdv2_VERSION, 2
.set CYIPBLOCK_m0s8exco_VERSION, 2
.set CYIPBLOCK_m0s8ioss_VERSION, 1
.set CYIPBLOCK_m0s8lcd_VERSION, 2
.set CYIPBLOCK_m0s8lpcomp_VERSION, 2
.set CYIPBLOCK_m0s8mca_VERSION, 1
.set CYIPBLOCK_m0s8pass4a_VERSION, 2
.set CYIPBLOCK_m0s8peri_VERSION, 1
.set CYIPBLOCK_m0s8scb_VERSION, 2
.set CYIPBLOCK_m0s8tcpwm_VERSION, 2
.set CYIPBLOCK_m0s8wco_VERSION, 1
.set CYIPBLOCK_s8srsslt_VERSION, 1
.set DMA_CHANNELS_USED__MASK, 0
.set CYDEV_BOOTLOADER_ENABLE, 0
.endif
