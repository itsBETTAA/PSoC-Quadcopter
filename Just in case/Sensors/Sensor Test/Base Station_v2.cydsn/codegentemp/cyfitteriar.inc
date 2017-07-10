#ifndef INCLUDED_CYFITTERIAR_INC
#define INCLUDED_CYFITTERIAR_INC
    INCLUDE cydeviceiar_trm.inc

/* I2C_SCB */
I2C_SCB__BIST_CONTROL EQU CYREG_SCB0_BIST_CONTROL
I2C_SCB__BIST_DATA EQU CYREG_SCB0_BIST_DATA
I2C_SCB__CTRL EQU CYREG_SCB0_CTRL
I2C_SCB__EZ_DATA00 EQU CYREG_SCB0_EZ_DATA00
I2C_SCB__EZ_DATA01 EQU CYREG_SCB0_EZ_DATA01
I2C_SCB__EZ_DATA02 EQU CYREG_SCB0_EZ_DATA02
I2C_SCB__EZ_DATA03 EQU CYREG_SCB0_EZ_DATA03
I2C_SCB__EZ_DATA04 EQU CYREG_SCB0_EZ_DATA04
I2C_SCB__EZ_DATA05 EQU CYREG_SCB0_EZ_DATA05
I2C_SCB__EZ_DATA06 EQU CYREG_SCB0_EZ_DATA06
I2C_SCB__EZ_DATA07 EQU CYREG_SCB0_EZ_DATA07
I2C_SCB__EZ_DATA08 EQU CYREG_SCB0_EZ_DATA08
I2C_SCB__EZ_DATA09 EQU CYREG_SCB0_EZ_DATA09
I2C_SCB__EZ_DATA10 EQU CYREG_SCB0_EZ_DATA10
I2C_SCB__EZ_DATA11 EQU CYREG_SCB0_EZ_DATA11
I2C_SCB__EZ_DATA12 EQU CYREG_SCB0_EZ_DATA12
I2C_SCB__EZ_DATA13 EQU CYREG_SCB0_EZ_DATA13
I2C_SCB__EZ_DATA14 EQU CYREG_SCB0_EZ_DATA14
I2C_SCB__EZ_DATA15 EQU CYREG_SCB0_EZ_DATA15
I2C_SCB__EZ_DATA16 EQU CYREG_SCB0_EZ_DATA16
I2C_SCB__EZ_DATA17 EQU CYREG_SCB0_EZ_DATA17
I2C_SCB__EZ_DATA18 EQU CYREG_SCB0_EZ_DATA18
I2C_SCB__EZ_DATA19 EQU CYREG_SCB0_EZ_DATA19
I2C_SCB__EZ_DATA20 EQU CYREG_SCB0_EZ_DATA20
I2C_SCB__EZ_DATA21 EQU CYREG_SCB0_EZ_DATA21
I2C_SCB__EZ_DATA22 EQU CYREG_SCB0_EZ_DATA22
I2C_SCB__EZ_DATA23 EQU CYREG_SCB0_EZ_DATA23
I2C_SCB__EZ_DATA24 EQU CYREG_SCB0_EZ_DATA24
I2C_SCB__EZ_DATA25 EQU CYREG_SCB0_EZ_DATA25
I2C_SCB__EZ_DATA26 EQU CYREG_SCB0_EZ_DATA26
I2C_SCB__EZ_DATA27 EQU CYREG_SCB0_EZ_DATA27
I2C_SCB__EZ_DATA28 EQU CYREG_SCB0_EZ_DATA28
I2C_SCB__EZ_DATA29 EQU CYREG_SCB0_EZ_DATA29
I2C_SCB__EZ_DATA30 EQU CYREG_SCB0_EZ_DATA30
I2C_SCB__EZ_DATA31 EQU CYREG_SCB0_EZ_DATA31
I2C_SCB__I2C_CFG EQU CYREG_SCB0_I2C_CFG
I2C_SCB__I2C_CTRL EQU CYREG_SCB0_I2C_CTRL
I2C_SCB__I2C_M_CMD EQU CYREG_SCB0_I2C_M_CMD
I2C_SCB__I2C_S_CMD EQU CYREG_SCB0_I2C_S_CMD
I2C_SCB__I2C_STATUS EQU CYREG_SCB0_I2C_STATUS
I2C_SCB__INTR_CAUSE EQU CYREG_SCB0_INTR_CAUSE
I2C_SCB__INTR_I2C_EC EQU CYREG_SCB0_INTR_I2C_EC
I2C_SCB__INTR_I2C_EC_MASK EQU CYREG_SCB0_INTR_I2C_EC_MASK
I2C_SCB__INTR_I2C_EC_MASKED EQU CYREG_SCB0_INTR_I2C_EC_MASKED
I2C_SCB__INTR_M EQU CYREG_SCB0_INTR_M
I2C_SCB__INTR_M_MASK EQU CYREG_SCB0_INTR_M_MASK
I2C_SCB__INTR_M_MASKED EQU CYREG_SCB0_INTR_M_MASKED
I2C_SCB__INTR_M_SET EQU CYREG_SCB0_INTR_M_SET
I2C_SCB__INTR_RX EQU CYREG_SCB0_INTR_RX
I2C_SCB__INTR_RX_MASK EQU CYREG_SCB0_INTR_RX_MASK
I2C_SCB__INTR_RX_MASKED EQU CYREG_SCB0_INTR_RX_MASKED
I2C_SCB__INTR_RX_SET EQU CYREG_SCB0_INTR_RX_SET
I2C_SCB__INTR_S EQU CYREG_SCB0_INTR_S
I2C_SCB__INTR_S_MASK EQU CYREG_SCB0_INTR_S_MASK
I2C_SCB__INTR_S_MASKED EQU CYREG_SCB0_INTR_S_MASKED
I2C_SCB__INTR_S_SET EQU CYREG_SCB0_INTR_S_SET
I2C_SCB__INTR_SPI_EC EQU CYREG_SCB0_INTR_SPI_EC
I2C_SCB__INTR_SPI_EC_MASK EQU CYREG_SCB0_INTR_SPI_EC_MASK
I2C_SCB__INTR_SPI_EC_MASKED EQU CYREG_SCB0_INTR_SPI_EC_MASKED
I2C_SCB__INTR_TX EQU CYREG_SCB0_INTR_TX
I2C_SCB__INTR_TX_MASK EQU CYREG_SCB0_INTR_TX_MASK
I2C_SCB__INTR_TX_MASKED EQU CYREG_SCB0_INTR_TX_MASKED
I2C_SCB__INTR_TX_SET EQU CYREG_SCB0_INTR_TX_SET
I2C_SCB__RX_CTRL EQU CYREG_SCB0_RX_CTRL
I2C_SCB__RX_FIFO_CTRL EQU CYREG_SCB0_RX_FIFO_CTRL
I2C_SCB__RX_FIFO_RD EQU CYREG_SCB0_RX_FIFO_RD
I2C_SCB__RX_FIFO_RD_SILENT EQU CYREG_SCB0_RX_FIFO_RD_SILENT
I2C_SCB__RX_FIFO_STATUS EQU CYREG_SCB0_RX_FIFO_STATUS
I2C_SCB__RX_MATCH EQU CYREG_SCB0_RX_MATCH
I2C_SCB__SPI_CTRL EQU CYREG_SCB0_SPI_CTRL
I2C_SCB__SPI_STATUS EQU CYREG_SCB0_SPI_STATUS
I2C_SCB__SS0_POSISTION EQU 0
I2C_SCB__SS1_POSISTION EQU 1
I2C_SCB__SS2_POSISTION EQU 2
I2C_SCB__SS3_POSISTION EQU 3
I2C_SCB__STATUS EQU CYREG_SCB0_STATUS
I2C_SCB__TX_CTRL EQU CYREG_SCB0_TX_CTRL
I2C_SCB__TX_FIFO_CTRL EQU CYREG_SCB0_TX_FIFO_CTRL
I2C_SCB__TX_FIFO_STATUS EQU CYREG_SCB0_TX_FIFO_STATUS
I2C_SCB__TX_FIFO_WR EQU CYREG_SCB0_TX_FIFO_WR
I2C_SCB__UART_CTRL EQU CYREG_SCB0_UART_CTRL
I2C_SCB__UART_RX_CTRL EQU CYREG_SCB0_UART_RX_CTRL
I2C_SCB__UART_RX_STATUS EQU CYREG_SCB0_UART_RX_STATUS
I2C_SCB__UART_TX_CTRL EQU CYREG_SCB0_UART_TX_CTRL

/* I2C_SCB_IRQ */
I2C_SCB_IRQ__INTC_CLR_EN_REG EQU CYREG_CM0_ICER
I2C_SCB_IRQ__INTC_CLR_PD_REG EQU CYREG_CM0_ICPR
I2C_SCB_IRQ__INTC_MASK EQU 0x400
I2C_SCB_IRQ__INTC_NUMBER EQU 10
I2C_SCB_IRQ__INTC_PRIOR_MASK EQU 0xC00000
I2C_SCB_IRQ__INTC_PRIOR_NUM EQU 3
I2C_SCB_IRQ__INTC_PRIOR_REG EQU CYREG_CM0_IPR2
I2C_SCB_IRQ__INTC_SET_EN_REG EQU CYREG_CM0_ISER
I2C_SCB_IRQ__INTC_SET_PD_REG EQU CYREG_CM0_ISPR

/* I2C_SCBCLK */
I2C_SCBCLK__DIVIDER_MASK EQU 0x0000FFFF
I2C_SCBCLK__ENABLE EQU CYREG_CLK_DIVIDER_A00
I2C_SCBCLK__ENABLE_MASK EQU 0x80000000
I2C_SCBCLK__MASK EQU 0x80000000
I2C_SCBCLK__REGISTER EQU CYREG_CLK_DIVIDER_A00

/* I2C_scl */
I2C_scl__0__DM__MASK EQU 0x07
I2C_scl__0__DM__SHIFT EQU 0
I2C_scl__0__DR EQU CYREG_PRT4_DR
I2C_scl__0__HSIOM EQU CYREG_HSIOM_PORT_SEL4
I2C_scl__0__HSIOM_GPIO EQU 0
I2C_scl__0__HSIOM_I2C EQU 14
I2C_scl__0__HSIOM_I2C_SCL EQU 14
I2C_scl__0__HSIOM_MASK EQU 0x0000000F
I2C_scl__0__HSIOM_SHIFT EQU 0
I2C_scl__0__HSIOM_SPI EQU 15
I2C_scl__0__HSIOM_SPI_MOSI EQU 15
I2C_scl__0__HSIOM_UART EQU 9
I2C_scl__0__HSIOM_UART_RX EQU 9
I2C_scl__0__INTCFG EQU CYREG_PRT4_INTCFG
I2C_scl__0__INTSTAT EQU CYREG_PRT4_INTSTAT
I2C_scl__0__MASK EQU 0x01
I2C_scl__0__PC EQU CYREG_PRT4_PC
I2C_scl__0__PC2 EQU CYREG_PRT4_PC2
I2C_scl__0__PORT EQU 4
I2C_scl__0__PS EQU CYREG_PRT4_PS
I2C_scl__0__SHIFT EQU 0
I2C_scl__DR EQU CYREG_PRT4_DR
I2C_scl__INTCFG EQU CYREG_PRT4_INTCFG
I2C_scl__INTSTAT EQU CYREG_PRT4_INTSTAT
I2C_scl__MASK EQU 0x01
I2C_scl__PC EQU CYREG_PRT4_PC
I2C_scl__PC2 EQU CYREG_PRT4_PC2
I2C_scl__PORT EQU 4
I2C_scl__PS EQU CYREG_PRT4_PS
I2C_scl__SHIFT EQU 0

/* I2C_sda */
I2C_sda__0__DM__MASK EQU 0x38
I2C_sda__0__DM__SHIFT EQU 3
I2C_sda__0__DR EQU CYREG_PRT4_DR
I2C_sda__0__HSIOM EQU CYREG_HSIOM_PORT_SEL4
I2C_sda__0__HSIOM_GPIO EQU 0
I2C_sda__0__HSIOM_I2C EQU 14
I2C_sda__0__HSIOM_I2C_SDA EQU 14
I2C_sda__0__HSIOM_MASK EQU 0x000000F0
I2C_sda__0__HSIOM_SHIFT EQU 4
I2C_sda__0__HSIOM_SPI EQU 15
I2C_sda__0__HSIOM_SPI_MISO EQU 15
I2C_sda__0__HSIOM_UART EQU 9
I2C_sda__0__HSIOM_UART_TX EQU 9
I2C_sda__0__INTCFG EQU CYREG_PRT4_INTCFG
I2C_sda__0__INTSTAT EQU CYREG_PRT4_INTSTAT
I2C_sda__0__MASK EQU 0x02
I2C_sda__0__PC EQU CYREG_PRT4_PC
I2C_sda__0__PC2 EQU CYREG_PRT4_PC2
I2C_sda__0__PORT EQU 4
I2C_sda__0__PS EQU CYREG_PRT4_PS
I2C_sda__0__SHIFT EQU 1
I2C_sda__DR EQU CYREG_PRT4_DR
I2C_sda__INTCFG EQU CYREG_PRT4_INTCFG
I2C_sda__INTSTAT EQU CYREG_PRT4_INTSTAT
I2C_sda__MASK EQU 0x02
I2C_sda__PC EQU CYREG_PRT4_PC
I2C_sda__PC2 EQU CYREG_PRT4_PC2
I2C_sda__PORT EQU 4
I2C_sda__PS EQU CYREG_PRT4_PS
I2C_sda__SHIFT EQU 1

/* GPOUT */
GPOUT__0__DM__MASK EQU 0x07
GPOUT__0__DM__SHIFT EQU 0
GPOUT__0__DR EQU CYREG_PRT0_DR
GPOUT__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
GPOUT__0__HSIOM_MASK EQU 0x0000000F
GPOUT__0__HSIOM_SHIFT EQU 0
GPOUT__0__INTCFG EQU CYREG_PRT0_INTCFG
GPOUT__0__INTSTAT EQU CYREG_PRT0_INTSTAT
GPOUT__0__MASK EQU 0x01
GPOUT__0__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
GPOUT__0__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
GPOUT__0__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
GPOUT__0__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
GPOUT__0__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
GPOUT__0__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
GPOUT__0__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
GPOUT__0__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
GPOUT__0__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
GPOUT__0__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
GPOUT__0__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
GPOUT__0__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
GPOUT__0__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
GPOUT__0__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
GPOUT__0__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
GPOUT__0__PC EQU CYREG_PRT0_PC
GPOUT__0__PC2 EQU CYREG_PRT0_PC2
GPOUT__0__PORT EQU 0
GPOUT__0__PS EQU CYREG_PRT0_PS
GPOUT__0__SHIFT EQU 0
GPOUT__DR EQU CYREG_PRT0_DR
GPOUT__INTCFG EQU CYREG_PRT0_INTCFG
GPOUT__INTSTAT EQU CYREG_PRT0_INTSTAT
GPOUT__MASK EQU 0x01
GPOUT__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
GPOUT__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
GPOUT__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
GPOUT__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
GPOUT__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
GPOUT__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
GPOUT__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
GPOUT__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
GPOUT__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
GPOUT__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
GPOUT__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
GPOUT__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
GPOUT__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
GPOUT__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
GPOUT__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
GPOUT__PC EQU CYREG_PRT0_PC
GPOUT__PC2 EQU CYREG_PRT0_PC2
GPOUT__PORT EQU 0
GPOUT__PS EQU CYREG_PRT0_PS
GPOUT__SHIFT EQU 0
GPOUT__SNAP EQU CYREG_PRT0_INTSTAT

/* serial_rx */
serial_rx__0__DM__MASK EQU 0x7000
serial_rx__0__DM__SHIFT EQU 12
serial_rx__0__DR EQU CYREG_PRT0_DR
serial_rx__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
serial_rx__0__HSIOM_GPIO EQU 0
serial_rx__0__HSIOM_I2C EQU 14
serial_rx__0__HSIOM_I2C_SCL EQU 14
serial_rx__0__HSIOM_MASK EQU 0x000F0000
serial_rx__0__HSIOM_SHIFT EQU 16
serial_rx__0__HSIOM_SPI EQU 15
serial_rx__0__HSIOM_SPI_MOSI EQU 15
serial_rx__0__HSIOM_UART EQU 9
serial_rx__0__HSIOM_UART_RX EQU 9
serial_rx__0__INTCFG EQU CYREG_PRT0_INTCFG
serial_rx__0__INTSTAT EQU CYREG_PRT0_INTSTAT
serial_rx__0__MASK EQU 0x10
serial_rx__0__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
serial_rx__0__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
serial_rx__0__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
serial_rx__0__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
serial_rx__0__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
serial_rx__0__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
serial_rx__0__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
serial_rx__0__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
serial_rx__0__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
serial_rx__0__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
serial_rx__0__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
serial_rx__0__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
serial_rx__0__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
serial_rx__0__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
serial_rx__0__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
serial_rx__0__PC EQU CYREG_PRT0_PC
serial_rx__0__PC2 EQU CYREG_PRT0_PC2
serial_rx__0__PORT EQU 0
serial_rx__0__PS EQU CYREG_PRT0_PS
serial_rx__0__SHIFT EQU 4
serial_rx__DR EQU CYREG_PRT0_DR
serial_rx__INTCFG EQU CYREG_PRT0_INTCFG
serial_rx__INTSTAT EQU CYREG_PRT0_INTSTAT
serial_rx__MASK EQU 0x10
serial_rx__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
serial_rx__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
serial_rx__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
serial_rx__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
serial_rx__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
serial_rx__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
serial_rx__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
serial_rx__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
serial_rx__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
serial_rx__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
serial_rx__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
serial_rx__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
serial_rx__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
serial_rx__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
serial_rx__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
serial_rx__PC EQU CYREG_PRT0_PC
serial_rx__PC2 EQU CYREG_PRT0_PC2
serial_rx__PORT EQU 0
serial_rx__PS EQU CYREG_PRT0_PS
serial_rx__SHIFT EQU 4

/* serial_SCB */
serial_SCB__BIST_CONTROL EQU CYREG_SCB1_BIST_CONTROL
serial_SCB__BIST_DATA EQU CYREG_SCB1_BIST_DATA
serial_SCB__CTRL EQU CYREG_SCB1_CTRL
serial_SCB__EZ_DATA00 EQU CYREG_SCB1_EZ_DATA00
serial_SCB__EZ_DATA01 EQU CYREG_SCB1_EZ_DATA01
serial_SCB__EZ_DATA02 EQU CYREG_SCB1_EZ_DATA02
serial_SCB__EZ_DATA03 EQU CYREG_SCB1_EZ_DATA03
serial_SCB__EZ_DATA04 EQU CYREG_SCB1_EZ_DATA04
serial_SCB__EZ_DATA05 EQU CYREG_SCB1_EZ_DATA05
serial_SCB__EZ_DATA06 EQU CYREG_SCB1_EZ_DATA06
serial_SCB__EZ_DATA07 EQU CYREG_SCB1_EZ_DATA07
serial_SCB__EZ_DATA08 EQU CYREG_SCB1_EZ_DATA08
serial_SCB__EZ_DATA09 EQU CYREG_SCB1_EZ_DATA09
serial_SCB__EZ_DATA10 EQU CYREG_SCB1_EZ_DATA10
serial_SCB__EZ_DATA11 EQU CYREG_SCB1_EZ_DATA11
serial_SCB__EZ_DATA12 EQU CYREG_SCB1_EZ_DATA12
serial_SCB__EZ_DATA13 EQU CYREG_SCB1_EZ_DATA13
serial_SCB__EZ_DATA14 EQU CYREG_SCB1_EZ_DATA14
serial_SCB__EZ_DATA15 EQU CYREG_SCB1_EZ_DATA15
serial_SCB__EZ_DATA16 EQU CYREG_SCB1_EZ_DATA16
serial_SCB__EZ_DATA17 EQU CYREG_SCB1_EZ_DATA17
serial_SCB__EZ_DATA18 EQU CYREG_SCB1_EZ_DATA18
serial_SCB__EZ_DATA19 EQU CYREG_SCB1_EZ_DATA19
serial_SCB__EZ_DATA20 EQU CYREG_SCB1_EZ_DATA20
serial_SCB__EZ_DATA21 EQU CYREG_SCB1_EZ_DATA21
serial_SCB__EZ_DATA22 EQU CYREG_SCB1_EZ_DATA22
serial_SCB__EZ_DATA23 EQU CYREG_SCB1_EZ_DATA23
serial_SCB__EZ_DATA24 EQU CYREG_SCB1_EZ_DATA24
serial_SCB__EZ_DATA25 EQU CYREG_SCB1_EZ_DATA25
serial_SCB__EZ_DATA26 EQU CYREG_SCB1_EZ_DATA26
serial_SCB__EZ_DATA27 EQU CYREG_SCB1_EZ_DATA27
serial_SCB__EZ_DATA28 EQU CYREG_SCB1_EZ_DATA28
serial_SCB__EZ_DATA29 EQU CYREG_SCB1_EZ_DATA29
serial_SCB__EZ_DATA30 EQU CYREG_SCB1_EZ_DATA30
serial_SCB__EZ_DATA31 EQU CYREG_SCB1_EZ_DATA31
serial_SCB__I2C_CFG EQU CYREG_SCB1_I2C_CFG
serial_SCB__I2C_CTRL EQU CYREG_SCB1_I2C_CTRL
serial_SCB__I2C_M_CMD EQU CYREG_SCB1_I2C_M_CMD
serial_SCB__I2C_S_CMD EQU CYREG_SCB1_I2C_S_CMD
serial_SCB__I2C_STATUS EQU CYREG_SCB1_I2C_STATUS
serial_SCB__INTR_CAUSE EQU CYREG_SCB1_INTR_CAUSE
serial_SCB__INTR_I2C_EC EQU CYREG_SCB1_INTR_I2C_EC
serial_SCB__INTR_I2C_EC_MASK EQU CYREG_SCB1_INTR_I2C_EC_MASK
serial_SCB__INTR_I2C_EC_MASKED EQU CYREG_SCB1_INTR_I2C_EC_MASKED
serial_SCB__INTR_M EQU CYREG_SCB1_INTR_M
serial_SCB__INTR_M_MASK EQU CYREG_SCB1_INTR_M_MASK
serial_SCB__INTR_M_MASKED EQU CYREG_SCB1_INTR_M_MASKED
serial_SCB__INTR_M_SET EQU CYREG_SCB1_INTR_M_SET
serial_SCB__INTR_RX EQU CYREG_SCB1_INTR_RX
serial_SCB__INTR_RX_MASK EQU CYREG_SCB1_INTR_RX_MASK
serial_SCB__INTR_RX_MASKED EQU CYREG_SCB1_INTR_RX_MASKED
serial_SCB__INTR_RX_SET EQU CYREG_SCB1_INTR_RX_SET
serial_SCB__INTR_S EQU CYREG_SCB1_INTR_S
serial_SCB__INTR_S_MASK EQU CYREG_SCB1_INTR_S_MASK
serial_SCB__INTR_S_MASKED EQU CYREG_SCB1_INTR_S_MASKED
serial_SCB__INTR_S_SET EQU CYREG_SCB1_INTR_S_SET
serial_SCB__INTR_SPI_EC EQU CYREG_SCB1_INTR_SPI_EC
serial_SCB__INTR_SPI_EC_MASK EQU CYREG_SCB1_INTR_SPI_EC_MASK
serial_SCB__INTR_SPI_EC_MASKED EQU CYREG_SCB1_INTR_SPI_EC_MASKED
serial_SCB__INTR_TX EQU CYREG_SCB1_INTR_TX
serial_SCB__INTR_TX_MASK EQU CYREG_SCB1_INTR_TX_MASK
serial_SCB__INTR_TX_MASKED EQU CYREG_SCB1_INTR_TX_MASKED
serial_SCB__INTR_TX_SET EQU CYREG_SCB1_INTR_TX_SET
serial_SCB__RX_CTRL EQU CYREG_SCB1_RX_CTRL
serial_SCB__RX_FIFO_CTRL EQU CYREG_SCB1_RX_FIFO_CTRL
serial_SCB__RX_FIFO_RD EQU CYREG_SCB1_RX_FIFO_RD
serial_SCB__RX_FIFO_RD_SILENT EQU CYREG_SCB1_RX_FIFO_RD_SILENT
serial_SCB__RX_FIFO_STATUS EQU CYREG_SCB1_RX_FIFO_STATUS
serial_SCB__RX_MATCH EQU CYREG_SCB1_RX_MATCH
serial_SCB__SPI_CTRL EQU CYREG_SCB1_SPI_CTRL
serial_SCB__SPI_STATUS EQU CYREG_SCB1_SPI_STATUS
serial_SCB__SS0_POSISTION EQU 0
serial_SCB__SS1_POSISTION EQU 1
serial_SCB__SS2_POSISTION EQU 2
serial_SCB__SS3_POSISTION EQU 3
serial_SCB__STATUS EQU CYREG_SCB1_STATUS
serial_SCB__TX_CTRL EQU CYREG_SCB1_TX_CTRL
serial_SCB__TX_FIFO_CTRL EQU CYREG_SCB1_TX_FIFO_CTRL
serial_SCB__TX_FIFO_STATUS EQU CYREG_SCB1_TX_FIFO_STATUS
serial_SCB__TX_FIFO_WR EQU CYREG_SCB1_TX_FIFO_WR
serial_SCB__UART_CTRL EQU CYREG_SCB1_UART_CTRL
serial_SCB__UART_RX_CTRL EQU CYREG_SCB1_UART_RX_CTRL
serial_SCB__UART_RX_STATUS EQU CYREG_SCB1_UART_RX_STATUS
serial_SCB__UART_TX_CTRL EQU CYREG_SCB1_UART_TX_CTRL

/* serial_SCB_IRQ */
serial_SCB_IRQ__INTC_CLR_EN_REG EQU CYREG_CM0_ICER
serial_SCB_IRQ__INTC_CLR_PD_REG EQU CYREG_CM0_ICPR
serial_SCB_IRQ__INTC_MASK EQU 0x800
serial_SCB_IRQ__INTC_NUMBER EQU 11
serial_SCB_IRQ__INTC_PRIOR_MASK EQU 0xC0000000
serial_SCB_IRQ__INTC_PRIOR_NUM EQU 3
serial_SCB_IRQ__INTC_PRIOR_REG EQU CYREG_CM0_IPR2
serial_SCB_IRQ__INTC_SET_EN_REG EQU CYREG_CM0_ISER
serial_SCB_IRQ__INTC_SET_PD_REG EQU CYREG_CM0_ISPR

/* serial_SCBCLK */
serial_SCBCLK__DIVIDER_MASK EQU 0x0000FFFF
serial_SCBCLK__ENABLE EQU CYREG_CLK_DIVIDER_B00
serial_SCBCLK__ENABLE_MASK EQU 0x80000000
serial_SCBCLK__MASK EQU 0x80000000
serial_SCBCLK__REGISTER EQU CYREG_CLK_DIVIDER_B00

/* serial_tx */
serial_tx__0__DM__MASK EQU 0x38000
serial_tx__0__DM__SHIFT EQU 15
serial_tx__0__DR EQU CYREG_PRT0_DR
serial_tx__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
serial_tx__0__HSIOM_GPIO EQU 0
serial_tx__0__HSIOM_I2C EQU 14
serial_tx__0__HSIOM_I2C_SDA EQU 14
serial_tx__0__HSIOM_MASK EQU 0x00F00000
serial_tx__0__HSIOM_SHIFT EQU 20
serial_tx__0__HSIOM_SPI EQU 15
serial_tx__0__HSIOM_SPI_MISO EQU 15
serial_tx__0__HSIOM_UART EQU 9
serial_tx__0__HSIOM_UART_TX EQU 9
serial_tx__0__INTCFG EQU CYREG_PRT0_INTCFG
serial_tx__0__INTSTAT EQU CYREG_PRT0_INTSTAT
serial_tx__0__MASK EQU 0x20
serial_tx__0__OUT_SEL EQU CYREG_UDB_PA0_CFG10
serial_tx__0__OUT_SEL_SHIFT EQU 10
serial_tx__0__OUT_SEL_VAL EQU -1
serial_tx__0__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
serial_tx__0__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
serial_tx__0__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
serial_tx__0__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
serial_tx__0__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
serial_tx__0__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
serial_tx__0__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
serial_tx__0__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
serial_tx__0__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
serial_tx__0__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
serial_tx__0__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
serial_tx__0__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
serial_tx__0__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
serial_tx__0__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
serial_tx__0__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
serial_tx__0__PC EQU CYREG_PRT0_PC
serial_tx__0__PC2 EQU CYREG_PRT0_PC2
serial_tx__0__PORT EQU 0
serial_tx__0__PS EQU CYREG_PRT0_PS
serial_tx__0__SHIFT EQU 5
serial_tx__DR EQU CYREG_PRT0_DR
serial_tx__INTCFG EQU CYREG_PRT0_INTCFG
serial_tx__INTSTAT EQU CYREG_PRT0_INTSTAT
serial_tx__MASK EQU 0x20
serial_tx__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
serial_tx__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
serial_tx__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
serial_tx__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
serial_tx__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
serial_tx__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
serial_tx__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
serial_tx__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
serial_tx__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
serial_tx__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
serial_tx__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
serial_tx__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
serial_tx__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
serial_tx__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
serial_tx__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
serial_tx__PC EQU CYREG_PRT0_PC
serial_tx__PC2 EQU CYREG_PRT0_PC2
serial_tx__PORT EQU 0
serial_tx__PS EQU CYREG_PRT0_PS
serial_tx__SHIFT EQU 5

/* RED_LED */
RED_LED__0__DM__MASK EQU 0x38
RED_LED__0__DM__SHIFT EQU 3
RED_LED__0__DR EQU CYREG_PRT0_DR
RED_LED__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
RED_LED__0__HSIOM_MASK EQU 0x000000F0
RED_LED__0__HSIOM_SHIFT EQU 4
RED_LED__0__INTCFG EQU CYREG_PRT0_INTCFG
RED_LED__0__INTSTAT EQU CYREG_PRT0_INTSTAT
RED_LED__0__MASK EQU 0x02
RED_LED__0__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
RED_LED__0__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
RED_LED__0__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
RED_LED__0__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
RED_LED__0__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
RED_LED__0__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
RED_LED__0__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
RED_LED__0__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
RED_LED__0__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
RED_LED__0__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
RED_LED__0__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
RED_LED__0__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
RED_LED__0__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
RED_LED__0__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
RED_LED__0__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
RED_LED__0__PC EQU CYREG_PRT0_PC
RED_LED__0__PC2 EQU CYREG_PRT0_PC2
RED_LED__0__PORT EQU 0
RED_LED__0__PS EQU CYREG_PRT0_PS
RED_LED__0__SHIFT EQU 1
RED_LED__DR EQU CYREG_PRT0_DR
RED_LED__INTCFG EQU CYREG_PRT0_INTCFG
RED_LED__INTSTAT EQU CYREG_PRT0_INTSTAT
RED_LED__MASK EQU 0x02
RED_LED__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
RED_LED__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
RED_LED__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
RED_LED__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
RED_LED__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
RED_LED__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
RED_LED__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
RED_LED__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
RED_LED__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
RED_LED__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
RED_LED__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
RED_LED__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
RED_LED__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
RED_LED__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
RED_LED__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
RED_LED__PC EQU CYREG_PRT0_PC
RED_LED__PC2 EQU CYREG_PRT0_PC2
RED_LED__PORT EQU 0
RED_LED__PS EQU CYREG_PRT0_PS
RED_LED__SHIFT EQU 1

/* butt_pin */
butt_pin__0__DM__MASK EQU 0x1C0
butt_pin__0__DM__SHIFT EQU 6
butt_pin__0__DR EQU CYREG_PRT2_DR
butt_pin__0__HSIOM EQU CYREG_HSIOM_PORT_SEL2
butt_pin__0__HSIOM_MASK EQU 0x00000F00
butt_pin__0__HSIOM_SHIFT EQU 8
butt_pin__0__INTCFG EQU CYREG_PRT2_INTCFG
butt_pin__0__INTSTAT EQU CYREG_PRT2_INTSTAT
butt_pin__0__MASK EQU 0x04
butt_pin__0__PA__CFG0 EQU CYREG_UDB_PA2_CFG0
butt_pin__0__PA__CFG1 EQU CYREG_UDB_PA2_CFG1
butt_pin__0__PA__CFG10 EQU CYREG_UDB_PA2_CFG10
butt_pin__0__PA__CFG11 EQU CYREG_UDB_PA2_CFG11
butt_pin__0__PA__CFG12 EQU CYREG_UDB_PA2_CFG12
butt_pin__0__PA__CFG13 EQU CYREG_UDB_PA2_CFG13
butt_pin__0__PA__CFG14 EQU CYREG_UDB_PA2_CFG14
butt_pin__0__PA__CFG2 EQU CYREG_UDB_PA2_CFG2
butt_pin__0__PA__CFG3 EQU CYREG_UDB_PA2_CFG3
butt_pin__0__PA__CFG4 EQU CYREG_UDB_PA2_CFG4
butt_pin__0__PA__CFG5 EQU CYREG_UDB_PA2_CFG5
butt_pin__0__PA__CFG6 EQU CYREG_UDB_PA2_CFG6
butt_pin__0__PA__CFG7 EQU CYREG_UDB_PA2_CFG7
butt_pin__0__PA__CFG8 EQU CYREG_UDB_PA2_CFG8
butt_pin__0__PA__CFG9 EQU CYREG_UDB_PA2_CFG9
butt_pin__0__PC EQU CYREG_PRT2_PC
butt_pin__0__PC2 EQU CYREG_PRT2_PC2
butt_pin__0__PORT EQU 2
butt_pin__0__PS EQU CYREG_PRT2_PS
butt_pin__0__SHIFT EQU 2
butt_pin__DR EQU CYREG_PRT2_DR
butt_pin__INTCFG EQU CYREG_PRT2_INTCFG
butt_pin__INTSTAT EQU CYREG_PRT2_INTSTAT
butt_pin__MASK EQU 0x04
butt_pin__PA__CFG0 EQU CYREG_UDB_PA2_CFG0
butt_pin__PA__CFG1 EQU CYREG_UDB_PA2_CFG1
butt_pin__PA__CFG10 EQU CYREG_UDB_PA2_CFG10
butt_pin__PA__CFG11 EQU CYREG_UDB_PA2_CFG11
butt_pin__PA__CFG12 EQU CYREG_UDB_PA2_CFG12
butt_pin__PA__CFG13 EQU CYREG_UDB_PA2_CFG13
butt_pin__PA__CFG14 EQU CYREG_UDB_PA2_CFG14
butt_pin__PA__CFG2 EQU CYREG_UDB_PA2_CFG2
butt_pin__PA__CFG3 EQU CYREG_UDB_PA2_CFG3
butt_pin__PA__CFG4 EQU CYREG_UDB_PA2_CFG4
butt_pin__PA__CFG5 EQU CYREG_UDB_PA2_CFG5
butt_pin__PA__CFG6 EQU CYREG_UDB_PA2_CFG6
butt_pin__PA__CFG7 EQU CYREG_UDB_PA2_CFG7
butt_pin__PA__CFG8 EQU CYREG_UDB_PA2_CFG8
butt_pin__PA__CFG9 EQU CYREG_UDB_PA2_CFG9
butt_pin__PC EQU CYREG_PRT2_PC
butt_pin__PC2 EQU CYREG_PRT2_PC2
butt_pin__PORT EQU 2
butt_pin__PS EQU CYREG_PRT2_PS
butt_pin__SHIFT EQU 2
butt_pin__SNAP EQU CYREG_PRT2_INTSTAT

/* BOARD_LED */
BOARD_LED__0__DM__MASK EQU 0x1C0000
BOARD_LED__0__DM__SHIFT EQU 18
BOARD_LED__0__DR EQU CYREG_PRT1_DR
BOARD_LED__0__HSIOM EQU CYREG_HSIOM_PORT_SEL1
BOARD_LED__0__HSIOM_MASK EQU 0x0F000000
BOARD_LED__0__HSIOM_SHIFT EQU 24
BOARD_LED__0__INTCFG EQU CYREG_PRT1_INTCFG
BOARD_LED__0__INTSTAT EQU CYREG_PRT1_INTSTAT
BOARD_LED__0__MASK EQU 0x40
BOARD_LED__0__PA__CFG0 EQU CYREG_UDB_PA1_CFG0
BOARD_LED__0__PA__CFG1 EQU CYREG_UDB_PA1_CFG1
BOARD_LED__0__PA__CFG10 EQU CYREG_UDB_PA1_CFG10
BOARD_LED__0__PA__CFG11 EQU CYREG_UDB_PA1_CFG11
BOARD_LED__0__PA__CFG12 EQU CYREG_UDB_PA1_CFG12
BOARD_LED__0__PA__CFG13 EQU CYREG_UDB_PA1_CFG13
BOARD_LED__0__PA__CFG14 EQU CYREG_UDB_PA1_CFG14
BOARD_LED__0__PA__CFG2 EQU CYREG_UDB_PA1_CFG2
BOARD_LED__0__PA__CFG3 EQU CYREG_UDB_PA1_CFG3
BOARD_LED__0__PA__CFG4 EQU CYREG_UDB_PA1_CFG4
BOARD_LED__0__PA__CFG5 EQU CYREG_UDB_PA1_CFG5
BOARD_LED__0__PA__CFG6 EQU CYREG_UDB_PA1_CFG6
BOARD_LED__0__PA__CFG7 EQU CYREG_UDB_PA1_CFG7
BOARD_LED__0__PA__CFG8 EQU CYREG_UDB_PA1_CFG8
BOARD_LED__0__PA__CFG9 EQU CYREG_UDB_PA1_CFG9
BOARD_LED__0__PC EQU CYREG_PRT1_PC
BOARD_LED__0__PC2 EQU CYREG_PRT1_PC2
BOARD_LED__0__PORT EQU 1
BOARD_LED__0__PS EQU CYREG_PRT1_PS
BOARD_LED__0__SHIFT EQU 6
BOARD_LED__DR EQU CYREG_PRT1_DR
BOARD_LED__INTCFG EQU CYREG_PRT1_INTCFG
BOARD_LED__INTSTAT EQU CYREG_PRT1_INTSTAT
BOARD_LED__MASK EQU 0x40
BOARD_LED__PA__CFG0 EQU CYREG_UDB_PA1_CFG0
BOARD_LED__PA__CFG1 EQU CYREG_UDB_PA1_CFG1
BOARD_LED__PA__CFG10 EQU CYREG_UDB_PA1_CFG10
BOARD_LED__PA__CFG11 EQU CYREG_UDB_PA1_CFG11
BOARD_LED__PA__CFG12 EQU CYREG_UDB_PA1_CFG12
BOARD_LED__PA__CFG13 EQU CYREG_UDB_PA1_CFG13
BOARD_LED__PA__CFG14 EQU CYREG_UDB_PA1_CFG14
BOARD_LED__PA__CFG2 EQU CYREG_UDB_PA1_CFG2
BOARD_LED__PA__CFG3 EQU CYREG_UDB_PA1_CFG3
BOARD_LED__PA__CFG4 EQU CYREG_UDB_PA1_CFG4
BOARD_LED__PA__CFG5 EQU CYREG_UDB_PA1_CFG5
BOARD_LED__PA__CFG6 EQU CYREG_UDB_PA1_CFG6
BOARD_LED__PA__CFG7 EQU CYREG_UDB_PA1_CFG7
BOARD_LED__PA__CFG8 EQU CYREG_UDB_PA1_CFG8
BOARD_LED__PA__CFG9 EQU CYREG_UDB_PA1_CFG9
BOARD_LED__PC EQU CYREG_PRT1_PC
BOARD_LED__PC2 EQU CYREG_PRT1_PC2
BOARD_LED__PORT EQU 1
BOARD_LED__PS EQU CYREG_PRT1_PS
BOARD_LED__SHIFT EQU 6

/* GREEN_LED */
GREEN_LED__0__DM__MASK EQU 0xE00
GREEN_LED__0__DM__SHIFT EQU 9
GREEN_LED__0__DR EQU CYREG_PRT0_DR
GREEN_LED__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
GREEN_LED__0__HSIOM_MASK EQU 0x0000F000
GREEN_LED__0__HSIOM_SHIFT EQU 12
GREEN_LED__0__INTCFG EQU CYREG_PRT0_INTCFG
GREEN_LED__0__INTSTAT EQU CYREG_PRT0_INTSTAT
GREEN_LED__0__MASK EQU 0x08
GREEN_LED__0__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
GREEN_LED__0__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
GREEN_LED__0__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
GREEN_LED__0__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
GREEN_LED__0__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
GREEN_LED__0__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
GREEN_LED__0__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
GREEN_LED__0__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
GREEN_LED__0__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
GREEN_LED__0__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
GREEN_LED__0__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
GREEN_LED__0__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
GREEN_LED__0__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
GREEN_LED__0__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
GREEN_LED__0__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
GREEN_LED__0__PC EQU CYREG_PRT0_PC
GREEN_LED__0__PC2 EQU CYREG_PRT0_PC2
GREEN_LED__0__PORT EQU 0
GREEN_LED__0__PS EQU CYREG_PRT0_PS
GREEN_LED__0__SHIFT EQU 3
GREEN_LED__DR EQU CYREG_PRT0_DR
GREEN_LED__INTCFG EQU CYREG_PRT0_INTCFG
GREEN_LED__INTSTAT EQU CYREG_PRT0_INTSTAT
GREEN_LED__MASK EQU 0x08
GREEN_LED__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
GREEN_LED__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
GREEN_LED__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
GREEN_LED__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
GREEN_LED__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
GREEN_LED__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
GREEN_LED__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
GREEN_LED__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
GREEN_LED__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
GREEN_LED__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
GREEN_LED__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
GREEN_LED__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
GREEN_LED__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
GREEN_LED__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
GREEN_LED__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
GREEN_LED__PC EQU CYREG_PRT0_PC
GREEN_LED__PC2 EQU CYREG_PRT0_PC2
GREEN_LED__PORT EQU 0
GREEN_LED__PS EQU CYREG_PRT0_PS
GREEN_LED__SHIFT EQU 3

/* YELLOW_LED */
YELLOW_LED__0__DM__MASK EQU 0x1C0
YELLOW_LED__0__DM__SHIFT EQU 6
YELLOW_LED__0__DR EQU CYREG_PRT0_DR
YELLOW_LED__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
YELLOW_LED__0__HSIOM_MASK EQU 0x00000F00
YELLOW_LED__0__HSIOM_SHIFT EQU 8
YELLOW_LED__0__INTCFG EQU CYREG_PRT0_INTCFG
YELLOW_LED__0__INTSTAT EQU CYREG_PRT0_INTSTAT
YELLOW_LED__0__MASK EQU 0x04
YELLOW_LED__0__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
YELLOW_LED__0__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
YELLOW_LED__0__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
YELLOW_LED__0__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
YELLOW_LED__0__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
YELLOW_LED__0__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
YELLOW_LED__0__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
YELLOW_LED__0__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
YELLOW_LED__0__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
YELLOW_LED__0__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
YELLOW_LED__0__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
YELLOW_LED__0__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
YELLOW_LED__0__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
YELLOW_LED__0__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
YELLOW_LED__0__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
YELLOW_LED__0__PC EQU CYREG_PRT0_PC
YELLOW_LED__0__PC2 EQU CYREG_PRT0_PC2
YELLOW_LED__0__PORT EQU 0
YELLOW_LED__0__PS EQU CYREG_PRT0_PS
YELLOW_LED__0__SHIFT EQU 2
YELLOW_LED__DR EQU CYREG_PRT0_DR
YELLOW_LED__INTCFG EQU CYREG_PRT0_INTCFG
YELLOW_LED__INTSTAT EQU CYREG_PRT0_INTSTAT
YELLOW_LED__MASK EQU 0x04
YELLOW_LED__PA__CFG0 EQU CYREG_UDB_PA0_CFG0
YELLOW_LED__PA__CFG1 EQU CYREG_UDB_PA0_CFG1
YELLOW_LED__PA__CFG10 EQU CYREG_UDB_PA0_CFG10
YELLOW_LED__PA__CFG11 EQU CYREG_UDB_PA0_CFG11
YELLOW_LED__PA__CFG12 EQU CYREG_UDB_PA0_CFG12
YELLOW_LED__PA__CFG13 EQU CYREG_UDB_PA0_CFG13
YELLOW_LED__PA__CFG14 EQU CYREG_UDB_PA0_CFG14
YELLOW_LED__PA__CFG2 EQU CYREG_UDB_PA0_CFG2
YELLOW_LED__PA__CFG3 EQU CYREG_UDB_PA0_CFG3
YELLOW_LED__PA__CFG4 EQU CYREG_UDB_PA0_CFG4
YELLOW_LED__PA__CFG5 EQU CYREG_UDB_PA0_CFG5
YELLOW_LED__PA__CFG6 EQU CYREG_UDB_PA0_CFG6
YELLOW_LED__PA__CFG7 EQU CYREG_UDB_PA0_CFG7
YELLOW_LED__PA__CFG8 EQU CYREG_UDB_PA0_CFG8
YELLOW_LED__PA__CFG9 EQU CYREG_UDB_PA0_CFG9
YELLOW_LED__PC EQU CYREG_PRT0_PC
YELLOW_LED__PC2 EQU CYREG_PRT0_PC2
YELLOW_LED__PORT EQU 0
YELLOW_LED__PS EQU CYREG_PRT0_PS
YELLOW_LED__SHIFT EQU 2

/* lipo_gpout */
lipo_gpout__INTC_CLR_EN_REG EQU CYREG_CM0_ICER
lipo_gpout__INTC_CLR_PD_REG EQU CYREG_CM0_ICPR
lipo_gpout__INTC_MASK EQU 0x01
lipo_gpout__INTC_NUMBER EQU 0
lipo_gpout__INTC_PRIOR_MASK EQU 0xC0
lipo_gpout__INTC_PRIOR_NUM EQU 3
lipo_gpout__INTC_PRIOR_REG EQU CYREG_CM0_IPR0
lipo_gpout__INTC_SET_EN_REG EQU CYREG_CM0_ISER
lipo_gpout__INTC_SET_PD_REG EQU CYREG_CM0_ISPR

/* read_battery */
read_battery__INTC_CLR_EN_REG EQU CYREG_CM0_ICER
read_battery__INTC_CLR_PD_REG EQU CYREG_CM0_ICPR
read_battery__INTC_MASK EQU 0x04
read_battery__INTC_NUMBER EQU 2
read_battery__INTC_PRIOR_MASK EQU 0xC00000
read_battery__INTC_PRIOR_NUM EQU 3
read_battery__INTC_PRIOR_REG EQU CYREG_CM0_IPR0
read_battery__INTC_SET_EN_REG EQU CYREG_CM0_ISER
read_battery__INTC_SET_PD_REG EQU CYREG_CM0_ISPR

/* Miscellaneous */
CYDEV_BCLK__HFCLK__HZ EQU 24000000
CYDEV_BCLK__HFCLK__KHZ EQU 24000
CYDEV_BCLK__HFCLK__MHZ EQU 24
CYDEV_BCLK__SYSCLK__HZ EQU 24000000
CYDEV_BCLK__SYSCLK__KHZ EQU 24000
CYDEV_BCLK__SYSCLK__MHZ EQU 24
CYDEV_CHIP_DIE_LEOPARD EQU 1
CYDEV_CHIP_DIE_PSOC4A EQU 12
CYDEV_CHIP_DIE_PSOC5LP EQU 19
CYDEV_CHIP_DIE_PSOC5TM EQU 20
CYDEV_CHIP_DIE_TMA4 EQU 2
CYDEV_CHIP_DIE_UNKNOWN EQU 0
CYDEV_CHIP_FAMILY_FM0P EQU 4
CYDEV_CHIP_FAMILY_FM3 EQU 5
CYDEV_CHIP_FAMILY_FM4 EQU 6
CYDEV_CHIP_FAMILY_PSOC3 EQU 1
CYDEV_CHIP_FAMILY_PSOC4 EQU 2
CYDEV_CHIP_FAMILY_PSOC5 EQU 3
CYDEV_CHIP_FAMILY_UNKNOWN EQU 0
CYDEV_CHIP_FAMILY_USED EQU CYDEV_CHIP_FAMILY_PSOC4
CYDEV_CHIP_JTAG_ID EQU 0x04C81193
CYDEV_CHIP_MEMBER_3A EQU 1
CYDEV_CHIP_MEMBER_4A EQU 12
CYDEV_CHIP_MEMBER_4C EQU 18
CYDEV_CHIP_MEMBER_4D EQU 8
CYDEV_CHIP_MEMBER_4E EQU 4
CYDEV_CHIP_MEMBER_4F EQU 13
CYDEV_CHIP_MEMBER_4G EQU 2
CYDEV_CHIP_MEMBER_4H EQU 11
CYDEV_CHIP_MEMBER_4I EQU 17
CYDEV_CHIP_MEMBER_4J EQU 9
CYDEV_CHIP_MEMBER_4K EQU 10
CYDEV_CHIP_MEMBER_4L EQU 16
CYDEV_CHIP_MEMBER_4M EQU 15
CYDEV_CHIP_MEMBER_4N EQU 6
CYDEV_CHIP_MEMBER_4O EQU 5
CYDEV_CHIP_MEMBER_4P EQU 14
CYDEV_CHIP_MEMBER_4Q EQU 7
CYDEV_CHIP_MEMBER_4U EQU 3
CYDEV_CHIP_MEMBER_5A EQU 20
CYDEV_CHIP_MEMBER_5B EQU 19
CYDEV_CHIP_MEMBER_FM3 EQU 24
CYDEV_CHIP_MEMBER_FM4 EQU 25
CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 EQU 21
CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 EQU 22
CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 EQU 23
CYDEV_CHIP_MEMBER_UNKNOWN EQU 0
CYDEV_CHIP_MEMBER_USED EQU CYDEV_CHIP_MEMBER_4A
CYDEV_CHIP_DIE_EXPECT EQU CYDEV_CHIP_MEMBER_USED
CYDEV_CHIP_DIE_ACTUAL EQU CYDEV_CHIP_DIE_EXPECT
CYDEV_CHIP_REV_LEOPARD_ES1 EQU 0
CYDEV_CHIP_REV_LEOPARD_ES2 EQU 1
CYDEV_CHIP_REV_LEOPARD_ES3 EQU 3
CYDEV_CHIP_REV_LEOPARD_PRODUCTION EQU 3
CYDEV_CHIP_REV_PSOC4A_ES0 EQU 17
CYDEV_CHIP_REV_PSOC4A_PRODUCTION EQU 17
CYDEV_CHIP_REV_PSOC5LP_ES0 EQU 0
CYDEV_CHIP_REV_PSOC5LP_PRODUCTION EQU 0
CYDEV_CHIP_REV_PSOC5TM_ES0 EQU 0
CYDEV_CHIP_REV_PSOC5TM_ES1 EQU 1
CYDEV_CHIP_REV_PSOC5TM_PRODUCTION EQU 1
CYDEV_CHIP_REV_TMA4_ES EQU 17
CYDEV_CHIP_REV_TMA4_ES2 EQU 33
CYDEV_CHIP_REV_TMA4_PRODUCTION EQU 17
CYDEV_CHIP_REVISION_3A_ES1 EQU 0
CYDEV_CHIP_REVISION_3A_ES2 EQU 1
CYDEV_CHIP_REVISION_3A_ES3 EQU 3
CYDEV_CHIP_REVISION_3A_PRODUCTION EQU 3
CYDEV_CHIP_REVISION_4A_ES0 EQU 17
CYDEV_CHIP_REVISION_4A_PRODUCTION EQU 17
CYDEV_CHIP_REVISION_4C_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4D_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4E_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4F_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA EQU 0
CYDEV_CHIP_REVISION_4F_PRODUCTION_256K EQU 0
CYDEV_CHIP_REVISION_4G_ES EQU 17
CYDEV_CHIP_REVISION_4G_ES2 EQU 33
CYDEV_CHIP_REVISION_4G_PRODUCTION EQU 17
CYDEV_CHIP_REVISION_4H_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4I_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4J_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4K_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4L_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4M_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4N_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4O_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4P_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4Q_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4U_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_5A_ES0 EQU 0
CYDEV_CHIP_REVISION_5A_ES1 EQU 1
CYDEV_CHIP_REVISION_5A_PRODUCTION EQU 1
CYDEV_CHIP_REVISION_5B_ES0 EQU 0
CYDEV_CHIP_REVISION_5B_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_FM3_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_FM4_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_USED EQU CYDEV_CHIP_REVISION_4A_PRODUCTION
CYDEV_CHIP_REV_EXPECT EQU CYDEV_CHIP_REVISION_USED
CYDEV_CONFIG_READ_ACCELERATOR EQU 1
CYDEV_CONFIG_UNUSED_IO_AllowButWarn EQU 0
CYDEV_CONFIG_UNUSED_IO_AllowWithInfo EQU 1
CYDEV_CONFIG_UNUSED_IO_Disallowed EQU 2
CYDEV_CONFIG_UNUSED_IO EQU CYDEV_CONFIG_UNUSED_IO_Disallowed
CYDEV_CONFIGURATION_COMPRESSED EQU 1
CYDEV_CONFIGURATION_MODE_COMPRESSED EQU 0
CYDEV_CONFIGURATION_MODE EQU CYDEV_CONFIGURATION_MODE_COMPRESSED
CYDEV_CONFIGURATION_MODE_DMA EQU 2
CYDEV_CONFIGURATION_MODE_UNCOMPRESSED EQU 1
CYDEV_DEBUG_PROTECT_KILL EQU 4
CYDEV_DEBUG_PROTECT_OPEN EQU 1
CYDEV_DEBUG_PROTECT EQU CYDEV_DEBUG_PROTECT_OPEN
CYDEV_DEBUG_PROTECT_PROTECTED EQU 2
CYDEV_DEBUGGING_DPS_Disable EQU 3
CYDEV_DEBUGGING_DPS_SWD EQU 2
CYDEV_DEBUGGING_DPS EQU CYDEV_DEBUGGING_DPS_SWD
CYDEV_DEBUGGING_ENABLE EQU 1
CYDEV_DFT_SELECT_CLK0 EQU 1
CYDEV_DFT_SELECT_CLK1 EQU 2
CYDEV_HEAP_SIZE EQU 0x80
CYDEV_IMO_TRIMMED_BY_USB EQU 0
CYDEV_IMO_TRIMMED_BY_WCO EQU 0
CYDEV_IS_EXPORTING_CODE EQU 0
CYDEV_IS_IMPORTING_CODE EQU 0
CYDEV_PROJ_TYPE EQU 0
CYDEV_PROJ_TYPE_BOOTLOADER EQU 1
CYDEV_PROJ_TYPE_LAUNCHER EQU 5
CYDEV_PROJ_TYPE_LOADABLE EQU 2
CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER EQU 4
CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER EQU 3
CYDEV_PROJ_TYPE_STANDARD EQU 0
CYDEV_STACK_SIZE EQU 0x0400
CYDEV_USE_BUNDLED_CMSIS EQU 1
CYDEV_VARIABLE_VDDA EQU 1
CYDEV_VDDA_MV EQU 3300
CYDEV_VDDD_MV EQU 3300
CYDEV_WDT_GENERATE_ISR EQU 1
CYIPBLOCK_M0S8_CTBM_VERSION EQU 0
CYIPBLOCK_m0s8cpuss_VERSION EQU 0
CYIPBLOCK_m0s8csd_VERSION EQU 0
CYIPBLOCK_m0s8gpio2_VERSION EQU 0
CYIPBLOCK_m0s8hsiom4a_VERSION EQU 0
CYIPBLOCK_m0s8lcd_VERSION EQU 0
CYIPBLOCK_m0s8lpcomp_VERSION EQU 0
CYIPBLOCK_m0s8pclk_VERSION EQU 0
CYIPBLOCK_m0s8sar_VERSION EQU 0
CYIPBLOCK_m0s8scb_VERSION EQU 0
CYIPBLOCK_m0s8srssv2_VERSION EQU 1
CYIPBLOCK_m0s8tcpwm_VERSION EQU 0
CYIPBLOCK_m0s8udbif_VERSION EQU 0
CYIPBLOCK_S8_GPIO_VERSION EQU 2
CYDEV_BOOTLOADER_ENABLE EQU 0

#endif /* INCLUDED_CYFITTERIAR_INC */