/*******************************************************************************
* File Name: serial.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_serial_H)
#define CY_UART_serial_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define serial_RX_ENABLED                     (1u)
#define serial_TX_ENABLED                     (1u)
#define serial_HD_ENABLED                     (0u)
#define serial_RX_INTERRUPT_ENABLED           (0u)
#define serial_TX_INTERRUPT_ENABLED           (0u)
#define serial_INTERNAL_CLOCK_USED            (1u)
#define serial_RXHW_ADDRESS_ENABLED           (0u)
#define serial_OVER_SAMPLE_COUNT              (8u)
#define serial_PARITY_TYPE                    (0u)
#define serial_PARITY_TYPE_SW                 (0u)
#define serial_BREAK_DETECT                   (0u)
#define serial_BREAK_BITS_TX                  (13u)
#define serial_BREAK_BITS_RX                  (13u)
#define serial_TXCLKGEN_DP                    (1u)
#define serial_USE23POLLING                   (1u)
#define serial_FLOW_CONTROL                   (0u)
#define serial_CLK_FREQ                       (0u)
#define serial_TX_BUFFER_SIZE                 (4u)
#define serial_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define serial_CONTROL_REG_REMOVED            (0u)
#else
    #define serial_CONTROL_REG_REMOVED            (1u)
#endif /* End serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct serial_backupStruct_
{
    uint8 enableState;

    #if(serial_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End serial_CONTROL_REG_REMOVED */

} serial_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void serial_Start(void) ;
void serial_Stop(void) ;
uint8 serial_ReadControlRegister(void) ;
void serial_WriteControlRegister(uint8 control) ;

void serial_Init(void) ;
void serial_Enable(void) ;
void serial_SaveConfig(void) ;
void serial_RestoreConfig(void) ;
void serial_Sleep(void) ;
void serial_Wakeup(void) ;

/* Only if RX is enabled */
#if( (serial_RX_ENABLED) || (serial_HD_ENABLED) )

    #if (serial_RX_INTERRUPT_ENABLED)
        #define serial_EnableRxInt()  CyIntEnable (serial_RX_VECT_NUM)
        #define serial_DisableRxInt() CyIntDisable(serial_RX_VECT_NUM)
        CY_ISR_PROTO(serial_RXISR);
    #endif /* serial_RX_INTERRUPT_ENABLED */

    void serial_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void serial_SetRxAddress1(uint8 address) ;
    void serial_SetRxAddress2(uint8 address) ;

    void  serial_SetRxInterruptMode(uint8 intSrc) ;
    uint8 serial_ReadRxData(void) ;
    uint8 serial_ReadRxStatus(void) ;
    uint8 serial_GetChar(void) ;
    uint16 serial_GetByte(void) ;
    uint8 serial_GetRxBufferSize(void)
                                                            ;
    void serial_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define serial_GetRxInterruptSource   serial_ReadRxStatus

#endif /* End (serial_RX_ENABLED) || (serial_HD_ENABLED) */

/* Only if TX is enabled */
#if(serial_TX_ENABLED || serial_HD_ENABLED)

    #if(serial_TX_INTERRUPT_ENABLED)
        #define serial_EnableTxInt()  CyIntEnable (serial_TX_VECT_NUM)
        #define serial_DisableTxInt() CyIntDisable(serial_TX_VECT_NUM)
        #define serial_SetPendingTxInt() CyIntSetPending(serial_TX_VECT_NUM)
        #define serial_ClearPendingTxInt() CyIntClearPending(serial_TX_VECT_NUM)
        CY_ISR_PROTO(serial_TXISR);
    #endif /* serial_TX_INTERRUPT_ENABLED */

    void serial_SetTxInterruptMode(uint8 intSrc) ;
    void serial_WriteTxData(uint8 txDataByte) ;
    uint8 serial_ReadTxStatus(void) ;
    void serial_PutChar(uint8 txDataByte) ;
    void serial_PutString(const char8 string[]) ;
    void serial_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void serial_PutCRLF(uint8 txDataByte) ;
    void serial_ClearTxBuffer(void) ;
    void serial_SetTxAddressMode(uint8 addressMode) ;
    void serial_SendBreak(uint8 retMode) ;
    uint8 serial_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define serial_PutStringConst         serial_PutString
    #define serial_PutArrayConst          serial_PutArray
    #define serial_GetTxInterruptSource   serial_ReadTxStatus

#endif /* End serial_TX_ENABLED || serial_HD_ENABLED */

#if(serial_HD_ENABLED)
    void serial_LoadRxConfig(void) ;
    void serial_LoadTxConfig(void) ;
#endif /* End serial_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_serial) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    serial_CyBtldrCommStart(void) CYSMALL ;
    void    serial_CyBtldrCommStop(void) CYSMALL ;
    void    serial_CyBtldrCommReset(void) CYSMALL ;
    cystatus serial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus serial_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_serial)
        #define CyBtldrCommStart    serial_CyBtldrCommStart
        #define CyBtldrCommStop     serial_CyBtldrCommStop
        #define CyBtldrCommReset    serial_CyBtldrCommReset
        #define CyBtldrCommWrite    serial_CyBtldrCommWrite
        #define CyBtldrCommRead     serial_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_serial) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define serial_BYTE2BYTE_TIME_OUT (25u)
    #define serial_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define serial_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define serial_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define serial_SET_SPACE      (0x00u)
#define serial_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (serial_TX_ENABLED) || (serial_HD_ENABLED) )
    #if(serial_TX_INTERRUPT_ENABLED)
        #define serial_TX_VECT_NUM            (uint8)serial_TXInternalInterrupt__INTC_NUMBER
        #define serial_TX_PRIOR_NUM           (uint8)serial_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* serial_TX_INTERRUPT_ENABLED */

    #define serial_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define serial_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define serial_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(serial_TX_ENABLED)
        #define serial_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (serial_HD_ENABLED) */
        #define serial_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (serial_TX_ENABLED) */

    #define serial_TX_STS_COMPLETE            (uint8)(0x01u << serial_TX_STS_COMPLETE_SHIFT)
    #define serial_TX_STS_FIFO_EMPTY          (uint8)(0x01u << serial_TX_STS_FIFO_EMPTY_SHIFT)
    #define serial_TX_STS_FIFO_FULL           (uint8)(0x01u << serial_TX_STS_FIFO_FULL_SHIFT)
    #define serial_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << serial_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (serial_TX_ENABLED) || (serial_HD_ENABLED)*/

#if( (serial_RX_ENABLED) || (serial_HD_ENABLED) )
    #if(serial_RX_INTERRUPT_ENABLED)
        #define serial_RX_VECT_NUM            (uint8)serial_RXInternalInterrupt__INTC_NUMBER
        #define serial_RX_PRIOR_NUM           (uint8)serial_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* serial_RX_INTERRUPT_ENABLED */
    #define serial_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define serial_RX_STS_BREAK_SHIFT             (0x01u)
    #define serial_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define serial_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define serial_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define serial_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define serial_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define serial_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define serial_RX_STS_MRKSPC           (uint8)(0x01u << serial_RX_STS_MRKSPC_SHIFT)
    #define serial_RX_STS_BREAK            (uint8)(0x01u << serial_RX_STS_BREAK_SHIFT)
    #define serial_RX_STS_PAR_ERROR        (uint8)(0x01u << serial_RX_STS_PAR_ERROR_SHIFT)
    #define serial_RX_STS_STOP_ERROR       (uint8)(0x01u << serial_RX_STS_STOP_ERROR_SHIFT)
    #define serial_RX_STS_OVERRUN          (uint8)(0x01u << serial_RX_STS_OVERRUN_SHIFT)
    #define serial_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << serial_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define serial_RX_STS_ADDR_MATCH       (uint8)(0x01u << serial_RX_STS_ADDR_MATCH_SHIFT)
    #define serial_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << serial_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define serial_RX_HW_MASK                     (0x7Fu)
#endif /* End (serial_RX_ENABLED) || (serial_HD_ENABLED) */

/* Control Register definitions */
#define serial_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define serial_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define serial_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define serial_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define serial_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define serial_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define serial_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define serial_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define serial_CTRL_HD_SEND               (uint8)(0x01u << serial_CTRL_HD_SEND_SHIFT)
#define serial_CTRL_HD_SEND_BREAK         (uint8)(0x01u << serial_CTRL_HD_SEND_BREAK_SHIFT)
#define serial_CTRL_MARK                  (uint8)(0x01u << serial_CTRL_MARK_SHIFT)
#define serial_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << serial_CTRL_PARITY_TYPE0_SHIFT)
#define serial_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << serial_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define serial_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define serial_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define serial_SEND_BREAK                         (0x00u)
#define serial_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define serial_REINIT                             (0x02u)
#define serial_SEND_WAIT_REINIT                   (0x03u)

#define serial_OVER_SAMPLE_8                      (8u)
#define serial_OVER_SAMPLE_16                     (16u)

#define serial_BIT_CENTER                         (serial_OVER_SAMPLE_COUNT - 2u)

#define serial_FIFO_LENGTH                        (4u)
#define serial_NUMBER_OF_START_BIT                (1u)
#define serial_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define serial_TXBITCTR_BREAKBITS8X   ((serial_BREAK_BITS_TX * serial_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define serial_TXBITCTR_BREAKBITS ((serial_BREAK_BITS_TX * serial_OVER_SAMPLE_COUNT) - 1u)

#define serial_HALF_BIT_COUNT   \
                            (((serial_OVER_SAMPLE_COUNT / 2u) + (serial_USE23POLLING * 1u)) - 2u)
#if (serial_OVER_SAMPLE_COUNT == serial_OVER_SAMPLE_8)
    #define serial_HD_TXBITCTR_INIT   (((serial_BREAK_BITS_TX + \
                            serial_NUMBER_OF_START_BIT) * serial_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define serial_RXBITCTR_INIT  ((((serial_BREAK_BITS_RX + serial_NUMBER_OF_START_BIT) \
                            * serial_OVER_SAMPLE_COUNT) + serial_HALF_BIT_COUNT) - 1u)

#else /* serial_OVER_SAMPLE_COUNT == serial_OVER_SAMPLE_16 */
    #define serial_HD_TXBITCTR_INIT   ((8u * serial_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define serial_RXBITCTR_INIT      (((7u * serial_OVER_SAMPLE_COUNT) - 1u) + \
                                                      serial_HALF_BIT_COUNT)
#endif /* End serial_OVER_SAMPLE_COUNT */

#define serial_HD_RXBITCTR_INIT                   serial_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 serial_initVar;
#if (serial_TX_INTERRUPT_ENABLED && serial_TX_ENABLED)
    extern volatile uint8 serial_txBuffer[serial_TX_BUFFER_SIZE];
    extern volatile uint8 serial_txBufferRead;
    extern uint8 serial_txBufferWrite;
#endif /* (serial_TX_INTERRUPT_ENABLED && serial_TX_ENABLED) */
#if (serial_RX_INTERRUPT_ENABLED && (serial_RX_ENABLED || serial_HD_ENABLED))
    extern uint8 serial_errorStatus;
    extern volatile uint8 serial_rxBuffer[serial_RX_BUFFER_SIZE];
    extern volatile uint8 serial_rxBufferRead;
    extern volatile uint8 serial_rxBufferWrite;
    extern volatile uint8 serial_rxBufferLoopDetect;
    extern volatile uint8 serial_rxBufferOverflow;
    #if (serial_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 serial_rxAddressMode;
        extern volatile uint8 serial_rxAddressDetected;
    #endif /* (serial_RXHW_ADDRESS_ENABLED) */
#endif /* (serial_RX_INTERRUPT_ENABLED && (serial_RX_ENABLED || serial_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define serial__B_UART__AM_SW_BYTE_BYTE 1
#define serial__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define serial__B_UART__AM_HW_BYTE_BY_BYTE 3
#define serial__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define serial__B_UART__AM_NONE 0

#define serial__B_UART__NONE_REVB 0
#define serial__B_UART__EVEN_REVB 1
#define serial__B_UART__ODD_REVB 2
#define serial__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define serial_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define serial_NUMBER_OF_STOP_BITS    (1u)

#if (serial_RXHW_ADDRESS_ENABLED)
    #define serial_RX_ADDRESS_MODE    (0u)
    #define serial_RX_HW_ADDRESS1     (0u)
    #define serial_RX_HW_ADDRESS2     (0u)
#endif /* (serial_RXHW_ADDRESS_ENABLED) */

#define serial_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << serial_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << serial_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << serial_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << serial_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << serial_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << serial_RX_STS_BREAK_SHIFT) \
                                        | (0 << serial_RX_STS_OVERRUN_SHIFT))

#define serial_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << serial_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << serial_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << serial_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << serial_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define serial_CONTROL_REG \
                            (* (reg8 *) serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define serial_CONTROL_PTR \
                            (  (reg8 *) serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(serial_TX_ENABLED)
    #define serial_TXDATA_REG          (* (reg8 *) serial_BUART_sTX_TxShifter_u0__F0_REG)
    #define serial_TXDATA_PTR          (  (reg8 *) serial_BUART_sTX_TxShifter_u0__F0_REG)
    #define serial_TXDATA_AUX_CTL_REG  (* (reg8 *) serial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define serial_TXDATA_AUX_CTL_PTR  (  (reg8 *) serial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define serial_TXSTATUS_REG        (* (reg8 *) serial_BUART_sTX_TxSts__STATUS_REG)
    #define serial_TXSTATUS_PTR        (  (reg8 *) serial_BUART_sTX_TxSts__STATUS_REG)
    #define serial_TXSTATUS_MASK_REG   (* (reg8 *) serial_BUART_sTX_TxSts__MASK_REG)
    #define serial_TXSTATUS_MASK_PTR   (  (reg8 *) serial_BUART_sTX_TxSts__MASK_REG)
    #define serial_TXSTATUS_ACTL_REG   (* (reg8 *) serial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define serial_TXSTATUS_ACTL_PTR   (  (reg8 *) serial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(serial_TXCLKGEN_DP)
        #define serial_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) serial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define serial_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) serial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define serial_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) serial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define serial_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) serial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define serial_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) serial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define serial_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) serial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define serial_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) serial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define serial_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) serial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define serial_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) serial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define serial_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) serial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* serial_TXCLKGEN_DP */

#endif /* End serial_TX_ENABLED */

#if(serial_HD_ENABLED)

    #define serial_TXDATA_REG             (* (reg8 *) serial_BUART_sRX_RxShifter_u0__F1_REG )
    #define serial_TXDATA_PTR             (  (reg8 *) serial_BUART_sRX_RxShifter_u0__F1_REG )
    #define serial_TXDATA_AUX_CTL_REG     (* (reg8 *) serial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define serial_TXDATA_AUX_CTL_PTR     (  (reg8 *) serial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define serial_TXSTATUS_REG           (* (reg8 *) serial_BUART_sRX_RxSts__STATUS_REG )
    #define serial_TXSTATUS_PTR           (  (reg8 *) serial_BUART_sRX_RxSts__STATUS_REG )
    #define serial_TXSTATUS_MASK_REG      (* (reg8 *) serial_BUART_sRX_RxSts__MASK_REG )
    #define serial_TXSTATUS_MASK_PTR      (  (reg8 *) serial_BUART_sRX_RxSts__MASK_REG )
    #define serial_TXSTATUS_ACTL_REG      (* (reg8 *) serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define serial_TXSTATUS_ACTL_PTR      (  (reg8 *) serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End serial_HD_ENABLED */

#if( (serial_RX_ENABLED) || (serial_HD_ENABLED) )
    #define serial_RXDATA_REG             (* (reg8 *) serial_BUART_sRX_RxShifter_u0__F0_REG )
    #define serial_RXDATA_PTR             (  (reg8 *) serial_BUART_sRX_RxShifter_u0__F0_REG )
    #define serial_RXADDRESS1_REG         (* (reg8 *) serial_BUART_sRX_RxShifter_u0__D0_REG )
    #define serial_RXADDRESS1_PTR         (  (reg8 *) serial_BUART_sRX_RxShifter_u0__D0_REG )
    #define serial_RXADDRESS2_REG         (* (reg8 *) serial_BUART_sRX_RxShifter_u0__D1_REG )
    #define serial_RXADDRESS2_PTR         (  (reg8 *) serial_BUART_sRX_RxShifter_u0__D1_REG )
    #define serial_RXDATA_AUX_CTL_REG     (* (reg8 *) serial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define serial_RXBITCTR_PERIOD_REG    (* (reg8 *) serial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define serial_RXBITCTR_PERIOD_PTR    (  (reg8 *) serial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define serial_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) serial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define serial_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) serial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define serial_RXBITCTR_COUNTER_REG   (* (reg8 *) serial_BUART_sRX_RxBitCounter__COUNT_REG )
    #define serial_RXBITCTR_COUNTER_PTR   (  (reg8 *) serial_BUART_sRX_RxBitCounter__COUNT_REG )

    #define serial_RXSTATUS_REG           (* (reg8 *) serial_BUART_sRX_RxSts__STATUS_REG )
    #define serial_RXSTATUS_PTR           (  (reg8 *) serial_BUART_sRX_RxSts__STATUS_REG )
    #define serial_RXSTATUS_MASK_REG      (* (reg8 *) serial_BUART_sRX_RxSts__MASK_REG )
    #define serial_RXSTATUS_MASK_PTR      (  (reg8 *) serial_BUART_sRX_RxSts__MASK_REG )
    #define serial_RXSTATUS_ACTL_REG      (* (reg8 *) serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define serial_RXSTATUS_ACTL_PTR      (  (reg8 *) serial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (serial_RX_ENABLED) || (serial_HD_ENABLED) */

#if(serial_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define serial_INTCLOCK_CLKEN_REG     (* (reg8 *) serial_IntClock__PM_ACT_CFG)
    #define serial_INTCLOCK_CLKEN_PTR     (  (reg8 *) serial_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define serial_INTCLOCK_CLKEN_MASK    serial_IntClock__PM_ACT_MSK
#endif /* End serial_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(serial_TX_ENABLED)
    #define serial_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End serial_TX_ENABLED */

#if(serial_HD_ENABLED)
    #define serial_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End serial_HD_ENABLED */

#if( (serial_RX_ENABLED) || (serial_HD_ENABLED) )
    #define serial_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (serial_RX_ENABLED) || (serial_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define serial_WAIT_1_MS      serial_BL_CHK_DELAY_MS   

#define serial_TXBUFFERSIZE   serial_TX_BUFFER_SIZE
#define serial_RXBUFFERSIZE   serial_RX_BUFFER_SIZE

#if (serial_RXHW_ADDRESS_ENABLED)
    #define serial_RXADDRESSMODE  serial_RX_ADDRESS_MODE
    #define serial_RXHWADDRESS1   serial_RX_HW_ADDRESS1
    #define serial_RXHWADDRESS2   serial_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define serial_RXAddressMode  serial_RXADDRESSMODE
#endif /* (serial_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define serial_initvar                    serial_initVar

#define serial_RX_Enabled                 serial_RX_ENABLED
#define serial_TX_Enabled                 serial_TX_ENABLED
#define serial_HD_Enabled                 serial_HD_ENABLED
#define serial_RX_IntInterruptEnabled     serial_RX_INTERRUPT_ENABLED
#define serial_TX_IntInterruptEnabled     serial_TX_INTERRUPT_ENABLED
#define serial_InternalClockUsed          serial_INTERNAL_CLOCK_USED
#define serial_RXHW_Address_Enabled       serial_RXHW_ADDRESS_ENABLED
#define serial_OverSampleCount            serial_OVER_SAMPLE_COUNT
#define serial_ParityType                 serial_PARITY_TYPE

#if( serial_TX_ENABLED && (serial_TXBUFFERSIZE > serial_FIFO_LENGTH))
    #define serial_TXBUFFER               serial_txBuffer
    #define serial_TXBUFFERREAD           serial_txBufferRead
    #define serial_TXBUFFERWRITE          serial_txBufferWrite
#endif /* End serial_TX_ENABLED */
#if( ( serial_RX_ENABLED || serial_HD_ENABLED ) && \
     (serial_RXBUFFERSIZE > serial_FIFO_LENGTH) )
    #define serial_RXBUFFER               serial_rxBuffer
    #define serial_RXBUFFERREAD           serial_rxBufferRead
    #define serial_RXBUFFERWRITE          serial_rxBufferWrite
    #define serial_RXBUFFERLOOPDETECT     serial_rxBufferLoopDetect
    #define serial_RXBUFFER_OVERFLOW      serial_rxBufferOverflow
#endif /* End serial_RX_ENABLED */

#ifdef serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define serial_CONTROL                serial_CONTROL_REG
#endif /* End serial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(serial_TX_ENABLED)
    #define serial_TXDATA                 serial_TXDATA_REG
    #define serial_TXSTATUS               serial_TXSTATUS_REG
    #define serial_TXSTATUS_MASK          serial_TXSTATUS_MASK_REG
    #define serial_TXSTATUS_ACTL          serial_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(serial_TXCLKGEN_DP)
        #define serial_TXBITCLKGEN_CTR        serial_TXBITCLKGEN_CTR_REG
        #define serial_TXBITCLKTX_COMPLETE    serial_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define serial_TXBITCTR_PERIOD        serial_TXBITCTR_PERIOD_REG
        #define serial_TXBITCTR_CONTROL       serial_TXBITCTR_CONTROL_REG
        #define serial_TXBITCTR_COUNTER       serial_TXBITCTR_COUNTER_REG
    #endif /* serial_TXCLKGEN_DP */
#endif /* End serial_TX_ENABLED */

#if(serial_HD_ENABLED)
    #define serial_TXDATA                 serial_TXDATA_REG
    #define serial_TXSTATUS               serial_TXSTATUS_REG
    #define serial_TXSTATUS_MASK          serial_TXSTATUS_MASK_REG
    #define serial_TXSTATUS_ACTL          serial_TXSTATUS_ACTL_REG
#endif /* End serial_HD_ENABLED */

#if( (serial_RX_ENABLED) || (serial_HD_ENABLED) )
    #define serial_RXDATA                 serial_RXDATA_REG
    #define serial_RXADDRESS1             serial_RXADDRESS1_REG
    #define serial_RXADDRESS2             serial_RXADDRESS2_REG
    #define serial_RXBITCTR_PERIOD        serial_RXBITCTR_PERIOD_REG
    #define serial_RXBITCTR_CONTROL       serial_RXBITCTR_CONTROL_REG
    #define serial_RXBITCTR_COUNTER       serial_RXBITCTR_COUNTER_REG
    #define serial_RXSTATUS               serial_RXSTATUS_REG
    #define serial_RXSTATUS_MASK          serial_RXSTATUS_MASK_REG
    #define serial_RXSTATUS_ACTL          serial_RXSTATUS_ACTL_REG
#endif /* End  (serial_RX_ENABLED) || (serial_HD_ENABLED) */

#if(serial_INTERNAL_CLOCK_USED)
    #define serial_INTCLOCK_CLKEN         serial_INTCLOCK_CLKEN_REG
#endif /* End serial_INTERNAL_CLOCK_USED */

#define serial_WAIT_FOR_COMLETE_REINIT    serial_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_serial_H */


/* [] END OF FILE */
