/*******************************************************************************
* File Name: xbeeSerial.h
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


#if !defined(CY_UART_xbeeSerial_H)
#define CY_UART_xbeeSerial_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define xbeeSerial_RX_ENABLED                     (1u)
#define xbeeSerial_TX_ENABLED                     (1u)
#define xbeeSerial_HD_ENABLED                     (0u)
#define xbeeSerial_RX_INTERRUPT_ENABLED           (0u)
#define xbeeSerial_TX_INTERRUPT_ENABLED           (0u)
#define xbeeSerial_INTERNAL_CLOCK_USED            (1u)
#define xbeeSerial_RXHW_ADDRESS_ENABLED           (0u)
#define xbeeSerial_OVER_SAMPLE_COUNT              (8u)
#define xbeeSerial_PARITY_TYPE                    (0u)
#define xbeeSerial_PARITY_TYPE_SW                 (0u)
#define xbeeSerial_BREAK_DETECT                   (0u)
#define xbeeSerial_BREAK_BITS_TX                  (13u)
#define xbeeSerial_BREAK_BITS_RX                  (13u)
#define xbeeSerial_TXCLKGEN_DP                    (1u)
#define xbeeSerial_USE23POLLING                   (1u)
#define xbeeSerial_FLOW_CONTROL                   (0u)
#define xbeeSerial_CLK_FREQ                       (0u)
#define xbeeSerial_TX_BUFFER_SIZE                 (4u)
#define xbeeSerial_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define xbeeSerial_CONTROL_REG_REMOVED            (0u)
#else
    #define xbeeSerial_CONTROL_REG_REMOVED            (1u)
#endif /* End xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct xbeeSerial_backupStruct_
{
    uint8 enableState;

    #if(xbeeSerial_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End xbeeSerial_CONTROL_REG_REMOVED */

} xbeeSerial_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void xbeeSerial_Start(void) ;
void xbeeSerial_Stop(void) ;
uint8 xbeeSerial_ReadControlRegister(void) ;
void xbeeSerial_WriteControlRegister(uint8 control) ;

void xbeeSerial_Init(void) ;
void xbeeSerial_Enable(void) ;
void xbeeSerial_SaveConfig(void) ;
void xbeeSerial_RestoreConfig(void) ;
void xbeeSerial_Sleep(void) ;
void xbeeSerial_Wakeup(void) ;

/* Only if RX is enabled */
#if( (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) )

    #if (xbeeSerial_RX_INTERRUPT_ENABLED)
        #define xbeeSerial_EnableRxInt()  CyIntEnable (xbeeSerial_RX_VECT_NUM)
        #define xbeeSerial_DisableRxInt() CyIntDisable(xbeeSerial_RX_VECT_NUM)
        CY_ISR_PROTO(xbeeSerial_RXISR);
    #endif /* xbeeSerial_RX_INTERRUPT_ENABLED */

    void xbeeSerial_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void xbeeSerial_SetRxAddress1(uint8 address) ;
    void xbeeSerial_SetRxAddress2(uint8 address) ;

    void  xbeeSerial_SetRxInterruptMode(uint8 intSrc) ;
    uint8 xbeeSerial_ReadRxData(void) ;
    uint8 xbeeSerial_ReadRxStatus(void) ;
    uint8 xbeeSerial_GetChar(void) ;
    uint16 xbeeSerial_GetByte(void) ;
    uint8 xbeeSerial_GetRxBufferSize(void)
                                                            ;
    void xbeeSerial_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define xbeeSerial_GetRxInterruptSource   xbeeSerial_ReadRxStatus

#endif /* End (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) */

/* Only if TX is enabled */
#if(xbeeSerial_TX_ENABLED || xbeeSerial_HD_ENABLED)

    #if(xbeeSerial_TX_INTERRUPT_ENABLED)
        #define xbeeSerial_EnableTxInt()  CyIntEnable (xbeeSerial_TX_VECT_NUM)
        #define xbeeSerial_DisableTxInt() CyIntDisable(xbeeSerial_TX_VECT_NUM)
        #define xbeeSerial_SetPendingTxInt() CyIntSetPending(xbeeSerial_TX_VECT_NUM)
        #define xbeeSerial_ClearPendingTxInt() CyIntClearPending(xbeeSerial_TX_VECT_NUM)
        CY_ISR_PROTO(xbeeSerial_TXISR);
    #endif /* xbeeSerial_TX_INTERRUPT_ENABLED */

    void xbeeSerial_SetTxInterruptMode(uint8 intSrc) ;
    void xbeeSerial_WriteTxData(uint8 txDataByte) ;
    uint8 xbeeSerial_ReadTxStatus(void) ;
    void xbeeSerial_PutChar(uint8 txDataByte) ;
    void xbeeSerial_PutString(const char8 string[]) ;
    void xbeeSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void xbeeSerial_PutCRLF(uint8 txDataByte) ;
    void xbeeSerial_ClearTxBuffer(void) ;
    void xbeeSerial_SetTxAddressMode(uint8 addressMode) ;
    void xbeeSerial_SendBreak(uint8 retMode) ;
    uint8 xbeeSerial_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define xbeeSerial_PutStringConst         xbeeSerial_PutString
    #define xbeeSerial_PutArrayConst          xbeeSerial_PutArray
    #define xbeeSerial_GetTxInterruptSource   xbeeSerial_ReadTxStatus

#endif /* End xbeeSerial_TX_ENABLED || xbeeSerial_HD_ENABLED */

#if(xbeeSerial_HD_ENABLED)
    void xbeeSerial_LoadRxConfig(void) ;
    void xbeeSerial_LoadTxConfig(void) ;
#endif /* End xbeeSerial_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_xbeeSerial) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    xbeeSerial_CyBtldrCommStart(void) CYSMALL ;
    void    xbeeSerial_CyBtldrCommStop(void) CYSMALL ;
    void    xbeeSerial_CyBtldrCommReset(void) CYSMALL ;
    cystatus xbeeSerial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus xbeeSerial_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_xbeeSerial)
        #define CyBtldrCommStart    xbeeSerial_CyBtldrCommStart
        #define CyBtldrCommStop     xbeeSerial_CyBtldrCommStop
        #define CyBtldrCommReset    xbeeSerial_CyBtldrCommReset
        #define CyBtldrCommWrite    xbeeSerial_CyBtldrCommWrite
        #define CyBtldrCommRead     xbeeSerial_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_xbeeSerial) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define xbeeSerial_BYTE2BYTE_TIME_OUT (25u)
    #define xbeeSerial_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define xbeeSerial_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define xbeeSerial_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define xbeeSerial_SET_SPACE      (0x00u)
#define xbeeSerial_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (xbeeSerial_TX_ENABLED) || (xbeeSerial_HD_ENABLED) )
    #if(xbeeSerial_TX_INTERRUPT_ENABLED)
        #define xbeeSerial_TX_VECT_NUM            (uint8)xbeeSerial_TXInternalInterrupt__INTC_NUMBER
        #define xbeeSerial_TX_PRIOR_NUM           (uint8)xbeeSerial_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* xbeeSerial_TX_INTERRUPT_ENABLED */

    #define xbeeSerial_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define xbeeSerial_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define xbeeSerial_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(xbeeSerial_TX_ENABLED)
        #define xbeeSerial_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (xbeeSerial_HD_ENABLED) */
        #define xbeeSerial_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (xbeeSerial_TX_ENABLED) */

    #define xbeeSerial_TX_STS_COMPLETE            (uint8)(0x01u << xbeeSerial_TX_STS_COMPLETE_SHIFT)
    #define xbeeSerial_TX_STS_FIFO_EMPTY          (uint8)(0x01u << xbeeSerial_TX_STS_FIFO_EMPTY_SHIFT)
    #define xbeeSerial_TX_STS_FIFO_FULL           (uint8)(0x01u << xbeeSerial_TX_STS_FIFO_FULL_SHIFT)
    #define xbeeSerial_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << xbeeSerial_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (xbeeSerial_TX_ENABLED) || (xbeeSerial_HD_ENABLED)*/

#if( (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) )
    #if(xbeeSerial_RX_INTERRUPT_ENABLED)
        #define xbeeSerial_RX_VECT_NUM            (uint8)xbeeSerial_RXInternalInterrupt__INTC_NUMBER
        #define xbeeSerial_RX_PRIOR_NUM           (uint8)xbeeSerial_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* xbeeSerial_RX_INTERRUPT_ENABLED */
    #define xbeeSerial_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define xbeeSerial_RX_STS_BREAK_SHIFT             (0x01u)
    #define xbeeSerial_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define xbeeSerial_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define xbeeSerial_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define xbeeSerial_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define xbeeSerial_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define xbeeSerial_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define xbeeSerial_RX_STS_MRKSPC           (uint8)(0x01u << xbeeSerial_RX_STS_MRKSPC_SHIFT)
    #define xbeeSerial_RX_STS_BREAK            (uint8)(0x01u << xbeeSerial_RX_STS_BREAK_SHIFT)
    #define xbeeSerial_RX_STS_PAR_ERROR        (uint8)(0x01u << xbeeSerial_RX_STS_PAR_ERROR_SHIFT)
    #define xbeeSerial_RX_STS_STOP_ERROR       (uint8)(0x01u << xbeeSerial_RX_STS_STOP_ERROR_SHIFT)
    #define xbeeSerial_RX_STS_OVERRUN          (uint8)(0x01u << xbeeSerial_RX_STS_OVERRUN_SHIFT)
    #define xbeeSerial_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << xbeeSerial_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define xbeeSerial_RX_STS_ADDR_MATCH       (uint8)(0x01u << xbeeSerial_RX_STS_ADDR_MATCH_SHIFT)
    #define xbeeSerial_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << xbeeSerial_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define xbeeSerial_RX_HW_MASK                     (0x7Fu)
#endif /* End (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) */

/* Control Register definitions */
#define xbeeSerial_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define xbeeSerial_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define xbeeSerial_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define xbeeSerial_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define xbeeSerial_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define xbeeSerial_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define xbeeSerial_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define xbeeSerial_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define xbeeSerial_CTRL_HD_SEND               (uint8)(0x01u << xbeeSerial_CTRL_HD_SEND_SHIFT)
#define xbeeSerial_CTRL_HD_SEND_BREAK         (uint8)(0x01u << xbeeSerial_CTRL_HD_SEND_BREAK_SHIFT)
#define xbeeSerial_CTRL_MARK                  (uint8)(0x01u << xbeeSerial_CTRL_MARK_SHIFT)
#define xbeeSerial_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << xbeeSerial_CTRL_PARITY_TYPE0_SHIFT)
#define xbeeSerial_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << xbeeSerial_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define xbeeSerial_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define xbeeSerial_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define xbeeSerial_SEND_BREAK                         (0x00u)
#define xbeeSerial_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define xbeeSerial_REINIT                             (0x02u)
#define xbeeSerial_SEND_WAIT_REINIT                   (0x03u)

#define xbeeSerial_OVER_SAMPLE_8                      (8u)
#define xbeeSerial_OVER_SAMPLE_16                     (16u)

#define xbeeSerial_BIT_CENTER                         (xbeeSerial_OVER_SAMPLE_COUNT - 2u)

#define xbeeSerial_FIFO_LENGTH                        (4u)
#define xbeeSerial_NUMBER_OF_START_BIT                (1u)
#define xbeeSerial_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define xbeeSerial_TXBITCTR_BREAKBITS8X   ((xbeeSerial_BREAK_BITS_TX * xbeeSerial_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define xbeeSerial_TXBITCTR_BREAKBITS ((xbeeSerial_BREAK_BITS_TX * xbeeSerial_OVER_SAMPLE_COUNT) - 1u)

#define xbeeSerial_HALF_BIT_COUNT   \
                            (((xbeeSerial_OVER_SAMPLE_COUNT / 2u) + (xbeeSerial_USE23POLLING * 1u)) - 2u)
#if (xbeeSerial_OVER_SAMPLE_COUNT == xbeeSerial_OVER_SAMPLE_8)
    #define xbeeSerial_HD_TXBITCTR_INIT   (((xbeeSerial_BREAK_BITS_TX + \
                            xbeeSerial_NUMBER_OF_START_BIT) * xbeeSerial_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define xbeeSerial_RXBITCTR_INIT  ((((xbeeSerial_BREAK_BITS_RX + xbeeSerial_NUMBER_OF_START_BIT) \
                            * xbeeSerial_OVER_SAMPLE_COUNT) + xbeeSerial_HALF_BIT_COUNT) - 1u)

#else /* xbeeSerial_OVER_SAMPLE_COUNT == xbeeSerial_OVER_SAMPLE_16 */
    #define xbeeSerial_HD_TXBITCTR_INIT   ((8u * xbeeSerial_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define xbeeSerial_RXBITCTR_INIT      (((7u * xbeeSerial_OVER_SAMPLE_COUNT) - 1u) + \
                                                      xbeeSerial_HALF_BIT_COUNT)
#endif /* End xbeeSerial_OVER_SAMPLE_COUNT */

#define xbeeSerial_HD_RXBITCTR_INIT                   xbeeSerial_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 xbeeSerial_initVar;
#if (xbeeSerial_TX_INTERRUPT_ENABLED && xbeeSerial_TX_ENABLED)
    extern volatile uint8 xbeeSerial_txBuffer[xbeeSerial_TX_BUFFER_SIZE];
    extern volatile uint8 xbeeSerial_txBufferRead;
    extern uint8 xbeeSerial_txBufferWrite;
#endif /* (xbeeSerial_TX_INTERRUPT_ENABLED && xbeeSerial_TX_ENABLED) */
#if (xbeeSerial_RX_INTERRUPT_ENABLED && (xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED))
    extern uint8 xbeeSerial_errorStatus;
    extern volatile uint8 xbeeSerial_rxBuffer[xbeeSerial_RX_BUFFER_SIZE];
    extern volatile uint8 xbeeSerial_rxBufferRead;
    extern volatile uint8 xbeeSerial_rxBufferWrite;
    extern volatile uint8 xbeeSerial_rxBufferLoopDetect;
    extern volatile uint8 xbeeSerial_rxBufferOverflow;
    #if (xbeeSerial_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 xbeeSerial_rxAddressMode;
        extern volatile uint8 xbeeSerial_rxAddressDetected;
    #endif /* (xbeeSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (xbeeSerial_RX_INTERRUPT_ENABLED && (xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define xbeeSerial__B_UART__AM_SW_BYTE_BYTE 1
#define xbeeSerial__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define xbeeSerial__B_UART__AM_HW_BYTE_BY_BYTE 3
#define xbeeSerial__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define xbeeSerial__B_UART__AM_NONE 0

#define xbeeSerial__B_UART__NONE_REVB 0
#define xbeeSerial__B_UART__EVEN_REVB 1
#define xbeeSerial__B_UART__ODD_REVB 2
#define xbeeSerial__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define xbeeSerial_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define xbeeSerial_NUMBER_OF_STOP_BITS    (1u)

#if (xbeeSerial_RXHW_ADDRESS_ENABLED)
    #define xbeeSerial_RX_ADDRESS_MODE    (0u)
    #define xbeeSerial_RX_HW_ADDRESS1     (0u)
    #define xbeeSerial_RX_HW_ADDRESS2     (0u)
#endif /* (xbeeSerial_RXHW_ADDRESS_ENABLED) */

#define xbeeSerial_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << xbeeSerial_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << xbeeSerial_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << xbeeSerial_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << xbeeSerial_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << xbeeSerial_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << xbeeSerial_RX_STS_BREAK_SHIFT) \
                                        | (0 << xbeeSerial_RX_STS_OVERRUN_SHIFT))

#define xbeeSerial_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << xbeeSerial_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << xbeeSerial_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << xbeeSerial_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << xbeeSerial_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define xbeeSerial_CONTROL_REG \
                            (* (reg8 *) xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define xbeeSerial_CONTROL_PTR \
                            (  (reg8 *) xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(xbeeSerial_TX_ENABLED)
    #define xbeeSerial_TXDATA_REG          (* (reg8 *) xbeeSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define xbeeSerial_TXDATA_PTR          (  (reg8 *) xbeeSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define xbeeSerial_TXDATA_AUX_CTL_REG  (* (reg8 *) xbeeSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define xbeeSerial_TXDATA_AUX_CTL_PTR  (  (reg8 *) xbeeSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define xbeeSerial_TXSTATUS_REG        (* (reg8 *) xbeeSerial_BUART_sTX_TxSts__STATUS_REG)
    #define xbeeSerial_TXSTATUS_PTR        (  (reg8 *) xbeeSerial_BUART_sTX_TxSts__STATUS_REG)
    #define xbeeSerial_TXSTATUS_MASK_REG   (* (reg8 *) xbeeSerial_BUART_sTX_TxSts__MASK_REG)
    #define xbeeSerial_TXSTATUS_MASK_PTR   (  (reg8 *) xbeeSerial_BUART_sTX_TxSts__MASK_REG)
    #define xbeeSerial_TXSTATUS_ACTL_REG   (* (reg8 *) xbeeSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define xbeeSerial_TXSTATUS_ACTL_PTR   (  (reg8 *) xbeeSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(xbeeSerial_TXCLKGEN_DP)
        #define xbeeSerial_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define xbeeSerial_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define xbeeSerial_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define xbeeSerial_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define xbeeSerial_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define xbeeSerial_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define xbeeSerial_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define xbeeSerial_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define xbeeSerial_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define xbeeSerial_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) xbeeSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* xbeeSerial_TXCLKGEN_DP */

#endif /* End xbeeSerial_TX_ENABLED */

#if(xbeeSerial_HD_ENABLED)

    #define xbeeSerial_TXDATA_REG             (* (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define xbeeSerial_TXDATA_PTR             (  (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define xbeeSerial_TXDATA_AUX_CTL_REG     (* (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define xbeeSerial_TXDATA_AUX_CTL_PTR     (  (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define xbeeSerial_TXSTATUS_REG           (* (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_REG )
    #define xbeeSerial_TXSTATUS_PTR           (  (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_REG )
    #define xbeeSerial_TXSTATUS_MASK_REG      (* (reg8 *) xbeeSerial_BUART_sRX_RxSts__MASK_REG )
    #define xbeeSerial_TXSTATUS_MASK_PTR      (  (reg8 *) xbeeSerial_BUART_sRX_RxSts__MASK_REG )
    #define xbeeSerial_TXSTATUS_ACTL_REG      (* (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define xbeeSerial_TXSTATUS_ACTL_PTR      (  (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End xbeeSerial_HD_ENABLED */

#if( (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) )
    #define xbeeSerial_RXDATA_REG             (* (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define xbeeSerial_RXDATA_PTR             (  (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define xbeeSerial_RXADDRESS1_REG         (* (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define xbeeSerial_RXADDRESS1_PTR         (  (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define xbeeSerial_RXADDRESS2_REG         (* (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define xbeeSerial_RXADDRESS2_PTR         (  (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define xbeeSerial_RXDATA_AUX_CTL_REG     (* (reg8 *) xbeeSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define xbeeSerial_RXBITCTR_PERIOD_REG    (* (reg8 *) xbeeSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define xbeeSerial_RXBITCTR_PERIOD_PTR    (  (reg8 *) xbeeSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define xbeeSerial_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) xbeeSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define xbeeSerial_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) xbeeSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define xbeeSerial_RXBITCTR_COUNTER_REG   (* (reg8 *) xbeeSerial_BUART_sRX_RxBitCounter__COUNT_REG )
    #define xbeeSerial_RXBITCTR_COUNTER_PTR   (  (reg8 *) xbeeSerial_BUART_sRX_RxBitCounter__COUNT_REG )

    #define xbeeSerial_RXSTATUS_REG           (* (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_REG )
    #define xbeeSerial_RXSTATUS_PTR           (  (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_REG )
    #define xbeeSerial_RXSTATUS_MASK_REG      (* (reg8 *) xbeeSerial_BUART_sRX_RxSts__MASK_REG )
    #define xbeeSerial_RXSTATUS_MASK_PTR      (  (reg8 *) xbeeSerial_BUART_sRX_RxSts__MASK_REG )
    #define xbeeSerial_RXSTATUS_ACTL_REG      (* (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define xbeeSerial_RXSTATUS_ACTL_PTR      (  (reg8 *) xbeeSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) */

#if(xbeeSerial_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define xbeeSerial_INTCLOCK_CLKEN_REG     (* (reg8 *) xbeeSerial_IntClock__PM_ACT_CFG)
    #define xbeeSerial_INTCLOCK_CLKEN_PTR     (  (reg8 *) xbeeSerial_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define xbeeSerial_INTCLOCK_CLKEN_MASK    xbeeSerial_IntClock__PM_ACT_MSK
#endif /* End xbeeSerial_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(xbeeSerial_TX_ENABLED)
    #define xbeeSerial_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End xbeeSerial_TX_ENABLED */

#if(xbeeSerial_HD_ENABLED)
    #define xbeeSerial_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End xbeeSerial_HD_ENABLED */

#if( (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) )
    #define xbeeSerial_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define xbeeSerial_WAIT_1_MS      xbeeSerial_BL_CHK_DELAY_MS   

#define xbeeSerial_TXBUFFERSIZE   xbeeSerial_TX_BUFFER_SIZE
#define xbeeSerial_RXBUFFERSIZE   xbeeSerial_RX_BUFFER_SIZE

#if (xbeeSerial_RXHW_ADDRESS_ENABLED)
    #define xbeeSerial_RXADDRESSMODE  xbeeSerial_RX_ADDRESS_MODE
    #define xbeeSerial_RXHWADDRESS1   xbeeSerial_RX_HW_ADDRESS1
    #define xbeeSerial_RXHWADDRESS2   xbeeSerial_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define xbeeSerial_RXAddressMode  xbeeSerial_RXADDRESSMODE
#endif /* (xbeeSerial_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define xbeeSerial_initvar                    xbeeSerial_initVar

#define xbeeSerial_RX_Enabled                 xbeeSerial_RX_ENABLED
#define xbeeSerial_TX_Enabled                 xbeeSerial_TX_ENABLED
#define xbeeSerial_HD_Enabled                 xbeeSerial_HD_ENABLED
#define xbeeSerial_RX_IntInterruptEnabled     xbeeSerial_RX_INTERRUPT_ENABLED
#define xbeeSerial_TX_IntInterruptEnabled     xbeeSerial_TX_INTERRUPT_ENABLED
#define xbeeSerial_InternalClockUsed          xbeeSerial_INTERNAL_CLOCK_USED
#define xbeeSerial_RXHW_Address_Enabled       xbeeSerial_RXHW_ADDRESS_ENABLED
#define xbeeSerial_OverSampleCount            xbeeSerial_OVER_SAMPLE_COUNT
#define xbeeSerial_ParityType                 xbeeSerial_PARITY_TYPE

#if( xbeeSerial_TX_ENABLED && (xbeeSerial_TXBUFFERSIZE > xbeeSerial_FIFO_LENGTH))
    #define xbeeSerial_TXBUFFER               xbeeSerial_txBuffer
    #define xbeeSerial_TXBUFFERREAD           xbeeSerial_txBufferRead
    #define xbeeSerial_TXBUFFERWRITE          xbeeSerial_txBufferWrite
#endif /* End xbeeSerial_TX_ENABLED */
#if( ( xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED ) && \
     (xbeeSerial_RXBUFFERSIZE > xbeeSerial_FIFO_LENGTH) )
    #define xbeeSerial_RXBUFFER               xbeeSerial_rxBuffer
    #define xbeeSerial_RXBUFFERREAD           xbeeSerial_rxBufferRead
    #define xbeeSerial_RXBUFFERWRITE          xbeeSerial_rxBufferWrite
    #define xbeeSerial_RXBUFFERLOOPDETECT     xbeeSerial_rxBufferLoopDetect
    #define xbeeSerial_RXBUFFER_OVERFLOW      xbeeSerial_rxBufferOverflow
#endif /* End xbeeSerial_RX_ENABLED */

#ifdef xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define xbeeSerial_CONTROL                xbeeSerial_CONTROL_REG
#endif /* End xbeeSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(xbeeSerial_TX_ENABLED)
    #define xbeeSerial_TXDATA                 xbeeSerial_TXDATA_REG
    #define xbeeSerial_TXSTATUS               xbeeSerial_TXSTATUS_REG
    #define xbeeSerial_TXSTATUS_MASK          xbeeSerial_TXSTATUS_MASK_REG
    #define xbeeSerial_TXSTATUS_ACTL          xbeeSerial_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(xbeeSerial_TXCLKGEN_DP)
        #define xbeeSerial_TXBITCLKGEN_CTR        xbeeSerial_TXBITCLKGEN_CTR_REG
        #define xbeeSerial_TXBITCLKTX_COMPLETE    xbeeSerial_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define xbeeSerial_TXBITCTR_PERIOD        xbeeSerial_TXBITCTR_PERIOD_REG
        #define xbeeSerial_TXBITCTR_CONTROL       xbeeSerial_TXBITCTR_CONTROL_REG
        #define xbeeSerial_TXBITCTR_COUNTER       xbeeSerial_TXBITCTR_COUNTER_REG
    #endif /* xbeeSerial_TXCLKGEN_DP */
#endif /* End xbeeSerial_TX_ENABLED */

#if(xbeeSerial_HD_ENABLED)
    #define xbeeSerial_TXDATA                 xbeeSerial_TXDATA_REG
    #define xbeeSerial_TXSTATUS               xbeeSerial_TXSTATUS_REG
    #define xbeeSerial_TXSTATUS_MASK          xbeeSerial_TXSTATUS_MASK_REG
    #define xbeeSerial_TXSTATUS_ACTL          xbeeSerial_TXSTATUS_ACTL_REG
#endif /* End xbeeSerial_HD_ENABLED */

#if( (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) )
    #define xbeeSerial_RXDATA                 xbeeSerial_RXDATA_REG
    #define xbeeSerial_RXADDRESS1             xbeeSerial_RXADDRESS1_REG
    #define xbeeSerial_RXADDRESS2             xbeeSerial_RXADDRESS2_REG
    #define xbeeSerial_RXBITCTR_PERIOD        xbeeSerial_RXBITCTR_PERIOD_REG
    #define xbeeSerial_RXBITCTR_CONTROL       xbeeSerial_RXBITCTR_CONTROL_REG
    #define xbeeSerial_RXBITCTR_COUNTER       xbeeSerial_RXBITCTR_COUNTER_REG
    #define xbeeSerial_RXSTATUS               xbeeSerial_RXSTATUS_REG
    #define xbeeSerial_RXSTATUS_MASK          xbeeSerial_RXSTATUS_MASK_REG
    #define xbeeSerial_RXSTATUS_ACTL          xbeeSerial_RXSTATUS_ACTL_REG
#endif /* End  (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) */

#if(xbeeSerial_INTERNAL_CLOCK_USED)
    #define xbeeSerial_INTCLOCK_CLKEN         xbeeSerial_INTCLOCK_CLKEN_REG
#endif /* End xbeeSerial_INTERNAL_CLOCK_USED */

#define xbeeSerial_WAIT_FOR_COMLETE_REINIT    xbeeSerial_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_xbeeSerial_H */


/* [] END OF FILE */
