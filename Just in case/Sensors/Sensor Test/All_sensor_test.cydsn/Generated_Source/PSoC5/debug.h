/*******************************************************************************
* File Name: debug.h
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


#if !defined(CY_UART_debug_H)
#define CY_UART_debug_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define debug_RX_ENABLED                     (1u)
#define debug_TX_ENABLED                     (1u)
#define debug_HD_ENABLED                     (0u)
#define debug_RX_INTERRUPT_ENABLED           (0u)
#define debug_TX_INTERRUPT_ENABLED           (0u)
#define debug_INTERNAL_CLOCK_USED            (1u)
#define debug_RXHW_ADDRESS_ENABLED           (0u)
#define debug_OVER_SAMPLE_COUNT              (8u)
#define debug_PARITY_TYPE                    (0u)
#define debug_PARITY_TYPE_SW                 (0u)
#define debug_BREAK_DETECT                   (0u)
#define debug_BREAK_BITS_TX                  (13u)
#define debug_BREAK_BITS_RX                  (13u)
#define debug_TXCLKGEN_DP                    (1u)
#define debug_USE23POLLING                   (1u)
#define debug_FLOW_CONTROL                   (0u)
#define debug_CLK_FREQ                       (0u)
#define debug_TX_BUFFER_SIZE                 (4u)
#define debug_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define debug_CONTROL_REG_REMOVED            (0u)
#else
    #define debug_CONTROL_REG_REMOVED            (1u)
#endif /* End debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct debug_backupStruct_
{
    uint8 enableState;

    #if(debug_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End debug_CONTROL_REG_REMOVED */

} debug_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void debug_Start(void) ;
void debug_Stop(void) ;
uint8 debug_ReadControlRegister(void) ;
void debug_WriteControlRegister(uint8 control) ;

void debug_Init(void) ;
void debug_Enable(void) ;
void debug_SaveConfig(void) ;
void debug_RestoreConfig(void) ;
void debug_Sleep(void) ;
void debug_Wakeup(void) ;

/* Only if RX is enabled */
#if( (debug_RX_ENABLED) || (debug_HD_ENABLED) )

    #if (debug_RX_INTERRUPT_ENABLED)
        #define debug_EnableRxInt()  CyIntEnable (debug_RX_VECT_NUM)
        #define debug_DisableRxInt() CyIntDisable(debug_RX_VECT_NUM)
        CY_ISR_PROTO(debug_RXISR);
    #endif /* debug_RX_INTERRUPT_ENABLED */

    void debug_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void debug_SetRxAddress1(uint8 address) ;
    void debug_SetRxAddress2(uint8 address) ;

    void  debug_SetRxInterruptMode(uint8 intSrc) ;
    uint8 debug_ReadRxData(void) ;
    uint8 debug_ReadRxStatus(void) ;
    uint8 debug_GetChar(void) ;
    uint16 debug_GetByte(void) ;
    uint8 debug_GetRxBufferSize(void)
                                                            ;
    void debug_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define debug_GetRxInterruptSource   debug_ReadRxStatus

#endif /* End (debug_RX_ENABLED) || (debug_HD_ENABLED) */

/* Only if TX is enabled */
#if(debug_TX_ENABLED || debug_HD_ENABLED)

    #if(debug_TX_INTERRUPT_ENABLED)
        #define debug_EnableTxInt()  CyIntEnable (debug_TX_VECT_NUM)
        #define debug_DisableTxInt() CyIntDisable(debug_TX_VECT_NUM)
        #define debug_SetPendingTxInt() CyIntSetPending(debug_TX_VECT_NUM)
        #define debug_ClearPendingTxInt() CyIntClearPending(debug_TX_VECT_NUM)
        CY_ISR_PROTO(debug_TXISR);
    #endif /* debug_TX_INTERRUPT_ENABLED */

    void debug_SetTxInterruptMode(uint8 intSrc) ;
    void debug_WriteTxData(uint8 txDataByte) ;
    uint8 debug_ReadTxStatus(void) ;
    void debug_PutChar(uint8 txDataByte) ;
    void debug_PutString(const char8 string[]) ;
    void debug_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void debug_PutCRLF(uint8 txDataByte) ;
    void debug_ClearTxBuffer(void) ;
    void debug_SetTxAddressMode(uint8 addressMode) ;
    void debug_SendBreak(uint8 retMode) ;
    uint8 debug_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define debug_PutStringConst         debug_PutString
    #define debug_PutArrayConst          debug_PutArray
    #define debug_GetTxInterruptSource   debug_ReadTxStatus

#endif /* End debug_TX_ENABLED || debug_HD_ENABLED */

#if(debug_HD_ENABLED)
    void debug_LoadRxConfig(void) ;
    void debug_LoadTxConfig(void) ;
#endif /* End debug_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debug) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    debug_CyBtldrCommStart(void) CYSMALL ;
    void    debug_CyBtldrCommStop(void) CYSMALL ;
    void    debug_CyBtldrCommReset(void) CYSMALL ;
    cystatus debug_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus debug_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debug)
        #define CyBtldrCommStart    debug_CyBtldrCommStart
        #define CyBtldrCommStop     debug_CyBtldrCommStop
        #define CyBtldrCommReset    debug_CyBtldrCommReset
        #define CyBtldrCommWrite    debug_CyBtldrCommWrite
        #define CyBtldrCommRead     debug_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debug) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define debug_BYTE2BYTE_TIME_OUT (25u)
    #define debug_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define debug_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define debug_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define debug_SET_SPACE      (0x00u)
#define debug_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (debug_TX_ENABLED) || (debug_HD_ENABLED) )
    #if(debug_TX_INTERRUPT_ENABLED)
        #define debug_TX_VECT_NUM            (uint8)debug_TXInternalInterrupt__INTC_NUMBER
        #define debug_TX_PRIOR_NUM           (uint8)debug_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* debug_TX_INTERRUPT_ENABLED */

    #define debug_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define debug_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define debug_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(debug_TX_ENABLED)
        #define debug_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (debug_HD_ENABLED) */
        #define debug_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (debug_TX_ENABLED) */

    #define debug_TX_STS_COMPLETE            (uint8)(0x01u << debug_TX_STS_COMPLETE_SHIFT)
    #define debug_TX_STS_FIFO_EMPTY          (uint8)(0x01u << debug_TX_STS_FIFO_EMPTY_SHIFT)
    #define debug_TX_STS_FIFO_FULL           (uint8)(0x01u << debug_TX_STS_FIFO_FULL_SHIFT)
    #define debug_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << debug_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (debug_TX_ENABLED) || (debug_HD_ENABLED)*/

#if( (debug_RX_ENABLED) || (debug_HD_ENABLED) )
    #if(debug_RX_INTERRUPT_ENABLED)
        #define debug_RX_VECT_NUM            (uint8)debug_RXInternalInterrupt__INTC_NUMBER
        #define debug_RX_PRIOR_NUM           (uint8)debug_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* debug_RX_INTERRUPT_ENABLED */
    #define debug_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define debug_RX_STS_BREAK_SHIFT             (0x01u)
    #define debug_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define debug_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define debug_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define debug_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define debug_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define debug_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define debug_RX_STS_MRKSPC           (uint8)(0x01u << debug_RX_STS_MRKSPC_SHIFT)
    #define debug_RX_STS_BREAK            (uint8)(0x01u << debug_RX_STS_BREAK_SHIFT)
    #define debug_RX_STS_PAR_ERROR        (uint8)(0x01u << debug_RX_STS_PAR_ERROR_SHIFT)
    #define debug_RX_STS_STOP_ERROR       (uint8)(0x01u << debug_RX_STS_STOP_ERROR_SHIFT)
    #define debug_RX_STS_OVERRUN          (uint8)(0x01u << debug_RX_STS_OVERRUN_SHIFT)
    #define debug_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << debug_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define debug_RX_STS_ADDR_MATCH       (uint8)(0x01u << debug_RX_STS_ADDR_MATCH_SHIFT)
    #define debug_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << debug_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define debug_RX_HW_MASK                     (0x7Fu)
#endif /* End (debug_RX_ENABLED) || (debug_HD_ENABLED) */

/* Control Register definitions */
#define debug_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define debug_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define debug_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define debug_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define debug_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define debug_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define debug_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define debug_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define debug_CTRL_HD_SEND               (uint8)(0x01u << debug_CTRL_HD_SEND_SHIFT)
#define debug_CTRL_HD_SEND_BREAK         (uint8)(0x01u << debug_CTRL_HD_SEND_BREAK_SHIFT)
#define debug_CTRL_MARK                  (uint8)(0x01u << debug_CTRL_MARK_SHIFT)
#define debug_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << debug_CTRL_PARITY_TYPE0_SHIFT)
#define debug_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << debug_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define debug_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define debug_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define debug_SEND_BREAK                         (0x00u)
#define debug_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define debug_REINIT                             (0x02u)
#define debug_SEND_WAIT_REINIT                   (0x03u)

#define debug_OVER_SAMPLE_8                      (8u)
#define debug_OVER_SAMPLE_16                     (16u)

#define debug_BIT_CENTER                         (debug_OVER_SAMPLE_COUNT - 2u)

#define debug_FIFO_LENGTH                        (4u)
#define debug_NUMBER_OF_START_BIT                (1u)
#define debug_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define debug_TXBITCTR_BREAKBITS8X   ((debug_BREAK_BITS_TX * debug_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define debug_TXBITCTR_BREAKBITS ((debug_BREAK_BITS_TX * debug_OVER_SAMPLE_COUNT) - 1u)

#define debug_HALF_BIT_COUNT   \
                            (((debug_OVER_SAMPLE_COUNT / 2u) + (debug_USE23POLLING * 1u)) - 2u)
#if (debug_OVER_SAMPLE_COUNT == debug_OVER_SAMPLE_8)
    #define debug_HD_TXBITCTR_INIT   (((debug_BREAK_BITS_TX + \
                            debug_NUMBER_OF_START_BIT) * debug_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define debug_RXBITCTR_INIT  ((((debug_BREAK_BITS_RX + debug_NUMBER_OF_START_BIT) \
                            * debug_OVER_SAMPLE_COUNT) + debug_HALF_BIT_COUNT) - 1u)

#else /* debug_OVER_SAMPLE_COUNT == debug_OVER_SAMPLE_16 */
    #define debug_HD_TXBITCTR_INIT   ((8u * debug_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define debug_RXBITCTR_INIT      (((7u * debug_OVER_SAMPLE_COUNT) - 1u) + \
                                                      debug_HALF_BIT_COUNT)
#endif /* End debug_OVER_SAMPLE_COUNT */

#define debug_HD_RXBITCTR_INIT                   debug_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 debug_initVar;
#if (debug_TX_INTERRUPT_ENABLED && debug_TX_ENABLED)
    extern volatile uint8 debug_txBuffer[debug_TX_BUFFER_SIZE];
    extern volatile uint8 debug_txBufferRead;
    extern uint8 debug_txBufferWrite;
#endif /* (debug_TX_INTERRUPT_ENABLED && debug_TX_ENABLED) */
#if (debug_RX_INTERRUPT_ENABLED && (debug_RX_ENABLED || debug_HD_ENABLED))
    extern uint8 debug_errorStatus;
    extern volatile uint8 debug_rxBuffer[debug_RX_BUFFER_SIZE];
    extern volatile uint8 debug_rxBufferRead;
    extern volatile uint8 debug_rxBufferWrite;
    extern volatile uint8 debug_rxBufferLoopDetect;
    extern volatile uint8 debug_rxBufferOverflow;
    #if (debug_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 debug_rxAddressMode;
        extern volatile uint8 debug_rxAddressDetected;
    #endif /* (debug_RXHW_ADDRESS_ENABLED) */
#endif /* (debug_RX_INTERRUPT_ENABLED && (debug_RX_ENABLED || debug_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define debug__B_UART__AM_SW_BYTE_BYTE 1
#define debug__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define debug__B_UART__AM_HW_BYTE_BY_BYTE 3
#define debug__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define debug__B_UART__AM_NONE 0

#define debug__B_UART__NONE_REVB 0
#define debug__B_UART__EVEN_REVB 1
#define debug__B_UART__ODD_REVB 2
#define debug__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define debug_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define debug_NUMBER_OF_STOP_BITS    (1u)

#if (debug_RXHW_ADDRESS_ENABLED)
    #define debug_RX_ADDRESS_MODE    (0u)
    #define debug_RX_HW_ADDRESS1     (0u)
    #define debug_RX_HW_ADDRESS2     (0u)
#endif /* (debug_RXHW_ADDRESS_ENABLED) */

#define debug_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << debug_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << debug_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << debug_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << debug_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << debug_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << debug_RX_STS_BREAK_SHIFT) \
                                        | (0 << debug_RX_STS_OVERRUN_SHIFT))

#define debug_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << debug_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << debug_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << debug_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << debug_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define debug_CONTROL_REG \
                            (* (reg8 *) debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define debug_CONTROL_PTR \
                            (  (reg8 *) debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(debug_TX_ENABLED)
    #define debug_TXDATA_REG          (* (reg8 *) debug_BUART_sTX_TxShifter_u0__F0_REG)
    #define debug_TXDATA_PTR          (  (reg8 *) debug_BUART_sTX_TxShifter_u0__F0_REG)
    #define debug_TXDATA_AUX_CTL_REG  (* (reg8 *) debug_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define debug_TXDATA_AUX_CTL_PTR  (  (reg8 *) debug_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define debug_TXSTATUS_REG        (* (reg8 *) debug_BUART_sTX_TxSts__STATUS_REG)
    #define debug_TXSTATUS_PTR        (  (reg8 *) debug_BUART_sTX_TxSts__STATUS_REG)
    #define debug_TXSTATUS_MASK_REG   (* (reg8 *) debug_BUART_sTX_TxSts__MASK_REG)
    #define debug_TXSTATUS_MASK_PTR   (  (reg8 *) debug_BUART_sTX_TxSts__MASK_REG)
    #define debug_TXSTATUS_ACTL_REG   (* (reg8 *) debug_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define debug_TXSTATUS_ACTL_PTR   (  (reg8 *) debug_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(debug_TXCLKGEN_DP)
        #define debug_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) debug_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define debug_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) debug_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define debug_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) debug_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define debug_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) debug_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define debug_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) debug_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define debug_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) debug_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define debug_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) debug_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define debug_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) debug_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define debug_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) debug_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define debug_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) debug_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* debug_TXCLKGEN_DP */

#endif /* End debug_TX_ENABLED */

#if(debug_HD_ENABLED)

    #define debug_TXDATA_REG             (* (reg8 *) debug_BUART_sRX_RxShifter_u0__F1_REG )
    #define debug_TXDATA_PTR             (  (reg8 *) debug_BUART_sRX_RxShifter_u0__F1_REG )
    #define debug_TXDATA_AUX_CTL_REG     (* (reg8 *) debug_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define debug_TXDATA_AUX_CTL_PTR     (  (reg8 *) debug_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define debug_TXSTATUS_REG           (* (reg8 *) debug_BUART_sRX_RxSts__STATUS_REG )
    #define debug_TXSTATUS_PTR           (  (reg8 *) debug_BUART_sRX_RxSts__STATUS_REG )
    #define debug_TXSTATUS_MASK_REG      (* (reg8 *) debug_BUART_sRX_RxSts__MASK_REG )
    #define debug_TXSTATUS_MASK_PTR      (  (reg8 *) debug_BUART_sRX_RxSts__MASK_REG )
    #define debug_TXSTATUS_ACTL_REG      (* (reg8 *) debug_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define debug_TXSTATUS_ACTL_PTR      (  (reg8 *) debug_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End debug_HD_ENABLED */

#if( (debug_RX_ENABLED) || (debug_HD_ENABLED) )
    #define debug_RXDATA_REG             (* (reg8 *) debug_BUART_sRX_RxShifter_u0__F0_REG )
    #define debug_RXDATA_PTR             (  (reg8 *) debug_BUART_sRX_RxShifter_u0__F0_REG )
    #define debug_RXADDRESS1_REG         (* (reg8 *) debug_BUART_sRX_RxShifter_u0__D0_REG )
    #define debug_RXADDRESS1_PTR         (  (reg8 *) debug_BUART_sRX_RxShifter_u0__D0_REG )
    #define debug_RXADDRESS2_REG         (* (reg8 *) debug_BUART_sRX_RxShifter_u0__D1_REG )
    #define debug_RXADDRESS2_PTR         (  (reg8 *) debug_BUART_sRX_RxShifter_u0__D1_REG )
    #define debug_RXDATA_AUX_CTL_REG     (* (reg8 *) debug_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define debug_RXBITCTR_PERIOD_REG    (* (reg8 *) debug_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define debug_RXBITCTR_PERIOD_PTR    (  (reg8 *) debug_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define debug_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) debug_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define debug_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) debug_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define debug_RXBITCTR_COUNTER_REG   (* (reg8 *) debug_BUART_sRX_RxBitCounter__COUNT_REG )
    #define debug_RXBITCTR_COUNTER_PTR   (  (reg8 *) debug_BUART_sRX_RxBitCounter__COUNT_REG )

    #define debug_RXSTATUS_REG           (* (reg8 *) debug_BUART_sRX_RxSts__STATUS_REG )
    #define debug_RXSTATUS_PTR           (  (reg8 *) debug_BUART_sRX_RxSts__STATUS_REG )
    #define debug_RXSTATUS_MASK_REG      (* (reg8 *) debug_BUART_sRX_RxSts__MASK_REG )
    #define debug_RXSTATUS_MASK_PTR      (  (reg8 *) debug_BUART_sRX_RxSts__MASK_REG )
    #define debug_RXSTATUS_ACTL_REG      (* (reg8 *) debug_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define debug_RXSTATUS_ACTL_PTR      (  (reg8 *) debug_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (debug_RX_ENABLED) || (debug_HD_ENABLED) */

#if(debug_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define debug_INTCLOCK_CLKEN_REG     (* (reg8 *) debug_IntClock__PM_ACT_CFG)
    #define debug_INTCLOCK_CLKEN_PTR     (  (reg8 *) debug_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define debug_INTCLOCK_CLKEN_MASK    debug_IntClock__PM_ACT_MSK
#endif /* End debug_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(debug_TX_ENABLED)
    #define debug_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End debug_TX_ENABLED */

#if(debug_HD_ENABLED)
    #define debug_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End debug_HD_ENABLED */

#if( (debug_RX_ENABLED) || (debug_HD_ENABLED) )
    #define debug_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (debug_RX_ENABLED) || (debug_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define debug_WAIT_1_MS      debug_BL_CHK_DELAY_MS   

#define debug_TXBUFFERSIZE   debug_TX_BUFFER_SIZE
#define debug_RXBUFFERSIZE   debug_RX_BUFFER_SIZE

#if (debug_RXHW_ADDRESS_ENABLED)
    #define debug_RXADDRESSMODE  debug_RX_ADDRESS_MODE
    #define debug_RXHWADDRESS1   debug_RX_HW_ADDRESS1
    #define debug_RXHWADDRESS2   debug_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define debug_RXAddressMode  debug_RXADDRESSMODE
#endif /* (debug_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define debug_initvar                    debug_initVar

#define debug_RX_Enabled                 debug_RX_ENABLED
#define debug_TX_Enabled                 debug_TX_ENABLED
#define debug_HD_Enabled                 debug_HD_ENABLED
#define debug_RX_IntInterruptEnabled     debug_RX_INTERRUPT_ENABLED
#define debug_TX_IntInterruptEnabled     debug_TX_INTERRUPT_ENABLED
#define debug_InternalClockUsed          debug_INTERNAL_CLOCK_USED
#define debug_RXHW_Address_Enabled       debug_RXHW_ADDRESS_ENABLED
#define debug_OverSampleCount            debug_OVER_SAMPLE_COUNT
#define debug_ParityType                 debug_PARITY_TYPE

#if( debug_TX_ENABLED && (debug_TXBUFFERSIZE > debug_FIFO_LENGTH))
    #define debug_TXBUFFER               debug_txBuffer
    #define debug_TXBUFFERREAD           debug_txBufferRead
    #define debug_TXBUFFERWRITE          debug_txBufferWrite
#endif /* End debug_TX_ENABLED */
#if( ( debug_RX_ENABLED || debug_HD_ENABLED ) && \
     (debug_RXBUFFERSIZE > debug_FIFO_LENGTH) )
    #define debug_RXBUFFER               debug_rxBuffer
    #define debug_RXBUFFERREAD           debug_rxBufferRead
    #define debug_RXBUFFERWRITE          debug_rxBufferWrite
    #define debug_RXBUFFERLOOPDETECT     debug_rxBufferLoopDetect
    #define debug_RXBUFFER_OVERFLOW      debug_rxBufferOverflow
#endif /* End debug_RX_ENABLED */

#ifdef debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define debug_CONTROL                debug_CONTROL_REG
#endif /* End debug_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(debug_TX_ENABLED)
    #define debug_TXDATA                 debug_TXDATA_REG
    #define debug_TXSTATUS               debug_TXSTATUS_REG
    #define debug_TXSTATUS_MASK          debug_TXSTATUS_MASK_REG
    #define debug_TXSTATUS_ACTL          debug_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(debug_TXCLKGEN_DP)
        #define debug_TXBITCLKGEN_CTR        debug_TXBITCLKGEN_CTR_REG
        #define debug_TXBITCLKTX_COMPLETE    debug_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define debug_TXBITCTR_PERIOD        debug_TXBITCTR_PERIOD_REG
        #define debug_TXBITCTR_CONTROL       debug_TXBITCTR_CONTROL_REG
        #define debug_TXBITCTR_COUNTER       debug_TXBITCTR_COUNTER_REG
    #endif /* debug_TXCLKGEN_DP */
#endif /* End debug_TX_ENABLED */

#if(debug_HD_ENABLED)
    #define debug_TXDATA                 debug_TXDATA_REG
    #define debug_TXSTATUS               debug_TXSTATUS_REG
    #define debug_TXSTATUS_MASK          debug_TXSTATUS_MASK_REG
    #define debug_TXSTATUS_ACTL          debug_TXSTATUS_ACTL_REG
#endif /* End debug_HD_ENABLED */

#if( (debug_RX_ENABLED) || (debug_HD_ENABLED) )
    #define debug_RXDATA                 debug_RXDATA_REG
    #define debug_RXADDRESS1             debug_RXADDRESS1_REG
    #define debug_RXADDRESS2             debug_RXADDRESS2_REG
    #define debug_RXBITCTR_PERIOD        debug_RXBITCTR_PERIOD_REG
    #define debug_RXBITCTR_CONTROL       debug_RXBITCTR_CONTROL_REG
    #define debug_RXBITCTR_COUNTER       debug_RXBITCTR_COUNTER_REG
    #define debug_RXSTATUS               debug_RXSTATUS_REG
    #define debug_RXSTATUS_MASK          debug_RXSTATUS_MASK_REG
    #define debug_RXSTATUS_ACTL          debug_RXSTATUS_ACTL_REG
#endif /* End  (debug_RX_ENABLED) || (debug_HD_ENABLED) */

#if(debug_INTERNAL_CLOCK_USED)
    #define debug_INTCLOCK_CLKEN         debug_INTCLOCK_CLKEN_REG
#endif /* End debug_INTERNAL_CLOCK_USED */

#define debug_WAIT_FOR_COMLETE_REINIT    debug_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_debug_H */


/* [] END OF FILE */
