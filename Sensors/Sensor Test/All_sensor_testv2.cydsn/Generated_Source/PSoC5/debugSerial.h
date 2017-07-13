/*******************************************************************************
* File Name: debugSerial.h
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


#if !defined(CY_UART_debugSerial_H)
#define CY_UART_debugSerial_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define debugSerial_RX_ENABLED                     (1u)
#define debugSerial_TX_ENABLED                     (1u)
#define debugSerial_HD_ENABLED                     (0u)
#define debugSerial_RX_INTERRUPT_ENABLED           (0u)
#define debugSerial_TX_INTERRUPT_ENABLED           (0u)
#define debugSerial_INTERNAL_CLOCK_USED            (1u)
#define debugSerial_RXHW_ADDRESS_ENABLED           (0u)
#define debugSerial_OVER_SAMPLE_COUNT              (8u)
#define debugSerial_PARITY_TYPE                    (0u)
#define debugSerial_PARITY_TYPE_SW                 (0u)
#define debugSerial_BREAK_DETECT                   (0u)
#define debugSerial_BREAK_BITS_TX                  (13u)
#define debugSerial_BREAK_BITS_RX                  (13u)
#define debugSerial_TXCLKGEN_DP                    (1u)
#define debugSerial_USE23POLLING                   (1u)
#define debugSerial_FLOW_CONTROL                   (0u)
#define debugSerial_CLK_FREQ                       (0u)
#define debugSerial_TX_BUFFER_SIZE                 (4u)
#define debugSerial_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define debugSerial_CONTROL_REG_REMOVED            (0u)
#else
    #define debugSerial_CONTROL_REG_REMOVED            (1u)
#endif /* End debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct debugSerial_backupStruct_
{
    uint8 enableState;

    #if(debugSerial_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End debugSerial_CONTROL_REG_REMOVED */

} debugSerial_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void debugSerial_Start(void) ;
void debugSerial_Stop(void) ;
uint8 debugSerial_ReadControlRegister(void) ;
void debugSerial_WriteControlRegister(uint8 control) ;

void debugSerial_Init(void) ;
void debugSerial_Enable(void) ;
void debugSerial_SaveConfig(void) ;
void debugSerial_RestoreConfig(void) ;
void debugSerial_Sleep(void) ;
void debugSerial_Wakeup(void) ;

/* Only if RX is enabled */
#if( (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) )

    #if (debugSerial_RX_INTERRUPT_ENABLED)
        #define debugSerial_EnableRxInt()  CyIntEnable (debugSerial_RX_VECT_NUM)
        #define debugSerial_DisableRxInt() CyIntDisable(debugSerial_RX_VECT_NUM)
        CY_ISR_PROTO(debugSerial_RXISR);
    #endif /* debugSerial_RX_INTERRUPT_ENABLED */

    void debugSerial_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void debugSerial_SetRxAddress1(uint8 address) ;
    void debugSerial_SetRxAddress2(uint8 address) ;

    void  debugSerial_SetRxInterruptMode(uint8 intSrc) ;
    uint8 debugSerial_ReadRxData(void) ;
    uint8 debugSerial_ReadRxStatus(void) ;
    uint8 debugSerial_GetChar(void) ;
    uint16 debugSerial_GetByte(void) ;
    uint8 debugSerial_GetRxBufferSize(void)
                                                            ;
    void debugSerial_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define debugSerial_GetRxInterruptSource   debugSerial_ReadRxStatus

#endif /* End (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) */

/* Only if TX is enabled */
#if(debugSerial_TX_ENABLED || debugSerial_HD_ENABLED)

    #if(debugSerial_TX_INTERRUPT_ENABLED)
        #define debugSerial_EnableTxInt()  CyIntEnable (debugSerial_TX_VECT_NUM)
        #define debugSerial_DisableTxInt() CyIntDisable(debugSerial_TX_VECT_NUM)
        #define debugSerial_SetPendingTxInt() CyIntSetPending(debugSerial_TX_VECT_NUM)
        #define debugSerial_ClearPendingTxInt() CyIntClearPending(debugSerial_TX_VECT_NUM)
        CY_ISR_PROTO(debugSerial_TXISR);
    #endif /* debugSerial_TX_INTERRUPT_ENABLED */

    void debugSerial_SetTxInterruptMode(uint8 intSrc) ;
    void debugSerial_WriteTxData(uint8 txDataByte) ;
    uint8 debugSerial_ReadTxStatus(void) ;
    void debugSerial_PutChar(uint8 txDataByte) ;
    void debugSerial_PutString(const char8 string[]) ;
    void debugSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void debugSerial_PutCRLF(uint8 txDataByte) ;
    void debugSerial_ClearTxBuffer(void) ;
    void debugSerial_SetTxAddressMode(uint8 addressMode) ;
    void debugSerial_SendBreak(uint8 retMode) ;
    uint8 debugSerial_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define debugSerial_PutStringConst         debugSerial_PutString
    #define debugSerial_PutArrayConst          debugSerial_PutArray
    #define debugSerial_GetTxInterruptSource   debugSerial_ReadTxStatus

#endif /* End debugSerial_TX_ENABLED || debugSerial_HD_ENABLED */

#if(debugSerial_HD_ENABLED)
    void debugSerial_LoadRxConfig(void) ;
    void debugSerial_LoadTxConfig(void) ;
#endif /* End debugSerial_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debugSerial) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    debugSerial_CyBtldrCommStart(void) CYSMALL ;
    void    debugSerial_CyBtldrCommStop(void) CYSMALL ;
    void    debugSerial_CyBtldrCommReset(void) CYSMALL ;
    cystatus debugSerial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus debugSerial_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debugSerial)
        #define CyBtldrCommStart    debugSerial_CyBtldrCommStart
        #define CyBtldrCommStop     debugSerial_CyBtldrCommStop
        #define CyBtldrCommReset    debugSerial_CyBtldrCommReset
        #define CyBtldrCommWrite    debugSerial_CyBtldrCommWrite
        #define CyBtldrCommRead     debugSerial_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debugSerial) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define debugSerial_BYTE2BYTE_TIME_OUT (25u)
    #define debugSerial_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define debugSerial_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define debugSerial_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define debugSerial_SET_SPACE      (0x00u)
#define debugSerial_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (debugSerial_TX_ENABLED) || (debugSerial_HD_ENABLED) )
    #if(debugSerial_TX_INTERRUPT_ENABLED)
        #define debugSerial_TX_VECT_NUM            (uint8)debugSerial_TXInternalInterrupt__INTC_NUMBER
        #define debugSerial_TX_PRIOR_NUM           (uint8)debugSerial_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* debugSerial_TX_INTERRUPT_ENABLED */

    #define debugSerial_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define debugSerial_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define debugSerial_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(debugSerial_TX_ENABLED)
        #define debugSerial_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (debugSerial_HD_ENABLED) */
        #define debugSerial_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (debugSerial_TX_ENABLED) */

    #define debugSerial_TX_STS_COMPLETE            (uint8)(0x01u << debugSerial_TX_STS_COMPLETE_SHIFT)
    #define debugSerial_TX_STS_FIFO_EMPTY          (uint8)(0x01u << debugSerial_TX_STS_FIFO_EMPTY_SHIFT)
    #define debugSerial_TX_STS_FIFO_FULL           (uint8)(0x01u << debugSerial_TX_STS_FIFO_FULL_SHIFT)
    #define debugSerial_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << debugSerial_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (debugSerial_TX_ENABLED) || (debugSerial_HD_ENABLED)*/

#if( (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) )
    #if(debugSerial_RX_INTERRUPT_ENABLED)
        #define debugSerial_RX_VECT_NUM            (uint8)debugSerial_RXInternalInterrupt__INTC_NUMBER
        #define debugSerial_RX_PRIOR_NUM           (uint8)debugSerial_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* debugSerial_RX_INTERRUPT_ENABLED */
    #define debugSerial_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define debugSerial_RX_STS_BREAK_SHIFT             (0x01u)
    #define debugSerial_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define debugSerial_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define debugSerial_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define debugSerial_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define debugSerial_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define debugSerial_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define debugSerial_RX_STS_MRKSPC           (uint8)(0x01u << debugSerial_RX_STS_MRKSPC_SHIFT)
    #define debugSerial_RX_STS_BREAK            (uint8)(0x01u << debugSerial_RX_STS_BREAK_SHIFT)
    #define debugSerial_RX_STS_PAR_ERROR        (uint8)(0x01u << debugSerial_RX_STS_PAR_ERROR_SHIFT)
    #define debugSerial_RX_STS_STOP_ERROR       (uint8)(0x01u << debugSerial_RX_STS_STOP_ERROR_SHIFT)
    #define debugSerial_RX_STS_OVERRUN          (uint8)(0x01u << debugSerial_RX_STS_OVERRUN_SHIFT)
    #define debugSerial_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << debugSerial_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define debugSerial_RX_STS_ADDR_MATCH       (uint8)(0x01u << debugSerial_RX_STS_ADDR_MATCH_SHIFT)
    #define debugSerial_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << debugSerial_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define debugSerial_RX_HW_MASK                     (0x7Fu)
#endif /* End (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) */

/* Control Register definitions */
#define debugSerial_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define debugSerial_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define debugSerial_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define debugSerial_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define debugSerial_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define debugSerial_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define debugSerial_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define debugSerial_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define debugSerial_CTRL_HD_SEND               (uint8)(0x01u << debugSerial_CTRL_HD_SEND_SHIFT)
#define debugSerial_CTRL_HD_SEND_BREAK         (uint8)(0x01u << debugSerial_CTRL_HD_SEND_BREAK_SHIFT)
#define debugSerial_CTRL_MARK                  (uint8)(0x01u << debugSerial_CTRL_MARK_SHIFT)
#define debugSerial_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << debugSerial_CTRL_PARITY_TYPE0_SHIFT)
#define debugSerial_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << debugSerial_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define debugSerial_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define debugSerial_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define debugSerial_SEND_BREAK                         (0x00u)
#define debugSerial_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define debugSerial_REINIT                             (0x02u)
#define debugSerial_SEND_WAIT_REINIT                   (0x03u)

#define debugSerial_OVER_SAMPLE_8                      (8u)
#define debugSerial_OVER_SAMPLE_16                     (16u)

#define debugSerial_BIT_CENTER                         (debugSerial_OVER_SAMPLE_COUNT - 2u)

#define debugSerial_FIFO_LENGTH                        (4u)
#define debugSerial_NUMBER_OF_START_BIT                (1u)
#define debugSerial_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define debugSerial_TXBITCTR_BREAKBITS8X   ((debugSerial_BREAK_BITS_TX * debugSerial_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define debugSerial_TXBITCTR_BREAKBITS ((debugSerial_BREAK_BITS_TX * debugSerial_OVER_SAMPLE_COUNT) - 1u)

#define debugSerial_HALF_BIT_COUNT   \
                            (((debugSerial_OVER_SAMPLE_COUNT / 2u) + (debugSerial_USE23POLLING * 1u)) - 2u)
#if (debugSerial_OVER_SAMPLE_COUNT == debugSerial_OVER_SAMPLE_8)
    #define debugSerial_HD_TXBITCTR_INIT   (((debugSerial_BREAK_BITS_TX + \
                            debugSerial_NUMBER_OF_START_BIT) * debugSerial_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define debugSerial_RXBITCTR_INIT  ((((debugSerial_BREAK_BITS_RX + debugSerial_NUMBER_OF_START_BIT) \
                            * debugSerial_OVER_SAMPLE_COUNT) + debugSerial_HALF_BIT_COUNT) - 1u)

#else /* debugSerial_OVER_SAMPLE_COUNT == debugSerial_OVER_SAMPLE_16 */
    #define debugSerial_HD_TXBITCTR_INIT   ((8u * debugSerial_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define debugSerial_RXBITCTR_INIT      (((7u * debugSerial_OVER_SAMPLE_COUNT) - 1u) + \
                                                      debugSerial_HALF_BIT_COUNT)
#endif /* End debugSerial_OVER_SAMPLE_COUNT */

#define debugSerial_HD_RXBITCTR_INIT                   debugSerial_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 debugSerial_initVar;
#if (debugSerial_TX_INTERRUPT_ENABLED && debugSerial_TX_ENABLED)
    extern volatile uint8 debugSerial_txBuffer[debugSerial_TX_BUFFER_SIZE];
    extern volatile uint8 debugSerial_txBufferRead;
    extern uint8 debugSerial_txBufferWrite;
#endif /* (debugSerial_TX_INTERRUPT_ENABLED && debugSerial_TX_ENABLED) */
#if (debugSerial_RX_INTERRUPT_ENABLED && (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED))
    extern uint8 debugSerial_errorStatus;
    extern volatile uint8 debugSerial_rxBuffer[debugSerial_RX_BUFFER_SIZE];
    extern volatile uint8 debugSerial_rxBufferRead;
    extern volatile uint8 debugSerial_rxBufferWrite;
    extern volatile uint8 debugSerial_rxBufferLoopDetect;
    extern volatile uint8 debugSerial_rxBufferOverflow;
    #if (debugSerial_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 debugSerial_rxAddressMode;
        extern volatile uint8 debugSerial_rxAddressDetected;
    #endif /* (debugSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (debugSerial_RX_INTERRUPT_ENABLED && (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define debugSerial__B_UART__AM_SW_BYTE_BYTE 1
#define debugSerial__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define debugSerial__B_UART__AM_HW_BYTE_BY_BYTE 3
#define debugSerial__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define debugSerial__B_UART__AM_NONE 0

#define debugSerial__B_UART__NONE_REVB 0
#define debugSerial__B_UART__EVEN_REVB 1
#define debugSerial__B_UART__ODD_REVB 2
#define debugSerial__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define debugSerial_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define debugSerial_NUMBER_OF_STOP_BITS    (1u)

#if (debugSerial_RXHW_ADDRESS_ENABLED)
    #define debugSerial_RX_ADDRESS_MODE    (0u)
    #define debugSerial_RX_HW_ADDRESS1     (0u)
    #define debugSerial_RX_HW_ADDRESS2     (0u)
#endif /* (debugSerial_RXHW_ADDRESS_ENABLED) */

#define debugSerial_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << debugSerial_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << debugSerial_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << debugSerial_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << debugSerial_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << debugSerial_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << debugSerial_RX_STS_BREAK_SHIFT) \
                                        | (0 << debugSerial_RX_STS_OVERRUN_SHIFT))

#define debugSerial_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << debugSerial_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << debugSerial_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << debugSerial_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << debugSerial_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define debugSerial_CONTROL_REG \
                            (* (reg8 *) debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define debugSerial_CONTROL_PTR \
                            (  (reg8 *) debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(debugSerial_TX_ENABLED)
    #define debugSerial_TXDATA_REG          (* (reg8 *) debugSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define debugSerial_TXDATA_PTR          (  (reg8 *) debugSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define debugSerial_TXDATA_AUX_CTL_REG  (* (reg8 *) debugSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define debugSerial_TXDATA_AUX_CTL_PTR  (  (reg8 *) debugSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define debugSerial_TXSTATUS_REG        (* (reg8 *) debugSerial_BUART_sTX_TxSts__STATUS_REG)
    #define debugSerial_TXSTATUS_PTR        (  (reg8 *) debugSerial_BUART_sTX_TxSts__STATUS_REG)
    #define debugSerial_TXSTATUS_MASK_REG   (* (reg8 *) debugSerial_BUART_sTX_TxSts__MASK_REG)
    #define debugSerial_TXSTATUS_MASK_PTR   (  (reg8 *) debugSerial_BUART_sTX_TxSts__MASK_REG)
    #define debugSerial_TXSTATUS_ACTL_REG   (* (reg8 *) debugSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define debugSerial_TXSTATUS_ACTL_PTR   (  (reg8 *) debugSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(debugSerial_TXCLKGEN_DP)
        #define debugSerial_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define debugSerial_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define debugSerial_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define debugSerial_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define debugSerial_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define debugSerial_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define debugSerial_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define debugSerial_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define debugSerial_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define debugSerial_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) debugSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* debugSerial_TXCLKGEN_DP */

#endif /* End debugSerial_TX_ENABLED */

#if(debugSerial_HD_ENABLED)

    #define debugSerial_TXDATA_REG             (* (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define debugSerial_TXDATA_PTR             (  (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define debugSerial_TXDATA_AUX_CTL_REG     (* (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define debugSerial_TXDATA_AUX_CTL_PTR     (  (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define debugSerial_TXSTATUS_REG           (* (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debugSerial_TXSTATUS_PTR           (  (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debugSerial_TXSTATUS_MASK_REG      (* (reg8 *) debugSerial_BUART_sRX_RxSts__MASK_REG )
    #define debugSerial_TXSTATUS_MASK_PTR      (  (reg8 *) debugSerial_BUART_sRX_RxSts__MASK_REG )
    #define debugSerial_TXSTATUS_ACTL_REG      (* (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define debugSerial_TXSTATUS_ACTL_PTR      (  (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End debugSerial_HD_ENABLED */

#if( (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) )
    #define debugSerial_RXDATA_REG             (* (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define debugSerial_RXDATA_PTR             (  (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define debugSerial_RXADDRESS1_REG         (* (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define debugSerial_RXADDRESS1_PTR         (  (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define debugSerial_RXADDRESS2_REG         (* (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define debugSerial_RXADDRESS2_PTR         (  (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define debugSerial_RXDATA_AUX_CTL_REG     (* (reg8 *) debugSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define debugSerial_RXBITCTR_PERIOD_REG    (* (reg8 *) debugSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define debugSerial_RXBITCTR_PERIOD_PTR    (  (reg8 *) debugSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define debugSerial_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) debugSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define debugSerial_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) debugSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define debugSerial_RXBITCTR_COUNTER_REG   (* (reg8 *) debugSerial_BUART_sRX_RxBitCounter__COUNT_REG )
    #define debugSerial_RXBITCTR_COUNTER_PTR   (  (reg8 *) debugSerial_BUART_sRX_RxBitCounter__COUNT_REG )

    #define debugSerial_RXSTATUS_REG           (* (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debugSerial_RXSTATUS_PTR           (  (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debugSerial_RXSTATUS_MASK_REG      (* (reg8 *) debugSerial_BUART_sRX_RxSts__MASK_REG )
    #define debugSerial_RXSTATUS_MASK_PTR      (  (reg8 *) debugSerial_BUART_sRX_RxSts__MASK_REG )
    #define debugSerial_RXSTATUS_ACTL_REG      (* (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define debugSerial_RXSTATUS_ACTL_PTR      (  (reg8 *) debugSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) */

#if(debugSerial_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define debugSerial_INTCLOCK_CLKEN_REG     (* (reg8 *) debugSerial_IntClock__PM_ACT_CFG)
    #define debugSerial_INTCLOCK_CLKEN_PTR     (  (reg8 *) debugSerial_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define debugSerial_INTCLOCK_CLKEN_MASK    debugSerial_IntClock__PM_ACT_MSK
#endif /* End debugSerial_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(debugSerial_TX_ENABLED)
    #define debugSerial_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End debugSerial_TX_ENABLED */

#if(debugSerial_HD_ENABLED)
    #define debugSerial_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End debugSerial_HD_ENABLED */

#if( (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) )
    #define debugSerial_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define debugSerial_WAIT_1_MS      debugSerial_BL_CHK_DELAY_MS   

#define debugSerial_TXBUFFERSIZE   debugSerial_TX_BUFFER_SIZE
#define debugSerial_RXBUFFERSIZE   debugSerial_RX_BUFFER_SIZE

#if (debugSerial_RXHW_ADDRESS_ENABLED)
    #define debugSerial_RXADDRESSMODE  debugSerial_RX_ADDRESS_MODE
    #define debugSerial_RXHWADDRESS1   debugSerial_RX_HW_ADDRESS1
    #define debugSerial_RXHWADDRESS2   debugSerial_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define debugSerial_RXAddressMode  debugSerial_RXADDRESSMODE
#endif /* (debugSerial_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define debugSerial_initvar                    debugSerial_initVar

#define debugSerial_RX_Enabled                 debugSerial_RX_ENABLED
#define debugSerial_TX_Enabled                 debugSerial_TX_ENABLED
#define debugSerial_HD_Enabled                 debugSerial_HD_ENABLED
#define debugSerial_RX_IntInterruptEnabled     debugSerial_RX_INTERRUPT_ENABLED
#define debugSerial_TX_IntInterruptEnabled     debugSerial_TX_INTERRUPT_ENABLED
#define debugSerial_InternalClockUsed          debugSerial_INTERNAL_CLOCK_USED
#define debugSerial_RXHW_Address_Enabled       debugSerial_RXHW_ADDRESS_ENABLED
#define debugSerial_OverSampleCount            debugSerial_OVER_SAMPLE_COUNT
#define debugSerial_ParityType                 debugSerial_PARITY_TYPE

#if( debugSerial_TX_ENABLED && (debugSerial_TXBUFFERSIZE > debugSerial_FIFO_LENGTH))
    #define debugSerial_TXBUFFER               debugSerial_txBuffer
    #define debugSerial_TXBUFFERREAD           debugSerial_txBufferRead
    #define debugSerial_TXBUFFERWRITE          debugSerial_txBufferWrite
#endif /* End debugSerial_TX_ENABLED */
#if( ( debugSerial_RX_ENABLED || debugSerial_HD_ENABLED ) && \
     (debugSerial_RXBUFFERSIZE > debugSerial_FIFO_LENGTH) )
    #define debugSerial_RXBUFFER               debugSerial_rxBuffer
    #define debugSerial_RXBUFFERREAD           debugSerial_rxBufferRead
    #define debugSerial_RXBUFFERWRITE          debugSerial_rxBufferWrite
    #define debugSerial_RXBUFFERLOOPDETECT     debugSerial_rxBufferLoopDetect
    #define debugSerial_RXBUFFER_OVERFLOW      debugSerial_rxBufferOverflow
#endif /* End debugSerial_RX_ENABLED */

#ifdef debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define debugSerial_CONTROL                debugSerial_CONTROL_REG
#endif /* End debugSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(debugSerial_TX_ENABLED)
    #define debugSerial_TXDATA                 debugSerial_TXDATA_REG
    #define debugSerial_TXSTATUS               debugSerial_TXSTATUS_REG
    #define debugSerial_TXSTATUS_MASK          debugSerial_TXSTATUS_MASK_REG
    #define debugSerial_TXSTATUS_ACTL          debugSerial_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(debugSerial_TXCLKGEN_DP)
        #define debugSerial_TXBITCLKGEN_CTR        debugSerial_TXBITCLKGEN_CTR_REG
        #define debugSerial_TXBITCLKTX_COMPLETE    debugSerial_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define debugSerial_TXBITCTR_PERIOD        debugSerial_TXBITCTR_PERIOD_REG
        #define debugSerial_TXBITCTR_CONTROL       debugSerial_TXBITCTR_CONTROL_REG
        #define debugSerial_TXBITCTR_COUNTER       debugSerial_TXBITCTR_COUNTER_REG
    #endif /* debugSerial_TXCLKGEN_DP */
#endif /* End debugSerial_TX_ENABLED */

#if(debugSerial_HD_ENABLED)
    #define debugSerial_TXDATA                 debugSerial_TXDATA_REG
    #define debugSerial_TXSTATUS               debugSerial_TXSTATUS_REG
    #define debugSerial_TXSTATUS_MASK          debugSerial_TXSTATUS_MASK_REG
    #define debugSerial_TXSTATUS_ACTL          debugSerial_TXSTATUS_ACTL_REG
#endif /* End debugSerial_HD_ENABLED */

#if( (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) )
    #define debugSerial_RXDATA                 debugSerial_RXDATA_REG
    #define debugSerial_RXADDRESS1             debugSerial_RXADDRESS1_REG
    #define debugSerial_RXADDRESS2             debugSerial_RXADDRESS2_REG
    #define debugSerial_RXBITCTR_PERIOD        debugSerial_RXBITCTR_PERIOD_REG
    #define debugSerial_RXBITCTR_CONTROL       debugSerial_RXBITCTR_CONTROL_REG
    #define debugSerial_RXBITCTR_COUNTER       debugSerial_RXBITCTR_COUNTER_REG
    #define debugSerial_RXSTATUS               debugSerial_RXSTATUS_REG
    #define debugSerial_RXSTATUS_MASK          debugSerial_RXSTATUS_MASK_REG
    #define debugSerial_RXSTATUS_ACTL          debugSerial_RXSTATUS_ACTL_REG
#endif /* End  (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) */

#if(debugSerial_INTERNAL_CLOCK_USED)
    #define debugSerial_INTCLOCK_CLKEN         debugSerial_INTCLOCK_CLKEN_REG
#endif /* End debugSerial_INTERNAL_CLOCK_USED */

#define debugSerial_WAIT_FOR_COMLETE_REINIT    debugSerial_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_debugSerial_H */


/* [] END OF FILE */
