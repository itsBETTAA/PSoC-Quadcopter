/*******************************************************************************
* File Name: debSerial.h
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


#if !defined(CY_UART_debSerial_H)
#define CY_UART_debSerial_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define debSerial_RX_ENABLED                     (1u)
#define debSerial_TX_ENABLED                     (1u)
#define debSerial_HD_ENABLED                     (0u)
#define debSerial_RX_INTERRUPT_ENABLED           (0u)
#define debSerial_TX_INTERRUPT_ENABLED           (0u)
#define debSerial_INTERNAL_CLOCK_USED            (1u)
#define debSerial_RXHW_ADDRESS_ENABLED           (0u)
#define debSerial_OVER_SAMPLE_COUNT              (8u)
#define debSerial_PARITY_TYPE                    (0u)
#define debSerial_PARITY_TYPE_SW                 (0u)
#define debSerial_BREAK_DETECT                   (0u)
#define debSerial_BREAK_BITS_TX                  (13u)
#define debSerial_BREAK_BITS_RX                  (13u)
#define debSerial_TXCLKGEN_DP                    (1u)
#define debSerial_USE23POLLING                   (1u)
#define debSerial_FLOW_CONTROL                   (0u)
#define debSerial_CLK_FREQ                       (0u)
#define debSerial_TX_BUFFER_SIZE                 (4u)
#define debSerial_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define debSerial_CONTROL_REG_REMOVED            (0u)
#else
    #define debSerial_CONTROL_REG_REMOVED            (1u)
#endif /* End debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct debSerial_backupStruct_
{
    uint8 enableState;

    #if(debSerial_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End debSerial_CONTROL_REG_REMOVED */

} debSerial_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void debSerial_Start(void) ;
void debSerial_Stop(void) ;
uint8 debSerial_ReadControlRegister(void) ;
void debSerial_WriteControlRegister(uint8 control) ;

void debSerial_Init(void) ;
void debSerial_Enable(void) ;
void debSerial_SaveConfig(void) ;
void debSerial_RestoreConfig(void) ;
void debSerial_Sleep(void) ;
void debSerial_Wakeup(void) ;

/* Only if RX is enabled */
#if( (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) )

    #if (debSerial_RX_INTERRUPT_ENABLED)
        #define debSerial_EnableRxInt()  CyIntEnable (debSerial_RX_VECT_NUM)
        #define debSerial_DisableRxInt() CyIntDisable(debSerial_RX_VECT_NUM)
        CY_ISR_PROTO(debSerial_RXISR);
    #endif /* debSerial_RX_INTERRUPT_ENABLED */

    void debSerial_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void debSerial_SetRxAddress1(uint8 address) ;
    void debSerial_SetRxAddress2(uint8 address) ;

    void  debSerial_SetRxInterruptMode(uint8 intSrc) ;
    uint8 debSerial_ReadRxData(void) ;
    uint8 debSerial_ReadRxStatus(void) ;
    uint8 debSerial_GetChar(void) ;
    uint16 debSerial_GetByte(void) ;
    uint8 debSerial_GetRxBufferSize(void)
                                                            ;
    void debSerial_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define debSerial_GetRxInterruptSource   debSerial_ReadRxStatus

#endif /* End (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) */

/* Only if TX is enabled */
#if(debSerial_TX_ENABLED || debSerial_HD_ENABLED)

    #if(debSerial_TX_INTERRUPT_ENABLED)
        #define debSerial_EnableTxInt()  CyIntEnable (debSerial_TX_VECT_NUM)
        #define debSerial_DisableTxInt() CyIntDisable(debSerial_TX_VECT_NUM)
        #define debSerial_SetPendingTxInt() CyIntSetPending(debSerial_TX_VECT_NUM)
        #define debSerial_ClearPendingTxInt() CyIntClearPending(debSerial_TX_VECT_NUM)
        CY_ISR_PROTO(debSerial_TXISR);
    #endif /* debSerial_TX_INTERRUPT_ENABLED */

    void debSerial_SetTxInterruptMode(uint8 intSrc) ;
    void debSerial_WriteTxData(uint8 txDataByte) ;
    uint8 debSerial_ReadTxStatus(void) ;
    void debSerial_PutChar(uint8 txDataByte) ;
    void debSerial_PutString(const char8 string[]) ;
    void debSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void debSerial_PutCRLF(uint8 txDataByte) ;
    void debSerial_ClearTxBuffer(void) ;
    void debSerial_SetTxAddressMode(uint8 addressMode) ;
    void debSerial_SendBreak(uint8 retMode) ;
    uint8 debSerial_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define debSerial_PutStringConst         debSerial_PutString
    #define debSerial_PutArrayConst          debSerial_PutArray
    #define debSerial_GetTxInterruptSource   debSerial_ReadTxStatus

#endif /* End debSerial_TX_ENABLED || debSerial_HD_ENABLED */

#if(debSerial_HD_ENABLED)
    void debSerial_LoadRxConfig(void) ;
    void debSerial_LoadTxConfig(void) ;
#endif /* End debSerial_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debSerial) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    debSerial_CyBtldrCommStart(void) CYSMALL ;
    void    debSerial_CyBtldrCommStop(void) CYSMALL ;
    void    debSerial_CyBtldrCommReset(void) CYSMALL ;
    cystatus debSerial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus debSerial_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debSerial)
        #define CyBtldrCommStart    debSerial_CyBtldrCommStart
        #define CyBtldrCommStop     debSerial_CyBtldrCommStop
        #define CyBtldrCommReset    debSerial_CyBtldrCommReset
        #define CyBtldrCommWrite    debSerial_CyBtldrCommWrite
        #define CyBtldrCommRead     debSerial_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_debSerial) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define debSerial_BYTE2BYTE_TIME_OUT (25u)
    #define debSerial_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define debSerial_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define debSerial_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define debSerial_SET_SPACE      (0x00u)
#define debSerial_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (debSerial_TX_ENABLED) || (debSerial_HD_ENABLED) )
    #if(debSerial_TX_INTERRUPT_ENABLED)
        #define debSerial_TX_VECT_NUM            (uint8)debSerial_TXInternalInterrupt__INTC_NUMBER
        #define debSerial_TX_PRIOR_NUM           (uint8)debSerial_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* debSerial_TX_INTERRUPT_ENABLED */

    #define debSerial_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define debSerial_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define debSerial_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(debSerial_TX_ENABLED)
        #define debSerial_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (debSerial_HD_ENABLED) */
        #define debSerial_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (debSerial_TX_ENABLED) */

    #define debSerial_TX_STS_COMPLETE            (uint8)(0x01u << debSerial_TX_STS_COMPLETE_SHIFT)
    #define debSerial_TX_STS_FIFO_EMPTY          (uint8)(0x01u << debSerial_TX_STS_FIFO_EMPTY_SHIFT)
    #define debSerial_TX_STS_FIFO_FULL           (uint8)(0x01u << debSerial_TX_STS_FIFO_FULL_SHIFT)
    #define debSerial_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << debSerial_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (debSerial_TX_ENABLED) || (debSerial_HD_ENABLED)*/

#if( (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) )
    #if(debSerial_RX_INTERRUPT_ENABLED)
        #define debSerial_RX_VECT_NUM            (uint8)debSerial_RXInternalInterrupt__INTC_NUMBER
        #define debSerial_RX_PRIOR_NUM           (uint8)debSerial_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* debSerial_RX_INTERRUPT_ENABLED */
    #define debSerial_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define debSerial_RX_STS_BREAK_SHIFT             (0x01u)
    #define debSerial_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define debSerial_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define debSerial_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define debSerial_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define debSerial_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define debSerial_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define debSerial_RX_STS_MRKSPC           (uint8)(0x01u << debSerial_RX_STS_MRKSPC_SHIFT)
    #define debSerial_RX_STS_BREAK            (uint8)(0x01u << debSerial_RX_STS_BREAK_SHIFT)
    #define debSerial_RX_STS_PAR_ERROR        (uint8)(0x01u << debSerial_RX_STS_PAR_ERROR_SHIFT)
    #define debSerial_RX_STS_STOP_ERROR       (uint8)(0x01u << debSerial_RX_STS_STOP_ERROR_SHIFT)
    #define debSerial_RX_STS_OVERRUN          (uint8)(0x01u << debSerial_RX_STS_OVERRUN_SHIFT)
    #define debSerial_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << debSerial_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define debSerial_RX_STS_ADDR_MATCH       (uint8)(0x01u << debSerial_RX_STS_ADDR_MATCH_SHIFT)
    #define debSerial_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << debSerial_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define debSerial_RX_HW_MASK                     (0x7Fu)
#endif /* End (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) */

/* Control Register definitions */
#define debSerial_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define debSerial_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define debSerial_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define debSerial_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define debSerial_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define debSerial_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define debSerial_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define debSerial_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define debSerial_CTRL_HD_SEND               (uint8)(0x01u << debSerial_CTRL_HD_SEND_SHIFT)
#define debSerial_CTRL_HD_SEND_BREAK         (uint8)(0x01u << debSerial_CTRL_HD_SEND_BREAK_SHIFT)
#define debSerial_CTRL_MARK                  (uint8)(0x01u << debSerial_CTRL_MARK_SHIFT)
#define debSerial_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << debSerial_CTRL_PARITY_TYPE0_SHIFT)
#define debSerial_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << debSerial_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define debSerial_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define debSerial_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define debSerial_SEND_BREAK                         (0x00u)
#define debSerial_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define debSerial_REINIT                             (0x02u)
#define debSerial_SEND_WAIT_REINIT                   (0x03u)

#define debSerial_OVER_SAMPLE_8                      (8u)
#define debSerial_OVER_SAMPLE_16                     (16u)

#define debSerial_BIT_CENTER                         (debSerial_OVER_SAMPLE_COUNT - 2u)

#define debSerial_FIFO_LENGTH                        (4u)
#define debSerial_NUMBER_OF_START_BIT                (1u)
#define debSerial_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define debSerial_TXBITCTR_BREAKBITS8X   ((debSerial_BREAK_BITS_TX * debSerial_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define debSerial_TXBITCTR_BREAKBITS ((debSerial_BREAK_BITS_TX * debSerial_OVER_SAMPLE_COUNT) - 1u)

#define debSerial_HALF_BIT_COUNT   \
                            (((debSerial_OVER_SAMPLE_COUNT / 2u) + (debSerial_USE23POLLING * 1u)) - 2u)
#if (debSerial_OVER_SAMPLE_COUNT == debSerial_OVER_SAMPLE_8)
    #define debSerial_HD_TXBITCTR_INIT   (((debSerial_BREAK_BITS_TX + \
                            debSerial_NUMBER_OF_START_BIT) * debSerial_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define debSerial_RXBITCTR_INIT  ((((debSerial_BREAK_BITS_RX + debSerial_NUMBER_OF_START_BIT) \
                            * debSerial_OVER_SAMPLE_COUNT) + debSerial_HALF_BIT_COUNT) - 1u)

#else /* debSerial_OVER_SAMPLE_COUNT == debSerial_OVER_SAMPLE_16 */
    #define debSerial_HD_TXBITCTR_INIT   ((8u * debSerial_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define debSerial_RXBITCTR_INIT      (((7u * debSerial_OVER_SAMPLE_COUNT) - 1u) + \
                                                      debSerial_HALF_BIT_COUNT)
#endif /* End debSerial_OVER_SAMPLE_COUNT */

#define debSerial_HD_RXBITCTR_INIT                   debSerial_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 debSerial_initVar;
#if (debSerial_TX_INTERRUPT_ENABLED && debSerial_TX_ENABLED)
    extern volatile uint8 debSerial_txBuffer[debSerial_TX_BUFFER_SIZE];
    extern volatile uint8 debSerial_txBufferRead;
    extern uint8 debSerial_txBufferWrite;
#endif /* (debSerial_TX_INTERRUPT_ENABLED && debSerial_TX_ENABLED) */
#if (debSerial_RX_INTERRUPT_ENABLED && (debSerial_RX_ENABLED || debSerial_HD_ENABLED))
    extern uint8 debSerial_errorStatus;
    extern volatile uint8 debSerial_rxBuffer[debSerial_RX_BUFFER_SIZE];
    extern volatile uint8 debSerial_rxBufferRead;
    extern volatile uint8 debSerial_rxBufferWrite;
    extern volatile uint8 debSerial_rxBufferLoopDetect;
    extern volatile uint8 debSerial_rxBufferOverflow;
    #if (debSerial_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 debSerial_rxAddressMode;
        extern volatile uint8 debSerial_rxAddressDetected;
    #endif /* (debSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (debSerial_RX_INTERRUPT_ENABLED && (debSerial_RX_ENABLED || debSerial_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define debSerial__B_UART__AM_SW_BYTE_BYTE 1
#define debSerial__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define debSerial__B_UART__AM_HW_BYTE_BY_BYTE 3
#define debSerial__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define debSerial__B_UART__AM_NONE 0

#define debSerial__B_UART__NONE_REVB 0
#define debSerial__B_UART__EVEN_REVB 1
#define debSerial__B_UART__ODD_REVB 2
#define debSerial__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define debSerial_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define debSerial_NUMBER_OF_STOP_BITS    (1u)

#if (debSerial_RXHW_ADDRESS_ENABLED)
    #define debSerial_RX_ADDRESS_MODE    (0u)
    #define debSerial_RX_HW_ADDRESS1     (0u)
    #define debSerial_RX_HW_ADDRESS2     (0u)
#endif /* (debSerial_RXHW_ADDRESS_ENABLED) */

#define debSerial_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << debSerial_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << debSerial_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << debSerial_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << debSerial_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << debSerial_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << debSerial_RX_STS_BREAK_SHIFT) \
                                        | (0 << debSerial_RX_STS_OVERRUN_SHIFT))

#define debSerial_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << debSerial_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << debSerial_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << debSerial_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << debSerial_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define debSerial_CONTROL_REG \
                            (* (reg8 *) debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define debSerial_CONTROL_PTR \
                            (  (reg8 *) debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(debSerial_TX_ENABLED)
    #define debSerial_TXDATA_REG          (* (reg8 *) debSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define debSerial_TXDATA_PTR          (  (reg8 *) debSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define debSerial_TXDATA_AUX_CTL_REG  (* (reg8 *) debSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define debSerial_TXDATA_AUX_CTL_PTR  (  (reg8 *) debSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define debSerial_TXSTATUS_REG        (* (reg8 *) debSerial_BUART_sTX_TxSts__STATUS_REG)
    #define debSerial_TXSTATUS_PTR        (  (reg8 *) debSerial_BUART_sTX_TxSts__STATUS_REG)
    #define debSerial_TXSTATUS_MASK_REG   (* (reg8 *) debSerial_BUART_sTX_TxSts__MASK_REG)
    #define debSerial_TXSTATUS_MASK_PTR   (  (reg8 *) debSerial_BUART_sTX_TxSts__MASK_REG)
    #define debSerial_TXSTATUS_ACTL_REG   (* (reg8 *) debSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define debSerial_TXSTATUS_ACTL_PTR   (  (reg8 *) debSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(debSerial_TXCLKGEN_DP)
        #define debSerial_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define debSerial_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define debSerial_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define debSerial_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define debSerial_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define debSerial_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define debSerial_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define debSerial_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define debSerial_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define debSerial_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) debSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* debSerial_TXCLKGEN_DP */

#endif /* End debSerial_TX_ENABLED */

#if(debSerial_HD_ENABLED)

    #define debSerial_TXDATA_REG             (* (reg8 *) debSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define debSerial_TXDATA_PTR             (  (reg8 *) debSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define debSerial_TXDATA_AUX_CTL_REG     (* (reg8 *) debSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define debSerial_TXDATA_AUX_CTL_PTR     (  (reg8 *) debSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define debSerial_TXSTATUS_REG           (* (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debSerial_TXSTATUS_PTR           (  (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debSerial_TXSTATUS_MASK_REG      (* (reg8 *) debSerial_BUART_sRX_RxSts__MASK_REG )
    #define debSerial_TXSTATUS_MASK_PTR      (  (reg8 *) debSerial_BUART_sRX_RxSts__MASK_REG )
    #define debSerial_TXSTATUS_ACTL_REG      (* (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define debSerial_TXSTATUS_ACTL_PTR      (  (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End debSerial_HD_ENABLED */

#if( (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) )
    #define debSerial_RXDATA_REG             (* (reg8 *) debSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define debSerial_RXDATA_PTR             (  (reg8 *) debSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define debSerial_RXADDRESS1_REG         (* (reg8 *) debSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define debSerial_RXADDRESS1_PTR         (  (reg8 *) debSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define debSerial_RXADDRESS2_REG         (* (reg8 *) debSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define debSerial_RXADDRESS2_PTR         (  (reg8 *) debSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define debSerial_RXDATA_AUX_CTL_REG     (* (reg8 *) debSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define debSerial_RXBITCTR_PERIOD_REG    (* (reg8 *) debSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define debSerial_RXBITCTR_PERIOD_PTR    (  (reg8 *) debSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define debSerial_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) debSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define debSerial_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) debSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define debSerial_RXBITCTR_COUNTER_REG   (* (reg8 *) debSerial_BUART_sRX_RxBitCounter__COUNT_REG )
    #define debSerial_RXBITCTR_COUNTER_PTR   (  (reg8 *) debSerial_BUART_sRX_RxBitCounter__COUNT_REG )

    #define debSerial_RXSTATUS_REG           (* (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debSerial_RXSTATUS_PTR           (  (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_REG )
    #define debSerial_RXSTATUS_MASK_REG      (* (reg8 *) debSerial_BUART_sRX_RxSts__MASK_REG )
    #define debSerial_RXSTATUS_MASK_PTR      (  (reg8 *) debSerial_BUART_sRX_RxSts__MASK_REG )
    #define debSerial_RXSTATUS_ACTL_REG      (* (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define debSerial_RXSTATUS_ACTL_PTR      (  (reg8 *) debSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) */

#if(debSerial_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define debSerial_INTCLOCK_CLKEN_REG     (* (reg8 *) debSerial_IntClock__PM_ACT_CFG)
    #define debSerial_INTCLOCK_CLKEN_PTR     (  (reg8 *) debSerial_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define debSerial_INTCLOCK_CLKEN_MASK    debSerial_IntClock__PM_ACT_MSK
#endif /* End debSerial_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(debSerial_TX_ENABLED)
    #define debSerial_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End debSerial_TX_ENABLED */

#if(debSerial_HD_ENABLED)
    #define debSerial_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End debSerial_HD_ENABLED */

#if( (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) )
    #define debSerial_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define debSerial_WAIT_1_MS      debSerial_BL_CHK_DELAY_MS   

#define debSerial_TXBUFFERSIZE   debSerial_TX_BUFFER_SIZE
#define debSerial_RXBUFFERSIZE   debSerial_RX_BUFFER_SIZE

#if (debSerial_RXHW_ADDRESS_ENABLED)
    #define debSerial_RXADDRESSMODE  debSerial_RX_ADDRESS_MODE
    #define debSerial_RXHWADDRESS1   debSerial_RX_HW_ADDRESS1
    #define debSerial_RXHWADDRESS2   debSerial_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define debSerial_RXAddressMode  debSerial_RXADDRESSMODE
#endif /* (debSerial_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define debSerial_initvar                    debSerial_initVar

#define debSerial_RX_Enabled                 debSerial_RX_ENABLED
#define debSerial_TX_Enabled                 debSerial_TX_ENABLED
#define debSerial_HD_Enabled                 debSerial_HD_ENABLED
#define debSerial_RX_IntInterruptEnabled     debSerial_RX_INTERRUPT_ENABLED
#define debSerial_TX_IntInterruptEnabled     debSerial_TX_INTERRUPT_ENABLED
#define debSerial_InternalClockUsed          debSerial_INTERNAL_CLOCK_USED
#define debSerial_RXHW_Address_Enabled       debSerial_RXHW_ADDRESS_ENABLED
#define debSerial_OverSampleCount            debSerial_OVER_SAMPLE_COUNT
#define debSerial_ParityType                 debSerial_PARITY_TYPE

#if( debSerial_TX_ENABLED && (debSerial_TXBUFFERSIZE > debSerial_FIFO_LENGTH))
    #define debSerial_TXBUFFER               debSerial_txBuffer
    #define debSerial_TXBUFFERREAD           debSerial_txBufferRead
    #define debSerial_TXBUFFERWRITE          debSerial_txBufferWrite
#endif /* End debSerial_TX_ENABLED */
#if( ( debSerial_RX_ENABLED || debSerial_HD_ENABLED ) && \
     (debSerial_RXBUFFERSIZE > debSerial_FIFO_LENGTH) )
    #define debSerial_RXBUFFER               debSerial_rxBuffer
    #define debSerial_RXBUFFERREAD           debSerial_rxBufferRead
    #define debSerial_RXBUFFERWRITE          debSerial_rxBufferWrite
    #define debSerial_RXBUFFERLOOPDETECT     debSerial_rxBufferLoopDetect
    #define debSerial_RXBUFFER_OVERFLOW      debSerial_rxBufferOverflow
#endif /* End debSerial_RX_ENABLED */

#ifdef debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define debSerial_CONTROL                debSerial_CONTROL_REG
#endif /* End debSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(debSerial_TX_ENABLED)
    #define debSerial_TXDATA                 debSerial_TXDATA_REG
    #define debSerial_TXSTATUS               debSerial_TXSTATUS_REG
    #define debSerial_TXSTATUS_MASK          debSerial_TXSTATUS_MASK_REG
    #define debSerial_TXSTATUS_ACTL          debSerial_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(debSerial_TXCLKGEN_DP)
        #define debSerial_TXBITCLKGEN_CTR        debSerial_TXBITCLKGEN_CTR_REG
        #define debSerial_TXBITCLKTX_COMPLETE    debSerial_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define debSerial_TXBITCTR_PERIOD        debSerial_TXBITCTR_PERIOD_REG
        #define debSerial_TXBITCTR_CONTROL       debSerial_TXBITCTR_CONTROL_REG
        #define debSerial_TXBITCTR_COUNTER       debSerial_TXBITCTR_COUNTER_REG
    #endif /* debSerial_TXCLKGEN_DP */
#endif /* End debSerial_TX_ENABLED */

#if(debSerial_HD_ENABLED)
    #define debSerial_TXDATA                 debSerial_TXDATA_REG
    #define debSerial_TXSTATUS               debSerial_TXSTATUS_REG
    #define debSerial_TXSTATUS_MASK          debSerial_TXSTATUS_MASK_REG
    #define debSerial_TXSTATUS_ACTL          debSerial_TXSTATUS_ACTL_REG
#endif /* End debSerial_HD_ENABLED */

#if( (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) )
    #define debSerial_RXDATA                 debSerial_RXDATA_REG
    #define debSerial_RXADDRESS1             debSerial_RXADDRESS1_REG
    #define debSerial_RXADDRESS2             debSerial_RXADDRESS2_REG
    #define debSerial_RXBITCTR_PERIOD        debSerial_RXBITCTR_PERIOD_REG
    #define debSerial_RXBITCTR_CONTROL       debSerial_RXBITCTR_CONTROL_REG
    #define debSerial_RXBITCTR_COUNTER       debSerial_RXBITCTR_COUNTER_REG
    #define debSerial_RXSTATUS               debSerial_RXSTATUS_REG
    #define debSerial_RXSTATUS_MASK          debSerial_RXSTATUS_MASK_REG
    #define debSerial_RXSTATUS_ACTL          debSerial_RXSTATUS_ACTL_REG
#endif /* End  (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) */

#if(debSerial_INTERNAL_CLOCK_USED)
    #define debSerial_INTCLOCK_CLKEN         debSerial_INTCLOCK_CLKEN_REG
#endif /* End debSerial_INTERNAL_CLOCK_USED */

#define debSerial_WAIT_FOR_COMLETE_REINIT    debSerial_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_debSerial_H */


/* [] END OF FILE */
