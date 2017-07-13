/*******************************************************************************
* File Name: gpsSerial.h
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


#if !defined(CY_UART_gpsSerial_H)
#define CY_UART_gpsSerial_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define gpsSerial_RX_ENABLED                     (1u)
#define gpsSerial_TX_ENABLED                     (1u)
#define gpsSerial_HD_ENABLED                     (0u)
#define gpsSerial_RX_INTERRUPT_ENABLED           (1u)
#define gpsSerial_TX_INTERRUPT_ENABLED           (1u)
#define gpsSerial_INTERNAL_CLOCK_USED            (1u)
#define gpsSerial_RXHW_ADDRESS_ENABLED           (0u)
#define gpsSerial_OVER_SAMPLE_COUNT              (8u)
#define gpsSerial_PARITY_TYPE                    (0u)
#define gpsSerial_PARITY_TYPE_SW                 (0u)
#define gpsSerial_BREAK_DETECT                   (0u)
#define gpsSerial_BREAK_BITS_TX                  (13u)
#define gpsSerial_BREAK_BITS_RX                  (13u)
#define gpsSerial_TXCLKGEN_DP                    (1u)
#define gpsSerial_USE23POLLING                   (1u)
#define gpsSerial_FLOW_CONTROL                   (0u)
#define gpsSerial_CLK_FREQ                       (0u)
#define gpsSerial_TX_BUFFER_SIZE                 (100u)
#define gpsSerial_RX_BUFFER_SIZE                 (100u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define gpsSerial_CONTROL_REG_REMOVED            (0u)
#else
    #define gpsSerial_CONTROL_REG_REMOVED            (1u)
#endif /* End gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct gpsSerial_backupStruct_
{
    uint8 enableState;

    #if(gpsSerial_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End gpsSerial_CONTROL_REG_REMOVED */

} gpsSerial_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void gpsSerial_Start(void) ;
void gpsSerial_Stop(void) ;
uint8 gpsSerial_ReadControlRegister(void) ;
void gpsSerial_WriteControlRegister(uint8 control) ;

void gpsSerial_Init(void) ;
void gpsSerial_Enable(void) ;
void gpsSerial_SaveConfig(void) ;
void gpsSerial_RestoreConfig(void) ;
void gpsSerial_Sleep(void) ;
void gpsSerial_Wakeup(void) ;

/* Only if RX is enabled */
#if( (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) )

    #if (gpsSerial_RX_INTERRUPT_ENABLED)
        #define gpsSerial_EnableRxInt()  CyIntEnable (gpsSerial_RX_VECT_NUM)
        #define gpsSerial_DisableRxInt() CyIntDisable(gpsSerial_RX_VECT_NUM)
        CY_ISR_PROTO(gpsSerial_RXISR);
    #endif /* gpsSerial_RX_INTERRUPT_ENABLED */

    void gpsSerial_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void gpsSerial_SetRxAddress1(uint8 address) ;
    void gpsSerial_SetRxAddress2(uint8 address) ;

    void  gpsSerial_SetRxInterruptMode(uint8 intSrc) ;
    uint8 gpsSerial_ReadRxData(void) ;
    uint8 gpsSerial_ReadRxStatus(void) ;
    uint8 gpsSerial_GetChar(void) ;
    uint16 gpsSerial_GetByte(void) ;
    uint8 gpsSerial_GetRxBufferSize(void)
                                                            ;
    void gpsSerial_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define gpsSerial_GetRxInterruptSource   gpsSerial_ReadRxStatus

#endif /* End (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) */

/* Only if TX is enabled */
#if(gpsSerial_TX_ENABLED || gpsSerial_HD_ENABLED)

    #if(gpsSerial_TX_INTERRUPT_ENABLED)
        #define gpsSerial_EnableTxInt()  CyIntEnable (gpsSerial_TX_VECT_NUM)
        #define gpsSerial_DisableTxInt() CyIntDisable(gpsSerial_TX_VECT_NUM)
        #define gpsSerial_SetPendingTxInt() CyIntSetPending(gpsSerial_TX_VECT_NUM)
        #define gpsSerial_ClearPendingTxInt() CyIntClearPending(gpsSerial_TX_VECT_NUM)
        CY_ISR_PROTO(gpsSerial_TXISR);
    #endif /* gpsSerial_TX_INTERRUPT_ENABLED */

    void gpsSerial_SetTxInterruptMode(uint8 intSrc) ;
    void gpsSerial_WriteTxData(uint8 txDataByte) ;
    uint8 gpsSerial_ReadTxStatus(void) ;
    void gpsSerial_PutChar(uint8 txDataByte) ;
    void gpsSerial_PutString(const char8 string[]) ;
    void gpsSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void gpsSerial_PutCRLF(uint8 txDataByte) ;
    void gpsSerial_ClearTxBuffer(void) ;
    void gpsSerial_SetTxAddressMode(uint8 addressMode) ;
    void gpsSerial_SendBreak(uint8 retMode) ;
    uint8 gpsSerial_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define gpsSerial_PutStringConst         gpsSerial_PutString
    #define gpsSerial_PutArrayConst          gpsSerial_PutArray
    #define gpsSerial_GetTxInterruptSource   gpsSerial_ReadTxStatus

#endif /* End gpsSerial_TX_ENABLED || gpsSerial_HD_ENABLED */

#if(gpsSerial_HD_ENABLED)
    void gpsSerial_LoadRxConfig(void) ;
    void gpsSerial_LoadTxConfig(void) ;
#endif /* End gpsSerial_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_gpsSerial) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    gpsSerial_CyBtldrCommStart(void) CYSMALL ;
    void    gpsSerial_CyBtldrCommStop(void) CYSMALL ;
    void    gpsSerial_CyBtldrCommReset(void) CYSMALL ;
    cystatus gpsSerial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus gpsSerial_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_gpsSerial)
        #define CyBtldrCommStart    gpsSerial_CyBtldrCommStart
        #define CyBtldrCommStop     gpsSerial_CyBtldrCommStop
        #define CyBtldrCommReset    gpsSerial_CyBtldrCommReset
        #define CyBtldrCommWrite    gpsSerial_CyBtldrCommWrite
        #define CyBtldrCommRead     gpsSerial_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_gpsSerial) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define gpsSerial_BYTE2BYTE_TIME_OUT (25u)
    #define gpsSerial_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define gpsSerial_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define gpsSerial_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define gpsSerial_SET_SPACE      (0x00u)
#define gpsSerial_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (gpsSerial_TX_ENABLED) || (gpsSerial_HD_ENABLED) )
    #if(gpsSerial_TX_INTERRUPT_ENABLED)
        #define gpsSerial_TX_VECT_NUM            (uint8)gpsSerial_TXInternalInterrupt__INTC_NUMBER
        #define gpsSerial_TX_PRIOR_NUM           (uint8)gpsSerial_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* gpsSerial_TX_INTERRUPT_ENABLED */

    #define gpsSerial_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define gpsSerial_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define gpsSerial_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(gpsSerial_TX_ENABLED)
        #define gpsSerial_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (gpsSerial_HD_ENABLED) */
        #define gpsSerial_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (gpsSerial_TX_ENABLED) */

    #define gpsSerial_TX_STS_COMPLETE            (uint8)(0x01u << gpsSerial_TX_STS_COMPLETE_SHIFT)
    #define gpsSerial_TX_STS_FIFO_EMPTY          (uint8)(0x01u << gpsSerial_TX_STS_FIFO_EMPTY_SHIFT)
    #define gpsSerial_TX_STS_FIFO_FULL           (uint8)(0x01u << gpsSerial_TX_STS_FIFO_FULL_SHIFT)
    #define gpsSerial_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << gpsSerial_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (gpsSerial_TX_ENABLED) || (gpsSerial_HD_ENABLED)*/

#if( (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) )
    #if(gpsSerial_RX_INTERRUPT_ENABLED)
        #define gpsSerial_RX_VECT_NUM            (uint8)gpsSerial_RXInternalInterrupt__INTC_NUMBER
        #define gpsSerial_RX_PRIOR_NUM           (uint8)gpsSerial_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* gpsSerial_RX_INTERRUPT_ENABLED */
    #define gpsSerial_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define gpsSerial_RX_STS_BREAK_SHIFT             (0x01u)
    #define gpsSerial_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define gpsSerial_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define gpsSerial_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define gpsSerial_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define gpsSerial_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define gpsSerial_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define gpsSerial_RX_STS_MRKSPC           (uint8)(0x01u << gpsSerial_RX_STS_MRKSPC_SHIFT)
    #define gpsSerial_RX_STS_BREAK            (uint8)(0x01u << gpsSerial_RX_STS_BREAK_SHIFT)
    #define gpsSerial_RX_STS_PAR_ERROR        (uint8)(0x01u << gpsSerial_RX_STS_PAR_ERROR_SHIFT)
    #define gpsSerial_RX_STS_STOP_ERROR       (uint8)(0x01u << gpsSerial_RX_STS_STOP_ERROR_SHIFT)
    #define gpsSerial_RX_STS_OVERRUN          (uint8)(0x01u << gpsSerial_RX_STS_OVERRUN_SHIFT)
    #define gpsSerial_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << gpsSerial_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define gpsSerial_RX_STS_ADDR_MATCH       (uint8)(0x01u << gpsSerial_RX_STS_ADDR_MATCH_SHIFT)
    #define gpsSerial_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << gpsSerial_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define gpsSerial_RX_HW_MASK                     (0x7Fu)
#endif /* End (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) */

/* Control Register definitions */
#define gpsSerial_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define gpsSerial_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define gpsSerial_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define gpsSerial_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define gpsSerial_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define gpsSerial_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define gpsSerial_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define gpsSerial_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define gpsSerial_CTRL_HD_SEND               (uint8)(0x01u << gpsSerial_CTRL_HD_SEND_SHIFT)
#define gpsSerial_CTRL_HD_SEND_BREAK         (uint8)(0x01u << gpsSerial_CTRL_HD_SEND_BREAK_SHIFT)
#define gpsSerial_CTRL_MARK                  (uint8)(0x01u << gpsSerial_CTRL_MARK_SHIFT)
#define gpsSerial_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << gpsSerial_CTRL_PARITY_TYPE0_SHIFT)
#define gpsSerial_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << gpsSerial_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define gpsSerial_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define gpsSerial_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define gpsSerial_SEND_BREAK                         (0x00u)
#define gpsSerial_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define gpsSerial_REINIT                             (0x02u)
#define gpsSerial_SEND_WAIT_REINIT                   (0x03u)

#define gpsSerial_OVER_SAMPLE_8                      (8u)
#define gpsSerial_OVER_SAMPLE_16                     (16u)

#define gpsSerial_BIT_CENTER                         (gpsSerial_OVER_SAMPLE_COUNT - 2u)

#define gpsSerial_FIFO_LENGTH                        (4u)
#define gpsSerial_NUMBER_OF_START_BIT                (1u)
#define gpsSerial_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define gpsSerial_TXBITCTR_BREAKBITS8X   ((gpsSerial_BREAK_BITS_TX * gpsSerial_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define gpsSerial_TXBITCTR_BREAKBITS ((gpsSerial_BREAK_BITS_TX * gpsSerial_OVER_SAMPLE_COUNT) - 1u)

#define gpsSerial_HALF_BIT_COUNT   \
                            (((gpsSerial_OVER_SAMPLE_COUNT / 2u) + (gpsSerial_USE23POLLING * 1u)) - 2u)
#if (gpsSerial_OVER_SAMPLE_COUNT == gpsSerial_OVER_SAMPLE_8)
    #define gpsSerial_HD_TXBITCTR_INIT   (((gpsSerial_BREAK_BITS_TX + \
                            gpsSerial_NUMBER_OF_START_BIT) * gpsSerial_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define gpsSerial_RXBITCTR_INIT  ((((gpsSerial_BREAK_BITS_RX + gpsSerial_NUMBER_OF_START_BIT) \
                            * gpsSerial_OVER_SAMPLE_COUNT) + gpsSerial_HALF_BIT_COUNT) - 1u)

#else /* gpsSerial_OVER_SAMPLE_COUNT == gpsSerial_OVER_SAMPLE_16 */
    #define gpsSerial_HD_TXBITCTR_INIT   ((8u * gpsSerial_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define gpsSerial_RXBITCTR_INIT      (((7u * gpsSerial_OVER_SAMPLE_COUNT) - 1u) + \
                                                      gpsSerial_HALF_BIT_COUNT)
#endif /* End gpsSerial_OVER_SAMPLE_COUNT */

#define gpsSerial_HD_RXBITCTR_INIT                   gpsSerial_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 gpsSerial_initVar;
#if (gpsSerial_TX_INTERRUPT_ENABLED && gpsSerial_TX_ENABLED)
    extern volatile uint8 gpsSerial_txBuffer[gpsSerial_TX_BUFFER_SIZE];
    extern volatile uint8 gpsSerial_txBufferRead;
    extern uint8 gpsSerial_txBufferWrite;
#endif /* (gpsSerial_TX_INTERRUPT_ENABLED && gpsSerial_TX_ENABLED) */
#if (gpsSerial_RX_INTERRUPT_ENABLED && (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED))
    extern uint8 gpsSerial_errorStatus;
    extern volatile uint8 gpsSerial_rxBuffer[gpsSerial_RX_BUFFER_SIZE];
    extern volatile uint8 gpsSerial_rxBufferRead;
    extern volatile uint8 gpsSerial_rxBufferWrite;
    extern volatile uint8 gpsSerial_rxBufferLoopDetect;
    extern volatile uint8 gpsSerial_rxBufferOverflow;
    #if (gpsSerial_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 gpsSerial_rxAddressMode;
        extern volatile uint8 gpsSerial_rxAddressDetected;
    #endif /* (gpsSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (gpsSerial_RX_INTERRUPT_ENABLED && (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define gpsSerial__B_UART__AM_SW_BYTE_BYTE 1
#define gpsSerial__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define gpsSerial__B_UART__AM_HW_BYTE_BY_BYTE 3
#define gpsSerial__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define gpsSerial__B_UART__AM_NONE 0

#define gpsSerial__B_UART__NONE_REVB 0
#define gpsSerial__B_UART__EVEN_REVB 1
#define gpsSerial__B_UART__ODD_REVB 2
#define gpsSerial__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define gpsSerial_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define gpsSerial_NUMBER_OF_STOP_BITS    (1u)

#if (gpsSerial_RXHW_ADDRESS_ENABLED)
    #define gpsSerial_RX_ADDRESS_MODE    (0u)
    #define gpsSerial_RX_HW_ADDRESS1     (0u)
    #define gpsSerial_RX_HW_ADDRESS2     (0u)
#endif /* (gpsSerial_RXHW_ADDRESS_ENABLED) */

#define gpsSerial_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << gpsSerial_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << gpsSerial_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << gpsSerial_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << gpsSerial_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << gpsSerial_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << gpsSerial_RX_STS_BREAK_SHIFT) \
                                        | (0 << gpsSerial_RX_STS_OVERRUN_SHIFT))

#define gpsSerial_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << gpsSerial_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << gpsSerial_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << gpsSerial_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << gpsSerial_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define gpsSerial_CONTROL_REG \
                            (* (reg8 *) gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define gpsSerial_CONTROL_PTR \
                            (  (reg8 *) gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(gpsSerial_TX_ENABLED)
    #define gpsSerial_TXDATA_REG          (* (reg8 *) gpsSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define gpsSerial_TXDATA_PTR          (  (reg8 *) gpsSerial_BUART_sTX_TxShifter_u0__F0_REG)
    #define gpsSerial_TXDATA_AUX_CTL_REG  (* (reg8 *) gpsSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define gpsSerial_TXDATA_AUX_CTL_PTR  (  (reg8 *) gpsSerial_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define gpsSerial_TXSTATUS_REG        (* (reg8 *) gpsSerial_BUART_sTX_TxSts__STATUS_REG)
    #define gpsSerial_TXSTATUS_PTR        (  (reg8 *) gpsSerial_BUART_sTX_TxSts__STATUS_REG)
    #define gpsSerial_TXSTATUS_MASK_REG   (* (reg8 *) gpsSerial_BUART_sTX_TxSts__MASK_REG)
    #define gpsSerial_TXSTATUS_MASK_PTR   (  (reg8 *) gpsSerial_BUART_sTX_TxSts__MASK_REG)
    #define gpsSerial_TXSTATUS_ACTL_REG   (* (reg8 *) gpsSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define gpsSerial_TXSTATUS_ACTL_PTR   (  (reg8 *) gpsSerial_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(gpsSerial_TXCLKGEN_DP)
        #define gpsSerial_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define gpsSerial_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define gpsSerial_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define gpsSerial_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define gpsSerial_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define gpsSerial_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define gpsSerial_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define gpsSerial_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define gpsSerial_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define gpsSerial_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) gpsSerial_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* gpsSerial_TXCLKGEN_DP */

#endif /* End gpsSerial_TX_ENABLED */

#if(gpsSerial_HD_ENABLED)

    #define gpsSerial_TXDATA_REG             (* (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define gpsSerial_TXDATA_PTR             (  (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__F1_REG )
    #define gpsSerial_TXDATA_AUX_CTL_REG     (* (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define gpsSerial_TXDATA_AUX_CTL_PTR     (  (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define gpsSerial_TXSTATUS_REG           (* (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_REG )
    #define gpsSerial_TXSTATUS_PTR           (  (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_REG )
    #define gpsSerial_TXSTATUS_MASK_REG      (* (reg8 *) gpsSerial_BUART_sRX_RxSts__MASK_REG )
    #define gpsSerial_TXSTATUS_MASK_PTR      (  (reg8 *) gpsSerial_BUART_sRX_RxSts__MASK_REG )
    #define gpsSerial_TXSTATUS_ACTL_REG      (* (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define gpsSerial_TXSTATUS_ACTL_PTR      (  (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End gpsSerial_HD_ENABLED */

#if( (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) )
    #define gpsSerial_RXDATA_REG             (* (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define gpsSerial_RXDATA_PTR             (  (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__F0_REG )
    #define gpsSerial_RXADDRESS1_REG         (* (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define gpsSerial_RXADDRESS1_PTR         (  (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__D0_REG )
    #define gpsSerial_RXADDRESS2_REG         (* (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define gpsSerial_RXADDRESS2_PTR         (  (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__D1_REG )
    #define gpsSerial_RXDATA_AUX_CTL_REG     (* (reg8 *) gpsSerial_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define gpsSerial_RXBITCTR_PERIOD_REG    (* (reg8 *) gpsSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define gpsSerial_RXBITCTR_PERIOD_PTR    (  (reg8 *) gpsSerial_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define gpsSerial_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) gpsSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define gpsSerial_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) gpsSerial_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define gpsSerial_RXBITCTR_COUNTER_REG   (* (reg8 *) gpsSerial_BUART_sRX_RxBitCounter__COUNT_REG )
    #define gpsSerial_RXBITCTR_COUNTER_PTR   (  (reg8 *) gpsSerial_BUART_sRX_RxBitCounter__COUNT_REG )

    #define gpsSerial_RXSTATUS_REG           (* (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_REG )
    #define gpsSerial_RXSTATUS_PTR           (  (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_REG )
    #define gpsSerial_RXSTATUS_MASK_REG      (* (reg8 *) gpsSerial_BUART_sRX_RxSts__MASK_REG )
    #define gpsSerial_RXSTATUS_MASK_PTR      (  (reg8 *) gpsSerial_BUART_sRX_RxSts__MASK_REG )
    #define gpsSerial_RXSTATUS_ACTL_REG      (* (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define gpsSerial_RXSTATUS_ACTL_PTR      (  (reg8 *) gpsSerial_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) */

#if(gpsSerial_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define gpsSerial_INTCLOCK_CLKEN_REG     (* (reg8 *) gpsSerial_IntClock__PM_ACT_CFG)
    #define gpsSerial_INTCLOCK_CLKEN_PTR     (  (reg8 *) gpsSerial_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define gpsSerial_INTCLOCK_CLKEN_MASK    gpsSerial_IntClock__PM_ACT_MSK
#endif /* End gpsSerial_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(gpsSerial_TX_ENABLED)
    #define gpsSerial_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End gpsSerial_TX_ENABLED */

#if(gpsSerial_HD_ENABLED)
    #define gpsSerial_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End gpsSerial_HD_ENABLED */

#if( (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) )
    #define gpsSerial_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define gpsSerial_WAIT_1_MS      gpsSerial_BL_CHK_DELAY_MS   

#define gpsSerial_TXBUFFERSIZE   gpsSerial_TX_BUFFER_SIZE
#define gpsSerial_RXBUFFERSIZE   gpsSerial_RX_BUFFER_SIZE

#if (gpsSerial_RXHW_ADDRESS_ENABLED)
    #define gpsSerial_RXADDRESSMODE  gpsSerial_RX_ADDRESS_MODE
    #define gpsSerial_RXHWADDRESS1   gpsSerial_RX_HW_ADDRESS1
    #define gpsSerial_RXHWADDRESS2   gpsSerial_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define gpsSerial_RXAddressMode  gpsSerial_RXADDRESSMODE
#endif /* (gpsSerial_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define gpsSerial_initvar                    gpsSerial_initVar

#define gpsSerial_RX_Enabled                 gpsSerial_RX_ENABLED
#define gpsSerial_TX_Enabled                 gpsSerial_TX_ENABLED
#define gpsSerial_HD_Enabled                 gpsSerial_HD_ENABLED
#define gpsSerial_RX_IntInterruptEnabled     gpsSerial_RX_INTERRUPT_ENABLED
#define gpsSerial_TX_IntInterruptEnabled     gpsSerial_TX_INTERRUPT_ENABLED
#define gpsSerial_InternalClockUsed          gpsSerial_INTERNAL_CLOCK_USED
#define gpsSerial_RXHW_Address_Enabled       gpsSerial_RXHW_ADDRESS_ENABLED
#define gpsSerial_OverSampleCount            gpsSerial_OVER_SAMPLE_COUNT
#define gpsSerial_ParityType                 gpsSerial_PARITY_TYPE

#if( gpsSerial_TX_ENABLED && (gpsSerial_TXBUFFERSIZE > gpsSerial_FIFO_LENGTH))
    #define gpsSerial_TXBUFFER               gpsSerial_txBuffer
    #define gpsSerial_TXBUFFERREAD           gpsSerial_txBufferRead
    #define gpsSerial_TXBUFFERWRITE          gpsSerial_txBufferWrite
#endif /* End gpsSerial_TX_ENABLED */
#if( ( gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED ) && \
     (gpsSerial_RXBUFFERSIZE > gpsSerial_FIFO_LENGTH) )
    #define gpsSerial_RXBUFFER               gpsSerial_rxBuffer
    #define gpsSerial_RXBUFFERREAD           gpsSerial_rxBufferRead
    #define gpsSerial_RXBUFFERWRITE          gpsSerial_rxBufferWrite
    #define gpsSerial_RXBUFFERLOOPDETECT     gpsSerial_rxBufferLoopDetect
    #define gpsSerial_RXBUFFER_OVERFLOW      gpsSerial_rxBufferOverflow
#endif /* End gpsSerial_RX_ENABLED */

#ifdef gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define gpsSerial_CONTROL                gpsSerial_CONTROL_REG
#endif /* End gpsSerial_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(gpsSerial_TX_ENABLED)
    #define gpsSerial_TXDATA                 gpsSerial_TXDATA_REG
    #define gpsSerial_TXSTATUS               gpsSerial_TXSTATUS_REG
    #define gpsSerial_TXSTATUS_MASK          gpsSerial_TXSTATUS_MASK_REG
    #define gpsSerial_TXSTATUS_ACTL          gpsSerial_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(gpsSerial_TXCLKGEN_DP)
        #define gpsSerial_TXBITCLKGEN_CTR        gpsSerial_TXBITCLKGEN_CTR_REG
        #define gpsSerial_TXBITCLKTX_COMPLETE    gpsSerial_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define gpsSerial_TXBITCTR_PERIOD        gpsSerial_TXBITCTR_PERIOD_REG
        #define gpsSerial_TXBITCTR_CONTROL       gpsSerial_TXBITCTR_CONTROL_REG
        #define gpsSerial_TXBITCTR_COUNTER       gpsSerial_TXBITCTR_COUNTER_REG
    #endif /* gpsSerial_TXCLKGEN_DP */
#endif /* End gpsSerial_TX_ENABLED */

#if(gpsSerial_HD_ENABLED)
    #define gpsSerial_TXDATA                 gpsSerial_TXDATA_REG
    #define gpsSerial_TXSTATUS               gpsSerial_TXSTATUS_REG
    #define gpsSerial_TXSTATUS_MASK          gpsSerial_TXSTATUS_MASK_REG
    #define gpsSerial_TXSTATUS_ACTL          gpsSerial_TXSTATUS_ACTL_REG
#endif /* End gpsSerial_HD_ENABLED */

#if( (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) )
    #define gpsSerial_RXDATA                 gpsSerial_RXDATA_REG
    #define gpsSerial_RXADDRESS1             gpsSerial_RXADDRESS1_REG
    #define gpsSerial_RXADDRESS2             gpsSerial_RXADDRESS2_REG
    #define gpsSerial_RXBITCTR_PERIOD        gpsSerial_RXBITCTR_PERIOD_REG
    #define gpsSerial_RXBITCTR_CONTROL       gpsSerial_RXBITCTR_CONTROL_REG
    #define gpsSerial_RXBITCTR_COUNTER       gpsSerial_RXBITCTR_COUNTER_REG
    #define gpsSerial_RXSTATUS               gpsSerial_RXSTATUS_REG
    #define gpsSerial_RXSTATUS_MASK          gpsSerial_RXSTATUS_MASK_REG
    #define gpsSerial_RXSTATUS_ACTL          gpsSerial_RXSTATUS_ACTL_REG
#endif /* End  (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) */

#if(gpsSerial_INTERNAL_CLOCK_USED)
    #define gpsSerial_INTCLOCK_CLKEN         gpsSerial_INTCLOCK_CLKEN_REG
#endif /* End gpsSerial_INTERNAL_CLOCK_USED */

#define gpsSerial_WAIT_FOR_COMLETE_REINIT    gpsSerial_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_gpsSerial_H */


/* [] END OF FILE */
