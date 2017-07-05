/*******************************************************************************
* File Name: killSwitch.h
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


#if !defined(CY_UART_killSwitch_H)
#define CY_UART_killSwitch_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define killSwitch_RX_ENABLED                     (0u)
#define killSwitch_TX_ENABLED                     (1u)
#define killSwitch_HD_ENABLED                     (0u)
#define killSwitch_RX_INTERRUPT_ENABLED           (0u)
#define killSwitch_TX_INTERRUPT_ENABLED           (0u)
#define killSwitch_INTERNAL_CLOCK_USED            (1u)
#define killSwitch_RXHW_ADDRESS_ENABLED           (0u)
#define killSwitch_OVER_SAMPLE_COUNT              (8u)
#define killSwitch_PARITY_TYPE                    (0u)
#define killSwitch_PARITY_TYPE_SW                 (0u)
#define killSwitch_BREAK_DETECT                   (0u)
#define killSwitch_BREAK_BITS_TX                  (13u)
#define killSwitch_BREAK_BITS_RX                  (13u)
#define killSwitch_TXCLKGEN_DP                    (1u)
#define killSwitch_USE23POLLING                   (1u)
#define killSwitch_FLOW_CONTROL                   (0u)
#define killSwitch_CLK_FREQ                       (0u)
#define killSwitch_TX_BUFFER_SIZE                 (4u)
#define killSwitch_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define killSwitch_CONTROL_REG_REMOVED            (0u)
#else
    #define killSwitch_CONTROL_REG_REMOVED            (1u)
#endif /* End killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct killSwitch_backupStruct_
{
    uint8 enableState;

    #if(killSwitch_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End killSwitch_CONTROL_REG_REMOVED */

} killSwitch_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void killSwitch_Start(void) ;
void killSwitch_Stop(void) ;
uint8 killSwitch_ReadControlRegister(void) ;
void killSwitch_WriteControlRegister(uint8 control) ;

void killSwitch_Init(void) ;
void killSwitch_Enable(void) ;
void killSwitch_SaveConfig(void) ;
void killSwitch_RestoreConfig(void) ;
void killSwitch_Sleep(void) ;
void killSwitch_Wakeup(void) ;

/* Only if RX is enabled */
#if( (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) )

    #if (killSwitch_RX_INTERRUPT_ENABLED)
        #define killSwitch_EnableRxInt()  CyIntEnable (killSwitch_RX_VECT_NUM)
        #define killSwitch_DisableRxInt() CyIntDisable(killSwitch_RX_VECT_NUM)
        CY_ISR_PROTO(killSwitch_RXISR);
    #endif /* killSwitch_RX_INTERRUPT_ENABLED */

    void killSwitch_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void killSwitch_SetRxAddress1(uint8 address) ;
    void killSwitch_SetRxAddress2(uint8 address) ;

    void  killSwitch_SetRxInterruptMode(uint8 intSrc) ;
    uint8 killSwitch_ReadRxData(void) ;
    uint8 killSwitch_ReadRxStatus(void) ;
    uint8 killSwitch_GetChar(void) ;
    uint16 killSwitch_GetByte(void) ;
    uint8 killSwitch_GetRxBufferSize(void)
                                                            ;
    void killSwitch_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define killSwitch_GetRxInterruptSource   killSwitch_ReadRxStatus

#endif /* End (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) */

/* Only if TX is enabled */
#if(killSwitch_TX_ENABLED || killSwitch_HD_ENABLED)

    #if(killSwitch_TX_INTERRUPT_ENABLED)
        #define killSwitch_EnableTxInt()  CyIntEnable (killSwitch_TX_VECT_NUM)
        #define killSwitch_DisableTxInt() CyIntDisable(killSwitch_TX_VECT_NUM)
        #define killSwitch_SetPendingTxInt() CyIntSetPending(killSwitch_TX_VECT_NUM)
        #define killSwitch_ClearPendingTxInt() CyIntClearPending(killSwitch_TX_VECT_NUM)
        CY_ISR_PROTO(killSwitch_TXISR);
    #endif /* killSwitch_TX_INTERRUPT_ENABLED */

    void killSwitch_SetTxInterruptMode(uint8 intSrc) ;
    void killSwitch_WriteTxData(uint8 txDataByte) ;
    uint8 killSwitch_ReadTxStatus(void) ;
    void killSwitch_PutChar(uint8 txDataByte) ;
    void killSwitch_PutString(const char8 string[]) ;
    void killSwitch_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void killSwitch_PutCRLF(uint8 txDataByte) ;
    void killSwitch_ClearTxBuffer(void) ;
    void killSwitch_SetTxAddressMode(uint8 addressMode) ;
    void killSwitch_SendBreak(uint8 retMode) ;
    uint8 killSwitch_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define killSwitch_PutStringConst         killSwitch_PutString
    #define killSwitch_PutArrayConst          killSwitch_PutArray
    #define killSwitch_GetTxInterruptSource   killSwitch_ReadTxStatus

#endif /* End killSwitch_TX_ENABLED || killSwitch_HD_ENABLED */

#if(killSwitch_HD_ENABLED)
    void killSwitch_LoadRxConfig(void) ;
    void killSwitch_LoadTxConfig(void) ;
#endif /* End killSwitch_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_killSwitch) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    killSwitch_CyBtldrCommStart(void) CYSMALL ;
    void    killSwitch_CyBtldrCommStop(void) CYSMALL ;
    void    killSwitch_CyBtldrCommReset(void) CYSMALL ;
    cystatus killSwitch_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus killSwitch_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_killSwitch)
        #define CyBtldrCommStart    killSwitch_CyBtldrCommStart
        #define CyBtldrCommStop     killSwitch_CyBtldrCommStop
        #define CyBtldrCommReset    killSwitch_CyBtldrCommReset
        #define CyBtldrCommWrite    killSwitch_CyBtldrCommWrite
        #define CyBtldrCommRead     killSwitch_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_killSwitch) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define killSwitch_BYTE2BYTE_TIME_OUT (25u)
    #define killSwitch_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define killSwitch_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define killSwitch_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define killSwitch_SET_SPACE      (0x00u)
#define killSwitch_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (killSwitch_TX_ENABLED) || (killSwitch_HD_ENABLED) )
    #if(killSwitch_TX_INTERRUPT_ENABLED)
        #define killSwitch_TX_VECT_NUM            (uint8)killSwitch_TXInternalInterrupt__INTC_NUMBER
        #define killSwitch_TX_PRIOR_NUM           (uint8)killSwitch_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* killSwitch_TX_INTERRUPT_ENABLED */

    #define killSwitch_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define killSwitch_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define killSwitch_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(killSwitch_TX_ENABLED)
        #define killSwitch_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (killSwitch_HD_ENABLED) */
        #define killSwitch_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (killSwitch_TX_ENABLED) */

    #define killSwitch_TX_STS_COMPLETE            (uint8)(0x01u << killSwitch_TX_STS_COMPLETE_SHIFT)
    #define killSwitch_TX_STS_FIFO_EMPTY          (uint8)(0x01u << killSwitch_TX_STS_FIFO_EMPTY_SHIFT)
    #define killSwitch_TX_STS_FIFO_FULL           (uint8)(0x01u << killSwitch_TX_STS_FIFO_FULL_SHIFT)
    #define killSwitch_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << killSwitch_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (killSwitch_TX_ENABLED) || (killSwitch_HD_ENABLED)*/

#if( (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) )
    #if(killSwitch_RX_INTERRUPT_ENABLED)
        #define killSwitch_RX_VECT_NUM            (uint8)killSwitch_RXInternalInterrupt__INTC_NUMBER
        #define killSwitch_RX_PRIOR_NUM           (uint8)killSwitch_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* killSwitch_RX_INTERRUPT_ENABLED */
    #define killSwitch_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define killSwitch_RX_STS_BREAK_SHIFT             (0x01u)
    #define killSwitch_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define killSwitch_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define killSwitch_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define killSwitch_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define killSwitch_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define killSwitch_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define killSwitch_RX_STS_MRKSPC           (uint8)(0x01u << killSwitch_RX_STS_MRKSPC_SHIFT)
    #define killSwitch_RX_STS_BREAK            (uint8)(0x01u << killSwitch_RX_STS_BREAK_SHIFT)
    #define killSwitch_RX_STS_PAR_ERROR        (uint8)(0x01u << killSwitch_RX_STS_PAR_ERROR_SHIFT)
    #define killSwitch_RX_STS_STOP_ERROR       (uint8)(0x01u << killSwitch_RX_STS_STOP_ERROR_SHIFT)
    #define killSwitch_RX_STS_OVERRUN          (uint8)(0x01u << killSwitch_RX_STS_OVERRUN_SHIFT)
    #define killSwitch_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << killSwitch_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define killSwitch_RX_STS_ADDR_MATCH       (uint8)(0x01u << killSwitch_RX_STS_ADDR_MATCH_SHIFT)
    #define killSwitch_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << killSwitch_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define killSwitch_RX_HW_MASK                     (0x7Fu)
#endif /* End (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) */

/* Control Register definitions */
#define killSwitch_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define killSwitch_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define killSwitch_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define killSwitch_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define killSwitch_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define killSwitch_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define killSwitch_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define killSwitch_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define killSwitch_CTRL_HD_SEND               (uint8)(0x01u << killSwitch_CTRL_HD_SEND_SHIFT)
#define killSwitch_CTRL_HD_SEND_BREAK         (uint8)(0x01u << killSwitch_CTRL_HD_SEND_BREAK_SHIFT)
#define killSwitch_CTRL_MARK                  (uint8)(0x01u << killSwitch_CTRL_MARK_SHIFT)
#define killSwitch_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << killSwitch_CTRL_PARITY_TYPE0_SHIFT)
#define killSwitch_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << killSwitch_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define killSwitch_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define killSwitch_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define killSwitch_SEND_BREAK                         (0x00u)
#define killSwitch_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define killSwitch_REINIT                             (0x02u)
#define killSwitch_SEND_WAIT_REINIT                   (0x03u)

#define killSwitch_OVER_SAMPLE_8                      (8u)
#define killSwitch_OVER_SAMPLE_16                     (16u)

#define killSwitch_BIT_CENTER                         (killSwitch_OVER_SAMPLE_COUNT - 2u)

#define killSwitch_FIFO_LENGTH                        (4u)
#define killSwitch_NUMBER_OF_START_BIT                (1u)
#define killSwitch_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define killSwitch_TXBITCTR_BREAKBITS8X   ((killSwitch_BREAK_BITS_TX * killSwitch_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define killSwitch_TXBITCTR_BREAKBITS ((killSwitch_BREAK_BITS_TX * killSwitch_OVER_SAMPLE_COUNT) - 1u)

#define killSwitch_HALF_BIT_COUNT   \
                            (((killSwitch_OVER_SAMPLE_COUNT / 2u) + (killSwitch_USE23POLLING * 1u)) - 2u)
#if (killSwitch_OVER_SAMPLE_COUNT == killSwitch_OVER_SAMPLE_8)
    #define killSwitch_HD_TXBITCTR_INIT   (((killSwitch_BREAK_BITS_TX + \
                            killSwitch_NUMBER_OF_START_BIT) * killSwitch_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define killSwitch_RXBITCTR_INIT  ((((killSwitch_BREAK_BITS_RX + killSwitch_NUMBER_OF_START_BIT) \
                            * killSwitch_OVER_SAMPLE_COUNT) + killSwitch_HALF_BIT_COUNT) - 1u)

#else /* killSwitch_OVER_SAMPLE_COUNT == killSwitch_OVER_SAMPLE_16 */
    #define killSwitch_HD_TXBITCTR_INIT   ((8u * killSwitch_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define killSwitch_RXBITCTR_INIT      (((7u * killSwitch_OVER_SAMPLE_COUNT) - 1u) + \
                                                      killSwitch_HALF_BIT_COUNT)
#endif /* End killSwitch_OVER_SAMPLE_COUNT */

#define killSwitch_HD_RXBITCTR_INIT                   killSwitch_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 killSwitch_initVar;
#if (killSwitch_TX_INTERRUPT_ENABLED && killSwitch_TX_ENABLED)
    extern volatile uint8 killSwitch_txBuffer[killSwitch_TX_BUFFER_SIZE];
    extern volatile uint8 killSwitch_txBufferRead;
    extern uint8 killSwitch_txBufferWrite;
#endif /* (killSwitch_TX_INTERRUPT_ENABLED && killSwitch_TX_ENABLED) */
#if (killSwitch_RX_INTERRUPT_ENABLED && (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED))
    extern uint8 killSwitch_errorStatus;
    extern volatile uint8 killSwitch_rxBuffer[killSwitch_RX_BUFFER_SIZE];
    extern volatile uint8 killSwitch_rxBufferRead;
    extern volatile uint8 killSwitch_rxBufferWrite;
    extern volatile uint8 killSwitch_rxBufferLoopDetect;
    extern volatile uint8 killSwitch_rxBufferOverflow;
    #if (killSwitch_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 killSwitch_rxAddressMode;
        extern volatile uint8 killSwitch_rxAddressDetected;
    #endif /* (killSwitch_RXHW_ADDRESS_ENABLED) */
#endif /* (killSwitch_RX_INTERRUPT_ENABLED && (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define killSwitch__B_UART__AM_SW_BYTE_BYTE 1
#define killSwitch__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define killSwitch__B_UART__AM_HW_BYTE_BY_BYTE 3
#define killSwitch__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define killSwitch__B_UART__AM_NONE 0

#define killSwitch__B_UART__NONE_REVB 0
#define killSwitch__B_UART__EVEN_REVB 1
#define killSwitch__B_UART__ODD_REVB 2
#define killSwitch__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define killSwitch_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define killSwitch_NUMBER_OF_STOP_BITS    (1u)

#if (killSwitch_RXHW_ADDRESS_ENABLED)
    #define killSwitch_RX_ADDRESS_MODE    (0u)
    #define killSwitch_RX_HW_ADDRESS1     (0u)
    #define killSwitch_RX_HW_ADDRESS2     (0u)
#endif /* (killSwitch_RXHW_ADDRESS_ENABLED) */

#define killSwitch_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((0 << killSwitch_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << killSwitch_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << killSwitch_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << killSwitch_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << killSwitch_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << killSwitch_RX_STS_BREAK_SHIFT) \
                                        | (0 << killSwitch_RX_STS_OVERRUN_SHIFT))

#define killSwitch_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << killSwitch_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << killSwitch_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << killSwitch_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << killSwitch_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define killSwitch_CONTROL_REG \
                            (* (reg8 *) killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define killSwitch_CONTROL_PTR \
                            (  (reg8 *) killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(killSwitch_TX_ENABLED)
    #define killSwitch_TXDATA_REG          (* (reg8 *) killSwitch_BUART_sTX_TxShifter_u0__F0_REG)
    #define killSwitch_TXDATA_PTR          (  (reg8 *) killSwitch_BUART_sTX_TxShifter_u0__F0_REG)
    #define killSwitch_TXDATA_AUX_CTL_REG  (* (reg8 *) killSwitch_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define killSwitch_TXDATA_AUX_CTL_PTR  (  (reg8 *) killSwitch_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define killSwitch_TXSTATUS_REG        (* (reg8 *) killSwitch_BUART_sTX_TxSts__STATUS_REG)
    #define killSwitch_TXSTATUS_PTR        (  (reg8 *) killSwitch_BUART_sTX_TxSts__STATUS_REG)
    #define killSwitch_TXSTATUS_MASK_REG   (* (reg8 *) killSwitch_BUART_sTX_TxSts__MASK_REG)
    #define killSwitch_TXSTATUS_MASK_PTR   (  (reg8 *) killSwitch_BUART_sTX_TxSts__MASK_REG)
    #define killSwitch_TXSTATUS_ACTL_REG   (* (reg8 *) killSwitch_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define killSwitch_TXSTATUS_ACTL_PTR   (  (reg8 *) killSwitch_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(killSwitch_TXCLKGEN_DP)
        #define killSwitch_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define killSwitch_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define killSwitch_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define killSwitch_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define killSwitch_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define killSwitch_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define killSwitch_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define killSwitch_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define killSwitch_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define killSwitch_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) killSwitch_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* killSwitch_TXCLKGEN_DP */

#endif /* End killSwitch_TX_ENABLED */

#if(killSwitch_HD_ENABLED)

    #define killSwitch_TXDATA_REG             (* (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__F1_REG )
    #define killSwitch_TXDATA_PTR             (  (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__F1_REG )
    #define killSwitch_TXDATA_AUX_CTL_REG     (* (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define killSwitch_TXDATA_AUX_CTL_PTR     (  (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define killSwitch_TXSTATUS_REG           (* (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_REG )
    #define killSwitch_TXSTATUS_PTR           (  (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_REG )
    #define killSwitch_TXSTATUS_MASK_REG      (* (reg8 *) killSwitch_BUART_sRX_RxSts__MASK_REG )
    #define killSwitch_TXSTATUS_MASK_PTR      (  (reg8 *) killSwitch_BUART_sRX_RxSts__MASK_REG )
    #define killSwitch_TXSTATUS_ACTL_REG      (* (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define killSwitch_TXSTATUS_ACTL_PTR      (  (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End killSwitch_HD_ENABLED */

#if( (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) )
    #define killSwitch_RXDATA_REG             (* (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__F0_REG )
    #define killSwitch_RXDATA_PTR             (  (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__F0_REG )
    #define killSwitch_RXADDRESS1_REG         (* (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__D0_REG )
    #define killSwitch_RXADDRESS1_PTR         (  (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__D0_REG )
    #define killSwitch_RXADDRESS2_REG         (* (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__D1_REG )
    #define killSwitch_RXADDRESS2_PTR         (  (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__D1_REG )
    #define killSwitch_RXDATA_AUX_CTL_REG     (* (reg8 *) killSwitch_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define killSwitch_RXBITCTR_PERIOD_REG    (* (reg8 *) killSwitch_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define killSwitch_RXBITCTR_PERIOD_PTR    (  (reg8 *) killSwitch_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define killSwitch_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) killSwitch_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define killSwitch_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) killSwitch_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define killSwitch_RXBITCTR_COUNTER_REG   (* (reg8 *) killSwitch_BUART_sRX_RxBitCounter__COUNT_REG )
    #define killSwitch_RXBITCTR_COUNTER_PTR   (  (reg8 *) killSwitch_BUART_sRX_RxBitCounter__COUNT_REG )

    #define killSwitch_RXSTATUS_REG           (* (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_REG )
    #define killSwitch_RXSTATUS_PTR           (  (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_REG )
    #define killSwitch_RXSTATUS_MASK_REG      (* (reg8 *) killSwitch_BUART_sRX_RxSts__MASK_REG )
    #define killSwitch_RXSTATUS_MASK_PTR      (  (reg8 *) killSwitch_BUART_sRX_RxSts__MASK_REG )
    #define killSwitch_RXSTATUS_ACTL_REG      (* (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define killSwitch_RXSTATUS_ACTL_PTR      (  (reg8 *) killSwitch_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) */

#if(killSwitch_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define killSwitch_INTCLOCK_CLKEN_REG     (* (reg8 *) killSwitch_IntClock__PM_ACT_CFG)
    #define killSwitch_INTCLOCK_CLKEN_PTR     (  (reg8 *) killSwitch_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define killSwitch_INTCLOCK_CLKEN_MASK    killSwitch_IntClock__PM_ACT_MSK
#endif /* End killSwitch_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(killSwitch_TX_ENABLED)
    #define killSwitch_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End killSwitch_TX_ENABLED */

#if(killSwitch_HD_ENABLED)
    #define killSwitch_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End killSwitch_HD_ENABLED */

#if( (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) )
    #define killSwitch_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define killSwitch_WAIT_1_MS      killSwitch_BL_CHK_DELAY_MS   

#define killSwitch_TXBUFFERSIZE   killSwitch_TX_BUFFER_SIZE
#define killSwitch_RXBUFFERSIZE   killSwitch_RX_BUFFER_SIZE

#if (killSwitch_RXHW_ADDRESS_ENABLED)
    #define killSwitch_RXADDRESSMODE  killSwitch_RX_ADDRESS_MODE
    #define killSwitch_RXHWADDRESS1   killSwitch_RX_HW_ADDRESS1
    #define killSwitch_RXHWADDRESS2   killSwitch_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define killSwitch_RXAddressMode  killSwitch_RXADDRESSMODE
#endif /* (killSwitch_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define killSwitch_initvar                    killSwitch_initVar

#define killSwitch_RX_Enabled                 killSwitch_RX_ENABLED
#define killSwitch_TX_Enabled                 killSwitch_TX_ENABLED
#define killSwitch_HD_Enabled                 killSwitch_HD_ENABLED
#define killSwitch_RX_IntInterruptEnabled     killSwitch_RX_INTERRUPT_ENABLED
#define killSwitch_TX_IntInterruptEnabled     killSwitch_TX_INTERRUPT_ENABLED
#define killSwitch_InternalClockUsed          killSwitch_INTERNAL_CLOCK_USED
#define killSwitch_RXHW_Address_Enabled       killSwitch_RXHW_ADDRESS_ENABLED
#define killSwitch_OverSampleCount            killSwitch_OVER_SAMPLE_COUNT
#define killSwitch_ParityType                 killSwitch_PARITY_TYPE

#if( killSwitch_TX_ENABLED && (killSwitch_TXBUFFERSIZE > killSwitch_FIFO_LENGTH))
    #define killSwitch_TXBUFFER               killSwitch_txBuffer
    #define killSwitch_TXBUFFERREAD           killSwitch_txBufferRead
    #define killSwitch_TXBUFFERWRITE          killSwitch_txBufferWrite
#endif /* End killSwitch_TX_ENABLED */
#if( ( killSwitch_RX_ENABLED || killSwitch_HD_ENABLED ) && \
     (killSwitch_RXBUFFERSIZE > killSwitch_FIFO_LENGTH) )
    #define killSwitch_RXBUFFER               killSwitch_rxBuffer
    #define killSwitch_RXBUFFERREAD           killSwitch_rxBufferRead
    #define killSwitch_RXBUFFERWRITE          killSwitch_rxBufferWrite
    #define killSwitch_RXBUFFERLOOPDETECT     killSwitch_rxBufferLoopDetect
    #define killSwitch_RXBUFFER_OVERFLOW      killSwitch_rxBufferOverflow
#endif /* End killSwitch_RX_ENABLED */

#ifdef killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define killSwitch_CONTROL                killSwitch_CONTROL_REG
#endif /* End killSwitch_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(killSwitch_TX_ENABLED)
    #define killSwitch_TXDATA                 killSwitch_TXDATA_REG
    #define killSwitch_TXSTATUS               killSwitch_TXSTATUS_REG
    #define killSwitch_TXSTATUS_MASK          killSwitch_TXSTATUS_MASK_REG
    #define killSwitch_TXSTATUS_ACTL          killSwitch_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(killSwitch_TXCLKGEN_DP)
        #define killSwitch_TXBITCLKGEN_CTR        killSwitch_TXBITCLKGEN_CTR_REG
        #define killSwitch_TXBITCLKTX_COMPLETE    killSwitch_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define killSwitch_TXBITCTR_PERIOD        killSwitch_TXBITCTR_PERIOD_REG
        #define killSwitch_TXBITCTR_CONTROL       killSwitch_TXBITCTR_CONTROL_REG
        #define killSwitch_TXBITCTR_COUNTER       killSwitch_TXBITCTR_COUNTER_REG
    #endif /* killSwitch_TXCLKGEN_DP */
#endif /* End killSwitch_TX_ENABLED */

#if(killSwitch_HD_ENABLED)
    #define killSwitch_TXDATA                 killSwitch_TXDATA_REG
    #define killSwitch_TXSTATUS               killSwitch_TXSTATUS_REG
    #define killSwitch_TXSTATUS_MASK          killSwitch_TXSTATUS_MASK_REG
    #define killSwitch_TXSTATUS_ACTL          killSwitch_TXSTATUS_ACTL_REG
#endif /* End killSwitch_HD_ENABLED */

#if( (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) )
    #define killSwitch_RXDATA                 killSwitch_RXDATA_REG
    #define killSwitch_RXADDRESS1             killSwitch_RXADDRESS1_REG
    #define killSwitch_RXADDRESS2             killSwitch_RXADDRESS2_REG
    #define killSwitch_RXBITCTR_PERIOD        killSwitch_RXBITCTR_PERIOD_REG
    #define killSwitch_RXBITCTR_CONTROL       killSwitch_RXBITCTR_CONTROL_REG
    #define killSwitch_RXBITCTR_COUNTER       killSwitch_RXBITCTR_COUNTER_REG
    #define killSwitch_RXSTATUS               killSwitch_RXSTATUS_REG
    #define killSwitch_RXSTATUS_MASK          killSwitch_RXSTATUS_MASK_REG
    #define killSwitch_RXSTATUS_ACTL          killSwitch_RXSTATUS_ACTL_REG
#endif /* End  (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) */

#if(killSwitch_INTERNAL_CLOCK_USED)
    #define killSwitch_INTCLOCK_CLKEN         killSwitch_INTCLOCK_CLKEN_REG
#endif /* End killSwitch_INTERNAL_CLOCK_USED */

#define killSwitch_WAIT_FOR_COMLETE_REINIT    killSwitch_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_killSwitch_H */


/* [] END OF FILE */
