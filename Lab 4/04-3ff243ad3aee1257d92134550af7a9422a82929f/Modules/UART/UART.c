/*! @file UART.c
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-05
 */

/*!
**  @addtogroup UART_module UART module documentation
**  @{
*/


#include "UART.h"
#include "FIFO\FIFO.h"
#include "Critical\critical.h"
#include "MK64F12.h"
#include "drivers\fsl_port.h"
#include "drivers\fsl_clock.h"
#include "drivers\fsl_common.h"

//create FIFO as global variables
static TFIFO RxFIFO; /*!< receive FIFO struct variable*/
static TFIFO TxFIFO; /*!< transmit FIFO struct variable*/

const port_pin_config_t UART0_CONFIG =
{
    .pullSelect          = kPORT_PullDisable,
    .slewRate            = kPORT_SlowSlewRate,
    .passiveFilterEnable = kPORT_PassiveFilterDisable,
    .openDrainEnable     = kPORT_OpenDrainDisable,
    .driveStrength       = kPORT_LowDriveStrength,
    .mux                 = kPORT_MuxAlt3,
    .lockRegister        = kPORT_UnlockRegister
};


bool UART_Init(const uint32_t moduleClk, const uint32_t baudRate)
{
  uint16union_t sbr;  //store slave baudrate
  uint8_t brfa;       //store baudrate fine adjust


  if (baudRate == 0)
    return false;
//  //enable UART0 from System Clock Gating Control Register 4
//  SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
//  //enable Port B from System Clock Gating Control Register 5
//  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

  CLOCK_EnableClock(kCLOCK_Uart0);
  CLOCK_EnableClock(kCLOCK_PortB);

  PORT_SetPinConfig(PORTB, 16, &UART0_CONFIG);
  PORT_SetPinConfig(PORTB, 17, &UART0_CONFIG);

  //set baud rate and baud rate divider
  //UART baud rate target = 38400
  //SystemCoreClock = UART0 Module Clock (120MHz)
  //SBR[12:0] =  195
  //BRFD = 0.3125
  //BRFA = 1 0 0 1 1
  //additionally set BDH & BDL
  // UART BR = UART MOD CLK / (16 * (SBR[12:0] + BRFD))
  //38400 = SystemCoreClock / (16 * (195 + 10));
  //this is how to set BDH
  //UART0->BDH |= UART_BDH_SBR(0x00);
  sbr.l = moduleClk / (baudRate * 16);
  brfa  = (((double)moduleClk / (double)(baudRate * 16)) - sbr.l) * 32;
  //not enabled until RE and TE are set in C2 register
  UART0->BDH = (sbr.s.Hi & 0b00011111);
  UART0->BDL = sbr.s.Lo;
  UART0->C4 |= brfa;


  UART0->C2 |= UART_C2_RIE_MASK;

  //UART0 Receiver on & Transmitter on
  UART0->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);

  //UART0->CFIFO |= UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK;

  //set nexted vectored interrupt controllers for UART0
  //see p.75 in K64 manual
  //UART channel 0, Vector = 47, IRQ = 31, non-IPR = 0, IPR = 7
  __NVIC_ClearPendingIRQ(UART0_RX_TX_IRQn); //first clear any interrupts for UART channel 0
  __NVIC_EnableIRQ(UART0_RX_TX_IRQn); //then enable nested vectored interrupt control for UART channel 0


  //initialise the receive and transmit FIFO variables

  return FIFO_Init(&TxFIFO) & FIFO_Init(&RxFIFO);
}


bool UART_InChar(uint8_t* const dataPtr)
{
  //try to read data from rxfifo
  return FIFO_Get(&RxFIFO, dataPtr);
}

//
bool UART_OutChar(const uint8_t data)
{
  //read data has been received in UART_D, so put it in FIFO_Put(txFIFO, data)
  EnterCritical();
  bool success = FIFO_Put(&TxFIFO, data);
  if (success)
    UART0->C2 |= UART_C2_TIE_MASK; //byte ready for transfer
  ExitCritical();
  return success;
}



void UART0_RX_TX_IRQHandler(void)
{
  // Receive a character

  if (UART0->C2 & UART_C2_RIE_MASK)
  {
    if((UART0->S1 & UART_S1_RDRF_MASK)) // Clear RDRF flag by reading the status register then Data register
      FIFO_Put(&RxFIFO, UART0->D);   // Do something with the received byte
  }

  // Transmit a character
  if (UART0->C2 & UART_C2_TIE_MASK)
  {
    // Clear TDRE flag by reading the status register
    if (UART0->S1 & UART_S1_TDRE_MASK)
    {
      if (!FIFO_Get(&TxFIFO, (uint8_t*)&UART0->D))// Get a new byte and transmit it
        UART0->C2 &= ~UART_C2_TIE_MASK;
    }
  }
}



//void UART_Poll(void)
//{
//  //check S1 register for TDRE flag and RDRF flag
//  //if RDRF is set RxFIFO_Put(RxFIFO) & clear flag
//  //if TDRE is set TxFIFO_Get(TxFIFO) & clear flag
//  if (UART0->S1 & UART_S1_RDRF_MASK)
//    FIFO_Put(&RxFIFO, UART0->D);
//  if (UART0->S1 & UART_S1_TDRE_MASK)
//    FIFO_Get(&TxFIFO, (uint8_t*)&UART0->D);
//}

/*!
** @}
*/
