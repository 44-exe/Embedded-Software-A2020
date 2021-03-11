/*! @file UART.c
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Jeong Bin Lee
 *  @date 2020-03-16
 */

#include "UART.h"
#include "FIFO\FIFO.h"
#include "MK64F12.h"

//create FIFO as global variables
static TFIFO RxFIFO;
static TFIFO TxFIFO;

/*! @brief Sets up the UART interface before first use.
 *
 *  @param moduleClk The module clock rate in Hz.
 *  @param baudRate The desired baud rate in bits/sec.
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t moduleClk, const uint32_t baudRate)
{
  uint16union_t sbr;  //store slave baudrate
  uint8_t brfa;       //store baudrate fine adjust


  if (baudRate == 0)
    return false;
  //enable UART0 from System Clock Gating Control Register 4
  SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
  //enable Port B from System Clock Gating Control Register 5
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;


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

  //enable PORTA PCR16 & PCR17 ALT3 for UART0_RX & UART0_TX (according to p.248 in manual)
  PORTB->PCR[16] |= PORT_PCR_MUX(3);
  PORTB->PCR[17] |= PORT_PCR_MUX(3);

  //UART0 Receiver on & Transmitter on
  UART0->C2 |= UART_C2_RE_MASK | UART_C2_TE_MASK;


  //initialise the receive and transmit FIFO variables

  return FIFO_Init(&TxFIFO) & FIFO_Init(&RxFIFO);
}

/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return bool - TRUE if the receive FIFO returned a character.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_InChar(uint8_t* const dataPtr)
{
  //try to read data from rxfifo
  return FIFO_Get(&RxFIFO, dataPtr);
}

/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return bool - TRUE if the data was placed in the transmit FIFO.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_OutChar(const uint8_t data)
{
  //read data has been received in UART_D, so put it in FIFO_Put(txFIFO, data)
  return FIFO_Put(&TxFIFO, data);
}

/*! @brief Poll the UART status register to try and receive and/or transmit one character.
 *
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
void UART_Poll(void)
{
  //check S1 register for TDRE flag and RDRF flag
  //if RDRF is set RxFIFO_Put(RxFIFO) & clear flag
  //if TDRE is set TxFIFO_Get(TxFIFO) & clear flag
  if (UART0->S1 & UART_S1_RDRF_MASK)
    FIFO_Put(&RxFIFO, UART0->D);
  if (UART0->S1 & UART_S1_TDRE_MASK)
    FIFO_Get(&TxFIFO, (uint8_t*)&UART0->D);
}

