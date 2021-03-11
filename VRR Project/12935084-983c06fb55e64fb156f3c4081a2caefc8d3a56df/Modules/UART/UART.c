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
#include "OS.h"

//create FIFO as global variables
static TFIFO RxFIFO; /*!< receive FIFO struct variable*/
static TFIFO TxFIFO; /*!< transmit FIFO struct variable*/

static OS_ECB *UartRxSemaphore;
static OS_ECB *UartTxSemaphore;

OS_ECB *PacketPutSemaphore;

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


bool UART_Init(const uint32_t moduleClk, const UARTSetup_t* const uartSetup)
{
  uint16union_t sbr;  //store slave baudrate
  uint8_t brfa;       //store baudrate fine adjust


  if (uartSetup->baudRate == 0)
    return false;
//  //enable UART0 from System Clock Gating Control Register 4
//  SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
//  //enable Port B from System Clock Gating Control Register 5
//  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

  CLOCK_EnableClock(kCLOCK_Uart0);
  CLOCK_EnableClock(kCLOCK_PortB);

  PORT_SetPinConfig(PORTB, 16, &UART0_CONFIG);
  PORT_SetPinConfig(PORTB, 17, &UART0_CONFIG);

  UART0->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);


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
  sbr.l = moduleClk / (uartSetup->baudRate * 16);
  brfa  = (((double)moduleClk / (double)(uartSetup->baudRate * 16)) - sbr.l) * 32;
  //not enabled until RE and TE are set in C2 register
  UART0->BDH = (sbr.s.Hi & 0b00011111);
  UART0->BDL = sbr.s.Lo;
  UART0->C4 |= brfa;

  UART0->C2 |= UART_C2_RIE_MASK | UART_C2_TIE_MASK;

  //UART0 Receiver on & Transmitter on
  UART0->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);

  //UART0->CFIFO |= UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK;


  //initialise the receive and transmit FIFO variables

  UartRxSemaphore = OS_SemaphoreCreate(0);
  UartTxSemaphore = OS_SemaphoreCreate(0);

  OS_THREAD_STACK(STACK_UART0_TX, THREAD_STACK_SIZE);
  OS_THREAD_STACK(STACK_UART0_RX, THREAD_STACK_SIZE);

  if (OS_ThreadCreate(UART0TxThread, NULL, &STACK_UART0_TX[THREAD_STACK_SIZE - 1], uartSetup->txPriority) != OS_NO_ERROR)
    DEBUG_HALT();

  if (OS_ThreadCreate(UART0RxThread, NULL, &STACK_UART0_RX[THREAD_STACK_SIZE - 1], uartSetup->rxPriority) != OS_NO_ERROR)
    DEBUG_HALT();


  //set nexted vectored interrupt controllers for UART0
  //see p.75 in K64 manual
  //UART channel 0, Vector = 47, IRQ = 31, non-IPR = 0, IPR = 7
  __NVIC_ClearPendingIRQ(UART0_RX_TX_IRQn); //first clear any interrupts for UART channel 0
  __NVIC_EnableIRQ(UART0_RX_TX_IRQn); //then enable nested vectored interrupt control for UART channel 0

  return FIFO_Init(&TxFIFO) & FIFO_Init(&RxFIFO);
}


void UART_InChar(uint8_t* const dataPtr)
{
  FIFO_Get(&RxFIFO, dataPtr);
}


void UART_OutChar(const uint8_t data)
{
  FIFO_Put(&TxFIFO, data);
}


void UART0_RX_TX_IRQHandler(void)
{
  //static uint8_t dummyRead;
  OS_ISREnter();

  // Check for overrun
//  if (UART0->S1 & UART_S1_OR_MASK)
//   DEBUG_HALT();

  // Receive a character
  if (UART0->C2 & UART_C2_RIE_MASK)
  {
    if (UART0->S1 & UART_S1_RDRF_MASK) // Clear RDRF flag by reading the status register then Data register
    {
      //dummyRead = UART0->D; //dummy read to clear the RDRF flag
      UART0->C2 &= ~UART_C2_RIE_MASK;  // clear the mask
      (void)OS_SemaphoreSignal(UartRxSemaphore); //signal the waiting thread for Rx
    }
  }

  // Transmit a character
  if (UART0->C2 & UART_C2_TIE_MASK)
  {
    if (UART0->S1 & UART_S1_TDRE_MASK)
    {
      UART0->C2 &= ~UART_C2_TIE_MASK;  // clear the mask
      (void)OS_SemaphoreSignal(UartTxSemaphore); //wait for valid signal for Tx thread
    }
  }
  OS_ISRExit();
}

void UART0RxThread(void *pData)
{
  //process the arrival of data see hint 5.
  for (;;)
  {
    //wait for semaphore signal
    (void)OS_SemaphoreWait(UartRxSemaphore, 0);
    FIFO_Put(&RxFIFO, UART0->D);   // Do something with the received byte
    UART0->C2 |= UART_C2_RIE_MASK;
  }
}

void UART0TxThread(void *pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(UartTxSemaphore, 0); //wait for semaphore signal
    // Clear TDRE flag by reading the status register
    FIFO_Get(&TxFIFO, (uint8_t*)&UART0->D); //get a data to transmit
    UART0->C2 |= UART_C2_TIE_MASK; //enable transmit interrupt
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
