/*! @file RTC.c
 *
 *  @brief Routines for controlling the Real Time Clock (RTC).
 *
 *  This contains the functions for operating the real time clock (RTC).
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-04
 */

/*!
**  @addtogroup RTC_module RTC module documentation
**  @{
*/

// new types
#include "Types\types.h"
#include "RTC.h"
#include "device\MK64F12.h"
#include "drivers\fsl_clock.h"
#include "Critical\critical.h"

static void (*UserFunction)(void*); /*!< user callback function*/
static void* UserArguments; /*!< user callback arguments */


bool RTC_Init(void (*userFunction)(void*), void* userArguments)
{
  int i;
  UserFunction = userFunction;
  UserArguments = userArguments;

  //enable RTC clock gate
  CLOCK_EnableClock(kCLOCK_Rtc0);
  //SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;

  //interrupt occurs every second
  //set capacitor using equation:
  //Cload = (C1 * C2)/(C1 + C2) + Cstray
  //Cload = 6pF where C1 = C2 = 12pF according to schematics
  //assuming Cstray = 0
  RTC->CR |= RTC_CR_SC2P_MASK | RTC_CR_SC4P_MASK;

  //Enable the oscilator
  RTC->CR |= RTC_CR_OSCE_MASK;
  //oscillator startup time = 1000ms = 1sec
  for (i=0; i<1000000; i++); //waste 1sec for crystal to startup

  //enable interrupt for RTC time second interrupt
  RTC->IER |= RTC_IER_TSIE_MASK;
  //disable other interrupts
  RTC->IER &= ~(RTC_IER_TAIE_MASK | RTC_IER_TOIE_MASK | RTC_IER_TIIE_MASK);
  //enable time counter
  RTC->SR |= RTC_SR_TCE_MASK;

  //at the end lock the control register
  RTC->LR &= ~RTC_LR_CRL_MASK;

  //set nexted vectored interrupt controllers for RTC
  //see p.75 in K64 manual
  //RTC, Vector = 63, IRQ = 47, non-IPR = 1, IPR = 11
  __NVIC_ClearPendingIRQ(RTC_Seconds_IRQn); //first clear any interrupts for RTC
  __NVIC_EnableIRQ(RTC_Seconds_IRQn); //then enable nested vectored interrupt control for RTC

  return true;
}


void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  EnterCritical();
  RTC->SR &= ~RTC_SR_TCE_MASK; //disable timer to set timer
  RTC->TSR = hours * 3600 + minutes * 60 + seconds; //set timer register
  RTC->SR |= RTC_SR_TCE_MASK; //enable timer register
  ExitCritical();
}


void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  uint32_t totalSec, checkTotal;

  //see page 1173 in manual
  //check if the time is in sync
  do
  {
    totalSec = RTC->TSR;
    checkTotal = RTC->TSR;
  } while (totalSec != checkTotal);

  //convert total seconds to hours, minutes, seconds
  *hours = totalSec / 3600;
  if (*hours == 24) // reset hours as 24hrs system
    *hours = 0;
  *minutes = (totalSec % 3600) / 60;
  *seconds = (totalSec % 3600) % 60;
}


/*! @brief RTC interrupt request handler calls user function
 *
 */
void RTC_Seconds_IRQHandler(void)
{
  if (UserFunction)
    (*UserFunction)(UserArguments);
}

/*!
** @}
*/

