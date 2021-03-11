/*! @file PIT.c
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT).
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-04
 */

/*!
**  @addtogroup PIT_module PIT module documentation
**  @{
*/

// new types
#include "Types\types.h"
#include "device\MK64F12.h"
#include "PIT.h"
#include "drivers\fsl_clock.h"

static uint32_t PITClock; /*!<moduleClk from main*/
static void (*UserFunction)(void*); /*!<user callback function from main*/
static void *UserArguments; /*!<user call back argument*/


bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  if (moduleClk == 0)
    return false;

  //enable PIT clock
  CLOCK_EnableClock(kCLOCK_Pit0);

  //declare as global variables
  PITClock = moduleClk;
  UserFunction = userFunction;
  UserArguments = userArguments;

  //PIT module is activated by writing 0 to MDIS
  PIT->MCR &= ~PIT_MCR_MDIS_MASK;

  //FRZ stops PIT when debugging
  PIT->MCR |= PIT_MCR_FRZ_MASK;


  //restart PIT
  PIT_Set(500000000, true);


  //set nested vectored interrupt controllers for PIT
  //see p.75 in K64 manual
  //PIT channel 0, Vector = 64, IRQ = 48, non-IPR = 1, IPR = 12
  __NVIC_ClearPendingIRQ(PIT0_IRQn); //first clear any interrupts for PIT channel 0
  __NVIC_EnableIRQ(PIT0_IRQn); //then enable nested vectored interrupt control for PIT channel 0

  return true;
}


void PIT_Set(const uint32_t period, const bool restart)
{
  //LDVAL trigger = (period / clock period) - 1
  //LDVAL trigger = (period * clock frequency) - 1
  uint32_t timer = (((uint64_t)period * (uint64_t)PITClock) / 1e9) - 1;

  if (restart)
    PIT_Enable(false);

  //set timer value to 0x1C9C37F
  PIT->CHANNEL[0].LDVAL = timer;

  PIT_Enable(true);
}


void PIT_Enable(const bool enable)
{
  if (enable)
    PIT->CHANNEL[0].TCTRL |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK); //enable timer and timer interrupt
  else
    PIT->CHANNEL[0].TCTRL &= ~(PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK); //disable timer and timer interrupt
}


/*! @brief PIT interrupt request handler clears interrupt and calls user function
 *
 */
void PIT0_IRQHandler(void)
{
  //write 1 to clear interrupt trigger
  PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;

  if (UserFunction)
    (*UserFunction)(UserArguments);
}

/*!
** @}
*/
