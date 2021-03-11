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
#include "OS.h"
#include "LEDs\LEDs.h"

static uint32_t PITClock; /*!<moduleClk from main*/
static void (*UserFunction)(void*); /*!<user callback function from main*/
static void *UserArguments; /*!<user call back argument*/
OS_ECB *PIT_Semaphore; /*!< Signals PIT interrupt event */


bool PIT3_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
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

//  //start PIT 59523750 (16S/Hz) or 312500 (64S/Hz)
//  PIT3_Set(25000000, true);

  //create PIT semaphore for signaling event
  PIT_Semaphore = OS_SemaphoreCreate(0);


  /*  USING PIT3 BECAUSE PIT0 DIDNT WORK  */
  //set nested vectored interrupt controllers for PIT
  //see p.75 in K64 manual
  __NVIC_ClearPendingIRQ(PIT3_IRQn); //first clear any interrupts for PIT channel 3
  __NVIC_EnableIRQ(PIT3_IRQn); //then enable nested vectored interrupt control for PIT channel 3

  return true;
}


void PIT3_Set(const uint32_t period, const bool restart)
{
  //LDVAL trigger = (period / clock period) - 1
  //LDVAL trigger = (period * clock frequency) - 1
  uint32_t timer = (((uint64_t)period * (uint64_t)PITClock) / 1e9) - 1;

  if (restart)
    PIT3_Enable(false);

  //set timer value to 0x1C9C37F
  PIT->CHANNEL[3].LDVAL = timer;

  PIT3_Enable(true);
}


void PIT3_Enable(const bool enable)
{
  if (enable)
    PIT->CHANNEL[3].TCTRL |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK); //enable timer and timer interrupt
  else
    PIT->CHANNEL[3].TCTRL &= ~(PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK); //disable timer and timer interrupt
}


/*! @brief PIT interrupt request handler clears interrupt and calls user function
 *
 */
void PIT3_IRQHandler(void)
{
  //write 1 to clear interrupt trigger
  PIT->CHANNEL[3].TFLG |= PIT_TFLG_TIF_MASK;
  OS_SemaphoreSignal(PIT_Semaphore); //signal pit semaphore
}

/*! @brief Manages interrupts made by the PIT module
 *
 *  @param *pdata points to thread address
 *  @note requires successful init of semaphores
 */
void PITThread(void *pData)
{
  for (;;)
  {
    OS_SemaphoreWait(PIT_Semaphore, 0); //wait for pit semaphore

    if (UserFunction)
      (*UserFunction)(UserArguments);
  }
}

/*!
** @}
*/
