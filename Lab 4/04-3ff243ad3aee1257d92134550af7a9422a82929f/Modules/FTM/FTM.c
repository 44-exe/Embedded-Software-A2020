/*! @file FTM.c
 *
 *  @brief Routines for setting up the FlexTimer module (FTM).
 *
 *  This contains the functions for operating the FlexTimer module (FTM).
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-10
 */

/*!
**  @addtogroup FTM_module FTM module documentation
**  @{
*/

// new types
#include "Types\types.h"
#include "FTM.h"
#include "device\MK64F12.h"
#include "drivers\fsl_clock.h"
#include <math.h>

#define FTM_CHANNEL_SIZE 8

static void (*CallbackFunction[FTM_CHANNEL_SIZE])(void*); /*!< user callback function as an array */
static void *CallbackArguments[FTM_CHANNEL_SIZE]; /*!< user callback arguments as an array */


bool FTM_Init()
{
  //enable FTM clock
  //CLOCK_GetFixedFreqClkFreq
  //uint32_t freq = CLOCK_GetFreq(kCLOCK_McgFixedFreqClk);
  //1562500
  CLOCK_EnableClock(kCLOCK_Ftm0);

  //write to CNTIN (initial value of CNT register)
  //if 15th bit is set two's complement is written (negative numbers)
  //FTM period when using up counting = (MOD – CNTIN + 0x0001) × period of counter clock
  //FTM period when using up-down counting = 2 × (MOD – CNTIN) × period of counter clock
  FTM0->CNTIN = ~FTM_CNTIN_INIT_MASK;
  //write to MOD (modulo value defines the final value of FTM counting)
  FTM0->MOD = FTM_MOD_MOD_MASK;
  //write to CNT (counter value is loaded with CNTIN value)
  FTM0->CNT = ~FTM_CNT_COUNT_MASK;
  //write to CLKS[1:0] (in SC)
  FTM0->SC |= FTM_SC_CLKS(0b10) | FTM_SC_PS(0b101); //set prescale factor of 32

  __NVIC_ClearPendingIRQ(FTM0_IRQn);
  __NVIC_EnableIRQ(FTM0_IRQn);

  return true;
}

bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
//  The Output Compare mode is selected when:
//  • DECAPEN = 0
//  • COMBINE = 0
//  • CPWMS = 0, and
//  • MSnB:MSnA = 0:1

  //check if timerFunction is output channel
  if (aFTMChannel->timerFunction)
  {
    FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_MSB(0);
    FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_MSA(1);
    switch (aFTMChannel->ioType.outputAction)
    {
      case TIMER_OUTPUT_DISCONNECT:
        //revert to GPIO or other peripheral control
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSB(0);
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSA(0);
        break;
      case TIMER_OUTPUT_TOGGLE:
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSB(0);
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSA(1);
        break;
      case TIMER_OUTPUT_LOW:
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSB(1);
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSA(0);
        break;
      case TIMER_OUTPUT_HIGH:
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSB(1);
        FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_ELSA(1);
        break;
    }
  }
  else
    return false; //return false if channel is not an output compare

  CallbackFunction[aFTMChannel->channelNb] = aFTMChannel->callbackFunction;
  CallbackArguments[aFTMChannel->channelNb] = aFTMChannel->callbackArguments;

  return true;
}

bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  //check for output compare
  if (aFTMChannel->timerFunction)
  {
    if (FTM0->CONTROLS[aFTMChannel->channelNb].CnSC & FTM_CnSC_CHF_MASK)
      FTM0->CONTROLS[aFTMChannel->channelNb].CnSC &= ~FTM_CnSC_CHF_MASK;

    //enable channel interrupt
    FTM0->CONTROLS[aFTMChannel->channelNb].CnSC |= FTM_CnSC_CHIE_MASK;

    //the match value for the output modes
    FTM0->CONTROLS[aFTMChannel->channelNb].CnV = (aFTMChannel->delayNanoseconds + FTM0->CNT);
  }
  else
    return false; //channel is not output compare

//  When the FTM counter reaches FTM_CnV, the CHF bit in FTM_CnSC register is set and an interrupt is generated.
//  No matter which channel is used, the interrupt cycle time is FTM_MOD*(tick cycle time).

  return true;
}


/*! @brief FTM interrupt request handler reads CnSC register, clears interrupt and calls user function
 *
 */
void FTM0_IRQHandler(void)
{
  uint8_t channel = log2(FTM0->STATUS);

  if (FTM0->CONTROLS[channel].CnSC & FTM_CnSC_CHF_MASK && FTM0->CONTROLS[channel].CnSC & FTM_CnSC_CHIE_MASK)
  {
    FTM0->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;
    //reading the CnSC register while CHnF is set and then writing a 0 clears interrupt
    FTM0->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHIE_MASK;
    if (CallbackFunction[channel])
      (CallbackFunction[channel])(CallbackArguments[channel]);
  }
}

/*!
** @}
*/

