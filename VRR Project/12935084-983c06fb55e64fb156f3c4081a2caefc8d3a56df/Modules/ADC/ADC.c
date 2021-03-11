/*! @file ADC.c
 *
 *  @brief Converts analog signals to digital signals
 *
 *  This contains the functions for operating the analog-to-digital converter (ADC).
 *
 *  @author Jeong Bin Lee
 *  @date 2020-06-15
 */

/*!
**  @addtogroup ADC_module ADC module documentation
**  @{
*/

#include "Types\types.h"
#include "ADC.h"
#include "device\MK64F12.h"
#include "drivers\fsl_clock.h"
#include "Critical\critical.h"
#include <math.h>
#include "OS.h"
#include "LEDs\LEDs.h"
#include "FTM\FTM.h"
#include "FPC\FPC.h"
#include "InverseTime\InverseTime.h"

#define SAMPLES 100 /*!< number of samples in the buffer */

OS_ECB *ADC_RaiseSemaphore; //used to call Raise thread
OS_ECB *ADC_LowerSemaphore; //used to call Lower thread

static uint32_t RedLEDCount = 0; //used to keep track of Red LED

bool ADC_Init(ADCSetup_t setupADC)
{
  bool success;
//0. prior to calibration, configure clock source and frequency
  //low power config, voltage reference, sample time
  //and high speed config according to the clock source, application and needs
  CLOCK_EnableClock(kCLOCK_Adc0);

//1. calibrate the function
  ADC0->SC3 = ADC_SC3_CAL_MASK;

  //wait for the calibration to complete
  int i;
  for (i=0; i<100000; i++);

  //if calibration failed is set, return false
  if ((ADC0->SC3 & ADC_SC3_CALF_MASK) == ADC_SC3_CALF_MASK)
    return false;

  //to complete calibration, user must generate gain calibration values
  // 1. initialise or clear 16-bit variable in RAM
  uint16_t calVar;
  // 2. add the positive side calibration results CLP0,1,2,3,4 and CLPS to the variable
  calVar = (ADC0->CLP0) + (ADC0->CLP1) + (ADC0->CLP2);
  calVar += ((ADC0->CLP3) + (ADC0->CLP4) + (ADC0->CLPS));
  // 3. divide the variable by 2
  calVar /= 2;
  // 4. set the MSB of the variable
  calVar |= 0x8000;
  // 5. the previous two steps can be carried out using step 5 at page 887 in the manual
  //........
  // 6. store the value in the positive gain
  ADC0->PG = calVar;
  // 7. repeat the steps for minus gain
  //........
  calVar = (ADC0->CLM0) + (ADC0->CLP1) + (ADC0->CLM2);
  calVar += ((ADC0->CLM3) + (ADC0->CLM4) + (ADC0->CLMS));
  calVar /= 2;
  calVar |= 0x8000;
  //set negative gain error
  ADC0->MG = calVar;

  ADC0->CFG1 = ADC_CFG1_MODE(0b11); //set mode to 16-bit resolution

  ADC0->SC3 = ADC_SC3_ADCO_MASK; //Enable continuous conversion

  ADC0->SC1[0] = ADC_SC1_ADCH(0b10111); //select SE23 as input port

  success = FTM_Set(setupADC.FTMSetLower);
  success &= FTM_Set(setupADC.FTMSetRaise);
  success &= FTM_Set(setupADC.FTMOffLED);

  ADC_RaiseSemaphore = OS_SemaphoreCreate(0);
  ADC_LowerSemaphore = OS_SemaphoreCreate(0);

  return success;
}

uint16_t ADC_Read16Bit(void)
{
  while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)); //wait until conversion is complete
  return ADC0->R[0];
}

uint16_t ADC_TrueRMS(uint16_t sample)
{
  static double rms[SAMPLES]; //act as sample buffer
  double Vrms = 0;
  uint8_t i;
  static uint8_t countSample = 0; //count the number of samples in the buffer

  //shift the sample inside the buffer
  if (countSample >= SAMPLES)
  {
    for (i=0; i<SAMPLES-1; i++)
    {
      //shift the data in the buffer
      rms[i] = rms[i+1];
    }
    countSample = SAMPLES-1;
  }

  rms[countSample] = DigitalToVoltage(sample); //take a sample
  countSample++;// increment sample pointer

  Vrms = FPC_Vrms(rms, countSample);

  //compare the result to the limit range (2v to 3v)
  ADC_CompareVoltage(Vrms);

  //check if voltage is between 0v to 5v
  return CheckVoltageRange(Vrms);
}


void ADC_CompareVoltage(double voltage)
{
  double voltageDiff;

  if (voltage < 2)
  {
    ADC_Setup.CompareVoltage = true;
    LEDs_On(LED_RED);
    RedLEDCount++;
    voltageDiff = 2 - voltage;
    //start FTM timer to count down and set "Raise" or "Lower" events
    ADC_SetRaiseFTM(voltageDiff);
  }
  else if (3 < voltage)
  {
    ADC_Setup.CompareVoltage = true;
    LEDs_On(LED_RED);
    RedLEDCount++;
    voltageDiff = voltage - 3;
    //start FTM timer to count down and set "Raise" or "Lower" events
    ADC_SetLowerFTM(voltageDiff);
  }
  else
  {
    ADC_Setup.CompareVoltage = false;
    LEDs_Off(LED_RED);
    RedLEDCount = 0; //reset count
    InverseTime.EventMode = true;
    voltageDiff = 0;
  }
}

void ADC_SetLowerFTM(double voltage)
{
  //check time mode (inverse time or normal)
  if (ADC_Setup.CompareVoltage == true)
  {
    if (ADC_Setup.InverseTime == true)
    {
      InverseTimeLower_Set(voltage);
    }
    else if (RedLEDCount == 1)
    {
      //set delay to 5sec
      ADC_Setup.FTMSetLower->delayNanoseconds = 12207 * 5;
      FTM_StartTimer(&InverseTime.FTMLower);
      ADC_Setup.Event = false;
    }
  }
}

void ADC_SetRaiseFTM(double voltage)
{
  //check time mode (inverse time or normal)
  if (ADC_Setup.CompareVoltage == true)
  {
    if (ADC_Setup.InverseTime == true)
    {
      InverseTimeRaise_Set(voltage);
    }
    else if (RedLEDCount == 1)
    {
      //set delay to 5sec
      ADC_Setup.FTMSetRaise->delayNanoseconds = 12207 * 5;
      FTM_StartTimer(&InverseTime.FTMRaise);
      ADC_Setup.Event = false;
    }
  }
}


double DigitalToVoltage(uint16_t sampleVoltage)
{
  return FPC_Division((double)(sampleVoltage - 32768), 3276.8);
}

uint16_t VoltageToDigital(double voltage)
{
  return FPC_Addition(FPC_Multiplication(voltage, 3276.8), 32768);
}

uint16_t CheckVoltageRange(double voltage)
{
  //voltage range = 0v to 5v
  uint16_t digital = VoltageToDigital(voltage);
  if (voltage > 5)
    return VoltageToDigital(5);
  else if (voltage < 0)
    return VoltageToDigital(0);
  else
    return digital;
}


void ADCRaiseThread(void *pData)
{
  for (;;)
  {
    OS_SemaphoreWait(ADC_RaiseSemaphore, 0);

    //Raise event triggered, GREEN LED is on while until voltage is back in the limit
    if (ADC_Setup.CompareVoltage == true)  //only set LEDs if voltage is out of range
    {
      LEDs_On(LED_GREEN);
      FTM_StartTimer(ADC_Setup.FTMOffLED);
    }
    ADC_Setup.Event = true; //event finished
  }
}

void ADCLowerThread(void *pData)
{
  for (;;)
  {
    OS_SemaphoreWait(ADC_LowerSemaphore, 0);

    //Lower event triggered, BLUE LED is on while until voltage is back in the limit
    if (ADC_Setup.CompareVoltage == true)  //only set LEDs if voltage is out of range
    {
      LEDs_On(LED_BLUE);
      FTM_StartTimer(ADC_Setup.FTMOffLED);
    }
    ADC_Setup.Event = true; //event finished
  }
}


/*!
** @}
*/
