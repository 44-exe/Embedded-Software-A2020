/*! @file InverseTime.c
 *
 *  @brief Calculates and operates the inverse time for ADC.
 *
 *  This contains the functions for operating the inverse time mode.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-06-22
 */

/*!
**  @addtogroup InverseTime_module InverseTime module documentation
**  @{
*/

#include "Types\types.h"
#include "InverseTime.h"
#include "FPC\FPC.h"
#include "FTM\FTM.h"
#include "ADC\ADC.h"


/*! @brief Calculates and sets Raise inverse time.
 *
 *  @param devVolt deviated RMS voltage.
 *  @return double - nano seconds converted from clock tick.
 */
static double ClkToNanoFTM(uint32_t clk)
{
  return clk * FTMCLK;
}

/*! @brief Calculates and sets Raise inverse time.
 *
 *  @param devVolt deviated RMS voltage.
 *  @return uint32_t - clock tick converted from nano seconds.
 */
static uint32_t NanoToClkFTM(double nano)
{
  return nano * FTMCLK;
}

/*! @brief Calculates and sets Raise inverse time.
 *
 *  @param clk - number of tick passed in.
 *  @return double - nano seconds converted from clock tick.
 */
static double ClkToNanoOS(uint32_t clk)
{
  return clk * OSTIME;
}

/*! @brief Calculates inverse time using deviated voltage
 *
 *  @param devVolt - deviated RMS voltage.
 *  @return double - The calcualted inverse time.
 */
static double CalcInverseTime(double devVolt)
{
  return INVERSECONSTANT / devVolt; //  t = (0.5/Vdev) * 5  which is equivalent to 2.5/Vdev
}

/*! @brief Calculates the percent of time passed in the FTM
 *
 *  @param osclk - Number of clock ticks passed in the OS Time.
 *  @return double - passed time as a ratio.
 */
static double CalcTimePercent(uint32_t osclk)
{
  double time = ClkToNanoOS((osclk)); //convert the difference in clk to time

  if (InverseTime.FTMLower.delayNanoseconds != 0)
      return time/ClkToNanoFTM(InverseTime.FTMLower.delayNanoseconds); // percentage = time passed / total time

  if (InverseTime.FTMRaise.delayNanoseconds != 0)
    return time/ClkToNanoFTM(InverseTime.FTMRaise.delayNanoseconds); // percentage = time passed / total time
  else
    return 0;
}


void InverseTimeRaise_Set(double devVolt)
{
  static double prevVolt;
  static double timeRatio = 1;

  if (timeRatio != 0 && InverseTime.EventMode == true)
  {
    OS_TimeSet(0);
    timeRatio = 0;
    prevVolt = devVolt;
    InverseTime.EventMode = false;
  }
  else if (prevVolt != devVolt) //if there is a change in Vdev
  {
    timeRatio = CalcTimePercent(OS_TimeGet());
  }

  //set the inverse time to FTM
  InverseTime.FTMRaise.delayNanoseconds = NanoToClkFTM(CalcInverseTime(devVolt) * (1-timeRatio) +
      CalcInverseTime(prevVolt) * (timeRatio));
  //apply the new inverse time
  FTM_StartTimer(&InverseTime.FTMRaise); //start the time
}


void InverseTimeLower_Set(double devVolt)
{
  static double prevVolt;
  static double timeRatio = 1;

  if (timeRatio != 0 && InverseTime.EventMode == true)
  {
    OS_TimeSet(0);
    timeRatio = 0;
    prevVolt = devVolt;
    InverseTime.EventMode = false;
  }
  else if (prevVolt != devVolt) //if there is a change in Vdev
  {
    timeRatio = CalcTimePercent(OS_TimeGet());
  }

  //set the inverse time to FTM
  InverseTime.FTMLower.delayNanoseconds = NanoToClkFTM(CalcInverseTime(devVolt) * (1-timeRatio) +
      CalcInverseTime(prevVolt) * (timeRatio));
  //apply the new inverse time
  FTM_StartTimer(&InverseTime.FTMLower); //start the time
}

/*!
** @}
*/

