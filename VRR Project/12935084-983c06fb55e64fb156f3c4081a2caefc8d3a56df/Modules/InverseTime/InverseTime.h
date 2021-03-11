/*! @file InverseTime.h
 *
 *  @brief Calculates and operates the inverse time for ADC.
 *
 *  This contains the functions for operating the inverse time mode.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-06-22
 */

#ifndef INVERSETIME_INVERSETIME_H_
#define INVERSETIME_INVERSETIME_H_

#include "Types\types.h"
#include "FTM\FTM.h"


#define OSTIME 47.68371582 /*!< system core clock in nano seconds */
#define OSCLK 20971520 /*!< system core clock in Hz (before board clk init) */
#define FTMTIME 81920 /*!< FTM clock in nano seconds for 1Hz*/
#define FTMCLK 12207 /*!< FTM clock in Hz  for 1 sec*/
#define INVERSECONSTANT 2.5 /*!<constant used for calculating inverse time.*/

typedef struct
{
  bool InverseTime;
  bool CompareVoltage;
  bool EventMode;
  uint16union_t WaveFreq;
  TFTMChannel FTMRaise; /*!< FTM channel 0 */
  TFTMChannel FTMLower; /*!< FTM channel 1 */
} InverseTime_t;

extern InverseTime_t InverseTime; /*!< Inverse time structure used in ADC and to calculate Inverse Time */

/*! @brief Calculates and sets Raise inverse time.
 *
 *  @param devVolt deviated RMS voltage.
 */
void InverseTimeRaise_Set(double devVolt);

/*! @brief Calculates and sets Lower inverse time.
 *
 *  @param devVolt deviated RMS voltage.
 */
void InverseTimeLower_Set(double devVolt);



#endif /* INVERSETIME_INVERSETIME_H_ */
