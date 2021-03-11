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

#ifndef ADC_ADC_H
#define ADC_ADC_H

#include "Types\types.h"
#include "FTM\FTM.h"
#include "OS.h"


//Structure variables to set up ADC
//Additionally contains global boolean variable
typedef struct
{
  bool InverseTime;
  bool CompareVoltage;
  bool Event;
  TFTMChannel *FTMSetLower;  /*!< FTM channel 0 */
  TFTMChannel *FTMSetRaise;  /*!< FTM channel 1 */
  TFTMChannel *FTMOffLED;
} ADCSetup_t;

extern ADCSetup_t ADC_Setup;

extern OS_ECB *ADC_RaiseSemaphore;
extern OS_ECB *ADC_LowerSemaphore;

extern volatile uint16union_t *NvRaise; /*!<to store Raise count in Flash*/
extern volatile uint16union_t *NvLower; /*!<to store Lower count in Flash*/

/*! @brief Sets up the ADC before first use.
 *
 *  @param setupADC - Structure variable to set up ADC module.
 *  @return bool - TRUE if the ADC was successfully initialized.
 */
bool ADC_Init(ADCSetup_t setupADC);


/*! @brief Reads the data converted by ADC
 *
 *  @return uint16_t - the value of the register after converting values
 */
uint16_t ADC_Read16Bit(void);


/*! @brief Takes the raw digital sample of the voltage and convert it to RMS voltage.
 *
 *  @param sample - The sample voltage converted by ADC
 *  @return uint16_t - Calculated value of the voltage in RMS
 */
uint16_t ADC_TrueRMS(uint16_t sample);

/*! @brief Compares voltage and starts count down on the correct FTM channel
 *
 *  @param voltage - Vrms used to compare limit voltage
 */
void ADC_CompareVoltage(double voltage);

/*! @brief Sets FTM to trigger Lower Event
 *
 *  @param voltage - deviated voltage from the limit
 */
void ADC_SetLowerFTM(double voltage);

/*! @brief Sets FTM to trigger Raise Event
 *
 *  @param voltage - deviated voltage from the limit
 */
void ADC_SetRaiseFTM(double voltage);

/*! @brief Converts the digital value to analog voltage
 *
 *  @param sampleVoltage - sampled variable from ADCx->R[n] register
 *  @return double - return the analog voltage converted from digital voltage
 */
double DigitalToVoltage(uint16_t sampleVoltage);

/*! @brief Converts the analog voltage to digital value
 *
 *  @param sampleVoltage - analog voltage
 *  @return uint16_t - return the digital voltage converted from analog voltage
 */
uint16_t VoltageToDigital(double voltage);

/*! @brief Checks and return voltage to be in the range of VRR requirement
 *
 *  @param voltage - analog voltage for comparing
 *  @return uint16_t - return the digital voltage within VRR limit
 */
uint16_t CheckVoltageRange(double voltage);

/*! @brief Thread to start FTM timer for Raise Event
 *
 *  @param pData - pointer to data used in the thread (NULL)
 */
void ADCRaiseThread(void *pData);

/*! @brief Thread to start FTM timer for Lower Event
 *
 *  @param pData - pointer to data used in the thread (NULL)
 */
void ADCLowerThread(void *pData);


#endif /* ADC_ADC_H */


/*!
** @}
*/
