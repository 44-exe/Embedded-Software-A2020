/*! @file accel.c
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the FXOS8700CQ accelerometer.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-13
 */

/*!
**  @addtogroup accel_module accel module documentation
**  @{
*/

#include "accel.h"
#include "accel_FXO.h"
#include "I2C\I2C.h"
#include "Median\median.h"
#include "Critical\critical.h"
#include "MK64F12.h"
#include "drivers\fsl_port.h"
#include "drivers\fsl_clock.h"
#include "drivers\fsl_common.h"
#include "drivers\fsl_gpio.h"


AccelMode_t AccelMode; /*!< Mode for the accelerometer*/

AccelData_t AccelData; /*!< Data for accelerometer*/

AccelSetup_t Accel_Setup; /*!<stores Accel structure data*/

const port_pin_config_t PORTC_CONFIG =
{
    .pullSelect          = kPORT_PullDisable,
    .slewRate            = kPORT_SlowSlewRate,
    .passiveFilterEnable = kPORT_PassiveFilterDisable,
    .openDrainEnable     = kPORT_OpenDrainDisable,
    .driveStrength       = kPORT_LowDriveStrength,
    .mux                 = kPORT_MuxAsGpio,
    .lockRegister        = kPORT_UnlockRegister
};

const gpio_pin_config_t PORTCIN =
{
  kGPIO_DigitalInput,
  0
};

/*! @brief Median filters 3 bytes.
 *
 */
static void StandByMode(bool standby)
{
  if (standby)
  {
    //put it in standby mode
    CTRL_REG1_ACTIVE = 0;
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);
  }
  else
  {
    //put it in active mode
    CTRL_REG1_ACTIVE = 1;
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);
  }
}


bool Accel_Init(const uint32_t moduleClk, const AccelSetup_t* const accelSetup)
{
  if (moduleClk == 0)
    return false;

  //initialise call back functions as global variable
  Accel_Setup.dataReadyCallbackArguments = accelSetup->dataReadyCallbackArguments;
  Accel_Setup.dataReadyCallbackFunction = accelSetup->dataReadyCallbackFunction;

  //initialise I2C module values
  TI2CModule aI2CModule;
  aI2CModule.baudRate = ACCEL_BAUD_RATE;
  aI2CModule.primarySlaveAddress = ACCEL_ADDRESS; //Default I2C Slave Address 0x1D (see p5 of schematics)
  aI2CModule.readCompleteCallbackArguments = accelSetup->readCompleteCallbackArguments;
  aI2CModule.readCompleteCallbackFunction = accelSetup->readCompleteCallbackFunction;

  //initialise I2C
  I2C_Init(moduleClk, &aI2CModule);

  //select accelerometer as primary slave address
  I2C_SelectSlaveDevice(aI2CModule.primarySlaveAddress);

  uint8_t tmpData;
  //Check WHO_AM_I register
  I2C_PollRead(ADDRESS_WHO_AM_I, &tmpData, 1);
  if (tmpData != ACCEL_WHO_AM_I)
    return false;

  //put it in standby mode when initialising registers
  StandByMode(true);

  //CTRL_REG1_ACTIVE = 1; //active mode
  CTRL_REG1_F_READ = 1; //data format is limited to 8-bits
  CTRL_REG1_DR = DATE_RATE_6_25_HZ; //set output data rate to 6.25Hz
  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);

  CTRL_REG3_PP_OD = 0; //push-pull for INT2
  CTRL_REG3_IPOL = 1;  //use 0 for INT2 active low
  I2C_Write(ADDRESS_CTRL_REG3, CTRL_REG3);

  //interrupt flag bits are reset by reading the appropriate
  //source register for the function that generated the interrupt.
  CTRL_REG4 = 0;
  I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4);

  //INT2 = 0
  CTRL_REG5 = 0;
  I2C_Write(ADDRESS_CTRL_REG5, CTRL_REG5);

  //put it in active mode
  StandByMode(false);

  //initialise PORTC for interrupt
  CLOCK_EnableClock(kCLOCK_PortC);

  //INT2 interrupt2 see hint 8
  PORTC->PCR[13] = PORT_PCR_MUX(1) | PORT_PCR_ISF_MASK | PORT_PCR_IRQC(0b1001); //0b1010

  Accel_SetMode(ACCEL_MODE_POLL);

  //set nested vectored interrupt control for PORTC
  __NVIC_ClearPendingIRQ(PORTC_IRQn);
  __NVIC_EnableIRQ(PORTC_IRQn);

  return true;
}


void Accel_ReadXYZ(uint8_t data[3])
{
  //read data depending on the accel mode
  if (AccelMode)
    I2C_IntRead(ADDRESS_OUT_X_MSB, data, 3);
  else
    I2C_PollRead(ADDRESS_OUT_X_MSB, data, 3);
}


void Accel_SetMode(const AccelMode_t mode)
{
  EnterCritical();

  //put it in standby mode
  StandByMode(true);

  //sync mode = interrupt
  //or Async mode = polling
  if (mode)
  {
    //enable accel data interrupt
    AccelMode = ACCEL_MODE_INT;
    CTRL_REG4 = 1;
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4);
  }
  else
  {
    //disable accel data interrupt
    AccelMode = ACCEL_MODE_POLL;
    CTRL_REG4 = 0;
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4);
  }
  //put it back in active mode
  StandByMode(false);

  ExitCritical();
}

/*! @brief Shift the old values and include the new values
 *
 *  @param accelOldData array of old data stored.
 *  @param accelNewData new data received by the accelerometer.
 *  @return bool - true if the accelerometer module was successfully initialized.
 */
void Accel_SlidingWindow(uint8_t *accelOldData, uint8_t accelNewData)
{
  EnterCritical();
  uint8_t i;
  //shift all old data
  for (i=(sizeof(accelOldData) - 1); i>0; i--)
  {
    accelOldData[i] = accelOldData[i-1];
  }
  //include new data
  accelOldData[0] = accelNewData;
  ExitCritical();
}

/*! @brief PORTC interrupt request handler clears interrupt and calls user function
 *
 */
void PORTC_IRQHandler(void)
{
  //clear interrupt
  if (PORTC->PCR[13] & PORT_PCR_ISF_MASK)
  {
    PORTC->PCR[13] = PORT_PCR_ISF_MASK;

    //user callback function
    if(Accel_Setup.dataReadyCallbackFunction)
      Accel_Setup.dataReadyCallbackFunction(Accel_Setup.dataReadyCallbackArguments);
  }
}

/*!
** @}
*/

