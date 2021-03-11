/*! @file I2C.c
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  This contains the functions for operating the I2C (inter-integrated circuit) module.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-13
 */

/*!
**  @addtogroup I2C_module I2C module documentation
**  @{
*/

#include "stdlib.h"
#include "I2C.h"
#include "Critical\critical.h"
#include "device\MK64F12.h"
#include "drivers\fsl_port.h"
#include "drivers\fsl_clock.h"
#include "drivers\fsl_common.h"

TI2CModule I2CModule; /*!<I2C module structure variable*/

enum
{
  STATE_IDLE,
  STATE_DEVADDRESS,
  STATE_DEVADDRESS2,
  STATE_START,
  STATE_RESTART,
  STATE_REGADDRESS,
  STATE_DATA,
  STATE_STOP
};

static bool IntRead = true; /*!<Boolean variable for IntRead*/
static uint8_t NbBytes; /*!< Total number of bytes for IntRead*/
static uint8_t *Data; /*!< Pointer to the data for IntRead*/

const port_pin_config_t I2C0_CONFIG =
{
    .pullSelect          = kPORT_PullDisable,
    .slewRate            = kPORT_SlowSlewRate,
    .passiveFilterEnable = kPORT_PassiveFilterDisable,
    .openDrainEnable     = kPORT_OpenDrainEnable,
    .driveStrength       = kPORT_LowDriveStrength,
    .mux                 = kPORT_MuxAlt5,
    .lockRegister        = kPORT_UnlockRegister
};


bool I2C_Init(const uint32_t moduleClk, const TI2CModule* const aI2CModule)
{
  //see page 1563 for slave and master init

  if(moduleClk == 0)
    return false;

  uint32_t calcBR, tmpBR1;
  const uint32_t TARGETBR = 100000;
  uint8_t i, j, mult, icr;
  static const uint8_t MULT[] = {1, 2, 4};
  static const uint16_t DIVIDER[] = { 20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56, 68, 48, 56,
                                      64, 72, 80, 88, 104,128, 80,  96,  112, 128, 144, 160, 192, 240, 160, 192,
                                      224, 256, 288, 320, 384, 480, 320, 384, 448, 512, 576, 640, 768, 960, 640,
                                      768,  896,  1024, 1152, 1280, 1536, 1920, 1280, 1536, 1792, 2048, 2304,
                                      2560, 3072, 3840 };

  I2CModule.readCompleteCallbackFunction = aI2CModule->readCompleteCallbackFunction;
  I2CModule.readCompleteCallbackArguments = aI2CModule->readCompleteCallbackArguments;

  //enable clock gating for I2C
  CLOCK_EnableClock(kCLOCK_I2c0);

  //initialise the portE 24 and 25 to be I2C0_SCL and I2C_SDA (ALT5) for accelerometer
  //see page 5 on schematics
  PORT_SetPinConfig(PORTE, 24, &I2C0_CONFIG); //I2C0_SCL (open drain enabled)(pull up resistor enabled)
  PORT_SetPinConfig(PORTE, 25, &I2C0_CONFIG); //I2C0_SDA (open drain enabled)(pull up resistor enabled)

  //set up baud rate close to 100kbps, see 51.3.2 in manual
  //do an exhaustive search to find achievable baud rate closest to 100kbps (use 2 for loops)
  //also see table 51-2 in manual
  //I2C baud rate = I2C module clock speed (Hz)/(mult × SCL divider)
  //where mult  = 1, 2 or 4
  for (i=0; i<sizeof(MULT); i++)
  {
    for(j=0; j < (sizeof(DIVIDER)/sizeof(DIVIDER[0])); j++)
    {
      calcBR = (moduleClk / (MULT[i] * DIVIDER[j]));
      if(abs(TARGETBR - calcBR) < abs(TARGETBR - tmpBR1))
      {
        tmpBR1 = (moduleClk/(MULT[i] * DIVIDER[j]));
        mult = i;
        icr = j;
      }
    }
  }

  //set I2C baudrate
  I2C0->F = I2C_F_MULT(mult) | I2C_F_ICR(icr);
  //enable I2C module channel 0
  I2C0->C1 |= I2C_C1_IICEN_MASK;

  //I2C0->FLT = 0; //not using glitch filters

  //set nested vectored interrupt control for I2C
  __NVIC_ClearPendingIRQ(I2C0_IRQn);
  __NVIC_EnableIRQ(I2C0_IRQn);

  return true;
}


/*! @brief Selects the current slave device
 *
 * @param slaveAddress The slave device address.
 */
void I2C_SelectSlaveDevice(const uint8_t slaveAddress)
{
  I2CModule.primarySlaveAddress = slaveAddress;
}

/*! @brief Waits for the acknowledgment from the slave device
 *
 */
void I2CWaitAck(void)
{
  int timeOut = 100000;

  while ((I2C0->S & I2C_S_IICIF_MASK) == 0 || timeOut > 0)
    timeOut--;
}

/*! @brief Waits until the bus idle
 *
 *
 */
static void I2CWaitIdle(void)
{
  // Wait till bus is idle
  while(I2C0->S & I2C_S_BUSY_MASK);
}

/*! @brief Send a start signal
 *
 *
 */
static void I2CStartSignal(void)
{
  //when MST is changed from 0 to 1 START signal is generated
  I2C0->C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;
}

/*! @brief Send a stop signal
 *
 *
 */
static void I2CStopSignal(void)
{
  //when MST is changed from 1 to 0 STOP signal is generated
  I2C0->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK);
}

/*! @brief Check and clear the arbitration flag
 *
 *
 */
static void I2CArbitrationLost(void)
{
  if (I2C0->S & I2C_S_ARBL_MASK)
    I2C0->S = I2C_S_ARBL_MASK;
}


void I2C_Write(const uint8_t registerAddress, const uint8_t data)
{

  //uint8_t writeState = STATE_IDLE;
  EnterCritical();

  I2C_ArbitrationLost();
  //wait until idle
  I2C_WaitIdle();
  //send start signal
  I2C_StartSignal();
  //send slave address
  I2C0->D = (I2CModule.primarySlaveAddress << 1); //with a write bit
  I2C_WaitAck();
  //send register address
  I2C0->D = registerAddress;
  I2C_WaitAck();
  //read data
  I2C0->D = data;
  I2C_WaitAck();
  //send stop signal
  I2C_StopSignal();

  ExitCritical();
}

void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  EnterCritical();
  //use polling method to read data from register
  uint8_t i;
  //uint8_t readState = STATE_IDLE;
  //  The I2C_S_BUSY_MASK is used to determine whether the I2C bus is idle.
  //  You should check this before you initiate a communication.

  I2C_ArbitrationLost();
  //wait until it is idle
  I2C_WaitIdle();
  //send start signal
  I2C_StartSignal();
  //write slave address
  I2C0->D = (I2CModule.primarySlaveAddress << 1); //with a write bit
  I2C_WaitAck();
  //send register address
  I2C0->D = registerAddress;
  I2C_WaitAck();
  //re-send start signal
  I2C0->C1 |= I2C_C1_RSTA_MASK;
  //write slave address
  I2C0->D = (I2CModule.primarySlaveAddress << 1) | 0b1; //with read bit
  I2C_WaitAck();
  //enable read, NAK
  I2C0->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
  //dummy read data
  data[0] = I2C0->D;
  I2C_WaitAck();

  for(i=0; i< nbBytes - 1; i++)
  {
    //read data and wait for ack
    data[i] = I2C0->D;
    I2C_WaitAck();
  }

  I2C0->C1 |= I2C_C1_TXAK_MASK;
  data[i++] = I2C0->D; //read 2nd last byte
  I2C_WaitAck();
  //send stop signal
  I2C_StopSignal();
  data[i++] = I2C0->D; //read last byte

  ExitCritical();
}

void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  EnterCritical();
//  if (I2C0->C1 & I2C_C1_IICIE_MASK)
//  {
//    ExitCritical();
//    return;
//  }

  //enable interrupts for I2C
  I2C0->C1 |= I2C_C1_IICIE_MASK;
  //initialise interrupt read variables
  if (IntRead)
  {
    NbBytes = nbBytes;
    Data = data;//point at the same address as data
    IntRead = false;
  }

  //  The I2C_S_BUSY_MASK is used to determine whether the I2C bus is idle.
  //  You should check this before you initiate a communication.
  I2C_WaitIdle(); //wait until I2C is idle
  //send start signal
  I2C_StartSignal();

  //write device address + write/read = 0/1
  I2C0->D = (I2CModule.primarySlaveAddress << 1);
  //wait for ack
  I2C_WaitAck();
  //write register address
  I2C0->D = registerAddress;
  //wait for ack
  I2C_WaitAck();
  //repeat start condition
  I2C0->C1 |= I2C_C1_RSTA_MASK;
  //write device address + write/read = 0/1
  I2C0->D = (I2CModule.primarySlaveAddress << 1) | 0b00000001;
  //wait for ack
  I2C_WaitAck();
  //change to receive mode
  I2C0->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
  //send stop signal
  I2C_StopSignal();

  Data[0] = I2C0->D; //read from the register to initiate read
  //wait for ack
  I2C_WaitAck();

  ExitCritical();
}

/*! @brief I2C interrupt request handler
 *
 */
void I2C0_IRQHandler(void)
{
  static uint8_t nbBytes = 0;
  //A flowchart for a typical I2C interrupt service routine can be found in the
  //K70 Reference Manual as Figure 51-6.
  //see page 1565 of K64 manual for the flow chart
  //clear interrupt (w1c)
  I2C0->S = I2C_S_IICIF_MASK;

  //Check TCF mask p.1559
  if (I2C0->S & I2C_S_TCF_MASK)
  {
    if (!(I2C0->C1 & I2C_C1_TX_MASK))
    {
      switch(NbBytes)
      {
      //second last byte to read see page 1565
      case 2:
        //set TXAK
        I2C0->C1 |= I2C_C1_TXAK_MASK;
        Data[nbBytes] = I2C0->D;
        nbBytes++;
        NbBytes--;
        break;
      //last byte to read
      case 1:
        I2C_StopSignal();
        Data[nbBytes] = I2C0->D;
        nbBytes++;
        NbBytes--;
        break;
      //finished reading to the last byte
      case 0:
        //reinitialise the static and global variables
        nbBytes = 0;
        IntRead = true;
        //disable interrupt
        I2C0->C1 &= ~I2C_C1_IICIE_MASK;
        //run the user callback
        if(I2CModule.readCompleteCallbackFunction)
          I2CModule.readCompleteCallbackFunction(I2CModule.readCompleteCallbackArguments);
        break;
      //if NbBytes > 2
      default:
        Data[nbBytes] = I2C0->D;
        nbBytes++;
        NbBytes--;
        break;
      }
    }
  }
}


/*!
** @}
*/

