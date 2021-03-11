/*! @file
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the FXOS8700CQ accelerometer.
 *
 *  @author PMcL
 *  @date 2020-03-04
 */

#ifndef ACCEL_H
#define ACCEL_H

// New types
#include "Types\types.h"

// Accelerometer registers
#include "Accel\accel_FXO.h"

// Simple OS
#include "OS.h"

typedef enum
{
  ACCEL_MODE_POLL,
  ACCEL_MODE_INT
} AccelMode_t;

typedef struct
{
  uint32_t moduleClk;                             /*!< The module clock rate in Hz. */
  uint8_t configurePriority;                      /*!< configurePriority is the configure thread priority for the OS. */
  uint8_t dataReadyPriority;                      /*!< dataReadyPriority is the data ready thread priority for the OS. */
  uint8_t readCompletePriority;                   /*!< readCompletePriority is the read complete thread priority for the OS. */
  void (*dataReadyCallbackFunction)(void* pData); /*!< The user's data ready callback function. */
  void* dataReadyCallbackArguments;               /*!< The user's data ready callback function arguments. */
} AccelSetup_t;

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[3];				/*!< The accelerometer data accessed as an array. */
  struct
  {
    uint8_t x, y, z;			/*!< The accelerometer data accessed as individual axes. */
  } axes;
} AccelData_t;


extern AccelSetup_t AccelSetup;

extern AccelData_t AccelData;

extern AccelMode_t AccelMode;

#pragma pack(pop)

extern OS_ECB* Accel_ReadCompleteSemaphore;  /*!< A semaphore used to signal waiting threads that the accelerometer has completed a read operation. */
extern OS_ECB* Accel_DataReadySemaphore;     /*!< A semaphore used to signal waiting threads from the Accel Data Ready ISR. */

/*! @brief Initializes the accelerometer by calling the initialization routines of the supporting software modules.
 *
 *  @param moduleClk is the module clock rate in Hz.
 *  @param accelSetup is a pointer to an accelerometer setup structure.
 *  @return bool - true if the accelerometer module was successfully initialized.
 */
bool Accel_Init(const uint32_t moduleClk, const AccelSetup_t* const accelSetup);


/*! @brief Reads X, Y and Z accelerations.
 *  @param data is an array of 3 bytes where the X, Y and Z data are stored.
 */
void Accel_ReadXYZ(uint8_t data[3]);

/*! @brief Set the mode of the accelerometer.
 *  @param mode specifies either polled or interrupt driven operation.
 */
void Accel_SetMode(const AccelMode_t mode);


void AccelThread(void *pData);

#endif
