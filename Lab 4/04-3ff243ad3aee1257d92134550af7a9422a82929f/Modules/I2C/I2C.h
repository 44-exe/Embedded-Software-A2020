/*! @file
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  This contains the functions for operating the I2C (inter-integrated circuit) module.
 *
 *  @author PMcL
 *  @date 2015-09-17
 */

#ifndef I2C_H
#define I2C_H

// new types
#include "Types\types.h"

typedef struct
{
  uint8_t primarySlaveAddress;
  uint32_t baudRate;
  void (*readCompleteCallbackFunction)(void*);  /*!< The user's read complete callback function. */
  void* readCompleteCallbackArguments;          /*!< The user's read complete callback function arguments. */
} TI2CModule;

/*! @brief Sets up the I2C before first use.
 *
 *  @param moduleClk The module clock in Hz.
 *  @param aI2CModule is a structure containing the operating conditions for the module.
 *  @return BOOL - TRUE if the I2C module was successfully initialized.
 */
bool I2C_Init(const uint32_t moduleClk, const TI2CModule* const aI2CModule);

/*! @brief Selects the current slave device
 *
 * @param slaveAddress The slave device address.
 */
void I2C_SelectSlaveDevice(const uint8_t slaveAddress);

/*! @brief Write a byte of data to a specified register
 *
 * @param registerAddress The register address.
 * @param data The 8-bit data to write.
 */
void I2C_Write(const uint8_t registerAddress, const uint8_t data);

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses polling as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes);

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses interrupts as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes);

#endif
