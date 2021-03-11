/*!
** @file main.c
** @version 1.0
** @brief
**         Main module.
**         This module implements serial port (USB) communications with a PC.
**         The Simple Serial Communication Protocol is used.
**         The command handlers for a several commands (defined in the protocol) are implemented.
**
** @author Jeong Bin Lee
** @date   25/03/2020
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @{
*/
/* MODULE main */

#include "clock_config.h"
#include "pin_mux.h"
// New types
#include "Types\types.h"
// Serial Communication Interface
#include "UART\UART.h"
// Packet handling
#include "Packet\packet.h"
// Include system for module clock
#include "system_MK64F12.h"


// Version number
// TODO: Define the major version number and minor version number using const.
const uint8_t MAJOR = 0x01;
const uint8_t MINOR = 0x00;

// TODO: Define the UART baud rate using const.
const uint32_t baudRate = 38400;

// Commands
// TODO: Define the commands using enum.
enum
{
  MCU_STARTUP = 0x04,  //set startup command
  GET_VERSION = 0x09,  //set get version command
  MCU_NUMBER  = 0x0B   //set mcu number command
};

// ----------------------------------------
// Private global variables

// TODO: Define the MCU number.
static uint8_t McuLsb = 0xDC;//for setting MCU number
static uint8_t McuMsb = 0x13;//for settting MCU number


/*! @brief Sends startup packets to the PC.
 *
 *  @return bool - TRUE if sending the startup packets was successful.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool SendStartupPackets(void)
{
  bool success;
  // TODO: Send startup packets to the PC.
  //MCU_STARTUP
  success = Packet_Put(MCU_STARTUP, 0, 0, 0);
  //GET_VERSION
  success = Packet_Put(GET_VERSION, 'v', MAJOR, MINOR);
  //MCU_NUMBER
  success = Packet_Put(MCU_NUMBER, 0x01, McuLsb, McuMsb);

  return success;
}

/*! @brief Initializes the MCU by initializing all variables and then sending startup packets to the PC.
 *
 *  @return bool - TRUE if sending the startup packets was successful.
 */
static bool MCUInit(void)
{
  BOARD_InitPins();
  BOARD_InitBootClocks();

  // TODO: Initialize any modules that need to be initialized.
  return Packet_Init(SystemCoreClock, baudRate); //Packet_Init calls UART_Init which calls FIFO_Init
}

/*! @brief Respond to a Startup packet sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleStartupPacket(void)
{
  bool success = true;
  // TODO: Respond to a startup packet sent from the PC
  if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    success &= Packet_Put(0x04, 0x00, 0x00, 0x00);
    success &= Packet_Put(0x09, 'v', MAJOR, MINOR);
    success &= Packet_Put(0x0B, 0x01, McuLsb, McuMsb);
  }

  return success;
}


// TODO: Create individual functions to handle each command sent from the PC, similar to HandleStartupPacket above.

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleGetVersion(void)
{
  bool success;

  if (Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 0x0D)
    success = Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, MAJOR, MINOR);
  else
    success = false;

  return success;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleMCUNumber(void)
{
  bool success;

  if (Packet_Parameter1 == 0x01 && Packet_Parameter2 == 0x00 && Packet_Parameter3 == 0x00)
    success = Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, McuLsb, McuMsb);
  else if (Packet_Parameter1 == 0x02)
  {
    McuLsb = Packet_Parameter2;
    McuMsb = Packet_Parameter3;
    success = Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  }
  else
    success = false;

  return success;
}




/*! @brief Respond to packets sent from the PC.
 *
 *  @note Assumes that MCUInit has been called successfully.
 */
static void HandlePackets(void)
{
  // TODO: Create a packet handler
  // Pseudocode:
  // 1. Get a packet.
  // 2. "switch" on the Pcaket_Command, but ignore the top bit.
  // 3. Create a case for each command, which calls the individual command handler.
  // e.g.
  //      case CMD_STARTUP:
  //      success = HandleStartupPacket();
  //      break;
  // 4. Check whether an ACK was requested from the PC.
  // 5. If an ACK was requestedm then send an ACK/NAK packet based on the success of the command handler.

  bool success = false;
  //uint8_t command = Packet_Command & 0x0F;

  if (!Packet_Get())                    //attempt to get packet
    return;

  switch (Packet_Command & 0x0F)
  {
    case MCU_STARTUP:
      success = HandleStartupPacket();  //handle startup packets
      break;
    case GET_VERSION:
      success = HandleGetVersion();     //handle get version packet
      break;
    case MCU_NUMBER:
      success = HandleMCUNumber();      //handle mcu number packet
      break;
  }

  //check if ACK is requested
  if ((Packet_Command & 0x80) == 0x80 && (success == true))
    Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  else if ((Packet_Command & 0x80) == 0x80 && (success == false))
    Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
}

/*!
 * @brief Main function
 */
int main(void)
{
  MCUInit();
  SendStartupPackets();
  for (;;)
  {
    // TODO:
    // 1. Poll the UART.
    UART_Poll();
    // 2. Handle any packets received.
    HandlePackets();
  }
}

/* END main */
/*!
** @}
*/
