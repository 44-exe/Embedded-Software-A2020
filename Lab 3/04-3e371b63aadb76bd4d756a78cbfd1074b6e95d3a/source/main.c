/*!
** @file main.c
** @version 3.0
** @brief
**         Main module.
**         This module implements serial port (USB) communications with a PC.
**         The Simple Serial Communication Protocol is used.
**         The command handlers for a several commands (defined in the protocol) are implemented.
**
** @author Jeong Bin Lee and George El Bazouni
** @date   25/03/2020
*/

/*!
**  @addtogroup main_module main module documentation
**  @{
*/

/* MODULE main */

#include "clock_config.h"
#include "pin_mux.h"
// New types
#include "Types\types.h"
//Periodic Interrupt Timer
#include "PIT\PIT.h"
//Real Time Clock
#include "RTC\RTC.h"
//Flex Timer Module
#include "FTM\FTM.h"
// Serial Communication Interface
#include "UART\UART.h"
//Flash Interface
#include "Flash\Flash.h"
//LED interface
#include "LEDs\LEDs.h"
// Packet handling
#include "Packet\packet.h"
//critical
#include "Critical\critical.h"
// Include system for module clock
#include "system_MK64F12.h"
//clock generation
//#include "drivers\fsl_clock.h"


// Version number
// TODO: Define the major version number and minor version number using const.
const uint8_t MAJOR = 0x01; /*!< Version number High*/
const uint8_t MINOR = 0x00; /*!< Version number Low*/

const int STU_NUM = 5084;
const int MODE_NUM = 1;

// TODO: Define the UART baud rate using const.
const uint32_t BAUDRATE = 115200; /*!< baudrate*/

// Commands
// TODO: Define the commands using enum.
/*!
<<<<<<< HEAD
 * @enum Command packet enumerated
=======
 * @enum Command packets enumerated
>>>>>>> develop
 */
enum Command
{
  MCU_STARTUP       = 0x04,  //set startup command
  MCU_FLASH_PROGRAM = 0x07,  //set flash write command
  MCU_FLASH_READ    = 0x08,  //set flash read command
  MCU_VERSION       = 0x09,  //set get version command
  MCU_NUMBER        = 0x0B,  //set mcu number command
  MCU_TIME          = 0x0C,  //set mcu time command
  MCU_MODE          = 0x0D   //set mcu mode command
};

// ----------------------------------------
// Private global variables

// TODO: Define the MCU number.
//static uint8_t McuNumbLsb = 0xDC; //for setting MCU number
//static uint8_t McuNumbMsb = 0x13; //for settting MCU number
//
//static uint8_t McuModeLsb = 0x01; //for setting MCU mode
//static uint8_t McuModeMsb = 0x00; //for setting MCU mode

static volatile uint16union_t *NvMcuNb; /*!<non volatile MCU Number*/
static volatile uint16union_t *NvMcuMd; /*!<non volatile MCU Mode*/

static TFTMChannel ChannelFTM; /*!<stores FTM channel clock and current mode data*/


/*! @brief RTC callback function toggles red LED and sends time to PC
 *
 *  @param arg void pointer to null
 */
void RTCCallback(void* arg)
{
  uint8_t hours, minutes, seconds;
  //toggle red LED
  LEDs_Toggle(LED_RED);
  //send current time to PC
  RTC_Get(&hours, &minutes, &seconds);
  Packet_Put(MCU_TIME, hours, minutes, seconds);
}

/*! @brief PIT callback function toggles green LED
 *
 *  @param arg void pointer to null
 */
void PITCallback(void* arg)
{
  //toggle green LED
  LEDs_Toggle(LED_GREEN);
}

/*! @brief FTM callback function turns off blue LED
 *
 *  @param arg void pointer to null
 */
void FTMCallback(void* arg)
{
  //turn blue LED off
  LEDs_Off(LED_BLUE);
}

/*! @brief Sets initial FTM channel structure values
 *
 *  @note Assumes the FTM has been initialized.
 */
static void StructInit(void)
{
  //initialise FTM structure
  ChannelFTM.channelNb = 0; //channel 0
  ChannelFTM.delayNanoseconds = 48828;//CLOCK_GetFreq(kCLOCK_McgFixedFreqClk);//delay by 1 clock (48828 or 20480)
  ChannelFTM.ioType.outputAction = TIMER_OUTPUT_LOW;
  ChannelFTM.timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE;
  ChannelFTM.callbackFunction = FTMCallback;
  ChannelFTM.callbackArguments = NULL;
}

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
  success &= Packet_Put(MCU_VERSION, 'v', MAJOR, MINOR);
  //MCU_NUMBER
  success &= Packet_Put(MCU_NUMBER, 0x01, NvMcuNb->s.Lo, NvMcuNb->s.Hi);
  //MCU_MODE
  success &= Packet_Put(MCU_MODE, 0x01, NvMcuMd->s.Lo, NvMcuMd->s.Hi);

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

  //no interrupts while initialising modules
  EnterCritical();

  //uint32_t priMask = DisableGlobalIRQ();
  bool success;
  uint8_t hrs, min, sec;

  //Initiaise modules
  success  = Flash_Init(); //initialise Flash
  success &= Packet_Init(SystemCoreClock, BAUDRATE); //Packet_Init calls UART_Init which calls FIFO_Init
  success &= LEDs_Init(); //initialise LEDs

  success &= PIT_Init(CLOCK_GetFreq(kCLOCK_BusClk), PITCallback, NULL);

  success &= RTC_Init(RTCCallback, NULL);

  success &= FTM_Init();
  FTM_Set(&ChannelFTM);

  //enable interrupts after module initialisation
  ExitCritical();

  RTC_Get(&hrs, &min, &sec);
  RTC_Set(hrs,min,sec);

  //**********TEST OUT THE NUMBER THEN TEST OUT THE MODE***********//
  //allocate memory in flash to MCU number and MCU mode
  success &= Flash_AllocateVar((volatile void**)&NvMcuNb, sizeof(*NvMcuNb));
  success &= Flash_AllocateVar((volatile void**)&NvMcuMd, sizeof(*NvMcuMd));

  //Initialise the Flash memory with student number and MCU mode
  if (success && (NvMcuNb->l == 0xFFFF))
    Flash_Write16((volatile uint16_t*)NvMcuNb, STU_NUM);

  if (success && (NvMcuMd->l == 0xFFFF))
    Flash_Write16((volatile uint16_t*)NvMcuMd, MODE_NUM);

  //LEDs_On(LED_GREEN);

  //send the startup packets
  success &= SendStartupPackets();



  // TODO: Initialize any modules that need to be initialized.
  return success;
}

/*! @brief Respond to a Startup packet sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleStartupPacket(void)
{
  // TODO: Respond to a startup packet sent from the PC
  if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
    return SendStartupPackets();

  return false;
}

/*! @brief Programs one byte from Flash
 *
 *  @return bool - TRUE if the packet was programmed successfully.
 */
static bool HandleFlashProgram(void)
{
  bool success = false;
  uint32_t *address;
  //if Packet_Parameter1 == 0x08, erase all
  if (Packet_Parameter1 == 0x08)
    success = Flash_Erase();

  address = (uint32_t*)(Packet_Parameter1 + FLASH_DATA_START);
  if (Packet_Parameter1 <=7 && Packet_Parameter2 == 0x00)
    success = Flash_Write8((volatile uint8_t*)address, Packet_Parameter3);
  return success;
}

/*! @brief Reads one byte from Flash
 *
 *  @return bool - TRUE if the packet was read successfully.
 */
static bool HandleFlashRead(void)
{
  bool success = false;
  //read from the flash memory and return the value
  if (Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter23 == 0x0000)
    success = Packet_Put(MCU_FLASH_READ, Packet_Parameter1 + FLASH_DATA_START, Packet_Parameter2,
        _FB(Packet_Parameter1 + FLASH_DATA_START));
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
    success = Packet_Put(MCU_VERSION, Packet_Parameter1, MAJOR, MINOR);
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

  if (Packet_Parameter1 == 0x01 && Packet_Parameter23 == 0x0000)
    success = Packet_Put(MCU_NUMBER, Packet_Parameter1, NvMcuNb->s.Lo, NvMcuNb->s.Hi);
  else if (Packet_Parameter1 == 0x02)
  {
    Flash_Write16((volatile uint16_t*)NvMcuNb, Packet_Parameter23);
    success = Packet_Put(MCU_NUMBER, Packet_Parameter1, NvMcuNb->s.Lo, NvMcuNb->s.Hi);
  }
  else
    success = false;

  return success;
}

static bool HandleMCUTime(void)
{
  //check for corrent parameter
  if (Packet_Parameter1 < 24 && Packet_Parameter2 < 60 && Packet_Parameter3 < 60)
  {
    //set time
    RTC_Set(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    return true;
  }
  return false;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleMCUMode(void)
{
  bool success = false;

  switch (Packet_Parameter1)
  {
    case 1: //get
      if(Packet_Parameter23 == 0x0000)
        success = Packet_Put(MCU_MODE, Packet_Parameter1, NvMcuMd->s.Lo, NvMcuMd->s.Hi);
      success = false;
      break;
    case 2: //set
      Flash_Write16((volatile uint16_t*)NvMcuMd, Packet_Parameter23);
      success = Packet_Put(MCU_MODE, Packet_Parameter1, NvMcuMd->s.Lo, NvMcuMd->s.Hi);
      break;
  }
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
  // 2. "switch" on the Packet_Command, but ignore the top bit.
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
    case MCU_FLASH_PROGRAM:
      success = HandleFlashProgram();
      break;
    case MCU_FLASH_READ:
      success = HandleFlashRead();
      break;
    case MCU_VERSION:
      success = HandleGetVersion();     //handle get version packet
      break;
    case MCU_NUMBER:
      success = HandleMCUNumber();      //handle mcu number packet
      break;
    case MCU_TIME:
      success = HandleMCUTime();
      break;
    case MCU_MODE:
      success = HandleMCUMode();
      break;
  }

  if (success)
  {
    if (FTM_StartTimer(&ChannelFTM))
      LEDs_On(LED_BLUE);
  }

  //check if ACK is requested
  if ((Packet_Command & PACKET_ACK_MASK) == PACKET_ACK_MASK && (success == true))
    Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  else if ((Packet_Command & PACKET_ACK_MASK) == PACKET_ACK_MASK && (success == false))
    Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
}



// TODO: Upgrade to Lab 3 functionality (copy Lab 2 main and use that as a starting point).

/*!
 * @brief Main function
 */
int main(void)
{
  StructInit();

  //if initialisation failed return 0
  if (!MCUInit())
    return 0;


  for (;;)
  {
    // TODO:
    // 1. Poll the UART.
    //UART_Poll();
    // 2. Handle any packets received.
    HandlePackets();
  }
}

/* END main */
/*!
** @}
*/
