/*!
** @file
** @version 5.0
** @brief  Main module.
**
**   This file contains the high-level code for the project.
**   It initialises appropriate hardware subsystems,
**   creates application threads, and then starts the OS.
**
**   An example of three threads communicating via a semaphore
**   is given that flashes the tri-color LED. These should be removed
**   when the use of threads and the RTOS is understood.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */

// Provided drivers
#include "clock_config.h"
#include "pin_mux.h"

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"


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
//accelerometer
//#include "Accel\accel.h"
//I2C (Inter-Intergrated Circuit)
//#include "I2C\I2C.h"
// Include system for module clock
#include "system_MK64F12.h"
// Simple OS
#include "OS.h"

#define NB_LEDS 3


void FTMCallback(void *arg);
void HandlePacketThread(void *pData);


enum Priority
{
  PRIORITY_MCU_INIT,
  PRIORITY_UART0_RX,
  PRIORITY_UART0_TX,
  PRIORITY_HANDLEPACKET,
  PRIORITY_I2C,
  PRIORITY_ACCEL,
  PRIORITY_FTM,
  PRIORITY_PIT,
  PRIORITY_RTC
};


// Thread stacks
//OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
//static uint32_t MyLEDThreadStacks[NB_LEDS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08))); /*!< The stacks for the LED threads. */

//uint32_t STACK_MCU[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//uint32_t STACK_UART0_TX[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//uint32_t STACK_UART0_RX[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//uint32_t STACK_HANDLEPACKET[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

OS_THREAD_STACK(STACK_MCU, THREAD_STACK_SIZE);
OS_THREAD_STACK(STACK_HANDLEPACKET, THREAD_STACK_SIZE);
//OS_THREAD_STACK(STACK_UART0_TX, THREAD_STACK_SIZE);
//OS_THREAD_STACK(STACK_UART0_RX, THREAD_STACK_SIZE);


//OS_THREAD_STACK(STACK_I2C, THREAD_STACK_SIZE);
//OS_THREAD_STACK(STACK_ACCEL, THREAD_STACK_SIZE);
//OS_THREAD_STACK(STACK_FTM, THREAD_STACK_SIZE);
//OS_THREAD_STACK(STACK_PIT, THREAD_STACK_SIZE);
//OS_THREAD_STACK(STACK_RTC, THREAD_STACK_SIZE);

// ----------------------------------------
// Private global variables
static volatile uint16union_t *NvMcuNb; /*!<non volatile MCU Number*/
static volatile uint16union_t *NvMcuMd; /*!<non volatile MCU Mode*/

static const uint16_t MCGFFCLK = 48828; /*!< fixed frequency clock*/

const uint8_t MAJOR = 0x01; /*!< Version number High*/
const uint8_t MINOR = 0x00; /*!< Version number Low*/

const int STU_NUM = 5084; /*!< Last 4 digit of student number*/
const int MODE_NUM = 1;   /*!< MCU mode number*/

//OS_ECB *Accel_Ready_Semaphore;  /*!< Signals when Accelerometer is ready to be read */
//OS_ECB *Accel_complete_Semaphore; /*!< Signals when Accelerometer finishes reading data */
OS_ECB *RTC_Sempahore; /*!< Signals RTC value change */

// Commands
/*!
 * @enum Command packet enumerated
 */
enum Command
{
  MCU_STARTUP       = 0x04,  //set startup command
  MCU_FLASH_PROGRAM = 0x07,  //set flash write command
  MCU_FLASH_READ    = 8,  //set flash read command
  MCU_VERSION       = 0x09,  //set get version command
  MCU_PROTOCOL      = 0x0A,  //set protocol command
  MCU_NUMBER        = 0x0B,  //set mcu number command
  MCU_TIME          = 0x0C,  //set mcu time command
  MCU_MODE          = 0x0D,   //set mcu mode command
  MCU_ACCEL         = 0x10  //set mcu accel command
};


//static TFTMChannel ChannelFTM = {  /*!<stores FTM channel clock and current mode data*/
//    .channelNb = 0, //channel 0
//    .delayNanoseconds = MCGFFCLK,//CLOCK_GetFreq(kCLOCK_McgFixedFreqClk);//delay by 1 clock (48828 or 20480)
//    .ioType.outputAction = TIMER_OUTPUT_LOW,
//    .timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE,
//    .callbackFunction = FTMCallback,
//    .callbackArguments = NULL
//};

UARTSetup_t UART_Setup = { /*!< to initialise UART structure*/
    .baudRate = 115200,
    .rxPriority = PRIORITY_UART0_RX,
    .txPriority = PRIORITY_UART0_TX
};

/*! @brief Sends startup packets to the PC.
 *
 *  @return bool - TRUE if sending the startup packets was successful.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool SendStartupPackets(void)
{
  bool success = true;
  //MCU_STARTUP
  Packet_Put(MCU_STARTUP, 0, 0, 0);
  //GET_VERSION
  Packet_Put(MCU_VERSION, 'v', MAJOR, MINOR);
  //MCU_NUMBER
  Packet_Put(MCU_NUMBER, 0x01, NvMcuNb->s.Lo, NvMcuNb->s.Hi);
  //MCU_MODE
  Packet_Put(MCU_MODE, 0x01, NvMcuMd->s.Lo, NvMcuMd->s.Hi);
  //MCU_PROTOCOL
  Packet_Put(MCU_PROTOCOL, 0x01, 0x00, 0x00);

  return success;
}



/*! @brief Initializes the MCU by initializing all variables and then sending startup packets to the PC.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  //moduleinit
  // Board pin init
  BOARD_InitPins();
  BOARD_InitBootClocks();

  bool success;

  //Initiaise modules
  success  = Flash_Init(); //initialise Flash
  success &= Packet_Init(SystemCoreClock, &UART_Setup); //Packet_Init calls UART_Init which calls FIFO_Init
  success &= LEDs_Init(); //initialise LEDs

  //allocate memory in flash to MCU number and MCU mode
  success &= Flash_AllocateVar((volatile void**)&NvMcuNb, sizeof(*NvMcuNb));
  success &= Flash_AllocateVar((volatile void**)&NvMcuMd, sizeof(*NvMcuMd));

  //Initialise the Flash memory with student number and MCU mode
  if (success && (NvMcuNb->l == 0xFFFF))
    Flash_Write16((volatile uint16_t*)NvMcuNb, STU_NUM);

  if (success && (NvMcuMd->l == 0xFFFF))
    Flash_Write16((volatile uint16_t*)NvMcuMd, MODE_NUM);


//  HandlePacketSemaphore = OS_SemaphoreCreate(0);
//  PacketGetSemaphore = OS_SemaphoreCreate(0);

  success &= SendStartupPackets();

//  (void)OS_SemaphoreSignal(HandlePacketSemaphore);


//  // Generate the global LED semaphores
//  for (uint8_t ledNb = 0; ledNb < NB_LEDS; ledNb++)
//    MyLEDThreadData[ledNb].semaphore = OS_SemaphoreCreate(0);
//
//  // Signal the first LED to toggle
//  (void)OS_SemaphoreSignal(MyLEDThreadData[0].semaphore);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

void CreateThread(void)
{
  // Module Init Thread
  if (OS_ThreadCreate(InitModulesThread, NULL, &STACK_MCU[THREAD_STACK_SIZE - 1], PRIORITY_MCU_INIT) != OS_NO_ERROR)
    DEBUG_HALT();

  // Uart transmit Thread
 // if (OS_ThreadCreate(UART0TxThread, NULL, &STACK_UART0_TX[THREAD_STACK_SIZE - 1], PRIORITY_UART0_TX) != OS_NO_ERROR)
 //   DEBUG_HALT();

  // Uart receive Thread
 // if (OS_ThreadCreate(UART0RxThread, NULL, &STACK_UART0_RX[THREAD_STACK_SIZE - 1], PRIORITY_UART0_RX) != OS_NO_ERROR)
 //   DEBUG_HALT();

  // Packet Thread
  if (OS_ThreadCreate(HandlePacketThread, NULL, &STACK_HANDLEPACKET[THREAD_STACK_SIZE - 1], PRIORITY_HANDLEPACKET) != OS_NO_ERROR)
   DEBUG_HALT();

//  OS_ThreadCreate(I2CThread, NULL, &STACK_I2C[OS_CREATE_STACK_SIZE], PRIORITY_I2C);
//  OS_ThreadCreate(AccelThread, NULL, &STACK_ACCEL[OS_CREATE_STACK_SIZE], PRIORITY_ACCEL);

  // FTM Thread
 // if (OS_ThreadCreate(FTMThread, NULL, &STACK_FTM[THREAD_STACK_SIZE], PRIORITY_FTM) != OS_NO_ERROR)
 //   DEBUG_HALT();

  // PIT Thread
 // if (OS_ThreadCreate(PITThread, NULL, &STACK_PIT[THREAD_STACK_SIZE], PRIORITY_PIT) != OS_NO_ERROR)
 //   DEBUG_HALT();

  // RTC Thread
 // if (OS_ThreadCreate(RTCThread, NULL, &STACK_RTC[THREAD_STACK_SIZE], PRIORITY_RTC) != OS_NO_ERROR)
 //   DEBUG_HALT();
}



/*! @brief Respond to a Startup packet sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleStartupPacket(void)
{
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
    Packet_Put(MCU_FLASH_READ, Packet_Parameter1 + FLASH_DATA_START, Packet_Parameter2,
        _FB(Packet_Parameter1 + FLASH_DATA_START));
  return success;
}


/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleGetVersion(void)
{
  bool success = true;

  if (Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 0x0D)
    Packet_Put(MCU_VERSION, Packet_Parameter1, MAJOR, MINOR);
  else
    success = false;

  return success;
}


static bool HandleMCUProtocol(void)
{
  //for a get
  if(Packet_Parameter1 == 1)
  {
    Packet_Put(MCU_PROTOCOL, Packet_Parameter1, 0x00, 0x00);
    return true;
  }

//  //for a set
//  if (Packet_Parameter1 == 2)
//  {
//    //polling mode
//    if (Packet_Parameter2 == ACCEL_MODE_POLL)
//    {
//      Accel_SetMode(ACCEL_MODE_POLL);
//      Packet_Put(MCU_PROTOCOL, Packet_Parameter1, Packet_Parameter2, 0x00);
//      return true;
//    }
//    //interrupt mode
//    if (Packet_Parameter2 == ACCEL_MODE_INT)
//    {
//      Accel_SetMode(ACCEL_MODE_INT);
//      Packet_Put(MCU_PROTOCOL, Packet_Parameter1, Packet_Parameter2, 0x00);
//      return true;
//    }
//  }
  return false;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleMCUNumber(void)
{
  bool success = true;

  if (Packet_Parameter1 == 0x01 && Packet_Parameter23 == 0x0000)
    Packet_Put(MCU_NUMBER, Packet_Parameter1, NvMcuNb->s.Lo, NvMcuNb->s.Hi);
  else if (Packet_Parameter1 == 0x02)
  {
    Flash_Write16((volatile uint16_t*)NvMcuNb, Packet_Parameter23);
    Packet_Put(MCU_NUMBER, Packet_Parameter1, NvMcuNb->s.Lo, NvMcuNb->s.Hi);
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
    //RTC_Set(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
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
      {
        Packet_Put(MCU_MODE, Packet_Parameter1, NvMcuMd->s.Lo, NvMcuMd->s.Hi);
        return true;
      }
      success = false;
      break;
    case 2: //set
      Flash_Write16((volatile uint16_t*)NvMcuMd, Packet_Parameter23);
      Packet_Put(MCU_MODE, Packet_Parameter1, NvMcuMd->s.Lo, NvMcuMd->s.Hi);
      break;
  }
  return success;
}


/*! @brief Respond to packets sent from the PC.
 *
 *  @note Assumes that MCUInit has been called successfully.
 */
void HandlePacketThread(void *pData)
{
  // Pseudocode:
  // 1. Get a packet.
  // 2. "switch" on the Packet_Command, but ignore the top bit.
  // 3. Create a case for each command, which calls the individual command handler.
  // e.g.
  //      case CMD_STARTUP:
  //      success = HandleStartupPacket();
  //      break;
  // 4. Check whether an ACK was requested from the PC.
  // 5. If an ACK was requested then send an ACK/NAK packet based on the success of the command handler.

  for (;;)
  {
    bool success = false;

    uint8_t count = 0;

    //the loop is to receive a full packet before handling packets
    for(count = 0; count < 5; count++)
      Packet_Get();

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
        case MCU_PROTOCOL:
          success = HandleMCUProtocol();
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
//      if (success)
//      {
//        if (FTM_StartTimer(&ChannelFTM))
//          LEDs_On(LED_BLUE);
//      }

      //check if ACK is requested
      if ((Packet_Command & PACKET_ACK_MASK) == PACKET_ACK_MASK && (success == true))
        Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
      else if ((Packet_Command & PACKET_ACK_MASK) == PACKET_ACK_MASK && (success == false))
        Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
//    }
  }
}


/*! @brief Initialises the hardware, sets up threads, and starts the OS.
 *
 */
int main(void)
{
  // Initialize the RTOS
  OS_Init(SystemCoreClock);

  //create threads
  CreateThread();

  // Start multithreading - never returns!
  OS_Start();
  // If the program returns from OS_Start, we have a major problem!
  DEBUG_HALT();
  return 1;
}

/*!
** @}
*/
