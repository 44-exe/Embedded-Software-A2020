/*!
** @file main.c
** @version 1.0
** @brief  Main module.
**
** This file contains high level functions calling functions to bring all
** modules together to operate a voltage regulating relay (VRR).
**
**
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

#include <math.h>

// New types
#include "Types\types.h"
//Flex Timer Module
#include "FTM\FTM.h"
//Periodic Interrupt Timer
#include "PIT\PIT.h"
// Serial Communication Interface
#include "UART\UART.h"
//Flash Interface
#include "Flash\Flash.h"
//LED interface
#include "LEDs\LEDs.h"
// Packet handling
#include "Packet\packet.h"
//analog to digital
#include "ADC\ADC.h"
//Inverse Time calcualtion
#include "InverseTime\InverseTime.h"
//critical
#include "Critical\critical.h"
// Include system for module clock
#include "system_MK64F12.h"
// Simple OS
#include "OS.h"
//Function Generator
#include "FG.h"

/*! @brief FTM callback function Signals Lower Semaphore
 *
 *  @param arg void pointer to null
 */
void FTMCallback0(void *arg);

/*! @brief FTM callback function Signals Raise Semaphore
 *
 *  @param arg void pointer to null
 */
void FTMCallback1(void *arg);

/*! @brief FTM callback function turns off LEDs after 3sec
 *
 *  @param arg - Null
 */
void FTMCallback2(void *arg);

/*! @brief HandlePacketThread to handle packets in a thread.
 *
 *  @param pData - Null
 */
void HandlePacketThread(void *pData);

//enum priority to organise the priority of the thread
enum Priority
{
  PRIORITY_MCU_INIT,
  PRIORITY_UART0_RX,
  PRIORITY_PIT,
  PRIORITY_UART0_TX,
  PRIORITY_ADCRAISE,
  PRIORITY_ADCLOWER,
  PRIORITY_FGOUT,
  PRIORITY_FGPACKET,
  PRIORITY_FTM,
  PRIORITY_HANDLEPACKET,
};


// Thread stacks
OS_THREAD_STACK(STACK_MCU, THREAD_STACK_SIZE);
OS_THREAD_STACK(STACK_HANDLEPACKET, THREAD_STACK_SIZE);
OS_THREAD_STACK(STACK_PIT, THREAD_STACK_SIZE);
OS_THREAD_STACK(STACK_FTM, THREAD_STACK_SIZE);
OS_THREAD_STACK(STACK_ADCRAISE, THREAD_STACK_SIZE);
OS_THREAD_STACK(STACK_ADCLOWER, THREAD_STACK_SIZE);


// ----------------------------------------
// Private global variables
static const uint16_t MCGFFCLK = (12207 * 5); /*!< fixed frequency clock (5 sec)*/
const uint8_t MAJOR = 0x01; /*!< Version number High*/
const uint8_t MINOR = 0x00; /*!< Version number Low*/

const int STU_NUM = 5084; /*!< Last 4 digit of student number*/
const int MODE_NUM = 1;   /*!< MCU mode number*/

static volatile uint16union_t *NvMcuNb; /*!<non volatile MCU Number*/
static volatile uint16union_t *NvMcuMd; /*!<non volatile MCU Mode*/
volatile uint16union_t *NvRaise; /*!<to store Raise count in Flash*/
volatile uint16union_t *NvLower; /*!<to store Lower count in Flash*/
double Freq = 12500000;  /*!< used for frequency tracking of the Function Generator */

// Commands
/*!
 * @enum Command packet enumerated
 */
enum Command
{
  MCU_STARTUP       = 0x04,  //set startup command
  MCU_FLASH_PROGRAM = 0x07,  //set flash write command
  MCU_FLASH_READ    = 0x08,  //set flash read command
  MCU_VERSION       = 0x09,  //set get version command
  MCU_PROTOCOL      = 0x0A,  //set protocol command
  MCU_NUMBER        = 0x0B,  //set MCU number command
  MCU_TIME          = 0x0C,  //set MCU time command
  MCU_MODE          = 0x0D,  //set MCU mode command
  MCU_TIMING_MODE   = 0x10,  //set MCU timing mode command
  MCU_NUM_RAISE     = 0x11,  //set MCU number of raise command
  MCU_NUM_LOWER     = 0x12,  //set MCU number of lower command
  MCU_FREQUENCY     = 0x17,  //set MCU frequency
  MCU_VRMS          = 0x18,  //set MCU Vrms
  MCU_VRR           = 0xE0,  //set MCU VRR command
};


TFTMChannel Channel0FTM = {  /*!<stores FTM channel clock and current mode data*/
    .channelNb = 0, //channel 0
    .delayNanoseconds = MCGFFCLK,//CLOCK_GetFreq(kCLOCK_McgFixedFreqClk);//delay by 1 clock (48828 or 20480)
    .ioType.outputAction = TIMER_OUTPUT_LOW,
    .timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE,
    .callbackFunction = FTMCallback0,
    .callbackArguments = NULL
};

TFTMChannel Channel1FTM = {  /*!<stores FTM channel clock and current mode data*/
    .channelNb = 1, //channel 0
    .delayNanoseconds = MCGFFCLK,//CLOCK_GetFreq(kCLOCK_McgFixedFreqClk);//delay by 1 clock (48828 or 20480)
    .ioType.outputAction = TIMER_OUTPUT_LOW,
    .timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE,
    .callbackFunction = FTMCallback1,
    .callbackArguments = NULL
};

TFTMChannel Channel2FTM = {  /*!<stores FTM channel clock and current mode data*/
    .channelNb = 2, //channel 0
    .delayNanoseconds = 12207 * 3,//CLOCK_GetFreq(kCLOCK_McgFixedFreqClk);//delay by 1 clock (48828 or 20480)
    .ioType.outputAction = TIMER_OUTPUT_LOW,
    .timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE,
    .callbackFunction = FTMCallback2,
    .callbackArguments = NULL
};

ADCSetup_t ADC_Setup = {
    .InverseTime = false,         //initialise inverse time as false
    .Event = true,
    .FTMSetLower = &Channel0FTM,  //set FTM channel for Lower event
    .FTMSetRaise = &Channel1FTM,  //set FTM channel for Raise event
    .FTMOffLED   = &Channel2FTM,  //set FTM channel for turning off LEDs
};

UARTSetup_t UART_Setup = { /*!< to initialise UART structure*/
    .baudRate = 115200,
    .rxPriority = PRIORITY_UART0_RX,
    .txPriority = PRIORITY_UART0_TX
};

const FGSetup_t FG_SETUP = {
    .pPacket = &Packet, //pointer to a global packet structure
    .fgOutPriority = PRIORITY_FGOUT,
    .fgPacketPriority = PRIORITY_FGPACKET
};


InverseTime_t InverseTime; /*!< Inverse time structure used in ADC and to calculate Inverse Time */

/*! @brief PIT callback function samples Vrms from ADC
 *
 *  @param arg void pointer to null
 */
void PITCallback(void *arg)
{
  uint16union_t dataADC;

  dataADC.l = ADC_TrueRMS(ADC_Read16Bit()); //calculated true Vrms

//  dataADC.l = ADC_Read16Bit(); //raw digital data

  //send the RMS voltage to interface
  Packet_Put(0x50, 0x00, dataADC.s.Lo, dataADC.s.Hi);
}


void FTMCallback0(void *arg)
{
  static uint16_t count = 0;
  //at the end of FTM interrupt, if voltage is outside range signal semaphore
  if (ADC_Setup.CompareVoltage == true)
  {
    OS_SemaphoreSignal(ADC_LowerSemaphore);
    count = NvLower->s.Lo + 1;
    Flash_Write16((volatile uint16_t*)NvLower, count);
  }
}

void FTMCallback1(void *arg)
{
  static uint16_t count = 0;
  //at the end of FTM interrupt, if voltage is outside range signal semaphore
  if (ADC_Setup.CompareVoltage == true)
  {
    OS_SemaphoreSignal(ADC_RaiseSemaphore);
    count = NvRaise->l + 1;
    Flash_Write16((volatile uint16_t*)NvRaise, count);
  }
}

void FTMCallback2(void *arg)
{
  LEDs_Off(LED_GREEN);
  LEDs_Off(LED_BLUE);
}



/*! @brief Sends startup packets to the PC.
 *
 *  @return bool - TRUE if sending the startup packets was successful.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool SendStartupPackets(void)
{
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

  //for testing purposes
//  Packet_Put(MCU_VRR, 1, 0, 0);
//  Packet_Put(MCU_VRR, 2, 0, 5);
//  Packet_Put(MCU_VRR, 3, 0xCD, 2);
//  Packet_Put(MCU_VRR, 4, 0, 0);
//  Packet_Put(MCU_VRR, 8, 0, 0);
//  Packet_Put(MCU_VRR, 5, 0, 0);

  return true;
}


/*! @brief Initializes the MCU by initializing all variables and then sending startup packets to the PC.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  //moduleinit
  //Board pin init
  BOARD_InitPins();
  BOARD_InitBootClocks();

  bool success;

  EnterCritical();

  //Initiaise modules
  success  = Flash_Init(); //initialise Flash
  success &= Packet_Init(SystemCoreClock, &UART_Setup); //Packet_Init calls UART_Init which calls FIFO_Init
  success &= LEDs_Init(); //initialise LEDs

  // Initialize FG
  success = FG_Init(SystemCoreClock, &FG_SETUP);
  if (!success)
    DEBUG_HALT();

  //Initialise PIT3
  success &= PIT3_Init(SystemCoreClock, PITCallback, NULL);

  //initialise FTM
  success &= FTM_Init();

  success &= ADC_Init(ADC_Setup);
  if (!success)
    DEBUG_HALT();

  ExitCritical();

  //allocate memory in flash to MCU number and MCU mode
  success &= Flash_AllocateVar((volatile void**)&NvMcuNb, sizeof(*NvMcuNb));
  success &= Flash_AllocateVar((volatile void**)&NvMcuMd, sizeof(*NvMcuMd));

  //allocate memory in flash to Raise event and Lower event
  success &= Flash_AllocateVar((volatile void**)&NvRaise, sizeof(*NvRaise));
  success &= Flash_AllocateVar((volatile void**)&NvLower, sizeof(*NvLower));


  //Initialise the Flash memory with student number and MCU mode
  if (success && (NvMcuNb->l == 0xFFFF))
    Flash_Write16((volatile uint16_t*)NvMcuNb, STU_NUM);

  if (success && (NvMcuMd->l == 0xFFFF))
    Flash_Write16((volatile uint16_t*)NvMcuMd, MODE_NUM);

  //reset the count to 0 in flash memory when initiating modules
  if (success && (NvRaise->l != 0x0000))
    Flash_Write16((volatile uint16_t*)NvRaise, 0x0000);

  if (success && (NvLower->l != 0x0000))
    Flash_Write16((volatile uint16_t*)NvLower, 0x0000);


  //start PIT at 16 samples per cycle at 5Hz
  PIT3_Set(Freq, true);


  InverseTime.InverseTime = false;
  InverseTime.CompareVoltage = false;
  InverseTime.WaveFreq.l = 0;
  InverseTime.FTMRaise = Channel1FTM;
  InverseTime.FTMLower = Channel0FTM;

  success &= SendStartupPackets();

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Initialises and creates threads used in VRR
 *
 */
void CreateThread(void)
{
//   Module Init Thread
  if (OS_ThreadCreate(InitModulesThread, NULL, &STACK_MCU[THREAD_STACK_SIZE - 1], PRIORITY_MCU_INIT) != OS_NO_ERROR)
    DEBUG_HALT();

//   Packet Thread
  if (OS_ThreadCreate(HandlePacketThread, NULL, &STACK_HANDLEPACKET[THREAD_STACK_SIZE - 1], PRIORITY_HANDLEPACKET) != OS_NO_ERROR)
    DEBUG_HALT();

//   PIT Thread
  if (OS_ThreadCreate(PITThread, NULL, &STACK_PIT[THREAD_STACK_SIZE - 1], PRIORITY_PIT) != OS_NO_ERROR)
    DEBUG_HALT();

//   FTM Thread
  if (OS_ThreadCreate(FTMThread, NULL, &STACK_FTM[THREAD_STACK_SIZE - 1], PRIORITY_FTM) != OS_NO_ERROR)
    DEBUG_HALT();

//   ADC Raise Thread
  if (OS_ThreadCreate(ADCRaiseThread, NULL, &STACK_ADCRAISE[THREAD_STACK_SIZE - 1], PRIORITY_ADCRAISE) != OS_NO_ERROR)
    DEBUG_HALT();

//   ADC Lower Thread
  if (OS_ThreadCreate(ADCLowerThread, NULL, &STACK_ADCLOWER[THREAD_STACK_SIZE - 1], PRIORITY_ADCLOWER) != OS_NO_ERROR)
    DEBUG_HALT();
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
  if (Packet_Parameter1 <= 7 && Packet_Parameter2 == 0x00)
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


/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
//static bool HandleMCUTime(void)
//{
//  //check for corrent parameter
//  if (Packet_Parameter1 < 24 && Packet_Parameter2 < 60 && Packet_Parameter3 < 60)
//  {
//    //set time
//    RTC_Set(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
//    return true;
//  }
//  return false;
//}

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
      if (Packet_Parameter23 == 0x0000)
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
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleMCUTimingMode(void)
{
  //return false if packet parameter 2 and 3 is not equal to 0
  if (Packet_Parameter23 != 0)
    return false;

  switch (Packet_Parameter1)
  {
    case 0: //get
      if (ADC_Setup.InverseTime == false)
        Packet_Put(Packet_Command, 0x01, 0x00, 0x00);
      else if (ADC_Setup.InverseTime == true)
        Packet_Put(Packet_Command, 0x02, 0x00, 0x00);
      break;
    case 1: //definite (5sec)
      ADC_Setup.InverseTime = false; //set to definite time mode
      Packet_Put(MCU_VRR, 0x08, 0x00, 0x00);
      break;
    case 2: //inverse
      ADC_Setup.InverseTime = true; //set to inverse time mode
      Packet_Put(MCU_VRR, 0x08, 0x01, 0x00);
      break;
  }

  return true;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleMCUNumRaise(void)
{
  //return false if packet parameter 2 and 3 is not equal to 0
  if (Packet_Parameter23 != 0)
    return false;

  bool success = false;
  switch (Packet_Parameter1)
  {
    case 0: //get
      Packet_Put(Packet_Command, Packet_Parameter1, NvRaise->s.Lo, NvRaise->s.Hi);
      return true;
    case 1: //reset
      success = Flash_Write16((volatile uint16_t*)NvRaise, 0);
  }

  return success;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleMCUNumLower(void)
{
  //return false if packet parameter 2 and 3 is not equal to 0
  if (Packet_Parameter23 != 0)
    return false;

  bool success = false;
  switch (Packet_Parameter1)
  {
    case 0: //get
      Packet_Put(Packet_Command, Packet_Parameter1, NvLower->s.Lo, NvLower->s.Hi);
      return true;
    case 1: //reset
      success = Flash_Write16((volatile uint16_t*)NvLower, 0);
  }

  return success;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleFrequency(void)
{
  //Convert decimal frequency to packets and set it in MCU
  if (Packet_Command == 0x17)
    Packet_Put(Packet_Command, round((double)InverseTime.WaveFreq.s.Lo/25.6), InverseTime.WaveFreq.s.Hi, 0x00);

  Freq = InverseTime.WaveFreq.s.Hi + (round((double)InverseTime.WaveFreq.s.Lo/25.6)/10);
  if (Freq > 5.25)
    Freq = 5.25;
  PIT3_Set(((1/Freq)/16) * 1e9, true);

  return true;
}

/*! @brief Respond to packets sent from the PC.
 *
 *  @return bool - TRUE if the packet was handled successfully.
 *  @note Assumes that MCUInit has been called successfully.
 */
static bool HandleVrms(void)
{
  bool success = false;

//  Packet_Put(MCU_VRR, 0x00, Packet_Parameter1 * 25.6, Packet_Parameter2);

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
    for (count = 0; count < 5; count++)
    {
      Packet_Get();
    }

    switch (Packet_Command & 0x7F)
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
//      case MCU_TIME: //used in RTC (RTC not used in VRR)
//        success = HandleMCUTime();
        break;
      case MCU_MODE:
        success = HandleMCUMode();
        break;
      case MCU_TIMING_MODE: //VRR
        success = HandleMCUTimingMode();
        break;
      case MCU_NUM_RAISE: //VRR
        success = HandleMCUNumRaise();
        break;
      case MCU_NUM_LOWER: //VRR
        success = HandleMCUNumLower();
        break;
      case MCU_FREQUENCY: //VRR
        success = HandleFrequency();
        break;
      case MCU_VRMS: //VRR
        success = HandleVrms();
        break;
      default:
        (void)OS_SemaphoreSignal(FG_HandlePacketSemaphore);
        (void)OS_SemaphoreWait(FG_PacketProcessed, 0);
        success = FG_PacketHandledSuccessfully;

        if (Packet_Command == MCU_VRR && Packet_Parameter1 == 0x02)
        {
          InverseTime.WaveFreq.s.Hi = Packet_Parameter3; //stores the whole frequency of the function generator
          InverseTime.WaveFreq.s.Lo = Packet_Parameter2; //stores the decimal frequency of the function generator
          HandleFrequency(); //set frequency after receiving frequency packets
        }
        break;
    }

    //check if ACK is requested
    if ((Packet_Command & PACKET_ACK_MASK) == PACKET_ACK_MASK && (success == true))
      Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    else if ((Packet_Command & PACKET_ACK_MASK) == PACKET_ACK_MASK && (success == false))
      Packet_Put(Packet_Command & 0x0F, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
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

  return 0;
}

/*!
** @}
*/
