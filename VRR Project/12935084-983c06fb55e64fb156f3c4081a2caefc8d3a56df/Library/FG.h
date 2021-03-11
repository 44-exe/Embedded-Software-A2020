/*! @file
 *
 *  @brief Routines for implementing a function generator (FG).
 *
 *  This contains the functions for implementing an arbitrary waveform generator linked to the MCUPC interface.
 *
 *  @author PMcL
 *  @date 2020-06-05
 */

#ifndef FG_H
#define FG_H

// Simple OS
#include "OS.h"

// Packet handling
#include "Packet\packet.h"

extern bool FG_PacketHandledSuccessfully;

extern OS_ECB* FG_HandlePacketSemaphore; /*!< Semaphore used to initiate FG packet handling. */
extern OS_ECB* FG_PacketProcessed;          /*!< Semaphore used to signal the end of FG packet handling. */

typedef struct
{
  TPacket* pPacket;            /*!< A pointer to the global Packet structure. */
  uint8_t fgOutPriority;       /*!< The priority of the FG output value preparation thread. */
  uint8_t fgPacketPriority;    /*!< The priority of the FG packet handling thread. */
} FGSetup_t;

/*! @brief Initializes the FG before first use.
 *
 *  @param pitModuleClk The PIT module clock rate in Hz.
 *  @note Assumes that pitModuleClk has a period which can be expressed as an integral number of nanoseconds.
 *  @param pFGSetup A pointer to an FG setup structure which holds the initialisation information.
 *  @return bool - TRUE if module initialization was successful.
 */
bool FG_Init(const uint32_t pitModuleClk, const FGSetup_t* const pFGSetup);

/*! @brief Respond to FG packets sent from the PC.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @return void
 *  @note blocking function - thread will be suspended if there is no packet to handle.
 */
void FG_HandlePackets(void* pData);

/*! @brief Sends the status of the function generator.
 *
 *  @return void
 *  @note blocking function - thread will be suspended until it can send the status successfully.
 */
void FG_SendStatus(void);

#endif
