/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the Simple Serial Communication Protocol.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */

#ifndef PACKET_H
#define PACKET_H

// New types
#include "Types\types.h"

// Serial Communication Interface
#include "UART\UART.h"

// Packet structure
#define PACKET_NB_BYTES 5

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[PACKET_NB_BYTES];     /*!< The packet as an array of bytes. */
  struct
  {
    uint8_t command;		      /*!< The packet's command. */
    union
    {
      struct
      {
        uint8_t parameter1;	      /*!< The packet's 1st parameter. */
        uint8_t parameter2;	      /*!< The packet's 2nd parameter. */
        uint8_t parameter3;	      /*!< The packet's 3rd parameter. */
      } separate;
      struct
      {
        uint16_t parameter12;         /*!< Parameter 1 and 2 concatenated. */
        uint8_t parameter3;
      } combined12;
      struct
      {
        uint8_t paramater1;
        uint16_t parameter23;         /*!< Parameter 2 and 3 concatenated. */
      } combined23;
    } parameters;
    uint8_t checksum;
  } packetStruct;
} TPacket;

#pragma pack(pop)

#define Packet_Command     Packet.packetStruct.command
#define Packet_Parameter1  Packet.packetStruct.parameters.separate.parameter1
#define Packet_Parameter2  Packet.packetStruct.parameters.separate.parameter2
#define Packet_Parameter3  Packet.packetStruct.parameters.separate.parameter3
#define Packet_Parameter12 Packet.packetStruct.parameters.combined12.parameter12
#define Packet_Parameter23 Packet.packetStruct.parameters.combined23.parameter23
#define Packet_Checksum    Packet.packetStruct.checksum

extern TPacket Packet;

// Acknowledgment bit mask
extern const uint8_t PACKET_ACK_MASK;


/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param moduleClk The module clock rate in Hz.
 *  @param uartSetup is a structure containing the parameters to be used in setting up the UART.
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t moduelClk, const UARTSetup_t* const uartSetup);

/*! @brief Attempts to get a packet from the received data.
 *  Blocking implementation - if there is no packet to get, the thread will be suspended
 *
 *  @return void
 *  @note blocking function - threads will be suspended if there is no packet to get
 */
void Packet_Get(void);

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *  Blocking implementation - if there is no free space, the thread will be suspended
 *
 *  @note blocking function - threads will be suspended until they are able to put a packet
 */
void Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

#endif
