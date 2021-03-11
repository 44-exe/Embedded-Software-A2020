/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the Simple Serial Communication Protocol.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-04-15
 */


/*!
**  @addtogroup Packet_module Packet module documentation
**  @{
*/

// New types
#include "Types\types.h"
#include "packet.h"
#include "UART\UART.h"
#include "Critical\critical.h"

// Packet structure
#define PACKET_NB_BYTES 5

TPacket Packet; /*!< Packet structure variable*/

//uint8_t  Packet_Command,    /*!< The packet's command */
//    Packet_Parameter1, /*!< The packet's 1st parameter */
//    Packet_Parameter2, /*!< The packet's 2nd parameter */
//    Packet_Parameter3, /*!< The packet's 3rd parameter */
//    Packet_Checksum;   /*!< The packet's checksum */

// Acknowledgement bit mask
const uint8_t PACKET_ACK_MASK = 0x80;

bool Packet_Init(const uint32_t moduleClk, const uint32_t baudRate)
{
  return UART_Init(moduleClk, baudRate);
}

bool Packet_Get(void)
{
  uint8_t data;
  //private global variable to keep count of packets
  static int packetCount = 0;

  while (UART_InChar(&data))        //check if next byte is available
  {
    switch (packetCount)            //go through each case of packets based on number of existing bytes
    {
      case 0:
        Packet_Command = data;      //store the first byte into command
        packetCount++;
        return false;
      case 1:
        Packet_Parameter1 = data;   //store the second byte into Packet1
        packetCount++;
        return false;
      case 2:
        Packet_Parameter2 = data;   //store the third byte into Packet2
        packetCount++;
        return false;
      case 3:
        Packet_Parameter3 = data;   //store the fourth byte into Packet3
        packetCount++;
        return false;
      case 4:
        Packet_Checksum = data;     //store the last byte into Checksum
        packetCount++;
      case 5:
        //test to see if Checksum is correct
        if (Packet_Checksum == (Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3))
        {
          //reset packetCount
          packetCount = 0;
          return true;
        }
        else
        {
          //shift the data and get another byte into Checksum
          packetCount = 4;
          Packet_Command = Packet_Parameter1;
          Packet_Parameter1 = Packet_Parameter2;
          Packet_Parameter2 = Packet_Parameter3;
          Packet_Parameter3 = Packet_Checksum;
        }
        break;
    }
  }
  return false;
}


bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  bool success;
  uint8_t checksum;

  EnterCritical();

  //transmit command
  success = UART_OutChar(command);

  //transmit parameter1
  if (success)
    success &= UART_OutChar(parameter1);

  //transmit parameter2
  if (success)
    success &= UART_OutChar(parameter2);

  //transmit parameter3
  if (success)
    success &= UART_OutChar(parameter3);

  //build and transmit checksum
  if (success)
  {
    checksum = command ^ parameter1 ^ parameter2 ^ parameter3;
    success &= UART_OutChar(checksum);
  }
  ExitCritical();
  return success;
}

/*!
** @}
*/
