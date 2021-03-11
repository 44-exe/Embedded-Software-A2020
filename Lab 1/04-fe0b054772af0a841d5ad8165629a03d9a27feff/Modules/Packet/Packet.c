/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the Simple Serial Communication Protocol.
 *
 *  @author Jeong Bin Lee
 *  @date 2015-07-23
 */


// New types
#include "Types\types.h"
#include "packet.h"
#include "UART\UART.h"

// Packet structure
#define PACKET_NB_BYTES 5



uint8_t  Packet_Command,    /*!< The packet's command */
    Packet_Parameter1, /*!< The packet's 1st parameter */
    Packet_Parameter2, /*!< The packet's 2nd parameter */
    Packet_Parameter3, /*!< The packet's 3rd parameter */
    Packet_Checksum;   /*!< The packet's checksum */

// Acknowledgement bit mask
const uint8_t PACKET_ACK_MASK;

//private global variable to keep count of packets
static int PacketCount = 0;

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param moduleClk The module clock rate in Hz.
 *  @param baudRate The desired baud rate in bits/sec.
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t moduleClk, const uint32_t baudRate)
{
  return UART_Init(moduleClk, baudRate);
}


bool Packet_Get(void)
{
  uint8_t data;

  while (UART_InChar(&data))        //check if next byte is available
  {
    switch (PacketCount)            //go through each case of packets based on number of existing bytes
    {
      case 0:
        Packet_Command = data;      //store the first byte into command
        PacketCount++;
        //break;
        return false;
      case 1:
        Packet_Parameter1 = data;   //store the second byte into Packet1
        PacketCount++;
        //break;
        return false;
      case 2:
        Packet_Parameter2 = data;   //store the third byte into Packet2
        PacketCount++;
        //break;
        return false;
      case 3:
        Packet_Parameter3 = data;   //store the fourth byte into Packet3
        PacketCount++;
        return false;
      case 4:
        Packet_Checksum = data;     //store the last byte into Checksum
        PacketCount++;
      case 5:
        //test to see if Checksum is correct
        if (Packet_Checksum == (Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3))
        {
          //reset PacketCount
          PacketCount = 0;
          return true;
        }
        else
        {
          //shift the data and get another byte into Checksum
          PacketCount = 4;
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


/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  bool success;
  uint8_t checksum;

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

  return success;
}

