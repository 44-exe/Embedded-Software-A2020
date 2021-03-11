/*! @file
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the structure and "methods" for accessing a byte-wide FIFO.
 *
 *  @author George El Bazouni
 *  @date 25/03/2020
 */

// new types
#include "FIFO.h"


/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return bool - TRUE if the FIFO was successfully initialised
 */
bool FIFO_Init(TFIFO* const FIFO)
{
  FIFO->Start   = 0;      // First FIFO_Get function will start at the first byte in the FIFO buffer
  FIFO->End     = 0;      // First FIFO_Put function will start at the first byte in the FIFO buffer
  FIFO->NbBytes = 0;      // No bytes stored initially
// By initialising, any existing FIFO register data will be overwritten when reached in the array.

  if (FIFO->Start == 0 && FIFO->End == 0 && FIFO->NbBytes == 0)
    return true;          // boolean return for successful initialisation
  else
    return false;         // unsuccessful initialisation, will force project to stop
}


/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Put(TFIFO* const FIFO, const uint8_t data)
{
  if(FIFO->NbBytes == FIFO_SIZE)    //Check for: if stored bytes exceeds maximum buffer size
    return false; 			        //If max size reached, cancel receiving of byte from UART

  //put the item in the buffer and update the circular indexes
  FIFO->Buffer[FIFO->End]= data;    //current buffer position value overwritten by value in address pointed to from UART
  FIFO->NbBytes++;			        //increase counter for number of bytes currently stored by '1'.
  FIFO->End++;					    //Moves buffer address for next received byte to next slot

  if (FIFO->End == FIFO_SIZE) 	    // End needs to wrap-around to the beginning
    FIFO->End=0; 				    //Next received byte to overwrite stored byte in first buffer address.

  return true;
}

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct wilocationth data to be retrieved.
 *  @param dataPtr A pointer to a memory  to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Get(TFIFO* const FIFO, uint8_t* const dataPtr)
{
  if (FIFO->NbBytes == 0)    //Check for: any not transmitted stored bytes: '0' means nothing to send
    return false;

  // code to retrieve a character from the FIFO
  *dataPtr = FIFO->Buffer[FIFO->Start];   //points to address in buffer with next byte to transmit
  FIFO->NbBytes--;	            	      //decrements no. of not transmitted stored bytes by '1'
  FIFO->Start++;						  //moves pointer to next byte to be transmitted in the buffer.

  if (FIFO->Start == FIFO_SIZE)  //Start needs to wrap-around to the beginning
    FIFO->Start=0;               //Next transmitted byte to be sent from address of stored byte in first buffer address.

  return true;
}
