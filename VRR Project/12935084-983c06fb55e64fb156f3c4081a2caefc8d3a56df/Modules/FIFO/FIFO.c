/*! @file FIFO.c
 *
 *  @brief Routines to implement a FIFO buffer.
 *
 *  This contains the structure and "methods" for accessing a byte-wide FIFO.
 *
 *  @author George El Bazouni && Jeong Bin Lee
 *  @date 2020-03-25
 */

/*!
**  @addtogroup FIFO_module FIFO module documentation
**  @{
*/

// new types
#include "FIFO.h"
#include "Critical\critical.h"

bool FIFO_Init(TFIFO* const fifo)
{
  if (!fifo)
    return false;         // unsuccessful initialisation, will force project to stop

  fifo->Start   = 0;      // First FIFO_Get function will start at the first byte in the FIFO buffer
  fifo->End     = 0;      // First FIFO_Put function will start at the first byte in the FIFO buffer

  //see page 321 of topic notes
  fifo->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE); //to signal occurrence of one or more events
  fifo->ItemsAvailable = OS_SemaphoreCreate(0); //to signal occurrence of one or more events

  return true;          // boolean return for successful initialisation
}

void FIFO_Put(TFIFO* const fifo, const uint8_t data)
{
  //Check if there is space available in fifo
  OS_SemaphoreWait(fifo->SpaceAvailable, 0);
  OS_DisableInterrupts();
  //put the item in the buffer and update the circular indexes
  fifo->Buffer[fifo->End]= data;    //current buffer position value overwritten by value in address pointed to from UART
  //fifo->NbBytes++;			        //increase counter for number of bytes currently stored by '1'.
  fifo->End++;					    //Moves buffer address for next received byte to next slot

  if (fifo->End == FIFO_SIZE) 	    // End needs to wrap-around to the beginning
    fifo->End = 0; 				    //Next received byte to overwrite stored byte in first buffer address.
  OS_SemaphoreSignal(fifo->ItemsAvailable); //see page 321 of topic notes
  OS_EnableInterrupts();
//  OS_SemaphoreSignal(fifo->ItemsAvailable); //see page 321 of topic notes
}

void FIFO_Get(TFIFO* const fifo, uint8_t* const dataPtr)
{
  //Check for: any not transmitted stored bytes: '0' means nothing to send
  OS_SemaphoreWait(fifo->ItemsAvailable, 0);

  OS_DisableInterrupts();
  // code to retrieve a character from the FIFO
  *dataPtr = fifo->Buffer[fifo->Start];   //points to address in buffer with next byte to transmit
//    fifo->NbBytes--;	            	      //decrements no. of not transmitted stored bytes by '1'
  fifo->Start++;						  //moves pointer to next byte to be transmitted in the buffer.

  if (fifo->Start == FIFO_SIZE)  //Start needs to wrap-around to the beginning
    fifo->Start=0;               //Next transmitted byte to be sent from address of stored byte in first buffer address.
  OS_SemaphoreSignal(fifo->SpaceAvailable); // Since a buffer item was removed, there is now space left so signal the semaphore.
  OS_EnableInterrupts();
}

/*!
** @}
*/
