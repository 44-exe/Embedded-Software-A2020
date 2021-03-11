/*! @file Flash.c
 *
 *  @brief Routines for erasing and writing to the Flash.
 *
 *  This contains the functions needed for accessing the internal Flash.
 *
 *  @author Jeong Bin Lee and George El Bazouni
 *  @date 2020-04-15
 */

/*!
**  @addtogroup Flash_module Flash module documentation
**  @{
*/

// new types
//#include <stdlib.h>
//#include "Types\types.h"
#include "Flash.h"
#include "device\MK64F12.h"

//Flash Commands
const uint8_t FLASH_ERASE_SECTOR = 0x09; /*!< Flash command erase sector*/
const uint8_t FLASH_WRITE_PHRASE = 0x07; /*!< Flash command write phrase*/


static bool ByteFlash[8]; /*!< virtual memory for flash*/

static int AddressFlash[8]; /*!< flash address sector*/


bool Flash_Init(void)
{
  int i;  //count int to initialise all 8 bytes of used flash sector

  for (i = 0; i < 8; i++)
  {
    ByteFlash[i] = true; //if memory slot is empty == true, if not empty == false
    AddressFlash[i] =  FLASH_DATA_START + i;  //0x0008 0000 --- 0x0008 0007
  }

  return true;
}


bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  int i;

  if (size == 1)  //check if size == 1 (1 byte)
  {
    //first check if the address is empty or not
    //if not empty, go to next memory
    //if empty use that memory
    for (i = 0; i < 8; i += size)
    {
      if (ByteFlash[i])
      {
        *variable = (void *) AddressFlash[i];
        ByteFlash[i] = false;
        return true;
      }
    }
  }
  else if (size == 2)  //check if size == 2 (half word)
  {
    //first check if the address is empty or not
    //if not empty, go to next memory incremented by size
    //if empty use that memory
    for (i = 0; i < 8; i += size)
    {
      if (ByteFlash[i])
      {
        *variable = (void *) AddressFlash[i];
        ByteFlash[i] = false;
        ByteFlash[i+1] = false;
        return true;
      }
    }
  }
  else if (size == 4)  //check if size == 4 (word)
  {
    //first check if the address is empty or not
    //if not empty, go to next memory incremented by size
    //if empty use that memory
    for (i = 0; i < 8; i += size)
    {
      if (ByteFlash[i])
      {
        *variable = (void *) AddressFlash[i];
        ByteFlash[i] = false;
        ByteFlash[i+1] = false;
        ByteFlash[i+2] = false;
        ByteFlash[i+3] = false;
        return true;
      }
    }
  }
  else if (size == 8)
  {
    if (ByteFlash[0])
    {
      *variable = (void *) AddressFlash[0];
      for (i=0; i<size; i++)
        ByteFlash[i] = false;
      return true;
    }
  }
  return false;
}

/*! @brief Checks the status of Flash register and launches the command given.
 *
 *  @param commonCommandObject a structure variable to store data to be written into flash.
 *
 *  @return bool - TRUE if flash command was written properly
 */
static bool LaunchCommand(FCCOB_t* commonCommandObject)
{
  bool success = true;
  //wait while CCIF is 1, if CCIF is 0, cannot write to FCCOB
  while (!(FTFE->FSTAT & FTFE_FSTAT_CCIF_MASK));

  //these flags must be cleared before launching command (before writing 1 to CCIF)
  //see reference manual p.702
  if ((FTFE->FSTAT & FTFE_FSTAT_FPVIOL_MASK) == FTFE_FSTAT_FPVIOL_MASK)
  {
    FTFE->FSTAT |= FTFE_FSTAT_FPVIOL_MASK;
    success = false;
  }
  if ((FTFE->FSTAT & FTFE_FSTAT_ACCERR_MASK) == FTFE_FSTAT_ACCERR_MASK)
  {
    FTFE->FSTAT |= FTFE_FSTAT_ACCERR_MASK;
    success = false;
  }


  //Set FCCOB in terms of big endian
  FTFE->FCCOB0 = commonCommandObject->Flash_Command;
  FTFE->FCCOB1 = commonCommandObject->Address.s.Flash_Address[2];
  FTFE->FCCOB2 = commonCommandObject->Address.s.Flash_Address[1];
  FTFE->FCCOB3 = commonCommandObject->Address.s.Flash_Address[0];

  FTFE->FCCOB4 = commonCommandObject->Data.s.Flash_DataByte[3];
  FTFE->FCCOB5 = commonCommandObject->Data.s.Flash_DataByte[2];
  FTFE->FCCOB6 = commonCommandObject->Data.s.Flash_DataByte[1];
  FTFE->FCCOB7 = commonCommandObject->Data.s.Flash_DataByte[0];

  FTFE->FCCOB8 = commonCommandObject->Data.s.Flash_DataByte[7];
  FTFE->FCCOB9 = commonCommandObject->Data.s.Flash_DataByte[6];
  FTFE->FCCOBA = commonCommandObject->Data.s.Flash_DataByte[5];
  FTFE->FCCOBB = commonCommandObject->Data.s.Flash_DataByte[4];

  //after writing to FCCOB, set CCIF to 1
  FTFE->FSTAT |= FTFE_FSTAT_CCIF_MASK;

  //wait while command complete is 1
  while (!(FTFE->FSTAT & FTFE_FSTAT_CCIF_MASK));

  return success;
}

/*! @brief Writes a phrase into the FCCOB_t structure and launch write command to flash.
 *
 *  @param address 32 bit integer of address to be written to flash
 *  @param phrase 64 bit of data to be written to flash.
 *  @return bool - TRUE if the packet was handled successfully.
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  FCCOB_t write;
  //assign write phrase command
  write.Flash_Command = FLASH_WRITE_PHRASE;
  //assign address to be written to
  write.Address.Flash_AllAddress = address;
  //assign given phrase to the structure
  write.Data.Flash_AllByte = phrase.l;

  return LaunchCommand(&write);
}

/*! @brief Modifies the current phrase
 *
 *  @param address 32 bit integer of address to be written to flash
 *  @param phrase 64 bit of data to be written to flash.
 *  @return bool - TRUE if the phrase was written properly.
 */
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase)
{
  //first erase flash, then write entire phrase
  return Flash_Erase() && WritePhrase(address, phrase);
}


bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  uint64union_t Phrase; /*!< Carries high and low phrase components */
  uint32_t WriteAddress = (uint32_t)address; /*!< Start address */

  if (((WriteAddress / 4)) % 2 == 0) //checks if starting address is at 0
  {
    Phrase.s.Lo = data;
    Phrase.s.Hi = _FW(WriteAddress + 4);
    return ModifyPhrase(WriteAddress, Phrase); //Returns phrase to be written in flash and address
  }
  else
  {
    Phrase.s.Lo = _FW(WriteAddress - 4);
    Phrase.s.Hi = data;
    return ModifyPhrase(WriteAddress - 4, Phrase);
  }
}


bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  uint32union_t Word; /*!< Carries high and low word components */
  uint32_t WriteAddress = (uint32_t)address; /*!< Start address */

  if (WriteAddress % 4 == 0) //checks if starting address is at 0 or 4
  {
    Word.s.Lo = data;
    Word.s.Hi = _FH(WriteAddress+2);
    return Flash_Write32(&(_FW(WriteAddress)),Word.l); //Returns word to be written in flash and address to phrase modifier
  }
  else
  {
    Word.s.Lo = _FH(WriteAddress-2);
    Word.s.Hi = data;
    return Flash_Write32(&(_FW(WriteAddress-2)),Word.l);
  }
}


bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  uint16union_t Halfword; /*!< Carries high and low half-word components */
  uint32_t WriteAddress = (uint32_t)address; /*!< Start address */

  if (WriteAddress % 2 == 0) //checks if starting address is at 0, 2, 4, or 6
  {
    Halfword.s.Lo = data;
    Halfword.s.Hi = _FB(WriteAddress+1);
    return Flash_Write16(&(_FH(WriteAddress)),Halfword.l); //Returns half-word to be written in flash and address to word modifier
  }
  else
  {
    Halfword.s.Lo = _FB(WriteAddress-1);
    Halfword.s.Hi = data;
    return Flash_Write16(&(_FH(WriteAddress - 1)),Halfword.l);
  }

}


/*! @brief Erases the entire Flash sector.
 *
 *  @param address The address of the data.
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
static bool EraseSector(const uint32_t address)
{
  FCCOB_t erase;
  erase.Flash_Command = FTFE_FCCOB0_CCOBn(0x09); //Macro for accessing commands from MK64F12.h
  erase.Address.Flash_AllAddress = address; //clears the 24 bit starting address

  return LaunchCommand(&erase);
}


bool Flash_Erase(void)
{
  return EraseSector(FLASH_DATA_START);
}

/*!
** @}
*/

