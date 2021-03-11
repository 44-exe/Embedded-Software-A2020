/*! @file
 *
 *  @brief Routines to access the LEDs on the FRDM-K64F.
 *
 *  This contains the functions for operating the LEDs.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-04-14
 */

/*!
**  @addtogroup LEDs_module LEDs module documentation
**  @{
*/

// new types
#include "Types\types.h"
#include "LEDs.h"
#include "device\MK64F12.h"
#include "drivers\fsl_port.h"
#include "drivers\fsl_gpio.h"

const gpio_pin_config_t PORTLEDOUT =
{
  kGPIO_DigitalOutput,
  1
};

const port_pin_config_t SETPORT =
{
    .pullSelect = kPORT_PullDisable,
    .slewRate = kPORT_SlowSlewRate,
    .passiveFilterEnable = kPORT_PassiveFilterDisable,
    .openDrainEnable = kPORT_OpenDrainDisable,
    .driveStrength = kPORT_LowDriveStrength,
    .mux = kPORT_MuxAsGpio,
    .lockRegister = kPORT_UnlockRegister
};


bool LEDs_Init(void)
{
  CLOCK_EnableClock(kCLOCK_PortE);

  PORT_SetPinConfig(PORTB, 21, &SETPORT); //blue
  PORT_SetPinConfig(PORTB, 22, &SETPORT); //red
  PORT_SetPinConfig(PORTE, 26, &SETPORT); //green

  GPIO_PinInit(GPIOB, 21, &PORTLEDOUT);
  GPIO_PinInit(GPIOB, 22, &PORTLEDOUT);
  GPIO_PinInit(GPIOE, 26, &PORTLEDOUT);

  return true;
}


void LEDs_On(const LED_t color)
{
  //0 = red,  1 = green, 2 = blue
  if(color == 21 || color == 22)
    GPIOB->PCOR |= (1<<color);
  if(color == 26)
    GPIOE->PCOR |= (1<<color);
}


void LEDs_Off(const LED_t color)
{
  if(color == 21 || color == 22)
    GPIOB->PSOR |= (1<<color);
  if(color == 26)
    GPIOE->PSOR |= (1<<color);
}


void LEDs_Toggle(const LED_t color)
{
  if(color == 21 || color == 22)
    GPIOB->PTOR |= (1<<color);
  if(color == 26)
    GPIOE->PTOR |= (1<<color);
}


/*!
** @}
*/
