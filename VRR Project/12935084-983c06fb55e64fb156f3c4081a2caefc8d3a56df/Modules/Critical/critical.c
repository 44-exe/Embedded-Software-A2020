/*! @file
 *
 *  @brief Routines to implement protection of critical sections.
 *
 *  This contains the functions for entering and exiting critical sections.
 *
 *  @author PMcL
 *  @date 2020-03-03
 */

/*!
 *  @addtogroup Critical_module Critical module documentation
 *  @{
*/

// Critical sections
#include "Critical\critical.h"

// Globals

volatile uint8_t SR_reg;  // Current value of the FAULTMASK register
volatile uint8_t SR_lock = 0x00U;      // Lock

/*!
 * @}
*/
