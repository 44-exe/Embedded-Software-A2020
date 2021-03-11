/*! @file
 *
 *  @brief Routines to implement protection of critical sections.
 *
 *  This contains the functions for entering and exiting critical sections.
 *
 *  @author PMcL
 *  @date 2020-03-03
 */

#ifndef CRITICAL_H
#define CRITICAL_H

#include <stdint.h>

extern volatile uint8_t SR_reg;  // Current value of the FAULTMASK register
extern volatile uint8_t SR_lock; // Lock

// Save status register and disable interrupts
#define EnterCritical() \
 do {\
  uint8_t SR_reg_local;\
   __asm ( \
     "MRS R0, FAULTMASK\n\t" \
     "CPSID f\n\t"            \
     "STRB R0, %[output]"  \
     : [output] "=m" (SR_reg_local)\
     :: "r0");\
   if (++SR_lock == 1u) {\
     SR_reg = SR_reg_local;\
   }\
 } while(0)

 
// Restore status register
#define ExitCritical() \
 do {\
   if (--SR_lock == 0u) { \
     __asm (                 \
       "LDRB R0, %[input]\n\t"\
       "MSR FAULTMASK, R0;\n\t" \
       ::[input] "m" (SR_reg)  \
       : "r0");                \
   }\
 } while(0)

#endif
