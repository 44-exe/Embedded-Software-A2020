/*! @file
 *
 *  @brief Median filter.
 *
 *  This contains the functions for performing a median filter on byte-sized data.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-05-13
 */

// New types
#include "Types\types.h"
#include "Median\median.h"


uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3)
{
  //check if n1 is the middle number
  if (((n2 <= n1) && (n1 <= n3)) || ((n3 <= n1) && (n1 <= n2)))
    return n1;
  //check if n2 is the middle number
  else if (((n1 <= n2) && (n2 <= n3)) || ((n2 <= n1) && (n3 <= n2)))
    return n2;
  else
    return n3;
}

