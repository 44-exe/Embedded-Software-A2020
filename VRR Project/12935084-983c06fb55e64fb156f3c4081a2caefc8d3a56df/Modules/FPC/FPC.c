/*! @file ADC.c
 *
 *  @brief For efficient calculating without using decimal points.
 *
 *  This contains the functions for fixed-point calculation.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-06-15
 */

/*!
**  @addtogroup FPC_module FPC module documentation
**  @{
*/

#include "FPC.h"
#include <math.h>
#include <stdlib.h>

double FixedtoFloat(int32_t num)
{
  return ((double)num/(double)(1 << SCALE_FACTOR));
}

int32_t FloattoFixed(double num)
{
  return (int32_t)(round(num * (1 << SCALE_FACTOR)));
}

double FPC_Multiplication(double num1, double num2)
{
  return FixedtoFloat(FixedtoFloat(FloattoFixed(num1) * FloattoFixed(num2)));
}

double FPC_Division(double num1, double num2)
{
  return FixedtoFloat(FloattoFixed(num1 / num2));
}

double FPC_Addition(double num1, double num2)
{
  return FixedtoFloat(FloattoFixed(num1) + FloattoFixed(num2));
}

double FPC_Subtraction(double num1, double num2)
{
  return FixedtoFloat(FloattoFixed(num1) - FloattoFixed(num2));
}

//double FPC_Sqrt(double num)
//{
//  double sqrt;
//  double temp; //tmp variable to compare
//  uint8_t i = 0; //counter to stop it from inf loop
//  do
//  {
//    temp = sqrt;
//    sqrt = FPC_Division(FPC_Addition(FPC_Division(num, temp), temp), 2);
//    i++;
//  } while ((sqrt != temp) && (i < 30));
//  return sqrt;
//}

double FPC_Vrms(double *num, uint8_t numSample)
{
  uint8_t i;
  uint32_t volt[numSample], Vrms = 0;

  for (i=0; i<numSample - 1; i++)
  {
    //convert the number from float to fix
    volt[i] = (int32_t)(num[i] * (1 << SCALE_FACTOR));

    Vrms += volt[i] * volt[i];
  }
  return sqrt((((double)(Vrms)/(double)(1 << SCALE_FACTOR))/(double)(1 << SCALE_FACTOR))/numSample);
}

/*!
** @}
*/
