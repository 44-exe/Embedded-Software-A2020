/*! @file FPC.h
 *
 *  @brief For efficient calculating without using decimal points.
 *
 *  This contains the functions for fixed-point calculation.
 *
 *  @author Jeong Bin Lee
 *  @date 2020-06-20
 */

#ifndef FPC_FPC_H_
#define FPC_FPC_H_

// new types
#include "Types\types.h"

#define SCALE_FACTOR 8 /*!< to scale up binary numbers exponentially */


/*! @brief Convert a fixed point number to a floating point number.
 *
 *  @param num - Floating point number used for calculation.
 *  @return double - Resulting number from fixed point to floating point.
 */
double FixedtoFloat(int32_t num);

/*! @brief Convert a floating point point number to a fixed point number.
 *
 *  @param num - Floating point number used for calculation.
 *  @return int64 - Resulting number from floating point to fixed point.
 */
int32_t FloattoFixed(double num);

/*! @brief Multiplies two floating point point numbers using fixed point calculation.
 *
 *  @param num1 - Floating point number used for calculation.
 *  @param num2 - Floating point number used for calculation.
 *  @return double - Result of fixed-point calculation.
 */
double FPC_Multiplication(double num1, double num2);

/*! @brief Divide two floating point numbers using fixed point calculation.
 *
 *  @param num1 - Floating point number used for calculation.
 *  @param num2 - Floating point number used for calculation.
 *  @return double - Result of fixed-point calculation.
 */
double FPC_Division(double num1, double num2);

/*! @brief Add two floating point numbers using fixed point calculation.
 *
 *  @param num1 - Floating point number used for calculation.
 *  @param num2 - Floating point number used for calculation.
 *  @return double - Result of fixed-point calculation.
 */
double FPC_Addition(double num1, double num2);

/*! @brief Subtract two floating point numbers using fixed point calculation.
 *
 *  @param num1 - Floating point number used for calculation.
 *  @param num2 - Floating point number used for calculation.
 *  @return double - Result of fixed-point calculation.
 */
double FPC_Subtraction(double num1, double num2);

/*! @brief Square root a floating point number using fixed point calculation.
 *
 *  @param num - Floating point number used for calculation.
 *  @return double - Result of fixed-point calculation.
 */
//double FPC_Sqrt(double num);

/*! @brief Calculates and returns Vrms using fixed point calculation method
 *
 *  @param num - Sampled and converted analog voltage in an array
 *  @param numSample - Size of the sampled voltage in the array.
 *  @return double - Result of fixed-point calculation.
 */
double FPC_Vrms(double *num, uint8_t numSample);


#endif /* FPC_FPC_H_ */
