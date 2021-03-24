
#pragma once

namespace AeroOptimizer
{
	/*
	* F: R -> R,
	*
	* A type representing scalar-valued single-variable functions.
	*
	* @param double, the input to the function.
	*
	* @param double, the output of the function.
	*/
	typedef double (*ScalarToScalarFunction)(double);
	/*
	* F: R^k -> R^2k,
	*
	* A type representing matrix-valued multivariable functions.
	*
	* @param double*, a pointer to the 1-dimensional array containing the argument vector.
	* @param double*, a pointer to the 2-dimensional array to which the output is written.
	* @param int, the length of the argument array.
	*/
	typedef void (*VectorToMatrixFunction)(double*, double*, int);
	/*
	* F: R^k -> R^k,
	*
	* A type representing vector-valued multivariable functions.
	*
	* @param double*, a pointer to the array containing the argument vector.
	* @param double*, a pointer to the array to which the output is written.
	* @param int, the length of the argument array.
	*/
	typedef void (*VectorToVectorFunction)(double*, double*, int);
}