
#pragma once

#include <malloc.h>
#include <stdlib.h>

#include "LinearAlgebra.h"

namespace AeroOptimizer
{
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

	class NewtonsMethod
	{
	public:
		/*
		* Finds a root of the function f with Newtons method.
		*
		* @param f, the function to optimize.
		* @param jInverse, the inverse Jacobian of the function to optimize.
		* @param x, the initial guess to use in the algorithm. This will get mutated
		* into the final guess as the algorithm iterates.
		* @param n, the number of dimensions of the system.
		* @param iterations, the number of iterations to perform with the algorithm.
		*/
		static void FindRoot(
			VectorToVectorFunction f,
			VectorToMatrixFunction jInverse,
			double* x,
			int n,
			int iterations);
	};
}
