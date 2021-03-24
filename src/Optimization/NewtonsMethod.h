
#pragma once

#include <malloc.h>
#include <stdlib.h>

#include "../LinearAlgebra.h"
#include "../Typedefs.h"

namespace AeroOptimizer
{
    namespace Optimization
    {
		namespace NewtonsMethod
		{
			/*
			* Finds a root of the function f(x) with Newtons method.
			*
			* @param fOverItsDerivative, f / (df/dx) the function to optimize divided by its derivative.
			* @param initialGuess, the initial guess.
			* @param iterations, the number of iterations to perform with the algorithm.
			*
			* @return The final guess, arrived at after the specified number of iterations.
			*/
			double FindRoot1D(
				ScalarToScalarFunction fOverItsDerivative,
				double initialGuess,
				int iterations);
			/*
			* Finds a root of the function f(x) with Newtons method.
			*
			* @param f, the function to optimize.
			* @param reciprocalDerivative, a function to generate 1 / (df/dx).
			* @param initialGuess, the initial guess.
			* @param iterations, the number of iterations to perform with the algorithm.
			*
			* @return The final guess, arrived at after the specified number of iterations.
			*/
			double FindRoot1D(
				ScalarToScalarFunction f,
				ScalarToScalarFunction reciprocalDerivative,
				double initialGuess,
				int iterations);
			/*
			* Finds a root of the function f(x_1, x_2, ... , x_n) with Newtons method.
			* 
			* @param n, the number of dimensions of the system.
			* @param jInverseMultF, Jf the Jacobian multiplied by the function to optimize.
			* @param x, an array of length n representing the initial guess. This will get mutated into
			* the final guess as the algorithm iterates.
			* @param iterations, the number of iterations to perform with the algorithm.
			*/
			template <int n> 
			void FindRootND(
				VectorToVectorFunction jInverseMultF,
				double* x,
				int iterations);
			/*
			* Finds a root of the function f(x_1, x_2, ... , x_n) with Newtons method.
			*
			* @param f, the function to optimize.
			* @param jInverse, a function to generate the 2D array representing the n x n Jacobian matrix.
			* @param x, an array of length n representing the initial guess. This will get mutated into
			* the final guess as the algorithm iterates.
			* @param n, the number of dimensions of the system.
			* @param iterations, the number of iterations to perform with the algorithm.
			*/
			template <int n>
			void FindRootND(
				VectorToVectorFunction f,
				VectorToMatrixFunction jInverse,
				double* x,
				int iterations);
		};
    }
}
