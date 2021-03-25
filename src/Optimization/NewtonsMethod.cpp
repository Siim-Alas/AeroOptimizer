
#include "NewtonsMethod.h"

double AeroOptimizer::Optimization::NewtonsMethod::FindRoot1D(
	ScalarToScalarFunction fOverItsDerivative, 
	double initialGuess, 
	int iterations)
{
	double x = initialGuess;
	for (int i = 0; i < iterations; i++)
	{
		x -= fOverItsDerivative(x);
	}
	return x;
}

double AeroOptimizer::Optimization::NewtonsMethod::FindRoot1D(
	ScalarToScalarFunction f, 
	ScalarToScalarFunction reciprocalDerivative,
	double initialGuess, 
	int iterations)
{
	double x = initialGuess;
	for (int i = 0; i < iterations; i++)
	{
		x -= reciprocalDerivative(x) * f(x);
	}
	return x;
}

template <int n>
void AeroOptimizer::Optimization::NewtonsMethod::FindRootND(
	VectorToVectorFunction jInverseMultF, 
	double* x,
	int iterations)
{
	double rBuf[n];
	for (int i = 0; i < iterations; i++)
	{
		jInverseMultF(x, rBuf, n);

		// x_(n + 1) = x_n - (J^-1(x_n) * F(x_n))
		LinearAlgebra::SubtractVectors(x, rBuf, x, n);
	}
}

template <int n>
void AeroOptimizer::Optimization::NewtonsMethod::FindRootND(
	VectorToVectorFunction f, 
	VectorToMatrixFunction jInverse, 
	double* x, 
	int iterations)
{
	double fBuf[n];
	double jInvBuf[n * n];
	double rBuf[n];

	for (int i = 0; i < iterations; i++)
	{
		f(x, fBuf, n);
		jInverse(x, jInvBuf, n);

		// x_(n + 1) = x_n - (J^-1(x_n) * F(x_n))
		LinearAlgebra::MatrixVectorMult(jInvBuf, fBuf, rBuf, n);
		LinearAlgebra::SubtractVectors(x, rBuf, x, n);
	}
}
