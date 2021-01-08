
#include "NewtonsMethod.h"

void AeroOptimizer::NewtonsMethod::FindRoot(
	AeroOptimizer::VectorToVectorFunction f,
	AeroOptimizer::VectorToMatrixFunction jInverse,
	double* x, 
	int n, 
	int iterations)
{
	double* fBuf = (double*)malloc(n * sizeof(double));
	double* jInvBuf = (double*)malloc((n * sizeof(double)) * (n * sizeof(double)));
	double* rBuf = (double*)malloc(n * sizeof(double));
	for (int i = 0; i < iterations; i++)
	{
		f(x, fBuf, n);
		jInverse(x, jInvBuf, n);

		// x_(n + 1) = x_n - (J^-1(x_n) * F(x_n))
		LinearAlgebra::MatrixVectorMult(jInvBuf, fBuf, rBuf, n);
		LinearAlgebra::SubtractVectors(x, rBuf, x, n);
	}
	free(fBuf);
	free(jInvBuf);
	free(rBuf);
}
