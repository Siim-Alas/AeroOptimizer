
#include "LinearAlgebra.h"

void AeroOptimizer::LinearAlgebra::AddVectors(double* left, double* right, double* result, int n)
{
	for (int i = 0; i < n; i++)
	{
		result[i] = left[i] + right[i];
	}
}

double AeroOptimizer::LinearAlgebra::DotProduct(double* v1, double* v2, int n)
{
	double dotProd = 0;
	for (int i = 0; i < n; i++)
	{
		dotProd += v1[i] * v2[i];
	}
	return dotProd;
}

void AeroOptimizer::LinearAlgebra::MatrixVectorMult(double* M, double* v, double* result, int n)
{
	double sum;
	for (int i = 0; i < n; i++)
	{
		sum = 0;
		for (int j = 0; j < n; j++)
		{
			sum += M[i, j] * v[j];
		}
		result[i] = sum;
	}
}

void AeroOptimizer::LinearAlgebra::SubtractVectors(double* left, double* right, double* result, int n)
{
	for (int i = 0; i < n; i++)
	{
		result[i] = left[i] - right[i];
	}
}
