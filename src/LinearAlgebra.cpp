
#include "LinearAlgebra.h"

void AeroOptimizer::LinearAlgebra::AddVectors(const double* left, const double* right, double* result, int n)
{
	for (int i = 0; i < n; i++)
	{
		result[i] = left[i] + right[i];
	}
}

void AeroOptimizer::LinearAlgebra::CrossProduct(const double* left, const double* right, double* result)
{
	result[0] = (left[1] * right[2]) - (left[2] * right[1]);
	result[1] = (left[2] * right[0]) - (left[0] * right[2]);
	result[2] = (left[0] * right[1]) - (left[1] * right[0]);
}

double AeroOptimizer::LinearAlgebra::DotProduct(const double* v1, const double* v2, int n)
{
	double dotProd = 0;
	for (int i = 0; i < n; i++)
	{
		dotProd += v1[i] * v2[i];
	}
	return dotProd;
}

void AeroOptimizer::LinearAlgebra::MatrixVectorMult(const double* M, const double* v, double* result, int n)
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

void AeroOptimizer::LinearAlgebra::SubtractVectors(const double* left, const double* right, double* result, int n)
{
	for (int i = 0; i < n; i++)
	{
		result[i] = left[i] - right[i];
	}
}
