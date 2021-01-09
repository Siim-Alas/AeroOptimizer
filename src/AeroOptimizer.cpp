// AeroOptimizer.cpp : Defines the entry point for the application.

#include "AeroOptimizer.h"

// f = dM/da - sum_1_to_m((r_i - dr_i) x dF / da)
void f(double* x, double* output, int n) 
{
	// test function f(x) = x^2
	output[0] = x[0] * x[0];
}

// The inverse of the Jacobian of f
void jInverse(double* x, double* M, int n)
{
	M[0, 0] = 1 / (2 * x[0]);
}

int main()
{
	/*
	const int n = 1;
	double x[n]
	{ 
		-15 
	};

	AeroOptimizer::Optimization::NewtonsMethod::FindRootND(&f, &jInverse, x, n, 100);
	*/
	double y = AeroOptimizer::Optimization::NewtonsMethod::FindRoot1D(
		[](double x) -> double { return 0.5 * x; },
		15, 
		10);
	std::cout << y << std::endl;
	return 0;
}
