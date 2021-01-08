// AeroOptimizer.cpp : Defines the entry point for the application.
//

#include "AeroOptimizer.h"

// f = dM/da - sum_1_to_m((r_i - dr_i) x dF / da)
void f(double* x, double* output, int n) 
{

}

// The inverse of the Jacobian of f
void jInverse(double* x, double* M, int n)
{

}

int main()
{
	const int n = 10;
	double x[n];

	AeroOptimizer::NewtonsMethod::FindRoot(f, jInverse, x, n, 100);

	std::cout << "Hello!" << std::endl;
	return 0;
}
