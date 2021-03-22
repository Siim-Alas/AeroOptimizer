// AeroOptimizer.cpp : Defines the entry point for the application.

#include "AeroOptimizer.h"

int main()
{
	double requiredCma = -1.2;
	double cref = 0.2;
	double sref = 0.31;

	AeroOptimizer::Aerodynamics::Aerofoil* naca2421 = 
		new AeroOptimizer::Aerodynamics::Aerofoil(0.02, 0.25, 0.5 / (5 * M_PI / 180), 0.25);
	AeroOptimizer::Aerodynamics::Aircraft::Init(cref, requiredCma, sref);

	AeroOptimizer::Aerodynamics::Wing* foreWing =
		new AeroOptimizer::Aerodynamics::Wing(naca2421, 2 * M_PI / 180, 8, 0.13);
	foreWing->mass = 0.7;
	AeroOptimizer::Aerodynamics::Aircraft::AddWing(foreWing);

	AeroOptimizer::Aerodynamics::Wing* aftWing =
		new AeroOptimizer::Aerodynamics::Wing(naca2421, 0.75 * M_PI / 180, 8, 0.18);
	aftWing->mass = 0.5;
	aftWing->r[2] = 0.13;
	AeroOptimizer::Aerodynamics::Aircraft::AddWing(aftWing);

	double startingDistance = 0.35;
	double optimizedDistance = AeroOptimizer::Optimization::NewtonsMethod::FindRoot1D(
		&AeroOptimizer::Aerodynamics::Aircraft::CMaEquationRHS,
		&AeroOptimizer::Aerodynamics::Aircraft::CMaEquationRHSDerivativesReciprocal,
		startingDistance, 10);
	
	std::cout << optimizedDistance << std::endl;

	delete naca2421;
	AeroOptimizer::Aerodynamics::Aircraft::Dispose();
	delete foreWing;
	delete aftWing;

	return 0;
}
