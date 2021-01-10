
#include "Aerofoil.h"

AeroOptimizer::Aerodynamics::Aerofoil::Aerofoil(double cd0, double cl0, double cla, double clmind)
{
	Cd0 = cd0;
	Cl0 = cl0;
	Cla = cla;
	ClminD = clmind;
}
