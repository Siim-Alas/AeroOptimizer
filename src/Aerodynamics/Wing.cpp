
#include "Wing.h"

inline double AeroOptimizer::Aerodynamics::Wing::CalculateOswaldsSpanEfficiency(double ar)
{
	// This uses Raymer's estimation for straight wings: e_oswald = 1.78(1 - 0.045*AR)^0.68 - 0.64
	return 1.78 * std::pow(1 - (0.045 * ar), 0.68) - 0.64;
}

double AeroOptimizer::Aerodynamics::Wing::CD(double a)
{
	// Using the corrected drag model, CD = CDmin + k * (CL - CLminD)^2
	return CDmin + k * (CL(a) - CLminD)*(CL(a) - CLminD);
}

double AeroOptimizer::Aerodynamics::Wing::CDa(double a)
{
	// With the corrected drag model, CD = CDmin + k * (CL - CLminD)^2 =
	// = CDmin + k * (CL0 + a*CLa - CLminD)^2, so its derivative with respect to a is
	// CDa = 2CLa * k * (CL0 + a*CLa - CLminD)
	return 2 * CLa * k * (a * CLa + CL0 - CLminD);
}

double AeroOptimizer::Aerodynamics::Wing::CL(double a)
{
	return CL0 + a * CLa;
}

AeroOptimizer::Aerodynamics::Wing::Wing(const Aerofoil &aerofoil, double aoi, double ar, double s)
{
	AOI = aoi;
	AR = ar;
	CDmin = aerofoil.Cdmin;
	CL0 = aerofoil.Cl0;
	CLa = aerofoil.Cla;
	CLminD = aerofoil.ClminD;
	e_oswald = CalculateOswaldsSpanEfficiency(AR);
	k = 1 / (M_PI * AR * e_oswald);
	S = s;
}

AeroOptimizer::Aerodynamics::Wing::~Wing()
{
}
