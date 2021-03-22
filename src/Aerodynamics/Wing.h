
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "Aerofoil.h"
#include "PointMass.h"

namespace AeroOptimizer
{
	namespace Aerodynamics
	{
		class Wing : public PointMass
		{
		public:
			double AOI;
			double AR;
			double CL0;
			double CLa;
			double CLminD;
			double e_oswald;
			double k;
			double S;

			static double CalculateOswaldsSpanEfficiency(double ar);
			double CDa(double a);
			Wing(const Aerofoil &aerofoil, double aoi, double ar, double s);
			~Wing();
		};
	}
}