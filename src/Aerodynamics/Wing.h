
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "Aerofoil.h"

namespace AeroOptimizer
{
	namespace Aerodynamics
	{
		class Wing
		{
		public:
			double AOI = 0;
			double AR = 0;
			double CL0 = 0;
			double CLa = 0;
			double CLminD = 0;
			double e_oswald = 0;
			double k = 0;
			double mass = 0;
			double massOffset[3]{ 0, 0, 0 };
			double r[3]{ 0, 0, 0 };
			double S = 0;

			inline static double CalculateOswaldsSpanEfficiency(double ar);
			double CDa(double a);
			Wing(const Aerofoil &aerofoil, double aoi, double ar, double s);
			~Wing();
		};
	}
}