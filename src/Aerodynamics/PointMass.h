
#pragma once

namespace AeroOptimizer
{
	namespace Aerodynamics
	{
		class PointMass
		{
		public:
			/*
			* The mass (units: kg).
			*/
			double mass = 0;
			/*
			* The 3D position vector (in the cartesian x, y, z coordinates) of the location of the mass
			* (units: m).
			*/
			double r[3]{ 0, 0, 0 };
		};
	}
}