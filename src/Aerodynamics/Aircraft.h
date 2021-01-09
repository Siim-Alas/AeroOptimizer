
#pragma once

#include <vector>

#include "Wing.h"
#include "../LinearAlgebra.h"

namespace AeroOptimizer
{
	namespace Aerodynamics
	{
		class Aircraft
		{
		private:
			double _centreOfMass[3];
			double _CMa;
			double _cRef;
			double _mass;
			std::vector<PointMass> _pointMasses;
			double _SRef;
			std::vector<Wing> _wings;

			void CalculateCentreOfMass();
			void NudgeCentreOfMassAndMass(PointMass* pointMass);
		public:
			Aircraft(double cRef,double requiredCMa, double SRef);
			~Aircraft();
			void AddPointMass(PointMass* pointMass);
			void AddWing(Wing* wing);
			double CMaEquationRHS(double distanceBetweenForeAndAftWing);
		};
	}
}