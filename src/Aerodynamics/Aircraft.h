
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
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
			static double _centreOfMass[3];
			static double _CMa;
			static double _cRef;
			static double _mass;
			static std::vector<PointMass> _pointMasses;
			static double _SRef;
			static std::vector<Wing> _wings;

			static void CalculateCentreOfMass();
			static void NudgeCentreOfMassAndMass(const PointMass &pointMass);
		public:
			static void Init(double cRef,double requiredCMa, double SRef);
			static void AddPointMass(const PointMass &pointMass);
			static void AddWing(const Wing &wing);
			static double CMaEquationRHS(double distanceBetweenForeAndAftWing);
			static double CMaEquationRHSDerivativesReciprocal(double distanceBetweenForeAndAftWing);
			static void Dispose();
		};
	}
}