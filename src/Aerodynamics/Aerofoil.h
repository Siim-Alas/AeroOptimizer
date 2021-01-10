
#pragma once

namespace AeroOptimizer
{
	namespace Aerodynamics
	{
		class Aerofoil
		{
		public:
			/*
			* The section drag coefficient at zero angle of attack (unitless).
			*/
			double Cd0;
			/*
			* The section lift coefficient at zero angle of attack (unitless).
			*/
			double Cl0;
			/*
			* The partial derivateve of the coefficient of lift with respect to the angle of attack
			* in the linear range of angles of attack (units: 1 / Rad).
			*/
			double Cla;
			/*
			* The partial derivateve of the coefficient of lift with respect to the angle of attack
			* in the linear range of angles of attack (units: 1 / Rad).
			*/
			double ClminD;
			Aerofoil(double cd0, double cl0, double cla, double clmind);
		};
	}
}