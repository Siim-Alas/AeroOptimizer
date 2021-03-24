
#include "Aircraft.h"

double AeroOptimizer::Aerodynamics::Aircraft::_centreOfMass[3] = { 0, 0, 0 };
double AeroOptimizer::Aerodynamics::Aircraft::_CMa = 0;
double AeroOptimizer::Aerodynamics::Aircraft::_cRef = 0;
double AeroOptimizer::Aerodynamics::Aircraft::_mass = 0;
std::vector<AeroOptimizer::Aerodynamics::PointMass> AeroOptimizer::Aerodynamics::Aircraft::_pointMasses;
double AeroOptimizer::Aerodynamics::Aircraft::_SRef = 0;
std::vector<AeroOptimizer::Aerodynamics::Wing> AeroOptimizer::Aerodynamics::Aircraft::_wings;

void AeroOptimizer::Aerodynamics::Aircraft::CalculateCentreOfMass()
{
	_mass = 0;

	_centreOfMass[0] = 0;
	_centreOfMass[1] = 0;
	_centreOfMass[2] = 0;

	for (int i = 0; i < _pointMasses.size(); i++)
	{
		_mass += _pointMasses[i].mass;
		for (int j = 0; j < 3; j++)
		{
			_centreOfMass[j] += _pointMasses[i].mass * _pointMasses[i].r[j];
		}
	}
	for (int i = 0; i < _wings.size(); i++)
	{
		_mass += _wings[i].mass;
		double wingCOM[3];
		for (int j = 0; j < 3; j++)
		{
			AeroOptimizer::LinearAlgebra::AddVectors(_wings[i].r, _wings[i].massOffset, wingCOM, 3);
			_centreOfMass[j] += _wings[i].mass * wingCOM[j];
		}
	}
	if (_mass == 0)
	{
		return;
	}
	for (int i = 0; i < 3; i++)
	{
		_centreOfMass[i] /= _mass;
	}
}

void AeroOptimizer::Aerodynamics::Aircraft::NudgeCentreOfMassAndMass(const double* r, double mass)
{
	for (int i = 0; i < 3; i++)
	{
		// COM = ((r_0*m_0 + r_0*m_1 + ... + (pm.r*pm.mass)) / _mass) * (_mass / (_mass + mass)) =
		// = (r_0*m_0 + r_0*m_1 + ... + (pm.r*pm.mass)) / (_mass + mass)
		// Where pm is the point mass to be added
		_centreOfMass[i] += ((r[i] * mass) / _mass);
		_centreOfMass[i] *= (_mass / (_mass + mass));
	}
	_mass += mass;
}

void AeroOptimizer::Aerodynamics::Aircraft::Init(double cRef, double requiredCMa, double SRef)
{
	_CMa = requiredCMa;
	_cRef = cRef;
	_SRef = SRef;
}

void AeroOptimizer::Aerodynamics::Aircraft::AddPointMass(const PointMass &pointMass)
{
	_pointMasses.push_back(pointMass);
	NudgeCentreOfMassAndMass(pointMass.r, pointMass.mass);
}

void AeroOptimizer::Aerodynamics::Aircraft::AddWing(const Wing &wing)
{
	_wings.push_back(wing);
	double wingCOM[3];
	AeroOptimizer::LinearAlgebra::AddVectors(wing.r, wing.massOffset, wingCOM, 3);
	NudgeCentreOfMassAndMass(wingCOM, wing.mass);
}

double AeroOptimizer::Aerodynamics::Aircraft::CMaEquationRHS(double distanceBetweenForeAndAftWing)
{
	_wings[1].r[0] = _wings[0].r[0] - distanceBetweenForeAndAftWing;
	CalculateCentreOfMass();

	// dM / da normalized, by dividing by 0.5 * rho * V^2 * SRef
	double normalizeddMda = _CMa * _cRef;
	double riMinusCOM[3] = { 0, 0, 0 };
	double dFda[3] = { 0, 0, 0 };
	double tempBuf[3] = { 0, 0, 0 };
	double sumBuf[3] = { 0, 0, 0 };

	double a = 0;

	for (int i = 0; i < _wings.size(); i++)
	{
		AeroOptimizer::LinearAlgebra::SubtractVectors(_wings[i].r, _centreOfMass, riMinusCOM, 3);

		//         |cos(a)  sin(a)| |-CD(a)|
		// CF(a) = |              | |      | , from which
		//         |-sin(a) cos(a)| |-CL(a)|

		//			  |-sin(a)  cos(a)| |-CD(a)|   |cos(a)  sin(a)| |-dCD / da|
		// dCF / da = |               | |      | + |              | |         |
		//            |-cos(a) -sin(a)| |-CL(a)|   |-sin(a) cos(a)| |-dCL / da|

		dFda[0] = std::sin(a) * _wings[i].CD(a) - std::cos(a) * _wings[i].CL(a) \
			- std::cos(a) * _wings[i].CDa(a) + std::sin(a) * _wings[i].CLa;

		dFda[2] = std::cos(a) * _wings[i].CD(a) + std::sin(a) * _wings[i].CL(a) \
			+ std::sin(a) * _wings[i].CDa(a) - std::cos(a) * _wings[i].CLa;

		AeroOptimizer::LinearAlgebra::CrossProduct(riMinusCOM, dFda, tempBuf);
		AeroOptimizer::LinearAlgebra::AddVectors(sumBuf, tempBuf, sumBuf, 3);

		// This assumes that each consecutive member of wings is behind and thus affected previous one.
		// (General Aviation Design: Applied Methods and Procedures by Snorry Gudmundsson, pg. 467)
		a -= 2 * (a * _wings[i].CLa + _wings[i].CL0) / (M_PI * _wings[i].AR);
	}

	double result = normalizeddMda - sumBuf[1];
	return result;
}

double AeroOptimizer::Aerodynamics::Aircraft::CMaEquationRHSDerivativesReciprocal(double distanceBetweenForeAndAftWing)
{
	const double dx = 0.000001;
	return dx / (CMaEquationRHS(distanceBetweenForeAndAftWing + dx) - CMaEquationRHS(distanceBetweenForeAndAftWing));
}
