
#include "Aircraft.h"

void AeroOptimizer::Aerodynamics::Aircraft::CalculateCentreOfMass()
{
	_mass = 0;
	for (int i = 0; i < 3; i++)
	{
		_centreOfMass[i] = 0;
	}
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
		for (int j = 0; j < 3; j++)
		{
			_centreOfMass[j] += _wings[i].mass * _wings[i].r[j];
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

void AeroOptimizer::Aerodynamics::Aircraft::NudgeCentreOfMassAndMass(PointMass* pointMass)
{
	for (int i = 0; i < 3; i++)
	{
		// COM = ((r_0*m_0 + r_0*m_1 + ... + (pm.r*pm.mass)) / _mass) * (_mass / (_mass + mass)) =
		// = (r_0*m_0 + r_0*m_1 + ... + (pm.r*pm.mass)) / (_mass + mass)
		// Where pm is the point mass to be added
		_centreOfMass[i] += ((pointMass->r[i] * pointMass->mass) / _mass);
		_centreOfMass[i] *= (_mass / (_mass + pointMass->mass));
	}
	_mass += pointMass->mass;
}

AeroOptimizer::Aerodynamics::Aircraft::Aircraft(double cRef, double requiredCMa, double SRef)
{
	_centreOfMass[0] = 0;
	_centreOfMass[1] = 0;
	_centreOfMass[2] = 0;
	_CMa = requiredCMa;
	_cRef = cRef;
	_mass = 0;
	_SRef = SRef;
}

AeroOptimizer::Aerodynamics::Aircraft::~Aircraft()
{
	delete[] _centreOfMass;
}

void AeroOptimizer::Aerodynamics::Aircraft::AddPointMass(PointMass* pointMass)
{
	_pointMasses.push_back(*pointMass);
	NudgeCentreOfMassAndMass(pointMass);
}

void AeroOptimizer::Aerodynamics::Aircraft::AddWing(Wing* wing)
{
	_wings.push_back(*wing);
	NudgeCentreOfMassAndMass(wing);
}

double AeroOptimizer::Aerodynamics::Aircraft::CMaEquationRHS(double distanceBetweenForeAndAftWing)
{
	// dM / da normalized, by dividing by 0.5 * rho * V^2 * SRef
	double normalizeddMda = _CMa * _cRef;
	double* riMinusCOM = new double[3];
	double* dFda = new double[3];

	const double a = 0;

	// dF / da = (Cla * cos(a) + Cda * sin(a))(-k) + (Cla * sin(a) + Cda * cos(a))(-i)
	// First wing
	AeroOptimizer::LinearAlgebra::SubtractVectors(_wings[0].r, _centreOfMass, riMinusCOM, 3);
	dFda[0] = -1 * (_wings[0].CLa * std::sin(a) + _wings[0].CDa(a) * std::cos(a));
	dFda[2] = -1 * (_wings[0].CLa * std::cos(a) + _wings[0].CDa(a) * std::sin(a));

	delete[] riMinusCOM;
	delete[] dFda;
	return 0.0;
}
