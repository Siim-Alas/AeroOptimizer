
#include "Aircraft.h"

double AeroOptimizer::Aerodynamics::Aircraft::_centreOfMass[] = { 0, 0, 0 };
double AeroOptimizer::Aerodynamics::Aircraft::_CMa = 0;
double AeroOptimizer::Aerodynamics::Aircraft::_cRef = 0;
double AeroOptimizer::Aerodynamics::Aircraft::_mass = 0;
std::vector<AeroOptimizer::Aerodynamics::PointMass> AeroOptimizer::Aerodynamics::Aircraft::_pointMasses;
double AeroOptimizer::Aerodynamics::Aircraft::_SRef = 0;
std::vector<AeroOptimizer::Aerodynamics::Wing> AeroOptimizer::Aerodynamics::Aircraft::_wings;

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

void AeroOptimizer::Aerodynamics::Aircraft::Init(double cRef, double requiredCMa, double SRef)
{
	_CMa = requiredCMa;
	_cRef = cRef;
	_SRef = SRef;
}

void AeroOptimizer::Aerodynamics::Aircraft::Dispose()
{
	// delete[] _centreOfMass;
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
	_wings[1].r[0] = _wings[0].r[0] - distanceBetweenForeAndAftWing;
	CalculateCentreOfMass();

	// dM / da normalized, by dividing by 0.5 * rho * V^2 * SRef
	double normalizeddMda = _CMa * _cRef;
	double* riMinusCOM = new double[3]{ 0, 0, 0 };
	double* dFda = new double[3]{ 0, 0, 0 };
	double* tempBuf = new double[3]{ 0, 0, 0, };
	double* sumBuf = new double[3]{ 0, 0, 0 };

	double a = 0;

	for (int i = 0; i < _wings.size(); i++)
	{
		AeroOptimizer::LinearAlgebra::SubtractVectors(_wings[i].r, _centreOfMass, riMinusCOM, 3);
		// dF / da = (Cla * cos(a) + Cda * sin(a))(-k) + (Cla * sin(a) + Cda * cos(a))(-i)
		dFda[0] = -1 * (_wings[i].CLa * std::sin(a) + _wings[i].CDa(a) * std::cos(a));
		dFda[2] = -1 * (_wings[i].CLa * std::cos(a) + _wings[i].CDa(a) * std::sin(a));
		AeroOptimizer::LinearAlgebra::CrossProduct(riMinusCOM, dFda, tempBuf);
		AeroOptimizer::LinearAlgebra::AddVectors(sumBuf, tempBuf, sumBuf, 3);

		// This assumes that each consecutive member of wings is behind and thus affected previous one.
		// (General Aviation Design: Applied Methods and Procedures by Snorry Gudmundsson, pg. 467)
		a -= 2 * (a * _wings[i].CLa + _wings[i].CL0) / (M_PI * _wings[i].AR);
	}

	double result = normalizeddMda - sumBuf[1];

	delete[] riMinusCOM;
	delete[] dFda;
	delete[] tempBuf;
	delete[] sumBuf;

	return result;
}

double AeroOptimizer::Aerodynamics::Aircraft::CMaEquationRHSDerivativesReciprocal(double distanceBetweenForeAndAftWing)
{
	const double dx = 0.000001;
	return dx / (CMaEquationRHS(distanceBetweenForeAndAftWing + dx) - CMaEquationRHS(distanceBetweenForeAndAftWing));
}
