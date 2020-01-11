#include "CoupledSimulation.h"



CoupledSimulation::CoupledSimulation()
{
}


CoupledSimulation::~CoupledSimulation()
{
}

const char * CoupledSimulation::getTestCasesStr()
{
	return nullptr;
}

void CoupledSimulation::initUI(DrawingUtilitiesClass * DUC)
{
}

void CoupledSimulation::reset()
{
}

void CoupledSimulation::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
}

void CoupledSimulation::notifyCaseChanged(int testCase)
{
}

void CoupledSimulation::externalForcesCalculations(float timeElapsed)
{
}

void CoupledSimulation::simulateTimestep(float timeStep)
{
}

void CoupledSimulation::onClick(int x, int y)
{
}

void CoupledSimulation::onMouse(int x, int y)
{
}
