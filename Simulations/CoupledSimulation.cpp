#include "CoupledSimulation.h"



CoupledSimulation::CoupledSimulation()
{
	springSim = new MassSpringSystemSimulator(5776, 25000);
	drawTrampoline();
}

void CoupledSimulation::drawTrampoline()
{
	int count = 0;
	cout << springSim->masspointsCounter;
	for (float i = -0.375f; i < 0.375f; i += 0.01f) {
		for (float j = -0.375f; j < 0.375f; j += 0.01f) {
			springSim->addMassPoint(Vec3(i, -0.5f, j), Vec3(0, 0, 0), true);
			
		}
	}
	
	for (int i = 0; i < 5700; i++) {
		springSim->addSpring(i, i + 1, 0.01f);
		drawLine(i, i+1);
		springSim->addSpring(i, i + springSim->masspointsCounter, 0.01f);
		drawLine(i, i + springSim->masspointsCounter);
		springSim->addSpring(i, i + springSim->masspointsCounter + 1, sqrt(2) * 0.01f);
		drawLine(i, i + springSim->masspointsCounter + 1);
		cout << i << " ";
	}
	/*
	for (int i = 5700; i < 5775; i++) {
		springSim->addSpring(i, i + 1, 0.01f);
		drawLine(i, i + 1);
		cout << i << " ";
	}
	*/
	
}

void CoupledSimulation::drawLine(int point1, int point2) {
	this->DUC->beginLine();
	//DUC->drawLine(springSim->masspoints[point1].position, Vec3(0, 255, 0), springSim->masspoints[point2].position, Vec3(0, 255, 0));
	DUC->endLine();
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
	this->DUC = DUC;

}

void CoupledSimulation::reset()
{
}

void CoupledSimulation::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	for (int i = 0; i < springSim->masspointsCounter; i++) {
		DUC->drawSphere(springSim->masspoints[i].position, Vec3(0.005f, 0.005f, 0.005f));
	}
	
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


