#include "CoupledSimulation.h"



CoupledSimulation::CoupledSimulation()
{
	springSim = new MassSpringSystemSimulator(564001, 1689000);
	drawTrampoline();
}

void CoupledSimulation::drawTrampoline()
{
	
	int count = 0;
	for (float i = -0.375f; i < 0.375f; i += 0.001f) {
		for (float j = -0.375f; j < 0.375f; j += 0.001f) {
			springSim->addMassPoint(Vec3(i, -0.5f, j), Vec3(0, 0, 0), true);
			
		}
	}
	
	for (int i = 0; i < 563250; i++) {
		if (i % 750 == 0) {
			springSim->addSpring(i, i + 750, 0.001f);

			count++;
		}
		else if (i % 750 != 0) {
			springSim->addSpring(i, i + 1, 0.001f);
			springSim->addSpring(i, i + 750, 0.001f);
			springSim->addSpring(i, i + 751, sqrt(2) * 0.001f);

			count+=3;
		}
	}
	for (int i = 563250; i < 564001; i++) {
		springSim->addSpring(i, i + 1, 0.001f);
		count++;
	}
	cout << count;
	
	
}

void CoupledSimulation::drawLine(int point1, int point2) {
	DUC->beginLine();
	DUC->drawLine(springSim->masspoints[point1].position, Vec3(0, 255, 0), springSim->masspoints[point2].position, Vec3(0, 255, 0));
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
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(0, 0, 0), 1, Vec3(0, 1, 0));
	for (int i = 0; i < springSim->masspointsCounter; i++) {
		DUC->drawSphere(springSim->masspoints[i].position, Vec3(0.0005f, 0.0005f, 0.0005f));
	}
	
	for (int i = 0; i < 563250; i++) {
		drawLine(i, i+1);
		drawLine(i, i + 750);
		drawLine(i, i + 751);
	}
	
	for (int i = 563250; i < 564000; i++) {
		drawLine(i, i + 1);
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


