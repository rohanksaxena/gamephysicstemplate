#include "CoupledSimulation.h"
#include "CoupledCollision.h"


CoupledSimulation::CoupledSimulation()
{
	mpl = 0;
	step = 0.05f;
	border = 0.375f;
	springSim = new MassSpringSystemSimulator(5776, 17024);
	drawTrampoline();
	ball.init(Vec3(0, 0, 0), Vec3(0.1f, 0.1f, 0.1f), 100, SPHERE);
	damping = 0;
	m_externalForce = Vec3(0, -10, 0);
	
}

void CoupledSimulation::drawTrampoline()
{
	for (float i = -border; i < border; i += step) {
		mpl++;
		for (float j = -border; j < border; j += step) {
			springSim->addMassPoint(Vec3(i, -0.5f, j), Vec3(0, 0, 0), true);
		}
	}

	for (int i = 0; i < (springSim->masspointsCounter - mpl); i++) {
		if (i % (mpl - 1) == 0) {
			springSim->addSpring(i, i + (mpl - 1), step);
		}
		else if (i % (mpl - 1) != 0) {
			springSim->addSpring(i, i + 1, step);
			springSim->addSpring(i, i + (mpl - 1), step);
			springSim->addSpring(i, i + mpl, sqrt(2) * step);
		}
	}

	for (int i = (springSim->masspointsCounter - mpl); i < springSim->masspointsCounter - 1; i++) {
		springSim->addSpring(i, i + 1, step);
	}
	cout << springSim->masspointsCounter << " " << mpl << " " << springSim->springsCounter;
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
		DUC->drawSphere(springSim->masspoints[i].position, Vec3(0.005f, 0.005f, 0.005f));
	}

	for (int i = 0; i < (springSim->masspointsCounter - mpl); i++) {
		drawLine(i, i + 1);
		drawLine(i, i + (mpl - 1));
		drawLine(i, i + mpl);
	}

	for (int i = (springSim->masspointsCounter - mpl); i < springSim->masspointsCounter - 1; i++) {
		drawLine(i, i + 1);
	}
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(0, 0, 0), 1, Vec3(1, 1, 1));
	DUC->drawSphere(ball.comPosition, ball.size);
}

void CoupledSimulation::drawLine(int point1, int point2) {
	DUC->beginLine();
	DUC->drawLine(springSim->masspoints[point1].position, Vec3(0, 255, 0), springSim->masspoints[point2].position, Vec3(0, 255, 0));
	DUC->endLine();
}

void CoupledSimulation::notifyCaseChanged(int testCase)
{
}

void CoupledSimulation::externalForcesCalculations(float timeElapsed)
{
}

void CoupledSimulation::simulateTimestep(float timeStep)
{

	if (CoupledCollision::checkCollision(ball, springSim, border, step, mpl)) {
	}
	else {
		integrateBall(timeStep);
	}
}

void CoupledSimulation::integrateBall(float step)
{
	ball.comPosition += step * ball.comVelocity;
	ball.comVelocity += (step * (((ball.forces - (damping * ball.comVelocity)) / ball.mass) + m_externalForce));

	Quat temp = Quat(ball.angularVelocity.x, ball.angularVelocity.y, ball.angularVelocity.z, 0) * ball.orientation;

	ball.orientation += (step * 0.5f) * temp;
	float norm = ball.orientation.norm();
	ball.orientation = (1 / norm) * ball.orientation;

	ball.angularMomentum += step * (ball.torque - (0.5f * ball.angularMomentum));
	Mat4 transposeRotation = ball.orientation.getRotMat();
	transposeRotation.transpose();
	ball.intertiaTensorInverse = ball.orientation.getRotMat() * ball.intertiaTensorInverse * transposeRotation;
	ball.angularVelocity = ball.intertiaTensorInverse * ball.angularMomentum;

	ball.forces = Vec3(0, 0, 0);
	ball.torque = Vec3(0, 0, 0);
}

void CoupledSimulation::onClick(int x, int y)
{
}

void CoupledSimulation::onMouse(int x, int y)
{
}


