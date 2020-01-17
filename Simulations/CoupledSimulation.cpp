#include "CoupledSimulation.h"
#include "CoupledCollision.h"


CoupledSimulation::CoupledSimulation()
{
	mpl = 0;
	step = 0.05f;
	border = 0.375f;
	springSim = new MassSpringSystemSimulator(256, 705);
	springSim->setMass(5);
	springSim->setStiffness(50000);

	drawTrampoline();
	ball.init(Vec3(0, 0, 0), Vec3(0.1f, 0.1f, 0.1f), 1, SPHERE);
	damping = 0;
	m_externalForce = Vec3(0, -50, 0);

}

void CoupledSimulation::drawTrampoline()
{
	for (float i = -border; i <= border + step / 2; i += step) { // +step/2 for correction
		mpl++;
		for (float j = -border; j <= border + step / 2; j += step) {
			springSim->addMassPoint(Vec3(i, -0.5f, j), Vec3(0, 0, 0), false);
		}
	}
	//Fix borders
	for (int i = 0; i < springSim->masspointsCounter; i++) {
		if (i < mpl || i > pow(mpl, 2) - mpl || (i + 1) % mpl == 0 || i % mpl == 0) {
			springSim->masspoints[i].isFixed = true;
		}
	}

	for (int i = 0; i < (springSim->masspointsCounter - mpl); i++) {
		if ((i + 1) % mpl != 0) {
			springSim->addSpring(i, i + 1, step);
			springSim->addSpring(i, i + mpl + 1, sqrt(2) * step);

		}
		springSim->addSpring(i, i + mpl, step);
	}

	for (int i = (springSim->masspointsCounter - mpl); i < springSim->masspointsCounter - 1; i++) {
		springSim->addSpring(i, i + 1, step);
	}
	cout << springSim->masspointsCounter << " " << mpl << " " << springSim->springsCounter << "\n";
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
	DUC->setUpLighting(Vec3(0, 0, 0), Vec3(0, 0, 0), 1, Vec3(1, 1, 1));
	for (int i = 0; i < springSim->masspointsCounter; i++) {
		DUC->drawSphere(springSim->masspoints[i].position, Vec3(0.005f, 0.005f, 0.005f));
	}

	for (int i = 0; i < (springSim->masspointsCounter - mpl); i++) {
		if ((i + 1) % mpl != 0) {
			drawLine(i, i + 1);
			drawLine(i, i + mpl + 1);

		}
		drawLine(i, i + mpl);
	}

	for (int i = (springSim->masspointsCounter - mpl); i < springSim->masspointsCounter - 1; i++) {
		drawLine(i, i + 1);
	}

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
	int collIndex = CoupledCollision::checkCollision(ball, springSim, border, step, mpl);
	if (collIndex != -1) {
		handleCollision(collIndex);
	}
	
	integrateBall(timeStep); 

	for (int i = 1; i < springSim->getNumberOfSprings(); i++) {
		springSim->simulateLeapfrogStep(1, i, timeStep);
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

	//ball.forces = Vec3(0, 0, 0);
	//ball.torque = Vec3(0, 0, 0);
}

void CoupledSimulation::handleCollision(int index)
{
	Vec3 diff = ball.comPosition - springSim->masspoints[index].position;
	Vec3 normal = (diff) / sqrt(dot(diff, diff));
	Vec3 vrel = ball.comVelocity - springSim->masspoints[index].velocity;

	float normalVel = dot(normal, vrel);
	float c = 1.5f;
	if (normalVel < 0) {
		//calculating the inertia tensor inverse for the masspoint 
		float xx = 5.0f / (2.0f * springSim->m_fMass * pow(0.05f, 2));
		Mat4 mpi = Mat4(xx, 0, 0, 0,
			0, xx, 0, 0,
			0, 0, xx, 0,
			0, 0, 0, 1);//masspoint inertia tensor inverse

		float impulse = -(1 + c) * dot(vrel, normal) /
			((1 / ball.mass) +
			(1 / springSim->m_fMass) +
				dot(((cross(ball.intertiaTensorInverse * cross(Vec3(0,0,0), normal), Vec3(0, 0, 0))) +
				(cross(mpi * cross(Vec3(0, 0, 0), normal), Vec3(0, 0, 0)))), normal));
		//Ball
		ball.comVelocity.y += normal.y * impulse / ball.mass;


		ball.angularMomentum += (cross(Vec3(0, 0, 0), impulse * normal));

		//ObjB
		springSim->masspoints[index].velocity -= impulse * normal / springSim->m_fMass;
	}
}

void CoupledSimulation::onClick(int x, int y)
{
}

void CoupledSimulation::onMouse(int x, int y)
{
}


