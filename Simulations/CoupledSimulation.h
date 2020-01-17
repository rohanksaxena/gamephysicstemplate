
#ifndef COUPLEDSIMULATION_h
#define COUPLEDSIMULATION_h
#include "Simulator.h"
#include "MassSpringSystemSimulator.h"
#include "RigidBodySystem.h"

class CoupledSimulation :public Simulator
{
public:
	CoupledSimulation();
	~CoupledSimulation();
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);


	MassSpringSystemSimulator* springSim;
	RigidBodySystem ball;
	float damping;
	Vec3 m_externalForce;
	int mpl; // masspoints per line
	float step;
	float border;
	void drawTrampoline();
	void drawLine(int point1, int point2);
	void integrateBall(float step);
	void handleCollision(int index);
};
#endif

