
#ifndef COUPLEDSIMULATION_h
#define COUPLEDSIMULATION_h
#include "Simulator.h"
#include "MassSpringSystemSimulator.h"
#include "RigidBodySystem.h"
#include "DiffusionSimulator.h"

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
	DiffusionSimulator* diffusionSim;
	float damping;
	Vec3 m_externalForce;
	int mpl; // masspoints per line
	float step;
	float border;
	float mpSize;
	void drawTrampoline();
	void drawLine(int point1, int point2);
	void integrateBall(float step);
	void handleCollision(int index);
	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif

