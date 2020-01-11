
#ifndef COUPLEDSIMULATION_h
#define COUPLEDSIMULATION_h
#include "Simulator.h"

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
};
#endif

