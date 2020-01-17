#ifndef COUPLEDCOLLISION_h
#define COUPLEDCOLLISION_h

#include "RigidBodySystem.h"
#include "MassSpringSystemSimulator.h"

class CoupledCollision
{
public:
	CoupledCollision();
	~CoupledCollision();
	static int checkCollision(RigidBodySystem body, MassSpringSystemSimulator* masspointSystem, float border, float step, int mpl);
private:
	static int* searchArea(float dPosX, float dPosZ, float radius, int* areaSize, float border, float step, int mpl);
};
#endif

