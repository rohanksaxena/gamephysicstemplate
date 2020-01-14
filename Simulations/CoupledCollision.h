#ifndef COUPLEDCOLLISION_h
#define COUPLEDCOLLISION_h

#include "RigidBodySystem.h"
#include "MassSpringSystemSimulator.h"

class CoupledCollision
{
public:
	CoupledCollision();
	~CoupledCollision();
	static bool checkCollision(RigidBodySystem body, MassSpringSystemSimulator* masspointSystem);
private:
	static int* searchArea(float dPosX, float dPosZ, float radius, int* areaSize);
};
#endif

