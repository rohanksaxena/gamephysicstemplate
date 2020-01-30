
#ifndef RIGIDBODYSYSTEM_h
#define RIGIDBODYSYSTEM_h
#include "Simulator.h"

#define CUBE 0
#define SPHERE 1

class RigidBodySystem {
public:
	// Construtors
	
	RigidBodySystem();
	void init(Vec3 position, Vec3 size, float mass, int type);
	float mass;
	Vec3 comPosition;
	Vec3 comVelocity;
	Mat4 intertiaTensorInverse;
	Vec3 angularMomentum;
	Vec3 angularVelocity;
	Quat orientation;
	Vec3 size;
	Mat4 objToWorldMat;
	Vec3 forces;
	Vec3 torque;
	int type;
	//functions
	void precalculateIntertiaInverse();
	Vec3 worldToObj(Vec3 position);
	Mat4 calculateObjToWorldMatrix();
	bool containsPoint(Vec3 position);
private:

};
#endif