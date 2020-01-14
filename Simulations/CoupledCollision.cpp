#include "CoupledCollision.h"



CoupledCollision::CoupledCollision()
{
}


CoupledCollision::~CoupledCollision()
{
}


bool CoupledCollision::checkCollision(RigidBodySystem body, MassSpringSystemSimulator * masspointSystem)
{
	int index = -1;
	float dPosX;
	float dPosZ;
	Vec3 bodyPosition = body.comPosition;

	if (fmod(bodyPosition.x, 0.05f) != 0) {
		dPosX = (bodyPosition.x + 0.05f) - (bodyPosition.x + 0.05f, 0.05f); // Descritization
	}
	else {
		dPosX = bodyPosition.x;
	}
	if(fmod(bodyPosition.z, 0.05f) != 0) {
		dPosZ = (bodyPosition.z + 0.05f) - (bodyPosition.z + 0.05f, 0.05f); // Descritization
	}
	else {
		dPosZ = bodyPosition.z;
	}

	int* areaSize = (int*) malloc(sizeof(int));
	*areaSize = (pow((int)(body.size.x * 2/ 0.05f) + 1, 2));
	int * indexes = searchArea(dPosX, dPosZ, body.size.x, areaSize);
	float mPosY;

	for (int i = 0; i < *areaSize; i++) {
		mPosY = masspointSystem->masspoints[indexes[i]].position.y;
		if ((bodyPosition.y - mPosY) <= body.size.x) {
			return true;
		}
	}
	
	return false;
}

int * CoupledCollision::searchArea(float dPosX, float dPosZ, float radius, int* areaSize)
{
	int* searchArea = new int[*areaSize];
	int k = 0;
	int index;
	for (float i = dPosX - radius; i <= dPosX + radius; i += 0.05f) {
		if (i <= 0.375f && i >= -0.375f) {
			for (float j = dPosZ - radius; j <= dPosZ + radius; j += 0.05f) {
				if (j <= 0.375f && j >= -0.375f) {
					index = (((i + 0.375f) / 0.05f) * 15) + ((j + 0.375f) / 0.05f);
					searchArea[k++] = index;
				}
			}
		}
	}
	*areaSize = k;
	return searchArea;
}

