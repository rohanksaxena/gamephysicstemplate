#include "CoupledCollision.h"



CoupledCollision::CoupledCollision()
{
}


CoupledCollision::~CoupledCollision()
{
}


bool CoupledCollision::checkCollision(RigidBodySystem body, MassSpringSystemSimulator * masspointSystem, float border, float step, int mpl)
{
	float dPosX;
	float dPosZ;
	Vec3 bodyPosition = body.comPosition;

	if (fmod(bodyPosition.x, step) != 0) {
		dPosX = (bodyPosition.x + step) - (bodyPosition.x + step, step); // Descritization
	}
	else {
		dPosX = bodyPosition.x;
	}
	if(fmod(bodyPosition.z, step) != 0) {
		dPosZ = (bodyPosition.z + step) - (bodyPosition.z + step, step); // Descritization
	}
	else {
		dPosZ = bodyPosition.z;
	}

	int* areaSize = (int*) malloc(sizeof(int));
	*areaSize = (pow((int)(body.size.x * 2/ step), 2));
	int * indexes = searchArea(dPosX, dPosZ, body.size.x, areaSize, border, step, mpl);
	float mPosY;

	for (int i = 0; i < *areaSize; i++) {
		mPosY = masspointSystem->masspoints[indexes[i]].position.y;
		if ((bodyPosition.y - mPosY) <= body.size.x) {
			return true;
		}
	}
	
	return false;
}

int * CoupledCollision::searchArea(float dPosX, float dPosZ, float radius, int* areaSize, float border, float step, int mpl)
{
	int* searchArea = new int[*areaSize];
	int k = 0;
	int index;
	for (float i = dPosX - radius; i < dPosX + radius; i += step) {
		if (i <= border && i >= -border) {
			for (float j = dPosZ - radius; j < dPosZ + radius; j += step) {
				if (j <= border && j >= -border) {
					index = (((i + border) / step) * mpl) + ((j + border) / step);
					searchArea[k++] = index;
				}
			}
		}
	}
	*areaSize = k;
	return searchArea;
}

