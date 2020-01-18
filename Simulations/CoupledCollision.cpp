#include "CoupledCollision.h"



CoupledCollision::CoupledCollision()
{
}


CoupledCollision::~CoupledCollision()
{
}


int CoupledCollision::checkCollision(RigidBodySystem body, MassSpringSystemSimulator * masspointSystem, float border, float step, int mpl, DiffusionSimulator* diffSim)
{
	float dPosX;
	float dPosZ;
	Vec3 bodyPosition = body.comPosition;

	if (fmod(abs(-border - bodyPosition.x), step) != 0) {
		dPosX = -border + (round(abs(-border - bodyPosition.x) / step)) * step; // Discritization
	}
	else {
		dPosX = bodyPosition.x;
	}

	if (fmod(abs(-border - bodyPosition.z), step) != 0) {
		dPosZ = -border + (round(abs(-border - bodyPosition.z) / step)) * step;// Discritization
	}
	else {
		dPosZ = bodyPosition.z;
	}

	int* areaSize = (int*)malloc(sizeof(int));
	*areaSize = (pow(round(body.size.x / step) * 2.0f, 2));
	int * indexes = searchArea(dPosX, dPosZ, round(body.size.x / step) * step, areaSize, border, step, mpl);

	float mPosY;
	float distance;
	Vec3 diff;
	for (int i = 0; i < *areaSize; i++) {
		mPosY = masspointSystem->masspoints[indexes[i]].position.y;
		diff = masspointSystem->masspoints[indexes[i]].position - bodyPosition;
		distance = sqrt(dot(diff, diff));
		if (distance <= body.size.x) {
			for (int i = 0; i < *areaSize; i++) {
				if (!(indexes[i] < mpl || indexes[i] > pow(mpl, 2) - mpl || (indexes[i] + 1) % mpl == 0 || indexes[i] % mpl == 0)) {
					diffSim->T->grid[indexes[i] % mpl][indexes[i] / mpl] = -1;
				}
				
			}
			return indexes[i];
		}
	}
	return -1;
}

int * CoupledCollision::searchArea(float dPosX, float dPosZ, float radius, int* areaSize, float border, float step, int mpl)
{
	int* searchArea = new int[*areaSize];
	int k = 0;
	int index;
	for (float i = dPosX - radius; i < dPosX + radius; i += step) {
		if (i <= border + (step / 2) && i >= -border - (step / 2)) {
			for (float j = dPosZ - radius; j < dPosZ + radius; j += step) {
				if (j <= border + (step / 2) && j >= -border - (step / 2)) {
					index = ((abs(i + border) / step) * mpl) + (abs(j + border) / step);
					searchArea[k++] = index;
				}
			}
		}
	}
	*areaSize = k;
	return searchArea;
}

