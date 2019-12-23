#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid()
{
}

Grid::Grid(int n, int m) {
	this->n = n;
	this->m = m;
	grid = new float*[n];
	for (int i = 0; i < n; ++i)
		grid[i] = new float[m];

	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(-1.0f, 1.0f);
	for (int j = 0; j < m; j++) {
		for (int i = 0; i < n; i++) {
			grid[i][j] = randCol(eng);
		}
	}
	for (int i = 0; i < n; i++) {
		grid[i][0] = 0;
		grid[i][m - 1] = 0;
	}
	for (int j = 0; j < m; j++) {
		grid[n - 1][j] = 0;
		grid[0][j] = 0;
	}
}

void Grid::clearGrid() {
	for (int i = 0; i < n; ++i)
		delete[] grid[i];
	delete[] grid;
}

float Grid::getM() {
	return this->m;
}

float Grid::getN() {
	return this->n;
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	n = 16;
	m = 16;
	dx = 0.5f;
	dy = 0.5f;
	alpha = 0.3f;
	
	// to be implemented cmd
}

const char * DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "n", TW_TYPE_INT32, &n, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "m", TW_TYPE_INT32, &m, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha, "min=0.1f");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		T = new Grid(n, m);
		cout << "Explicit solver!\n";
		break;
	case 1:
		n = 16;
		m = 16;
		T = new Grid(n, m);
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float dt) {//add your own parameters
	if (n != T->getN() || m != T->getM()) {
		T->clearGrid();
		T = new Grid(n, m);
	}
	Grid* newT = new Grid(n, m);
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	for (int j = 1; j < m - 1; j++) {
		for (int i = 1; i < n - 1; i++) {
			float fDiffX = (T->grid[i - 1][j] - 2 * T->grid[i][j] + T->grid[i + 1][j]) / pow(dx, 2);
			float fDiffY = (T->grid[i][j - 1] - 2 * T->grid[i][j] + T->grid[i][j + 1]) / pow(dy, 2);

			newT->grid[i][j] = T->grid[i][j] + alpha * dt * (fDiffX + fDiffY);
		}
	}
	return newT;
}

void setupB(Grid* T, std::vector<Real>& b, double factor, float dt, float dx, int n, int m) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	float r = factor * dt / (2 * pow(dx, 2));
	SparseMatrix<Real> *B = new SparseMatrix<Real>(b.size());
	for (int i = 0; i < (*B).n; i++) {
		(*B).set_element(i, i, 1);
	}
	for (int i = 1; i < n - 1; i++) {
		for (int j = 1; j < m - 1; j++) {
			(*B).set_element(i * m + j, i * m + j, 1 - 4 * r);
			(*B).set_element(i * m + j, (i - 1) * m + j, r);
			(*B).set_element(i * m + j, (i + 1) * m + j, r);
			(*B).set_element(i * m + j, i * m + j - 1, r);
			(*B).set_element(i * m + j, i * m + j + 1, r);
		}
	}
	std::vector<Real> *c = new std::vector<Real>(b.size());
	for (int i = 0; i < T->getN(); i++) {
		for (int j = 0; j < T->getM(); j++) {
			(*c).at(i * T->getM() + j) = T->grid[i][j];
		}
	}
	multiply(*B, *c, b);
}

void fillT(std::vector<Real> x, Grid* T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	for (int i = 0; i < T->getN(); i++) {
		for (int j = 0; j < T->getM(); j++) {
			T->grid[i][j] = x.at(i * T->getM() + j);
		}
	}
}

void setupA(SparseMatrix<Real>& A, double factor, float dt, float dx, int n, int m) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	float r = factor * dt / (2 * pow(dx, 2));
	for (int i = 0; i < A.n; i++) {
		A.set_element(i, i, 1); // set diagonal
	}
	for (int i = 1; i < n - 1; i++) {
		for (int j = 1; j < m - 1; j++) {
			A.set_element(i * m + j, i * m + j, 1 + 4 * r);
			A.set_element(i * m + j, (i-1) * m  + j, -r);
			A.set_element(i * m + j, (i+1) * m + j, -r);
			A.set_element(i * m + j, i * m + j - 1, -r);
			A.set_element(i * m + j, i * m + j + 1, -r);
		}
	}
	
}


void DiffusionSimulator::diffuseTemperatureImplicit(float dt) {//add your own parameters

	if (n != T->getN() || m != T->getM()) {
		T->clearGrid();
		T = new Grid(n, m);
	}
	// solve A T = b
	// to be implemented
	const int N = n * m;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real>(N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1, dt, dx, n, m);
	setupB(T, *b, 0.1, dt, dx, n, m);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x, T);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	//if (m_iTestCase == 0) {
	float stepX = 1 / T->getN();
	float stepY = 1 / T->getM();

	for (int j = 0; j < T->getM(); j++) {
		for (int i = 0; i < T->getN(); i++) {
			setLighting(T->grid[i][j]);
			DUC->drawSphere(Vec3(i * stepX - 0.5f, j * stepY - 0.5f, 0), Vec3(0.05f, 0.05f, 0.05f));
		}
	}
}

void DiffusionSimulator::setLighting(float value) {
	if (value > 0) {
		DUC->setUpLighting(Vec3(0, 0, 0), Vec3(0, 0, 0), 1, Vec3(value, value, value));
	}
	else {
		DUC->setUpLighting(Vec3(0, 0, 0), Vec3(0, 0, 0), 1, Vec3(-value, 0, 0));
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
