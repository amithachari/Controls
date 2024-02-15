#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "LQR.h"
//#include "LQR.cpp"

using namespace Eigen;
using namespace std;


int main() {

	double m1 = 2;
	double m2 = 2;
	double k1 = 100;
	double k2 = 200;
	double d1 = 1;
	double d2 = 5;

	Matrix <double, 4, 4> Ac{ {0, 1, 0, 0},
							{-(k1 + k2) / m1 ,  -(d1 + d2) / m1 , k2 / m1 , d2 / m1},
							{0 , 0 ,  0 , 1},
							{k2 / m2,  d2 / m2, k2 / m2, d2 / m2} };
	Matrix <double, 4, 1> Bc{ {0},{0},{0},{1 / m2} };
	Matrix <double, 1, 4> Cc{ {1,0,0,0} };

	Matrix <double, 4, 1> x0{ {5},{-3},{10},{-1} };

	//total number of simulation steps
	int simulationTimeSteps = 50;
	//discretization time step
	double h = 0.1;
	//Extract rows and columns
	unsigned int n = Ac.rows();
	unsigned int m = Bc.cols();

	//Construct Weighting Matrix
	MatrixXd weightMatrixQ;
	weightMatrixQ = 100 * MatrixXd::Identity(n, n);


	MatrixXd weightMatrixR;
	weightMatrixR = 0.01 * MatrixXd::Identity(m, m);

	//Mixed weight matrix
	MatrixXd weightMatrixS;
	weightMatrixS.resize(n, m);
	weightMatrixS.setZero();

	int maxIteration = 5000;
	double toleranceConvergence = 1e-8;

	//Construct LQR Object
	LQRController lqr(Ac, Bc, weightMatrixQ, weightMatrixR, weightMatrixS);
	lqr.ComputeSolution(maxIteration, toleranceConvergence);
	lqr.SimulateSystem(x0, simulationTimeSteps, h);

	lqr.SaveData("computedK.csv", "computedAcl.csv", "computedSolutionRiccati.csv", "computedSimulatedStateTrajectory.csv");

	return 0;


}