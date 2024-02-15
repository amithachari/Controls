
#include <iostream>
#include<tuple>
#include<string>
#include<fstream>
#include<vector>
// Eigen header files
#include<Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/Eigenvalues> 
// LQR controller header file
#include "LQR.h"


/*
This constructor will initialize all the private variables,
and precompute some constants and matrices
*/
// Input arguments"
// Ainput, Binput - A and B system matrices 
// Qinput - state weighting matrix
// Rinput - control input weighting matrix
// Sinput - state-input mixed weighting matrix



LQRController::LQRController(MatrixXd Ainput, MatrixXd Binput,
    MatrixXd Qinput, MatrixXd Rinput, MatrixXd Sinput)
{
    A = Ainput; B = Binput; Q = Qinput; R = Rinput; S = Sinput;
    n = A.rows(); m = B.cols();


    // create an empty LQR gain matrix
    K.resize(m, n); K.setZero();
    // create an empty closed-loop matrix Acl=A-B*K
    Acl.resize(n, n); Acl.setZero();
    // create an empty matrix for storing the solution of the Riccati equation
    solutionRiccati.resize(n, n);
    solutionRiccati.setZero();
    // identity matrix for computations
    In = MatrixXd::Identity(n, n);

    // simulatedStateTrajectory - this private matrix is initialized and resized by the function SimulateSystem()
    // this matrix is the simulated closed-loop state trajectory 
    // this state trajectory is again resized by the function SimulateSystem()

}

void LQRController::ComputeInitialGuess(MatrixXd& initialGuessMatrix) {
    MatrixXd Anew;
    MatrixXd Bnew;

    Anew = A - B * (R.inverse()) * (S.transpose());
    Bnew = B * (R.inverse()) * (B.transpose());

    EigenSolver<MatrixXd> eigenValueSolver(Anew);

    complex<double> lambda;

    double bParam = 0;

    vector<double> realPartEigenValues;

    for (int i = 0; i < n; i++) {
        lambda = eigenValueSolver.eigenvalues()[i];
        realPartEigenValues.push_back(lambda.real());
    }
    //Compute bParam
    bParam = *(min_element(realPartEigenValues.begin(), realPartEigenValues.end()));

    bParam = -1 * bParam;

    double epsilon = 0.02; //Tuning parameter -> Change based on convergence speed

    bParam = max(bParam, 0.0) + epsilon;

    MatrixXd Abar;
    Abar = (bParam * In + Anew).transpose();

    MatrixXd RHS;
    RHS = 2 * Bnew * (Bnew.transpose());
    MatrixXd solutionLyapunov;
    solutionLyapunov.resize(n, n);
    solutionLyapunov.setZero();

    double residualLyap = 10e10;

    SolveLyapunovEquation(Abar, RHS, solutionLyapunov, residualLyap);

    initialGuessMatrix = (Bnew.transpose()) * (solutionLyapunov.inverse());
    initialGuessMatrix = 0.5 * (initialGuessMatrix + initialGuessMatrix.transpose());


}

/*
Solve Lyapunov Equation using vectorization and Kronecker operator method
Input Arguments:
Am - Input Matrix A
Rm - Right hand matrix Rm
Solution Matrix - passed as reference, solution stored in Matrix
Residual calculated as frobenius norm of the residual matrix
*/

void LQRController::SolveLyapunovEquation(const MatrixXd& Am, const MatrixXd& Rm, MatrixXd& SolutionMatrix, double& residual) {
    MatrixXd LHSMatrix;
    LHSMatrix = kroneckerProduct(In, Am.transpose()) + kroneckerProduct(Am.transpose(), In);
    MatrixXd solutionLyapunovVector;
    MatrixXd RHSVector;
    //Vector RHS Matrix to vector
    RHSVector = Rm.reshaped(n * n, 1);

    //Find solutions using various numerical methods
    //solutionLyapunovVector = LHSMatrix.colPivHouseholderQr().solve(RHSVector);
    //solutionLyapunovVector = LHSMatrix.fullPivLu().solve(RHSVector);
    //solutionLyapunovVector = (LHSMatrix.inverse()) * RHSVector;
    solutionLyapunovVector = LHSMatrix.completeOrthogonalDecomposition().solve(RHSVector);

    SolutionMatrix = solutionLyapunovVector.reshaped(n, n);
    SolutionMatrix = 0.5 * (SolutionMatrix + SolutionMatrix.transpose());
    MatrixXd residualMatrix;
    residualMatrix = (Am.transpose()) * SolutionMatrix + SolutionMatrix * Am - Rm;
    residual = residualMatrix.squaredNorm();

}


/*
Compute solution of LQR Problem
Input Arguments:
-maxNumberIterations
-Tolerance
*/

void LQRController::ComputeSolution(int maxNumberIterations, double tolerance) {
    MatrixXd initialGuess;
    initialGuess.resize(n, n);
    initialGuess.setZero();

    ComputeInitialGuess(initialGuess);

    //To check if the initial guess is stabilizing, otherwise the system won't converge
    MatrixXd initialK;
    MatrixXd initialAcl;
    initialK = (R.inverse()) * (B.transpose()) * initialGuess + S.transpose();
    initialAcl = A - B * initialK;
    //Compute Eigen values
    EigenSolver<MatrixXd> eigenValueSolver(initialAcl);
    cout << "/n The eigen values of the inital closed-loop matrix are: " << endl << eigenValueSolver.eigenvalues() << endl;
    cout << "If all the eigen values are strictly on the left half of the complex plane, the solution is stabilizing " << endl;

    solutionRiccati = initialGuess;
    MatrixXd solutionLyapunov;
    solutionLyapunov.resize(n, n);
    solutionLyapunov.setZero();

    MatrixXd RHS;
    MatrixXd tempMatrix;
    //Matrix used to compute relative error
    MatrixXd updateMatrix;
    double residualLyap = 10e10;
    double errorConvergence = 10e10;

    int currentIteration = 0;
    while (currentIteration <= maxNumberIterations && errorConvergence >= tolerance) {
        tempMatrix = (B.transpose()) * solutionRiccati + S.transpose();
        K = R.inverse() * tempMatrix;
        Acl = A - B * K;
        RHS = Q + (K.transpose()) * R * K - S * K - (K.transpose()) * (S.transpose());
        RHS = -RHS;
        SolveLyapunovEquation(Acl, RHS, solutionLyapunov, residualLyap);

        updateMatrix = solutionLyapunov - solutionRiccati;
        errorConvergence = (updateMatrix.cwiseAbs().colwise().sum().maxCoeff()) / (solutionRiccati.cwiseAbs().colwise().sum().maxCoeff());
        //Update solution for next iteration
        solutionRiccati = solutionLyapunov;
        currentIteration = currentIteration + 1;
    }

    //diagnostics
    if (errorConvergence < tolerance) {
        cout << "/n Solution Computed within tolerance limits" << endl;
        cout << "Converged Error: " << errorConvergence << endl;
        cout << "Number of iterations: " << currentIteration << endl;
    }

    //If solution didn't converge
    if (currentIteration > maxNumberIterations) {
        cout << "/n Solution didn't converge withing max iterations" << endl;
        cout << "However, the current error is not below the error tolerance. " << endl;
        cout << "Consider increasing the maximum number of iterations or decreasing the tolerance " << endl;
        cout << " Current error: " << errorConvergence << endl;
    }

    //Compute Eigen Values of the final computed Closed Loop Matrix
    EigenSolver<MatrixXd> eigenValueSolver2(Acl);
    cout << "\n The eigen Values of the final closed loop matrix: " << eigenValueSolver2.eigenvalues() << endl;
    cout << "\n Compute K matrix: " << K << endl;

}


void LQRController::SimulateSystem(MatrixXd x0, int simulationTimeSteps, double h) {
    simulatedStateTrajectory.resize(n, simulationTimeSteps);
    simulatedStateTrajectory.setZero();
    simulatedStateTrajectory.col(0) = x0;

    MatrixXd AclDiscrete;
    AclDiscrete = (In - h * Acl).inverse();
    simulatedStateTrajectory.col(0) = x0;

    for (int i = 0; i < simulationTimeSteps - 1; i++) {
        simulatedStateTrajectory.col(i + 1) = AclDiscrete * simulatedStateTrajectory.col(i);

    }
}



/*
Function to save variable in csv file
KFile - Name of the file used to stor the computed LQR Gain Matrix K
AclFile - Name of the file used ot store the closed Loop Matrix
SolutionoRiccatiFile - Name of the file to store solution of riccati equation
simulatedStateTrajectoryFile - name of the file used ot store the closed loop state trajectory

*/

void LQRController::SaveData(string KFile, string AclFile, string solutionRiccatiFile, string simulatedStateTrajectoryFile) const {
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

    ofstream file1(KFile);
    if (file1.is_open()) {
        file1 << K.format(CSVFormat);
        file1.close();
    }

    ofstream file2(AclFile);
    if (file2.is_open()) {
        file2 << Acl.format(CSVFormat);
        file2.close();
    }

    ofstream file3(solutionRiccatiFile);
    if (file3.is_open()) {
        file3 << solutionRiccati.format(CSVFormat);
        file3.close();
    }

    ofstream file4(simulatedStateTrajectoryFile);
    if (file4.is_open()) {
        file4 << simulatedStateTrajectory.format(CSVFormat);
        file4.close();
    }


}