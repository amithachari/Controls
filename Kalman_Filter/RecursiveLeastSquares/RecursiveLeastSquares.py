import numpy as np

class RecursiveLeastSquares(object):

    # x0 - initial estimate used to initialize the estimator
    # P0 - initial estimation error covariance matrix
    # R  - covariance matrix of the measurement noise
    def __init__(self, x0, P0, R):
        self.x0 = x0
        self.P0 = P0
        self.R = R

        self.currentTimeStep = 0

        self.estimates = []
        self.estimates.append(x0)

        self.estimationErrorCovarianceMatrices = []
        self.estimationErrorCovarianceMatrices.append(P0)

        self.gainMatrices = []

        self.errors = []


    # This function computes y, x, C, P
    def predict(self, measurementValue, C):
        import numpy as np

        Lmatrix = self.R + np.matmul(C, np.matmul(self.estimationErrorCovarianceMatrices[self.currentTimeStep],C.T))
        LmatrixInv = np.linalg.inv(Lmatrix)

        gainMatrix = np.matmul(self.estimationErrorCovarianceMatrices[self.currentTimeStep], np.matmul(C.T, LmatrixInv))

        error = measurementValue - np.matmul(C, self.estimates[self.currentTimeStep])

        estimate = self.estimates[self.currentTimeStep] + np.matmul(gainMatrix, error)

        Imkc = np.eye(np.size(self.x0), np.size(self.x0)) - np.matmul(gainMatrix, C)

        estimationErrorCovarianceMatrix = np.matmul(Imkc, self.estimationErrorCovarianceMatrices[self.currentTimeStep])

        self.estimates.append(estimate)

        self.estimationErrorCovarianceMatrices.append(estimationErrorCovarianceMatrix)

        self.gainMatrices.append(gainMatrix)

        self.errors.append(error)

        self.currentTimeStep = self.currentTimeStep + 1

