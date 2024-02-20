import numpy as np
import matplotlib.pyplot as plt
from RecursiveLeastSquares import RecursiveLeastSquares

# True value of parameters that will be estimated
initialPosition = 10
acceleration = 5
initialVelocity = -2

noiseStd = 1

simulationTime = np.linspace(0, 15, 2000)
position = np.zeros(np.size(simulationTime))

for i in np.arange(np.size(simulationTime)):
    position[i] = initialPosition + initialVelocity * simulationTime[i] + (acceleration * simulationTime[i] ** 2) / 2

positionNoisy = position + noiseStd * np.random.randn(np.size(simulationTime))

plotStep = 300
plt.plot(simulationTime[0:plotStep], position[0:plotStep], linewidth=4, label='Ideal Position')
plt.plot(simulationTime[0:plotStep], positionNoisy[0:plotStep], 'r', label='Observed Position')

plt.xlabel('time')
plt.ylabel('position')
plt.legend()
plt.savefig('data.png', dpi=400)
plt.show()

x0 = np.random.randn(3, 1)
P0 = 100 * np.eye(3, 3)
R = 0.5 * np.eye(1, 1)

RLS = RecursiveLeastSquares(x0, P0, R)

for j in np.arange(np.size(simulationTime)):
    C = np.array([[1, simulationTime[j], (simulationTime[j] ** 2) / 2]])
    RLS.predict(positionNoisy[j], C)

estimate1 = []
estimate2 = []
estimate3 = []

for j in np.arange(np.size(simulationTime)):
    estimate1.append(RLS.estimates[j][0])
    estimate2.append(RLS.estimates[j][1])
    estimate3.append(RLS.estimates[j][2])

estimate1true = initialPosition * np.ones(np.size(simulationTime))
estimate2true = initialVelocity * np.ones(np.size(simulationTime))
estimate3true = acceleration * np.ones(np.size(simulationTime))

# plot
steps = np.arange(np.size(simulationTime))
fig, ax = plt.subplots(3, 1, figsize=(10, 15))
ax[0].plot(steps, estimate1true, color='red', linestyle='-', linewidth=6, label='True Value of Position')
ax[0].plot(steps, estimate1, color='blue', linestyle='-', linewidth=3, label='Estimate Value of Position')
ax[0].set_xlabel("Discrete-time steps k", fontsize=14)
ax[0].set_ylabel("Position", fontsize=14)
ax[0].tick_params(axis='both', labelsize=12)
ax[0].grid()
ax[0].legend(fontsize=14)

ax[1].plot(steps, estimate2true, color='red', linestyle='-', linewidth=6, label='True Value of Velocity')
ax[1].plot(steps, estimate2, color='blue', linestyle='-', linewidth=3, label='Estimate Value of Velocity')
ax[1].set_xlabel("Discrete-time steps k", fontsize=14)
ax[1].set_ylabel("Velocity", fontsize=14)
ax[1].tick_params(axis='both', labelsize=12)
ax[1].grid()
ax[1].legend(fontsize=14)

ax[2].plot(steps, estimate3true, color='red', linestyle='-', linewidth=6, label='True Value of Acceleration')
ax[2].plot(steps, estimate3, color='blue', linestyle='-', linewidth=3, label='Estimate Value of Acceleration')
ax[2].set_xlabel("Discrete-time steps k", fontsize=14)
ax[2].set_ylabel("Acceleration", fontsize=14)
ax[2].tick_params(axis='both', labelsize=12)
ax[2].grid()
ax[2].legend(fontsize=14)

plt.show()
fig.savefig('plots.png', dpi=600)
