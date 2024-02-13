# Unconstrained Model Predictive Control implementation in Python

![image](https://github.com/amithachari/Controls/assets/64373075/5cf20ffb-1c48-4d59-92d5-3a2288c9fcbb)

The length of the prediction horizon is denoted by f. This is the future prediction horizon over which we will predict the state trajectory behavior. The prediction horizon is selected by the user and in some cases, can also be seen as a tuning parameter.
The length of the control horizon is denoted by v. This is the future control horizon over which we allow for the control inputs to change. We introduce the following limitation v\le f. After the control horizon, the control input is kept constant and equal to the last value in the control horizon. 
The control horizon is selected by the user and in some cases, can also be seen as a tuning parameter.

## Model used for testing MPC:
![image](https://github.com/amithachari/Controls/assets/64373075/6b3f70ee-06c4-4f2c-903d-49360f3355f2)

Step Response of the Plant without any controller:
![image](https://github.com/amithachari/Controls/assets/64373075/5a91ad82-0ef1-41ae-b589-17074bc17221)



## Results:
### Pulse Trajectory Tracking
![image](https://github.com/amithachari/Controls/assets/64373075/0ba3956d-332c-4cc2-9f3e-e8b72e5da921)

### Control Effort
![image](https://github.com/amithachari/Controls/assets/64373075/e824f1d2-b986-4ca4-ae09-d692fa6b0cf4)


Approach from:
https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-1-unconstrained-formulation-derivation-and-implementation-in-python-from-scratch/
