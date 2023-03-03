## Extended Kalman Filter for Bicycle Model
**Description**

This is an individual project to implement the Extended Kalman Filter using the bicycle model as a prediction model and 
GPS as noisy observations of position and orientation. The code is implemented using the navplan simulator and the main implementation is done in the
state_estimation.py file.
The bicycle model is used to predict the state of the car, which includes its position and orientation, 
while GPS provides noisy observations of the car's position and orientation. The car's length from the front wheel to the rear wheel is 4.9 meters.
The goal of the project is to implement the Extended Kalman Filter and use it to estimate the car's position and orientation more accurately, by fusing the
noisy GPS measurements with the predictions made by the bicycle model.


**Usage**

To use the implementation, run the state_estimation.py file. The simulator will run and provide GPS measurements, 
which will be used by the filter to estimate the car's position and orientation. The estimated states will be output to the console. 
The states will be returned using the same units as the GPS measurements, i.e., position in meters and orientation in radians.
