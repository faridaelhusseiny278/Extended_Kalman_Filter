#!/usr/bin/env python3
from configparser import NoSectionError
from py_compile import PycInvalidationMode
import rospy
import numpy as np
from numpy.linalg import pinv,inv
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import math


rospy.init_node("state_estimation")
#Z
gps_observation = [0,0,0] # x, y, theta
 #u
steering = 0
velocity = 0
dt=0.1
u= [[velocity],[steering]]

def callback_func(data): #update gps_observation
    global gps_observation
    gps_observation=np.array(data.data)
    #gps_observation= ypred

def callback_func2(data): #update steering
    global steering 
    steering=data.data
    # theta=prev_car_state[2] #el theta ely ablaha
    # steering= math.atan((4.9*car_state[2]-theta)/velocity*dt)

def callback_func3(data): #update velocity
    global velocity 
    velocity = data.data
    # theta=prev_car_state[2] 
    # X=prev_car_state[0]
    # velocity=(car_state[0]-X)/math.cos(theta)*dt

    
def KalmanFilter(x_cap_prev , p_cap_prev): # Create the Kalman Filter here to estimate the vehicle's x, y, and theta
    global gps_observation,steering,velocity
    gps_z=np.array([[gps_observation[0]],[gps_observation[1]],[gps_observation[2]]])
    Lr= 4.9/2

    beta=math.atan((Lr*math.tan(steering))/ 4.9 )
    F= np.array([ #kan feeh hena beta+theta bas shelt el beta
        [1,0, -1* velocity*math.sin(x_cap_prev[2])*dt],[0,1, velocity*dt*math.cos(x_cap_prev[2])],[0,0,1] 
        ])

    H= np.array([[1,0,0],[0,1,0],[0,0,1]])

    # B= np.array([[dt,0],[0,0],[0,0]])




    varGPS = 6.0 # Standard Deviation of GPS Measurement
    varspeed = 1.0 # Variance of the speed measurement
    # varyaw = 0.1 # Variance of the yawrate measurement
    # R = [
    #             [varGPS**2, 0.0, 0.0],
    #             [0.0, varGPS**2, 0.0],
    #             [0.0, 0.0, varspeed**2],
    #             ]
    R = np.diag([10,10,1])


    # sGPS     = 0.5*8.8*dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
    # sCourse  = 0.1*dt # assume 0.1rad/s as maximum turn rate for the vehicle
    # # sVelocity= 8.8*dt # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
    # # sYaw     = 1.0*dt # assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle

    # # Q=[
    # #     [sGPS**2, 0.0, 0.0],
    # #     [0.0, sGPS**2, 0.0],
    # #     [0.0, 0.0, sCourse**2],
    # #     ]
    Q = np.diag([1,1,1])
    Q[2,2] *= 5

    # x_cap_prev = x_cap_prev.reshape(-1)
    x_cap = np.array([ [x_cap_prev[0] + velocity *dt* math.cos(beta+x_cap_prev[2])]
    ,[x_cap_prev[1]+velocity*dt*math.sin(beta+x_cap_prev[2])],[(x_cap_prev[2]+velocity*math.tan(steering)*dt)/4.9]
    ])
    # print(x_cap.shape)

    I= np.eye(3)
    #x_cap=  F @ x_cap_prev + B @ u  #lama bakteb el x_cap keda byb2a kolo (3,3)ma3ad el x_cap btkon (3,1)
    p_cap= (F @ p_cap_prev @ F.T ) +   Q

    Jk= (H @ p_cap @ H.T) +R

    K= p_cap @ H.T @ inv(Jk)
    x_cap = np.squeeze(x_cap)[:,np.newaxis]
    y_pred= gps_z -  x_cap
    

    rospy.loginfo(x_cap)

    print(x_cap.shape)
    print(K.shape)
    print(y_pred.shape)


    x_pred= x_cap + (K @ y_pred)
    # p_pred= (I-K@ H) @p_cap_prev @ (I-K@H).T + K@R@K.T
    p_pred = (I-K) @ p_cap_prev
    
    print(y_pred.shape) #(3,3,3)
    print(x_pred.shape) #(3,3,3)
    print( p_pred.shape) #(3,3
    print(K.shape) #(3,3)
    print(p_cap.shape) #(3,3)
    print(Jk.shape) #(3,3)
    print(x_cap.shape) #(3,1,1)
    pred=[x_pred,p_pred,y_pred]
    return pred 





    

# Subscribe to the gps, steering, and velocity topics named below and update the global variables using callbacks
# /gps
# /car_actions/steer
# /car_actions/vel


# Publisher for the state
state_pub = rospy.Publisher('/vehicle_model/state', Float64MultiArray, queue_size=10)
sub= rospy.Subscriber('/gps',Float64MultiArray ,callback_func)
sub2= rospy.Subscriber('/car_actions/steer',Float64 ,callback_func2)
sub3= rospy.Subscriber('/car_actions/vel', Float64,callback_func3)

r = rospy.Rate(10)

# Initialize the start values and matrices here 
po =np.array ([[1000.0,0.0,0.0], [0.0,1000.0,0.0], [0.0,0.0,1000.0]])/100000
car_state = np.array([[0] ,[0], [0]]) 

while not rospy.is_shutdown():
    # Create the Kalman Filter here to estimate the vehicle's x, y, and theta
   
    
    prev_car_state=car_state
    pred=KalmanFilter(car_state , po )
    #de ely bt5aleeh 1 element bas leh
    car_state=np.array(pred[0])
    #print (pred)
    po=pred[1]
    ypred=pred[2]


    # Create msg to publish#
    current_state = Float64MultiArray()
    layout = MultiArrayLayout()
    dimension = MultiArrayDimension()
    dimension.label = "current_state"
    dimension.size = 3
    dimension.stride = 3
    layout.data_offset = 0
    layout.dim = [dimension]
    current_state.layout = layout
    current_state.data = car_state
    
    state_pub.publish(current_state)
    r.sleep()