import numpy as np
import rospy

# ROS messages
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from ekf_excercise.msg import control, state

rospy.init_node('simulator')
rate = 10
r = rospy.Rate(rate)

# Global variables
controls = [0,0]  # Steering, and Throttle

def update_controls(data):
    '''
    Reads controls and update the controls global variables

    Parameters
    ----------
    data: ekf_state_estimation_excercise.msg.control
    '''
    global controls
    controls = [data.steering, data.v]

def publish_state(cur_state,publisher):
    to_publish = state()
    to_publish.x = cur_state[0]
    to_publish.y = cur_state[1]
    to_publish.theta = cur_state[2]

    publisher.publish(to_publish)

# Topic parameters
gt_state_topic = rospy.get_param("gt_state_topic")
est_state_topic = rospy.get_param("est_state_topic")
noisy_state_topic = rospy.get_param("noisy_state_topic")
controller_topic = rospy.get_param("controller_topic")

# Noise parameters
noise_x = rospy.get_param("noise_x", 2.0)
noise_y = rospy.get_param("noise_y", 2.0)
noise_theta = rospy.get_param("noise_theta", 2.0)

# Create publishers
gt_state_pub = rospy.Publisher(gt_state_topic, state, queue_size=10)
noisy_state_pub = rospy.Publisher(noisy_state_topic, state, queue_size=10)

# Create subscribers
rospy.Subscriber(controller_topic,control,update_controls)

class Bicycle:
    '''
    Bicycle model using the rear axle as the CG

    Attributes
    ----------
    L: float
        Length of the vehicle
    rate: float
        Frequency of updates (used to estimate dt)
    '''
    def __init__(self, L, rate):
        self.L = L
        self.state = np.zeros(3)
        self.dt = 1/rate

    def forward(self, controls):
        '''
        Steps the model forward in time using the given controls

        Parameters
        ----------
        controls: list of 2 floats
            [steering, v]
        '''
        steering, v = controls
        x,y,theta = self.state

        x_dot = (v*np.cos(theta))
        y_dot = (v*np.sin(theta))
        theta_dot = (v*np.tan(steering))/self.L

        state_dot = np.array([x_dot, y_dot, theta_dot])

        self.state += state_dot*self.dt

        return self.state
    
    def get_state(self):
        '''
        Returns the current state of the vehicle

        Returns
        -------
        np.array of 3 floats [x,y,theta]
        '''
        return self.state
    
vehicle = Bicycle(L=2, rate=rate)

while not rospy.is_shutdown():
    r.sleep()

    # Ground Truth State and publish it
    gt_state = vehicle.forward(controls)
    publish_state(gt_state,gt_state_pub)

    # Noisy Observations of the state and publish it
    noisy_state = np.copy(gt_state)
    noisy_state[0] += np.random.normal(0,noise_x)
    noisy_state[1] += np.random.normal(0,noise_y)
    noisy_state[2] += np.random.normal(0,noise_theta)

    publish_state(noisy_state,noisy_state_pub)