import numpy as np
import rospy

# ROS messages
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from ekf_excercise.msg import control, state

rospy.init_node('state_estimator')

class EKF:

    '''
    Extended Kalman Filter using the rear CG bicycle model and observations of the states

    Attributes
    ----------
    rate: float
        Frequency of updates (used to estimate dt)
    '''

    
    
    def __init__(self, rate):
        self.dt = 1/rate
        self.x=0
        self.y=0
        self.theta=0
        self.l=2
        self.xk_predicted=0
        self.pk_predicted=0
        self.pk_actual=np.array([[0.01,0,0],[0,0.01,0],[0,0,1]])
        self.Q=np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
        self.jacobianf=0
        self.jacobianh=np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.state_steering=0
        self.state_v=0     
        self.state_x=0
        self.state_y=0
        self.state_theta=0
        self.Kk=0   
        self.R=np.array([[0.01,0,0],[0,0.01,0][0,0,0.01]])
        self.I=np.array([[1,0,0],[0,1,0],[0,0,1]])
    
    def predict(self,controls):
        '''
        Updates the current state using the controls (motion model)

        Parameters
        ----------
        controls: list of 2 floats
            [steering, v]
        '''
        self.xk_predicted=np.array([[self.x],[self.y],[self.theta]])+np.array([[controls[1]*math.cos(self.theta)],[controls[1]*math.sin(self.theta)][(controls[1]*math.tan(controls[0]))/self.l]])
        # Calculate the jacobian
        self.jacobianf=np.array([[1,0,-controls[1]*math.sin(self.theta)],[0,1,controls[1]*math.cos(self.theta)],[0,0,1]])
        

        # Step forward
        self.pk_predicted=self.jacobianf@self.pk_actual@self.np.transpose(self.jacobianf)+self.Q
        self.Kk=self.pk_predicted@self.jacobianh@(np.linalg.inv(self.jacobianh@self.pk_predicted@self.jacobianh+self.R))
        
        
    
    def correct(self,z_t):
        '''
        Updates the current states using an observation

        Parameters
        ----------
        z_t: list of 2 floats
            [x,y,theta] observations
        '''
      
        
        
        
    
    def get_current_state(self):
        '''
        Gets current state (its mean)

        Returns
        -------
        np.array of 3 floats [x,y,theta]
        '''
        

def update_controls(data):
    '''
    Reads controls and update the controls global variables

    Parameters
    ----------
    data: ekf_excercise.msg.control
    '''
    
    
    EKF_filter.state_steering=data.steering
    EKF_filter.state_v=data.v
    controls=[state_steering,state_v]
    
    global EKF_filter
    

def update_state(data):
    '''
    Reads noisy state observations and update current state and publishes it

    Parameters
    ----------
    data: ekf_excercise.msg.state
    '''
    
    EKF_filter.state_x=data.x
    self.state_y=data.y
    self.state_theta=data.theta
    z_t=[[self.state_x],[self.state_y],[self.state_theta]]
    
    global EKF_filter
    

# Topic parameters
est_state_topic = rospy.get_param("est_state_topic")
noisy_state_topic = rospy.get_param("noisy_state_topic")
controller_topic = rospy.get_param("controller_topic")

# Initialize the filter
EKF_filter = EKF(10)

# Create publishers
est_state_pub = rospy.Publisher(est_state_topic, state, queue_size=10)

# Create subscribers
rospy.Subscriber(noisy_state_topic,state,update_state)
rospy.Subscriber(controller_topic,control,update_controls)

rospy.spin()
