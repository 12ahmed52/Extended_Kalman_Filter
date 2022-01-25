import numpy as np
import rospy

# ROS messages
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from ekf_excercise.msg import control, state

rospy.init_node('controller')

class Pure_Pursuit():
    '''
    Pure Pursuit class for lateral control
    '''
    def __init__(self, L=2, delta_limit=0.48):
        self.L = L
        self.delta_min = -1*delta_limit
        self.delta_max = delta_limit


    def get_lateral_control(self,current_xy,current_yaw,next_waypoint):
        '''
        Pure pursuit lateral controller
        
        Parameters
        ----------
        current_xy: list(float,float)
            current x and y position of the car
        current_yaw: float (rad)
            current heading of the car
        next_waypoint: shape = (2,)
            Next target x and y for the car to reach
            
        Returns
        -------
        steering_angle: float (rad)
            Target steering angle (steer to this angle using the low level controller)
        '''
        alpha = np.arctan2(next_waypoint[1]-current_xy[1],next_waypoint[0]-current_xy[0])-current_yaw
        Id = 2
        
        steering_angle = np.arctan2(2*self.L*np.sin(alpha),Id)

        steering_angle = np.clip(steering_angle,self.delta_min,self.delta_max)
        
        return steering_angle


def update_controls(data):
    '''
    Reads controls and update the controls global variables

    Parameters
    ----------
    data: ekf_excercise.msg.state
    '''
    global waypoints, next_waypoint_idx, threshold, car_speed

    state = np.array([data.x, data.y, data.theta])

    # Pure pursuit
    current_xy = state[:2]
    current_yaw = state[2]

    # Get next waypoint
    if np.linalg.norm(waypoints[next_waypoint_idx]-current_xy) < threshold:
        next_waypoint_idx = (next_waypoint_idx+1)%waypoints.shape[0]

    steering = controller.get_lateral_control(current_xy, current_yaw, waypoints[next_waypoint_idx])

    to_publish = control()
    to_publish.steering = steering
    to_publish.v = car_speed

    control_pub.publish(to_publish)

# Topic parameters
est_state_topic = rospy.get_param("est_state_topic")
controller_topic = rospy.get_param("controller_topic")
car_speed = rospy.get_param("car_speed",3)
threshold = rospy.get_param("waypoint_proximity_threshold",2)

controller = Pure_Pursuit()
waypoints = [[0,0],[10,10],[20,0],[10,-10],[0,0],[-10,10],[-20,0],[-10,-10],[0,0]]
waypoints = np.array(waypoints)
next_waypoint_idx = 0

# Create publishers
control_pub = rospy.Publisher(controller_topic, control, queue_size=10)

# Create subscribers
rospy.Subscriber(est_state_topic,state,update_controls)

rospy.spin()