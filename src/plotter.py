import numpy as np
import rospy
import matplotlib.pyplot as plt

import matplotlib.patches
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle

# ROS messages
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from ekf_excercise.msg import control, state

rospy.init_node('plotter')


def get_rect(x,y,L,W,theta,color, alpha=1.0):
    '''
    Create a matplotlib patch to draw a rectangle using a given center, shape, and angle
    
    Parameters
    ----------
    x: float, in matplotlib units (unit on the plot)
        X center of the rectangle
    y: float, in matplotlib units (unit on the plot)
        Y center of the rectangle
    L: float, in matplotlib units (unit on the plot)
        Length of the rectangle along the x-axis
    W: float, in matplotlib units (unit on the plot)
        Width of the rectangle along the y-axis
    theta: float, in rads
        The angle to rotate the rectangle with after it has been placed at x,y
    color: string
        Color of the rectangle patch
    alpha: float
        Transparency of the rectangle
        
    Returns
    -------
    matplotlib.patches.Rectangle
        The rectangle patch with the desired properties
    '''
    rx, ry = x-L/2, y-W/2  # Get bottom left corner
    
    d = np.sqrt(L*L/4 + W*W/4)
    alpha = np.arctan(W/L)
    angle = theta+alpha
    
    dx = d*np.cos(angle)-L/2
    dy = d*np.sin(angle)-W/2
    
    return Rectangle((rx-dx, ry-dy), L, W, np.rad2deg(theta), color=color, alpha=alpha)



class Plotter:
    def __init__(self):
        self.car_patches_r = []
        self.car_patches_g = []
        self.L = 2
        self.W = 0.5
        self.wheel_l = 0.5
        self.wheel_w = 0.15
        self.b = 1.4
    
    def setup(self):
        '''
        
        '''
        fig = plt.figure(figsize=(13,7))
        plt.clf()
        gs = GridSpec(1,1,figure=fig)
        ax_pos = fig.add_subplot(gs[:,0])
        plt.grid("Both")
        
        waypoints = [[0,0],[10,10],[20,0],[10,-10],[0,0],[-10,10],[-20,0],[-10,-10],[0,0]]
        waypoints = np.array(waypoints)
        ax_pos.plot(waypoints[:,0], waypoints[:,1])
        
        plt.tight_layout()

    def update(self, gt_state, est_state, controls):
        fig = plt.gcf()
        ax_list = fig.axes

        for patch in self.car_patches_r:
            patch.remove()

        for patch in self.car_patches_g:
            patch.remove()
                
        self.car_patches_r = self.get_car_patches(*gt_state,controls[0],'red')
        self.car_patches_g = self.get_car_patches(*est_state,controls[0],'green')
        for patch in self.car_patches_r:
            ax_list[0].add_patch(patch)
        for patch in self.car_patches_g:
            ax_list[0].add_patch(patch)

        plt.pause(0.05)

    def get_car_patches(self, x, y, theta, sigma, color='red'):
        '''
        Create patches (rectangles) to draw a car

        Parameters
        ----------
        x: float
            X-position
        y: float
            Y-position
        theta:float
            Orientation
        sigma: float
            Steering Angle
        color: str
            Color of the body of the vehicle
        '''
        L,W,wheel_l,wheel_w,b = self.L,self.W,self.wheel_l,self.wheel_w,self.b
        
        front_wheels = np.array([x + b/2*np.cos(theta), y + b/2*np.sin(theta)])
        rear_wheels = np.array([x - b/2*np.cos(theta), y - b/2*np.sin(theta)])
        cross_wheel = np.array([W/2*np.cos(np.pi/2+theta), W/2*np.sin(np.pi/2+theta)])

        car = get_rect(x,y,L,W,theta,color)
        wheel_front_right = get_rect(*(front_wheels+cross_wheel),wheel_l,wheel_w,theta+sigma,'blue')
        wheel_front_left = get_rect(*(front_wheels-cross_wheel),wheel_l,wheel_w,theta+sigma,'blue')
        wheel_rear_right = get_rect(*(rear_wheels+cross_wheel),wheel_l,wheel_w,theta,'blue')
        wheel_rear_left = get_rect(*(rear_wheels-cross_wheel),wheel_l,wheel_w,theta,'blue')
        return [car,wheel_front_right,wheel_front_left,wheel_rear_right,wheel_rear_left]

def update_controls(data):
    '''
    Reads controls and update the controls global variables

    Parameters
    ----------
    data: ekf_excercise.msg.control
    '''
    global est_state, noisy_state, gt_state, controls, car_patches
    controls = np.array([data.steering, data.v])

def update_noisy_state(data):
    global est_state, noisy_state, gt_state, controls
    noisy_state = np.array([data.x, data.y, data.theta])

def update_est_state(data):
    global est_state, noisy_state, gt_state, controls
    est_state = np.array([data.x, data.y, data.theta])

def update_gt_state(data):
    global est_state, noisy_state, gt_state, controls
    gt_state = np.array([data.x, data.y, data.theta])

# Global variables
est_state = np.zeros(3)
noisy_state = np.zeros(3)
gt_state = np.zeros(3)
controls = np.zeros(2)

# Topic parameters
gt_state_topic = rospy.get_param("gt_state_topic")
est_state_topic = rospy.get_param("est_state_topic")
noisy_state_topic = rospy.get_param("noisy_state_topic")
controller_topic = rospy.get_param("controller_topic")

# Create subscribers
rospy.Subscriber(est_state_topic,state,update_est_state)
rospy.Subscriber(gt_state_topic,state,update_gt_state)
rospy.Subscriber(noisy_state_topic,state,update_noisy_state)
rospy.Subscriber(controller_topic,control,update_controls)

plotter = Plotter()
plotter.setup()

rate = 10
r = rospy.Rate(rate)
while not rospy.is_shutdown():
    r.sleep()
    plotter.update(gt_state,est_state,controls)