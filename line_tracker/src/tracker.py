#!/usr/bin/env python

'''
start to inch up x gains to go faster but it's looking more stable
came down on top of AR tag because it was too slow
go up by 50%
still need to test on curves
'''
###########
# IMPORTS #
###########
import numpy as np
import rospy
import cv2
import math
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from aero_control.msg import Line
import mavros
from mavros_msgs.msg import State
from threading import Thread
from aero_control.msg import Line, LineArray

import sys, os
sys.path.append(os.path.join(sys.path[0], '../..'))
from common import coordinate_transforms

#############
# CONSTANTS #
#############
_RATE = 10 # (Hz) rate for rospy.rate
_MAX_SPEED = 1.0 # (m/s)
_MAX_CLIMB_RATE = 0.5 # m/s
_MAX_ROTATION_RATE = .5 # rad/s 
IMAGE_HEIGHT = 128
IMAGE_WIDTH = 128
CENTER = np.array([IMAGE_WIDTH//2, IMAGE_HEIGHT//2]) # Center of the image frame. We will treat this as the center of mass of the drone
EXTEND_STRAIGHT = 60 # Number of pixels forward to extrapolate the line
KP_X_STRAIGHT = .006 #.013
KP_Y_STRAIGHT = .006#.007
KP_Z_STRAIGHT = 1.5 # ignore this
KP_Z_STRAIGHT_W = 1 #2
KD=1

'''EXTEND_CURVE = 20
KP_X_CURVE = 0.005
KP_Y_CURVE = 0.008 # turned up y from 0.013
KP_Z_CURVE_W = 3.5'''

TARGET_Z = 0.7
DISPLAY = True

#########################
# COORDINATE TRANSFORMS #
#########################
# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()

##############
# CONTROLLER #
##############
class LineController:
    '''
    Note: LineController assumes a downward camera reference frame, thus there is
        no configurable input parameter for the reference frame
    '''

    def __init__(self):
        # Create node with name 'tracker'
        rospy.init_node('tracker')

        # Initialize instance of CvBridge to convert images between OpenCV images and ROS images
        self.bridge = CvBridge()
        
        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped' is recieved 
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)
        # Quaternion representing the rotation of the drone's body frame in the NED frame. initiallize to identity quaternion
        self.quat_bu_lenu = (0, 0, 0, 1)

        # A subscriber to the topic '/mavros/state'. self.state_sub_cb is called when a message of type 'State' is recieved
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_sub_cb)
        # State of the drone. 
        #   self.state.mode = flight mode (e.g. 'OFFBOARD', 'POSCTL', etc), 
        #   self.state.armed = are motors armed (True or False), etc.
        self.mode = State().mode

        # A subscriber to the topic '/line/param'. self.line_sub_cb is called when a message of type 'Line' is recieved 
        self.line_sub = rospy.Subscriber('/line/param', LineArray, self.line_sub_cb)

        # A subscriber to the topic '/aero_downward_camera/image'. self.image_sub_cb is called when a message is recieved 
        self.camera_sub = rospy.Subscriber('/aero_downward_camera/image', Image, self.camera_sub_cb)
        self.image = None

        # A publisher which will publish an image annotated with the detected line to the topic 'line/tracker_image'
        self.tracker_image_pub = rospy.Publisher('/tracker_image', Image, queue_size=1)

        # A publisher which will publish the desired linear and anglar velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/tracker/vel', Twist, queue_size = 1)

        # Linear setpoint velocities in downward camera frame
        self.vx__dc = 0.0
        self.vy__dc = 0.0
        self.vz__dc = 0.0

        # Yaw setpoint velocities in downward camera frame
        self.wz__dc = 0.0

        self.height = 0.0

        # Publishing rate
        self.rate = rospy.Rate(_RATE)

        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False
        
        self.error=np.array([0.0,0.0,0.0])
        self.prev_error=np.array([0.0,0.0,0.0])


    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def pos_sub_cb(self, posestamped):
        """
        Callback function which is called when a new message of type PoseStamped is recieved by self.position_subscriber.
            Args: 
                - posestamped = ROS PoseStamped message
        """
        self.quat_bu_lenu = (   posestamped.pose.orientation.x, 
                                posestamped.pose.orientation.y, 
                                posestamped.pose.orientation.z, 
                                posestamped.pose.orientation.w)
        self.height = posestamped.pose.position.z

    def state_sub_cb(self, state):
        """
        Callback function which is called when a new message of type State is recieved by self.state_subscriber.
            
            Args:
                - state = mavros State message
        """
        self.mode = state.mode

    def camera_sub_cb(self, image):
        """
        Callback function which is called when a new message of type Image is recieved by self.camera_sub.
            Args: 
                - image = ROS Image message
        """
        # Convert Image msg to and OpenCV image
        self.image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(image, "8UC1"), cv2.COLOR_GRAY2BGR)

    def line_sub_cb(self, param):
        """
        Callback function which is called when a new message of type Line is recieved by self.line_sub.
        Notes:
        - This is the function that maps a detected line into a velocity 
        command
            
            Args:
                - param: parameters that define the center and direction of detected line
        """

        '''TODO-START: FILL IN CODE HERE '''
        
        if len(param.arr) != 0:
            x,y,vx,vy = param.arr[0].x,param.arr[0].y,param.arr[0].vx,param.arr[0].vy
           
            V=np.array([CENTER[0]-x, CENTER[1]-y])
            U=np.array([vx/(math.sqrt(vx**2+vy**2)), vy/(math.sqrt(vx**2+vy**2))])
            if U[0]<0:
                U[0] = -U[0]

            closest = (x+np.dot(U,V)*U[0],y+np.dot(U,V)*U[1])
            target=(closest[0]+EXTEND_STRAIGHT*U[0], closest[1]+EXTEND_STRAIGHT*U[1])
            error =  (target[0]-CENTER[0],target[1]-CENTER[1])
            
            error_angle = math.atan2(vy, vx)

            #on a curve
            '''if abs(error_angle)>(math.pi/12) :
                rospy.logerr('correcting curve')
                target=(closest[0]+EXTEND_CURVE*U[0], closest[1]+EXTEND_CURVE*U[1])
                error =  (target[0]-CENTER[0],target[1]-CENTER[1])

                self.vx__dc =KP_X_CURVE*error[0]
                self.vy__dc =KP_Y_CURVE*error[1]
                self.wz__dc = KP_Z_CURVE_W*error_angle
                
                self.prev_error=self.error
                self.error=np.append(V,vy/vx)
                self.derivativeControl(self.prev_error,self.error)

            target=(closest[0]+EXTEND_STRAIGHT*U[0], closest[1]+EXTEND_STRAIGHT*U[1])
            error =  (target[0]-CENTER[0],target[1]-CENTER[1])'''

            self.vx__dc =KP_X_STRAIGHT*error[0]
            self.vy__dc =KP_Y_STRAIGHT*error[1]
            self.wz__dc = KP_Z_STRAIGHT_W*error_angle
            
            '''self.prev_error=self.error
            self.error=np.append(V,vy/vx)
            self.derivativeControl(self.prev_error,self.error)'''
            

            #y super off
            '''elif abs(error[1])>40:
                rospy.logerr('correcting y')

                self.vx__dc =KP_X_CURVE*error[0]
                self.wz__dc = KP_Z_STRAIGHT_W*error_angle
                self.wz__dc = KP_Y_CURVE*error[1]'''

            #on as straight line    
            if DISPLAY:
                image = self.image.copy()
                # Draw circle at closest 
                cv2.circle(image, (int(closest[0]), int(closest[1])), 5, (255,128,255), -1)
                # Get unit error vector
                #unit_error = error/np.linalg.norm(error)
                # Draw line from CENTER to target
                cv2.line(image, (int(CENTER[0]), int(CENTER[1])), (int(target[0]), int(target[1])), (255, 0, 0), 2)
                # Convert color to a ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
                # Publish annotated image
                self.tracker_image_pub.publish(image_msg)
                #rospy.loginfo(self.vz__dc)
                

        else:
            self.vx__dc = 0.0
            self.vy__dc = 0.0

            if DISPLAY:
                image = self.image.copy()
                image_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
                # Publish annotated image
                self.tracker_image_pub.publish(image_msg)
                #rospy.loginfo(self.vz__dc)
                
        
        
        # Find the closest point on the line to the center of the image
        # and aim for a point a distance of EXTEND_STRAIGHT (in pixels) from the closest point on the line


        # Find error between the center of the image and the target point
        # and use your knowledge of controlers to set linear velocities in downward camera frame based on error
        

        # Get angle between x-axis and the tangent vector to the line
        # and set angular velocites based on error between forward pointing vector and tangent vector
        

        '''TODO-END '''
        
    def derivativeControl(self,prev_error,error):
        
        '''
        Takes the difference between the error now and the last error
        If the difference is increasing we will increase control over KP in that respective axis
        if the difference is decreasing we will decrease control over KP
        
        '''
        secondDerivative_x=error[0]-prev_error[0]
        secondDerivative_y=error[1]-prev_error[1]
        secondDerivative_zw=error[2]-prev_error[2]
       
        '''self.vx__dc+=KD*secondDerivative_x*_RATE
        self.vy__dc+=KD*secondDerivative_y*_RATE
        self.wz__dc+=KD*secondDerivative_zw*_RATE'''
            
    def integralControl(self,past_errors):
        return


        ### DO NOT MODIFY ###
    
        # publish tracker commands to an image that can be visualized on
        # a camera feed

        


    #############
    # STREAMING #
    #############
    def start(self): 
        """
        Start thread to stream velocity commands.
        """
        self.offboard_command_streaming_thread = Thread(target=self.stream_offboard_velocity_setpoints)
        self.offboard_command_streaming_thread.start()

    def stop(self):
        """
        Stop streaming thread.
        """
        self.stopped = True
        try:
            self.offboard_command_streaming_thread.join()
        except AttributeError:
            pass

    def stream_offboard_velocity_setpoints(self):
        """
        Continually publishes Twist commands in the local lenu reference frame.
        """
        # Create twist message for velocity setpoint represented in lenu coords
        velsp__lenu = Twist()

        # Continualy publish velocity commands
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return

            #self.vz__dc = -1*(TARGET_Z - self.height)*KP_Z_STRAIGHT
            
            # Create velocity setpoint
            # NOTE: velsp__lenu is a Twist message, not a simple array or list. To access and assign the x,y,z
            #       components of the translational velocity, you need to use velsp__lenu.linear.x, 
            #       velsp__lenu.linear.y, velsp__lenu.linear.z

            # Set linear velocity (convert command velocity from downward camera frame to lenu)
            vx, vy, vz = coord_transforms.get_v__lenu((self.vx__dc, self.vy__dc, self.vz__dc), 
                                                        'dc', self.quat_bu_lenu)
            velsp__lenu.linear.x = vx
            velsp__lenu.linear.y = vy
            velsp__lenu.linear.z = vz

            # Set angular velocity (convert command angular velocity from downward camera to lenu)
            _, _, wz = coord_transforms.get_v__lenu((0.0, 0.0, self.wz__dc), 
                                                    'dc', self.quat_bu_lenu)

            velsp__lenu.angular.x = 0.0
            velsp__lenu.angular.y = 0.0
            velsp__lenu.angular.z = wz

            # enforce safe velocity limits
            if _MAX_SPEED < 0.0 or _MAX_CLIMB_RATE < 0.0 or _MAX_ROTATION_RATE < 0.0:
                raise Exception("_MAX_SPEED,_MAX_CLIMB_RATE, and _MAX_ROTATION_RATE must be positive")
            velsp__lenu.linear.x = min(max(vx,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(vy,-_MAX_SPEED), _MAX_SPEED)
            #velsp__lenu.linear.z = min(max(vz,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)
            velsp__lenu.angular.z = min(max(wz,-_MAX_ROTATION_RATE), _MAX_ROTATION_RATE)

            # Publish setpoint velocity
            #print(velsp__lenu.linear.x)
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()



if __name__ == "__main__":

    # Create Controller instance
    controller = LineController()
    # Start streaming setpoint velocites
    controller.start()

    rospy.spin()

    # Stop streaming setpoint velocites
    controller.stop()