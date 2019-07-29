#!/usr/bin/env python
###########
# IMPORTS #
###########
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import mavros
from mavros_msgs.msg import State
from threading import Thread
import math
import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

import sys, os
sys.path.append(os.path.join(sys.path[0], '../..'))

#############
# CONSTANTS #
#############
_RATE = 10 # (Hz) rate for rospy.rate
_MAX_SPEED = 1 # (m/s)
_MAX_CLIMB_RATE = 0.5 # m/s 

##############
# CONTROLLER #
##############
class VelocityMerger:

    def __init__(self):
        # Create node with name 'controller'
        rospy.init_node('velocity_merger')
    
        self.obstacle_sub=rospy.Subscriber('/obstacle_avoid/vel', Twist, self.obstacle_sub_cb)
        
        self.tracker_sub = rospy.Subscriber('/tracker/vel', Twist, self.tracker_sub_cb)
        
        self.velocity_pub=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        
        

        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped' is recieved 
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)
        # Quaternion representing the rotation of the drone's body frame (bu) in the LENU frame. 
        # Initiallize to identity quaternion, as bu is aligned with lenu when the drone starts up.
        self.quat_bu_lenu = (0, 0, 0, 1)
        self.height =0

        # A subscriber to the topic '/mavros/state'. self.state_sub_cb is called when a message of type 'State' is recieved
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_sub_cb)
        # Flight mode of the drone ('OFFBOARD', 'POSCTL', 'MANUAL', etc.)
        self.mode = State().mode

        # A publisher which will publish the desired linear and anglar velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        # Initialize linear setpoint velocities
        self.line_vx = 0
        self.line_vy = 0
        self.line_angular_z = 0

        self.obs_vz = 0

        # Publishing rate
        self.rate = rospy.Rate(_RATE)
        
        self.current_marker = None


        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False


    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def pos_sub_cb(self, posestamped):
        """
        Updates the orientation the drone (the bu frame) related to the lenu frame
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
        Callback function which is called when a new message of type State is recieved 
        by self.state_subscriber to update the drone's mode (MANUAL, POSCTL, or OFFBOARD)
            
            Args:
                - state = mavros State message
        """
        self.mode = state.mode
    
    def obstacle_sub_cb(self, msg):
        self.obs_vz = msg.linear.z
        
        
    def tracker_sub_cb(self, msg):
        self.line_vx = msg.linear.x
        self.line_vy = msg.linear.y
        self.line_angular_z = msg.angular.z
    
    
    
    def pos_sub_cb(self, posestamped):
        """
        Updates the orientation the drone (the bu frame) related to the lenu frame
            Args: 
                - posestamped = ROS PoseStamped message
        """
        self.quat_bu_lenu = (   posestamped.pose.orientation.x, 
                                posestamped.pose.orientation.y, 
                                posestamped.pose.orientation.z, 
                                posestamped.pose.orientation.w)
        self.height = posestamped.pose.position.z
        self.position_x = posestamped.pose.position.x

   

    #############
    # STREAMING #
    #############
    def start(self): 
        """
        Start thread to stream velocity commands.
        """
        #rospy.logerr('start')
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
        Continually publishes Twist commands in the local lenu reference frame. Our desired velocities (in the local frame) are in self.vx, self.vy, self.vz (linear velocity) 
        and self.wx, self.wy, self.wz (rotational velocities around their respective axes)
        """
        # Create twist message for velocity setpoint represented in lenu coordinates
        velsp__lenu = Twist()

        # Continually publish velocity commands
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return

            '''
            Create velocity setpoint
            NOTE: velsp__lenu is a Twist message, not a simple array or list. It
            is composed of two 3-long vectors: linear and angular.
            To access and assign the x,y,z components of the translational velocity, 
            you need to use velsp__lenu.linear.x, etc.
            See documentation here: https://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
        
            Use the provided functions to calculate the desired velocity of the body-up frame with respect to the 
            local ENU frame, expressed in local ENU coordinates (i.e. vsp_bu_lenu__lenu).
            Encode this in the linear portion of the Twist message.
            
            Use the coord_transforms.get_v__lenu function in aero_control/common/coordinate_transforms.py 
            to convert velocities in control_reference_frame to lenu frame.
            
            Don't forget to prevent vx, vy from exceeding _MAX_SPEED and vz from exceeding _MAX_CLIMB_RATE
            '''
            
            '''if distance is less than 1m:
                    translate whatever
                else:
                    set velocities using controller
            #else    '''
            #rospy.logerr('streaming')
            self.curr_time = rospy.get_time()


            #or (seen_recently and self.position_x-self.start_position_x > 0.5 and abs(self.target_z -self.height) > .1:

            velsp__lenu.angular.x = 0.0
            velsp__lenu.angular.y = 0.0
            velsp__lenu.angular.z = self.line_angular_z

            #print(self.line_vx)
            #print(self.line_vy)
            velsp__lenu.linear.x = self.line_vx
            velsp__lenu.linear.y = self.line_vy
            velsp__lenu.linear.z = self.obs_vz

            # Publish setpoint velocity
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()

    ########
    # WAIT #
    ########
    def wait(self):
        """
        If the drone is not in OFFBOARD mode, loops until the drone is put into OFFBOARD mode. If the drone is in OFFBOARD mode,
        loops until the drone is taken out of OFFBOARD mode.
        """
        # Wait till drone is put into OFFBOARD mode
        if self.mode != 'OFFBOARD':
            rospy.loginfo('Open loop controller: Waiting to enter OFFBOARD mode')
            while self.mode != 'OFFBOARD' and not rospy.is_shutdown():
                self.rate.sleep()
            rospy.loginfo('Open loop controller: {} mode ...'.format(self.mode))
    
        # Wait till drone is taken out of OFFBOARD mode
        else:
            rospy.loginfo('Open loop controller: Waiting to exit OFFBOARD mode')
            while self.mode == 'OFFBOARD' and not rospy.is_shutdown():
                self.rate.sleep()
            rospy.loginfo('Open loop controller: {} mode ...'.format(self.mode))

    ###############
    # TRANSLATION #
    ###############
    def translate(self, displacement, speed):
        """
        Given a displacement vector (dx, dy, dz) and a speed, sets the command velocities so that the drone moves from its current
        location (x, y, z) to the point (x+dx, y+dy, z+dz),
            Args:
                - displacement = (dx, dy, dz) (m)
                - speed = (m/s), must be positive
        """
        # Clip speed at _MAX_SPEED
        speed = min(speed, _MAX_SPEED)
        # Raise error if speed is 0 or not positive
        if speed <= 0:
            raise ValueError("Speed must be positive")
        ''' TODO-START
        Set self.vx, self.vy, self.vz to the correct speeds, and set the correct time 
        for the velocity commands to be published for.
        '''
        move_time = math.sqrt(displacement[0]**2 + displacement[1]**2 + displacement[2]**2) / speed

        self.vx = displacement[0]/move_time
        self.vy = displacement[1]/move_time
        self.vz = displacement[2]/move_time
        
        
        ''' TODO-END'''
        
        rospy.loginfo('Open loop controller: Time of translation: {:.2f}'.format(move_time))
        rospy.loginfo('Open loop controller: Displacement vector: {}'.format(displacement))
        
        # Wait for us to finish publishing velocities
        rospy.sleep(move_time)

        # Reset command velocites to 0
        self.vx = self.vy = self.vz = 0
        rospy.loginfo('Open loop controller: Done')


    
if __name__ == "__main__":

    # Create Controller instance
    controller = VelocityMerger()
    # Start streaming setpoint velocites
    
    controller.start()
    rospy.spin()
    controller.stop()
    
    '''
    Execute maneuver
    TODO-START: call controller.translate with a 3-tuple and scalar, positive speed. 
    3-tuple is to total change in position desired
    '''
