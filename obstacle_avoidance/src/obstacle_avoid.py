#!/usr/bin/env python

'''
TODO:
not rising at the right time
rise higher?
recalibrate
nan tags?
'''
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
from common import coordinate_transforms

#############
# CONSTANTS #
#############
_RATE = 10 # (Hz) rate for rospy.rate
_MAX_SPEED = 1 # (m/s)
_MAX_CLIMB_RATE = 0.5 # m/s 
_COORDINATE_FRAMES = {'lenu','lned','bu','bd','dc','fc'}
#KP_X = .015 #.01
#KP_Y = .009 #.009
KP_Z = 1.5 #1.5
DETECTION_THRESHOLD = 1.0 #1 m
RISE_FALL_HEIGHT = 0.4
NORMAL_TARGET_Z = 0.75
VELOCITY_X = 0.4


WAIT_TIME = 6 #amount of time to wait while flying over/under obstacle before going back to .75m


#########################
# COORDINATE TRANSFORMS #
#########################
# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()

##############
# CONTROLLER #
##############
class ObstacleAvoider:

    def __init__(self, control_reference_frame='bu'):
        # Create node with name 'controller'
        rospy.init_node('obstacle_avoider')
        
        if control_reference_frame not in _COORDINATE_FRAMES:
            raise ValueError("Invalid control reference frame: " + control_reference_frame)

        self.control_reference_frame=control_reference_frame

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

        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker",AlvarMarkers,self.ar_pose_cb) 

        # A publisher which will publish the desired linear and anglar velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        # Initialize linear setpoint velocities
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Publishing rate
        self.rate = rospy.Rate(_RATE)
        
        self.current_marker = None

        self.target_z = 0.75
        self.start_time = 0.0
        self.curr_time = 0.0
        self.seen_recently = False

        self.start_position_x = 0.0


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
        self.position_x = posestamped.pose.position.x

    def state_sub_cb(self, state):
        """
        Callback function which is called when a new message of type State is recieved 
        by self.state_subscriber to update the drone's mode (MANUAL, POSCTL, or OFFBOARD)
            
            Args:
                - state = mavros State message
        """
        self.mode = state.mode


    def ar_pose_cb(self,msg):
        '''Callback for when the drone sees an AR tag

        Parameters
        ----------
        msg : ar_pose_marker
            list of the poses of ALL the observed AR tags, with respect to the output frame.
            The 1st is not necessarily the closest.
            It can have a length of 0 - that indicates no AR tags were detected.
        
        TODO: figure out what current_marker should be
        '''

        '''
        in forward camera frame
        x  - right wing
        positive y - down
        z - along nose forward
        '''
        

        smallest_dist = 1000
        for marker in msg.markers:
            if (self.find_horiz_dist(marker) < smallest_dist):
                self.current_marker = marker
                smallest_dist = self.find_horiz_dist(marker)
        
        if (self.current_marker is not None and self.seen_recently is False):
            
            rospy.logerr('new marker found')
            print(self.current_marker.id)
            self.check_dist()
            self.current_marker = None
            #print('set to none')
        

    def check_dist(self):
        print('horizdist: '+ str(self.find_horiz_dist(self.current_marker)))
        if self.find_horiz_dist(self.current_marker) <= DETECTION_THRESHOLD:
            self.seen_recently = True
            self.start_time =rospy.get_time()

            if self.find_height_obst(self.current_marker) > NORMAL_TARGET_Z:
                #go under
                print(self.find_height_obst(self.current_marker))
                print(self.find_vert_dist(self.current_marker))
                print("go under")
                self.start_position_x = self.position_x
                self.target_z = self.find_height_obst(self.current_marker) - RISE_FALL_HEIGHT
                rospy.logerr("set height: " + str(self.target_z))
            else:
                #go over
                print(self.find_height_obst(self.current_marker))
                print(self.find_vert_dist(self.current_marker))

                print('go over')
                self.start_position_x = self.position_x
                self.target_z = self.find_height_obst(self.current_marker) + RISE_FALL_HEIGHT 
                rospy.logerr("set height: "+ str(self.target_z))


        
    

    def find_horiz_dist(self, tag):
        return tag.pose.pose.position.z

    def find_vert_dist(self, tag):
        #returns vertical distance between drone and AR tag
        #positive if AR tag is below drone
        #negative if AR tag is above drone
        return tag.pose.pose.position.y
    

    def find_height_obst(self, tag):
        #return height of AR tag
        return self.height - self.find_vert_dist(tag)

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

            if abs(self.target_z -self.height) > .1:
                self.vx =0.0
            else:
                self.vx = VELOCITY_X 
            
            if self.curr_time-self.start_time > WAIT_TIME:
                self.target_z = NORMAL_TARGET_Z
                self.seen_recently = False
                self.start_time = 0.0
                self.start_position_x = 1000

            self.vz = KP_Z*(self.target_z -self.height)


            #happens no matter what
            vx_lenu = coord_transforms.get_v__lenu((self.vx, self.vy, self.vz), 'bu', self.quat_bu_lenu)[0]
            vy_lenu = coord_transforms.get_v__lenu((self.vx, self.vy, self.vz), 'bu', self.quat_bu_lenu)[1]
            vz_lenu = coord_transforms.get_v__lenu((self.vx, self.vy, self.vz), 'bu', self.quat_bu_lenu)[2]

            velsp__lenu.angular.x = 0.0
            velsp__lenu.angular.y = 0.0
            velsp__lenu.angular.z = 0.0

            velsp__lenu.linear.x = min(max(vx_lenu,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(vy_lenu,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.z = min(max(vz_lenu,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)

            

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
    cframe = 'bu' # Reference frame commands are given in
    controller = ObstacleAvoider(control_reference_frame=cframe)
    # Start streaming setpoint velocites
    
    controller.start()
    rospy.spin()
    controller.stop()
    
    '''
    Execute maneuver
    TODO-START: call controller.translate with a 3-tuple and scalar, positive speed. 
    3-tuple is to total change in position desired
    '''
    
